#include "SFCrashHandling.hpp"

extern "C" {
	#include "XPLMProcessing.h"
	#include "XPLMUtilities.h"
	#include "XPLMDisplay.h"
	#include "XPLMGraphics.h"
	#include "XPLMDataAccess.h"
	#include "XPLMPlugin.h"
	#include "XPLMProcessing.h"
	#include "XPLMScenery.h"
	#include "XPLMMenus.h"
	#include "XPLMGraphics.h"
	#include "XPLMPlanes.h"
	#include "XPLMDataAccess.h"
	#include "XPLMNavigation.h"
	#include "XPWidgets.h"
	#include "XPStandardWidgets.h"
	#include "XPLMInstance.h"
	#include <string.h>
	#include <math.h>

	#include <acfutils/crc64.h>
	#include <acfutils/glew.h>
	#include <acfutils/osrand.h>
	#include <acfutils/dr.h>
	#include <acfutils/geom.h>
	#include <acfutils/log.h>
	#include <acfutils/acf_file.h>

    #include <libquat.h>
}

#include <map>
#include <vector>
#include <string>

#include <tinyxml2.h>

#if APL==0
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif

#define MSG_ADD_DATAREF 0x01000000


#ifndef XPLM300
	#error This is made to be compiled against the XPLM300 SDK
#endif


/*
* Global Variables.  We will store our single window globally.  We also record
* whether the mouse is down from our mouse handler.  The drawing handler looks
* at this information and draws the appropriate display.
*
*/



#include "SASLMessageDefinitions.h"

const char * marker_objPath = "jetway_marker/jetway_marker.obj";

static XPLMObjectRef _markerObjectRef = nullptr;
static XPLMInstanceRef _markerInstanceRef = nullptr;

#define earth_rad   6378145.0

const double METERS_PER_FOOT = 0.3048;

static inline const char *acf_find_prop(const acf_file_t *acf, const std::string property)
{
	return acf_prop_find(acf, property.c_str());
}

// Turns a heading between [0, 360] into one between [-180, 180]
static inline double
shift_normalized_hdg(double hdg) {
	if (hdg > 180) {
		hdg -= 360;
	} else if (hdg < -180) {
		hdg += 360;
	}
	if (hdg > 180) hdg = 180;
	if (hdg < -180) hdg = -180;
	return hdg;
}

static bool notYetFullyLoaded = true;

static float flightLoopCallback(float elapsedMe, float elapsedSim, int counter, void * refcon); // Declare callback for handling networking data each flight loop

static float DataRefEditorRegistrationFlightLoopCallback(float elapsedme, float elapsedSim, int counter, void * refcon);

static void menu_handler(void * in_menu_ref, void * in_item_ref);


static int g_menu_container_idx; // The index of our menu item in the Plugins menu
static XPLMMenuID samIAmMenuID;

static dr_t	sam_jetway_rotate1_dr; // -/+ 90 degrees | Rotation of the entire jetway around Z-axis at 0,0,0
static dr_t sam_jetway_rotate2_dr; // -/+ 90 degrees | Rotation of the docking door
static dr_t sam_jetway_rotate3_dr; // -/+ 6 degrees  | Vertical rotation of the gangway around X-axis
static dr_t	sam_jetway_extent_dr; // 0-10 meters | Gangway extent in meters along Y-axis
static dr_t sam_jetway_wheels_dr; // -/+ 2 meters | Distance of vertical translation of the wheel pillar
static dr_t sam_jetway_wheelrotatel_dr, sam_jetway_wheelrotater_dr; // 0-360 degrees | Left wheel rotation (forward)
static dr_t sam_jetway_wheelrotatec_dr; //  -/+ 90 degrees | Wheel turn axis rotation
static dr_t sam_jetway_warnlight_dr; //  0/1 | status 1 = Jetway is operating

static dr_t draw_object_x_dr, draw_object_y_dr, draw_object_z_dr, draw_object_psi_dr;

static dr_t local_x_dr, local_y_dr, local_z_dr;
static dr_t q_dr, lat_ref_dr, lon_ref_dr;
static dr_t true_psi_dr;

static double aircraftLat, aircraftLon, aircraftAlt, aircraftHdg;

static XPLMProbeRef _terrainProbe;
static XPLMProbeInfo_t _terrainProbeInfo;

typedef enum DoorType
{
    LF1Door,
    LF2Door,
    LR1Door,
    OTHERDoor
} DoorType;

typedef struct door_info_s {
	float axialShift;
	float lateralShift;
	float verticalShift;
	float hdgOffset; // Door might be on curved part of aircraft
	double lat;
	double lon;
	double alt;
	float  hdg;
	DoorType type;
} door_info_t;

static bool has_board_1 = false;
static bool has_board_2 = false;
static door_info_t door_1;
static door_info_t door_2;



template<typename T>
bool fuzzyCompare(T a, T b, int k)
{
     return std::abs(a - b) <= std::numeric_limits<T>::epsilon() * std::abs(a + b) * k;
}

bool same_spot(std::pair<float, float> spot1, std::pair<float, float> spot2)
{
	return fuzzyCompare(spot1.first, spot2.first, 4) && fuzzyCompare(spot1.second, spot2.second, 4);
}

typedef struct jetway_info_s {
	std::string name;
	double lat;
	double lon;
	double alt;
	float hdg;
	float rotate1;
	float rotate2;
	float rotate3;
	float extent;
	float wheels;
	float wheelrotatel;
	float wheelrotater;
	float wheelrotatec;
	int warnlight;
	// For random animation for now at least...
	float rotate1_target;
	float rotate2_target;
	float rotate3_target;
	float extent_target;
	// Params of the jetway for animation...
	float height;
	float wheelPos;
	float cabinPos;
	float cabinLength;
	float wheelDiameter;
	float wheelDistance;
	float minRot1;
	float maxRot1;
	float minRot2;
	float maxRot2;
	float minRot3;
	float maxRot3;
	float initialRot1;
	float initialRot2;
	float initialRot3;
	float initialExtent;
	float initialWheelRotateC;
	float minExtent;
	float maxExtent;
	float minWheels;
	float maxWheels;

	XPLMInstanceRef markerInstanceRef;
} jetway_info_t;

static jetway_info_t* nearest_jetway = nullptr;

void slide_to(float *current, float target, float step) {
	if (*current < target) {
		*current = std::min(target, *current + std::abs(step));
	} else if(*current > target) {
		*current = std::max(target, *current - std::abs(step));
	}	
}

void slide_back_and_forth(float *current, float *target, float lower, float upper, float step) {
	if (*current == *target) {
		if (*target == upper) {
			*target = lower;
		} else {
			*target = upper;
		}
	} else {
		slide_to(current, *target, step);
	}
}

void random_animate_jetway(jetway_info_t *jetway) {

	if (jetway->rotate2 != jetway->rotate2_target) {
		slide_to(&jetway->rotate2, jetway->rotate2_target, 0.217);
	} else if (jetway->rotate1 != jetway->rotate1_target) {
		slide_to(&jetway->rotate1, jetway->rotate1_target, 0.0317);
	} else {
		slide_to(&jetway->rotate3, jetway->rotate3_target, 0.00217);
		slide_to(&jetway->extent, jetway->extent_target, 0.0179);
	}
	// Handle the wheel pillar...

	// wheelPos is the distance of the wheel pillar from the rotunda...
	jetway->wheels = (jetway->wheelPos + jetway->extent) * sin(DEG2RAD(jetway->rotate3));
}

std::map<std::pair<float, float>, jetway_info_t> samXMLjetways;

std::map<std::pair<float, float>, jetway_info_t> jetways;

jetway_info_t* jetway_found_in_map(std::pair<float, float>& spot, std::map<std::pair<float, float>, jetway_info_t>& mappedJetways) {
	for(auto& [spotKey, jetway] : mappedJetways) {
		if (same_spot(spot, spotKey)) {
			return &jetway;
		}
	}
	return nullptr;
}

jetway_info_t* jetway_found(std::pair<float, float> spot) {
	jetway_found_in_map(spot, jetways);
}

std::vector<std::pair<double, jetway_info_t*>> jetways_within_range_in_map(std::pair<float, float> spot, float range, std::map<std::pair<float, float>, jetway_info_t>& mappedJetways) {
	geo_pos2_t targetLoc = GEO_POS2(spot.first, spot.second);
	
	std::vector<std::pair<double, jetway_info_t*>> nearbyJetways;

	for(auto& [spotKey, jetway] : mappedJetways) {
		
		auto jetwayLoc = GEO_POS2(spotKey.first, spotKey.second);

		double dist = gc_distance(targetLoc, jetwayLoc);

		if (dist < range) {
			nearbyJetways.push_back(std::make_pair(dist, &jetway));
		}
	}

	return nearbyJetways;
}


std::vector<std::pair<double, jetway_info_t*>> jetways_within_range(std::pair<float, float> spot, float range) {

	// TODO: We should actually make this look at the distance between door and parked cab position
	//       of the jetbridge!

	return jetways_within_range_in_map(spot, range, jetways);
}

jetway_info_t* jetway_found_sam_xml(std::pair<float, float> spot, float targetHdg, float tolerance) {

	float nearest_distance = 5*tolerance;
    jetway_info_t* candidate_jetway = nullptr;
	const float hdgTolerance = 180.0f;

	float candidate_jetway_hdg_difference = 0;

	std::vector<std::pair<double, jetway_info_t*>> nearbyDistJetways = jetways_within_range_in_map(spot, 5*tolerance, samXMLjetways);

    logMsg("[DEBUG] Looking at a total of %d sam xml jetways to find a match...", nearbyDistJetways.size());

    for (auto& distJetway : nearbyDistJetways) {

    	logMsg("[DEBUG] Distance found is %f meters", distJetway.first);

    	if (distJetway.first < nearest_distance) {
    		float headingDifference = std::abs(shift_normalized_hdg(normalize_hdg(distJetway.second->hdg - targetHdg)));
    		if (headingDifference < hdgTolerance) {
	    		nearest_distance = distJetway.first;
    			candidate_jetway = distJetway.second;
    			candidate_jetway_hdg_difference = headingDifference;
        	} else {
        		logMsg("[WARN] Found closer possible jetway %s but heading differs by %f, more than %f degrees!", distJetway.second->name.c_str(), headingDifference, hdgTolerance);
        	}
        } else {
        	logMsg("[WARN] Jetway %s is not closer", distJetway.second->name.c_str());
        }	
    }

    if (candidate_jetway) { 
    	logMsg("[INFO] Found closest possible SAM jetway %s, heading differs by %f degrees!", candidate_jetway->name.c_str(), candidate_jetway_hdg_difference);	
    }

    return candidate_jetway;
}


bool_t jetway_w_cb(dr_t *dr, void *valuein)
{
	UNUSED(dr);
	UNUSED(valuein);

	logMsg("[ERROR] Jetway write callback next expected to be called, and does nothing presently");
	return B_TRUE;
}

bool_t jetway_rotate1_r_cb(dr_t *dr, void *valueout)
{
	UNUSED(dr);
	
	double worldLatitude, worldLongitude, worldAltitude;

	XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
                          dr_getf(&draw_object_y_dr),    
                          dr_getf(&draw_object_z_dr),    
                          &worldLatitude,  
                          &worldLongitude,    
                          &worldAltitude);
	
	float drawObjectHeading = dr_getf(&draw_object_psi_dr);

	std::pair<float, float> spot = std::pair(worldLatitude, worldLongitude);

	jetway_info_t *foundJetway = jetway_found(spot);

	if (!foundJetway) {

		foundJetway = jetway_found_sam_xml(spot, drawObjectHeading, 150.0);

		jetway_info_t newJetway;

		if (foundJetway) {
			newJetway = *foundJetway;

			// Lets use the precision of the sim... greater than the xml file!
			newJetway.lat = worldLatitude;
			newJetway.lon = worldLongitude;
			newJetway.alt = worldAltitude;
			newJetway.hdg = dr_getf(&draw_object_psi_dr);

			logMsg("[DEBUG] Found a SAM xml specified jetway with name %s", newJetway.name.c_str());
		} else {

			newJetway.lat = worldLatitude;
			newJetway.lon = worldLongitude;
			newJetway.alt = worldAltitude;
			newJetway.hdg = drawObjectHeading;
			newJetway.rotate1 = 0; //-85.968;
			newJetway.rotate2 = 0; //-29.644;
			newJetway.rotate3 = 0; //-3.534;
			newJetway.extent =  0; //7.07;
			newJetway.warnlight = 0;

			// C10 at Fly Tampa EHAM
			newJetway.height = 4.849999;
			newJetway.wheelPos = 11.0;
			newJetway.cabinPos = 15.5699997;
			newJetway.cabinLength = 2.700000005;
			newJetway.wheelDiameter = 0.85;
			newJetway.wheelDistance = 2.0;
			newJetway.minRot1 = -150;
			newJetway.maxRot1 = -30;
			newJetway.minRot2 = -150;
			newJetway.maxRot2 = 150;
			newJetway.minRot3 = -6;
			newJetway.maxRot3 = 6;
			newJetway.minExtent = 0;
			newJetway.maxExtent = 20;
			newJetway.minWheels = -2.4;
			newJetway.maxWheels = 1.28;

			newJetway.rotate1_target = newJetway.maxRot1;
			newJetway.rotate2_target = newJetway.maxRot2;
			newJetway.rotate3_target = newJetway.maxRot3;
			newJetway.extent_target = newJetway.maxExtent;
		
			logMsg("[WARN] Found an unknown new jetway at at lat %f, lon %f, alt %f, now have %d jetways", worldLatitude, worldLongitude, worldAltitude, jetways.size());
		}

		jetways[spot] = newJetway;

	} else {
		
		// Lets use the precision of the sim... greater than the xml file!
		foundJetway->lat = worldLatitude;
		foundJetway->lon = worldLongitude;
		foundJetway->alt = worldAltitude;
		foundJetway->hdg = dr_getf(&draw_object_psi_dr);

		if (foundJetway == nearest_jetway) {
			random_animate_jetway(foundJetway);
		} else {
			foundJetway->rotate1 = foundJetway->initialRot1;
			foundJetway->rotate2 = foundJetway->initialRot2;
			foundJetway->rotate3 = foundJetway->initialRot3;
			foundJetway->extent  = foundJetway->initialExtent;
			foundJetway->wheels = (foundJetway->wheelPos + foundJetway->extent) * sin(DEG2RAD(foundJetway->rotate3));
		}

		*((float*) valueout) = foundJetway->rotate1;
	}
	
	return B_TRUE;
}

bool_t jetway_rotate2_r_cb(dr_t *dr, void *valueout)
{
	UNUSED(dr);
	
	double worldLatitude, worldLongitude, worldAltitude;

	XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
                          dr_getf(&draw_object_y_dr),    
                          dr_getf(&draw_object_z_dr),    
                          &worldLatitude,  
                          &worldLongitude,    
                          &worldAltitude);
	
	std::pair<float, float> spot = std::pair(worldLatitude, worldLongitude);

	jetway_info_t *foundJetway = jetway_found(spot);

	if (foundJetway) {
		*((float*) valueout) = foundJetway->rotate2;
	}
	
	return B_TRUE;
}

bool_t jetway_rotate3_r_cb(dr_t *dr, void *valueout)
{
	UNUSED(dr);
	
	double worldLatitude, worldLongitude, worldAltitude;

	XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
                          dr_getf(&draw_object_y_dr),    
                          dr_getf(&draw_object_z_dr),    
                          &worldLatitude,  
                          &worldLongitude,    
                          &worldAltitude);
	
	std::pair<float, float> spot = std::pair(worldLatitude, worldLongitude);

	jetway_info_t *foundJetway = jetway_found(spot);

	if (foundJetway) {
		*((float*) valueout) = foundJetway->rotate3;
	}
	

	return B_TRUE;
}

bool_t jetway_extent_r_cb(dr_t *dr, void *valueout)
{
	UNUSED(dr);
	
	double worldLatitude, worldLongitude, worldAltitude;

	XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
                          dr_getf(&draw_object_y_dr),    
                          dr_getf(&draw_object_z_dr),    
                          &worldLatitude,  
                          &worldLongitude,    
                          &worldAltitude);
	
	std::pair<float, float> spot = std::pair(worldLatitude, worldLongitude);

	jetway_info_t *foundJetway = jetway_found(spot);

	if (foundJetway) {
		*((float*) valueout) = foundJetway->extent;
	}
	

	return B_TRUE;
}

bool_t jetway_wheels_r_cb(dr_t *dr, void *valueout)
{
	UNUSED(dr);
	
	double worldLatitude, worldLongitude, worldAltitude;

	XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
                          dr_getf(&draw_object_y_dr),    
                          dr_getf(&draw_object_z_dr),    
                          &worldLatitude,  
                          &worldLongitude,    
                          &worldAltitude);
	
	std::pair<float, float> spot = std::pair(worldLatitude, worldLongitude);

	jetway_info_t *foundJetway = jetway_found(spot);

	if (foundJetway) {
		*((float*) valueout) = foundJetway->wheels;
	}
	
	return B_TRUE;
}

bool_t jetway_wheelrotate_r_cb(dr_t *dr, void *valueout)
{

	double worldLatitude, worldLongitude, worldAltitude;

	XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
                          dr_getf(&draw_object_y_dr),    
                          dr_getf(&draw_object_z_dr),    
                          &worldLatitude,  
                          &worldLongitude,    
                          &worldAltitude);
	
	std::pair<float, float> spot = std::pair(worldLatitude, worldLongitude);

	jetway_info_t *foundJetway = jetway_found(spot);

	if (foundJetway) {
		if (dr == &sam_jetway_wheelrotatel_dr) {
			*((float*) valueout) = foundJetway->wheelrotatel;
		} else if (dr == &sam_jetway_wheelrotater_dr) {
			*((float*) valueout) = foundJetway->wheelrotater;
		} else {
			logMsg("[ERROR] Not expected to get here");
			assert(false);
		}
	}
	
	return B_TRUE;
}

bool_t jetway_wheelrotatec_r_cb(dr_t *dr, void *valueout)
{
	UNUSED(dr);
	
	double worldLatitude, worldLongitude, worldAltitude;

	XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
                          dr_getf(&draw_object_y_dr),    
                          dr_getf(&draw_object_z_dr),    
                          &worldLatitude,  
                          &worldLongitude,    
                          &worldAltitude);
	
	std::pair<float, float> spot = std::pair(worldLatitude, worldLongitude);

	jetway_info_t *foundJetway = jetway_found(spot);

	if (foundJetway) {
		*((float*) valueout) = foundJetway->wheelrotatec;
	}

	return B_TRUE;
}

bool_t jetway_warnlight_r_cb(dr_t *dr, void *valueout)
{
	UNUSED(dr);
	
	double worldLatitude, worldLongitude, worldAltitude;

	XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
                          dr_getf(&draw_object_y_dr),    
                          dr_getf(&draw_object_z_dr),    
                          &worldLatitude,  
                          &worldLongitude,    
                          &worldAltitude);
	
	std::pair<float, float> spot = std::pair(worldLatitude, worldLongitude);

	jetway_info_t *foundJetway = jetway_found(spot);

	if (foundJetway) {
		if (foundJetway == nearest_jetway) {
			*((float*) valueout) = 1;
		} else {
			*((float*) valueout) = 0;
		}
		//*((float*) valueout) = foundJetway->warnlight;
	}

	return B_TRUE;
}

void parseSAMxmlFile(std::string path)
{
	tinyxml2::XMLDocument doc;
	doc.LoadFile(path.c_str());

	// TODO: We don't check "libraryInstance" attribute, if it is "true" then we are missing lots 
	//       of these parameters, where do we find default values?  Are all library instances the same?
	//       I THINK NOT.  I believe they use a whole different set of datarefs with different indices
	//       such sam/jetway/18/rotate1.  So we have to figure out to drive those as well! Ugh.

	// TODO: Make this handle exceptions for corrupt file formatting...

	// TODO: Make this a separate thread so that this doesn't hang sim loading!
	
	auto jetwaysSec = doc.FirstChildElement("scenery")->FirstChildElement("jetways");
	unsigned int count = 0;
	for (tinyxml2::XMLElement* jetwayXMLelem = jetwaysSec->FirstChildElement(); jetwayXMLelem != NULL; jetwayXMLelem = jetwayXMLelem->NextSiblingElement())
	{
		jetway_info_t newSAMxmlJetway;

		tinyxml2::XMLError errretval;

		if (jetwayXMLelem->Attribute("name")) {
			newSAMxmlJetway.name = std::string(jetwayXMLelem->Attribute("name"));
		} else {
			logMsg("[ERROR] Couldn't find name!");
			continue;
		}
		
		// if (jetwayXMLelem->Attribute("latitude")) {
		// 	newSAMxmlJetway.lat = std::stod(std::string(jetwayXMLelem->Attribute("latitude")));
		// } else {
		// 	logMsg("[ERROR] Couldn't find latitude!");
		// 	continue;
		// }

		// if (jetwayXMLelem->Attribute("longitude")) {
		// 	newSAMxmlJetway.lon = std::stod(std::string(jetwayXMLelem->Attribute("longitude")));
		// } else {
		// 	logMsg("[ERROR] Couldn't find longitude!");
		// 	continue;
		// }

		errretval = jetwayXMLelem->QueryDoubleAttribute("latitude", &newSAMxmlJetway.lat);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find latitude!");
			continue;
		}

		errretval = jetwayXMLelem->QueryDoubleAttribute("longitude", &newSAMxmlJetway.lon);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find longitude!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("heading", &newSAMxmlJetway.hdg);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find heading!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("height", &newSAMxmlJetway.height);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find height!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("wheelPos", &newSAMxmlJetway.wheelPos);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find wheelPos!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("cabinPos", &newSAMxmlJetway.cabinPos);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find cabinPos!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("cabinLength", &newSAMxmlJetway.cabinLength);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find cabinLength!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("wheelDiameter", &newSAMxmlJetway.wheelDiameter);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find wheelDiameter!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("wheelDistance", &newSAMxmlJetway.wheelDistance);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find wheelDistance!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("minRot1", &newSAMxmlJetway.minRot1);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find minRot1!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("maxRot1", &newSAMxmlJetway.maxRot1);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find maxRot1!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("minRot2", &newSAMxmlJetway.minRot2);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find minRot2!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("maxRot2", &newSAMxmlJetway.maxRot2);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find maxRot2!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("minRot3", &newSAMxmlJetway.minRot3);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find minRot3!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("maxRot3", &newSAMxmlJetway.maxRot3);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find maxRot3!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("minExtent", &newSAMxmlJetway.minExtent);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find minExtent!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("maxExtent", &newSAMxmlJetway.maxExtent);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find maxExtent!");
			continue;
		}


		errretval = jetwayXMLelem->QueryFloatAttribute("minWheels", &newSAMxmlJetway.minWheels);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find minWheels!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("maxWheels", &newSAMxmlJetway.maxWheels);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find maxWheels!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("initialRot1", &newSAMxmlJetway.initialRot1);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find initialRot1!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("initialRot2", &newSAMxmlJetway.initialRot2);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find initialRot2!");
			continue;
		}

		errretval = jetwayXMLelem->QueryFloatAttribute("initialRot3", &newSAMxmlJetway.initialRot3);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find initialRot3!");
			continue;
		}
		
		errretval = jetwayXMLelem->QueryFloatAttribute("initialExtent", &newSAMxmlJetway.initialExtent);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find initialExtent!");
			continue;
		}
	
		newSAMxmlJetway.initialWheelRotateC = 0; // CONCERN: This seems like something SAM should specify but doesn't!

		newSAMxmlJetway.rotate1 = newSAMxmlJetway.initialRot1;
		newSAMxmlJetway.rotate2 = newSAMxmlJetway.initialRot2;
		newSAMxmlJetway.rotate3 = newSAMxmlJetway.initialRot3;
		newSAMxmlJetway.extent = newSAMxmlJetway.initialExtent;
		newSAMxmlJetway.wheelrotatec = newSAMxmlJetway.initialWheelRotateC;

		newSAMxmlJetway.markerInstanceRef = NULL;

		auto spot = std::make_pair(newSAMxmlJetway.lat, newSAMxmlJetway.lon);

		samXMLjetways[spot] = newSAMxmlJetway;

	    count++;
	}
	logMsg("[DEBUG] Found %d jetways for scenery %s", count, doc.FirstChildElement("scenery")->Attribute("name"));
}

void findSAMxmlFilesInCustomSceneries()
{
	char tempCharPath[4096];
    
    XPLMGetSystemPath(tempCharPath);

    std::string customSceneryFolderPath = std::string(tempCharPath) + "Custom Scenery/";

    // TODO: We should read the scenery ini and follow symbolic links and blah blah blah....

    for(auto itEntry = fs::recursive_directory_iterator(customSceneryFolderPath);
	         itEntry != fs::recursive_directory_iterator(); 
	         ++itEntry ) {
	    const auto filenameStr = itEntry->path().filename().string();
	    if (itEntry.depth() < 2) {
	    	if (filenameStr == "sam.xml") {
	    		//logMsg("[DEBUG] Found SAM xml file %s", itEntry->path().string().c_str());
	    		parseSAMxmlFile(itEntry->path().string());
	    	}
	    }
	}

	logMsg("[INFO] Found a total of %d SAM jetways in all sam.xml files globally", samXMLjetways.size());
}


static double manualMarkerLat = 52.308887427053648;
static double manualMarkerLon = 4.7675393140753757;
static dr_t samiam_manual_marker_lat, samiam_manual_marker_lon;
XPLMInstanceRef _manualMarkerInstanceRef = nullptr;

void setMarkerPosition()
{
    if (!_markerObjectRef) {
    	return;
    }
        
    if(!_markerInstanceRef)
    {
        const char* datarefs[1];
        datarefs[0] = nullptr;
        _markerInstanceRef = XPLMCreateInstance(_markerObjectRef, datarefs);

        if (_markerInstanceRef == NULL) {
            logMsg("Could not create marker instance.");
            XPLMUnloadObject(_markerObjectRef);
            _markerObjectRef = nullptr;
        } else {
            logMsg("Marker instance creation succeeded.");
        }
    }

    if(_markerInstanceRef)
    {
        logMsg("Setting jetway marker position for jetway %s ...", nearest_jetway->name.c_str());
        XPLMDrawInfo_t drawInfo;
        drawInfo.structSize = sizeof(XPLMDrawInfo_t);

		double local_x_jetway, local_y_jetway, local_z_jetway;
        
        XPLMWorldToLocal(nearest_jetway->lat,  
                              nearest_jetway->lon,    
                              nearest_jetway->alt + nearest_jetway->height + 2,    
                              &local_x_jetway,    
                              &local_y_jetway,    
                              &local_z_jetway); 

        drawInfo.x = local_x_jetway;
        drawInfo.y = local_y_jetway;
        drawInfo.z = local_z_jetway;
        drawInfo.pitch = 0;
        drawInfo.roll = 0;
        drawInfo.heading = nearest_jetway->hdg;
        XPLMInstanceSetPosition(_markerInstanceRef, &drawInfo, nullptr);
    }
}

void setManualMarkerPosition()
{
    if (!_markerObjectRef) {
    	return;
    }
        
    if(!_manualMarkerInstanceRef)
    {
        const char* datarefs[1];
        datarefs[0] = nullptr;
        _manualMarkerInstanceRef = XPLMCreateInstance(_markerObjectRef, datarefs);

        if (_manualMarkerInstanceRef == NULL) {
            logMsg("Could not create manual marker instance.");
        } else {
            logMsg("Manual marker instance creation succeeded.");
        }
    }

    if(_manualMarkerInstanceRef)
    {
        logMsg("Setting manual marker position to %f, %f", manualMarkerLat, manualMarkerLon);
        XPLMDrawInfo_t drawInfo;
        drawInfo.structSize = sizeof(XPLMDrawInfo_t);

		double local_x_marker, local_y_marker, local_z_marker;
        
        XPLMWorldToLocal(manualMarkerLat,  
                              manualMarkerLon,    
                              aircraftAlt + 2,    
                              &local_x_marker,    
                              &local_y_marker,    
                              &local_z_marker); 

        drawInfo.x = local_x_marker;
        drawInfo.y = local_y_marker;
        drawInfo.z = local_z_marker;
        drawInfo.pitch = 0;
        drawInfo.roll = 0;
        drawInfo.heading = 0;
        XPLMInstanceSetPosition(_manualMarkerInstanceRef, &drawInfo, nullptr);
    }
}


void load_marker_object_cb(XPLMObjectRef inObject, void *inRefcon)
{
    logMsg("Avatar object loaded successfully.");
    _markerObjectRef = inObject;  

    if (nearest_jetway) {
    	setMarkerPosition();
    } 
}




/*
* XPluginStart
*
* Our start routine registers our window and does any other initialization we
* must do.
*
*/
PLUGIN_API int XPluginStart(
	char *		outName,
	char *		outSig,
	char *		outDesc)
{

	XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

	/* First we must fill in the passed in buffers to describe our
	* plugin to the plugin-system. */

	strcpy(outName, "Sam I Am");
	strcpy(outSig, "justinsnapp.SamIAm");
	strcpy(outDesc, "Somtimes Always Maybe I Am");

	/*
	 * Always use Unix-native paths on the Mac!
	 */
	XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
	XPLMEnableFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS", 1);
	

	log_init(XPLMDebugString, "[SamIAm]");

	set_plugin_id(XPLMGetMyID());
	register_crash_handler();

	/*
	 * Initialize the CRC64 and PRNG machinery inside of libacfutils.
	 */
	crc64_init();
	
	uint64_t seed;
	
	if (!osrand(&seed, sizeof (seed)))
		seed = microclock() + clock();
	crc64_srand(seed);
	
	/*
	 * GLEW bootstrap
	 */
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		/* Problem: glewInit failed, something is seriously wrong. */
		logMsg("FATAL ERROR: cannot initialize libGLEW: %s",
		    glewGetErrorString(err));
		return 0;
	}
	if (!GLEW_VERSION_2_1) {
		logMsg("FATAL ERROR: your system doesn't support OpenGL 2.1");
		return 0;
	}

	#if defined(SOFTWARE_VERSION) && defined(COMMIT_HASH_SHORT)
	    logMsg("Sam I Am version %s (commit %s) is installed.", SOFTWARE_VERSION, COMMIT_HASH_SHORT);
	#endif
	
	g_menu_container_idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "Sam I Am", 0, 0);
	samIAmMenuID = XPLMCreateMenu("Sam I Am", XPLMFindPluginsMenu(), g_menu_container_idx, menu_handler, NULL);


	fdr_find(&draw_object_x_dr,   "sim/graphics/animation/draw_object_x");
	fdr_find(&draw_object_y_dr,   "sim/graphics/animation/draw_object_y");
	fdr_find(&draw_object_z_dr,   "sim/graphics/animation/draw_object_z");
	fdr_find(&draw_object_psi_dr, "sim/graphics/animation/draw_object_psi");

	fdr_find(&local_x_dr, "sim/flightmodel/position/local_x");
	fdr_find(&local_y_dr, "sim/flightmodel/position/local_y");
	fdr_find(&local_z_dr, "sim/flightmodel/position/local_z");

	fdr_find(&q_dr , "sim/flightmodel/position/q");
	fdr_find(&true_psi_dr, "sim/flightmodel/position/true_psi");

	fdr_find(&lat_ref_dr, "sim/flightmodel/position/lat_ref");
	fdr_find(&lon_ref_dr, "sim/flightmodel/position/lon_ref");

	_terrainProbe = XPLMCreateProbe(xplm_ProbeY);
	_terrainProbeInfo.structSize = (int) sizeof(XPLMProbeInfo_t);


	dr_create_f(&sam_jetway_rotate1_dr, NULL, B_FALSE, "sam/jetway/rotate1");
	sam_jetway_rotate1_dr.write_scalar_cb = jetway_w_cb;
	sam_jetway_rotate1_dr.read_scalar_cb = jetway_rotate1_r_cb;

	dr_create_f(&sam_jetway_rotate2_dr, NULL, B_FALSE, "sam/jetway/rotate2");
	sam_jetway_rotate2_dr.write_scalar_cb = jetway_w_cb;
	sam_jetway_rotate2_dr.read_scalar_cb = jetway_rotate2_r_cb;

	dr_create_f(&sam_jetway_rotate3_dr, NULL, B_FALSE, "sam/jetway/rotate3");
	sam_jetway_rotate3_dr.write_scalar_cb = jetway_w_cb;
	sam_jetway_rotate3_dr.read_scalar_cb = jetway_rotate3_r_cb;

	dr_create_f(&sam_jetway_extent_dr, NULL, B_FALSE, "sam/jetway/extent");
	sam_jetway_extent_dr.write_scalar_cb = jetway_w_cb;
	sam_jetway_extent_dr.read_scalar_cb = jetway_extent_r_cb;

	dr_create_f(&sam_jetway_wheels_dr, NULL, B_FALSE, "sam/jetway/wheels");
	sam_jetway_wheels_dr.write_scalar_cb = jetway_w_cb;
	sam_jetway_wheels_dr.read_scalar_cb = jetway_wheels_r_cb;

	dr_create_f(&sam_jetway_wheelrotatel_dr, NULL, B_FALSE, "sam/jetway/wheelrotatel");
	sam_jetway_wheelrotatel_dr.write_scalar_cb = jetway_w_cb;
	sam_jetway_wheelrotatel_dr.read_scalar_cb = jetway_wheelrotate_r_cb;

	dr_create_f(&sam_jetway_wheelrotater_dr, NULL, B_FALSE, "sam/jetway/wheelrotater");
	sam_jetway_wheelrotater_dr.write_scalar_cb = jetway_w_cb;
	sam_jetway_wheelrotater_dr.read_scalar_cb = jetway_wheelrotate_r_cb;

	dr_create_f(&sam_jetway_wheelrotatec_dr, NULL, B_FALSE, "sam/jetway/wheelrotatec");
	sam_jetway_wheelrotatec_dr.write_scalar_cb = jetway_w_cb;
	sam_jetway_wheelrotatec_dr.read_scalar_cb = jetway_wheelrotatec_r_cb;

	dr_create_i(&sam_jetway_warnlight_dr, NULL, B_FALSE, "sam/jetway/warnlight");
	sam_jetway_warnlight_dr.write_scalar_cb = jetway_w_cb;
	sam_jetway_warnlight_dr.read_scalar_cb = jetway_warnlight_r_cb;


	dr_create_f64(&samiam_manual_marker_lat, &manualMarkerLat, B_TRUE, "samiam/marker/manual/lat");
	dr_create_f64(&samiam_manual_marker_lon, &manualMarkerLon, B_TRUE, "samiam/marker/manual/lon");

	// This FLCB will register our custom dataref in DRE
	//XPLMRegisterFlightLoopCallback(DataRefEditorRegistrationFlightLoopCallback, 1, NULL);   
	
	// This will get called during each flight loop and we will handle network events
	XPLMRegisterFlightLoopCallback(flightLoopCallback, 1, NULL);

	findSAMxmlFilesInCustomSceneries();


	/************ LOAD MARKER OBJECT *************/

	char tempCharPath[4096];
    
    XPLMPluginID myPluginID = XPLMGetMyID();
    
    XPLMGetPluginInfo( myPluginID, NULL, tempCharPath, NULL, NULL);
    
    //CONCERN: This function causes X-Plane to crash?
    //fix_pathsep(tempCharPath);
        
    const char *os_sep = "/";

    size_t numOfPathParts;
    
    std::string pluginPath;

    char **pathSplitString = strsplit(tempCharPath, os_sep, B_TRUE, &numOfPathParts);
        
    for (int i = 0; i < numOfPathParts-2; i++) {
#if IBM 
        if (i > 0) pluginPath += os_sep; //std::string(os_sep);
#else 
        pluginPath += os_sep;
#endif
        pluginPath += pathSplitString[i];
    }

    free_strlist(pathSplitString, numOfPathParts);

    char *jetway_marker_object_path = mkpathname(pluginPath.c_str(), marker_objPath, NULL);

    logMsg("Attempting deferred load of avatar object at path: %s", jetway_marker_object_path);

    XPLMLoadObjectAsync(jetway_marker_object_path, load_marker_object_cb, NULL); 

    free(jetway_marker_object_path);

	return 1;
}

void readAircraftDoorProperties()
{
	static char aircraftPath[4096];
    static char aircraftFileName[1024];

    XPLMGetNthAircraftModel(0, aircraftFileName, aircraftPath); 

    // TODO: Lets first look for a livery specific samiam.xml file
    //       then look for aircraft folder samiam.xml file
    //       then maybe some kind of global info we ship with the plugin
    //       then and only then read the acf info?

    //       samiam.xml file could contain info about which doors to use
    //       if one or two jetways present and so on and so forth...

	acf_file_t *acf = acf_file_read(aircraftPath);
    
	if(acf) {

		float cgY_m = METERS_PER_FOOT * std::stof(acf_find_prop(acf, "acf/_cgY"));
		float cgZ_m = METERS_PER_FOOT * std::stof(acf_find_prop(acf, "acf/_cgZ"));


		const char *acf_prop = nullptr;

		if (acf_prop = acf_find_prop(acf, "acf/_has_board_1")) {
			has_board_1 = (std::stoi(acf_prop) == 1);
			if (has_board_1) {
				if (acf_prop = acf_find_prop(acf, "acf/_board_1/0")) {
					door_1.lateralShift = METERS_PER_FOOT * std::stof(acf_prop);
					door_1.verticalShift = METERS_PER_FOOT * std::stof(acf_find_prop(acf, "acf/_board_1/1")) - cgY_m;
					door_1.axialShift = METERS_PER_FOOT * std::stof(acf_find_prop(acf, "acf/_board_1/2")) - cgZ_m;
					door_1.hdgOffset = 0;
					door_1.type = LF1Door;
				} else {
					has_board_1 = false;
				}
			}
		} else {
			has_board_1 = false;
		}

		if (acf_prop = acf_find_prop(acf, "acf/_has_board_2")) {
			has_board_2 = (std::stoi(acf_prop) == 1);
			if (has_board_2) {
				if (acf_prop = acf_find_prop(acf, "acf/_board_2/0")) {
					door_2.lateralShift = METERS_PER_FOOT * std::stof(acf_prop);
					door_2.verticalShift = METERS_PER_FOOT * std::stof(acf_find_prop(acf, "acf/_board_2/1")) - cgY_m;
					door_2.axialShift = METERS_PER_FOOT * std::stof(acf_find_prop(acf, "acf/_board_2/2")) - cgZ_m;
					door_2.hdgOffset = 0;
					door_2.type = LF2Door;
				} else {
					has_board_2 = false;
				}
			}
		} else {
			has_board_1 = false;
		}
	}

	acf_file_free(acf);

	logMsg("[DEBUG] Has board 1: %d and axial shift = %f, lateral shit = %f and vertical shift = %f", has_board_1, door_1.axialShift, door_1.lateralShift, door_1.verticalShift);
}

float DataRefEditorRegistrationFlightLoopCallback(float elapsedme, float elapsedSim, int counter, void * refcon)
{
    XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
    if (PluginID != XPLM_NO_PLUGIN_ID){
        //XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)"SharedFlight/seat");
    }
    
    return 0.0f;  // Flight loop is called only once!
}

void setSAMmarkerPositions() {

	for (auto& [spot, jetway] : samXMLjetways) {

		if(!jetway.markerInstanceRef)
	    {
	        const char* datarefs[1];
	        datarefs[0] = nullptr;
	        jetway.markerInstanceRef = XPLMCreateInstance(_markerObjectRef, datarefs);

	        if (jetway.markerInstanceRef == NULL) {
	            logMsg("Could not create SAM jetway marker instance.");
	        } else {
	            logMsg("SAM jetway marker instance creation succeeded.");
	        }
	    }

	    if(jetway.markerInstanceRef)
	    {
	        logMsg("Setting jetway marker position for jetway %s ...", jetway.name.c_str());
	        XPLMDrawInfo_t drawInfo;
	        drawInfo.structSize = sizeof(XPLMDrawInfo_t);

			double local_x_jetway, local_y_jetway, local_z_jetway;
	        
	        XPLMWorldToLocal(jetway.lat,  
	                              jetway.lon,    
	                              jetway.alt + jetway.height + 2,    
	                              &local_x_jetway,    
	                              &local_y_jetway,    
	                              &local_z_jetway); 

	        XPLMProbeResult probe_result = XPLMProbeTerrainXYZ(_terrainProbe,
	          (float) local_x_jetway,
	          (float) local_y_jetway,
	          (float) local_z_jetway,
	          &_terrainProbeInfo);

		    double rotundaBaseLatitude, rotundaBaseLongitude, rotundaBaseMSL;
	        XPLMLocalToWorld(_terrainProbeInfo.locationX,    
	                         _terrainProbeInfo.locationY,    
	                         _terrainProbeInfo.locationZ,    
	                         &rotundaBaseLatitude,  
	                         &rotundaBaseLongitude,    
	                         &rotundaBaseMSL);

	        jetway.alt = rotundaBaseMSL;

	        double local_x_jetway2, local_y_jetway2, local_z_jetway2;

	        XPLMWorldToLocal(jetway.lat,  
	                              jetway.lon,    
	                              jetway.alt + jetway.height + 2,    
	                              &local_x_jetway2,    
	                              &local_y_jetway2,    
	                              &local_z_jetway2); 


	        drawInfo.x = local_x_jetway;
	        drawInfo.y = local_y_jetway;
	        drawInfo.z = local_z_jetway2;
	        drawInfo.pitch = 0;
	        drawInfo.roll = 0;
	        drawInfo.heading = jetway.hdg;
	        XPLMInstanceSetPosition(jetway.markerInstanceRef, &drawInfo, nullptr);

	    }
	}
}

float flightLoopCallback(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	
	// if (notYetFullyLoaded == true) {

	// 	XPLMPluginID  myPluginID = XPLMGetMyID();

	// 	char outFilePath[1000];
	// 	const char *directorySeperator;

	// 	directorySeperator = XPLMGetDirectorySeparator();

	// 	XPLMGetPluginInfo(myPluginID, NULL, outFilePath, NULL, NULL);    /* Can be NULL */

	// 	logMsg("Our plugin file path is: %s", outFilePath);
	// 	logMsg("Our directory seperator is: %s", directorySeperator);
		
	// 	notYetFullyLoaded = false;
	// }


	// Get the aircraft position....

	double local_x = dr_getf(&local_x_dr);
    double local_y = dr_getf(&local_y_dr);
    double local_z = dr_getf(&local_z_dr);

    // Get the aircraft true heading...
    aircraftHdg = dr_getf(&true_psi_dr);


    double dx, dy, dz;
   
    XPLMLocalToWorld(local_x,    
                         local_y,    
                         local_z,    
                         &aircraftLat,  
                         &aircraftLon,    
                         &aircraftAlt);


    /* Project into aircraft oriented frame.... */
    
    float current_opengl_xp_q[4];
    dr_getvf32(&q_dr, current_opengl_xp_q, 0, 4);

    struct quat current_opengl_quat = {current_opengl_xp_q[1], current_opengl_xp_q[2], 
                                       current_opengl_xp_q[3], current_opengl_xp_q[0]};

    double lat_ref = dr_getf(&lat_ref_dr);
    double lon_ref = dr_getf(&lon_ref_dr);

    geo_pos2_t refpt = GEO_POS2(lat_ref, lon_ref);

    struct quat local_2_ecmigl_quat = quat_local2ecmigl(refpt, 0);

    struct quat acf_q_earth_fixed = quat_rot_concat(local_2_ecmigl_quat, current_opengl_quat);

    // _pilotInputInfo.q[0] = acf_q_earth_fixed.w;
    // _pilotInputInfo.q[1] = acf_q_earth_fixed.x;
    // _pilotInputInfo.q[2] = acf_q_earth_fixed.y;
    // _pilotInputInfo.q[3] = acf_q_earth_fixed.z;

    /* Used for projection of aircraft position and velocity errors into aircraft frame */ 
    struct quat ecmigl_2_local_quat = quat_ecmigl2local(GEO_POS2(lat_ref, lon_ref), 0);
    
    struct quat acf_q_local = quat_rot_concat(ecmigl_2_local_quat, acf_q_earth_fixed);




    door_info_t *target_door = &door_1;

    // Make this a function that is door specific...

    

    struct quat body_frame_shift_vector = quat_from_vect3l_gl(VECT3L(target_door->lateralShift, target_door->verticalShift, target_door->axialShift));

    struct quat recovered_local_difference_vector = squat_multiply(acf_q_local, squat_multiply(body_frame_shift_vector, squat_inverse(acf_q_local)));
    vect3l_t vxyz_recovered_vect3 = quat_to_vect3l_gl(recovered_local_difference_vector);
        
    double local_x_value_shift = vxyz_recovered_vect3.x;
    double local_y_value_shift = vxyz_recovered_vect3.y;
    double local_z_value_shift = vxyz_recovered_vect3.z;

    XPLMLocalToWorld(local_x + local_x_value_shift,    
                          local_y + local_y_value_shift,    
                          local_z + local_z_value_shift,    
                          &target_door->lat,  
                          &target_door->lon,    
                          &target_door->alt);

    
    target_door->hdg = aircraftHdg + target_door->hdgOffset;


    // Find jetways that are within 200m of aircraft...
    float range = 200.0f;
    std::vector<std::pair<double, jetway_info_t*>> nearbyJetways = jetways_within_range(std::make_pair(target_door->lat, target_door->lon), range);

    
    float nearest_distance_sqrd = 2 * range * range;
    nearest_jetway = nullptr;

    double local_x_nearest_jetway, local_y_nearest_jetway, local_z_nearest_jetway;

    for (auto& distJetway : nearbyJetways) {

    	double local_x_jetway, local_y_jetway, local_z_jetway;
        
        XPLMWorldToLocal(distJetway.second->lat,  
                              distJetway.second->lon,    
                              distJetway.second->alt,    
                              &local_x_jetway,    
                              &local_y_jetway,    
                              &local_z_jetway); 

        dx = local_x_jetway - local_x;
        dy = local_y_jetway - local_y;
        dz = local_z_jetway - local_z;
        
	    struct quat local_difference_vector = quat_from_vect3l_gl(VECT3L(dx, dy, dz));
	    struct quat recovered_acf_difference_vector = squat_multiply(squat_inverse(acf_q_local), squat_multiply(local_difference_vector, acf_q_local));        
	    vect3l_t vxyz_vect3 = quat_to_vect3l_gl(recovered_acf_difference_vector);

	    double axial_error = vxyz_vect3.z;
        double vertical_error = vxyz_vect3.y;
        double lateral_error = vxyz_vect3.x;

        if (lateral_error < 0) {
        	float jetway_dist_sqrd = lateral_error * lateral_error + axial_error * axial_error;
        	if (jetway_dist_sqrd < nearest_distance_sqrd) {
        		nearest_distance_sqrd = jetway_dist_sqrd;
        		nearest_jetway = distJetway.second;
        		
        		local_x_nearest_jetway = local_x_jetway;
        		local_y_nearest_jetway = local_y_jetway;
        		local_z_nearest_jetway = local_z_jetway;

        		

        		//logMsg("[DEBUG] Candidate jetway %s to the left of aircraft at distance of %f meters", nearest_jetway->name.c_str(), std::sqrt(nearest_distance_sqrd));
        	}
        }	
    }

    if (nearest_jetway) {
    	logMsg("[DEBUG] Found jetway %s to the left of aircraft at distance of %f meters has the following params:", nearest_jetway->name.c_str(), std::sqrt(nearest_distance_sqrd));
    	logMsg("[DEBUG] latitude: %f", nearest_jetway->lat);
    	logMsg("[DEBUG] longitude: %f", nearest_jetway->lon);
    	logMsg("[DEBUG] heading: %f", nearest_jetway->hdg);
    	//logMsg("[DEBUG] minRot3: %f", nearest_jetway->minRot3);
    	//logMsg("[DEBUG] maxRot3: %f", nearest_jetway->maxRot3);
    	//logMsg("[DEBUG] minWheels: %f", nearest_jetway->minWheels);
    	//logMsg("[DEBUG] wheelPos: %f", nearest_jetway->wheelPos);
    }

    

    if (nearest_jetway) {
	    
	    setMarkerPosition();
    	

	    XPLMProbeResult probe_result = XPLMProbeTerrainXYZ(_terrainProbe,
          (float) local_x_nearest_jetway,
          (float) local_y_nearest_jetway,
          (float) local_z_nearest_jetway,
          &_terrainProbeInfo);

	    double rotundaBaseLatitude, rotundaBaseLongitude, rotundaBaseMSL;
        XPLMLocalToWorld(_terrainProbeInfo.locationX,    
                         _terrainProbeInfo.locationY,    
                         _terrainProbeInfo.locationZ,    
                         &rotundaBaseLatitude,  
                         &rotundaBaseLongitude,    
                         &rotundaBaseMSL);


    	// Need to calculate the lat and lon of where the cabin should be...

    	// If door .hdgOffset = 0 then we apply a negative (for left door) lateral offset of the cabinLength...
    	// in the aircraft coordinates.  If .hdgOffset > 0 then we do cos() that for lateral and -sin() for axial...

    	struct quat cabin_body_frame_shift_vector 
    				= quat_from_vect3l_gl(VECT3L(-nearest_jetway->cabinLength*cos(DEG2RAD(target_door->hdgOffset)), 
    											 0, -nearest_jetway->cabinLength*sin(DEG2RAD(target_door->hdgOffset))));

	    struct quat cabin_recovered_local_difference_vector = squat_multiply(acf_q_local, squat_multiply(cabin_body_frame_shift_vector, squat_inverse(acf_q_local)));
	    vect3l_t cabin_vxyz_recovered_vect3 = quat_to_vect3l_gl(cabin_recovered_local_difference_vector);
	        
	    double local_x_value_shift = vxyz_recovered_vect3.x;
	    double local_y_value_shift = vxyz_recovered_vect3.y;
	    double local_z_value_shift = vxyz_recovered_vect3.z;

	    double cabin_1_lat, cabin_1_lon, cabin_1_alt;

	    XPLMLocalToWorld(local_x + local_x_value_shift + cabin_vxyz_recovered_vect3.x,    
	                          local_y + local_y_value_shift + cabin_vxyz_recovered_vect3.y,    
	                          local_z + local_z_value_shift + cabin_vxyz_recovered_vect3.z,    
	                          &cabin_1_lat,  
	                          &cabin_1_lon,    
	                          &cabin_1_alt);

		//logMsg("[DEBUG] Aircraft alt is %f, door alt is %f, door vertical shift is %f", aircraftAlt, target_door->alt, target_door->verticalShift);

	    //logMsg("[DEBUG] Cabin 1 alt is %f", cabin_1_alt);

	    //logMsg("[DEBUG] Rotunda base is at alt %f meters msl", rotundaBaseMSL);

		geo_pos2_t rotunda_geo_pos2 = GEO_POS2(nearest_jetway->lat, nearest_jetway->lon);
		geo_pos2_t cabin_geo_pos2 = GEO_POS2(cabin_1_lat, cabin_1_lon);	                          

	    double bearing_from_rotunda_to_cabin = gc_point_hdg(rotunda_geo_pos2, cabin_geo_pos2);
	
	    nearest_jetway->rotate1_target = shift_normalized_hdg(normalize_hdg(bearing_from_rotunda_to_cabin - nearest_jetway->hdg));

		nearest_jetway->rotate2_target = shift_normalized_hdg(normalize_hdg(90 - (nearest_jetway->hdg + nearest_jetway->rotate1_target) + target_door->hdg));
		//logMsg("[DEBUG] Rotate 2 target is %f degrees", nearest_jetway->rotate2_target );

		// CONCERN: I think we should just convert the jetway position to local_x, local_y, local_z
		//          and use those and the shifts above to in opengl coordinates calculate these distances?

		double heightRise = cabin_1_alt - nearest_jetway->height - rotundaBaseMSL;

		//logMsg("[DEBUG] Height rise is %f", heightRise);

		double distance_from_rotunda_to_cabin = gc_distance(rotunda_geo_pos2, cabin_geo_pos2); 
		

		nearest_jetway->extent_target = std::sqrt(heightRise*heightRise + 
			                                      distance_from_rotunda_to_cabin*distance_from_rotunda_to_cabin)
											 - nearest_jetway->cabinPos;

		if (nearest_jetway->extent_target + nearest_jetway->cabinPos != 0) {
			nearest_jetway->rotate3_target = RAD2DEG(std::asin(heightRise/(nearest_jetway->extent_target + nearest_jetway->cabinPos)));
		} else {
			logMsg("[ERROR] Can't calculate rotate3 target because extent target + cabin position sums to zero, would have divide by zero!");
			nearest_jetway->rotate3_target = 0;
		}

	}

	
	if (_markerObjectRef) {
		setManualMarkerPosition();

		setSAMmarkerPositions();
	}

	//Return the interval we next want to be called in..
	return 1.0f;
}




/**********************************************/

/*
* XPluginStop
*
* Our cleanup routine deallocates our window.
*
*/
PLUGIN_API void	XPluginStop(void)
{
	XPLMUnregisterFlightLoopCallback(flightLoopCallback, NULL);	 //  Don't forget to unload this callback.  
	
    //XPLMUnregisterFlightLoopCallback(DataRefEditorRegistrationFlightLoopCallback, NULL);

	dr_delete(&sam_jetway_rotate1_dr);
	dr_delete(&sam_jetway_rotate2_dr);
	dr_delete(&sam_jetway_rotate3_dr);
	dr_delete(&sam_jetway_extent_dr);
	dr_delete(&sam_jetway_wheels_dr);
	dr_delete(&sam_jetway_wheelrotatel_dr);
	dr_delete(&sam_jetway_wheelrotater_dr);
	dr_delete(&sam_jetway_wheelrotatec_dr);
	dr_delete(&sam_jetway_warnlight_dr);

	if (_markerObjectRef) {
		XPLMUnloadObject(_markerObjectRef);
	}
  
    notYetFullyLoaded = true;

 	log_fini();
   
	unregister_crash_handler();

   	XPLMDebugString("SamIAm plugin stopped...\n");
}


/*
* XPluginDisable
*
* We do not need to do anything when we are disabled, but we must provide the handler.
*
*/
PLUGIN_API void XPluginDisable(void)
{

}

/*
* XPluginEnable.
*
* We don't do any enable-specific initialization, but we must return 1 to indicate
* that we may be enabled at this time.
*
*/
PLUGIN_API int XPluginEnable(void)
{
	return 1;
}

/*
* XPluginReceiveMessage
*
* Here we will detect things like plane changes, scenery reloads,
* and ALSO to communicate with the SASL UI written in LUA.
*
*/
PLUGIN_API void XPluginReceiveMessage(
	XPLMPluginID	inFromWho,
	int				inMessage,
	void *			inParam)
{
    
    switch (inMessage) {

    	case XPLM_MSG_PLANE_CRASHED:
    		/* This message is sent to your plugin whenever the user's plane crashes.      */

    		break;

    	case XPLM_MSG_PLANE_LOADED:
    		/* This message is sent to your plugin whenever a new plane is loaded.  The    *
			 * parameter is the number of the plane being loaded; 0 indicates the user's   *
			 * plane.                                                                      */
    		
    		//NOTE: This is an absurd aspect of the XPLM that a void* is actually an int!
    		if (inParam == 0) {
    			logMsg("Aircraft loaded...");

    			readAircraftDoorProperties();
    		}

    		break; 

    	case XPLM_MSG_AIRPORT_LOADED:
    		/* This messages is called whenever the user's plane is positioned at a new    *
			 * airport.                                                                    */
    		
    		//logMsg("New airport loaded...");


    		break; 
    	case XPLM_MSG_SCENERY_LOADED:
    		/* This message is sent whenever new scenery is loaded.  Use datarefs to       *
			 * determine the new scenery files that were loaded.                           */
    		
    		//logMsg("New scenery loaded...");

    		break; 
    	case XPLM_MSG_AIRPLANE_COUNT_CHANGED:
    		/* This message is sent whenever the user adjusts the number of X-Plane        *
			 * aircraft models.  You must use XPLMCountPlanes to find out how many planes  *
			 * are now available.  This message will only be sent in XP7 and higher        *
			 * because in XP6 the number of aircraft is not user-adjustable.               */

    		break; 

#if defined(XPLM200)
		case XPLM_MSG_PLANE_UNLOADED:
			/* This message is sent to your plugin whenever a plane is unloaded.  The      *
			 * parameter is the number of the plane being unloaded; 0 indicates the user's *
			 * plane.  The parameter is of type int, passed as the value of the pointer.   *
			 * (That is: the parameter is an int, not a pointer to an int.)                */

			//logMsg("Aircraft unloaded...");

			break;
#endif /* XPLM200 */
#if defined(XPLM210)
		case XPLM_MSG_WILL_WRITE_PREFS:
			/* This message is sent to your plugin right before X-Plane writes its         *
			 * preferences file.  You can use this for two purposes: to write your own     *
			 * preferences, and to modify any datarefs to influence preferences output.    *
			 * For example, if your plugin temporarily modifies saved preferences, you can *
			 * put them back to their default values here to avoid  having the tweaks be   *
			 * persisted if your plugin is not loaded on the next invocation of X-Plane.   */


			break;
#endif /* XPLM210 */
#if defined(XPLM210)
		case XPLM_MSG_LIVERY_LOADED:
			/* This message is sent to your plugin right after a livery is loaded for an   *
			 * airplane.  You can use this to check the new livery (via datarefs) and      *
			 * react accordingly.  The parameter is of type int, passed as the value of a  *
			 * pointer and represents the aicraft plane number - 0 is the user's plane.    */

			break;
#endif /* XPLM210 */

#if defined(XPLM301)
		case XPLM_MSG_ENTERED_VR:
			/* Sent to your plugin right before X-Plane enters virtual reality mode (at    *
			 * which time any windows that are not positioned in VR mode will no longer be *
			 * visible to the user).                                                       */
			break;
#endif /* XPLM301 */
#if defined(XPLM301)
		case XPLM_MSG_EXITING_VR:
			/* Sent to your plugin right before X-Plane leaves virtual reality mode (at    *
			 * which time you may want to clean up windows that are positioned in VR       *
			 * mode).                                                                      */
			break;
#endif /* XPLM301 */

    	
    }
    
}

void menu_handler(void * in_menu_ref, void * in_item_ref)
{
	if (!strcmp((const char *) in_item_ref, "Changelog")) {
		
	}
}


/* X-Plane 12 Native Jeways ....

	sim/graphics/animation/jetways/jw_base_rotation	float	n		
	sim/graphics/animation/jetways/jw_tunnel_pitch	float	n		
	sim/graphics/animation/jetways/jw_tunnel_extension	float	n		
	sim/graphics/animation/jetways/jw_cabin_rotation	float	n		
	sim/graphics/animation/jetways/jw_bogie_elevation	float	n		
	sim/graphics/animation/jetways/jw_bogie_rotation	float	n		
	sim/graphics/animation/jetways/jw_bogie_bogie_tilt	float	n		
	sim/graphics/animation/jetways/jw_wheel_left	float	n		
	sim/graphics/animation/jetways/jw_wheel_right	float	n		
	sim/graphics/animation/jetways/jw_stairs_angle	float	n		
	sim/graphics/animation/jetways/jw_stairs_bogie_angle	float	n		
	sim/graphics/animation/jetways/jw_is_moving

NOTE THEY ARE NOT WRITABLE.  WHY NOT ALLOW US TO WRITE AND OVERRIDE BEHAVIOR?

*/