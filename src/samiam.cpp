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
	#include <string.h>
	#include <math.h>

	#include <acfutils/crc64.h>
	#include <acfutils/glew.h>
	#include <acfutils/osrand.h>
	#include <acfutils/dr.h>
	#include <acfutils/geom.h>
	#include <acfutils/log.h>

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


#define earth_rad   6378145.0

static inline void
pl21_XPLMWorldToLocal(double lat, double lon, double ele,
    double *x, double *y, double *z)
{
    double ref_lat, ref_lon, ref_elev;
    double rad;
    double del_lon_rad, act_lat_rad;
    double sin_del_lon, cos_del_lon;
    double sin_act_lat, cos_act_lat;
    double sin_ref_lat, cos_ref_lat;
    double x_e, y_e, z_e;

    ASSERT(x != NULL);
    ASSERT(y != NULL);
    ASSERT(z != NULL);

    XPLMLocalToWorld(0, 0, 0, &ref_lat, &ref_lon, &ref_elev);

    rad = earth_rad + ele;
    del_lon_rad = DEG2RAD(lon) - DEG2RAD(ref_lon);
    act_lat_rad = DEG2RAD(lat);

    sin_del_lon = sinl(del_lon_rad);
    cos_del_lon = cosl(del_lon_rad);
    sin_act_lat = sinl(act_lat_rad);
    cos_act_lat = cosl(act_lat_rad);

    sin_ref_lat = sinl(DEG2RAD(ref_lat));
    cos_ref_lat = cosl(DEG2RAD(ref_lat));

    x_e = rad * (sin_del_lon * cos_act_lat);
    y_e = rad * (cos_del_lon * cos_act_lat) - earth_rad * cos_ref_lat;
    z_e = rad * (-sin_act_lat) + earth_rad * sin_ref_lat;

    *x = x_e;
    *y = y_e * cos_ref_lat - z_e * sin_ref_lat;
    *z = z_e * cos_ref_lat + y_e * sin_ref_lat;
}

static inline void
pl21_XPLMLocalToWorld(double x, double y, double z,
    double *lat, double *lon, double *ele)
{
    double ref_lat, ref_lon, ref_elev;
    double sin_ref_lat, cos_ref_lat;
    double x_e, y_e, z_e, rad;

    ASSERT(lat != NULL);
    ASSERT(lon != NULL);
    ASSERT(ele != NULL);

    XPLMLocalToWorld(0, 0, 0, &ref_lat, &ref_lon, &ref_elev);

    sin_ref_lat = sinl(DEG2RAD(ref_lat));
    cos_ref_lat = cosl(DEG2RAD(ref_lat));
    x_e = x;
    y_e = y * cos_ref_lat + z * sin_ref_lat + earth_rad * cos_ref_lat;
    z_e = z * cos_ref_lat - y * sin_ref_lat - earth_rad * sin_ref_lat;
    rad = sqrtl(x_e * x_e + y_e * y_e + z_e * z_e);

    *ele = rad - earth_rad;
    *lat = RAD2DEG(asinl(-z_e / rad));
    *lon = RAD2DEG(atan2l(x_e, y_e)) + ref_lon;
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

static double aircraftLat, aircraftLon, aircraftAlt;


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

	double bearing_from_rotunda_to_aircraft = gc_point_hdg(GEO_POS2(jetway->lat, jetway->lon), GEO_POS2(aircraftLat, aircraftLon));
	
	//slide_back_and_forth(&jetway->rotate1, &jetway->rotate1_target, jetway->minRot1, jetway->maxRot1, 0.01);
	
	jetway->rotate1 = normalize_hdg(bearing_from_rotunda_to_aircraft - jetway->hdg);

	slide_back_and_forth(&jetway->rotate2, &jetway->rotate2_target, jetway->minRot2, jetway->maxRot2, 0.0123);
	slide_back_and_forth(&jetway->rotate3, &jetway->rotate3_target, jetway->minRot3, jetway->maxRot3, 0.0217);
	
	slide_back_and_forth(&jetway->extent, &jetway->extent_target, jetway->minExtent, jetway->maxExtent, 0.0179);

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
	const float hdgTolerance = 3.0f;

	std::vector<std::pair<double, jetway_info_t*>> nearbyDistJetways = jetways_within_range_in_map(spot, 5*tolerance, samXMLjetways);

    logMsg("[DEBUG] Looking at a total of %d sam xml jetways to find a match...", nearbyDistJetways.size());

    for (auto& distJetway : nearbyDistJetways) {

    	logMsg("[DEBUG] Distance found is %f meters", distJetway.first);

    	if (distJetway.first < nearest_distance) {
    		float headingDifference = normalize_hdg(std::abs(distJetway.second->hdg - targetHdg));
    		if (headingDifference < hdgTolerance) {
	    		nearest_distance = distJetway.first;
    			candidate_jetway = distJetway.second;	
        	} else {
        		logMsg("[WARN] Found closer possible jetway %s but heading differs by %f, more than %f degrees!", distJetway.second->name.c_str(), headingDifference, hdgTolerance);
        	}
        } else {
        	logMsg("[WARN] Jetway %s is not closer", distJetway.second->name.c_str());
        }	
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

	pl21_XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
                          dr_getf(&draw_object_y_dr),    
                          dr_getf(&draw_object_z_dr),    
                          &worldLatitude,  
                          &worldLongitude,    
                          &worldAltitude);
	
	float drawObjectHeading = dr_getf(&draw_object_psi_dr);

	std::pair<float, float> spot = std::pair(worldLatitude, worldLongitude);

	jetway_info_t *foundJetway = jetway_found(spot);

	if (!foundJetway) {

		foundJetway = jetway_found_sam_xml(spot, drawObjectHeading, 50.0);

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
		
		if (foundJetway == nearest_jetway) {
			random_animate_jetway(foundJetway);
		} else {
			foundJetway->rotate1 = foundJetway->initialRot1;
		}

		*((float*) valueout) = foundJetway->rotate1;
	}
	
	return B_TRUE;
}

bool_t jetway_rotate2_r_cb(dr_t *dr, void *valueout)
{
	UNUSED(dr);
	
	double worldLatitude, worldLongitude, worldAltitude;

	pl21_XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
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

	pl21_XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
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

	pl21_XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
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

	pl21_XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
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

	pl21_XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
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

	pl21_XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
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

	pl21_XPLMLocalToWorld(dr_getf(&draw_object_x_dr),    
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

	fdr_find(&lat_ref_dr, "sim/flightmodel/position/lat_ref");
	fdr_find(&lon_ref_dr, "sim/flightmodel/position/lon_ref");

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

	// This FLCB will register our custom dataref in DRE
	//XPLMRegisterFlightLoopCallback(DataRefEditorRegistrationFlightLoopCallback, 1, NULL);   
	
	// This will get called during each flight loop and we will handle network events
	XPLMRegisterFlightLoopCallback(flightLoopCallback, 1, NULL);

	findSAMxmlFilesInCustomSceneries();

	return 1;
}


float DataRefEditorRegistrationFlightLoopCallback(float elapsedme, float elapsedSim, int counter, void * refcon)
{
    XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
    if (PluginID != XPLM_NO_PLUGIN_ID){
        //XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)"SharedFlight/seat");
    }
    
    return 0.0f;  // Flight loop is called only once!
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

    double dx, dy, dz;
   
    pl21_XPLMLocalToWorld(local_x,    
                         local_y,    
                         local_z,    
                         &aircraftLat,  
                         &aircraftLon,    
                         &aircraftAlt);

    // Find jetways that are within 200m of aircraft...
    float range = 200.0f;
    std::vector<std::pair<double, jetway_info_t*>> nearbyJetways = jetways_within_range(std::make_pair(aircraftLat, aircraftLon), range);

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
    
    //struct quat local_pilot_quat = {_pilotInputInfo.q[1], _pilotInputInfo.q[2], _pilotInputInfo.q[3], _pilotInputInfo.q[0]};
    struct quat acf_q_local = quat_rot_concat(ecmigl_2_local_quat, acf_q_earth_fixed);

    float nearest_distance_sqrd = 2 * range * range;
    nearest_jetway = nullptr;

    for (auto& distJetway : nearbyJetways) {

    	double local_x_jetway, local_y_jetway, local_z_jetway;
        
        pl21_XPLMWorldToLocal(distJetway.second->lat,  
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
        		//logMsg("[DEBUG] Candidate jetway %s to the left of aircraft at distance of %f meters", nearest_jetway->name.c_str(), std::sqrt(nearest_distance_sqrd));
        	}
        }	
    }

    if (nearest_jetway) {
    	logMsg("[DEBUG] Found jetway %s to the left of aircraft at distance of %f meters", nearest_jetway->name.c_str(), std::sqrt(nearest_distance_sqrd));
    	logMsg("[DEBUG] Jetway %s has the following params:\n", nearest_jetway->name.c_str());
    	logMsg("[DEBUG] minRot3: %f", nearest_jetway->minRot3);
    	logMsg("[DEBUG] maxRot3: %f", nearest_jetway->maxRot3);
    	logMsg("[DEBUG] minWheels: %f", nearest_jetway->minWheels);
    	logMsg("[DEBUG] wheelPos: %f", nearest_jetway->wheelPos);
    }

	//Return the interval we next want to be called in..
	return 5.0f;
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
