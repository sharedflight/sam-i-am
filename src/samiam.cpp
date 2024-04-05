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
#include <deque>
#include <string>
#include <algorithm>

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

const double KTS2MPS = 0.514444;

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

static int xp_ver = 0;

static dr_t	sam_jetway_rotate1_dr; // -/+ 90 degrees | Rotation of the entire jetway around Z-axis at 0,0,0
static dr_t sam_jetway_rotate2_dr; // -/+ 90 degrees | Rotation of the docking door
static dr_t sam_jetway_rotate3_dr; // -/+ 6 degrees  | Vertical rotation of the gangway around X-axis
static dr_t	sam_jetway_extent_dr; // 0-10 meters | Gangway extent in meters along Y-axis
static dr_t sam_jetway_wheels_dr; // -/+ 2 meters | Distance of vertical translation of the wheel pillar
static dr_t sam_jetway_wheelrotatel_dr, sam_jetway_wheelrotater_dr; // 0-360 degrees | Left wheel rotation (forward)
static dr_t sam_jetway_wheelrotatec_dr; //  -/+ 90 degrees | Wheel turn axis rotation
static dr_t sam_jetway_warnlight_dr; //  0/1 | status 1 = Jetway is operating

static dr_t sam3_docking_status_dr;
static dr_t sam3_vdgs_status_dr;
static dr_t sam3_docking_longitudinal_dr;
static dr_t sam3_docking_lateral_dr;
static dr_t sam_vdgs_status_dr;
static dr_t sam_docking_lateral_dr;
static dr_t sam_docking_longitudinal_dr;

static dr_t draw_object_x_dr, draw_object_y_dr, draw_object_z_dr, draw_object_psi_dr;

static dr_t local_x_dr, local_y_dr, local_z_dr;
static dr_t q_dr, lat_ref_dr, lon_ref_dr;
static dr_t true_psi_dr;

static dr_t gear_on_ground_dr;
static dr_t gear_deploy_dr;
static dr_t gear_types_dr;
static dr_t gear_steers_dr;
static dr_t tire_z_dr;

static dr_t beacon_on_dr;
static dr_t fnrml_gear_dr;
static dr_t groundspeed_dr;
static dr_t engine_burning_fuel_dr;

bool need_acf_properties = false;

typedef std::pair<double, double> LatLonSpot;

static XPLMProbeRef _terrainProbe;
static XPLMProbeInfo_t _terrainProbeInfo;


typedef enum OpsState
{
	NOSTATE,
	DISCONNECTED_TAXI_IN,
	CONNECTING,
	CONNECTED,
	DISCONNECTING,
	DISCONNECTED_TAXI_OUT
} OpsState;

static OpsState currentState;

typedef enum DoorType
{
    ANYDoor,
    LF1Door,
    LF2Door,
    LR1Door,
    OTHERDoor
} DoorType;

typedef enum DoorConnection
{
    DOORDISCONNECTED,
    DOORWILLCONNECT,
    DOORJETWAYMOVING,
    DOORCONNECTED
} DoorConnection;

template<typename T>
bool fuzzyCompare(T a, T b, int k)
{
     return std::abs(a - b) <= std::numeric_limits<T>::epsilon() * std::abs(a + b) * k;
}

bool same_spot(LatLonSpot spot1, LatLonSpot spot2)
{
	return fuzzyCompare(spot1.first, spot2.first, 4) && fuzzyCompare(spot1.second, spot2.second, 4);
}

struct aircraft_info_s;
struct door_info_s;
struct jetway_info_s;

typedef aircraft_info_s aircraft_info_t;
typedef door_info_s door_info_t;
typedef jetway_info_s jetway_info_t;

struct aircraft_info_s {
	double local_x;
	double local_y;
	double local_z;
	struct quat q_local;
	double lat;
	double lon;
	double alt;
	float hdg;
	bool beacon;
	bool engines;
	bool airborne;
	float velocity;
	double nosewheelZ;
	LatLonSpot nosewheelSpot;
	std::map<DoorType, door_info_t> doors;
};

typedef struct door_position_s {
	double lat;
	double lon;
	double alt;
	float hdg;
	double shift_x;
	double shift_y;
	double shift_z;
} door_position_t;

struct door_info_s {
	float axialShift;
	float lateralShift;
	float verticalShift;
	float hdgOffset; // Door might be on curved part of aircraft
	DoorType type;
	aircraft_info_t *aircraft;
	DoorConnection connection;
	jetway_info_t *jetway;
};

typedef struct dock_info_s {
	std::string name;
	double lat;
	double lon;
	double elev;
	double alt;
	float hdg;
	bool fixed;
	double dockLat;
	double dockLon;
	float dockHdg;
	int status;
	float latGuidance;
	float lonGuidance;
	XPLMInstanceRef markerInstanceRef;
} dock_info_t;

typedef struct jetway_targets_s {
	float rot1;
	float rot2;
	float rot3;
	float extent;
	float dist;
} jetway_targets_t;

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
	// float rotate1_target;
	// float rotate2_target;
	// float rotate3_target;
	// float extent_target;
	// For jetway connection...
	jetway_targets_t targets;
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
	DoorType forDoorLocation;
	XPLMInstanceRef markerInstanceRef;
};

typedef struct jetway_position_s {
	double local_x;
	double local_y;
	double local_z;
	double alt;
} jetway_position_t;


static aircraft_info_t aircraft;

static dock_info_t* nearest_dock = nullptr;

// These are read in for ALL scenery from the samiam.xml and sam.xml files...
std::map<LatLonSpot, jetway_info_t> samXMLjetways;
std::map<LatLonSpot, dock_info_t> samXMLdocks;

// These are based on what is FOUND in sim by datarefs being called by drawn objects...
// these are populated from the xml maps, and can also be culled (example while airborne)
std::map<LatLonSpot, jetway_info_t> jetways;
std::map<LatLonSpot, dock_info_t> docks;


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

// This code is adapted from read_gear_info() in bp.c 
// of the BetterPushbackC mod by Saso...
static bool
readAircraftGearInfo(void) {
    double tire_z[10];
    int gear_steers[10], gear_types[10], gear_on_ground[10];
    int gear_deploy[10];

    int nw_i;        /* nose gear index the gear tables */
    int n_gear = 0;        	/* number of gear */
    int gear_is[10];    /* gear index in the gear tables */

    dr_getvi(&gear_deploy_dr, gear_deploy, 0, 10);
    
    if (xp_ver >= 11000)
        dr_getvi(&gear_on_ground_dr, gear_on_ground, 0, 10);
    else
        memset(gear_on_ground, 0xff, sizeof(gear_on_ground));

    /* First determine where the gears are */
    for (int i = 0, n = dr_getvi(&gear_types_dr, gear_types, 0, 10); i < n; i++) {
        /*
         * Gear types are:
         * 0) Nothing.
         * 1) Skid.
         * 2+) Wheel based gear in various arrangements. A tug can
         *	provide a filter for this.
         *
         * Also make sure to ONLY select gears which are deployed and
         * are touching the ground. Some aircraft models have weird
         * gears which are, for whatever reason, hovering in mid air
         * (huh?).
         */
        if (gear_types[i] >= 2 && gear_on_ground[i] != 0 &&
            gear_deploy[i] != 0)
            gear_is[n_gear++] = i;
    }

    if (!n_gear) {
    	aircraft.nosewheelZ = 0;
    	logMsg("[ERROR] Failed to find gear!");
    	return false;
    }

    /* Read nosegear long axis deflections */
    VERIFY3S(dr_getvf(&tire_z_dr, tire_z, 0, 10), >=, n_gear);
    nw_i = -1;
    aircraft.nosewheelZ = 1e10;

    logMsg("[DEBUG] Here...");

    /* Next determine which gear steers. Pick the one most forward. */
    VERIFY3S(dr_getvi(&gear_steers_dr, gear_steers, 0, 10), >=, n_gear);
    for (int i = 0; i < n_gear; i++) {
        if (gear_steers[gear_is[i]] == 1 &&
            tire_z[gear_is[i]] < aircraft.nosewheelZ) {
            nw_i = gear_is[i];
            aircraft.nosewheelZ = tire_z[gear_is[i]];
        }
    }

    logMsg("[DEBUG] Found nosewheel index of %d", nw_i);

    /*
     * Aircraft appears to not have any steerable gears.
     * Hope same fix as on the tu154 helps here...
     */
    if (nw_i == -1) {
        nw_i = gear_is[0];
        aircraft.nosewheelZ = tire_z[gear_is[0]];
    }

    logMsg("[DEBUG] Found aircraft.nosewheelZ of %f", aircraft.nosewheelZ);

    // /* Nose gear strut length and tire radius */
    // VERIFY3S(dr_getvf(&drs.leg_len, &bp.acf.nw_len, bp.acf.nw_i, 1), ==, 1);
    // VERIFY3S(dr_getvf(&drs.tirrad, &bp.acf.tirrad, bp.acf.nw_i, 1), ==, 1);

    // /* Read nosewheel type */
    // bp.acf.nw_type = gear_types[bp.acf.nw_i];

    // /* Compute main gear Z deflection as mean of all main gears */
    // for (int i = 0; i < bp.acf.n_gear; i++) {
    //     if (bp.acf.gear_is[i] != bp.acf.nw_i)
    //         bp.acf.main_z += tire_z[bp.acf.gear_is[i]];
    // }
    // bp.acf.main_z /= bp.acf.n_gear - 1;

    return true;
}

void random_animate_dock(dock_info_t *dock) {
	
	if (need_acf_properties) return;

	auto noswewheelLoc = GEO_POS2(aircraft.nosewheelSpot.first, aircraft.nosewheelSpot.second);
	
	auto dockLoc = GEO_POS2(dock->dockLat, dock->dockLon);

	double dist = gc_distance(noswewheelLoc, dockLoc);

	dock->status = 0;

	if (dist < 100) {
		auto bearing = shift_normalized_hdg(normalize_hdg(dock->dockHdg - gc_point_hdg(noswewheelLoc, dockLoc)));

		// Note... at close distances bearing doesn't mean much...
		if (dist < 10 || std::abs(bearing) < 30) {

			// Calculate lateral and longitudinal offset... 

			dock->lonGuidance = static_cast<float>(dist * cos(DEG2RAD(bearing)));
			dock->latGuidance = static_cast<float>(-dist * sin(DEG2RAD(bearing))); // Note: sign flip	

			if (dock->lonGuidance < 0.05) {
				dock->status = 2;

				if (nearest_dock != dock) {
					nearest_dock = dock;
					logMsg("[DEBUG] Found new nearest dock %s out of %d docks", nearest_dock->name.c_str(), docks.size());
				}

			} else if (dock->lonGuidance < 25 || (dock == nearest_dock && ( std::abs(bearing) < 20 || dock->latGuidance < 15))) {
				dock->status = 1;
				
				if (nearest_dock != dock) {
					nearest_dock = dock;
					logMsg("[DEBUG] Found new nearest dock %s out of %d docks", nearest_dock->name.c_str(), docks.size());
				}

				//logMsg("[DEBUG] Dock %s: range %f meters, bearing %f, lon guidance %f, lat guidance %f, status %d", 
				//			dock->name.c_str(), dist, bearing, dock->lonGuidance, dock->latGuidance, dock->status);
			}

			// Only animate one dock at a time...
			if (nearest_dock && dock != nearest_dock) {
				dock->status = 0;
			}
		}
	} else {
		dock->status = 0;
	}
}

bool random_animate_jetway(jetway_info_t *jetway) {

	if (need_acf_properties) return true;

	bool still_moving = true;

	if (jetway->rotate2 != jetway->targets.rot2) {
		logMsg("[DEBUG] Moving rotate 2..");
		slide_to(&jetway->rotate2, jetway->targets.rot2, 0.217);
	} else if (jetway->rotate1 != jetway->targets.rot1) {
		logMsg("[DEBUG] Moving rotate 1..");
		slide_to(&jetway->rotate1, jetway->targets.rot1, 0.0317);
	} else if (jetway->rotate3 != jetway->targets.rot3 || 
			   jetway->extent != jetway->targets.extent) {
		logMsg("[DEBUG] Moving rotate 3 and extent..");
		slide_to(&jetway->rotate3, jetway->targets.rot3, 0.00217);
		slide_to(&jetway->extent, jetway->targets.extent, 0.0179);
	} else {
		still_moving = false;
	}

	// Handle the wheel pillar...

	// wheelPos is the distance of the wheel pillar from the rotunda...
	jetway->wheels = (jetway->wheelPos + jetway->extent) * sin(DEG2RAD(jetway->rotate3));

	return still_moving;
}

template<typename T>
T* sameLatLonSpotInMap(LatLonSpot& spot, std::map<LatLonSpot, T>& mapping)
{
     for(auto& [spotKey, item] : mapping) {
		if (same_spot(spot, spotKey)) {
			return &item;
		}
	}
	return nullptr;
}

// CONCERN: Both of these should also be passed the heading and that should be checked in this as well...!!

jetway_info_t* jetway_found(LatLonSpot spot) {
	return sameLatLonSpotInMap(spot, jetways);
}

dock_info_t* dock_found(LatLonSpot spot) {
	return sameLatLonSpotInMap(spot, docks);
}

template<typename T>
std::vector<std::pair<double, T*>> within_range_in_map(LatLonSpot spot, float range, std::map<LatLonSpot, T>& mapping) {
	geo_pos2_t targetLoc = GEO_POS2(spot.first, spot.second);
	
	std::vector<std::pair<double, T*>> nearbyItems;

	for(auto& [spotKey, item] : mapping) {
		
		auto itemLoc = GEO_POS2(spotKey.first, spotKey.second);

		double dist = gc_distance(targetLoc, itemLoc);

		if (dist < range) {
			nearbyItems.push_back(std::make_pair(dist, &item));
		}
	}

	return nearbyItems;
}

std::vector<jetway_info_t*> nearby_jetways(LatLonSpot spot, float range) {

	geo_pos2_t targetLoc = GEO_POS2(spot.first, spot.second);
	
	std::vector<jetway_info_t*> nearby;

	for(auto& [spotKey, jetway] : jetways) {
		
		auto jetwayLoc = GEO_POS2(spotKey.first, spotKey.second);

		double dist = gc_distance(targetLoc, jetwayLoc);

		if (dist < range) {
			nearby.push_back(&jetway);
		}
	}

	return nearby;
}

std::vector<std::pair<double, jetway_info_t*>> jetways_within_range(LatLonSpot spot, float range) {

	// TODO: We should actually make this look at the distance between door and parked cab position
	//       of the jetbridge!

	return within_range_in_map(spot, range, jetways);
}

jetway_info_t* jetway_found_sam_xml(LatLonSpot spot, float targetHdg, float tolerance) {

	float nearest_distance = 5*tolerance;
    jetway_info_t* candidate_jetway = nullptr;
	const float hdgTolerance = 3.0f;

	float candidate_jetway_hdg_difference = 0;

	std::vector<std::pair<double, jetway_info_t*>> nearbyDistJetways = within_range_in_map(spot, 5*tolerance, samXMLjetways);

    //logMsg("[DEBUG] Looking at a total of %d sam xml jetways to find a match...", nearbyDistJetways.size());

    for (auto& distJetway : nearbyDistJetways) {

    	if (distJetway.first < nearest_distance) {
    		float headingDifference = std::abs(shift_normalized_hdg(normalize_hdg(distJetway.second->hdg - targetHdg)));
    		if (headingDifference < hdgTolerance) {
	    		nearest_distance = distJetway.first;
    			candidate_jetway = distJetway.second;
    			candidate_jetway_hdg_difference = headingDifference;
        	}
        }
    }

    //if (candidate_jetway) { 
    	//logMsg("[INFO] Found closest possible SAM jetway %s, heading differs by %f degrees!", candidate_jetway->name.c_str(), candidate_jetway_hdg_difference);	
    //}

    return candidate_jetway;
}



dock_info_t* dock_found_sam_xml(LatLonSpot spot, float targetHdg, float tolerance) {

	float nearest_distance = 5*tolerance;
    dock_info_t* candidate_dock = nullptr;
	const float hdgTolerance = 3.0f;

	float candidate_dock_hdg_difference = 0;

	std::vector<std::pair<double, dock_info_t*>> nearbyDistDocks = within_range_in_map(spot, 5*tolerance, samXMLdocks);

    //logMsg("[DEBUG] Looking at a total of %d sam xml docks to find a match...", nearbyDistDocks.size());

    for (auto& distDock : nearbyDistDocks) {

    	if (distDock.first < nearest_distance) {
    		float headingDifference = std::abs(shift_normalized_hdg(normalize_hdg(distDock.second->hdg - targetHdg)));
    		if (headingDifference < hdgTolerance) {
	    		nearest_distance = distDock.first;
    			candidate_dock = distDock.second;
    			candidate_dock_hdg_difference = headingDifference;
        	}
        }	
    }

    if (candidate_dock) { 
    	logMsg("[INFO] Found closest possible SAM dock %s, heading differs by %f degrees!", candidate_dock->name.c_str(), candidate_dock_hdg_difference);	
    }

    return candidate_dock;
}


bool_t jetway_w_cb(dr_t *dr, void *valuein)
{
	UNUSED(dr);
	UNUSED(valuein);

	logMsg("[ERROR] Jetway write callback next expected to be called, and does nothing presently");
	return B_TRUE;
}

bool_t docking_w_cb(dr_t *dr, void *valuein)
{
	UNUSED(dr);
	UNUSED(valuein);

	logMsg("[ERROR] Docking write callback next expected to be called, and does nothing presently");
	return B_TRUE;
}

bool_t docking_status_r_cb(dr_t *dr, void *valueout)
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

	LatLonSpot spot = std::pair(worldLatitude, worldLongitude);

	dock_info_t *foundDock = dock_found(spot);

	if (!foundDock) {

		foundDock = dock_found_sam_xml(spot, drawObjectHeading, 5.0);

		if (foundDock) {
		
			dock_info_t newDock;

		
			newDock = *foundDock;

			// Lets use the precision of the sim... greater than the xml file!
			newDock.lat = worldLatitude;
			newDock.lon = worldLongitude;
			newDock.alt = worldAltitude;
			newDock.hdg = dr_getf(&draw_object_psi_dr);

			docks[spot] = newDock;

			logMsg("[DEBUG] Found a SAM xml specified dock with name %s", newDock.name.c_str());
		} else {		
			//logMsg("[WARN] Found an unknown new dock at at lat %f, lon %f, alt %f", worldLatitude, worldLongitude, worldAltitude);
			*((float*) valueout) = 0;
		}
		
	} else {
		
		// Lets use the precision of the sim... greater than the xml file!
		

		foundDock->lat = worldLatitude;
		foundDock->lon = worldLongitude;
		foundDock->alt = worldAltitude;
		foundDock->hdg = dr_getf(&draw_object_psi_dr);

		random_animate_dock(foundDock);

		*((int*) valueout) = foundDock->status;
	}
	
	return B_TRUE;

}

bool_t docking_lateral_r_cb(dr_t *dr, void *valueout)
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

	LatLonSpot spot = std::pair(worldLatitude, worldLongitude);

	dock_info_t *foundDock = dock_found(spot);

	if (foundDock) {
		*((float*) valueout) = foundDock->latGuidance;
	} else {
		*((float*) valueout) = 0;
	}

	return B_TRUE;
}

bool_t docking_longitudinal_r_cb(dr_t *dr, void *valueout)
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

	LatLonSpot spot = std::pair(worldLatitude, worldLongitude);

	dock_info_t *foundDock = dock_found(spot);

	if (foundDock) {
		*((float*) valueout) = foundDock->lonGuidance;
	} else {
		*((float*) valueout) = 0;
	}

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

	LatLonSpot spot = std::pair(worldLatitude, worldLongitude);

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

			jetways[spot] = newJetway;

			//logMsg("[DEBUG] Found a SAM xml specified jetway with name %s", newJetway.name.c_str());
		} else {		
			//logMsg("[WARN] Found an unknown new jetway at at lat %f, lon %f, alt %f, now have %d jetways", worldLatitude, worldLongitude, worldAltitude, jetways.size());
		}

	} else {
		
		// Lets use the precision of the sim... greater than the xml file!
		foundJetway->lat = worldLatitude;
		foundJetway->lon = worldLongitude;
		foundJetway->alt = worldAltitude;
		foundJetway->hdg = dr_getf(&draw_object_psi_dr);

		// if (foundJetway == nearest_jetway) {
		// 	random_animate_jetway(foundJetway);
		// } else {
		// 	foundJetway->rotate1 = foundJetway->initialRot1;
		// 	foundJetway->rotate2 = foundJetway->initialRot2;
		// 	foundJetway->rotate3 = foundJetway->initialRot3;
		// 	foundJetway->extent  = foundJetway->initialExtent;
		// 	foundJetway->wheels = (foundJetway->wheelPos + foundJetway->extent) * sin(DEG2RAD(foundJetway->rotate3));
		// }

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
	
	LatLonSpot spot = std::pair(worldLatitude, worldLongitude);

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
	
	LatLonSpot spot = std::pair(worldLatitude, worldLongitude);

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
	
	LatLonSpot spot = std::pair(worldLatitude, worldLongitude);

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
	
	LatLonSpot spot = std::pair(worldLatitude, worldLongitude);

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
	
	LatLonSpot spot = std::pair(worldLatitude, worldLongitude);

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
	
	LatLonSpot spot = std::pair(worldLatitude, worldLongitude);

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
	
	LatLonSpot spot = std::pair(worldLatitude, worldLongitude);

	jetway_info_t *foundJetway = jetway_found(spot);

	if (foundJetway) {
		// if (foundJetway == nearest_jetway) {
		// 	*((float*) valueout) = 1;
		// } else {
			*((float*) valueout) = 0;
		//}
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

		if (auto jetway_name = jetwayXMLelem->Attribute("name")) {
			newSAMxmlJetway.name = std::string(jetway_name);
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

		if (jetwayXMLelem->Attribute("forDoorLocation")) {
			auto doorLoc = std::string(jetwayXMLelem->Attribute("forDoorLocation"));
			if (doorLoc == "LF1") {
				newSAMxmlJetway.forDoorLocation = LF1Door; 
			} else if (doorLoc == "LF2") {
				newSAMxmlJetway.forDoorLocation = LF2Door; 
			} else {
				logMsg("[WARN] Found a door location of %s for jetway %s", doorLoc.c_str(), newSAMxmlJetway.name.c_str());
				newSAMxmlJetway.forDoorLocation = OTHERDoor;
			}
		} else {
			newSAMxmlJetway.forDoorLocation = ANYDoor;
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

	auto docksSec = doc.FirstChildElement("scenery")->FirstChildElement("docks");
	count = 0;
	for (tinyxml2::XMLElement* dockXMLelem = docksSec->FirstChildElement(); dockXMLelem != NULL; dockXMLelem = dockXMLelem->NextSiblingElement())
	{
		dock_info_t newSAMxmlDock;

		tinyxml2::XMLError errretval;

		if (auto dock_id = dockXMLelem->Attribute("id")) {
			newSAMxmlDock.name = std::string(dock_id);
		} else {
			logMsg("[ERROR] Couldn't find name!");
			continue;
		}
		
		errretval = dockXMLelem->QueryDoubleAttribute("latitude", &newSAMxmlDock.lat);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find latitude!");
			continue;
		}

		errretval = dockXMLelem->QueryDoubleAttribute("longitude", &newSAMxmlDock.lon);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find longitude!");
			continue;
		}

		errretval = dockXMLelem->QueryDoubleAttribute("elevation", &newSAMxmlDock.elev);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find elevation!");
			continue;
		}

		errretval = dockXMLelem->QueryFloatAttribute("heading", &newSAMxmlDock.hdg);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find heading!");
			continue;
		}

		errretval = dockXMLelem->QueryBoolAttribute("fixed", &newSAMxmlDock.fixed);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[WARN] Couldn't find fixed attribute for dock %s, will assume its false", newSAMxmlDock.name.c_str());
			newSAMxmlDock.fixed = false;
			//continue;
		}

		errretval = dockXMLelem->QueryDoubleAttribute("dockLatitude", &newSAMxmlDock.dockLat);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find dock latitude!");
			continue;
		}

		errretval = dockXMLelem->QueryDoubleAttribute("dockLongitude", &newSAMxmlDock.dockLon);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find dock longitude!");
			continue;
		}

		errretval = dockXMLelem->QueryFloatAttribute("dockHeading", &newSAMxmlDock.dockHdg);
		if (errretval != tinyxml2::XML_SUCCESS) {
			logMsg("[ERROR] Couldn't find dock heading!");
			continue;
		}
	
		newSAMxmlDock.markerInstanceRef = NULL;

		auto spot = std::make_pair(newSAMxmlDock.lat, newSAMxmlDock.lon);

		samXMLdocks[spot] = newSAMxmlDock;

	    count++;
	}
	logMsg("[DEBUG] Found %d docks for scenery %s", count, doc.FirstChildElement("scenery")->Attribute("name"));
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

// TEMPORARY...
static int dockingStatus3 = 0, dockingStatus = 0;
static float lateralGuidance = 0, longitudinalGuidance = 0;

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
        // logMsg("Setting jetway marker position for jetway %s ...", nearest_jetway->name.c_str());
        // XPLMDrawInfo_t drawInfo;
        // drawInfo.structSize = sizeof(XPLMDrawInfo_t);

		// double local_x_jetway, local_y_jetway, local_z_jetway;
        
        // XPLMWorldToLocal(nearest_jetway->lat,  
        //                       nearest_jetway->lon,    
        //                       nearest_jetway->alt + nearest_jetway->height + 2,    
        //                       &local_x_jetway,    
        //                       &local_y_jetway,    
        //                       &local_z_jetway); 

        // drawInfo.x = local_x_jetway;
        // drawInfo.y = local_y_jetway;
        // drawInfo.z = local_z_jetway;
        // drawInfo.pitch = 0;
        // drawInfo.roll = 0;
        // drawInfo.heading = nearest_jetway->hdg;
        // XPLMInstanceSetPosition(_markerInstanceRef, &drawInfo, nullptr);
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
                              aircraft.alt + 2,    
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

    setMarkerPosition(); 
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


	xp_ver = XPLMGetDatai(XPLMFindDataRef("sim/version/xplane_internal_version"));

	if (xp_ver >= 11000) {
	        fdr_find(&gear_on_ground_dr, "sim/flightmodel2/gear/on_ground");
	} else {
		logMsg("[ERROR] X-Plane version is too old! How ancient are you?");
		return 0;
	}
	fdr_find(&gear_deploy_dr, "sim/aircraft/parts/acf_gear_deploy");
	fdr_find(&gear_types_dr, "sim/aircraft/parts/acf_gear_type");
	fdr_find(&gear_steers_dr, "sim/aircraft/overflow/acf_gear_steers");
	fdr_find(&tire_z_dr, "sim/flightmodel/parts/tire_z_no_deflection");

	fdr_find(&beacon_on_dr, "sim/cockpit2/switches/beacon_on");
	fdr_find(&fnrml_gear_dr, "sim/flightmodel/forces/fnrml_gear");
	fdr_find(&groundspeed_dr, "sim/flightmodel2/position/groundspeed");
	fdr_find(&engine_burning_fuel_dr, "sim/flightmodel2/engines/engine_is_burning_fuel");


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

	dr_create_i(&sam3_docking_status_dr, NULL, B_FALSE, "sam3/docking/status");
	sam3_docking_status_dr.write_scalar_cb = docking_w_cb;
	sam3_docking_status_dr.read_scalar_cb = docking_status_r_cb;
	
	dr_create_i(&sam3_vdgs_status_dr, NULL, B_FALSE, "sam3/vdgs/status");
	sam3_vdgs_status_dr.write_scalar_cb = docking_w_cb;
	sam3_vdgs_status_dr.read_scalar_cb = docking_status_r_cb;

	dr_create_i(&sam_vdgs_status_dr, NULL, B_FALSE, "sam/vdgs/status");
	sam_vdgs_status_dr.write_scalar_cb = docking_w_cb;
	sam_vdgs_status_dr.read_scalar_cb = docking_status_r_cb;

	dr_create_f(&sam3_docking_longitudinal_dr, NULL, B_FALSE, "sam3/docking/longitudinal");
	sam3_docking_longitudinal_dr.write_scalar_cb = docking_w_cb;
	sam3_docking_longitudinal_dr.read_scalar_cb = docking_longitudinal_r_cb;

	dr_create_f(&sam3_docking_lateral_dr, NULL, B_FALSE, "sam3/docking/lateral");
	sam3_docking_lateral_dr.write_scalar_cb = docking_w_cb;
	sam3_docking_lateral_dr.read_scalar_cb = docking_lateral_r_cb;

	dr_create_f(&sam_docking_longitudinal_dr, NULL, B_FALSE, "sam/docking/longitudinal");
	sam_docking_longitudinal_dr.write_scalar_cb = docking_w_cb;
	sam_docking_longitudinal_dr.read_scalar_cb = docking_longitudinal_r_cb;

	dr_create_f(&sam_docking_lateral_dr, NULL, B_FALSE, "sam/docking/lateral");
	sam_docking_lateral_dr.write_scalar_cb = docking_w_cb;
	sam_docking_lateral_dr.read_scalar_cb = docking_lateral_r_cb;


	// dr_create_i(&sam3_docking_status_dr, &dockingStatus3, B_TRUE, "sam3/docking/status");
	// dr_create_i(&sam3_vdgs_status_dr, &dockingStatus3, B_TRUE, "sam3/vdgs/status");
	// dr_create_i(&sam_vdgs_status_dr, &dockingStatus, B_TRUE, "sam/vdgs/status");
	// dr_create_f(&sam3_docking_longitudinal_dr, &longitudinalGuidance, B_TRUE, "sam3/docking/longitudinal");
	// dr_create_f(&sam3_docking_lateral_dr, &lateralGuidance, B_TRUE, "sam3/docking/lateral");
	// dr_create_f(&sam_docking_longitudinal_dr, &longitudinalGuidance, B_TRUE, "sam/docking/longitudinal");
	// dr_create_f(&sam_docking_lateral_dr, &lateralGuidance, B_TRUE, "sam/docking/lateral");
	


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

bool readAircraftDoorProperties()
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
			bool has_board_1 = (std::stoi(acf_prop) == 1);
			if (has_board_1) {
				if (acf_prop = acf_find_prop(acf, "acf/_board_1/0")) {
					
					door_info_t door_1;

					door_1.lateralShift = METERS_PER_FOOT * std::stof(acf_prop);
					door_1.verticalShift = METERS_PER_FOOT * std::stof(acf_find_prop(acf, "acf/_board_1/1")) - cgY_m;
					door_1.axialShift = METERS_PER_FOOT * std::stof(acf_find_prop(acf, "acf/_board_1/2")) - cgZ_m;
					door_1.hdgOffset = 0;
					door_1.type = LF1Door;
					door_1.aircraft = &aircraft;
					door_1.jetway = nullptr;
					door_1.connection = DOORDISCONNECTED;

					aircraft.doors[LF1Door] = door_1;
				}
			}
		}

		if (acf_prop = acf_find_prop(acf, "acf/_has_board_2")) {
			bool has_board_2 = (std::stoi(acf_prop) == 1);
			if (has_board_2) {
				if (acf_prop = acf_find_prop(acf, "acf/_board_2/0")) {
					
					door_info_t door_2;

					door_2.lateralShift = METERS_PER_FOOT * std::stof(acf_prop);
					door_2.verticalShift = METERS_PER_FOOT * std::stof(acf_find_prop(acf, "acf/_board_2/1")) - cgY_m;
					door_2.axialShift = METERS_PER_FOOT * std::stof(acf_find_prop(acf, "acf/_board_2/2")) - cgZ_m;
					door_2.hdgOffset = 0;
					door_2.type = LF2Door;
					door_2.aircraft = &aircraft;
					door_2.jetway = nullptr;
					door_2.connection = DOORDISCONNECTED;

					aircraft.doors[LF2Door] = door_2;

				}
			}
		}
	}

	acf_file_free(acf);

	return true;
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

void checkEnginesRunning()
{
	// sim/flightmodel2/engines/engine_is_burning_fuel
    int engines_burning[16];
    dr_getvi(&engine_burning_fuel_dr, engines_burning, 0, 16);
    aircraft.engines = false;
    for(auto i = 0; i < 16; i++) {
    	if (engines_burning[i] == 1) {
    		aircraft.engines = true;
    		break;
    	}
    }
}

void checkWeightOnWheels()
{
    // float gear_deflections[NUMBER_OF_GEAR];
	// XPLMGetDatavf(_tire_vertical_deflection_mtr_vf, &gear_deflections[0], 0, NUMBER_OF_GEAR);
	// for (unsigned int gear_indx = 0; gear_indx < NUMBER_OF_GEAR; gear_indx++) {
	// 	if (gear_deflections[gear_indx] > 0.0) {
	// 		aircraft.airborne = false;
    //     }
	// }
	// aircraft.airborne = true;
}

void updateAircraft()
{
	// Get the aircraft position....

	aircraft.local_x = dr_getf(&local_x_dr);
    aircraft.local_y = dr_getf(&local_y_dr);
    aircraft.local_z = dr_getf(&local_z_dr);

    // Get the aircraft true heading...
    aircraft.hdg = dr_getf(&true_psi_dr);

    aircraft.beacon = (dr_geti(&beacon_on_dr) == 1);
    checkEnginesRunning();
    //checkWeightOnWheels();
    aircraft.airborne = !(dr_getf(&fnrml_gear_dr) > 0);
    aircraft.velocity = dr_getf(&groundspeed_dr); // meters per second

    XPLMLocalToWorld(aircraft.local_x,    
                     aircraft.local_y,    
                     aircraft.local_z,    
                     &aircraft.lat,  
                     &aircraft.lon,    
                     &aircraft.alt);

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
    
    aircraft.q_local = quat_rot_concat(ecmigl_2_local_quat, acf_q_earth_fixed);

    // Calculate nosewheel spot...
    {
    	struct quat body_frame_shift_vector = quat_from_vect3l_gl(VECT3L(0, 0, aircraft.nosewheelZ));

	    struct quat recovered_local_difference_vector = squat_multiply(aircraft.q_local, squat_multiply(body_frame_shift_vector, squat_inverse(aircraft.q_local)));
	    vect3l_t vxyz_recovered_vect3 = quat_to_vect3l_gl(recovered_local_difference_vector);
	        
	    double local_x_value_shift = vxyz_recovered_vect3.x;
	    double local_y_value_shift = vxyz_recovered_vect3.y;
	    double local_z_value_shift = vxyz_recovered_vect3.z;

	    double nosewheelAlt;

	    XPLMLocalToWorld(aircraft.local_x + local_x_value_shift,    
                         aircraft.local_y + local_y_value_shift,    
                         aircraft.local_z + local_z_value_shift,    
                         &aircraft.nosewheelSpot.first,  
                         &aircraft.nosewheelSpot.second,    
                         &nosewheelAlt);
    }

}

door_position_t door_position(door_info_t *door) {

	if (!door->aircraft) {
		logMsg("[ERROR] Door found unattached to aircraft ... Blame Boeing? Spirit?");
		assert(false);
	}

	door_position_t door_position;

	struct quat body_frame_shift_vector = quat_from_vect3l_gl(VECT3L(door->lateralShift, door->verticalShift, door->axialShift));

    struct quat recovered_local_difference_vector = squat_multiply(door->aircraft->q_local, squat_multiply(body_frame_shift_vector, squat_inverse(door->aircraft->q_local)));
    vect3l_t vxyz_recovered_shift_vect3 = quat_to_vect3l_gl(recovered_local_difference_vector);
    
    door_position.shift_x = vxyz_recovered_shift_vect3.x;
    door_position.shift_y = vxyz_recovered_shift_vect3.y;
    door_position.shift_z = vxyz_recovered_shift_vect3.z;

    XPLMLocalToWorld(door->aircraft->local_x + door_position.shift_x,    
                     door->aircraft->local_y + door_position.shift_y,    
                     door->aircraft->local_z + door_position.shift_z,    
                      &door_position.lat,  
                      &door_position.lon,    
                      &door_position.alt);

    door_position.hdg = door->aircraft->hdg + door->hdgOffset;

    logMsg("[DEBUG] Got here...");

    return door_position;
}

LatLonSpot door_spot(door_info_t *door) {
	auto position = door_position(door);
	return std::make_pair(position.lat, position.lon);
}

std::string door_type_name(DoorType type) {
	
	std::string name = "<UNKNOWN>";

	switch(type) {
		case ANYDoor:
			name = "ANY Door";
			break;
		case LF1Door:
			name = "LF1 Door";
			break;
		case LF2Door:
			name = "LF2 Door";
			break;
		case LR1Door:
			name = "LR1 Door";
			break;
		case OTHERDoor:
			name = "OTHER Door";
			break;
		case DISCONNECTED_TAXI_OUT:
			logMsg("[INFO] New ops state is DISCONNECTED_TAXI_OUT");
			break;
	}
	return name;
}

jetway_position_t jetway_position(jetway_info_t *jetway)
{
	jetway_position_t jetway_pos;

	XPLMWorldToLocal(jetway->lat,  
                     jetway->lon,    
                     jetway->alt,    
                      &jetway_pos.local_x,    
                      &jetway_pos.local_y,    
                      &jetway_pos.local_z); 

	XPLMProbeResult probe_result = XPLMProbeTerrainXYZ(_terrainProbe,
	          (float) jetway_pos.local_x,
	          (float) jetway_pos.local_y,
	          (float) jetway_pos.local_z,
	          &_terrainProbeInfo);

    double rotundaBaseLatitude, rotundaBaseLongitude, rotundaBaseMSL;
    XPLMLocalToWorld(_terrainProbeInfo.locationX,    
                     _terrainProbeInfo.locationY,    
                     _terrainProbeInfo.locationZ,    
                     &rotundaBaseLatitude,  
                     &rotundaBaseLongitude,    
                     &rotundaBaseMSL);

    jetway_pos.alt = rotundaBaseMSL;

    logMsg("[DEBUG] Difference between jetway->alt of %f and rotundaBaseMSL %f", jetway->alt, rotundaBaseMSL);

    return jetway_pos;
}	

std::pair<bool, jetway_targets_t> jetway_can_reach_door(jetway_info_t *jetway, door_info_t *door)
{
	//Check if door can reach airplane door....

	jetway_targets_t target;

	if (jetway->forDoorLocation != ANYDoor && jetway->forDoorLocation != door->type) {
		return std::pair(false, target);
	}

	logMsg("[DEBUG] Checking if jetway %s can reach door %s", jetway->name.c_str(), door_type_name(door->type).c_str());

	door_position_t door_pos = door_position(door);

	jetway_position_t jetway_pos = jetway_position(jetway);

	// Need to calculate the lat and lon of where the cabin should be...

	// If door .hdgOffset = 0 then we apply a negative (for left door) lateral offset of the cabinLength...
	// in the aircraft coordinates.  If .hdgOffset > 0 then we do cos() that for lateral and -sin() for axial...

	struct quat cabin_body_frame_shift_vector 
				= quat_from_vect3l_gl(VECT3L(-jetway->cabinLength*cos(DEG2RAD(door->hdgOffset)), 
											 0, -jetway->cabinLength*sin(DEG2RAD(door->hdgOffset))));

    struct quat cabin_recovered_local_difference_vector = squat_multiply(aircraft.q_local, squat_multiply(cabin_body_frame_shift_vector, squat_inverse(aircraft.q_local)));
    vect3l_t cabin_vxyz_recovered_vect3 = quat_to_vect3l_gl(cabin_recovered_local_difference_vector);
    
    double cabin_lat, cabin_lon, cabin_alt;

    XPLMLocalToWorld(aircraft.local_x + door_pos.shift_x + cabin_vxyz_recovered_vect3.x,    
                     aircraft.local_y + door_pos.shift_x + cabin_vxyz_recovered_vect3.y,    
                     aircraft.local_z + door_pos.shift_x + cabin_vxyz_recovered_vect3.z,    
                          &cabin_lat,  
                          &cabin_lon,    
                          &cabin_alt);

	geo_pos2_t rotunda_geo_pos2 = GEO_POS2(jetway->lat, jetway->lon);
	geo_pos2_t cabin_geo_pos2 = GEO_POS2(cabin_lat, cabin_lon);	                          

    double bearing_from_rotunda_to_cabin = gc_point_hdg(rotunda_geo_pos2, cabin_geo_pos2);

    target.rot1 = shift_normalized_hdg(normalize_hdg(bearing_from_rotunda_to_cabin - jetway->hdg));

    logMsg("[DEBUG] Found target rot1 of %f vs (min %f and max %f)", target.rot1, jetway->minRot1, jetway->maxRot1);

    if (target.rot1 > jetway->maxRot1 || target.rot1 < jetway->minRot1) {
    	return std::pair(false, target);
    }

	target.rot2 = shift_normalized_hdg(normalize_hdg(90 - (jetway->hdg + target.rot1) + aircraft.hdg + door->hdgOffset));
	
    logMsg("[DEBUG] Found target rot2 of %f vs (min %f and max %f)", target.rot2, jetway->minRot2, jetway->maxRot2);

	if (target.rot2 > jetway->maxRot2 || target.rot2 < jetway->minRot2) {
		return std::pair(false, target);
	}

	double heightRise = cabin_alt - jetway->height - jetway_pos.alt;

	logMsg("[DEBUG] Found heightRise of %f", heightRise);

	double distance_from_rotunda_to_cabin = gc_distance(rotunda_geo_pos2, cabin_geo_pos2); 
	
	// Use this for sorting jetway connections...
	// CONCERN: Should it be distance to door? 
	target.dist = distance_from_rotunda_to_cabin;

	target.extent = std::sqrt(heightRise*heightRise + 
		                            distance_from_rotunda_to_cabin*distance_from_rotunda_to_cabin)
									    - jetway->cabinPos;

	logMsg("[DEBUG] Found target extent of %f vs (min %f and max %f)", target.extent, jetway->minExtent, jetway->maxExtent);

	if (target.extent > jetway->maxExtent || target.extent < jetway->minExtent) {
		return std::pair(false, target);
	}

	if (target.extent + jetway->cabinPos != 0) {
		target.rot3 = RAD2DEG(std::asin(heightRise/(target.extent + jetway->cabinPos)));
	} else {
		logMsg("[ERROR] Can't calculate rotate3 target because extent target + cabin position sums to zero, would have divide by zero!");
		return std::pair(false, target);
	}

	logMsg("[DEBUG] Found target rot3 of %f vs (min %f and max %f)", target.rot3, jetway->minRot3, jetway->maxRot3);

	// if (target.rot3 > jetway->maxRot3 || target.rot3 < jetway->minRot3) {
	// 	return std::pair(false, target);
	// }

	return std::pair(true, target);;
}

static std::deque<door_info_t*> connectionPlan;
static std::deque<door_info_t*> disconnectionPlan;

bool jetwayInConnectionPlan(jetway_info_t *jetway) {
	for (auto it = connectionPlan.begin(); it != connectionPlan.end(); it++) {
		if ((*it)->jetway == jetway) {
			return true;
		}
	}

	return false;
}

void connectDoor(door_info_t *door) {
		
	// Get nearby jetways... within 200m of aircraft...
	float range = 200.0f;
	
	std::vector<jetway_info_t*> jetways = nearby_jetways(door_spot(door), range);
	
	jetway_info_t *foundJetway = nullptr;
	jetway_targets_t foundTargets;
	float foundDistance = range;

	logMsg("[DEBUG] Found %d jetways within range of %f meters", jetways.size(), range);

	for(auto jetway : jetways) {
		
		// NOTE: Only look at jetways don't already plan to connect...
		if (!jetwayInConnectionPlan(jetway)) {
			auto [canReach, computedTargets] = jetway_can_reach_door(jetway, door);

			if (canReach) {
				logMsg("[DEBUG] Found jetway %s can reach this door", jetway->name.c_str());
			}

			if (canReach && computedTargets.dist < range) {
				foundJetway = jetway;
				foundTargets = computedTargets;
			}
		}			
	}

	if (foundJetway) {
		foundJetway->targets = foundTargets;
		door->connection = DOORWILLCONNECT;
		door->jetway = foundJetway;

		connectionPlan.push_back(door);
	}

}

void computeConnectionPlan()
{

	// Figure out which jetbridges will connect to which doors...

	// Start with LF1 door...
	if (aircraft.doors.count(LF1Door)) {
		connectDoor(&aircraft.doors[LF1Door]);
	}

	if (aircraft.doors.count(LF2Door)) {
		connectDoor(&aircraft.doors[LF2Door]);
	}

	logMsg("[DEBUG] Found %d jetways to connect...", connectionPlan.size());
	//for (auto& [doorType, door] : aircraft.doors) {
	for (auto door : connectionPlan) {
		logMsg("[DEBUG] Will connect door %s to jetway %s", door_type_name(door->type).c_str(), door->jetway->name.c_str());
		
		if (currentState == CONNECTED) {
			logMsg("[INFO] We have loaded in at a gate, perform immediate jetway connections...");
			door->connection = DOORCONNECTED;
			door->jetway->rotate1 = door->jetway->targets.rot1;
			door->jetway->rotate2 = door->jetway->targets.rot2;
			door->jetway->rotate3 = door->jetway->targets.rot3;
			door->jetway->extent = door->jetway->targets.extent;
		
			disconnectionPlan.push_front(door);
		}
	}

	if (currentState == CONNECTED) {
		connectionPlan.clear();
	}

	if (!connectionPlan.empty()) {
		auto door = connectionPlan.front();
		logMsg("[DEBUG] Begin moving jetway %s to door %s", 
			door->jetway->name.c_str(), door_type_name(door->type).c_str());
	}
}

void computeDisconnectPlan()
{
	if(!connectionPlan.empty()) {

		// Assume the first jetway is the one in progress of being moved...
		// So we will push it to the disconnectionPlan to move first
		// push back since we will draw from the back...

		disconnectionPlan.push_back(connectionPlan.front());
		connectionPlan.pop_front();

		// For the rest we will assume they haven't moved jetways yet
		// and disassociate them...

		while(!connectionPlan.empty()) {
			auto door = connectionPlan.front();
			door->jetway = nullptr;
			door->connection = DOORDISCONNECTED;
			connectionPlan.pop_front();
		}
	}

	assert(connectionPlan.empty());
}

void setOpsState(OpsState newOpsState)
{
	if (newOpsState == currentState) {
		return;
	}

	switch(newOpsState) {
		case NOSTATE:
			// Aircraft has loaded in, lets determine our state...
			logMsg("[INFO] New ops state is NOSTATE");
			break;
		case DISCONNECTED_TAXI_IN:
			logMsg("[INFO] New ops state is DISCONNECTED_TAXI_IN");
			break;
		case CONNECTING:
			logMsg("[INFO] New ops state is CONNECTING");
			computeConnectionPlan();
			break;
		case CONNECTED:
			logMsg("[INFO] New ops state is CONNECTED");
			break;
		case DISCONNECTING:
			logMsg("[INFO] New ops state is DISCONNECTING");
			computeDisconnectPlan();
			break;
		case DISCONNECTED_TAXI_OUT:
			logMsg("[INFO] New ops state is DISCONNECTED_TAXI_OUT");
			break;
	}

	currentState = newOpsState;
}

void determineLoadinOpsState()
{
	/* New aircraft loaded, lets determine our state...
			-> Parked at gate (beacon off / engines off) -> CONNECTED
 
 			-> On runway, etc. (beacon on or engines running) -> DISCONNECTED_TAXI_IN
	*/

	if (!aircraft.engines && !aircraft.beacon) {
		setOpsState(CONNECTED);
		computeConnectionPlan();
	} else {
		setOpsState(DISCONNECTED_TAXI_IN);
	}
}

void verifyTaxiIn()
{
	// CONCERN: Should we consider engines running as well?

	if (!aircraft.beacon && aircraft.velocity < 0.1 * KTS2MPS) {
		setOpsState(CONNECTING);
	}
}

bool pushbackActive()
{
	return false;
}

void verifyTaxiOut()
{
	/*
	*	During taxi-out
	* 		we don't display marshallers. We are in taxi-out if beacon is on and any of the following
	*       are true:
	* 					1. No engines running
	* 					2. Better pushback (or X-plane pushback) active
	* 					3. Aircraft has not exceeded 10 knots
	*/

	// ONLY STATE TRANSITION POSSIBLE IS TO TAXI IN... (even if airborne we will be taxi-in)

	if (aircraft.beacon) {
		if (!aircraft.engines || pushbackActive() || aircraft.velocity < 10 * KTS2MPS) {
			return;
		} 
	}

	// CONCERN: Maybe once we have traveled some distance from the gate we would renable
	//          taxi-in as well?

	setOpsState(DISCONNECTED_TAXI_IN);

}

bool connectingFinished() {
	for (auto& [doorType, door] : aircraft.doors) {
		if (door.connection == DOORJETWAYMOVING) {
			return false;
		}
	}
	return true;
}

bool disconnectingFinished() {
	for (auto& [doorType, door] : aircraft.doors) {
		if (door.connection != DOORDISCONNECTED) {
			logMsg("[DEBUG] Found door that is not disconnected...");
			return false;
		}
	}
	return true;
}

void verifyConnecting()
{
	if (aircraft.beacon || aircraft.engines || aircraft.velocity > 0.1 * KTS2MPS) {
		setOpsState(DISCONNECTING);
	}

	if (connectingFinished()) {
		setOpsState(CONNECTED);
	}
}

void verifyConnected()
{
	if (aircraft.beacon || aircraft.engines || aircraft.velocity > 0.1 * KTS2MPS) {
		setOpsState(DISCONNECTING);
	}
}

void verifyDisconnecting()
{
	if (disconnectingFinished()) {
		setOpsState(DISCONNECTED_TAXI_OUT);
	}	
}

/****************************************************************************
 * 
 *   State logic.  
 * 
 *        New aircraft loaded -> Parked at gate (beacon off / engines off)
 * 
 * 							  -> On runway, etc. (beacon on or engines running)
 * 
 *        Disconnected -> Connecting -> Connected -> Disconnecting -> Disconnected
 * 
 * 
 *		When disconnected (taxi-in) we monitor aircraft position to look for a marshaller
 *      to display to guide pilot in.
 * 
 * 		Diconnected split into two states... Taxi-out and Taxi-in.  During taxi-out
 * 		we don't display marshallers. We are in taxi-out if any of the following
 *      are true:
 * 					1. No engines running
 * 					2. Better pushback (or X-plane pushback) active
 * 					3. Aircraft has not exceeded 10 knots
 * 
 * 
 * 		We begin connecting either via a command or when aircraft is not moving 
 *      and beacon light is off.
 *		Connecting we animate the jetway connection. Marshaller disappears once
 *      we begin connecting.
 * 
 * 			During connecting we first determine what jetways will beconnected
 * 			to the aircraft.  We look at:
 * 
 * 				1. What jetways can reach which doors.  Respect the "forDoorLocation"
 * 				   value and doors defined in samiam.xml file as well as door operations
 * 				   specified in the file.
 * 
 * 				2. Animate the doors accordingly.
 *
 *      Connected.  We monitor aircraft movement and periodically adjust the
 * 	    door if aircraft settles/rises during loading/unloading beyond some
 *      threshold amount.
 * 
 *      Disconnecting.  We begin disconnecting either via command or when 
 *		beacon light is turned on AND doors closed (if we know the door datarefs) 
 * 		 
 * 
 ****************************************************************************/

void updateOpsState()
{
	switch(currentState) {
		case NOSTATE:
			// Aircraft has loaded in, lets determine our state...
			determineLoadinOpsState();
			break;
		case DISCONNECTED_TAXI_IN:
			verifyTaxiIn();
			break;
		case CONNECTING:
			verifyConnecting();
			break;
		case CONNECTED:
			verifyConnected();
			break;
		case DISCONNECTING:
			verifyDisconnecting();
			break;
		case DISCONNECTED_TAXI_OUT:
			verifyTaxiOut();
			break;
	}
}

void handleJetwayAutoleveling()
{
	for (auto& [doorType, door] : aircraft.doors) {
		if (door.connection == DOORCONNECTED) {
			if (door.jetway) {
				logMsg("[DEBUG] We need to worry about autoleveling jetway %s to this door", door.jetway->name.c_str());
			} else {
				logMsg("[ERROR] We have a connected door but no jetway!");
				door.connection = DOORDISCONNECTED;
			}
		}
	}
}

void handleJetwayConnectionAnimation()
{
	logMsg("[DEBUG] Have %d doors to connect...", connectionPlan.size());

	if (!connectionPlan.empty()) {

		auto door = connectionPlan.front();

		assert(door->connection == DOORJETWAYMOVING);
		
		auto still_moving = random_animate_jetway(door->jetway);

		if (!still_moving) {
			door->connection = DOORCONNECTED;
			logMsg("[DEBUG] Finised moving jetway %s to door %s", 
				door->jetway->name.c_str(), door_type_name(door->type).c_str());
			disconnectionPlan.push_front(connectionPlan.front());
			connectionPlan.pop_front();
		
			if (!connectionPlan.empty()) {
				door = connectionPlan.front();
				logMsg("[DEBUG] Begin moving jetway %s to door %s", 
					door->jetway->name.c_str(), door_type_name(door->type).c_str());
			}
		
		}

	}
	//  else {
	// 	setOpsState(CONNECTED);
	// }
}

void setJetwayTargetsToInitialPosition(jetway_info_t *jetway)
{
	jetway->targets.rot1   = jetway->initialRot1;
	jetway->targets.rot2   = jetway->initialRot2;
	jetway->targets.rot3   = jetway->initialRot3;
	jetway->targets.extent = jetway->initialExtent;
}

void handleJetwayDisconnectAnimation()
{

	logMsg("[DEBUG] Have %d doors to disconnect...", disconnectionPlan.size());

	if (!disconnectionPlan.empty()) {

		auto door = disconnectionPlan.back();

		assert(door->connection == DOORJETWAYMOVING || door->connection == DOORCONNECTED);
		
		setJetwayTargetsToInitialPosition(door->jetway);

		auto still_moving = random_animate_jetway(door->jetway);

		if (!still_moving) {
			door->connection = DOORDISCONNECTED;
			logMsg("[DEBUG] Finised removing jetway %s from door %s", 
				door->jetway->name.c_str(), door_type_name(door->type).c_str());
			disconnectionPlan.pop_back();
		
			if (!disconnectionPlan.empty()) {
				door = disconnectionPlan.back();
				logMsg("[DEBUG] Begin removing jetway %s from door %s", 
					door->jetway->name.c_str(), door_type_name(door->type).c_str());
			}
		
		}
	}
	// } else {
	// 	setOpsState(DISCONNECTED_TAXI_OUT);
	// }
}

float flightLoopCallback(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	
	if (need_acf_properties) {
		if (readAircraftDoorProperties() && 
			readAircraftGearInfo()) {
		
			currentState = NOSTATE; // Aircraft has loaded in, lets determine our state... 
		
			need_acf_properties = false;
		} else {
			return 1.0f;
		}
	}

	updateAircraft();

	updateOpsState();

	if (currentState == CONNECTED) {
		handleJetwayAutoleveling();
	} else if (currentState == CONNECTING) {
		handleJetwayConnectionAnimation();
	} else if (currentState == DISCONNECTING) {
		handleJetwayDisconnectAnimation();
	}
    
	if (_markerObjectRef) {
		//setManualMarkerPosition();

		//setSAMmarkerPositions();
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
	XPLMDebugString("SamIAm plugin stopping...\n");

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
    			need_acf_properties = true;
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

NOTE THEY ARE NOT WRITABLE.  THEY ARE USED AS INSTANCED DATAREFS, SO NO WAY 
FOR US TO TAKE THEM OVER.  BUMMER.

*/