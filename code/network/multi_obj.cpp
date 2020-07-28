/*
 * Copyright (C) Volition, Inc. 1999.  All rights reserved.
 *
 * All source code herein is the property of Volition, Inc. You may not sell 
 * or otherwise commercially exploit the source or things you created based on the 
 * source.
 *
*/


#include <algorithm>

#include "network/multi_obj.h"
#include "globalincs/globals.h"
#include "freespace.h"
#include "io/timer.h"
#include "io/key.h"
#include "globalincs/linklist.h"
#include "network/multimsgs.h"
#include "network/multiutil.h"
#include "network/multi_options.h"
#include "network/multi_rate.h"
#include "network/multi.h"
#include "object/object.h"
#include "object/objectshield.h"
#include "ship/ship.h"
#include "playerman/player.h"
#include "math/spline.h"
#include "physics/physics.h"
#include "ship/afterburner.h"
#include "cfile/cfile.h"
#include "debugconsole/console.h"


// ---------------------------------------------------------------------------------------------------
// OBJECT UPDATE STRUCTS
// 

extern const std::uint32_t MAX_TIME;

// One frame record per ship with each contained array holding one element for each frame.
struct oo_ship_position_records {
	vec3d positions[MAX_FRAMES_RECORDED];							// The recorded ship positions, cur_frame_index is the index.
	matrix orientations[MAX_FRAMES_RECORDED];						// The recorded ship orientations, cur_frame_index is the index. 
	vec3d velocities[MAX_FRAMES_RECORDED];							// The recorded ship velocities (required for additive velocity shots and auto aim), cur_frame_index is the index.
};

// keeps track of what has been sent to each player, helps cut down on bandwidth, allowing only new information to be sent instead of old.
struct oo_info_sent_to_players {	
	int timestamp;					// The overall timestamp which is used to decide if a new packet should be sent to this player for this ship.

	vec3d position;					// If they are stationary, there's no need to update their position.
	float hull;						// no need to send hull if hull hasn't changed.

	// shields are a little special because they are constantly regenerating, so if they are *not* at full strength we need to send them.
	bool perfect_shields_sent;		

	int ai_mode;					// what ai mode was last sent.
	int ai_submode;					// what ai submode was last sent.
	int target_signature;			// what target_signature was last sent (used for AI portion of OO packet)

	SCP_vector<float> subsystems;	// We need a vector to keep track of all subsystems.
};

struct oo_netplayer_records{
	SCP_vector<oo_info_sent_to_players> last_sent;			// Subcategory of which player did I send this info to?  Corresponds to net_player index.
	// This is not yet implemented, but may be necessary for autoaim to work in more busy scenes.  Basically, if you're switching targets,
	// autoaim may succeed on the client but head to the wrong target on the server.
//	int player_target_record[MAX_FRAMES_RECORDED];			// For rollback, we need to keep track of the player's targets. Uses frame as its index.
};

// Tracking Received info and timing per ship
struct oo_packet_and_interp_tracking {
	int cur_pack_pos_frame;				// the last position packet arrival frame
	int prev_pack_pos_frame;			// the prev position packet arrival frame

	bool client_simulation_mode;		// if the packets come in too late, a toggle to sim things like normal

	bool prev_packet_positionless;		// a flag that marks if the last packet as having no new position or orientation info.

	float pos_time_delta;				// How much time passed between position packets, in the same format as flFrametime
	int pos_timestamp;					// Time that FSO processes the most recent position packet
	vec3d old_packet_position;			// The last packet's pos
	vec3d new_packet_position;			// The current packet's pos
	vec3d position_error;				// Position error that is removed over a few frames

	angles old_angles;					// The last packet's orientation (in angles)
	angles new_angles;					// The current packet's orientation (in angles)
	angles anticipated_angles_a;		// The first angles we get from calculating the spline
	angles anticipated_angles_b;		// The second angles we get from calculating the spline
	angles anticipated_angles_c;		// The third angles we get from calculating the spline
	matrix new_orientation;				// The "uncompressed" orientation calculated from the packet's new angles

	vec3d new_velocity;					// The velocity we calculate from the packet
	vec3d anticipated_velocity1;		// The first velocity we get from calculating the spline
	vec3d anticipated_velocity2;		// The second velocity we get from calculating the spline
	vec3d anticipated_velocity3;		// The third velocity we get from calculating the spline

	bez_spline pos_spline;				// The points we calculated which the bezier interpolation is based on.

	// bashing the last received desired velocity and desired rotational velocity allows us to keep 
	// anything unexpected messing with where this ship should be.
	vec3d cur_pack_des_vel;				// 
	vec3d cur_pack_local_des_vel;		// desired velocity is in global coordinates normally, but this optimizes some.
	vec3d cur_pack_des_rot_vel;

	// Frame numbers that helps us figure out if we should ignore new information coming from the server because
	// we already received a newer packet than this one for all the info, or the most specific information.
	bool odd_wrap;						// Helps keep track of which larger wrap this ship has been updated with, odd or even. (wraps every ~65300 frames) 
	int most_recent_packet;				// what is the seq number of the most recent packet, not worried about type of packet.
	int pos_comparison_frame;			// what frame was the last position information received? 
	int prev_pos_comparison_frame;		// what frame was the second to last position information received?
	int hull_comparison_frame;			// what frame was the last hull information received?
	int shields_comparison_frame;		// what frame was the last shield information received?
	SCP_vector<int> subsystems_comparison_frame; // what frame was the last subsystem information received? (for each subsystem)
	int ai_comparison_frame;			// what frame was the last ai information received?
};

// Keep track of the ships we'll need to restore later after rollback is done.
struct oo_rollback_restore_record {
	object* roll_objp;		// object pointer to the ship we need to restore. 
	vec3d position;			// position to restore to
	matrix orientation;		// orientation to restore to
	vec3d velocity;			// velocity to restore to
};

// Keeps track of how many shots that need to be fired in rollback mode.
struct oo_unsimulated_shots {
	object* shooterp;		// pointer to the shooting object (ship)
	vec3d pos;				// the relative position calculated from the non-homing packet.
	matrix orient;			// the relative orientation from the non-homing packet.
	bool secondary_shot;	// is this a dumbfire missile shot?
};

// our main struct for keeping track of all interpolation and oo packet info.
struct oo_general_info {
	// info that helps us figure out what is the best reference object available when sending a rollback shot.
	// We go by what is the most recent object update packet received, and then by distance.
	int ref_timestamp;							// what time did we receive the reference object
	ushort most_recent_updated_net_signature;	// what is the net signature of the reference object.
	ushort most_recent_frame;					// what is the frame from the update of most recently updated object.

	// The previously received frametimes.  One entry for *every* frame, received or not, up to the last received frame.
	// For frames it does not receive, it assumes that the frame time is the same as the frames around it.
	SCP_vector<ubyte> received_frametimes;

	// Frame tracking info, we can have up to INT_MAX frames, but to save on bandwidth and memory we have to "wrap"
	// the index.  Cur_frame_index goes up to MAX_FRAMES_RECORDED and makes info easy to access, wrap_count counts 
	// how many times that is reset, up to MAX_SERVER_TRACKER_SMALL_WRAPS, we multiply these two to get the seq_num
	// sent to oo_packet recipients. larger_wrap_count allows us to figure out if we are in an "odd" larger wrap
	int number_of_frames;									// how many frames have we gone through, total.
	short larger_wrap_count;								// how many of the larger wrap, used to set an OO_packet flag.
	ushort wrap_count;										// how many times have we wrapped the cur_frame_index.
	ubyte cur_frame_index;									// the current frame index (to access the recorded info)

	int timestamps[MAX_FRAMES_RECORDED];					// The timestamp for the recorded frame
	SCP_vector<oo_ship_position_records> frame_info;		// Actually keeps track of ship physics info.  Uses net_signature as its index.
	SCP_vector<oo_netplayer_records> player_frame_info;		// keeps track of player targets and what has been sent to each player. Uses player as the index


	SCP_vector<oo_packet_and_interp_tracking> interp;		// Tracking received info and interpolation timing per ship, uses net_signature as its index.

	// rollback info
	bool rollback_mode;										// are we currently creating and moving weapons from the client primary fire packets
	SCP_vector<object*> rollback_wobjp_created_this_frame;	// the weapons created this rollback frame.
	SCP_vector<object*> rollback_wobjp;						// a list of the weapons that were created, so that we can roll them into the current simulation

	SCP_vector<object*> rollback_ships;						// a list of ships that take part in roll back, no quick index, must be iterated through.
	SCP_vector<oo_rollback_restore_record> restore_points;	// where to move ships back to when done with rollback. no quick index, must be iterated through.
	SCP_vector<oo_unsimulated_shots> 
		rollback_shots_to_be_fired[MAX_FRAMES_RECORDED];				// the shots we will need to fire and simulate during rollback, organized into the frames they will be fired
	SCP_vector<int>rollback_collide_list;					// the list of ships and weapons that we need to pass to collision detection during rollback.
};

oo_general_info Oo_info;

// flags
bool Afterburn_hack = false;			// HACK!!!

// for multilock
#define OOC_INDEX_NULLPTR_SUBSYSEM			255			// If a lock has a nullptr subsystem, send this as the invalid index.
#define OOC_MAX_LOCKS							375			// Because of limited packet size, this is approximately the safe maximum of locks. 

// returns the last frame's index.
int multi_find_prev_frame_idx();

// quickly lookup how much time has passed since the given frame.
uint multi_ship_record_get_time_elapsed(int original_frame, int new_frame);


// See if a newly arrived packet is a good new option as a reference object
void multi_ship_record_rank_seq_num(object* objp, ushort seq_num);

// Set the points for bezier interpolation
void multi_oo_calc_interp_splines(object* objp, matrix *new_orient, physics_info *new_phys_info);

// helper function that updates all interpolation info for a specific ship from a packet
void multi_oo_maybe_update_interp_info(object* objp, vec3d* new_pos, angles* new_ori_angles, matrix* new_ori_mat, physics_info* new_phys_info, bool adjust_pos, bool newest_pos);

// recalculate how much time is between position packets
float multi_oo_calc_pos_time_difference(int net_sig_idx);


// how much data we're willing to put into a given oo packet
#define OO_MAX_SIZE					480

// tolerance for bashing position
#define OO_POS_UPDATE_TOLERANCE	150.0f

// new improved - more compacted info type
#define OO_POS_AND_ORIENT_NEW		(1<<0)		// To update position and orientation. Because getting accurate velocity requires orientation, and accurate orienation requires velocity
#define OO_HULL_NEW					(1<<1)		// To Update Hull
#define OO_SHIELDS_NEW				(1<<2)		// To Update Shields.
#define OO_AFTERBURNER_NEW			(1<<3)		// Flag for Afterburner hack
#define OO_SUBSYSTEMS_NEW			(1<<4)		// Send Subsystem Info
#define OO_PRIMARY_BANK				(1<<5)		// if this is set, fighter has selected bank one
#define OO_PRIMARY_LINKED			(1<<6)		// if this is set, banks are linked
#define OO_TRIGGER_DOWN				(1<<7)		// if this is set, trigger is DOWN
#define OO_SUPPORT_SHIP				(1<<8)		// Send extra info for the support ship.
#define OO_AI_NEW					(1<<9)		// Send updated AI Info

#define OO_ODD_WRAP					(1<<12)		// Is the sent frame an odd wrap? Initially not wrapped (0), then odd wrap (1), etc.

#define OO_VIEW_CONE_DOT			(0.1f)
#define OO_VIEW_DIFF_TOL			(0.15f)			// if the dotproducts differ this far between frames, he's coming into view

// no timestamp should ever have sat for longer than this. 
#define OO_MAX_TIMESTAMP			2500

// distance class
#define OO_NEAR						0
#define OO_NEAR_DIST					(200.0f)
#define OO_MIDRANGE					1
#define OO_MIDRANGE_DIST			(600.0f)
#define OO_FAR							2
#define OO_FAR_DIST					(1400.0f)

// how often we should send full hull/shield updates
#define OO_HULL_SHIELD_TIME		600
#define OO_SUBSYS_TIME				1000

// for making the frame record wrapping predictable. 273 is the highest that we can put without a remainder. ~(65536/30) 
#define MAX_SERVER_TRACKER_SMALL_WRAPS 2184
#define SERVER_TRACKER_LARGE_WRAP_TOTAL (MAX_SERVER_TRACKER_SMALL_WRAPS * MAX_FRAMES_RECORDED)
#define HAS_WRAPPED_MINIMUM			(SERVER_TRACKER_LARGE_WRAP_TOTAL - (MAX_FRAMES_RECORDED * 2)) 

// timestamp values for object update times based on client's update level.
// Cyborg17 - This is the one update number I have adjusted, because it's the player's target.
int Multi_oo_target_update_times[MAX_OBJ_UPDATE_LEVELS] = 
{
	50, 				// 20x a second 
	50, 				// 20x a second
	20,				// 50x a second
	20,				// 50x a second
};

// for near ships
int Multi_oo_front_near_update_times[MAX_OBJ_UPDATE_LEVELS] =
{
	150,				// low update
	100,				// medium update
	66,				// high update
	66,				// LAN update
};

// for medium ships
int Multi_oo_front_medium_update_times[MAX_OBJ_UPDATE_LEVELS] =
{
	250,				// low update
	180, 				// medium update
	120,				// high update
	66,					// LAN update
};

// for far ships
int Multi_oo_front_far_update_times[MAX_OBJ_UPDATE_LEVELS] =
{
	750,				// low update
	350, 				// medium update
	150, 				// high update
	66,					// LAN update
};

// for near ships
int Multi_oo_rear_near_update_times[MAX_OBJ_UPDATE_LEVELS] = 
{
	300,				// low update
	200,				// medium update
	100,				// high update
	66,					// LAN update
};

// for medium ships
int Multi_oo_rear_medium_update_times[MAX_OBJ_UPDATE_LEVELS] = 
{
	800,				// low update
	600,				// medium update
	300,				// high update
	66,					// LAN update
};

// for far ships
int Multi_oo_rear_far_update_times[MAX_OBJ_UPDATE_LEVELS] = 
{
	2500, 				// low update
	1500,				// medium update
	400,				// high update
	66,					// LAN update
};

// ship index list for possibly sorting ships based upon distance, etc
short OO_ship_index[MAX_SHIPS];

// Cyborg17 - I'm leaving this system in place, just in case, although I never used it. 
// It needs cleanup in keycontrol.cpp before it can be used.
int OO_update_index = -1;							// The player index that allows us to look up multi rate through the debug
													//  console and display it on the hud.


// ---------------------------------------------------------------------------------------------------
// POSITION AND ORIENTATION RECORDING
// if it breaks, find Cyborg17

// We record positions and orientations in Oo_info.frame_info so that we can create a weapon in the same relative 
// circumstances as on the client.  I was directly in front, 600 meters away when I fired?  Well, now the client 
// will tell the server that and the server will rewind part of its simulation to recreate that shot and then 
// redo its simulation. There will still be some very small differences between server and client caused by the 
// fact that the flFrametimes will be different, but changing that would be impossible.
// ---------------------------------------------------------------------------------------------------

// Add a new ship to the tracking struct
void multi_ship_record_add_ship(int obj_num)
{
	object* objp = &Objects[obj_num];
	int net_sig_idx = objp->net_signature;

	// check for a ship that will enter the mission later on and has not yet had its net_signature set.
	// These ships will be added later when they are actually in the mission
	if (net_sig_idx == 0) {
		return;
	}

	// our target size is the number of ships in the vector plus one because net_signatures start at 1 and size gives the number of elements, and this should be a new element.
	int current_size = (int)Oo_info.frame_info.size();
	// if we're right where we should be.
	if (net_sig_idx == current_size) {
		Oo_info.frame_info.push_back(Oo_info.frame_info[0]);
		Oo_info.interp.push_back(Oo_info.interp[0]);
		for (int i = 0; i < MAX_PLAYERS; i++) {
			Oo_info.player_frame_info[i].last_sent.push_back( Oo_info.player_frame_info[i].last_sent[0] );
		}

	} // if not, create the storage for it.
	else if (net_sig_idx > current_size) {
		while (net_sig_idx >= current_size) {
			Oo_info.frame_info.push_back(Oo_info.frame_info[0]);
			Oo_info.interp.push_back(Oo_info.interp[0]);
			for (int i = 0; i < MAX_PLAYERS; i++) {
				Oo_info.player_frame_info[i].last_sent.push_back( Oo_info.player_frame_info[i].last_sent[0] );
			}
			current_size++;
		}
	} 

	Assertion(net_sig_idx <= (current_size + 1), "New entry into the multi ship traker struct does not equal the index that should belong to it.\nNet_signature: %d and current_size %d\n", net_sig_idx, current_size);

	ship_info* sip = &Ship_info[Ships[objp->instance].ship_info_index];

	// To use vectors for the subsystems, we have to init the subsystem them here.
	uint subsystem_count = (uint)sip->n_subsystems;

	Oo_info.interp[net_sig_idx].subsystems_comparison_frame.reserve(subsystem_count);

	while (Oo_info.interp[net_sig_idx].subsystems_comparison_frame.size() < subsystem_count) {
		Oo_info.interp[net_sig_idx].subsystems_comparison_frame.push_back(-1);

		for (int i = 0; i < MAX_PLAYERS; i++) {
			Oo_info.player_frame_info[i].last_sent[net_sig_idx].subsystems.push_back(-1.0f) ;
		}
	}

	// store info from the obj struct if it's already in the mission, otherwise, let the physics update call take care of it when the first frame starts.
	if (Game_mode & GM_IN_MISSION) {

		// only add positional info if they are in the mission.
		Oo_info.frame_info[net_sig_idx].positions[Oo_info.cur_frame_index] = objp->pos;
		Oo_info.frame_info[net_sig_idx].orientations[Oo_info.cur_frame_index] = objp->orient;
	}
}

// Update the tracking struct whenever the object is updated in-game
void multi_ship_record_update_all() 
{
	Assertion(MULTIPLAYER_MASTER, "Non-server accessed a server only function multi_ship_record_update_all(). Please report!!");

	if (!MULTIPLAYER_MASTER) {
		return;
	}

	int net_sig_idx;
	object* objp;

	for (ship & cur_ship : Ships) {
		
		if (cur_ship.objnum == -1) {
			continue;
		}		
		
		objp = &Objects[cur_ship.objnum];

		if (objp == nullptr) {
			continue;
		}
		 
		net_sig_idx = objp->net_signature;

		Assertion(net_sig_idx <= STANDALONE_SHIP_SIG, "Multi tracker got an invalid index of %d while updating it records. This is likely a coder error, please report!", net_sig_idx);
		// make sure it's a valid index.
		if (net_sig_idx < SHIP_SIG_MIN || net_sig_idx == STANDALONE_SHIP_SIG || net_sig_idx > SHIP_SIG_MAX) {
			 continue;
		}

		Oo_info.frame_info[net_sig_idx].positions[Oo_info.cur_frame_index] = objp->pos;
		Oo_info.frame_info[net_sig_idx].orientations[Oo_info.cur_frame_index] = objp->orient;
		Oo_info.frame_info[net_sig_idx].velocities[Oo_info.cur_frame_index] = objp->phys_info.vel;
	}
}

// Increment the tracker per frame, before packets are processed
void multi_ship_record_increment_frame() 
{
	Oo_info.number_of_frames++;
	Oo_info.cur_frame_index++;

	// Because we are only tracking 30 frames (up to a quarter second on a 120 frame machine), we will have to wrap the index often
	if (Oo_info.cur_frame_index == MAX_FRAMES_RECORDED) {
		Oo_info.cur_frame_index = 0;
		Oo_info.wrap_count++;
		if (Oo_info.wrap_count == MAX_SERVER_TRACKER_SMALL_WRAPS) {
			Oo_info.wrap_count = 0;
			Oo_info.larger_wrap_count++;
		}
	}

	Oo_info.timestamps[Oo_info.cur_frame_index] = timestamp();
}

// returns the last frame's index.
int multi_find_prev_frame_idx() 
{
	if (Oo_info.cur_frame_index == 0) {
		return MAX_FRAMES_RECORDED - 1;
	} else {
		return Oo_info.cur_frame_index - 1;
	}
}

// Finds the first frame that is before the incoming timestamp.
int multi_ship_record_find_frame(ushort client_frame, int time_elapsed)
{	
	// figure out the correct wrap
	int wrap = client_frame  / MAX_FRAMES_RECORDED;

	// figure out the frame index within the wrap
	int frame = client_frame % MAX_FRAMES_RECORDED;

	// if the wrap is not the same.
	if (wrap != Oo_info.wrap_count) {
		// somewhat out of date but still salvagable case.
		if (wrap == (Oo_info.wrap_count - 1)) {

			// but we can't use it if it would index to info in the current wrap instead of the previous wrap.
			if (frame <= Oo_info.cur_frame_index) {
				return -1;
			}
		// Just in case the larger wrap just happened....
		} else if ((wrap == MAX_SERVER_TRACKER_SMALL_WRAPS - 1) && Oo_info.wrap_count == 0){
			// again we can't use it if it would index to info in the current wrap instead of the previous wrap.
			if (frame <= Oo_info.cur_frame_index) {
				return -1;
			}

		} // otherwise the request is unsalvagable.
		else {
			return -1;
		}
	}

	// Now that the wrap has been verified, if time_elapsed is zero return the frame it gave us.
	if(time_elapsed == 0){
		return frame;
	}

	// Otherwise, look for the frame that the client is saying to look for.  If we hit the frame the client sent, return.
	// get how many frames we would go into the future.
	int target_timestamp = Oo_info.timestamps[frame] + time_elapsed;

	for (int i = Oo_info.cur_frame_index - 1; i > -1; i--) {

		// Check to see if the client's timestamp matches the recorded frames.
		if ((Oo_info.timestamps[i] <= target_timestamp) && (Oo_info.timestamps[i + 1] > target_timestamp)) {
			return i;
		}
		else if (i == frame) {
			return -1;
		}
	}

	// Check for an end of the wrap condition.
	if ((Oo_info.timestamps[MAX_FRAMES_RECORDED - 1] <= target_timestamp) && (Oo_info.timestamps[0] > target_timestamp)) {
		return MAX_FRAMES_RECORDED - 1;
	}

	// Check the oldest frames.
	for (int i = MAX_FRAMES_RECORDED - 2; i > Oo_info.cur_frame_index; i--) {
		if ((Oo_info.timestamps[i] <= target_timestamp) && (Oo_info.timestamps[i + 1] > target_timestamp)) {
			return i;
		}
		else if (i == frame) {
			return -1;
		}
	}

	// this is if again the request is way too delayed to be valid, but somehow wasn't caught earlier.
	return -1;
}

// Quick lookup for the record of position.
vec3d multi_ship_record_lookup_position(object* objp, int frame) 
{
	Assertion(objp != nullptr, "nullptr given to multi_ship_record_lookup_position. \nThis should be handled earlier in the code, please report!");
	return Oo_info.frame_info[objp->net_signature].positions[frame];
}

// Quick lookup for the record of orientation.
matrix multi_ship_record_lookup_orientation(object* objp, int frame) 
{
	Assertion(objp != nullptr, "nullptr given to multi_ship_record_lookup_position. \nThis should be handled earlier in the code, please report!");
	if (objp == nullptr) {
		return vmd_identity_matrix;
	}
	return Oo_info.frame_info[objp->net_signature].orientations[frame];
}

// quickly lookup how much time has passed between two frames.
uint multi_ship_record_get_time_elapsed(int original_frame, int new_frame) 
{
	// Bogus values
	Assertion(original_frame <= MAX_FRAMES_RECORDED, "Function multi_ship_record_get_time_elapsed() got passed an invalid original frame, this is a code error, please report. ");
	Assertion(new_frame <= MAX_FRAMES_RECORDED, "Function multi_ship_record_get_time_elapsed() got passed an invalid new frame, this is a code error, please report. ");
	if (original_frame >= MAX_FRAMES_RECORDED || new_frame >= MAX_FRAMES_RECORDED) {
		return 0;
	}

	return Oo_info.timestamps[new_frame] - Oo_info.timestamps[original_frame];
}

// figures out how much time has passed bwetween the two frames.
int multi_ship_record_find_time_after_frame(int starting_frame, int ending_frame, int time_elapsed) 
{
	starting_frame = starting_frame % MAX_FRAMES_RECORDED;

	int return_value = time_elapsed - (Oo_info.timestamps[ending_frame] - Oo_info.timestamps[starting_frame]);
	return return_value;
}

// ---------------------------------------------------------------------------------------------------
// Client side frame tracking, for now used only for referenced in fire packets to improve client accuracy.
// 

// See if a newly arrived object update packet should be the new reference for the improved primary fire packet 
void multi_ship_record_rank_seq_num(object* objp, ushort seq_num) 
{
	// see if it's more recent.  Most recent is best.
	if (seq_num > Oo_info.most_recent_frame || Oo_info.most_recent_updated_net_signature == 0) {
		Oo_info.most_recent_updated_net_signature = objp->net_signature;
		Oo_info.most_recent_frame = seq_num;
		Oo_info.ref_timestamp = timestamp();

	} // if this packet is from the same frame, the closer ship makes for a slightly more accurate reference point
	else if (seq_num == Oo_info.most_recent_frame) {
		object* temp_reference_object = multi_get_network_object(Oo_info.most_recent_updated_net_signature);
		// check the distance
		if ( (temp_reference_object == nullptr) || vm_vec_dist_squared(&temp_reference_object->pos, &Objects[Player->objnum].pos) > vm_vec_dist_squared(&objp->pos, &Objects[Player->objnum].pos) ) {
			Oo_info.most_recent_updated_net_signature = objp->net_signature;
			Oo_info.most_recent_frame = seq_num;
			Oo_info.ref_timestamp = timestamp();
		}
	}	// the wrap case (which should be rare), this could potentially break if the mission designer leaves
		// just 1 player completely by themselves on a long mission at *just* the right time.
	else if ((Oo_info.most_recent_frame > 65300) && seq_num < 65300) {
		Oo_info.most_recent_updated_net_signature = objp->net_signature;
		Oo_info.most_recent_frame = seq_num;
		Oo_info.ref_timestamp = timestamp();
	}
}

// Quick lookup for the most recently received ship
ushort multi_client_lookup_ref_obj_net_sig()
{	
	return Oo_info.most_recent_updated_net_signature;
}

// Quick lookup for the most recently received frame
ushort multi_client_lookup_frame_idx()
{
	return Oo_info.most_recent_frame;
}

// Quick lookup for the most recently received timestamp.
int multi_client_lookup_frame_timestamp()
{
	return Oo_info.ref_timestamp;
}

// Resets what info we have sent and interpolation info for a respawn
// To be safe, I believe that most info should be reset.
void multi_oo_respawn_reset_info(ushort net_sig) 
{
	Assertion(net_sig != 0, "Multi_reset_oo_info got passed an invalid value. This is a coder error, please report.");
	if (net_sig == 0) {
		return;
	}

	// When a player respawns, they keep their net signature, so clean up all the info that could mess things up in the future.
	for (auto & player_record : Oo_info.player_frame_info) {
		player_record.last_sent[net_sig].timestamp = -1;
		player_record.last_sent[net_sig].position = vmd_zero_vector;
		player_record.last_sent[net_sig].hull = -1.0f;
		player_record.last_sent[net_sig].ai_mode = -1;
		player_record.last_sent[net_sig].ai_submode = -1;
		player_record.last_sent[net_sig].target_signature = -1;
		player_record.last_sent[net_sig].perfect_shields_sent = false;
		for (uint i = 0; i < player_record.last_sent[net_sig].subsystems.size(); i++) {
			player_record.last_sent[net_sig].subsystems[i] = -1;
		}
	}

	oo_packet_and_interp_tracking* interp = &Oo_info.interp[net_sig];
	// To ensure clean interpolation, we should probably just reset everything.
	interp->ai_comparison_frame = -MAX_FRAMES_RECORDED;
	interp->cur_pack_pos_frame = -1;
	interp->prev_pack_pos_frame = -1;
	interp->pos_comparison_frame = -MAX_FRAMES_RECORDED;
	interp->prev_pos_comparison_frame = -MAX_FRAMES_RECORDED;
	interp->hull_comparison_frame = -MAX_FRAMES_RECORDED;
	interp->shields_comparison_frame = -MAX_FRAMES_RECORDED;
		for (auto & subsys : interp->subsystems_comparison_frame ){
			subsys = -MAX_FRAMES_RECORDED; // ship adder recreates the vector entries
		}

	interp->old_packet_position = vmd_zero_vector;
	interp->new_packet_position = vmd_zero_vector;
	interp->position_error = vmd_zero_vector;
	interp->pos_time_delta = 0.0f;

	interp->new_angles = vmd_zero_angles;
	interp->old_angles = vmd_zero_angles;
	interp->anticipated_angles_a = vmd_zero_angles;
	interp->anticipated_angles_b = vmd_zero_angles;
	interp->anticipated_angles_c = vmd_zero_angles;
	interp->new_orientation = vmd_identity_matrix;

	interp->client_simulation_mode = true;
	interp->prev_packet_positionless = false;

	interp->new_velocity = vmd_zero_vector;
	interp->anticipated_velocity1 = vmd_zero_vector;
	interp->anticipated_velocity2 = vmd_zero_vector;
	interp->anticipated_velocity3 = vmd_zero_vector;

	interp->cur_pack_des_rot_vel = vmd_zero_vector;
	interp->cur_pack_local_des_vel = vmd_zero_vector;
}

// ---------------------------------------------------------------------------------------------------
// OBJECT UPDATE FUNCTIONS
//

object *OO_player_obj;
int OO_sort = 1;

bool multi_oo_sort_func(const short &index1, const short &index2)
{
	object *obj1, *obj2;
	float dist1, dist2;
	float dot1, dot2;
	vec3d v1, v2;

	// if the indices are bogus, or the objnums are bogus, return ">"
	if((index1 < 0) || (index2 < 0) || (Ships[index1].objnum < 0) || (Ships[index2].objnum < 0)){
		return false;
	}

	// get the 2 objects
	obj1 = &Objects[Ships[index1].objnum];
	obj2 = &Objects[Ships[index2].objnum];

	// get the distance and dot product to the player obj for both
	vm_vec_sub(&v1, &OO_player_obj->pos, &obj1->pos);
	dist1 = vm_vec_normalize_safe(&v1);
	vm_vec_sub(&v2, &OO_player_obj->pos, &obj2->pos);
	dist2 = vm_vec_normalize_safe(&v2);
	dot1 = vm_vec_dot(&OO_player_obj->orient.vec.fvec, &v1);
	dot2 = vm_vec_dot(&OO_player_obj->orient.vec.fvec, &v2);

	// objects in front take precedence
	if((dot1 < 0.0f) && (dot2 >= 0.0f)){
		return false;
	} else if((dot2 < 0.0f) && (dot1 >= 0.0f)){
		return true;
	}

	// otherwise go by distance
	return (dist1 < dist2);
}

// build the list of ship indices to use when updating for this player
void multi_oo_build_ship_list(net_player *pl)
{
	int ship_index;
	int idx;
	ship_obj *moveup;
	object *player_obj;

	// set all indices to be -1
	for(idx = 0;idx<MAX_SHIPS; idx++){
		OO_ship_index[idx] = -1;
	}

	// get the player object
	if(pl->m_player->objnum < 0){
		return;
	}
	player_obj = &Objects[pl->m_player->objnum];
	
	// go through all other relevant objects
	ship_index = 0;
	for ( moveup = GET_FIRST(&Ship_obj_list); moveup != END_OF_LIST(&Ship_obj_list); moveup = GET_NEXT(moveup) ) {
		// if it is an invalid ship object, skip it
		if((moveup->objnum < 0) || (Objects[moveup->objnum].instance < 0) || (Objects[moveup->objnum].type != OBJ_SHIP)){
			continue;
		}

		// if we're a standalone server, don't send any data regarding its pseudo-ship
		if((Game_mode & GM_STANDALONE_SERVER) && ((&Objects[moveup->objnum] == Player_obj) || (Objects[moveup->objnum].net_signature == STANDALONE_SHIP_SIG)) ){
			continue;
		}		
			
		// must be a ship, a weapon, and _not_ an observer
		if (Objects[moveup->objnum].flags[Object::Object_Flags::Should_be_dead]){
			continue;
		}

		// don't send info for dying ships -- Cyborg17 - Or dead ships that are going to respawn later.
		if (Ships[Objects[moveup->objnum].instance].flags[Ship::Ship_Flags::Dying] || Ships[Objects[moveup->objnum].instance].flags[Ship::Ship_Flags::Exploded]) {
			continue;
		}		

		// never update the knossos device
		if ((Ships[Objects[moveup->objnum].instance].ship_info_index >= 0) && (Ships[Objects[moveup->objnum].instance].ship_info_index < ship_info_size()) && (Ship_info[Ships[Objects[moveup->objnum].instance].ship_info_index].flags[Ship::Info_Flags::Knossos_device])){
			continue;
		}
				
		// don't send him info for himself
		if ( &Objects[moveup->objnum] == player_obj ){
			continue;
		}

		// don't send info for his targeted ship here, since its always done first
		if((pl->s_info.target_objnum != -1) && (moveup->objnum == pl->s_info.target_objnum)){
			continue;
		}

		// add the ship 
		if(ship_index < MAX_SHIPS){
			OO_ship_index[ship_index++] = (short)Objects[moveup->objnum].instance;
		}
	}

	// maybe sort the thing here
	OO_player_obj = player_obj;
	if (OO_sort) {
		std::sort(OO_ship_index, OO_ship_index + ship_index, multi_oo_sort_func);
	}
}

// pack information for a client (myself), return bytes added
int multi_oo_pack_client_data(ubyte *data)
{
	ubyte out_flags;
	ushort tnet_signature;
	char t_subsys, l_subsys;
	int packet_size = 0;

	// get our firing stuff
	out_flags = Net_player->s_info.accum_buttons;	

	// zero these values for now
	Net_player->s_info.accum_buttons = 0;

	// add any necessary targeting flags
	if ( Player_ai->current_target_is_locked ){
		out_flags |= OOC_TARGET_LOCKED;
	}
	if ( Player_ai->ai_flags[AI::AI_Flags::Seek_lock] ){	
		out_flags |= OOC_TARGET_SEEK_LOCK;
	}
	if ( Player->locking_on_center ){
		out_flags |= OOC_LOCKING_ON_CENTER;
	}
	if ( (Player_ship != NULL) && (Player_ship->flags[Ship::Ship_Flags::Trigger_down]) ){
		out_flags |= OOC_TRIGGER_DOWN;
	}

	if ( (Player_obj != NULL) && Player_obj->phys_info.flags & PF_AFTERBURNER_ON){
		out_flags |= OOC_AFTERBURNER_ON;
	}

	// send my bank info
	if(Player_ship != NULL){
		if(Player_ship->weapons.current_primary_bank > 0){
			out_flags |= OOC_PRIMARY_BANK;
		}

		// linked or not
		if(Player_ship->flags[Ship::Ship_Flags::Primary_linked]){
			out_flags |= OOC_PRIMARY_LINKED;
		}
	}

	// copy the final flags in
	ADD_DATA( out_flags );

	// client targeting information	
	t_subsys = -1;
	l_subsys = -1;

	// if nothing targeted
	if(Player_ai->target_objnum == -1){
		tnet_signature = 0;
	}
	// if something is targeted 
	else {
		// target net signature
		tnet_signature = Objects[Player_ai->target_objnum].net_signature;
			
		// targeted subsys index
		if(Player_ai->targeted_subsys != NULL){
			t_subsys = (char)ship_get_index_from_subsys( Player_ai->targeted_subsys, Player_ai->target_objnum );
		}

		// locked targeted subsys index
		if(Player->locking_subsys != NULL){
			l_subsys = (char)ship_get_index_from_subsys( Player->locking_subsys, Player_ai->target_objnum );
		}
	}

	// add them all
	ADD_USHORT( tnet_signature );
	ADD_DATA( t_subsys );
	ADD_DATA( l_subsys );

	return packet_size;
}

// pack the appropriate info into the data
#define PACK_PERCENT(v) { std::uint8_t upercent; if(v < 0.0f){v = 0.0f;} upercent = (v * 255.0f) <= 255.0f ? (std::uint8_t)(v * 255.0f) : (std::uint8_t)255; memcpy(data + packet_size + header_bytes, &upercent, sizeof(std::uint8_t)); packet_size++; }
#define PACK_BYTE(v) { memcpy( data + packet_size + header_bytes, &v, 1 ); packet_size += 1; }
#define PACK_SHORT(v) { std::int16_t swap = INTEL_SHORT(v); memcpy( data + packet_size + header_bytes, &swap, sizeof(std::int16_t) ); packet_size += sizeof(std::int16_t); }
#define PACK_USHORT(v) { std::uint16_t swap = INTEL_SHORT(v); memcpy( data + packet_size + header_bytes, &swap, sizeof(std::uint16_t) ); packet_size += sizeof(std::uint16_t); }
#define PACK_INT(v) { std::int32_t swap = INTEL_INT(v); memcpy( data + packet_size + header_bytes, &swap, sizeof(std::int32_t) ); packet_size += sizeof(std::int32_t); }
#define PACK_ULONG(v) { std::uint64_t swap = INTEL_LONG(v); memcpy( data + packet_size + header_bytes, &swap, sizeof(std::uint64_t) ); packet_size += sizeof(std::uint64_t); }
int multi_oo_pack_data(net_player *pl, object *objp, ushort oo_flags, ubyte *data_out)
{
	ubyte data[255];
	ubyte data_size = 0, ret = 0;	
	ship *shipp;	
	ship_info *sip;
	float temp;	
	int header_bytes;
	int packet_size = 0;

	// make sure we have a valid ship
	Assert(objp->type == OBJ_SHIP);
	if((objp->instance >= 0) && (Ships[objp->instance].ship_info_index >= 0)){
		shipp = &Ships[objp->instance];
		sip = &Ship_info[shipp->ship_info_index];
	} else {
		return 0;
	}			

	// invalid player
	if(pl == nullptr || shipp == nullptr){
		return 0;
	}

	// if no flags we now send an "empty" packet that tells the client "Keep this ship where it belongs"

	// if i'm the client, make sure I only send certain things	
	if (MULTIPLAYER_CLIENT) {
		Assert(!(oo_flags & (OO_HULL_NEW | OO_SHIELDS_NEW | OO_SUBSYSTEMS_NEW)));
		oo_flags &= ~(OO_HULL_NEW | OO_SHIELDS_NEW | OO_SUBSYSTEMS_NEW);
	} 

	// header sizes -- Cyborg17 - Note this is in place because the size of the packet is 
	// determined at the end of this function, and so we have to keep track of how much
	// we are adding to the packet throughout.
	if(MULTIPLAYER_MASTER){
		header_bytes = 7;
	} else {
		header_bytes = 5;
	}	

	ushort temp_timestamp = (ushort)(Oo_info.timestamps[Oo_info.cur_frame_index] - Oo_info.timestamps[multi_find_prev_frame_idx()]);

	// only the very longest frames are going to have greater than 255 ms, so cap it at that.
	if (temp_timestamp > 255) {
		temp_timestamp = 255;
	}
	ubyte timestamp_out = (ubyte)temp_timestamp;
	PACK_BYTE(timestamp_out);

	// putting this in the position bucket because it's mainly to help with position interpolation
	multi_rate_add(NET_PLAYER_NUM(pl), "pos", 1);


	// if we're a client (and therefore sending control info), pack client-specific info
	if((Net_player != nullptr) && !(Net_player->flags & NETINFO_FLAG_AM_MASTER)){
		packet_size += multi_oo_pack_client_data(data + packet_size + header_bytes);		
	}		
		
	// position - Now includes, position, orientation, velocity, rotational velocity, desired velocity and desired rotational velocity.
	if ( oo_flags & OO_POS_AND_ORIENT_NEW ) {	
		ret = (ubyte)multi_pack_unpack_position( 1, data + packet_size + header_bytes, &objp->pos );
		packet_size += ret;

		// datarate tracking.
		multi_rate_add(NET_PLAYER_NUM(pl), "pos", ret);

		// orientation (now done via angles)
		angles temp_angles;
		vm_extract_angles_matrix_alternate(&temp_angles, &objp->orient);	

		ret = (ubyte)multi_pack_unpack_orient( 1, data + packet_size + header_bytes, &temp_angles);

		packet_size += ret;
		// datarate tracking.
		multi_rate_add(NET_PLAYER_NUM(pl), "ori", ret);	

		// velocity -- Tried to do this by calculation instead but kept running into issues. 
		ret = (ubyte)multi_pack_unpack_vel(1, data + packet_size + header_bytes, &objp->orient, &objp->phys_info);

		packet_size += ret;
		// datarate tracking.
		multi_rate_add(NET_PLAYER_NUM(pl), "pos", ret);	

		// Rotational Velocity,
		ret = (ubyte)multi_pack_unpack_rotvel( 1, data + packet_size + header_bytes, &objp->phys_info );

		packet_size += ret;	

		// datarate tracking.		
		multi_rate_add(NET_PLAYER_NUM(pl), "ori", ret);		
		ret = 0;

		// in order to send data by axis we must rotate the global velocity into local coordinates
		vec3d local_desired_vel;

		vm_vec_rotate(&local_desired_vel, &objp->phys_info.desired_vel, &objp->orient);
		
		ret = multi_pack_unpack_desired_vel_and_desired_rotvel(1, data + packet_size + header_bytes, &objp->phys_info, &local_desired_vel);

		packet_size += ret;
	}

	// datarate records	
	multi_rate_add(NET_PLAYER_NUM(pl), "fth", ret);	

	// hull info
	if (oo_flags & OO_HULL_NEW) {
		// add the hull value for this guy		
		temp = get_hull_pct(objp);
		if ((temp < 0.004f) && (temp > 0.0f)) {
			temp = 0.004f;		// 0.004 is the lowest positive value we can have before we zero out when packing
		}
		PACK_PERCENT(temp);
		multi_rate_add(NET_PLAYER_NUM(pl), "hul", 1);
	}

	if (oo_flags & OO_SHIELDS_NEW) {
		float quad = shield_get_max_quad(objp);

		for (int i = 0; i < objp->n_quadrants; i++) {
			temp = (objp->shield_quadrant[i] / quad);
			PACK_PERCENT(temp);
		}
				
		multi_rate_add(NET_PLAYER_NUM(pl), "shl", objp->n_quadrants);	
	}	

	// Cyborg17 - only server should send this
	// just in case we have some kind of invalid data (should've been taken care of earlier in this function)
	if (MULTIPLAYER_MASTER && shipp->ship_info_index >= 0) {	
	// Cyborg17 - add the # of only the subsystems being changed, and their data
		ship_subsys* subsystem;
		ubyte count = 0;
		ubyte flagged_subsystem_list[MAX_MODEL_SUBSYSTEMS];
		float subsystem_list_health[MAX_MODEL_SUBSYSTEMS];
		int i = 0;

		for (subsystem = GET_FIRST(&shipp->subsys_list); subsystem != END_OF_LIST(&shipp->subsys_list);
			subsystem = GET_NEXT(subsystem)) {
				// Don't send destroyed subsystems, (another packet handles that), but check to see if the subsystem changed since the last update. 
			if ((subsystem->current_hits != 0.0f) && (subsystem->current_hits != Oo_info.player_frame_info[pl->player_id].last_sent[objp->net_signature].subsystems[i])) {
				// store the values for use later.
				flagged_subsystem_list[count] = (ubyte)i;
				// this should be safe because we only work with subsystems that have health.
				subsystem_list_health[count] = subsystem->current_hits / subsystem->max_hits;
				// and also track the list of subsystems that we packed by index
				count++;
			}
			i++;
		}
		
		// Only send info if the count is greater than zero and if we're *not* on the very first frame when everything is going to be 100%, anyway.
		if (count > 0 && Oo_info.number_of_frames != 0){
			Assertion(count <= MAX_MODEL_SUBSYSTEMS, "Object Update packet exceeded limit for number of subsystems. This is a coder error, please report!\n");
			oo_flags |= OO_SUBSYSTEMS_NEW;

			// pack the count of subsystems first.
			PACK_BYTE(count);
			// now we'll pack the actual information
			for (int j = 0; j < count; j++) {
				PACK_BYTE(flagged_subsystem_list[j]);
				PACK_PERCENT(subsystem_list_health[j]);
				Oo_info.player_frame_info[pl->player_id].last_sent[objp->net_signature].subsystems[flagged_subsystem_list[j]] = subsystem_list_health[j];
			}
		}
	}

	// Cyborg17 - only server should send this
	if (oo_flags & OO_AI_NEW){
		// ai mode info
		ubyte umode = (ubyte)(Ai_info[shipp->ai_index].mode);
		short submode = (short)(Ai_info[shipp->ai_index].submode);
		ushort target_signature;

		target_signature = 0;
		if ( Ai_info[shipp->ai_index].target_objnum != -1 ){
			target_signature = Objects[Ai_info[shipp->ai_index].target_objnum].net_signature;
		}

		PACK_BYTE( umode );
		PACK_SHORT( submode );
		PACK_USHORT( target_signature );	

		multi_rate_add(NET_PLAYER_NUM(pl), "aim", 5);

		// primary weapon energy
		temp = shipp->weapon_energy / sip->max_weapon_reserve;
		PACK_PERCENT(temp);
	}		

	// if this ship is a support ship, send some extra info
	if(MULTIPLAYER_MASTER && (sip->flags[Ship::Info_Flags::Support]) && (shipp->ai_index >= 0) && (shipp->ai_index < MAX_AI_INFO)){
		oo_flags |= OO_SUPPORT_SHIP;
		ushort dock_sig;

		PACK_ULONG( Ai_info[shipp->ai_index].ai_flags.to_u64() );
		PACK_INT( Ai_info[shipp->ai_index].mode );
		PACK_INT( Ai_info[shipp->ai_index].submode );

		if((Ai_info[shipp->ai_index].support_ship_objnum < 0) || (Ai_info[shipp->ai_index].support_ship_objnum >= MAX_OBJECTS)){
			dock_sig = 0;
		} else {
			dock_sig = Objects[Ai_info[shipp->ai_index].support_ship_objnum].net_signature;
		}		

		PACK_USHORT( dock_sig );
	}

	// afterburner info
	oo_flags &= ~OO_AFTERBURNER_NEW;
	if(objp->phys_info.flags & PF_AFTERBURNER_ON){
		oo_flags |= OO_AFTERBURNER_NEW;
	}

	// make sure we have a chunk of data that will fit.
	Assert(packet_size < 500);
	if(packet_size >= 500){
		return 0;
	}
	data_size = (ubyte)packet_size;

	// reset packet_size so that we add the header at the beginning of the packet where it belongs.
	packet_size = 0;
	// don't add for clients
	if(MULTIPLAYER_MASTER){		
		multi_rate_add(NET_PLAYER_NUM(pl), "sig", 2);
		ADD_USHORT( objp->net_signature );
	}	

	if (Oo_info.larger_wrap_count % 2 > 0) {
		oo_flags |= OO_ODD_WRAP;
	}

	multi_rate_add(NET_PLAYER_NUM(pl), "flg", 1);
	ADD_USHORT( oo_flags );

	multi_rate_add(NET_PLAYER_NUM(pl), "siz", 1);
	ADD_DATA( data_size );	
	ushort seq = Oo_info.cur_frame_index + (MAX_FRAMES_RECORDED * Oo_info.wrap_count);
	
	multi_rate_add(NET_PLAYER_NUM(pl), "seq", 2);
	ADD_USHORT( seq );
	packet_size += data_size;

	// copy to the outgoing data
	memcpy(data_out, data, packet_size);	
	
	return packet_size;	
}

// unpack information for a client , return bytes processed
int multi_oo_unpack_client_data(net_player *pl, ubyte *data)
{
	ubyte in_flags;
	ship *shipp = NULL;
	object *objp = NULL;
	int offset = 0;

	if (pl == NULL)
		Error(LOCATION, "Invalid net_player pointer passed to multi_oo_unpack_client\n");
	
	memcpy(&in_flags, data, sizeof(ubyte));	
	offset++;

	// get the player ship and object
	if((pl->m_player->objnum >= 0) && (Objects[pl->m_player->objnum].type == OBJ_SHIP) && (Objects[pl->m_player->objnum].instance >= 0)){
		objp = &Objects[pl->m_player->objnum];
		shipp = &Ships[objp->instance];
	}
		
	// if we have a valid netplayer pointer
	if((pl != NULL) && !(pl->flags & NETINFO_FLAG_RESPAWNING) && !(pl->flags & NETINFO_FLAG_LIMBO)){
		// primary fired
		pl->m_player->ci.fire_primary_count = 0;		

		// secondary fired
		pl->m_player->ci.fire_secondary_count = 0;
		if ( in_flags & OOC_FIRE_SECONDARY ){
			pl->m_player->ci.fire_secondary_count = 1;
		}

		// countermeasure fired		
		pl->m_player->ci.fire_countermeasure_count = 0;		

		// set up aspect locking information
		pl->m_player->locking_on_center = 0;
		if ( in_flags & OOC_LOCKING_ON_CENTER ){
			pl->m_player->locking_on_center = 1;
		}		

		// trigger down, bank info
		if(shipp != NULL){
			if(in_flags & OOC_TRIGGER_DOWN){
				shipp->flags.set(Ship::Ship_Flags::Trigger_down);
			} else {
				shipp->flags.remove(Ship::Ship_Flags::Trigger_down);
			}
			
			if(in_flags & OOC_PRIMARY_BANK){		
				shipp->weapons.current_primary_bank = 1;
			} else {
				shipp->weapons.current_primary_bank = 0;
			}

			// linked or not								
            shipp->flags.remove(Ship::Ship_Flags::Primary_linked);
			if(in_flags & OOC_PRIMARY_LINKED){				
				shipp->flags.set(Ship::Ship_Flags::Primary_linked);
			}
		}

		// other locking information
		if((shipp != NULL) && (shipp->ai_index != -1)){			
			Ai_info[shipp->ai_index].current_target_is_locked = ( in_flags & OOC_TARGET_LOCKED) ? 1 : 0;
			Ai_info[shipp->ai_index].ai_flags.set(AI::AI_Flags::Seek_lock, (in_flags & OOC_TARGET_SEEK_LOCK) != 0);
		}

		// afterburner status
		if ( (objp != NULL) && (in_flags & OOC_AFTERBURNER_ON) ) {
			Afterburn_hack = true;
		}
	}
	
	// client targeting information	
	ushort tnet_sig;
	char t_subsys,l_subsys;
	object *tobj;

	// get the data
	GET_USHORT(tnet_sig);
	GET_DATA(t_subsys);
	GET_DATA(l_subsys);

	// try and find the targeted object
	tobj = NULL;
	if(tnet_sig != 0){
		tobj = multi_get_network_object( tnet_sig );
	}
	// maybe fill in targeted object values
	if((tobj != NULL) && (pl != NULL) && (pl->m_player->objnum != -1)){
		// assign the target object
		if(Objects[pl->m_player->objnum].type == OBJ_SHIP){
			Ai_info[Ships[Objects[pl->m_player->objnum].instance].ai_index].target_objnum = OBJ_INDEX(tobj);
		}
		pl->s_info.target_objnum = OBJ_INDEX(tobj);

		// assign subsystems if possible					
		if(Objects[pl->m_player->objnum].type == OBJ_SHIP){		
			Ai_info[Ships[Objects[pl->m_player->objnum].instance].ai_index].targeted_subsys = NULL;
			if((t_subsys != -1) && (tobj->type == OBJ_SHIP)){
				Ai_info[Ships[Objects[pl->m_player->objnum].instance].ai_index].targeted_subsys = ship_get_indexed_subsys( &Ships[tobj->instance], t_subsys);
			}
		}

		pl->m_player->locking_subsys = NULL;
		if(Objects[pl->m_player->objnum].type == OBJ_SHIP){		
			if((l_subsys != -1) && (tobj->type == OBJ_SHIP)){
				pl->m_player->locking_subsys = ship_get_indexed_subsys( &Ships[tobj->instance], l_subsys);
			}				
		}
	}				

	return offset;
}

// unpack the object data, return bytes processed
// Cyborg17 - This function has been revamped to ignore out of date information by type.  For example, if we got pos info
// more recently, but the packet has the newest AI info, we will still use the AI info, even though it's not the newest
// packet.
#define UNPACK_PERCENT(v)					{ ubyte temp_byte; memcpy(&temp_byte, data + offset, sizeof(ubyte)); v = (float)temp_byte / 255.0f; offset++;}
int multi_oo_unpack_data(net_player* pl, ubyte* data)
{
	int offset = 0;
	object* pobjp;
	ushort net_sig = 0;
	ubyte data_size;
	ushort seq_num, oo_flags;
	float fpct;
	ship* shipp;
	ship_info* sip;

	// ---------------------------------------------------------------------------------------------------------------
	// Header Processing
	// ---------------------------------------------------------------------------------------------------------------

	// add the object's net signature, type and oo_flags
	if (!(Net_player->flags & NETINFO_FLAG_AM_MASTER)) {
		GET_USHORT(net_sig);
	}

	// clients always pos and orient stuff only
	GET_USHORT(oo_flags);
	GET_DATA(data_size);
	GET_USHORT(seq_num);
	if (MULTIPLAYER_MASTER) {
		// client cannot send these types because the server is in charge of all of these things.
		Assertion(!(oo_flags & (OO_AI_NEW | OO_SHIELDS_NEW | OO_HULL_NEW | OO_SUBSYSTEMS_NEW | OO_SUPPORT_SHIP)), "Invalid flag from client, please report! oo_flags value: %d\n", oo_flags);
		if (oo_flags & (OO_AI_NEW | OO_SHIELDS_NEW | OO_HULL_NEW | OO_SUBSYSTEMS_NEW | OO_SUPPORT_SHIP)) {
			offset += data_size;
			return offset;
		}
	}
	// try and find the object
	if (MULTIPLAYER_CLIENT) {
		pobjp = multi_get_network_object(net_sig);
	}
	else {
		if ((pl != nullptr) && (pl->m_player->objnum != -1)) {
			pobjp = &Objects[pl->m_player->objnum];
			// Cyborg17 - We still need the net_signature if we're the Server because that's how we track interpolation info now.
			net_sig = pobjp->net_signature;
		}
		else {
			pobjp = nullptr;
		}
	}

	// if we can't find the object, skip the packet
	if ( (pobjp == nullptr) || (pobjp->type != OBJ_SHIP) || (pobjp->instance < 0) || (pobjp->instance >= MAX_SHIPS) || (Ships[pobjp->instance].ship_info_index < 0) || (Ships[pobjp->instance].ship_info_index >= ship_info_size())) {
		offset += data_size;
		return offset;
	}

	// ship pointer
	shipp = &Ships[pobjp->instance];
	sip = &Ship_info[shipp->ship_info_index];

	// used to access stuff in vectors by index
	bool pre_wrap_packet = false;

	Assertion(net_sig <= Oo_info.interp.size(), "Somehow there weren't enough copies of the interpolation tracking info created.");

	oo_packet_and_interp_tracking* interp_data = &Oo_info.interp[net_sig];

	// two variables to make the following code more readable.
	int most_recent = interp_data->most_recent_packet;
	bool prev_odd_wrap = interp_data->odd_wrap;

	// same wrap case ... if they are both true or both false.
	if ( (oo_flags & OO_ODD_WRAP && prev_odd_wrap == true ) || (!(oo_flags & OO_ODD_WRAP) && (prev_odd_wrap == false) )) {
		// just check that it's in order before saying it's the most recent.
		if (seq_num > most_recent) {

			interp_data->most_recent_packet = seq_num;

		}
		// we do not need to mark anything for out of order packets within the same wrap. Checks within
		// each individual section will handle the rest.

	} // not the same wrap
	else {
		// this means we just wrapped and we have the first packet from the new wrap
		if (seq_num < most_recent) {

			// with a new wrap, we have to adjust the individual tracker 
			// records so that FSO can tell that the incoming frames after this are newer than what it already saw.
			if ( most_recent - seq_num > HAS_WRAPPED_MINIMUM ) {
				interp_data->most_recent_packet = seq_num;
				interp_data->odd_wrap = !interp_data->odd_wrap;

				interp_data->pos_comparison_frame -= SERVER_TRACKER_LARGE_WRAP_TOTAL;
				interp_data->prev_pos_comparison_frame -= SERVER_TRACKER_LARGE_WRAP_TOTAL;
				interp_data->hull_comparison_frame -= SERVER_TRACKER_LARGE_WRAP_TOTAL;
				interp_data->shields_comparison_frame -= SERVER_TRACKER_LARGE_WRAP_TOTAL;

				for (auto subsys_frame = interp_data->subsystems_comparison_frame.begin(); subsys_frame != interp_data->subsystems_comparison_frame.end(); subsys_frame++) {

					subsys_frame -= SERVER_TRACKER_LARGE_WRAP_TOTAL;
				}
				interp_data->ai_comparison_frame -= SERVER_TRACKER_LARGE_WRAP_TOTAL;

			}
		} // if this a pre-wrap out-of-order packet, we have to mark it as so, so that we adjust seq_num for the individual checks
		else {

			pre_wrap_packet = true;
		}
	}

	// Cyborg17 - determine if this is the most recently updated ship.  If it is, it will become the ship that the
	// client will use as its reference when sending a primary shot packet.
	if (MULTIPLAYER_CLIENT) {
		multi_ship_record_rank_seq_num(pobjp, seq_num);
	}

	int pos_and_time_data_size = 0;

	// get the timestamp that belonged to this server for this frame.
	// Because we want as many timestamps as possible, we want to record what we get, no matter when it came from.
	ubyte received_timestamp;
	GET_DATA(received_timestamp);
	pos_and_time_data_size++;

	// figure out how many items we may have to create
	int temp_diff = (int)seq_num - (int)Oo_info.received_frametimes.size() + 1;
	// if it already has enough slots, just fill in the value.
	if (temp_diff <= 0) {
		Oo_info.received_frametimes[seq_num] = received_timestamp;
	}	// if there weren't enough slots, create the necessary slots.
	else {
		// loop is checked against 1, because once there is only a difference of 1, we should add the timestamp onto the end.
		for (int i = temp_diff; i > 1; i--) {
			// keep adding zero to the timestamps we have not yet received, because that is our impossible value.
			Oo_info.received_frametimes.push_back(0);
		}
		// lastly, add the timestamp we received to the end.
		Oo_info.received_frametimes.push_back(received_timestamp);
	}
	

	// ---------------------------------------------------------------------------------------------------------------
	// SPECIAL CLIENT INFO
	// ---------------------------------------------------------------------------------------------------------------

	// if this is from a player, read his button info
	if(MULTIPLAYER_MASTER){
		int r0 = multi_oo_unpack_client_data(pl, data + offset);		
		pos_and_time_data_size += r0;
		offset += r0;
	}

	// ---------------------------------------------------------------------------------------------------------------
	// CRITICAL OBJECT UPDATE SHIZ
	// ---------------------------------------------------------------------------------------------------------------
	// (Positon, Orientation, and the related velocities and desired velocities)

	// Have "new info" default to the old info before reading
	vec3d new_pos = pobjp->pos;
	angles new_angles;
	matrix new_orient = pobjp->orient;
	physics_info new_phys_info = pobjp->phys_info;

	bool pos_new = false, adjust_interp_pos = false;

	int frame_comparison = seq_num;

	// calculate which seq_num to compare against.
	if (pre_wrap_packet) {
		frame_comparison -= SERVER_TRACKER_LARGE_WRAP_TOTAL;
	}

	if ( oo_flags & OO_POS_AND_ORIENT_NEW) {

		// unpack position
		int r1 = multi_pack_unpack_position(0, data + offset, &new_pos);
		offset += r1;
		pos_and_time_data_size += r1;

		// unpack orientation
		int r2 = multi_pack_unpack_orient( 0, data + offset, &new_angles );
		offset += r2;
		pos_and_time_data_size += r2;
		
		// new version of the orient packer sends angles instead to save on bandwidth, so we'll need the orienation from that.
		vm_angles_2_matrix(&new_orient, &new_angles);
//		vm_orthogonalize_matrix(&new_orient);

		int r3 = multi_pack_unpack_vel(0, data + offset, &new_orient, &new_phys_info);
		offset += r3;
		pos_and_time_data_size += r3;

		int r4 = multi_pack_unpack_rotvel( 0, data + offset, &new_phys_info );
		offset += r4;
		pos_and_time_data_size += r4;

		vec3d local_desired_vel = vmd_zero_vector;

		ubyte r5 = multi_pack_unpack_desired_vel_and_desired_rotvel(0, data + offset, &pobjp->phys_info, &local_desired_vel);
		offset += r5;
		// change it back to global coordinates.
		vm_vec_unrotate(&new_phys_info.desired_vel, &local_desired_vel, &new_orient);
		pos_and_time_data_size += r5;

		// make sure this is the newest frame sent and then start storing everything.
		if (frame_comparison > interp_data->pos_comparison_frame) {
			// mark this packet as a brand new update.
			pos_new = true;

			// make sure to turn off no position change mode.
			interp_data->prev_packet_positionless = false;

			// update timing info, position
			interp_data->prev_pack_pos_frame = interp_data->cur_pack_pos_frame;
			interp_data->prev_pos_comparison_frame = interp_data->pos_comparison_frame;
			interp_data->cur_pack_pos_frame = interp_data->pos_comparison_frame = seq_num;
			// double check that we have valid data
			if (interp_data->prev_pack_pos_frame != interp_data->cur_pack_pos_frame) {
				adjust_interp_pos = true;
			}

			interp_data->pos_timestamp = timestamp();

		} // if we actually received a slightly old frame...
		else if (frame_comparison > interp_data->prev_pos_comparison_frame){
			//update timing info.
			if (seq_num != interp_data->cur_pack_pos_frame) {
				interp_data->prev_pack_pos_frame = interp_data->prev_pos_comparison_frame = seq_num;
				adjust_interp_pos = true;
			}
		}

		// implement desired_vel and desired rotvel
		if (pos_new) {
			interp_data->cur_pack_des_vel = new_phys_info.desired_vel;
			interp_data->cur_pack_local_des_vel = local_desired_vel;
			interp_data->cur_pack_des_rot_vel = new_phys_info.desired_rotvel;

			pobjp->phys_info = new_phys_info;			
		}

		float temp_distance = vm_vec_dist(&new_pos, &pobjp->pos);
		
		// Cyborg17 - fully bash if we're past the position update tolerance or not moving
		// Past the update tolerance will cause a jump, but it should be nice and smooth immediately afterwards
		if(pos_new && (temp_distance > OO_POS_UPDATE_TOLERANCE || new_phys_info.vel == vmd_zero_vector)){
			pobjp->pos = new_pos;
			//Also, make sure that FSO knows that it does not need to smooth anything out
			interp_data->position_error = vmd_zero_vector;
		} 	// When not bashing, find how much error to smooth out during interpolation.
		else {
			vm_vec_sub(&interp_data->position_error, &pobjp->pos, &new_pos);
		}

		// for orientation, we should bash for brand new orientations and then let interpolation do its work.  
		// This is similar to the straight siming that was done before, but the new interpolation makes 
		// the movement smoother by having the two simulated together when creating the spline 
		// and using the same time deltas.
		if (pos_new){
			pobjp->orient = new_orient;
			interp_data->new_orientation = new_orient;
		}

		multi_oo_maybe_update_interp_info(pobjp, &new_pos, &new_angles, &new_orient, &new_phys_info, adjust_interp_pos, pos_new);

	} // in order to allow the server to send only new pos and ori info, we have to do a couple checks here
	else if (seq_num == interp_data->most_recent_packet){
		if (interp_data->prev_packet_positionless == false) {
			interp_data->prev_packet_positionless = true;
			interp_data->prev_pack_pos_frame = interp_data->cur_pack_pos_frame ;
			interp_data->cur_pack_pos_frame = seq_num;
			
			// bash the ship's velocity and rotational velocity to zero
			pobjp->phys_info.vel = vmd_zero_vector;
			pobjp->phys_info.desired_vel = vmd_zero_vector;
			pobjp->phys_info.rotvel = vmd_zero_vector;
			pobjp->phys_info.desired_rotvel = vmd_zero_vector;

			// if we haven't moved, there is *no* position error.
			interp_data->position_error = vmd_zero_vector;
		}
	}

	// Packet processing needs to stop here if the ship is still arriving, leaving, dead or dying to prevent bugs.
	if (shipp->is_arriving() || shipp->is_dying_or_departing() || shipp->flags[Ship::Ship_Flags::Exploded]) {
		int header_bytes = (MULTIPLAYER_MASTER) ? 5 : 7;		
		offset = header_bytes + data_size;
		return offset;
	}

	// ---------------------------------------------------------------------------------------------------------------
	// SHIP STATUS
	// ---------------------------------------------------------------------------------------------------------------
	
	// hull info
	if ( oo_flags & OO_HULL_NEW ){
		UNPACK_PERCENT(fpct);
		if (interp_data->hull_comparison_frame < frame_comparison) {
			pobjp->hull_strength = fpct * Ships[pobjp->instance].ship_max_hull_strength;
			interp_data->hull_comparison_frame = seq_num;
		}
	}	

	// update shields
	if (oo_flags & OO_SHIELDS_NEW) {
		float quad = shield_get_max_quad(pobjp);

		// check before unpacking here so we don't have to recheck for each quadrant.
		if (interp_data->shields_comparison_frame < frame_comparison) {
			for (int i = 0; i < pobjp->n_quadrants; i++) {
				UNPACK_PERCENT(fpct);
				pobjp->shield_quadrant[i] = fpct * quad;
			}
			interp_data->shields_comparison_frame = seq_num;
		}
		else {
			for (int i = 0; i < pobjp->n_quadrants; i++) {
				UNPACK_PERCENT(fpct);
			}
		}
	}


	if (oo_flags & OO_SUBSYSTEMS_NEW) {
		ubyte n_subsystems, subsys_count = 0;
		ship_subsys* subsysp, * firstsubsys = GET_FIRST(&shipp->subsys_list);
		ubyte current_subsystem = 0;
		float current_percent = 0.0f;

		// get the number of subsystems
		GET_DATA(n_subsystems);

		// store the old offset in case of misaligned info.
		int pre_sub_offset = offset;

		// Before we start the loop, we need to get the first subsystem
		GET_DATA(current_subsystem);

		// this iterates through the packet and subsytem list simultaneously, changing values only when it finds a match.
		for (subsysp = firstsubsys; subsysp != END_OF_LIST(&shipp->subsys_list); subsysp = GET_NEXT(subsysp)) {

			auto idx = std::distance(firstsubsys, subsysp);

			if (current_subsystem != idx) {
				// the current subsystem was not sent by the server, so try the next subsystem.
				continue;
			}

			// we found a match, so grab the next byte, so we can calculate the new hitpoints
			UNPACK_PERCENT(current_percent);
			subsys_count++;

			// update frame is *per* subsystem here
			if (frame_comparison > interp_data->subsystems_comparison_frame[idx]) {
				subsysp->current_hits = current_percent * subsysp->max_hits;

				// Aggregate if necessary.
				if (!(subsysp->flags[Ship::Subsystem_Flags::No_aggregate])) {
					shipp->subsys_info[subsysp->system_info->type].aggregate_current_hits += subsysp->current_hits;
				}
			}

			// Stop the loop once we've found them all.
			if (subsys_count == n_subsystems) {
				break;
			}

			// retrieve the next subsystem
			GET_DATA(current_subsystem);
		}

		// if not all were found in the loop because of mismatched tables or other bugs, unpack the rest and ignore them to ensure aligned packets
		if (subsys_count != n_subsystems) {
			offset = pre_sub_offset + (n_subsystems * 2);
		}

		// recalculate all ship subsystems
		ship_recalc_subsys_strength(shipp);
	}

	// ---------------------------------------------------------------------------------------------------------------
	// AI & SUPPORT SHIP INFO
	// ---------------------------------------------------------------------------------------------------------------

	if ( oo_flags & OO_AI_NEW ) {
		// ai mode info
		ubyte umode;
		short submode;
		ushort target_signature;
		object *target_objp;

		// AI info
		GET_DATA( umode );
		GET_SHORT( submode );
		GET_USHORT( target_signature );		

		// primary weapon energy		
		float weapon_energy_pct;
		UNPACK_PERCENT(weapon_energy_pct);

		if( frame_comparison > interp_data->ai_comparison_frame ){
			if ( shipp->ai_index >= 0 ){
				// make sure to undo the wrap if it occurred during compression for unset ai mode.
				if (umode == 255) {
					Ai_info[shipp->ai_index].mode = -1; 
				}
				else {
					Ai_info[shipp->ai_index].mode = umode;
				}
				Ai_info[shipp->ai_index].submode = submode;		

				// set this guys target objnum
				target_objp = multi_get_network_object( target_signature );
				if ( target_objp == nullptr ){
					Ai_info[shipp->ai_index].target_objnum = -1;
				} else {
					Ai_info[shipp->ai_index].target_objnum = OBJ_INDEX(target_objp);
				}
			}

			shipp->weapon_energy = sip->max_weapon_reserve * weapon_energy_pct;
		}		
	}	

	if(oo_flags & OO_SUPPORT_SHIP){
		ushort dock_sig;
		int ai_mode, ai_submode;
		std::uint64_t ai_flags;

		// flag		
		GET_ULONG(ai_flags);
		GET_INT(ai_mode);
		GET_INT(ai_submode);
		GET_USHORT(dock_sig);		

		// valid ship?							
		if((shipp != nullptr) && (shipp->ai_index >= 0) && (shipp->ai_index < MAX_AI_INFO)){
			// bash ai info, this info does not get rebashed, because it is not as vital.
			Ai_info[shipp->ai_index].ai_flags.from_u64(ai_flags);
			Ai_info[shipp->ai_index].mode = ai_mode;
			Ai_info[shipp->ai_index].submode = ai_submode;

			object *objp = multi_get_network_object( dock_sig );
			if(objp != nullptr){
				Ai_info[shipp->ai_index].support_ship_objnum = OBJ_INDEX(objp);
			}
		}			
	} 

	// make sure the ab hack is reset before we read in new info
	Afterburn_hack = false;

	// afterburner info
	if ( (oo_flags & OO_AFTERBURNER_NEW) || Afterburn_hack ) {
		// maybe turn them on
		if(!(pobjp->phys_info.flags & PF_AFTERBURNER_ON)){
			afterburners_start(pobjp);
		}

		// make sure the ab hack is reset before we read in new info
		Afterburn_hack = false;
	} else {
		// maybe turn them off
		if(pobjp->phys_info.flags & PF_AFTERBURNER_ON){
			afterburners_stop(pobjp);
		}
	}

	// primary info (only clients care about this)
	if( !MULTIPLAYER_MASTER && (shipp != nullptr) ){
		// what bank
		if(oo_flags & OO_PRIMARY_BANK){
			shipp->weapons.current_primary_bank = 1;
		} else {
			shipp->weapons.current_primary_bank = 0;
		}

		// linked or not
        shipp->flags.remove(Ship::Ship_Flags::Primary_linked);
		if(oo_flags & OO_PRIMARY_LINKED){
			shipp->flags.set(Ship::Ship_Flags::Primary_linked);
		}

		// trigger down or not - server doesn't care about this. he'll get it from clients anyway		
		shipp->flags.remove(Ship::Ship_Flags::Trigger_down);
		if(oo_flags & OO_TRIGGER_DOWN){
			shipp->flags.set(Ship::Ship_Flags::Trigger_down);
		}		
	}
	
	// if we're the multiplayer server, set eye position and orient
	if(MULTIPLAYER_MASTER && (pl != nullptr) && (pobjp != nullptr)){
		pl->s_info.eye_pos = pobjp->pos;
		pl->s_info.eye_orient = pobjp->orient;
	} 		
	return offset;
}

// reset the timestamp appropriately for the passed in object
void multi_oo_reset_timestamp(net_player *pl, object *objp, int range, int in_cone)
{
	int stamp = 0;	

	// if this is the guy's target, 
	if((pl->s_info.target_objnum != -1) && (pl->s_info.target_objnum == OBJ_INDEX(objp))){
		stamp = Multi_oo_target_update_times[pl->p_info.options.obj_update_level];
	} else {
		// reset the timestamp appropriately
		if(in_cone){
			// base it upon range
			switch(range){
			case OO_NEAR:
				stamp = Multi_oo_front_near_update_times[pl->p_info.options.obj_update_level];
				break;

			case OO_MIDRANGE:
				stamp = Multi_oo_front_medium_update_times[pl->p_info.options.obj_update_level];
				break;

			case OO_FAR:
				stamp = Multi_oo_front_far_update_times[pl->p_info.options.obj_update_level];
				break;
			}
		} else {
			// base it upon range
			switch(range){
			case OO_NEAR:
				stamp = Multi_oo_rear_near_update_times[pl->p_info.options.obj_update_level];
				break;

			case OO_MIDRANGE:
				stamp = Multi_oo_rear_medium_update_times[pl->p_info.options.obj_update_level];
				break;

			case OO_FAR:
				stamp = Multi_oo_rear_far_update_times[pl->p_info.options.obj_update_level];
				break;
			}
		}						
	}

	// reset the timestamp for this object
	if(objp->type == OBJ_SHIP){
		Oo_info.player_frame_info[pl->player_id].last_sent[objp->net_signature].timestamp = timestamp(stamp);
	} 
}

// determine what needs to get sent for this player regarding the passed object, and when
int multi_oo_maybe_update(net_player *pl, object *obj, ubyte *data)
{
	ushort oo_flags = 0;
	int stamp;
	int player_index;
	vec3d player_eye;
	vec3d obj_dot;
	float eye_dot, dist;
	int in_cone;
	int range;
	ship *shipp;
	ship_info *sip;

	// if the timestamp has elapsed for this guy, send stuff
	player_index = NET_PLAYER_INDEX(pl);
	if(!(player_index >= 0) || !(player_index < MAX_PLAYERS)){
		return 0;
	}

	int net_sig_idx = obj->net_signature;

	// determine what the timestamp is for this object
	if(obj->type == OBJ_SHIP){
		stamp = Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].timestamp;
	} else {
		return 0;
	}

	// stamp hasn't popped yet
	if((stamp != -1) && !timestamp_elapsed_safe(stamp, OO_MAX_TIMESTAMP)){
		return 0;
	}
	
	// if we're supposed to update this guy	start figuring out what we should send.

	// get the ship pointer
	shipp = &Ships[obj->instance];

	// get ship info pointer
	sip = nullptr;
	if(shipp->ship_info_index >= 0){
		sip = &Ship_info[shipp->ship_info_index];
	}
	
	// check dot products		
	player_eye = pl->s_info.eye_orient.vec.fvec;
	vm_vec_sub(&obj_dot, &obj->pos, &pl->s_info.eye_pos);
	in_cone = 0;
	if (!(IS_VEC_NULL(&obj_dot))) {
		vm_vec_normalize(&obj_dot);
		eye_dot = vm_vec_dot(&obj_dot, &player_eye);		
		in_cone = (eye_dot >= OO_VIEW_CONE_DOT) ? 1 : 0;
	}
							
	// determine distance (near, medium, far)
	vm_vec_sub(&obj_dot, &obj->pos, &pl->s_info.eye_pos);
	dist = vm_vec_mag(&obj_dot);		
	if(dist < OO_NEAR_DIST){
		range = OO_NEAR;
	} else if(dist < OO_MIDRANGE_DIST){
		range = OO_MIDRANGE;
	} else {
		range = OO_FAR;
	}

	// reset the timestamp for the next update for this guy
	multi_oo_reset_timestamp(pl, obj, range, in_cone);
	
	// position should be almost constant, except for ships that aren't moving.
	if ( (Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].position != obj->pos) && (vm_vec_mag_quick(&obj->phys_info.vel) > 0.0f ) ) {
		oo_flags |= OO_POS_AND_ORIENT_NEW;
		// update the last position sent, will be done in each of the cases below.
		Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].position = obj->pos;
	}   // same with orientation
	else if (obj->phys_info.rotvel != vmd_zero_vector) {
		oo_flags |= OO_POS_AND_ORIENT_NEW;
	} 	// add info for a targeted object
	else if((pl->s_info.target_objnum != -1) && (OBJ_INDEX(obj) == pl->s_info.target_objnum)){
		oo_flags |= OO_POS_AND_ORIENT_NEW;
	}	// add info which is contingent upon being "in front"			
	else if(in_cone){
		oo_flags |= OO_POS_AND_ORIENT_NEW;
	}						
		


	// if its a small ship, add weapon link info
	// Cyborg17 - these don't take any extra space because they are part of the flags variable, so it's ok to send them with every packet.
	if((sip != nullptr) && (sip->is_fighter_bomber())){
		// primary bank 0 or 1
		if(shipp->weapons.current_primary_bank > 0){
			oo_flags |= OO_PRIMARY_BANK;
		}

		// linked or not
		if(shipp->flags[Ship::Ship_Flags::Primary_linked]){
			oo_flags |= OO_PRIMARY_LINKED;
		}

		// trigger down or not
		if(shipp->flags[Ship::Ship_Flags::Trigger_down]){
			oo_flags |= OO_TRIGGER_DOWN;
		}
	}	
		
	// maybe update hull
	if(Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].hull != obj->hull_strength){
		oo_flags |= (OO_HULL_NEW);
		Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].hull = obj->hull_strength;		
	}

	float temp_max = shield_get_max_quad(obj);
	bool all_max = true;
	// Client and server deal with ship death differently, so sending this info for dead ships can cause bugs
	if (!(shipp->is_dying_or_departing() || shipp->flags[Ship::Ship_Flags::Exploded])) {
		// maybe update shields, which are constantly repairing, and should be regularly updated, unless they are already spotless.
		for (auto quadrant : obj->shield_quadrant) {
			if (quadrant != temp_max) {
				all_max = false;
				break;
			}
		}
	}

	if (all_max) {
		// shields are currently perfect, were they perfect last time?
		if ( Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].perfect_shields_sent == false){
			// send the newly perfected shields
			oo_flags |= OO_SHIELDS_NEW;
		}
		// make sure to mark it as perfect for next time.
		Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].perfect_shields_sent = true;
	} // if they're not perfect, make sure they're marked as not perfect.
	else {
		Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].perfect_shields_sent = false;
		oo_flags |= OO_SHIELDS_NEW;
	}


	ai_info *aip = &Ai_info[shipp->ai_index];

	// check to see if the AI mode updated
	if ((Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].ai_mode != aip->mode) 
		|| (Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].ai_submode != aip->submode) 
		|| (Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].target_signature != aip->target_signature)) {

		// send, if so.
		oo_flags |= OO_AI_NEW;

		// set new values to check against later.
		Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].ai_mode = aip->mode;
		Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].ai_submode = aip->submode;
		Oo_info.player_frame_info[pl->player_id].last_sent[net_sig_idx].target_signature = aip->target_signature;
	}

	// finally, pack stuff only if we have to 	
	int packed = multi_oo_pack_data(pl, obj, oo_flags, data);	

	// bytes packed
	return packed;
}

// process all other objects for this player
void multi_oo_process_all(net_player *pl)
{
	ubyte data[MAX_PACKET_SIZE];
	ubyte data_add[MAX_PACKET_SIZE];
	ubyte stop;
	int add_size;	
	int packet_size = 0;
	int idx;
		
	object *moveup;	

	// if the player has an invalid objnum..
	if(pl->m_player->objnum < 0){
		return;
	}

	object *targ_obj;	

	// build the list of ships to check against
	multi_oo_build_ship_list(pl);

	// do nothing if he has no object targeted, or if he has a weapon targeted
	if((pl->s_info.target_objnum != -1) && (Objects[pl->s_info.target_objnum].type == OBJ_SHIP)){
		// build the header
		BUILD_HEADER(OBJECT_UPDATE);		
	
		// get a pointer to the object
		targ_obj = &Objects[pl->s_info.target_objnum];
	
		// run through the maybe_update function
		add_size = multi_oo_maybe_update(pl, targ_obj, data_add);

		// copy in any relevant data
		if(add_size){
			stop = 0xff;			
			multi_rate_add(NET_PLAYER_NUM(pl), "stp", 1);
			ADD_DATA(stop);

			memcpy(data + packet_size, data_add, add_size);
			packet_size += add_size;		
		}
	} else {
		// just build the header for the rest of the function
		BUILD_HEADER(OBJECT_UPDATE);		
	}
		
	idx = 0;
	// rely on logical-AND shortcut evaluation to prevent array out-of-bounds read of OO_ship_index[idx]
	while((idx < MAX_SHIPS) && (OO_ship_index[idx] >= 0)){
		// if this guy is over his datarate limit, do nothing
		if(multi_oo_rate_exceeded(pl)){
			nprintf(("Network","Capping client\n"));
			idx++;

			continue;
		}			

		// get the object
		moveup = &Objects[Ships[OO_ship_index[idx]].objnum];

		// maybe send some info		
		add_size = multi_oo_maybe_update(pl, moveup, data_add);

		// if this data is too much for the packet, send off what we currently have and start over
		if(packet_size + add_size > OO_MAX_SIZE){
			stop = 0x00;			
			multi_rate_add(NET_PLAYER_NUM(pl), "stp", 1);
			ADD_DATA(stop);
									
			multi_io_send(pl, data, packet_size);
			pl->s_info.rate_bytes += packet_size + UDP_HEADER_SIZE;

			packet_size = 0;
			BUILD_HEADER(OBJECT_UPDATE);			
		}

		if(add_size){
			stop = 0xff;			
			multi_rate_add(NET_PLAYER_NUM(pl), "stp", 1);
			ADD_DATA(stop);

			// copy in the data
			memcpy(data + packet_size,data_add,add_size);
			packet_size += add_size;
		}

		// next ship
		idx++;
	}

	// if we have anything more than 3 byte in the packet, send the last one off
	if(packet_size > 3){
		stop = 0x00;		
		multi_rate_add(NET_PLAYER_NUM(pl), "stp", 1);
		ADD_DATA(stop);
								
		multi_io_send(pl, data, packet_size);
		pl->s_info.rate_bytes += packet_size + UDP_HEADER_SIZE;
	}
}

// process all object update details for this frame
void multi_oo_process()
{
	int idx;	
	
	// process each player
	for(idx=0; idx<MAX_PLAYERS; idx++){
		if(MULTI_CONNECTED(Net_players[idx]) && !MULTI_STANDALONE(Net_players[idx]) && (Net_player != &Net_players[idx]) /*&& !MULTI_OBSERVER(Net_players[idx])*/ ){
			// now process the rest of the objects
			multi_oo_process_all(&Net_players[idx]);

			// do firing stuff for this player
			if((Net_players[idx].m_player != nullptr) && (Net_players[idx].m_player->objnum >= 0) && !(Net_players[idx].flags & NETINFO_FLAG_LIMBO) && !(Net_players[idx].flags & NETINFO_FLAG_RESPAWNING)){
				if((Objects[Net_players[idx].m_player->objnum].flags[Object::Object_Flags::Player_ship]) && !(Objects[Net_players[idx].m_player->objnum].flags[Object::Object_Flags::Should_be_dead])){
					obj_player_fire_stuff( &Objects[Net_players[idx].m_player->objnum], Net_players[idx].m_player->ci );
				}
			}
		}
	}
}

// process incoming object update data
void multi_oo_process_update(ubyte *data, header *hinfo)
{	
	ubyte stop;	
	int player_index;	
	int offset = HEADER_LENGTH;
	net_player *pl = nullptr;	

	// determine what player this came from 
	player_index = find_player_id(hinfo->id);
	if(player_index != -1){						
		pl = &Net_players[player_index];
	}
	// otherwise its a "regular" object update packet on a client from the server. use "myself" as the reference player
	else {						
		pl = Net_player;
	}

	GET_DATA(stop);
	
	while(stop == 0xff){
		// process the data
		offset += multi_oo_unpack_data(pl, data + offset);

		GET_DATA(stop);
	}
	PACKET_SET_SIZE();
}

// initialize all object update info (call whenever entering gameplay state)
void multi_init_oo_and_ship_tracker()
{
	// setup initial object update info	
	// Part 1: Get the non-repeating parts of the struct set.

	Oo_info.ref_timestamp = -1;
	Oo_info.most_recent_updated_net_signature = 0;
	Oo_info.most_recent_frame = 0;
	Oo_info.received_frametimes.clear();

	Oo_info.number_of_frames = 0;
	Oo_info.wrap_count = 0;
	Oo_info.larger_wrap_count = 0;
	Oo_info.cur_frame_index = 0;
	for (int i = 0; i < MAX_FRAMES_RECORDED; i++) {
		Oo_info.timestamps[i] = MAX_TIME; // This needs to be Max time (or at least some absurdly high number) for rollback to work correctly
	}

	Oo_info.rollback_mode = false;
	Oo_info.rollback_wobjp.clear();
	Oo_info.rollback_collide_list.clear();
	Oo_info.rollback_ships.clear();
	for (int i = 0; i < MAX_FRAMES_RECORDED; i++) {
		Oo_info.rollback_shots_to_be_fired[i].clear();
		Oo_info.rollback_shots_to_be_fired[i].reserve(20);
	}

	// Part 2: Init/Reset the repeating parts of the struct. 
	Oo_info.frame_info.clear();		
	Oo_info.player_frame_info.clear();
	Oo_info.interp.clear();

	Oo_info.frame_info.reserve(MAX_SHIPS); // Reserving up to a reasonable number of ships here should help optimize a little bit.
	Oo_info.player_frame_info.reserve(MAX_PLAYERS); // Reserve up to the max players
	Oo_info.interp.reserve(MAX_SHIPS);

	oo_ship_position_records temp_position_records;
	oo_netplayer_records temp_netplayer_records;

	for (int i = 0; i < MAX_FRAMES_RECORDED; i++) {
		temp_position_records.orientations[i] = vmd_identity_matrix;
		temp_position_records.positions[i] = vmd_zero_vector;
		temp_position_records.velocities[i] = vmd_zero_vector;
	}

	int cur = 0;
	oo_info_sent_to_players temp_sent_to_player;

	temp_sent_to_player.timestamp = timestamp(cur);
	temp_sent_to_player.position = vmd_zero_vector;
	temp_sent_to_player.hull = 0.0f;
	temp_sent_to_player.ai_mode = 0;
	temp_sent_to_player.ai_submode = -1;
	temp_sent_to_player.target_signature = 0;
	temp_sent_to_player.perfect_shields_sent = false;

	// See if *any* of the subsystems changed, so we have to allow for a variable number of subsystems within a variable number of ships.
	temp_sent_to_player.subsystems.reserve(MAX_MODEL_SUBSYSTEMS);
	temp_sent_to_player.subsystems.push_back(0.0f);

	temp_netplayer_records.last_sent.push_back(temp_sent_to_player);
	Oo_info.frame_info.push_back(temp_position_records);
	
	for (int i = 0; i < MAX_PLAYERS; i++) {
		Oo_info.player_frame_info.push_back(temp_netplayer_records);
	}

	// create a temporary struct and then push it to the vector
	oo_packet_and_interp_tracking temp_interp;

	temp_interp.cur_pack_pos_frame = -1;
	temp_interp.prev_pack_pos_frame = -1;

	temp_interp.client_simulation_mode = true;
	temp_interp.prev_packet_positionless = false;

	temp_interp.pos_time_delta = -1.0f;
	temp_interp.old_packet_position = vmd_zero_vector;
	temp_interp.new_packet_position = vmd_zero_vector;
	temp_interp.position_error = vmd_zero_vector;

	temp_interp.old_angles = vmd_zero_angles;
	temp_interp.new_angles = vmd_zero_angles;
	temp_interp.anticipated_angles_a = vmd_zero_angles;
	temp_interp.anticipated_angles_b = vmd_zero_angles;
	temp_interp.anticipated_angles_c = vmd_zero_angles;
	temp_interp.new_orientation = vmd_identity_matrix;

	temp_interp.new_velocity = vmd_zero_vector;
	temp_interp.anticipated_velocity1 = vmd_zero_vector;
	temp_interp.anticipated_velocity2 = vmd_zero_vector;
	temp_interp.anticipated_velocity3 = vmd_zero_vector;

	temp_interp.pos_spline = bez_spline();

	temp_interp.cur_pack_des_vel = vmd_zero_vector;
	temp_interp.cur_pack_local_des_vel = vmd_zero_vector;
	temp_interp.cur_pack_des_rot_vel = vmd_zero_vector;

	temp_interp.odd_wrap = false;
	temp_interp.most_recent_packet = -1;
	temp_interp.pos_comparison_frame = -1;
	temp_interp.prev_pos_comparison_frame = -1;
	temp_interp.hull_comparison_frame = -1;
	temp_interp.shields_comparison_frame = -1;

	temp_interp.subsystems_comparison_frame.reserve(MAX_MODEL_SUBSYSTEMS);
	temp_interp.subsystems_comparison_frame.push_back(-1);

	temp_interp.ai_comparison_frame = -1;
	Oo_info.interp.push_back(temp_interp);

	// reset datarate stamp now
	extern int OO_gran;
	for(int i=0; i<MAX_PLAYERS; i++){
		Net_players[i].s_info.rate_stamp = timestamp( (int)(1000.0f / (float)OO_gran) );
	}
}

// send control info for a client (which is basically a "reverse" object update)
void multi_oo_send_control_info()
{
	ubyte data[MAX_PACKET_SIZE], stop;
	ubyte data_add[MAX_PACKET_SIZE];
	ushort oo_flags;	
	int add_size;
	int packet_size = 0;

	// if I'm dying or my object type is not a ship, bail here
	if((Player_obj != nullptr) && (Player_ship->flags[Ship::Ship_Flags::Dying])){
		return;
	}	
	
	// build the header
	BUILD_HEADER(OBJECT_UPDATE);		

	// pos and orient always
	oo_flags = OO_POS_AND_ORIENT_NEW;		

	// pack the appropriate info into the data
	add_size = multi_oo_pack_data(Net_player, Player_obj, oo_flags, data_add);

	// copy in any relevant data
	if(add_size){
		stop = 0xff;		
		multi_rate_add(NET_PLAYER_NUM(Net_player), "stp", 1);
		
		ADD_DATA(stop);

		memcpy(data + packet_size, data_add, add_size);
		packet_size += add_size;		
	}

	// add the final stop byte
	stop = 0x0;	
	multi_rate_add(NET_PLAYER_NUM(Net_player), "stp", 1);
	ADD_DATA(stop);

	// send to the server
	if(Netgame.server != nullptr){								
		multi_io_send(Net_player, data, packet_size);
	}
}

// Sends a packet from the server to the client, syncing the player's position/orientation to the
// Server's. Allows for use of certain SEXPs in multiplayer.
void multi_oo_send_changed_object(object *changedobj)
{
	ubyte data[MAX_PACKET_SIZE], stop;
	ubyte data_add[MAX_PACKET_SIZE];
	ushort oo_flags;	
	int add_size;
	int packet_size = 0;
	int idx = 0;
#ifndef NDEBUG
	nprintf(("Network","Attempting to affect player object.\n"));
#endif
	for (; idx < MAX_PLAYERS; idx++)
	{
		if( changedobj == &(Objects[Net_players[idx].m_player->objnum]) ) {
			break;
		}
	}
#ifndef NDEBUG
	nprintf(("Network","Index for changed object found: [%d].\n",idx));
#endif
	if( idx >= MAX_PLAYERS ) {
		return;
	}
	// build the header
	BUILD_HEADER(OBJECT_UPDATE);		

	// pos and orient always
	oo_flags = (OO_POS_AND_ORIENT_NEW);

	// pack the appropriate info into the data
	add_size = multi_oo_pack_data(&Net_players[idx], changedobj, oo_flags, data_add);

	// copy in any relevant data
	if(add_size){
		stop = 0xff;		
		multi_rate_add(idx, "stp", 1);
		
		ADD_DATA(stop);

		memcpy(data + packet_size, data_add, add_size);
		packet_size += add_size;		
	}

	// add the final stop byte
	stop = 0x0;	
	multi_rate_add(idx, "stp", 1);
	ADD_DATA(stop);

	multi_io_send(&Net_players[idx], data, packet_size);
}


// updates all interpolation info for a specific ship
void multi_oo_maybe_update_interp_info(object* objp, vec3d* new_pos, angles* new_ori_angles, matrix* new_ori_mat, physics_info* new_phys_info, bool adjust_pos, bool newest_pos)
{
	Assert(objp != nullptr);

	if (objp == nullptr) {
		return;
	}

	int net_sig_idx = objp->net_signature;

	Assert(net_sig_idx >= 0);
	// store and replace interpolation info
	if (adjust_pos) {
		// if this is the newest position packet, update everything
		if (newest_pos) {
			Oo_info.interp[net_sig_idx].old_packet_position = Oo_info.interp[net_sig_idx].new_packet_position;
			Oo_info.interp[net_sig_idx].new_packet_position = *new_pos;

			Oo_info.interp[net_sig_idx].old_angles = Oo_info.interp[net_sig_idx].new_angles;
			Oo_info.interp[net_sig_idx].new_angles = *new_ori_angles;
		} // if this is the second newest, update that instead
		else {
			Oo_info.interp[net_sig_idx].old_packet_position = *new_pos;
			Oo_info.interp[net_sig_idx].old_angles = *new_ori_angles;
		}

		// now we'll update the interpolation splines if both points have been set.
		if ( Oo_info.interp[net_sig_idx].prev_pack_pos_frame > -1) {
			multi_oo_calc_interp_splines(objp, new_ori_mat, new_phys_info);
		}
	}
}

// display any oo info on the hud
void multi_oo_display()
{
#ifndef NDEBUG	
#endif
}


// ---------------------------------------------------------------------------------------------------
// DATARATE DEFINES/VARS
//

// low object update datarate limit
#define OO_LIMIT_LOW				1800
#define OO_LIMIT_MED				3400
#define OO_LIMIT_HIGH				100000000

// timestamp for sending control info (movement only - we'll send button info all the time)
#define OO_CIRATE					85					// 15x a second
int Multi_cirate_stamp			= -1;				// timestamp for waiting on control info time
int Multi_cirate_can_send		= 1;				// if we can send control info this frame

// global max rates
int OO_server_rate = -1;							// max _total_ bandwidth to send to all clients
int OO_client_rate = -1;							// max bandwidth to go to an individual client

// update timestamp for server datarate checking
#define RATE_UPDATE_TIME		1250				// in ms
int OO_server_rate_stamp = -1;

// bandwidth granularity
int OO_gran = 1;
DCF(oog, "Sets bandwidth granularity (Multiplayer)")
{
	if (dc_optional_string_either("help", "--help")) {
		dc_printf("Usage: oog <OO_gran>\n");
		dc_printf("Sets bandwidth granularity\n");
		return;
	}

	if (dc_optional_string_either("status", "--status") || dc_optional_string_either("?", "--?")) {
		dc_printf("Current Granularity is '%i' (default is 1)", OO_gran);
		return;
	}

	dc_stuff_int(&OO_gran);
	dc_printf("Ganularity set to %i", OO_gran);
}

// process datarate limiting stuff for the server
void multi_oo_server_process();

// process datarate limiting stuff for the client
void multi_oo_client_process();

// update the server datarate
void multi_oo_update_server_rate();


// ---------------------------------------------------------------------------------------------------
// DATARATE FUNCTIONS
//

// process all object update datarate details
void multi_oo_rate_process()
{
	// if I have no valid player, drop out here
	if(Net_player == nullptr){
		return;
	}

	// if we're not in mission, don't do anything
	if(!(Game_mode & GM_IN_MISSION)){
		return;
	}

	// if I'm the server of a game, process server stuff
	if(Net_player->flags & NETINFO_FLAG_AM_MASTER){
		multi_oo_server_process();
	}
	// otherwise process client-side stuff
	else {
		multi_oo_client_process();
	}
}

// process datarate limiting stuff for the server
void multi_oo_server_process()
{
	int idx;
	
	// go through all players
	for(idx=0;idx<MAX_PLAYERS;idx++){
		if(MULTI_CONNECTED(Net_players[idx]) && !MULTI_SERVER(Net_players[idx])){
			// if his timestamp is -1 or has expired, reset it and zero his rate byte count
			if((Net_players[idx].s_info.rate_stamp == -1) || timestamp_elapsed_safe(Net_players[idx].s_info.rate_stamp, OO_MAX_TIMESTAMP) || (abs(timestamp() - Net_players[idx].s_info.rate_stamp) >= (int)(1000.0f / (float)OO_gran)) ){
				Net_players[idx].s_info.rate_stamp = timestamp( (int)(1000.0f / (float)OO_gran) );
				Net_players[idx].s_info.rate_bytes = 0;
			}
		}
	}

	// determine if we should be updating the server datarate
	if((OO_server_rate_stamp == -1) || timestamp_elapsed_safe(OO_server_rate_stamp, OO_MAX_TIMESTAMP)){
		// reset the timestamp
		OO_server_rate_stamp = timestamp(RATE_UPDATE_TIME);

		// update the server datarate
		multi_oo_update_server_rate();

		// nprintf(("Network","UPDATING SERVER DATARATE\n"));
	}	
}

// process datarate limiting stuff for the client
void multi_oo_client_process()
{
	// if the timestamp is -1 or has elapsed, reset it
	if((Multi_cirate_stamp == -1) || timestamp_elapsed_safe(Multi_cirate_stamp, OO_CIRATE)){
		Multi_cirate_can_send = 1;
		Multi_cirate_stamp = timestamp(OO_CIRATE);
	}	
}


// datarate limiting system for server -------------------------------------

// initialize the rate limiting system for all players
void multi_oo_rate_init_all()
{
	int idx;

	// if I don't have a net_player, bail here
	if(Net_player == nullptr){
		return;
	}

	// if I'm the server of the game
	if(Net_player->flags & NETINFO_FLAG_AM_MASTER){	
		// go through all players
		for(idx=0;idx<MAX_PLAYERS;idx++){
			if(MULTI_CONNECTED(Net_players[idx])){
				multi_oo_rate_init(&Net_players[idx]);
			}
		}

		OO_server_rate_stamp = -1;
	}
	// if i'm the client, initialize my control info datarate stuff
	else {
		Multi_cirate_stamp = -1;
		Multi_cirate_can_send = 1;
	}
}

// initialize the rate limiting for the passed in player
void multi_oo_rate_init(net_player *pl)
{
	// reinitialize his datarate timestamp
	pl->s_info.rate_stamp = -1;
	pl->s_info.rate_bytes = 0;
}

// if the given net-player has exceeded his datarate limit
int multi_oo_rate_exceeded(net_player *pl)
{
	int rate_compare;
		
	// check against the guy's object update level
	switch(pl->p_info.options.obj_update_level){
	// low update level
	case OBJ_UPDATE_LOW:
		// the low object update limit
		rate_compare = OO_LIMIT_LOW;
		break;

	// medium update level
	case OBJ_UPDATE_MEDIUM:		
		// the low object update limit
		rate_compare = OO_LIMIT_MED;
		break;

	// high update level - super high datarate (no capping, just intelligent updating)
	case OBJ_UPDATE_HIGH:
		rate_compare = OO_LIMIT_HIGH;
		break;

	// LAN - no rate max
	case OBJ_UPDATE_LAN:
		return 0;

	// default level
	default:
		Int3();
		rate_compare = OO_LIMIT_LOW;
		break;
	}

	// if the server global rate PER CLIENT (OO_client_rate) is actually lower
	if(OO_client_rate < rate_compare){
		rate_compare = OO_client_rate;
	}

	// compare his bytes sent against the allowable amount
	if(pl->s_info.rate_bytes >= rate_compare){
		return 1;
	}

	// we're allowed to send
	return 0;
}

// if it is ok for me to send a control info (will be ~N times a second)
int multi_oo_cirate_can_send()
{
	// if we're allowed to send
	if(Multi_cirate_can_send){
		Multi_cirate_can_send = 0;
		return 1;
	} 
	
	return 0;		
}

// dynamically update the server capped bandwidth rate
void multi_oo_update_server_rate()
{	
	int num_connections;	
	
	// bail conditions
	if((Net_player == nullptr) || !(Net_player->flags & NETINFO_FLAG_AM_MASTER)){
		return;
	}

	// get the # of connections
	num_connections = multi_num_connections();
	if(!(Game_mode & GM_STANDALONE_SERVER)){
		num_connections--;
	}
	// make sure we always pretend there's at least one guy available
	if(num_connections <= 0){
		num_connections = 1;
	}
		
	// set the data rate	
	switch(Net_player->p_info.options.obj_update_level){
	// LAN update level
	case OBJ_UPDATE_LAN:
		// set to something super big so we don't limit anything
		OO_server_rate = 500000000;
		break;

	// high update level
	case OBJ_UPDATE_HIGH:
		// set to 0 so we don't limit anything
		OO_server_rate = Multi_options_g.datarate_cap;
		break;

	// medium update level
	case OBJ_UPDATE_MEDIUM:
		// set the rate to be "medium" update level
		OO_server_rate = OO_LIMIT_MED;
		break;

	// low update level 
	case OBJ_UPDATE_LOW:
		// set the rate to be the "low" update level
		OO_server_rate = OO_LIMIT_LOW;
		break;

	default:
		Int3();
		return;
	}	

	// set the individual client level
	OO_client_rate = (int)(((float)OO_server_rate / (float)OO_gran) / (float)num_connections);
}

// is this object one which needs to go through the interpolation
int multi_oo_is_interp_object(object *objp)
{	
	// if not multiplayer, skip it
	if(!(Game_mode & GM_MULTIPLAYER)){
		return 0;
	}

	// if its not a ship, skip it
	if(objp->type != OBJ_SHIP){
		return 0;
	}

	// other bogus cases
	if((objp->instance < 0) || (objp->instance >= MAX_SHIPS)){
		return 0;
	}

	// if I'm a client and this is not me, I need to interp it
	if(!MULTIPLAYER_MASTER){
		if(objp != Player_obj){
			return 1;
		} else {
			return 0;
		}
	}

	// servers only interpolate other player ships
	if(!(objp->flags[Object::Object_Flags::Player_ship])){
		return 0;
	}

	// here we know its a player ship - is it mine?
	if(objp == Player_obj){
		return 0;
	}

	// interp it
	return 1;
}

// Calculate the physics info in the current frame for a ship based on the data calculated in multi_oo_calc_interp_splines
void multi_oo_interp(object* objp)
{
	// make sure its a valid ship
	Assert(Game_mode & GM_MULTIPLAYER);
	Assert(objp->net_signature <= STANDALONE_SHIP_SIG);

	if (objp->type != OBJ_SHIP || objp->net_signature == STANDALONE_SHIP_SIG) {
		return;
	}
	if ((objp->instance < 0) || (objp->instance >= MAX_SHIPS)) {
		return;
	}
	// Now that we know we have a valid ship, do stream weapon firing for this ship before we do anything else that makes us abort
	Assert(objp != Player_obj);
	if (objp != Player_obj) {
		if (MULTIPLAYER_CLIENT) {
			ship_fire_primary(objp, 1, 0);
		}
	}

	vec3d store_old_pos = objp->pos;
	ushort net_sig_idx = objp->net_signature;

	oo_packet_and_interp_tracking* interp_data = &Oo_info.interp[net_sig_idx];

	if (interp_data == nullptr) {
		return;
	}

	float packet_delta = interp_data->pos_time_delta;

	// if this ship doesn't have enough data points yet or somehow else invalid, pretend it's a normal ship and skip it.
	if (interp_data->prev_pack_pos_frame == -1) {
		physics_sim_vel(&objp->pos, &objp->phys_info, flFrametime, &objp->orient);
		physics_sim_rot(&objp->orient, &objp->phys_info, flFrametime);

	} // once there are enough data points, we begin interpolating.
	else {
		// figure out how much time has passed
		int temp_numerator = timestamp() - interp_data->pos_timestamp;

		// add the ~1/2 of ping to keep the players in better sync
		if (MULTIPLAYER_MASTER) {
			int player_id = multi_find_player_by_net_signature(net_sig_idx);
			temp_numerator += Net_players[player_id].s_info.ping.ping_avg / 2;
		}
		else {
			temp_numerator += Net_players[0].s_info.ping.ping_avg / 2;
		}

		// divide in order to change to flFrametime format
		float time_elapsed = i2fl(temp_numerator) / TIMESTAMP_FREQUENCY;

		// Cyborg17 - Here's the new timing calculation: we subtract the last packet's arrival time 
		// from the current timestamp to see how long it's been and add 1/2 of the current ping, and 
		// then we divide by the difference in time between the last two packets.
		// We add one because we do not want to go back into data from before the current packet was received.
		float time_factor = (time_elapsed / packet_delta) + 1.0f;

		// if there was no movement, bash position and velocity, rotation is handled after.
		if (interp_data->prev_packet_positionless) {
			interp_data->client_simulation_mode = false;
			objp->pos = interp_data->new_packet_position;
			objp->orient = interp_data->new_orientation;
			objp->phys_info.vel = vmd_zero_vector;
		} // Overshoting in this frame or some edge case bug. Just sim the ship from the known values.
		else if (time_factor > 4.0f || time_factor < 1.0f) {
			// if transitioning to uninterpolated movement, we need to jump to the end of the simulated points 
			// and then simulate forward some if there's extra time. 
			float regular_sim_delta;

			if (!interp_data->client_simulation_mode) {
				interp_data->client_simulation_mode = true;
				objp->pos = interp_data->new_packet_position;
				objp->orient = interp_data->new_orientation;
				regular_sim_delta = time_elapsed - (2 * packet_delta);
			}
			else {
				regular_sim_delta = flFrametime;
			}
			// Continue simulating if we have time that we need to simulate and exclude fake values.
			if (regular_sim_delta > 0.001f && regular_sim_delta < 0.500f) {
				// make sure to bash desired velocity and rotational velocity in this case.
				objp->phys_info.desired_vel = interp_data->cur_pack_des_vel;
				objp->phys_info.desired_rotvel = interp_data->cur_pack_des_rot_vel;
				physics_sim_vel(&objp->pos, &objp->phys_info, regular_sim_delta, &objp->orient);
				physics_sim_rot(&objp->orient, &objp->phys_info, regular_sim_delta);
			}
		} // valid time factors.
		else {
			interp_data->client_simulation_mode = false;

			// Cyborg17 - we are no longer blending two interpolation curves.  I'm not sure *how*, but they were somehow making it look 
			// less erratic when the timing was broken in the first place. 
			float u = (time_factor) / 4.0f;
			vec3d interp_point;
			interp_data->pos_spline.bez_get_point(&interp_point, u);
			// now to smooth out the error that the client caused during the last round of interpolation.
			if ((time_factor < 2.0f) && (time_factor > 0.0f) && (vm_vec_mag_squared(&interp_data->position_error) > 0.0f)) {
				vec3d remove_error_vector;
				// .5 and 2 are multiplicative inverses. We want all the error gone at time_factor 2.
				float temp_error_factor = 1 - (time_factor * 0.5f);
				vm_vec_copy_scale(&remove_error_vector, &interp_data->position_error, temp_error_factor);
				vm_vec_add2(&interp_point, &remove_error_vector);
			}

			// set the new position.
			objp->pos = interp_point;

			// Now rotational interpolation
			// exactly on the middle point, save some time and just put the ship on that orientation.
			if (time_factor == 2.0f) {
				vm_angles_2_matrix(&objp->orient, &interp_data->anticipated_angles_a);
				objp->phys_info.vel = interp_data->anticipated_velocity1;
			} // Same for being exactly on the end point

			else if (time_factor == 3.0f) {
				vm_angles_2_matrix(&objp->orient, &interp_data->anticipated_angles_b);
				objp->phys_info.vel = interp_data->anticipated_velocity2;
			}
			else if (time_factor == 4.0f) {
				vm_angles_2_matrix(&objp->orient, &interp_data->anticipated_angles_c);
				objp->phys_info.vel = interp_data->anticipated_velocity3;

			} // in case we have to do our interpolation. We cannot do anything if it's less than 1 because those are actually old values that *should* never happen.
			else if (time_factor > 1.0f) {
				angles temp_angles, old_angles, new_angles;
				vec3d old_velocity, new_velocity;
				// Between packet and first interpolated angles
				if (time_factor < 2.0f) {

					old_angles = interp_data->new_angles;
					new_angles = interp_data->anticipated_angles_a;
					old_velocity = interp_data->new_velocity;
					new_velocity = interp_data->anticipated_velocity1;

					time_factor--;
				} // between interpolated angles a and b
				else if (time_factor < 3.0f) {

					old_angles = interp_data->anticipated_angles_a;
					new_angles = interp_data->anticipated_angles_b;
					old_velocity = interp_data->anticipated_velocity1;
					new_velocity = interp_data->anticipated_velocity2;

					time_factor -= 2.0f;
				} // between interpolated angles b and c
				else if (time_factor < 4.0f) {

					old_angles = interp_data->anticipated_angles_b;
					new_angles = interp_data->anticipated_angles_c;
					old_velocity = interp_data->anticipated_velocity2;
					new_velocity = interp_data->anticipated_velocity3;

					time_factor -= 3.0f;
				}

				vm_interpolate_angles_quick(&temp_angles, &old_angles, &new_angles, time_factor);
				vm_angles_2_matrix(&objp->orient, &temp_angles);
				vm_vec_scale(&new_velocity, time_factor);
				vm_vec_scale(&old_velocity, 1 - time_factor);
				vm_vec_add(&objp->phys_info.vel, &new_velocity, &old_velocity);
			}
		}
	}

	// this gets rid of ships shaking in place, but once the velocity has started, it's a free for all.
	vec3d temp_rubberband_test, local_displacement, temp_local_vel, local_rubberband_correction, global_rubberband_correction;
	vm_vec_sub(&temp_rubberband_test, &objp->pos, &store_old_pos);

	vm_vec_rotate(&local_displacement, &temp_rubberband_test, &objp->orient);
	vm_vec_rotate(&temp_local_vel, &objp->phys_info.vel, &objp->orient);

	local_rubberband_correction = vmd_zero_vector;

	// a difference in sign means something just rubberbanded.
	if ( ((local_displacement.xyz.x < 0.0f) && (temp_local_vel.xyz.x > 0.0f)) || ((local_displacement.xyz.x > 0.0f) && (temp_local_vel.xyz.x < 0.0f)) ) {
		local_rubberband_correction.xyz.x = -local_displacement.xyz.x;
	}
	if ( ((local_displacement.xyz.y < 0.0f) && (temp_local_vel.xyz.y > 0.0f)) || ((local_displacement.xyz.y > 0.0f) && (temp_local_vel.xyz.y < 0.0f)) ) {
		local_rubberband_correction.xyz.y = -local_displacement.xyz.y;
	}
	if ( ((local_displacement.xyz.z < 0.0f) && (temp_local_vel.xyz.z > 0.0f)) || ((local_displacement.xyz.z > 0.0f) && (temp_local_vel.xyz.z < 0.0f)) ) {
		local_rubberband_correction.xyz.z = -local_displacement.xyz.z;
	}

	vm_vec_unrotate(&global_rubberband_correction, &local_rubberband_correction, &objp->orient);

	vm_vec_add2(&objp->pos, &global_rubberband_correction);

	// duplicate the rest of the physics engine's calls here to make the simulation more exact.
	objp->phys_info.speed = vm_vec_mag(&objp->phys_info.vel);
	objp->phys_info.fspeed = vm_vec_dot(&objp->orient.vec.fvec, &objp->phys_info.vel);
}

// Establish the values that FSO will later interpolate with based on packet data.
void multi_oo_calc_interp_splines(object* objp, matrix *new_orient, physics_info *new_phys_info)
{
	Assert(objp != nullptr);

	if (objp == nullptr) {
		return;
	}

	ushort net_sig_idx = objp->net_signature;
	
	// find the float time version of how much time has passed
	float delta = multi_oo_calc_pos_time_difference(net_sig_idx);
	// if an error or invalid value, use the local timestamps instead of those received. Should be rare.
	if (delta <= 0.0f) {
		delta = float(timestamp() - Oo_info.received_frametimes[Oo_info.interp[net_sig_idx].pos_timestamp]) / 1000.0f;
	}

	Oo_info.interp[net_sig_idx].pos_time_delta = delta;

	// create the vector which will hold the spline
	vec3d point1, point2, point3;
	vec3d *pts[3] = {&point1, &point2, &point3};
	matrix matrix_copy;
	physics_info physics_copy;

	// 3 point curve, 5 time deltas. Point1 is 1 delta before (prev packet positon), point2 is 1 delta ahead, 
	// point3 is 3 deltas ahead. This is to allow for the time period to be even between the three interpolation points,
	// while also doing interpolation for a longer period of time.

	point1 = Oo_info.interp[net_sig_idx].old_packet_position;
	point2 = Oo_info.interp[net_sig_idx].new_packet_position; 
	matrix_copy = *new_orient;
	physics_copy = *new_phys_info;
	Oo_info.interp[net_sig_idx].new_velocity = physics_copy.vel;

	angles angle_equivalent;

	// ------------ CALCUATION 1 ------------ //
	// Calculate up to 1 delta past the packet, then store point2 as point two in the bezier, and the angles.
	physics_sim(&point2, &matrix_copy, &physics_copy, delta);			
	vm_extract_angles_matrix_alternate(&angle_equivalent, &matrix_copy);

	// Adjust desired velocity, because it's in world coordinates, and it doesn't make sense to reuse the same one four more times.
	vm_vec_unrotate(&physics_copy.desired_vel, &Oo_info.interp[net_sig_idx].cur_pack_local_des_vel, &matrix_copy);

	Oo_info.interp[net_sig_idx].anticipated_angles_a = angle_equivalent;

	// Since point2 is calculated, pass it off to point3 to finish off the calculations, which is 2 more deltas later
	point3 = point2;

	// ------------ CALCUATION 2 ------------ //

	Oo_info.interp[net_sig_idx].anticipated_velocity1 = physics_copy.vel;
	
	// Only the angles get stored here.
	physics_sim(&point3, &matrix_copy, &physics_copy, delta);			
	vm_extract_angles_matrix_alternate(&angle_equivalent, &matrix_copy);

	// Readjust desired velocity, assuming that you would have the same throttle.
	vm_vec_unrotate(&physics_copy.desired_vel, &Oo_info.interp[net_sig_idx].cur_pack_local_des_vel, &matrix_copy);

	Oo_info.interp[net_sig_idx].anticipated_angles_b = angle_equivalent;
	Oo_info.interp[net_sig_idx].anticipated_velocity2 = physics_copy.vel;

	// ------------ CALCUATION 3 ------------ //
	physics_sim(&point3, &matrix_copy, &physics_copy, delta);			
	vm_extract_angles_matrix_alternate(&angle_equivalent, &matrix_copy);

	// Readjust desired velocity, assuming that you would have the same throttle.
	vm_vec_unrotate(&physics_copy.desired_vel, &Oo_info.interp[net_sig_idx].cur_pack_local_des_vel, &matrix_copy);

	Oo_info.interp[net_sig_idx].anticipated_angles_c = angle_equivalent;
	Oo_info.interp[net_sig_idx].anticipated_velocity3 = physics_copy.vel;

	// Set the points to the bezier
	Oo_info.interp[net_sig_idx].pos_spline.bez_set_points(3, pts);
}

// Calculates how much time has gone by between the two most recent frames 
float multi_oo_calc_pos_time_difference(int net_sig_idx) 
{
	int old_frame = Oo_info.interp[net_sig_idx].prev_pack_pos_frame;
	int new_frame = Oo_info.interp[net_sig_idx].cur_pack_pos_frame;
	
	// make sure we have enough packets so far. (old_frame is updated after new_frame)
	if (old_frame == -1 ) {
		return -1.0f;
	}
	
	if (old_frame == new_frame) {
		nprintf(("network","multi_oo_calc_pos_time_difference somehow showed the same frame for old and new frame.\n"));
	}
	
	if (old_frame == new_frame) {
		return -1.0f;
	}

	float temp_sum = 0.0f;
	int frame_time = Oo_info.received_frametimes[old_frame];

	// add up the frametimes in between, not including the old_frame's frametime because that was the amount of time from
	// the frame before that to the old_frame.
	for (int i = old_frame + 1; i <= new_frame; i++) {
		// a zero value means we haven't received that frame yet.
		if (Oo_info.received_frametimes[i] > 0) {
			frame_time = Oo_info.received_frametimes[i];
		}
		temp_sum += frame_time;
	}
	temp_sum /= 1000.0f; // convert from timestamp to float frame time

	return temp_sum;
}

bool display_oo_bez = false;
DCF(bez, "Toggles rendering of player ship trajectory interpolation splines (Multiplayer) *disabled*")
{
	if (dc_optional_string_either("status", "--status") || dc_optional_string_either("?", "--?")) {
		dc_printf("Rendering of interpolation splines is '%s'", display_oo_bez ? "ON" : "OFF");
		return;
	}

	display_oo_bez = !display_oo_bez;

	dc_printf("%showing positional interp spline", display_oo_bez ? "S" : "Not s");
}

void oo_display()
{
/*	int idx;	


	gr_set_color_fast(&Color_bright);

	for(idx=0; idx<MAX_SHIPS; idx++){		
		// invalid ship
		if(Ships[idx].objnum < 0){
			continue;
		}

		// time between updates
		if( (oo_arrive_time_count[idx] == 5) && (idx != (Player_ship - Ships)) ){
			gr_printf(20, 40, "avg time between updates : %f", oo_arrive_time_avg_diff[idx]);			
		}			
		
		// interpolation splines
		if( (oo_interp_count[idx] == 2) && (display_oo_bez) ){
			oo_interp_splines[idx][0].bez_render(10, &Color_bright_red);			// bad path
			oo_interp_splines[idx][1].bez_render(10, &Color_bright_green);		// good path
		}
	}
	*/
}
