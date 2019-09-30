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

// One record per ship.
struct multi_oo_server_frame_record {

	int initial_frame;												// to keep track of when each ship was added to the struct, as compared to total frames.           
	int death_or_depart_frame;										// to keep track of when a ship's info should no longer be trusted for interpolation

	vec3d positions[MAX_FRAMES_RECORDED];							// The recorded ship positions 
	matrix orientations[MAX_FRAMES_RECORDED];						// The recorded ship orientations 
	vec3d rotational_velocities[MAX_FRAMES_RECORDED];				// The recorded rotational velocities, used in interpolation.

};

struct oo_info_sent_to_player {
	ushort update_frame;
	vec3d position;			// If they are stationary, there's no need to update their position.
	vec3d rot_vel;			// If their rotation is not accelerating, no need to update their rotation.
	float hull;
	short ai_mode;
	short ai_submode;
	ushort target_signature;

	bool perfect_shields_sent;		// if the last packet sent perfect shields, we may not need to send them again.
	SCP_vector<float> subsystems;	// See if *any* of the subsystems changed, so we have to allow for a variable number of subsystems within a variable number of ships.
};

struct oo_last_sent_record{
	SCP_vector<oo_info_sent_to_player> players; // Subcategory of which player did I send this info to?  Should correspond to net_player indeces.
};

// Struct for keeping track of *all* server record info.
// The frame record wraps to keep from using too much memory or CPU 
struct multi_oo_frame_tracker_info {
	int number_of_frames;											// how many frames have we gone through, total.
	ubyte wrap_count;												// how many times have we wrapped?  Just the smaller type.
	ushort cur_idx;													// the frame that the server is currently keeping track of.

	ubyte frame_timestamp_count;									// counts the number of times the timestamp has been sent to the client. 
	int timestamps[MAX_FRAMES_RECORDED];						// The timestamp for the given frame
	SCP_vector<multi_oo_server_frame_record> frame_info;			// Actually keeps track of ship physics info on the server.  Uses net_signature as its index.
	
	SCP_vector<oo_last_sent_record> ship_last_sent;						// One entry for each ship, what did we last send?  Index is net_signature.
};

multi_oo_frame_tracker_info Svr_frames;				// All online object information for the server

// ---------------------------------------------------------------------------------------------------
// Tracking Received info and timing

typedef struct interp_message_tracking {
	// arrival timing 
	int cur_pack_pos_frame;				// the last position packet arrival frame
	int prev_pack_pos_frame;			// the prev position packet arrival frame

	int cur_pack_ori_frame;				// the last orientation arrival frame
	int prev_pack_ori_frame;			// the prev orientation arrival frame

	// interpolation 
	ubyte interp_count;					// how many interpolation points do we have
	vec3d old_points;					// The last packet's pos
	vec3d new_points;					// The current packet's pos
	angles old_angles;					// The last packet's orientation (in angles)
	angles new_angles;					// The current packet's orientation (in angles)
	bez_spline splines[2];				// The curves that interpolation follows

	// other recorded values
	vec3d new_rot_vel;

	// part of the interpolation for slowly removing the error in position and orientation
	vec3d position_error;				// Position error that is gradually removed 
	angles orientation_error;			// Orientation error that is gradually removed.

	// Frame numbers that helps us figure out if we should ignore new information coming from the server because
	// we already received a newer packet than this one.
	int most_recent_packet;
	int minimum_packet;
	int rot_vel_frame;
	int hull_frame;
	int shields_frame;
	SCP_vector<int> subsystems_frame;
	int ai_frame;

} interp_message_tracking;

// our struct for keeping track of incoming object update information.
typedef struct oo_general{

	// info that helps us figure out what is the best reference object available.
	// We go by what is the most recent packet received, and then by distance.
	int ref_timestamp;
	ushort most_recent_updated_ship_index;
	ushort most_recent_frame;
	float distance_to_most_recent;

	// The previously received frames' timestamps.  One entry for *every* frame, received or not, up to the last received frame.
	SCP_vector<ushort> received_timestamps;

	// Client tracking of interp info and last received data.  One entry for every ship.
	SCP_vector<interp_message_tracking> interp;



} oo_general;

oo_general Oo_general_info;

// flags
bool Afterburn_hack = false;			// HACK!!!


void multi_oo_calc_interp_splines(int ship_index, vec3d *cur_pos, physics_info *cur_phys_info, vec3d *new_pos, matrix *new_orient, physics_info *new_phys_info);

// how much data we're willing to put into a given oo packet
#define OO_MAX_SIZE					480

// tolerance for bashing position
#define OO_POS_UPDATE_TOLERANCE	150.0f

// new improved - more compacted info type
#define OO_POS_NEW					(1<<0)		// To update position
#define OO_ORIENT_NEW				(1<<1)		// To update orientation, currently position and orientation are set at the same time.
#define OO_ROT_VEL_NEW				(1<<2)		// Also, update rotational velocity only when necessary
#define OO_HULL_NEW					(1<<3)		// To Update Hull
#define OO_SHIELDS_NEW				(1<<4)		// To Update Shields.
#define OO_AFTERBURNER_NEW			(1<<5)		// Flag for Afterburner hack
#define OO_SUBSYSTEMS_NEW			(1<<6)		// Send Subsystem Info
#define OO_PRIMARY_BANK				(1<<7)		// if this is set, fighter has selected bank one
#define OO_PRIMARY_LINKED			(1<<8)		// if this is set, banks are linked
#define OO_TRIGGER_DOWN				(1<<9)		// if this is set, trigger is DOWN
#define OO_SUPPORT_SHIP				(1<<10)		// Send extra info for the support ship.
#define OO_AI_NEW					(1<<11)		// Send updated AI Info
#define OO_TIMESTAMP				(1<<12)		// Send the current Timestamp.

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

// for making the frame_record_info wrapping predictable. 182 is the highest that the client can still handle. (65536/360) 
#define MAX_SERVER_TRACKER_SMALL_WRAPS 182
#define SERVER_TRACKER_LARGE_WRAP_TOTAL (MAX_SERVER_TRACKER_SMALL_WRAPS * MAX_FRAMES_RECORDED)

// timestamp values for object update times based on client's update level.
// Cyborg17 - This is the one update number we should adjust, because it's the player's target.
int Multi_oo_target_update_times[MAX_OBJ_UPDATE_LEVELS] = 
{
	50, 				// 30x a second 
	50, 				// 30x a second
	20,				// 60x a second
	20,				// 60x a second
};

// for near ships
int Multi_oo_front_near_update_times[MAX_OBJ_UPDATE_LEVELS] =
{
	150,				// low update
	100,				// medium update
	66,				// high update
	66,
};

// for medium ships
int Multi_oo_front_medium_update_times[MAX_OBJ_UPDATE_LEVELS] =
{
	250,				// low update
	180, 				// medium update
	120,				// high update
	66,
};

// for far ships
int Multi_oo_front_far_update_times[MAX_OBJ_UPDATE_LEVELS] =
{
	750,				// low update
	350, 				// medium update
	150, 				// high update
	66,
};

// for near ships
int Multi_oo_rear_near_update_times[MAX_OBJ_UPDATE_LEVELS] = 
{
	300,				// low update
	200,				// medium update
	100,				// high update
	66,
};

// for medium ships
int Multi_oo_rear_medium_update_times[MAX_OBJ_UPDATE_LEVELS] = 
{
	800,				// low update
	600,				// medium update
	300,				// high update
	66,
};

// for far ships
int Multi_oo_rear_far_update_times[MAX_OBJ_UPDATE_LEVELS] = 
{
	2500, 			// low update
	1500,				// medium update
	400,				// high update
	66,
};

// ship index list for possibly sorting ships based upon distance, etc
short OO_ship_index[MAX_SHIPS];

// Cyborg17 - I'm leaving this system in place, just in case. Just know it needs cleanup in keycontrol.cpp before it can be used.
int OO_update_index = -1;							// The player index that allows us to look up multi rate through the debug
													//  console and display it on the hud.


// ---------------------------------------------------------------------------------------------------
// POSITION AND ORIENTATION RECORDING
// if it breaks, find Cyborg17 so you can yell at him

// This section is almost all server side
// We record positions and orientations in the multi_ship_frames struct so that we can intialize a weapon in the same relative 
// circumstances as on the client.  I was directly in front, 600 meters away when I fired?  Well, now the client will tell the 
// server that and the server will rewind part of its simulation to recreate that shot.
// ---------------------------------------------------------------------------------------------------

// Server Only - Add a new object on *ship creation* to the tracking struct
void multi_ship_record_add_ship_server(int obj_num, bool in_mission)
{
	mprintf(("Ship added to server record struct.\n"));
	Assertion(MULTIPLAYER_MASTER, "Non-server accessed a server only function. Please report!!");

	if (!MULTIPLAYER_MASTER) {
		return;
	}

	object* objp = &Objects[obj_num];

	// check for a ship that will enter the mission later on and has not yet had its net_signature set.
	if (objp->net_signature == 0) {
		return;
	}

	// our target size is the number of ships in the vector plus one because net_signatures start at 1 and size gives the number of elements, and this should be a new element.
	int target_size = (int)Svr_frames.frame_info.size() + 1;

	mprintf(("The object adder says that the target size is %d and the incoming net_signature is %d\n", target_size, objp->net_signature));	

	// if we're right where we should be.
	if (objp->net_signature == target_size) {
		mprintf(("Server tracker believes object signature is right on target.\n"));
		Svr_frames.frame_info.push_back(*new(multi_oo_server_frame_record));		

	}	// if the storage already exists.
	else if (objp->net_signature < (target_size)) {
		mprintf(("Server tracker belives that the object already had storage available. No storage added.\n"));

	}	// if not, create the storage.
	else if (objp->net_signature > (target_size)) {
		mprintf(("Server tracker belives that the object is way past the storage available..\n"));
		Svr_frames.frame_info.push_back(*new(multi_oo_server_frame_record));

	} // and if I missed something.
	else {
		mprintf(("Some sort of logic was misssed in the object adder function.\n"));
	}
	// Have this ready for later when we're sure that it works correctly at least most of the time.
//	Assertion(objp->net_signature == target_size, "New entry into the ship traker struct does not equal the index that should belong to it.\nNet_signature: %d and target_size %d\n", objp->net_signature, target_size);

// net_signature is 1 indexed, so we have to adjust to correctly access info the vector.
	int net_sig_idx = objp->net_signature - 1;

	// store info from the obj struct if it's already in the mission, otherwise, let the physics update call take care of it when the first frame starts.
	if (in_mission) {
		// only add positional info if they are in the mission.
		Svr_frames.frame_info[net_sig_idx].initial_frame = Svr_frames.number_of_frames;
		Svr_frames.frame_info[net_sig_idx].positions[Svr_frames.cur_idx] = objp->pos;
		Svr_frames.frame_info[net_sig_idx].orientations[Svr_frames.cur_idx] = objp->orient;
		Svr_frames.frame_info[net_sig_idx].rotational_velocities[Svr_frames.cur_idx] = objp->phys_info.rotvel;
		Svr_frames.frame_info[net_sig_idx].death_or_depart_frame = -1;
	}
	else {
		Svr_frames.frame_info[net_sig_idx].initial_frame = -1;
		Svr_frames.frame_info[net_sig_idx].death_or_depart_frame = -1;
	}
}

// TODO: Find the best place for this.
void multi_ship_record_mark_as_dead_or_departed(int net_signature)
{
	mprintf(("I'm the last one to run!!! 1\n"));
	if (MULTIPLAYER_MASTER && (Game_mode & GM_MULTIPLAYER)){
		Svr_frames.frame_info[net_signature - 1].death_or_depart_frame = Svr_frames.number_of_frames;
	}
}

// Update the tracking struct whenever the object is updated in-game
// Server Only
void multi_ship_record_update_all() 
{
	Assertion(MULTIPLAYER_MASTER, "Non-server accessed a server only function. Please report!!");

	if (!MULTIPLAYER_MASTER) {
		return;
	}

	int net_sig_idx;
	object* objp;

	for (ship & cur_ship : Ships) {
		// net_signature is 1 indexed, so we have to subtract to correctly access info in the vector.
		 objp = &Objects[cur_ship.objnum];

		 if (objp == nullptr) {
			 mprintf(("Null Pointer *WAS* to blame after all."));
			 continue;
		 }

		 net_sig_idx = objp->net_signature - 1;
		 if (net_sig_idx < 0) {
			 mprintf(("Yup, it looks like 0 is a valid net_signature after all."));
			 continue;
		 }

		 if (net_sig_idx > 999) {
			 continue;
		 }

		 mprintf(("Ok.  signature was: %d .  Also cur_idx is: %d", net_sig_idx, Svr_frames.cur_idx));

		// Update the position and orientation here in order to record the movement
		Svr_frames.frame_info[net_sig_idx].positions[Svr_frames.cur_idx] = objp->pos;
		Svr_frames.frame_info[net_sig_idx].orientations[Svr_frames.cur_idx] = objp->orient;
		Svr_frames.frame_info[net_sig_idx].rotational_velocities[Svr_frames.cur_idx] = objp->phys_info.rotvel;
	}
}

// Increment the tracker per frame, before packets are processed
// Server Only
void multi_ship_record_increment_frame() 
{
	mprintf(("I'm the last one to run!!! 2\n"));

	Svr_frames.number_of_frames++;
	Svr_frames.cur_idx++;

	// Because we are only tracking 240 frames (2 secs on a 120 fps machine), we will have to wrap the index often
	if (Svr_frames.cur_idx == MAX_FRAMES_RECORDED) {
		Svr_frames.cur_idx = 0;
		Svr_frames.wrap_count++;
		if (Svr_frames.wrap_count == MAX_SERVER_TRACKER_SMALL_WRAPS) {
			Svr_frames.wrap_count = 0;
		}
	}
	Svr_frames.timestamps[Svr_frames.cur_idx] = timestamp();
	// reset the number of times we've sent the timestamp to the client.
	Svr_frames.frame_timestamp_count = 0;
}

// returns the last frame's index.
int multi_find_prev_frame_idx() {
	if (Svr_frames.cur_idx == 0) {
		return MAX_FRAMES_RECORDED - 1;
	} else {
		return Svr_frames.cur_idx - 1;
	}
}

ubyte multi_ship_record_calculate_wrap(ushort combined_frame) {
	// unpack the wrap and frame from the client packet
	mprintf(("I'm the last one to run!!! 3\n"));

	int frame = combined_frame % MAX_FRAMES_RECORDED;
	return (combined_frame - frame) / MAX_FRAMES_RECORDED;
}

// In order to find the right frame to start our weapon simulation, we look for the first
// frame that will be before our incoming timestamp.
int multi_ship_record_find_frame(ushort client_frame, ubyte wrap, int time_elapsed)
{
	mprintf(("I'm the last one to run!!! 4\n"));

	// unpack the wrap and frame from the client packet
	int frame = client_frame % MAX_FRAMES_RECORDED;

	bool same_wrap = false;
	// get how many frames we would go into the future.
	int target_timestamp = Svr_frames.timestamps[frame] + time_elapsed;

	// easy case, client is reasonably up to date, so just return the frame.
	if (wrap == Svr_frames.wrap_count) {
		same_wrap = true;

	}	// somewhat out of date but still salvagable case.
	else if (wrap == (Svr_frames.wrap_count - 1)) {
		// but we can't use it if it would index to info in the current wrap instead of the previous wrap.
		if (frame >= Svr_frames.cur_idx) {
			mprintf(("Client requested frame from wrong wrap.\n"));
			return -1;
		}

		// Just in case the larger wrap just happened....
	} else if (wrap == MAX_SERVER_TRACKER_SMALL_WRAPS && Svr_frames.wrap_count == 0){
		if (frame >= Svr_frames.cur_idx) {
			mprintf(("Client requested frame from wrong wrap.\n"));
			return -1;
		}

		// request is way too old.
	} else {
		mprintf(("Client requested frame from wrong wrap.\n"));
		return -1;
	}

	// Now that the wrap has been verified, if time_elapsed is zero return the frame it gave us.
	if(time_elapsed == 0){
		return frame;
	}

	// Otherwise, look for the frame that the client is saying to look for.  If we hit the frame the client sent, abort.
	for (int i = Svr_frames.cur_idx - 1; i > -1; i--){
		// Check to see if the client's timestamp matches the recorded frames.
		if ((Svr_frames.timestamps[i] <= target_timestamp) && (Svr_frames.timestamps[i + 1] > target_timestamp))
			return i;
		else if (i == frame)
			return -1;
	}

	// Check for the wrap.
	if ((Svr_frames.timestamps[MAX_FRAMES_RECORDED - 1] <= target_timestamp) && (Svr_frames.timestamps[0] > target_timestamp)) {
		return MAX_FRAMES_RECORDED - 1;
	}

	// Check the oldest frames.
	for (int i = MAX_FRAMES_RECORDED - 2; i > Svr_frames.cur_idx; i--) {
		if ((Svr_frames.timestamps[i] <= target_timestamp) && (Svr_frames.timestamps[i + 1] > target_timestamp))
			return i;
		else if (i == frame)
			return -1;
	}

	// this is if again the request is way too delayed to be valid, but somehow wasn't caught earlier.
	mprintf(("g) Timestamp given to frame tracker is invalid. And in a section that should be logically dead\n"));
	return -1;
}

// verify that a given frame exists for a given ship, requires the sequence number that the client sends.
bool multi_ship_record_verify_frame(object* objp, int combined_frame) 
{

	mprintf(("Just in case frame verficiation... verification!  \n"));

	// calculate the exact frame that combined_frame should be, if the largest wrap has occured. 
	if (Svr_frames.number_of_frames >= SERVER_TRACKER_LARGE_WRAP_TOTAL) {
		int temp = Svr_frames.number_of_frames % SERVER_TRACKER_LARGE_WRAP_TOTAL;
		ushort larger_wraps = (ushort)((Svr_frames.number_of_frames - temp)  / SERVER_TRACKER_LARGE_WRAP_TOTAL);
		combined_frame += (larger_wraps * SERVER_TRACKER_LARGE_WRAP_TOTAL);
		mprintf(("Part1 was true (somehow), Vars are temp %d, larger_wraps %d, combined_frame %d  \n", temp, larger_wraps, combined_frame));
	}

	// do a quick check to see if the frame sent by the client was pre-wrap and decrement, if so.
	if (combined_frame > Svr_frames.number_of_frames) {
		mprintf(("Part2 was true, combined_frame %d, number_of_frames %d \n", combined_frame, Svr_frames.number_of_frames));
		// if it looks like pre-wrap and then subtracts to less than zero, it's invalid data.
		if (combined_frame - SERVER_TRACKER_LARGE_WRAP_TOTAL < 0) {
			mprintf(("Invalid data!\n"));
			return false;
		}

		combined_frame -= SERVER_TRACKER_LARGE_WRAP_TOTAL;
		mprintf(("Combined_frame decremented to %d combined_frame \n", combined_frame));
	}

	// combined_frame should now house the correct frame, no we compare to verify. TODO: finish the death_or_depart variable assignments 
	if (/*combined_frame < object_updates.svr_frames.death_or_depart_frame[objp->net_signature] && */ combined_frame >= Svr_frames.frame_info[objp->net_signature].initial_frame) {
		mprintf(("Final frame verification SUCCEEEEEEEEEDED!!!!.\n"));

		return true;
	}
	mprintf(("Final frame verification failed.\n"));
	return false;
}

// Quick lookup for the record of position.
vec3d multi_ship_record_lookup_position(object* objp, int frame) {
	mprintf(("I'm the last one to run!!! 5\n"));
	int net_sig_idx = objp->net_signature - 1;
	return Svr_frames.frame_info[frame].positions[net_sig_idx];
}

// Quick lookup for the record of orientation.
matrix multi_ship_record_lookup_orientation(object* objp, int frame) {
	mprintf(("I'm the last one to run!!! 6\n"));
	int net_sig_idx = objp->net_signature - 1;
	return Svr_frames.frame_info[frame].orientations[net_sig_idx];
}

// quickly lookup how much time has passed since the given frame.
uint multi_ship_record_get_time_elapsed(ushort original_frame) {
	mprintf(("I'm the last one to run!!! 7\n"));

	return Svr_frames.timestamps[Svr_frames.cur_idx] - Svr_frames.timestamps[original_frame];
}

// 
int multi_ship_record_adjust_timestamp(int starting_frame, int ending_frame, int time_elapsed) {
	mprintf(("I'm the last one to run!!! 8\n"));

	starting_frame = starting_frame % MAX_FRAMES_RECORDED;

	int return_value = time_elapsed - (Svr_frames.timestamps[ending_frame] - Svr_frames.timestamps[starting_frame]);
	mprintf(("Adjust timestamp's value is: %d",return_value));
	return return_value;
}

// find the exact point on the server that the client sees by interpolating (linearly)
void multi_ship_record_interp_between_frames(vec3d* interp_pos, matrix* interp_ori, int net_signature, ushort original_frame, int time_elapsed) {
	mprintf(("I'm the last one to run!!! 9\n"));

	int timestamp_difference;
	float scale_factor;
	int next_frame;

	// figure out what the other frame index would be.
	if (original_frame == MAX_FRAMES_RECORDED - 1) {
		next_frame = 0;
	}	else {
		next_frame = original_frame + 1;
	}

	// find how much time was there between frames
	timestamp_difference = Svr_frames.timestamps[next_frame] - Svr_frames.timestamps[original_frame];
	Assertion(timestamp_difference >= 0, "While interpolating, timestamp difference was invalid value: %d", timestamp_difference);

	// figure out how far between the points we would be
	scale_factor = (float)(time_elapsed / timestamp_difference);

	vec3d pos_a, pos_b, temp_vec;

	// retreive the points
	pos_a = Svr_frames.frame_info[original_frame].positions[net_signature];
	pos_b = Svr_frames.frame_info[next_frame].positions[net_signature];

	// and calculate
	vm_vec_sub(&temp_vec, &pos_b, &pos_a);
	vm_vec_scale(&temp_vec, scale_factor);
	vm_vec_add(interp_pos, &temp_vec, &pos_a);

	angles angles_a, angles_b, angles_final;

	// get the angles
	vm_extract_angles_matrix(&angles_a, &Svr_frames.frame_info[original_frame].orientations[net_signature]);
	vm_extract_angles_matrix(&angles_b, &Svr_frames.frame_info[next_frame].orientations[net_signature]);

	// and calculate
	vm_interpolate_angles_quick(&angles_final, &angles_a, &angles_b, scale_factor, &Svr_frames.frame_info[next_frame].rotational_velocities[net_signature]);

	vm_angles_2_matrix(interp_ori, &angles_final);
}

// TODO: FIX AND FIND PLACE TO CALL
// Clear all records from tracking struct, should only be called on game exit.
void multi_ship_record_clear_all(){ 
	mprintf(("I'm the last one to run!!! 10\n"));

	delete[] &Svr_frames;
	delete[] &Oo_general_info;
}

// ---------------------------------------------------------------------------------------------------
// Client side frame tracking, for now used only for referenced in fire packets to improve client accuracy.
// 

// Client Only - Add a new object on *ship creation* to the tracking struct
void multi_ship_record_add_ship_client(int obj_num)
{
	mprintf(("Ship added to client record struct.\n"));
	Assertion(!MULTIPLAYER_MASTER, "Server accessed a client only function. Please report!!");

	if (MULTIPLAYER_MASTER) {
		return;
	}

	object* objp = &Objects[obj_num];

	// check for a ship that will enter the mission later on and has not yet had its net_signature set.
	if (objp->net_signature == 0) {
		return;
	}

	// our target size is the number of ships in the vector plus one because net_signatures start at 1 and size gives the number of elements (1 indexed also), and this should be a new element.
	int target_size = (int)Oo_general_info.interp.size() + 1;

	mprintf(("The client object adder says that the target size is %d and the incoming net_signature is %d\n", target_size, objp->net_signature));	

	// if we're right where we should be.
	if (objp->net_signature == target_size) {
		mprintf(("Server client tracker believes object signature is right on target.\n"));
		Oo_general_info.interp.push_back(*new(interp_message_tracking));		

	}	// if the storage already exists.
	else if (objp->net_signature < (target_size)) {
		mprintf(("Server client tracker belives that the object already had storage available. No storage added.\n"));

	}	// if not, create the storage.
	else if (objp->net_signature > (target_size)) {
		mprintf(("Server client tracker belives that the object is way past the storage available..\n"));
		for (int i = objp->net_signature - (target_size); i > 0; i--) {
			Oo_general_info.interp.push_back(*new(interp_message_tracking));
		}
	} // and if I missed something.
	else {
		mprintf(("Some sort of logic was misssed in the client object adder function.\n"));
	}
	// Have this ready for later when we're sure that it works correctly at least most of the time.
	//	Assertion(objp->net_signature == target_size, "New entry into the ship traker struct does not equal the index that should belong to it.\nNet_signature: %d and target_size %d\n", objp->net_signature, target_size);

	// net_signature is 1 indexed, so we have to adjust to correctly access info the vector.
	int net_sig_idx = objp->net_signature - 1;

	// Initialize the vector
	Oo_general_info.interp[net_sig_idx].cur_pack_pos_frame = -1;
	Oo_general_info.interp[net_sig_idx].prev_pack_pos_frame = -1;
	Oo_general_info.interp[net_sig_idx].cur_pack_ori_frame = -1;
	Oo_general_info.interp[net_sig_idx].prev_pack_ori_frame = -1;
	Oo_general_info.interp[net_sig_idx].interp_count = 0;
}

// See if a newly arrived object update packet should be the new reference for the improved primary fire packet 
void multi_ship_record_rank_seq_num(object* objp, ushort seq_num) 
{
	mprintf(("I'm the last one to run!!! 11\n"));

	// see if it's more recent.  Most recent is best.
	if (seq_num > Oo_general_info.most_recent_frame) {
		Oo_general_info.most_recent_updated_ship_index = objp->net_signature;
		Oo_general_info.most_recent_frame = seq_num;
		Oo_general_info.ref_timestamp = timestamp();
		Oo_general_info.distance_to_most_recent = vm_vec_dist_squared(&objp->pos, &Objects[Player->objnum].pos);

	} // if this packet is from the same frame,the closer ship makes for a slightly more accurate reference point
	else if (seq_num == Oo_general_info.most_recent_frame) {
		float temp_distance = vm_vec_dist_squared(&objp->pos, &Objects[Player->objnum].pos);
		if (Oo_general_info.distance_to_most_recent > temp_distance) {
			Oo_general_info.most_recent_updated_ship_index = objp->net_signature;
			Oo_general_info.most_recent_frame = seq_num;
			Oo_general_info.ref_timestamp = timestamp();
			Oo_general_info.distance_to_most_recent = temp_distance;
		}
	}	// the wrap case (which should be rare), this could potentially break if the mission designer leaves
		// just 1 player completely by themselves on a long mission at *just* the right time.
	else if ((Oo_general_info.most_recent_frame > 65300) && seq_num < MAX_FRAMES_RECORDED) {
		Oo_general_info.most_recent_updated_ship_index = objp->net_signature;
		Oo_general_info.most_recent_frame = seq_num;
		Oo_general_info.ref_timestamp = timestamp();
		Oo_general_info.distance_to_most_recent = vm_vec_dist_squared(&objp->pos, &Objects[Player->objnum].pos);
	}
}

// Client side! Quick lookup for the most recently received ship
ushort multi_client_lookup_frame_ship_index()
{
	mprintf(("I'm the last one to run!!! 12\n"));

	return Oo_general_info.most_recent_updated_ship_index;
}

// Client side! Quick lookup for the most recently received frame
ushort multi_client_lookup_frame_idx()
{
	mprintf(("I'm the last one to run!!! 13\n"));

	return Oo_general_info.most_recent_frame;
}

// Client side! Quick lookup for the most recently received timestamp.
int multi_client_lookup_frame_timestamp()
{
	mprintf(("I'm the last one to run!!! 14\n"));

	return Oo_general_info.ref_timestamp;
}


// ---------------------------------------------------------------------------------------------------
// OBJECT UPDATE FUNCTIONS
//

object *OO_player_obj;
int OO_sort = 1;

bool multi_oo_sort_func(const short &index1, const short &index2)
{
	mprintf(("I'm the last one to run!!! 16\n"));

	object *obj1, *obj2;
	float dist1, dist2;
	float dot1, dot2;
	vec3d v1, v2;
	vec3d vn1, vn2;

	// if the indices are bogus, or the objnums are bogus, return ">"
	if((index1 < 0) || (index2 < 0) || (Ships[index1].objnum < 0) || (Ships[index2].objnum < 0)){
		return false;
	}

	// get the 2 objects
	obj1 = &Objects[Ships[index1].objnum];
	obj2 = &Objects[Ships[index2].objnum];

	// get the distance and dot product to the player obj for both
	vm_vec_sub(&v1, &OO_player_obj->pos, &obj1->pos);
	dist1 = vm_vec_copy_normalize(&vn1, &v1);
	vm_vec_sub(&v2, &OO_player_obj->pos, &obj2->pos);
	dist2 = vm_vec_copy_normalize(&vn2, &v2);
	dot1 = vm_vec_dot(&OO_player_obj->orient.vec.fvec, &vn1);
	dot2 = vm_vec_dot(&OO_player_obj->orient.vec.fvec, &vn2);

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
	mprintf(("I'm the last one to run!!! 20\n"));

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

		// don't send info for dying ships
		if (Ships[Objects[moveup->objnum].instance].flags[Ship::Ship_Flags::Dying]){
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
	mprintf(("I'm the last one to run!!! 21\n"));

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
// Cyborg 17 - This is for percent values that could be from -100% to +100%. For use when exact values are as important as negative values.
// We loose a tiny amount of accuracy by using 254 instead of 255, but this way the receiving computer will be able to tell without extra logic
// whenever the float == 0.0f .
#define PACK_POSITIVE_NEGATIVE_PERCENT(v) { std::uint8_t upercent; if(v < -1.0f){v = -1.0f;} v++; upercent = (v * 127.0f) <= 254.0f ? (std::uint8_t)(v * 127.0f) : (std::uint8_t)254; memcpy(data + packet_size + header_bytes, &upercent, sizeof(std::uint8_t)); packet_size++; }

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
	if(pl == NULL){
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
		header_bytes = 3;
	}	

	// Send the timestamp to the client to help the client sync more cleanly to the server up to 10 times per frame.
	// This increases the chance of a timely delivery.
	if (MULTIPLAYER_MASTER && Svr_frames.frame_timestamp_count < 10) {

		oo_flags |= OO_TIMESTAMP;
		ushort temp_timestamp = Svr_frames.timestamps[Svr_frames.cur_idx] - Svr_frames.timestamps[multi_find_prev_frame_idx()];
		PACK_USHORT(temp_timestamp);
		Svr_frames.frame_timestamp_count++;
		mprintf(("Packet foo: Timestamp is %d\n", temp_timestamp));

		// putting this in the position bucket because it's mainly to help with position interpolation
		multi_rate_add(NET_PLAYER_NUM(pl), "pos", 1);
	}

	// if we're a client (and therefore sending control info), pack client-specific info
	if((Net_player != NULL) && !(Net_player->flags & NETINFO_FLAG_AM_MASTER)){
		packet_size += multi_oo_pack_client_data(data + packet_size + header_bytes);		
	}		
		
	// position 
	if ( oo_flags & OO_POS_NEW ) {		
		ret = (ubyte)multi_pack_unpack_position( 1, data + packet_size + header_bytes, &objp->pos );
		packet_size += ret;
		mprintf(("Packet foo: object pos is: %f %f %f\n", objp->pos.xyz.x, objp->pos.xyz.y, objp->pos.xyz.z));
		// global records
		multi_rate_add(NET_PLAYER_NUM(pl), "pos", ret);
	}			

	// orientation	
	if(oo_flags & OO_ORIENT_NEW){
		angles temp_angles;
		vm_extract_angles_matrix(&temp_angles, &objp->orient);	

		ret = (ubyte)multi_pack_unpack_orient( 1, data + packet_size + header_bytes, &temp_angles);

		mprintf(("Packet foo: Orientation fvec is %f, %f, %f\n", objp->orient.vec.fvec.xyz.x, objp->orient.vec.fvec.xyz.y, objp->orient.vec.fvec.xyz.z));
		mprintf(("Packet foo: Orientation rvec is %f, %f, %f\n", objp->orient.vec.rvec.xyz.x, objp->orient.vec.rvec.xyz.y, objp->orient.vec.rvec.xyz.z));
		mprintf(("Packet foo: Orientation uvec is %f, %f, %f\n", objp->orient.vec.uvec.xyz.x, objp->orient.vec.uvec.xyz.y, objp->orient.vec.uvec.xyz.z));
		packet_size += ret;
		multi_rate_add(NET_PLAYER_NUM(pl), "ori", ret);				

		ret = (ubyte)multi_pack_unpack_rotvel( 1, data + packet_size + header_bytes, &objp->orient, &objp->pos, &objp->phys_info );

		mprintf(("Packet foo: rotational vel is %f, %f, %f\n", objp->phys_info.rotvel.xyz.x, objp->phys_info.rotvel.xyz.y, objp->phys_info.rotvel.xyz.z));

		packet_size += ret;	

		// Cyborg17 - Send desired rotational velocity. But since we do have stationary ships, like sentries, that have no 
		// rotational, so we need to check for 0.0f.  We can also save bandwidth by only sending those axes that the ship *can* use.
		// This *will* work if client has matching table info, which it *should*, just like it does on subsystem info.
		float temp_desired_rot = 0.0f;
		if (objp->phys_info.max_rotvel.xyz.x > 0.0f) {
			// Pack the desired rotvel 
			temp_desired_rot = (objp->phys_info.desired_rotvel.xyz.x / objp->phys_info.max_rotvel.xyz.x);
			PACK_POSITIVE_NEGATIVE_PERCENT(temp_desired_rot);
			mprintf(("Packet foo: desired rot vel x is %f\n", temp_desired_rot));
			ret++;
		}
		if (objp->phys_info.max_rotvel.xyz.y > 0.0f) {
			temp_desired_rot = (objp->phys_info.desired_rotvel.xyz.y / objp->phys_info.max_rotvel.xyz.y);
			PACK_POSITIVE_NEGATIVE_PERCENT(temp_desired_rot);
			mprintf(("Packet foo: desired rot vel y is %f\n", temp_desired_rot));
			ret++;
		}
		if (objp->phys_info.max_rotvel.xyz.z > 0.0f) {
			temp_desired_rot = (objp->phys_info.desired_rotvel.xyz.z / objp->phys_info.max_rotvel.xyz.z);
			PACK_POSITIVE_NEGATIVE_PERCENT(temp_desired_rot);
			mprintf(("Packet foo: desired rot vel z is %f\n", temp_desired_rot));

			ret++;
		}

		// global records		
		multi_rate_add(NET_PLAYER_NUM(pl), "ori", ret);		
		ret = 0;
	}

	// velocity (Only send if we've moved!)
	if (oo_flags & OO_POS_NEW) {
		ret = (ubyte)multi_pack_unpack_vel( 1, data + packet_size + header_bytes, &objp->orient, &objp->pos, &objp->phys_info );
		packet_size += ret;		
		
		mprintf(("Packet foo: vel is %f, %f, %f\n", objp->phys_info.vel.xyz.x, objp->phys_info.vel.xyz.y, objp->phys_info.vel.xyz.z));

		// global records		
		multi_rate_add(NET_PLAYER_NUM(pl), "pos", ret);				
		ret = 0;
	}	
		


	// forward and lateral desired velocity	
	if (oo_flags & OO_POS_NEW) {
		float temp_desired_vel;
		if (objp->phys_info.max_vel.xyz.x > 0.0f) {
			temp_desired_vel = objp->phys_info.desired_vel.xyz.x / objp->phys_info.max_vel.xyz.x;
			PACK_POSITIVE_NEGATIVE_PERCENT(temp_desired_vel);
			mprintf(("Packet foo: desired_vel x is %f\n", temp_desired_vel));
			ret++;
		}
		if (objp->phys_info.max_vel.xyz.y > 0.0f) {
			temp_desired_vel = objp->phys_info.desired_vel.xyz.y / objp->phys_info.max_vel.xyz.y;
			PACK_POSITIVE_NEGATIVE_PERCENT(temp_desired_vel);
			mprintf(("Packet foo: desired_vel y is %f\n", temp_desired_vel));
			ret++;
		}
		if (objp->phys_info.max_vel.xyz.z > 0.0f) {
			temp_desired_vel = objp->phys_info.desired_vel.xyz.z / objp->phys_info.max_vel.xyz.z;
			PACK_POSITIVE_NEGATIVE_PERCENT(temp_desired_vel);
			mprintf(("Packet foo: desired_vel z is %f\n", temp_desired_vel));
			ret++;
		}
	}

	// global records	
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

	// just in case we have some kind of invalid data (should've been taken care of earlier in this function)
	if (shipp->ship_info_index >= 0) {	
	// add the # of only the subsystems being changed, and their data
		ship_subsys* subsystem;
		ubyte count = 0;
		ubyte flagged_subsystem_list[MAX_MODEL_SUBSYSTEMS];
		float subsystem_list_health[MAX_MODEL_SUBSYSTEMS];
		int i = 0;

		for (subsystem = GET_FIRST(&shipp->subsys_list); subsystem != END_OF_LIST(&shipp->subsys_list);
			subsystem = GET_NEXT(subsystem)) {
				// Don't send destroyed subsystems, (another packet handles that), but check to see if the subsystem changed since the last update. 
			if ((subsystem->current_hits != 0.0f) && (subsystem->current_hits != Svr_frames.ship_last_sent[objp->net_signature - 1].players[pl->player_id].subsystems[i])) {
				// store the values for use later.
				flagged_subsystem_list[count] = i;
				// this should be safe because we only work with subsystems that have health.
				subsystem_list_health[count] = subsystem->current_hits / subsystem->max_hits;
				// and also track the list of subsystems that we packed by index
				count++;
			}
		}
		
		// Only send info if the count is greater than zero.
		if (count > 0){
			Assertion(count <= MAX_MODEL_SUBSYSTEMS, "Object Update packet exceeded limit for number of subsystems. This is a fatal error in the code, please report!");
			oo_flags |= OO_SUBSYSTEMS_NEW;

			// pack the count of subsystems first.
			PACK_BYTE(count);
			// now we'll pack the actual information
			for (int i = 0; i < count; i++) {
				PACK_BYTE(flagged_subsystem_list[i]);
				PACK_PERCENT(subsystem_list_health[i]);
			}
		}
	}
	
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

	// make sure we have a valid chunk of data
	// Clients: must be able to accomodate the data_size and shipp->np_updates[NET_PLAYER_NUM(pl)].seq before the data itself
	// Server: TODO
	Assert(packet_size < 500);
	if(packet_size >= 500){
		return 0;
	}
	data_size = (ubyte)packet_size;

	// reset packet_size so that we add the header at the beginning of the packet where it belongs.
	packet_size = 0;
	// don't add for clients
	if(Net_player->flags & NETINFO_FLAG_AM_MASTER){		
		multi_rate_add(NET_PLAYER_NUM(pl), "sig", 2);
		ADD_USHORT( objp->net_signature );		
		mprintf(("packet foo: flag value is: %d", oo_flags));
		multi_rate_add(NET_PLAYER_NUM(pl), "flg", 1);
		ADD_USHORT( oo_flags );
	}	

	multi_rate_add(NET_PLAYER_NUM(pl), "siz", 1);
	ADD_DATA( data_size );	
	mprintf(("packet foo: data size is: %d", data_size));
	// send the exact frame that we're on.
	ushort seq = Svr_frames.cur_idx + (MAX_FRAMES_RECORDED * Svr_frames.wrap_count);
	
	multi_rate_add(NET_PLAYER_NUM(pl), "seq", 2);
	ADD_USHORT( seq );
	mprintf(("packet foo: seq value is: %d", seq));
	packet_size += data_size;

	mprintf(("packet foo: flag value is: %d", oo_flags));

	// copy to the outgoing data
	memcpy(data_out, data, packet_size);	
	
	return packet_size;	
}

// unpack information for a client , return bytes processed
int multi_oo_unpack_client_data(net_player *pl, ubyte *data)
{
	ushort in_flags;
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
// Cyborg17 -- Unpacks percents that go from -100% to positive 100% with low resolution. Output is from -100% to 100%.
#define UNPACK_POSITIVE_NEGATIVE_PERCENT(v) { ubyte temp_byte; memcpy(&temp_byte, data + offset, sizeof(ubyte)); v = (float)temp_byte / 127.0f; v--; offset++;}
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

	// add the object's net signature, type and oo_flags
	if (!(Net_player->flags & NETINFO_FLAG_AM_MASTER)) {
		GET_USHORT(net_sig);
		GET_USHORT(oo_flags);
		mprintf(("Packet foo: net_sig is %d (start new packet)\n", net_sig));
		mprintf(("Packet foo: oo_flags is %d\n", oo_flags));
	}
	// clients always pos and orient stuff only
	else {
		oo_flags = (OO_POS_NEW | OO_ORIENT_NEW);
	}
	GET_DATA(data_size);
	mprintf(("data_size is %d", data_size));
	GET_USHORT(seq_num);
	mprintf(("seq_num is %d", seq_num));

	// try and find the object
	if (!(Net_player->flags & NETINFO_FLAG_AM_MASTER)) {
		pobjp = multi_get_network_object(net_sig);
	}
	else {
		if ((pl != NULL) && (pl->m_player->objnum != -1)) {
			pobjp = &Objects[pl->m_player->objnum];
			// Cyborg17 - We still need the net_signature if we're the Server because that's how we track interpolation now.
			net_sig = pobjp->net_signature;
		}
		else {
			pobjp = NULL;
		}
	}

	// if we can't find the object, set pointer to bogus object to continue reading the data
	if ( (pobjp == nullptr) || (pobjp->type != OBJ_SHIP) || (pobjp->instance < 0) || (pobjp->instance >= MAX_SHIPS) || (Ships[pobjp->instance].ship_info_index < 0) || (Ships[pobjp->instance].ship_info_index >= ship_info_size())) {
		offset += data_size;
		return offset;
	}

	// ship pointer
	shipp = &Ships[pobjp->instance];
	sip = &Ship_info[shipp->ship_info_index];

	// ---------------------------------------------------------------------------------------------------------------
	// CRITICAL OBJECT UPDATE SHIZ
	// ---------------------------------------------------------------------------------------------------------------

	int net_sig_idx = net_sig - 1;

	if (seq_num == Oo_general_info.interp[net_sig_idx].most_recent_packet) {
		mprintf(("\n\n\nWARNING!! PACKET RECEIVED TWICE IN A ROW!!\n\n\n")); //TODO: REMOVE Before final version
	}
	// if the packet seems to be out of order
	if(seq_num < Oo_general_info.interp[net_sig_idx].most_recent_packet){
		// Test for the wraparound case:
		// On a 120 FPS machine, 64500 leaves about 4-5 seconds where a wrap is possible, after 9 minutes of straight flying.
		if ((Oo_general_info.interp[net_sig_idx].most_recent_packet - seq_num) >= 64500) {
			Oo_general_info.interp[net_sig_idx].most_recent_packet = seq_num; // now we know that things have wrapped.

			// If we have finally received the first wrapped packet, we need to adjust the frames that the client has recorded in each 
			// category. This adjustment will only happen once per wrap because of the check below.
			Oo_general_info.interp[net_sig_idx].rot_vel_frame -= SERVER_TRACKER_LARGE_WRAP_TOTAL;
			Oo_general_info.interp[net_sig_idx].hull_frame -= SERVER_TRACKER_LARGE_WRAP_TOTAL;
			Oo_general_info.interp[net_sig_idx].shields_frame -= SERVER_TRACKER_LARGE_WRAP_TOTAL;

			for (auto subsys = Oo_general_info.interp[net_sig_idx].subsystems_frame.begin(); subsys != Oo_general_info.interp[net_sig_idx].subsystems_frame.end(); subsys++) { 
				subsys -= SERVER_TRACKER_LARGE_WRAP_TOTAL;
			}
			Oo_general_info.interp[net_sig_idx].ai_frame -= SERVER_TRACKER_LARGE_WRAP_TOTAL;
		}
	// if the packet appears to be in order, we still need to make sure to only change the most recent frame if we're sure that 
	// the incoming packet isn't prewrap (every ship must be updated every 2 seconds)

	// This packet is new because it's not pre-wrap, this check will hopefully guarantee that the client will only see the wrap once.
	}	else if ((seq_num - Oo_general_info.interp[net_sig_idx].most_recent_packet) < MAX_FRAMES_RECORDED) {
		Oo_general_info.interp[net_sig_idx].most_recent_packet = seq_num;

	// This packet is probably mostly trash because it is so we decrement the incoming sequence as well, so it can be compared to the 
	// already adjusted frames correctly.
	}	else {
		seq_num -= SERVER_TRACKER_LARGE_WRAP_TOTAL;
	}


	mprintf(("Packet foo: seq_num is %d\n", seq_num));


	// Cyborg17 - determine if this is the most recently updated ship.  If it is, it will become the ship that the
	// client will use as its reference when sending a primary shot packet.
	if (MULTIPLAYER_CLIENT) {
		multi_ship_record_rank_seq_num(pobjp, seq_num);
	}

	// get the timestamp that belonged to this server for this frame.
	// Because we want as many timestamps as possible, we want to record what we get, no matter when it came from.
	if (oo_flags & OO_TIMESTAMP){
		ushort timestamp;
		GET_USHORT(timestamp);
		mprintf(("Packet foo: timestamp is %d\n", timestamp));
		// figure out how many items we may have to create
		int temp_diff = seq_num - Oo_general_info.received_timestamps.size() + 1;
		// if it already has enough slots, just fill in the value.
		if (temp_diff <= 0) {
			Oo_general_info.received_timestamps[seq_num] = timestamp;
		}	// if there weren't enough slots, create the necessary slots.
		else {
			// loop is checked against 1, because once there is only a difference of 1, we should add the timestamp onto the end.
			for (int i = temp_diff; i > 1; i--) {
				// keep adding zero to the timestamps we have not yet received, because that is an impossible value.
				Oo_general_info.received_timestamps.push_back(0);
			}
			// lastly, add the timestamp we received to the end.
			Oo_general_info.received_timestamps.push_back(timestamp);
		}	
	}

	// if this is from a player, read his button info
	if(Net_player->flags & NETINFO_FLAG_AM_MASTER){
		offset += multi_oo_unpack_client_data(pl, data + offset);		
	}	

	// default to the old info before reading the new info
	vec3d new_pos = pobjp->pos;
	physics_info new_phys_info = pobjp->phys_info;
	matrix new_orient = pobjp->orient;
	bool pos_new = false, adjust_interp_pos = false;
	
	// position
	if ( oo_flags & OO_POS_NEW ) {

		int r1 = multi_pack_unpack_position(0, data + offset, &new_pos);
		mprintf(("Packet foo: position is %f %f %f\n", new_pos.xyz.x, new_pos.xyz.y, new_pos.xyz.z));

		offset += r1;

		if (seq_num > Oo_general_info.interp[net_sig_idx].cur_pack_pos_frame) {
			Oo_general_info.interp[net_sig_idx].prev_pack_pos_frame = Oo_general_info.interp[net_sig_idx].cur_pack_pos_frame;
			Oo_general_info.interp[net_sig_idx].cur_pack_pos_frame = seq_num;
			multi_oo_calc_pos_time_difference(net_sig_idx);
			pos_new = true;
			adjust_interp_pos = true;
		} // if we actually received an old frame...
		else if (seq_num > Oo_general_info.interp[net_sig_idx].prev_pack_pos_frame){
			multi_oo_calc_pos_time_difference(net_sig_idx);
			Oo_general_info.interp[net_sig_idx].prev_pack_pos_frame = seq_num;
			adjust_interp_pos = true;
		}
	}

	bool ori_new = false, adjust_interp_ori = false;
	angles new_angles;

	// orientation	
	if ( oo_flags & OO_ORIENT_NEW ) {		
		int r2 = multi_pack_unpack_orient( 0, data + offset, &new_angles );
		offset += r2;		


		if (seq_num > Oo_general_info.interp[net_sig_idx].cur_pack_ori_frame) {
			Oo_general_info.interp[net_sig_idx].prev_pack_ori_frame = Oo_general_info.interp[net_sig_idx].cur_pack_ori_frame;
			Oo_general_info.interp[net_sig_idx].cur_pack_ori_frame = seq_num;
			ori_new = true;
			adjust_interp_ori = true;
		} // if we actually received an old frame...
		else if (seq_num > Oo_general_info.interp[net_sig_idx].prev_pack_ori_frame){
			Oo_general_info.interp[net_sig_idx].prev_pack_ori_frame = seq_num;
			adjust_interp_ori = true;
		}
		// new version of the orient packet sends angles instead to save on bandwidth, so we need the orienation from that.
		vm_angles_2_matrix(&new_orient, &new_angles);
		vm_orthogonalize_matrix(&new_orient);

		mprintf(("Packet foo: orietation fvec is %f %f %f\n", new_orient.vec.fvec.xyz.x, new_orient.vec.fvec.xyz.y, new_orient.vec.fvec.xyz.z));
		mprintf(("Packet foo: orietation uvec is %f %f %f\n", new_orient.vec.uvec.xyz.x, new_orient.vec.uvec.xyz.y, new_orient.vec.uvec.xyz.z));
		mprintf(("Packet foo: orietation rvec is %f %f %f\n", new_orient.vec.rvec.xyz.x, new_orient.vec.rvec.xyz.y, new_orient.vec.rvec.xyz.z));

		int r5 = multi_pack_unpack_rotvel( 0, data + offset, &new_orient, &new_pos, &new_phys_info );
		offset += r5;
		mprintf(("Packet foo: rotvel is %f %f %f\n", new_phys_info.rotvel.xyz.x, new_phys_info.rotvel.xyz.y, new_phys_info.rotvel.xyz.z));


		// Cyborg17 - Now unpack desired rotational Velocity.
		float temp_desired_rotvel;
		vec3d temp_vector = vmd_zero_vector;
		if (pobjp->phys_info.max_rotvel.xyz.x > 0.0f) {
			UNPACK_POSITIVE_NEGATIVE_PERCENT(temp_desired_rotvel)
				temp_vector.xyz.x = temp_desired_rotvel * pobjp->phys_info.max_rotvel.xyz.x;
			}
		if (pobjp->phys_info.max_rotvel.xyz.y > 0.0f) {
			UNPACK_POSITIVE_NEGATIVE_PERCENT(temp_desired_rotvel)
				new_phys_info.desired_rotvel.xyz.y = temp_desired_rotvel * pobjp->phys_info.max_rotvel.xyz.y;
		}
		if (pobjp->phys_info.max_rotvel.xyz.z > 0.0f) {
			UNPACK_POSITIVE_NEGATIVE_PERCENT(temp_desired_rotvel)
				new_phys_info.desired_rotvel.xyz.z = temp_desired_rotvel * pobjp->phys_info.max_rotvel.xyz.z;
		}
		if (adjust_interp_ori) {
			new_phys_info.desired_rotvel = temp_vector;
			mprintf(("Packet foo: desired_rotvel is %f %f %f\n", new_phys_info.desired_rotvel.xyz.x, new_phys_info.desired_rotvel.xyz.y, new_phys_info.desired_rotvel.xyz.z));
		}
	}

	// Velocity
	if (oo_flags & OO_POS_NEW) {
		int r3 = multi_pack_unpack_vel( 0, data + offset, &new_orient, &new_pos, &new_phys_info );
		mprintf(("Packet foo: velocity is %f %f %f\n", new_phys_info.vel.xyz.x, new_phys_info.vel.xyz.y, new_phys_info.vel.xyz.z));

		offset += r3;
		
		float temp_des_vel;
		vec3d temp_vec;
		if (pobjp->phys_info.max_vel.xyz.x > 0.0f) {
			UNPACK_POSITIVE_NEGATIVE_PERCENT(temp_des_vel);
			temp_vec.xyz.x = temp_des_vel * pobjp->phys_info.max_vel.xyz.x;
		}
		if (pobjp->phys_info.max_vel.xyz.y > 0.0f) {
			UNPACK_POSITIVE_NEGATIVE_PERCENT(temp_des_vel);
			temp_vec.xyz.y = temp_des_vel * pobjp->phys_info.max_vel.xyz.y;
		}
		if (pobjp->phys_info.max_vel.xyz.z > 0.0f) {
			UNPACK_POSITIVE_NEGATIVE_PERCENT(temp_des_vel);
			temp_vec.xyz.z = temp_des_vel * pobjp->phys_info.max_vel.xyz.z;
		}	
		if (adjust_interp_pos){
			new_phys_info.desired_vel = temp_vec;
			mprintf(("Packet foo: desired_vel is %f %f %f\n", new_phys_info.desired_vel.xyz.x, new_phys_info.desired_vel.xyz.y, new_phys_info.desired_vel.xyz.z));
		}
		

		// if we're past the position update tolerance, bash.
		// It will cause a jump, but it should be nice and smooth immediately afterwards
		if(vm_vec_dist(&new_pos, &pobjp->pos) > OO_POS_UPDATE_TOLERANCE){
			pobjp->pos = new_pos;
			// Cyborg17 - Also, make sure that FSO knows that it does not need to smooth anything out
			Oo_general_info.interp[net_sig_idx].position_error = vmd_zero_vector;
		// we can set the error to smooth things out if we are not jumping to a new position.
		} else {
			vm_vec_sub(&Oo_general_info.interp[net_sig_idx].position_error, &pobjp->pos, &new_pos);
		}
		mprintf(("Packet foo: position_error is %f %f %f\n", Oo_general_info.interp[net_sig_idx].position_error.xyz.x, Oo_general_info.interp[net_sig_idx].position_error.xyz.y, Oo_general_info.interp[net_sig_idx].position_error.xyz.z));
		
		multi_oo_maybe_update_interp_info(net_sig_idx, &new_pos, &new_angles, adjust_interp_pos, pos_new, adjust_interp_ori, ori_new);
		multi_oo_calc_interp_splines(pobjp->net_signature, &pobjp->pos, &pobjp->phys_info, &new_pos, &new_orient, &new_phys_info);


//		pobjp->phys_info.vel = new_phys_info.vel; // this should be set when we do our interpolation, above.	
		pobjp->phys_info.desired_vel = new_phys_info.desired_vel;
	} 

	// we'll just sim rotation straight. it works fine.
	if(oo_flags & OO_ORIENT_NEW){
		pobjp->orient = new_orient;
		pobjp->phys_info.rotvel = new_phys_info.rotvel;
		// pobjp->phys_info.desired_rotvel = vmd_zero_vector;
		pobjp->phys_info.desired_rotvel = new_phys_info.desired_rotvel;
	}	


	// ---------------------------------------------------------------------------------------------------------------
	// ANYTHING BELOW HERE WORKS FINE - nothing here which causes jumpiness or bandwidth problems :) WHEEEEE!
	// ---------------------------------------------------------------------------------------------------------------
	
	// hull info
	if ( oo_flags & OO_HULL_NEW ){
		UNPACK_PERCENT(fpct);
		if (Oo_general_info.interp[net_sig_idx].hull_frame < seq_num) {
			pobjp->hull_strength = fpct * Ships[pobjp->instance].ship_max_hull_strength;
			Oo_general_info.interp[net_sig_idx].hull_frame = seq_num;
		}
	}	

	// update shields
	if (oo_flags & OO_SHIELDS_NEW) {
		float quad = shield_get_max_quad(pobjp);

		if (Oo_general_info.interp[net_sig_idx].shields_frame < seq_num) {
			for (int i = 0; i < pobjp->n_quadrants; i++) {
				UNPACK_PERCENT(fpct);
				pobjp->shield_quadrant[i] = fpct * quad;
			}
			Oo_general_info.interp[net_sig_idx].shields_frame = seq_num;
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

		// Before we start the loop, we need to get the first subsystem
		GET_DATA(current_subsystem);

		// this iterates through the packet and subsytem list simultaneously, changing values only when it finds a match.
		for (subsysp = firstsubsys; subsysp != END_OF_LIST(&shipp->subsys_list); subsysp = GET_NEXT(subsysp)) {

			int idx = std::distance(firstsubsys, subsysp);

			if (current_subsystem != idx) {
				// the current subsystem was not sent by the server, so try the next subsystem.
				continue;
			}

			// we found a match, so grab the next byte, so we can calculate the new hitpoints
			UNPACK_PERCENT(current_percent);

			if (seq_num > Oo_general_info.interp[net_sig_idx].subsystems_frame[idx]) {
				subsysp->current_hits = current_percent * subsysp->max_hits;
				subsys_count++;

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

		// recalculate all ship subsystems
		ship_recalc_subsys_strength(shipp);
	}

	if ( oo_flags & OO_AI_NEW ) {
		// ai mode info
		ubyte umode;
		short submode;
		ushort target_signature;
		object *target_objp;

		GET_DATA( umode );
		GET_SHORT( submode );
		GET_USHORT( target_signature );		

		if(shipp->ai_index >= 0){
			Ai_info[shipp->ai_index].mode = umode;
			Ai_info[shipp->ai_index].submode = submode;		

			// set this guys target objnum
			target_objp = multi_get_network_object( target_signature );
			if ( target_objp == NULL ){
				Ai_info[shipp->ai_index].target_objnum = -1;
			} else {
				Ai_info[shipp->ai_index].target_objnum = OBJ_INDEX(target_objp);
			}
		}		

		// primary weapon energy		
		float weapon_energy_pct;
		UNPACK_PERCENT(weapon_energy_pct);
		shipp->weapon_energy = sip->max_weapon_reserve * weapon_energy_pct;		
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
		if((shipp != NULL) && (shipp->ai_index >= 0) && (shipp->ai_index < MAX_AI_INFO)){
			Ai_info[shipp->ai_index].ai_flags.from_u64(ai_flags);
			Ai_info[shipp->ai_index].mode = ai_mode;
			Ai_info[shipp->ai_index].submode = ai_submode;

			object *objp = multi_get_network_object( dock_sig );
			if(objp != NULL){
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
	if( !MULTIPLAYER_MASTER && (shipp != NULL) ){
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
	if(MULTIPLAYER_MASTER && (pl != NULL) && (pobjp != NULL)){
		pl->s_info.eye_pos = pobjp->pos;
		pl->s_info.eye_orient = pobjp->orient;
	} 		

	// flag the object as just updated
	// pobjp->flags |= OF_JUST_UPDATED;
	
	mprintf(("I'm through the packet safely!\n"));

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
		Ships[objp->instance].np_updates[NET_PLAYER_NUM(pl)].update_stamp = timestamp(stamp);
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

	// determine what the timestamp is for this object
	if(obj->type == OBJ_SHIP){
		stamp = Ships[obj->instance].np_updates[NET_PLAYER_NUM(pl)].update_stamp;
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
	sip = NULL;
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

	int net_sig_idx = obj->net_signature - 1;
	// position should be almost constant, but only if they've moved
	if (Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].position != obj->pos) {
		oo_flags |= OO_POS_NEW;
		// update the last position sent, will be done in each of the cases below.
		Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].position = obj->pos;
	}

	// same with orientation
	if (obj->phys_info.rotvel != vmd_zero_vector) {
		oo_flags |= OO_ORIENT_NEW;
	}

	if (Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].rot_vel != obj->phys_info.rotvel) {
		oo_flags |= OO_ROT_VEL_NEW;
		Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].rot_vel = obj->phys_info.rotvel;
	}

	// if its a small ship, add weapon link info
	// Cyborg17 - these don't take any extra space because they are part of the flags variable, so it's ok to send them with every packet.
	if((sip != NULL) && (sip->is_fighter_bomber())){
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
	if(Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].hull != obj->hull_strength){
		oo_flags |= (OO_HULL_NEW);
		Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].hull = obj->hull_strength;		
	}

	float temp_max = shield_get_max_quad(obj);
	bool all_max = true;
	// maybe update shields, which are constantly repairing, and should be regularly updated, unless they are already spotless.
	for (auto quadrant : obj->shield_quadrant) {
		if (quadrant != temp_max) {
			all_max = false;
			break;
		}
	}

	if (all_max) {
		// shields are currently perfect, were they perfect last time?
		if ( Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].perfect_shields_sent == false){
			// send the newly perfected shields
			oo_flags |= OO_SHIELDS_NEW;
		}
		// make sure to mark it as perfect for next time.
		Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].perfect_shields_sent = true;
	} // if they're not perfect, make sure they're marked as not perfect.
	else {
		Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].perfect_shields_sent = false;
		oo_flags |= OO_SHIELDS_NEW;
	}


	ai_info *aip = &Ai_info[shipp->ai_index];

	// check to see if the AI mode updated
	if ((Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].ai_mode != aip->mode) 
		|| (Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].ai_submode != aip->submode) 
		|| (Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].target_signature != aip->target_signature)) {

		// send, if so.
		oo_flags |= OO_AI_NEW;

		Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].ai_mode = aip->mode;
		Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].ai_submode = aip->submode;
		Svr_frames.ship_last_sent[net_sig_idx].players[pl->player_id].target_signature = aip->target_signature;
	}


	// add info for a targeted object
	if((pl->s_info.target_objnum != -1) && (OBJ_INDEX(obj) == pl->s_info.target_objnum)){
		oo_flags |= (OO_POS_NEW | OO_ORIENT_NEW);
	}
	// all other cases
	else {					
		// add info which is contingent upon being "in front"			
		if(in_cone){
			oo_flags |= OO_ORIENT_NEW;
		}						
	}		

	// pack stuff only if we have to 	
	int packed = multi_oo_pack_data(pl, obj, oo_flags ,data);	

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
			if((Net_players[idx].m_player != NULL) && (Net_players[idx].m_player->objnum >= 0) && !(Net_players[idx].flags & NETINFO_FLAG_LIMBO) && !(Net_players[idx].flags & NETINFO_FLAG_RESPAWNING)){
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
	net_player *pl = NULL;	

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
void multi_oo_gameplay_init()
{
	mprintf(("Multi_ship_record_init called.\n"));

	int cur, idx;
	ship *shipp;

	// setup initial object update info	
	cur = 0;

	for(int s_idx=0; s_idx<MAX_SHIPS; s_idx++){
		shipp = &Ships[s_idx];
		
		// update the timestamps
		for(idx=0;idx<MAX_PLAYERS;idx++){
			shipp->np_updates[idx].update_stamp = timestamp(cur);
		} 
	}

	// Get the non-repeating parts of the struct set.
	Svr_frames.frame_timestamp_count = 0;
	for (int i = 0; i < MAX_FRAMES_RECORDED; i++) {
		Svr_frames.timestamps[i] = -1;
	}
	Svr_frames.number_of_frames = 0;
	Svr_frames.wrap_count = 0;
	Svr_frames.cur_idx = 0;
	mprintf(("Double Checking that THIS IS THE ISSUE 1!\n"));
	Svr_frames.frame_info.clear();		// Until final cleanup and testing, I'm putting this here to help facilitate testing. 
	mprintf(("Looks like we're clear for now... 1\n"));

	// Although a memory hog, this allows us to be able to set net signatures all the way to the
	// max of permanent net sigantures.  Perhaps this can be replaced later with a more elegant way to
	// check for how many ships are going to be in the mission total.
	Svr_frames.frame_info.reserve(5000); // Reserving up to the ship limit here should help optimize a little bit.

	multi_oo_server_frame_record temp_server_frame_record;
	oo_last_sent_record temp_server_last_sent;

	temp_server_frame_record.death_or_depart_frame = -1;
	temp_server_frame_record.initial_frame = -1;
	for (int i = 0; i < 240; i++) {
		temp_server_frame_record.orientations[i] = vmd_identity_matrix;
		temp_server_frame_record.positions[i] = vmd_zero_vector;
		temp_server_frame_record.rotational_velocities[i] = vmd_zero_vector;
	}

	// if the last packet sent perfect shields, we may not need to send them again.
	//SCP_vector<ubyte> subsystems;	// See if *any* of the subsystems changed, so we have to allow for a variable number of subsystems within a variable number of ships.

	for (int i = 0; i < 12; i++) {
		temp_server_last_sent.players[i].update_frame = -1;
		temp_server_last_sent.players[i].position = vmd_zero_vector;
		temp_server_last_sent.players[i].rot_vel = vmd_zero_vector;
		temp_server_last_sent.players[i].hull = 0.0f;
		temp_server_last_sent.players[i].ai_mode = 0;
		temp_server_last_sent.players[i].ai_submode = -1;
		temp_server_last_sent.players[i].target_signature = 0;
		temp_server_last_sent.players[i].perfect_shields_sent = false;		
		
		for (int j = 0; j < MAX_MODEL_SUBSYSTEMS; j++) {
			temp_server_last_sent.players[i].subsystems.push_back(0.0f);
		}
	}

	for (int i = 0; i < 5000; i++) {
		Svr_frames.frame_info.push_back(temp_server_frame_record);
		Svr_frames.ship_last_sent.push_back(temp_server_last_sent);
	}
	
	mprintf(("Double Checking that THIS IS THE ISSUE!2\n"));
	Oo_general_info.interp.clear();
	mprintf(("Looks like we're clear, for now...2\n"));
	Oo_general_info.interp.reserve(5000);

	// create a temporary struct and then stuff it for all ships.
	interp_message_tracking temp_interp;

	temp_interp.cur_pack_pos_frame = -1;
	temp_interp.prev_pack_pos_frame = -1;
	temp_interp.cur_pack_ori_frame = -1;
	temp_interp.prev_pack_ori_frame = -1;
	temp_interp.interp_count = 0;
	temp_interp.old_points = vmd_zero_vector;
	temp_interp.new_points = vmd_zero_vector;
	temp_interp.old_angles = { 0.0f, 0.0f, 0.0f };
	temp_interp.new_angles = { 0.0f, 0.0f, 0.0f };

	temp_interp.position_error = vmd_zero_vector;
	temp_interp.orientation_error = { 0.0f, 0.0f, 0.0f };

	temp_interp.splines[0] = bez_spline();
	temp_interp.splines[1] = bez_spline();

	temp_interp.ai_frame = -1;
	temp_interp.most_recent_packet = -1;
	temp_interp.minimum_packet = -1;
	temp_interp.rot_vel_frame = -1;
	temp_interp.hull_frame = -1;
	temp_interp.shields_frame = -1;

	for (int i = 0; i < MAX_MODEL_SUBSYSTEMS; i++) {
		temp_interp.subsystems_frame.push_back(-1);
	}

	temp_interp.ai_frame = -1;

	for (int i = 1; i < REAL_SHIP_SIG_MAX; i++) {
		Oo_general_info.interp.push_back(temp_interp);
	}

	Oo_general_info.received_timestamps.clear();
	Oo_general_info.ref_timestamp = 0;
	Oo_general_info.most_recent_updated_ship_index = 0;
	Oo_general_info.distance_to_most_recent = 0.0f;
	Oo_general_info.most_recent_frame = 0;

	// reset datarate stamp now
	extern int OO_gran;
	for(idx=0; idx<MAX_PLAYERS; idx++){
		Net_players[idx].s_info.rate_stamp = timestamp( (int)(1000.0f / (float)OO_gran) );
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
	if((Player_obj != NULL) && (Player_ship->flags[Ship::Ship_Flags::Dying])){
		return;
	}	
	
	// build the header
	BUILD_HEADER(OBJECT_UPDATE);		

	// pos and orient always
	oo_flags = (OO_POS_NEW | OO_ORIENT_NEW);		

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
	if(Netgame.server != NULL){								
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
	oo_flags = (OO_POS_NEW | OO_ORIENT_NEW);

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

	// increment sequence #
//	Player_ship->np_updates[idx].seq++;

	multi_io_send(&Net_players[idx], data, packet_size);
}


// updates all interpolation info for a specific ship
void multi_oo_maybe_update_interp_info(int idx, vec3d* pos, angles* ori_angles, bool adjust_pos, bool newest_pos, bool adjust_ori, bool newest_ori)
{
	// store and replace interpolation info		
	// for position
	if (adjust_pos) {
		// if this is the newest position update everything
		if (newest_pos) {
			Oo_general_info.interp[idx].old_points = Oo_general_info.interp[idx].new_points;
			Oo_general_info.interp[idx].new_points = *pos;
		} // if this is the second newest, update that instead
		else {
			Oo_general_info.interp[idx].old_points = *pos;
		}
	}

	// for orientation
	if (adjust_ori) {
		if (newest_ori) {
			Oo_general_info.interp[idx].old_angles = Oo_general_info.interp[idx].new_angles;
			Oo_general_info.interp[idx].new_angles = *ori_angles;
		}		else {
			Oo_general_info.interp[idx].old_angles = *ori_angles;
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
	if(Net_player == NULL){
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
	if(Net_player == NULL){
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
	if((Net_player == NULL) || !(Net_player->flags & NETINFO_FLAG_AM_MASTER)){
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

// interp
void multi_oo_interp(object *objp)
{		
	// make sure its a valid ship
	Assert(Game_mode & GM_MULTIPLAYER);
	if(objp->type != OBJ_SHIP){
		return;
	}
	if((objp->instance < 0) || (objp->instance >= MAX_SHIPS)){
		return;
	}	

	// Now that we know we have a valid ship, do stream weapon firing for this ship before we do anything else that makes us abort
	Assert(objp != Player_obj);
	if (objp != Player_obj) {
		ship_fire_primary(objp, 1, 0);
	}
	mprintf(("Net_signature is .... %d \n", objp->net_signature));
	int net_sig_idx = objp->net_signature - 1;

	mprintf(("Where am I crasing 1\n"));

	// if this ship doesn't have enough data points yet, pretend it's a normal ship and skip it
	if(Oo_general_info.interp[net_sig_idx].interp_count < 2){
		mprintf(("Where am I crasing 2\n"));
		physics_sim(&objp->pos, &objp->orient, &objp->phys_info, flFrametime);
		return;
	}
	mprintf(("Where am I crasing 3\n"));

	// Calculate how much time is between the last two packets to see if we're about to overshoot the current packet.	
	int temp_diff = multi_oo_calc_pos_time_difference(net_sig_idx);

	if (temp_diff < 0) {
		
	}

	// Cyborg17 - Here's the new timing calculation: we subtract the last packet's arrival time oo_arrive_time[objp->instance][oo_arrive_time_count[objp->instance] - 1]
	// from the current frame time (f2fl(game_get_overall_frametime()) to see how long it's been, and then we divide by the average difference in time.  This gives us a
	// percent that tells us, for example, "35% of the time has elapsed until the next packet."
	float t = (float)((timestamp() - 0)/* what time the packet came in */ / temp_diff);

	// we've overshot. hmm. just keep the sim running I guess	
	if(t > 1.0f){

		physics_sim(&objp->pos, &objp->orient, &objp->phys_info, flFrametime);

		return;
	// in case they are slightly late, we'll just use the last good point.
	} else if (t > 1.0f) {

		t = 1.0f;
	}	

	// Cyborg17 - we are no longer blending two interpolation curves.  I'm not sure *how*, but they were somehow making it look 
	// less erratic when the timing was broken in the first place.
	float u = 0.5f + (t * 0.5f);

	vec3d interp_point;

	Oo_general_info.interp[net_sig_idx].splines[0].bez_get_point(&interp_point, u);

	vm_vec_scale(&interp_point, t);

	// Even thought we're not blending two curves, there is some small error introduced by the client's simulation that can be 
	// handled by removing "t" portion of it. If we've gotten to this point the ship is not so far from the interpolation that
	// we bash, and the partial removals should not be noticable... theoretically.  
	vm_vec_scale_add(&objp->pos, &interp_point, &Oo_general_info.interp[net_sig_idx].position_error, 1.0f - t);
	
	// run the sim for rotation	
	// TODO: add a bezier for rot_vel.  It was working really well previously when everything looked bad, but now it looks slightly choppy.
	physics_sim_rot(&objp->orient, &objp->phys_info, flFrametime);

}

float oo_error = 0.8f;
DCF(oo_error, "Sets error factor for flight path prediction physics (Multiplayer)")
{
	if (dc_optional_string_either("help", "--help")) {
		dc_printf("Usage: oo_error <value>\n");
		return;
	}

	if (dc_optional_string_either("status", "--status") || dc_optional_string_either("?", "--?")) {
		dc_printf("oo_error is currently %f", oo_error);
		return;
	}

	dc_stuff_float(&oo_error);
	
	dc_printf("oo_error set to %f", oo_error);
}

void multi_oo_calc_interp_splines(int net_signature, vec3d *cur_pos, physics_info *cur_phys_info, vec3d *new_pos, matrix *new_orient, physics_info *new_phys_info)
{
	vec3d a, b, c;
	matrix m_copy;
	physics_info p_copy;
	vec3d *pts[3] = {&a, &b, &c};

	// average time between packets
	int interp_interval = multi_oo_calc_pos_time_difference(net_signature);

	// would this cause us to rubber-band?
	vec3d v_norm = cur_phys_info->vel;	
	vec3d v_dir;
	vm_vec_sub(&v_dir, new_pos, cur_pos);	
	if(!IS_VEC_NULL_SQ_SAFE(&v_norm) && !IS_VEC_NULL_SQ_SAFE(&v_dir)){
		vm_vec_normalize(&v_dir);
		vm_vec_normalize(&v_norm);	
		if(vm_vec_dot(&v_dir, &v_norm) < 0.0f){
			*new_pos = *cur_pos;
		}
	}

	// get the spline representing where this new point tells us we'd be heading
	a = Oo_general_info.interp[net_signature].old_points; //-V519
	b = Oo_general_info.interp[net_signature].new_points; //-V519
	c = vmd_zero_vector;
	m_copy = *new_orient;
	p_copy = *new_phys_info;
	physics_sim(&c, &m_copy, &p_copy, interp_interval);			// next point, given this new info
	Oo_general_info.interp[net_signature].splines[0].bez_set_points(3, pts);

}

// Calculates how much time has gone by between the two most recent frames 
int multi_oo_calc_pos_time_difference(int net_sig_idx) {

	// make sure we have enough packets so far.
	if (Oo_general_info.interp[net_sig_idx].prev_pack_pos_frame == -1 || Oo_general_info.interp[net_sig_idx].cur_pack_pos_frame == -1 ) {
		
	}

}

int display_oo_bez = 0;
DCF(bez, "Toggles rendering of player ship trajectory interpolation splines (Multiplayer) *disabled*")
{
	if (dc_optional_string_either("status", "--status") || dc_optional_string_either("?", "--?")) {
		dc_printf("Rendering of interpolation splines is '%s'", display_oo_bez ? "ON" : "OFF");
		return;
	}

	display_oo_bez = !display_oo_bez;

	dc_printf("%showing interp splines", display_oo_bez ? "S" : "Not s");
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
