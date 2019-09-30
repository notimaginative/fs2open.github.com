/*
 * Copyright (C) Volition, Inc. 1999.  All rights reserved.
 *
 * All source code herein is the property of Volition, Inc. You may not sell 
 * or otherwise commercially exploit the source or things you created based on the 
 * source.
 *
*/

#ifndef _MULTI_NEW_OBJECT_UPDATE_HEADER_FILE
#define _MULTI_NEW_OBJECT_UPDATE_HEADER_FILE

#include "math/vecmat.h"

// ---------------------------------------------------------------------------------------------------
// OBJECT UPDATE DEFINES/VARS
//
#include "globalincs/pstypes.h"

struct interp_info;
class object;
struct header;
struct net_player;
class ship;


// client button info flags
#define OOC_FIRE_SECONDARY			(1<<0)
#define OOC_TARGET_LOCKED			(1<<1)
#define OOC_TARGET_SEEK_LOCK		(1<<2)
#define OOC_LOCKING_ON_CENTER		(1<<3)
#define OOC_TRIGGER_DOWN			(1<<4)
#define OOC_PRIMARY_BANK			(1<<5)
#define OOC_PRIMARY_LINKED			(1<<6)
#define OOC_AFTERBURNER_ON			(1<<7)
// NOTE: no additional flags here unless it's sent in an extra data byte

// Cyborg17, Server will be tracking the last two seconds of frames
#define MAX_FRAMES_RECORDED		240
#define PRIMARY_PACKET_CUTOFF			2000

// update info
typedef struct np_update {	
	int		update_stamp;				// global update stamp
} np_update;

// ---------------------------------------------------------------------------------------------------
// POSITION AND ORIENTATION RECORDING
// if it breaks, find Cyborg17 so you can yell at him
// This section is almost all server side

// Add a new ship *ON IN-GAME SHIP CREATION* to the tracking struct
void multi_ship_record_add_ship_server(int obj_num, bool in_mission = true);

// Mark when a ship died or departed....
void multi_ship_record_mark_as_dead_or_departed(int shipnum);

// Update the tracking struct whenver the object is updated in-game
void multi_ship_record_update_all();

// increment the tracker per frame, before incoming object packets are processed
void multi_ship_record_increment_frame();

// returns the last frame's index.
int multi_find_prev_frame_idx();

// figure out what was the correct wrap
ubyte multi_ship_record_calculate_wrap(ushort combined_frame);

// find the right frame to start our weapon simulation
int multi_ship_record_find_frame(ushort client_frame, ubyte wrap, int time_elapsed);

// verify that a given frame exists for a given ship, requires the sequence number that the client sends.
bool multi_ship_record_verify_frame(object* objp, int seq_num);

// a quick lookups for position and orientation
vec3d multi_ship_record_lookup_position(object* objp, int frame);

// a quick lookups for orientation
matrix multi_ship_record_lookup_orientation(object* objp, int frame);

// quickly lookup how much time has passed since the given frame.
uint multi_ship_record_get_time_elapsed(ushort original_frame);

int multi_ship_record_adjust_timestamp(int client_frame, int frame, int time_elapsed);

// find the exact point on the server that the client sees by interpolating 
void multi_ship_record_interp_between_frames(vec3d *interp_pos, matrix *interp_ori, int shipnum, ushort original_frame, int time_elapsed);

// ULTIMATE TODO, write new function that manages collision detection for inbetween frames.

// Clear all from tracking struct
void multi_ship_record_clear_all();

// ---------------------------------------------------------------------------------------------------
// Client side frame tracking, for now used only to help lookup info from packets to improve client accuracy.
// 

// For 
void multi_ship_record_add_ship_client(int obj_num);

// See if a newly arrived packet is a good new option as a reference object
void multi_ship_record_rank_seq_num(object* objp, ushort seq_num);

// Quick lookup for the most recently received frame
ushort multi_client_lookup_frame_ship_index();

// Quick lookup for the most recently received frame
ushort multi_client_lookup_frame_idx();

// Quick lookup for the most recently received timestamp.
int multi_client_lookup_frame_timestamp();

// ---------------------------------------------------------------------------------------------------
// OBJECT UPDATE FUNCTIONS
//

// process all object update details for this frame
void multi_oo_process();

// process incoming object update data
void multi_oo_process_update(ubyte *data, header *hinfo);

// initialize all object update timestamps (call whenever entering gameplay state)
void multi_oo_gameplay_init();

// send control info for a client (which is basically a "reverse" object update)
void multi_oo_send_control_info();
void multi_oo_send_changed_object(object *changedobj);

// helper function that updates all interpolation info for a specific ship from a packet
void multi_oo_maybe_update_interp_info(int idx, vec3d* pos, angles* ori_angles, bool adjust_pos, bool newest_pos, bool adjust_ori, bool newest_ori);

// reset all sequencing info
void multi_oo_reset_sequencing();

// is this object one which needs to go through the interpolation
int multi_oo_is_interp_object(object *objp);

// interp position and orientation
void multi_oo_interp(object *objp);

int multi_oo_calc_pos_time_difference(int net_sig_idx);

// Cyborg17 - sort through subsystems to make sure we only update the ones we need to update.
//int multi_pack_required_subsytems(ship* shipp, ubyte* data, int packet_size, int header_bytes);


// ---------------------------------------------------------------------------------------------------
// DATARATE DEFINES/VARS
//

#define OO_HIGH_RATE_DEFAULT				11000


// ---------------------------------------------------------------------------------------------------
// DATARATE FUNCTIONS
//

// process all object update datarate details
void multi_oo_rate_process();

// initialize all datarate checking stuff
void multi_oo_rate_init_all();

// initialize the rate limiting for the passed in player
void multi_oo_rate_init(net_player *pl);

// if the given net-player has exceeded his datarate limit, or if the overall datarate limit has been reached
int multi_oo_rate_exceeded(net_player *pl);

// if it is ok for me to send a control info (will be ~N times a second)
int multi_oo_cirate_can_send();

// display any oo info on the hud
void multi_oo_display();

// notify of a player join
void multi_oo_player_reset_all(net_player *pl = NULL);

#endif
