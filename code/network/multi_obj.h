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
struct physics_info;
struct weapon;


// client button info flags
#define OOC_FIRE_CONTROL_PRESSED			(1<<0)
#define OOC_TARGET_SEEK_LOCK		(1<<1)
#define OOC_TRIGGER_DOWN			(1<<2)
#define OOC_PRIMARY_BANK			(1<<3)
#define OOC_PRIMARY_LINKED			(1<<4)
#define OOC_AFTERBURNER_ON			(1<<5)
// NOTE: no additional flags here unless it's sent in an extra data byte

// Cyborg17, Server will be tracking only the last second of frames
#define MAX_FRAMES_RECORDED		30
#define PRIMARY_PACKET_CUTOFF			2000

// ---------------------------------------------------------------------------------------------------
// POSITION AND ORIENTATION RECORDING
// if it breaks, find Cyborg17 so you can yell at him
// This section is almost all server side

// Add a new ship *ON IN-GAME SHIP CREATION* to the tracking struct
void multi_ship_record_add_ship(int obj_num);

// Update the tracking struct whenver the object is updated in-game
void multi_ship_record_update_all();

// increment the tracker per frame, before incoming object packets are processed
void multi_ship_record_increment_frame();

// returns the last frame's index.
int multi_find_prev_frame_idx();

// figure out what was the correct wrap
ushort multi_ship_record_calculate_wrap(ushort combined_frame);

// find the right frame to start our weapon simulation
int multi_ship_record_find_frame(ushort client_frame, ushort wrap, int time_elapsed);

// verify that a given frame exists for a given ship, requires the sequence number that the client sends.
bool multi_ship_record_verify_frame(object* objp, int seq_num);

// a quick lookups for position and orientation
vec3d multi_ship_record_lookup_position(object* objp, int frame);

// a quick lookups for orientation
matrix multi_ship_record_lookup_orientation(object* objp, int frame);

// quickly lookup how much time has passed since the given frame.
uint multi_ship_record_get_time_elapsed(int original_frame, int new_frame);

int multi_ship_record_find_time_after_frame(int client_frame, int frame, int time_elapsed);

// This stores the information we got from the client to create later, and checks to see if this is the oldest shot we are going to fire during rollback.
void multi_ship_record_add_rollback_shot(object* pobjp, vec3d* pos, matrix* orient, int frame, bool secondary);

// Lookup whether rollback mode is on
bool multi_ship_record_get_rollback_wep_mode();

// Adds a weapon to the rollback tracker.
void multi_ship_record_add_rollback_wep(int wep_objnum);

// Manage rollback for a frame
void multi_ship_record_do_rollback();

// fire the rollback weapons that are in the rollback struct
void multi_oo_fire_rollback_shots(int frame_idx);

// moves all rollbacked ships back to the original frame
void multi_oo_restore_frame(int frame_idx);

// pushes the rollback weapons forward for a single rollback frame.
void multi_oo_simulate_rollback_shots(int frame_idx);

// restores ships to the positions they were in bedfore rollback.
void multi_record_restore_positions();



// ---------------------------------------------------------------------------------------------------
// Client side frame tracking, for now used only to help lookup info from packets to improve client accuracy.
// 

// See if a newly arrived packet is a good new option as a reference object
void multi_ship_record_rank_seq_num(object* objp, ushort seq_num);

// Quick lookup for the most recently received frame
ushort multi_client_lookup_ref_obj_net_sig();

// Quick lookup for the most recently received frame
ushort multi_client_lookup_frame_idx();

// Quick lookup for the most recently received timestamp.
int multi_client_lookup_frame_timestamp();

// Quick lookup for the most recent received frametime from the server.
int multi_client_lookup_current_frametime();

// reset all the necessary info for respawning player.
void multi_oo_respawn_reset_info(ushort net_sig);

// ---------------------------------------------------------------------------------------------------
// OBJECT UPDATE FUNCTIONS
//

// process all object update details for this frame
void multi_oo_process();

// process incoming object update data
void multi_oo_process_update(ubyte *data, header *hinfo);

// initialize all object update timestamps (call whenever entering gameplay state)
void multi_init_oo_and_ship_tracker();

// send control info for a client (which is basically a "reverse" object update)
void multi_oo_send_control_info();
void multi_oo_send_changed_object(object *changedobj);

// helper function that updates all interpolation info for a specific ship from a packet
void multi_oo_maybe_update_interp_info(object* objp, vec3d* new_pos, angles* new_ori_angles, matrix* new_ori_mat, physics_info* new_phys_info, bool adjust_pos, bool newest_pos);

// reset all sequencing info
void multi_oo_reset_sequencing();

// is this object one which needs to go through the interpolation
int multi_oo_is_interp_object(object *objp);

// interp position and orientation
void multi_oo_interp(object *objp);

// recalculate how much time is between position packets
float multi_oo_calc_pos_time_difference(int net_sig_idx);

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
