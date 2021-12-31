/*
 * Copyright (C) Volition, Inc. 1999.  All rights reserved.
 *
 * All source code herein is the property of Volition, Inc. You may not sell 
 * or otherwise commercially exploit the source or things you created based on
 * the source.
 */


#ifndef _FREESPACE_STANDALONE_SERVER_HEADER_FILE
#define _FREESPACE_STANDALONE_SERVER_HEADER_FILE

// ----------------------------------------------------------------------------------------
// external variables
//

struct net_player;

// ----------------------------------------------------------------------------------------
// generic dialog functions
//

// create the validate dialog 
void std_create_gen_dialog(const char *title);

// kill the validate dialog();
void std_destroy_gen_dialog();

// set the text in the filename of the validate dialog
// valid values for field_num == 0 .. 2
void std_gen_set_text(const char *str, int field_num);


// ----------------------------------------------------------------------------------------
// connection page/tab functions
//

// set the text box indicating how many players are connected, returning the determined count
int std_connect_set_connect_count();

// set the connect status (connected or not) of the game host
void std_connect_set_host_connect_status();

// set the game name for the standalone. passing NULL uses the default
void std_connect_set_gamename(const char *name);


// ----------------------------------------------------------------------------------------
// multiplayer page/tab functions
//

// set the mission time in seconds
void std_multi_set_standalone_missiontime(float mission_time);

// set the mission name
void std_multi_set_standalone_mission_name(const char *mission_name);

// initialize the goal tree for this mission 
void std_multi_setup_goal_tree();

// add all the goals from the current mission to the tree control
void std_multi_add_goals();

// update all the goals in the goal tree based upon the mission status
void std_multi_update_goals();

// update the netgame information area controls with the current Netgame settings
void std_multi_update_netgame_info_controls();


// ---------------------------------------------------------------------------------------
// debug page/tab functions
//

// set the text on the standalones state indicator box
void std_debug_set_standalone_state_string(const char *str);


// ---------------------------------------------------------------------------------------
// general functions
// 

// add a player and take care of updating all gui/data details
void std_add_player(net_player *p);

// remove a player and take care of updateing all gui/data details
int std_remove_player(net_player *p);

// set any relevant controls which display the framerate of the standalone
void std_set_standalone_fps(float fps);

// update any relveant controls which display the ping for the given player
void std_update_player_ping(net_player *p);

// reset all gui stuff for the standalone
void std_reset_standalone_gui();

// close down the standalone
void std_deinit_standalone();

// initialize the standalone
void std_init_standalone();

// do any gui related issues on the standalone (like periodically updating player stats, etc...)
void std_do_gui_frame();

// notify the user that the standalone has failed to login to the tracker on startup
void std_tracker_notify_login_fail();

// attempt to log the standalone into the tracker
void std_tracker_login();

// reset all stand gui timestamps
void std_reset_timestamps();

// add a line of text chat to the standalone
void std_add_chat_text(const char *text, int player_index, int add_id);

// if the standalone is host password protected
int std_is_host_passwd();

// if the given callsign is banned from the server
int std_player_is_banned(const char *name);

// add a callsign to the ban list
void std_add_ban(const char *name);

// add a line from the multi log
void std_multilog_add_line(const char *line);

#endif

