/*
 * Copyright (C) Volition, Inc. 2005.  All rights reserved.
 * 
 * All source code herein is the property of Volition, Inc. You may not sell 
 * or otherwise commercially exploit the source or things you created based on the 
 * source.
 *
*/



#ifdef _WIN32
#include <winsock2.h>
#else
#include <sys/wait.h>
#endif

#include "network/multi_pxo.h"
#include "anim/animplay.h"
#include "ui/ui.h"
#include "io/key.h"
#include "bmpman/bmpman.h"
#include "gamesnd/gamesnd.h"
#include "gamesequence/gamesequence.h"
#include "cfile/cfile.h"
#include "network/chat_api.h"
#include "popup/popup.h"
#include "freespace.h"
#include "graphics/font.h"
#include "network/multi.h"
#include "network/multiui.h"
#include "network/multi_log.h"
#include "stats/medals.h"
#include "globalincs/alphacolors.h"
#include "graphics/2d.h"
#include "graphics/generic.h"
#include "io/timer.h"
#include "inetfile/inetgetfile.h"
#include "cfile/cfilesystem.h"
#include "options/OptionsManager.h"
#include "osapi/osregistry.h"
#include "parse/parselo.h"
#include "stats/scoring.h"
#include "playerman/player.h"
#include "network/multi_fstracker.h"
#include "network/ptrack.h"
#include "network/gtrack.h"
#include "menuui/mainhallmenu.h"
#include "debugconsole/console.h"



// ----------------------------------------------------------------------------------------------------
// PXO DEFINES/VARS
//

// button definitions
#define MULTI_PXO_NUM_BUTTONS				15
#define MULTI_PXO_PLIST_UP					0
#define MULTI_PXO_PLIST_DOWN				1
#define MULTI_PXO_RANKINGS					2
#define MULTI_PXO_PINFO						3
#define MULTI_PXO_FIND						4
#define MULTI_PXO_MOTD						5
#define MULTI_PXO_JOIN						6
#define MULTI_PXO_JOIN_PRIV				7
#define MULTI_PXO_CHAN_UP					8
#define MULTI_PXO_CHAN_DOWN				9
#define MULTI_PXO_TEXT_UP					10
#define MULTI_PXO_TEXT_DOWN				11
#define MULTI_PXO_EXIT						12
#define MULTI_PXO_HELP						13
#define MULTI_PXO_GAMES						14


ui_button_info Multi_pxo_buttons[GR_NUM_RESOLUTIONS][MULTI_PXO_NUM_BUTTONS] = {
	{ // GR_640
		ui_button_info( "PXB_00",		1,		104,	-1,	-1,	0 ),					// scroll player list up
		ui_button_info( "PXB_01",		1,		334,	-1,	-1,	1 ),					// scroll player list down
		ui_button_info( "PXB_02",		18,	385,	-1,	-1,	2 ),					// rankings webpage
		ui_button_info( "PXB_03",		71,	385,	-1,	-1,	3 ),					// pilot info
		ui_button_info( "PXB_04",		115,	385,	-1,	-1,	4 ),					// find player
		ui_button_info( "PXB_05",		1,		443,	-1,	-1,	5 ),					// motd
		ui_button_info( "PXB_06",		330,	96,	-1,	-1,	6 ),					// join channel
		ui_button_info( "PXB_07",		330,	131,	-1,	-1,	7 ),					// join private channel
		ui_button_info( "PXB_08",		618,	92,	-1,	-1,	8 ),					// scroll channels up
		ui_button_info( "PXB_09",		618,	128,	-1,	-1,	9 ),					// scroll channels down
		ui_button_info( "PXB_10",		615,	171,	-1,	-1,	10 ),					// scroll text up
		ui_button_info( "PXB_11",		615,	355,	-1,	-1,	11 ),					// scroll text down
		ui_button_info( "PXB_12",		482,	435,	-1,	-1,	12 ),					// exit
		ui_button_info( "PXB_13",		533,	432,	-1,	-1,	13 ),					// help		
		ui_button_info( "PXB_14",		573,	432,	-1,	-1,	14 ),					// games list
	},
	{ // GR_1024
		ui_button_info( "2_PXB_00",		2,		166,	-1,	-1,	0 ),					// scroll player list up
		ui_button_info( "2_PXB_01",		2,		534,	-1,	-1,	1 ),					// scroll player list down
		ui_button_info( "2_PXB_02",		29,	616,	-1,	-1,	2 ),					// rankings webpage
		ui_button_info( "2_PXB_03",		114,	616,	-1,	-1,	3 ),					// pilot info
		ui_button_info( "2_PXB_04",		184,	616,	-1,	-1,	4 ),					// find player
		ui_button_info( "2_PXB_05",		2,		709,	-1,	-1,	5 ),					// motd
		ui_button_info( "2_PXB_06",		528,	119,	-1,	-1,	6 ),					// join channel
		ui_button_info( "2_PXB_07",		528,	175,	-1,	-1,	7 ),					// join private channel
		ui_button_info( "2_PXB_08",		989,	112,	-1,	-1,	8 ),					// scroll channels up
		ui_button_info( "2_PXB_09",		989,	170,	-1,	-1,	9 ),					// scroll channels down
		ui_button_info( "2_PXB_10",		984,	240,	-1,	-1,	10 ),					// scroll text up
		ui_button_info( "2_PXB_11",		984,	568,	-1,	-1,	11 ),					// scroll text down
		ui_button_info( "2_PXB_12",		771,	696,	-1,	-1,	12 ),					// exit
		ui_button_info( "2_PXB_13",		853,	691,	-1,	-1,	13 ),					// help		
		ui_button_info( "2_PXB_14",		917,	691,	-1,	-1,	14 ),					// games list
	},
};

#define MULTI_PXO_NUM_TEXT			16
UI_XSTR Multi_pxo_text[GR_NUM_RESOLUTIONS][MULTI_PXO_NUM_TEXT] = {
	{ // GR_640		
		{"Web",								1313, 20,	415,	UI_XSTR_COLOR_GREEN, -1, &Multi_pxo_buttons[0][MULTI_PXO_RANKINGS].button},
		{"Ranking",							1314, 6,		426,	UI_XSTR_COLOR_GREEN, -1, &Multi_pxo_buttons[0][MULTI_PXO_RANKINGS].button},
		{"Pilot",							1310,	68,	415,	UI_XSTR_COLOR_GREEN, -1, &Multi_pxo_buttons[0][MULTI_PXO_PINFO].button},
		{"Info",								1311,	72,	426,	UI_XSTR_COLOR_GREEN, -1, &Multi_pxo_buttons[0][MULTI_PXO_PINFO].button},
		{"Find",								1315,	119,	415,	UI_XSTR_COLOR_GREEN, -1, &Multi_pxo_buttons[0][MULTI_PXO_FIND].button},
		{"Motd",								1316,	36,	456,	UI_XSTR_COLOR_GREEN, -1, &Multi_pxo_buttons[0][MULTI_PXO_MOTD].button},	
		{"Join",								1505,	291,	100,	UI_XSTR_COLOR_PINK, -1, &Multi_pxo_buttons[0][MULTI_PXO_JOIN].button},
		{"Channel",							1317,	266,	112,	UI_XSTR_COLOR_PINK, -1, &Multi_pxo_buttons[0][MULTI_PXO_JOIN].button},	
		{"Join",								1506,	291,	134,	UI_XSTR_COLOR_PINK, -1, &Multi_pxo_buttons[0][MULTI_PXO_JOIN_PRIV].button},
		{"Private",							1318,	273,	146,	UI_XSTR_COLOR_PINK, -1, &Multi_pxo_buttons[0][MULTI_PXO_JOIN_PRIV].button},
		{"Exit",								1416,	493,	424,	UI_XSTR_COLOR_PINK, -1, &Multi_pxo_buttons[0][MULTI_PXO_EXIT].button},
		{"Help",								928,	535,	416,	UI_XSTR_COLOR_GREEN,	-1, &Multi_pxo_buttons[0][MULTI_PXO_HELP].button},
		{"Games",							1319,	579,	416,	UI_XSTR_COLOR_PINK, -1, &Multi_pxo_buttons[0][MULTI_PXO_GAMES].button},
		{"Players",							1269,	29,	102,	UI_XSTR_COLOR_GREEN,	-1, NULL},
		{"Players",							1269,	507,	90,	UI_XSTR_COLOR_GREEN,	-1, NULL},		
		{"Games",							1319,	568,	90,	UI_XSTR_COLOR_GREEN, -1, NULL}
	},
	{ // GR_1024
		{"Web",								1313, 32,	664,	UI_XSTR_COLOR_GREEN, -1, &Multi_pxo_buttons[1][MULTI_PXO_RANKINGS].button},
		{"Ranking",							1314, 9,		674,	UI_XSTR_COLOR_GREEN, -1, &Multi_pxo_buttons[1][MULTI_PXO_RANKINGS].button},
		{"Pilot",							1310,	109,	664,	UI_XSTR_COLOR_GREEN, -1, &Multi_pxo_buttons[1][MULTI_PXO_PINFO].button},
		{"Info",								1311,	115,	674,	UI_XSTR_COLOR_GREEN, -1, &Multi_pxo_buttons[1][MULTI_PXO_PINFO].button},
		{"Find",								1315,	190,	664,	UI_XSTR_COLOR_GREEN, -1, &Multi_pxo_buttons[1][MULTI_PXO_FIND].button},
		{"Motd",								1316,	58,	729,	UI_XSTR_COLOR_GREEN, -1, &Multi_pxo_buttons[1][MULTI_PXO_MOTD].button},	
		{"Join",								1505,	488,	129,	UI_XSTR_COLOR_PINK, -1, &Multi_pxo_buttons[1][MULTI_PXO_JOIN].button},
		{"Channel",							1317,	461,	139,	UI_XSTR_COLOR_PINK, -1, &Multi_pxo_buttons[1][MULTI_PXO_JOIN].button},	
		{"Join",								1506,	487,	184,	UI_XSTR_COLOR_PINK, -1, &Multi_pxo_buttons[1][MULTI_PXO_JOIN_PRIV].button},
		{"Private",							1318,	467,	194,	UI_XSTR_COLOR_PINK, -1, &Multi_pxo_buttons[1][MULTI_PXO_JOIN_PRIV].button},
		{"Exit",								1416,	789,	678,	UI_XSTR_COLOR_PINK, -1, &Multi_pxo_buttons[1][MULTI_PXO_EXIT].button},
		{"Help",								928,	857,	667,	UI_XSTR_COLOR_GREEN,	-1, &Multi_pxo_buttons[1][MULTI_PXO_HELP].button},
		{"Games",							1319,	917,	667,	UI_XSTR_COLOR_PINK, -1, &Multi_pxo_buttons[1][MULTI_PXO_GAMES].button},
		{"Players",							1269,	47,	163,	UI_XSTR_COLOR_GREEN,	-1, NULL},
		{"Players",							1269,	852,	109,	UI_XSTR_COLOR_GREEN,	-1, NULL},		
		{"Games",							1319,	926,	109,	UI_XSTR_COLOR_GREEN, -1, NULL}
	}
};

char Multi_pxo_bitmap_fname[GR_NUM_RESOLUTIONS][MAX_FILENAME_LEN] = {
	"PXOChat",
	"2_PXOChat"
};
char Multi_pxo_mask_fname[GR_NUM_RESOLUTIONS][MAX_FILENAME_LEN] = {
	"PXOChat-M",
	"2_PXOChat-M"
};

UI_WINDOW Multi_pxo_window;
int Multi_pxo_bitmap = -1;
int Multi_pxo_palette = -1;


// pxo animation
#define MULTI_PXO_ANIM_FNAME				"pxologo"
#define MULTI_PXO_ANIM_X					0
#define MULTI_PXO_ANIM_Y					4
generic_anim Multi_pxo_anim;

// rankings last clicked time
#define MULTI_PXO_RANK_TIME				(5.0f)	
float Multi_pxo_ranking_last = -1.0f;

// chat api vars
int Multi_pxo_must_connect = 0;					// if we still need to connect
int Multi_pxo_connected = 0;						// if we are connected
int Multi_pxo_must_validate = 0;					// if we need to validate on the tracker
int Multi_pxo_must_autojoin = 1;					// still need to autojoin a channel

// mode
#define MULTI_PXO_MODE_NORMAL			0			// normal mode
#define MULTI_PXO_MODE_PRIVATE		1			// private channel popup
#define MULTI_PXO_MODE_FIND			2			// find player popup
int Multi_pxo_mode = MULTI_PXO_MODE_NORMAL;

// our nick for this session
char Multi_pxo_nick[NAME_LENGTH+1];

// check for button presses
void multi_pxo_check_buttons();

// handle a button press
void multi_pxo_button_pressed(int n);

// condition function for popup_do_with_condition for connected to Parallax Online
// return 10 : on successful connect
int multi_pxo_connect_do();

// attempt to connect to Parallax Online, return success or fail
int multi_pxo_connect();

// process a "nick" change event
void multi_pxo_process_nick_change(char *data);

// blit everything on the "normal" screen
void multi_pxo_blit_all();

// get selected player information
void multi_pxo_get_data(char *name);

// handle being kicked
void multi_pxo_handle_kick();

// handle being disconnected
void multi_pxo_handle_disconnect();

// return string2, which is the first substring of string 1 without a space
// it is safe to pass the same pointer for both parameters
void multi_pxo_strip_space(char *string1,char *string2);

// unload the palette
void multi_pxo_unload_palette();

// if we're currently on a private channel
int multi_pxo_on_private_channel();

// convert string 1 into string 2, substituting underscores for spaces
void multi_pxo_underscore_nick(char *string1,char *string2);

// if the command is a potential "nick" command
int multi_pxo_is_nick_command(char *msg);


// status bar stuff -----------------------------------------------
int Multi_pxo_status_coords[GR_NUM_RESOLUTIONS][4] = {
	{ // GR_640
		95, 467, 354, 12
	},
	{ // GR_1024
		152, 750, 570, 12
	},
};

// the status text itself
char Multi_pxo_status_text[MAX_PXO_TEXT_LEN];

// set the status text
void multi_pxo_set_status_text(const char *txt);

// blit the status text
void multi_pxo_blit_status_text();

// some convenient macros
#define SWITCHING_CHANNELS() (Multi_pxo_channel_switch.num_users != -1)
#define ON_CHANNEL() (Multi_pxo_channel_current.num_users != -1)

// last channel we were on before going to the game list screen
char Multi_pxo_channel_last[MAX_CHANNEL_NAME_LEN+1] = "";
int Multi_pxo_use_last_channel = 0;

// all channels which are prefixed with this are "lobby" channels
#define MULTI_PXO_AUTOJOIN_PREFIX					"#lobby"	

// join this channel to get put in an appropriate lobby channel
#define MULTI_PXO_AUTOJOIN_CHANNEL					"#autoselect"

int Multi_pxo_chan_coords[GR_NUM_RESOLUTIONS][4] = {
	{ // GR_640
		369, 101, 241, 60
	},
	{ // GR_1024
		593, 124, 386, 100
	},
};

// this is the offset from the RIGHT side of the channel box
#define CHAN_PLAYERS_COLUMN		0
#define CHAN_GAMES_COLUMN			1
static int Multi_pxo_chan_column_offsets[GR_NUM_RESOLUTIONS][2] = {
	{ 81, 26 },
	{ 103, 35 }
};

#define CHANNEL_REFRESH_TIME			(75.0f)
float Multi_pxo_channel_last_refresh = -1.0f;

#define CHANNEL_SERVER_REFRESH_TIME	(35.0f)
float Multi_pxo_channel_server_refresh = -1.0f;

int Multi_pxo_max_chan_display[GR_NUM_RESOLUTIONS] = {
	6,		// GR_640
	10		// GR_1024
};

UI_BUTTON Multi_pxo_channel_button;

// the list of available (displayed) channels
SCP_vector<pxo_channel> Multi_pxo_channels;

// item we're going to start displaying at
int Multi_pxo_channel_start = 0;
int Multi_pxo_channel_start_index = -1;

// items we've currently got selected
int Multi_pxo_channel_select = 0;

// channel we're currently connected to, num_users == -1, if we're not connected
pxo_channel Multi_pxo_channel_current;

// channel we're currently trying to change to, num_users == -1, if we're not trying to change channels
pxo_channel Multi_pxo_channel_switch;

// get a list of channels on the server (clear any old list as well)
void multi_pxo_get_channels();

// clear the old channel list
void multi_pxo_clear_channels();

// parse the input string and make a list of new channels
void multi_pxo_make_channels(char *chan_str);

// create a new channel with the given name and place it on the channel list, return a pointer or NULL on fail
pxo_channel *multi_pxo_add_channel(char *name);

// lookup a channel with the specified name
pxo_channel *multi_pxo_find_channel(char *name);

// process the channel list (select, etc)
void multi_pxo_process_channels(bool api_access);

// display the channel list
void multi_pxo_blit_channels();

// scroll channel list up
void multi_pxo_scroll_channels_up();

// scroll channel list down
void multi_pxo_scroll_channels_down();

// attempt to join a channel
void multi_pxo_join_channel(pxo_channel *chan);

// handle any processing details if we're currently trying to join a channel
void multi_pxo_handle_channel_change();

// autojoin an appropriate channel
void multi_pxo_autojoin();

// does the string match the "autojoin" prefic
int multi_pxo_is_autojoin(char *name);

// send a request to refresh our channel server counts
void multi_pxo_channel_refresh_servers();

// refresh current channel server count
void multi_pxo_channel_refresh_current();

// channel list region
int Multi_pxo_player_coords[GR_NUM_RESOLUTIONS][4] = {
	{ // GR_640
		27, 121, 141, 261
	},
	{ // GR_1024
		43, 194, 154, 417
	},
};

int Multi_pxo_max_player_display[GR_NUM_RESOLUTIONS] = {
	25,	// GR_640
	41		// GR_1024
};
UI_BUTTON Multi_pxo_player_button;

// slider coords
int Multi_pxo_player_slider_coords[GR_NUM_RESOLUTIONS][4] = {
	{ // GR_640
		1, 139, 21, 192
	},
	{ // GR_1024
		2, 219, 33, 314
	}
};
const char *Multi_pxo_player_slider_name[GR_NUM_RESOLUTIONS] = {
	"slider",				// GR_640
	"2_slider"			// GR_1024
};

// the list of players in the current channel
SCP_vector<SCP_string> Multi_pxo_players;

// item we're going to start displaying at
int Multi_pxo_player_start = 0;
int Multi_pxo_player_start_index = -1;

// items we've currently got selected
int Multi_pxo_player_select = -1;

// clear the old player list
void multi_pxo_clear_players();

// create a new player with the given name and place it on the player list
void multi_pxo_add_player(const char *name);

// remove a player with the given name
void multi_pxo_del_player(const char *name);

// try and find a player with the given name, return a pointer to his entry (or NULL)
int multi_pxo_find_player(const char *name);

// process the player list (select, etc)
void multi_pxo_process_players(bool api_access);

// display the player list
void multi_pxo_blit_players();

// scroll player list up
void multi_pxo_scroll_players_up();

// scroll player list down
void multi_pxo_scroll_players_down();

DCF(players, "Adds the specified number of bogus players to the PXO listing (Multiplayer)")
{
	char name[512] = "";
	int i;
	// add a bunch of bogus players
	dc_stuff_int(&i);

	for(int idx = 0; idx < i; idx++){
		sprintf(name, "bogus player %d", idx);
		multi_pxo_add_player(name);
	}
}

// chat text stuff -----------------------------------------

int Multi_pxo_chat_title_y[GR_NUM_RESOLUTIONS] = {
	181,	// GR_640
	253	// GR_1024
};

int Multi_pxo_chat_coords[GR_NUM_RESOLUTIONS][4] = {
	{ // GR_640
		196, 197, 412, 185
	},
	{ // GR_1024
		314, 271, 665, 330
	}
};

int Multi_pxo_input_coords[GR_NUM_RESOLUTIONS][4] = {
	{ // GR_640
		196, 386, 407, 24
	},
	{ // GR_1024
		314, 617, 660, 38
	}	
};

int Multi_pxo_max_chat_display[GR_NUM_RESOLUTIONS] = {
	17,	// GR_640
	32		// GR_1024
};

// all messages from the server are prefixed with this
#define MULTI_PXO_SERVER_PREFIX		"*** "

// the "has left" message from the server
#define MULTI_PXO_HAS_LEFT				"has left"

#define MULTI_PXO_CHAT_LINE_LEN 512

// the chat list
SCP_list<chat_line> Multi_pxo_chat;

// the current line to start displaying from
int Multi_pxo_chat_start = 0;

// input box for text
UI_INPUTBOX Multi_pxo_chat_input;

// slider for chat
UI_SLIDER2 Multi_pxo_chat_slider;

int Multi_pxo_chat_slider_coords[GR_NUM_RESOLUTIONS][4] = {
	{ // GR_640
		620, 206, 21, 147
	},
	{ // GR_1024
		990, 295, 34, 269
	}
};

const char *Multi_pxo_chat_slider_name[GR_NUM_RESOLUTIONS] = {
	"slider",
	"2_slider"
};

// extra delay time when switching channels
#define MULTI_PXO_SWITCH_DELAY_TIME			2000
static UI_TIMESTAMP Multi_pxo_switch_delay;

// clear all lines of chat text in the chat area
void multi_pxo_chat_clear();

// blit the chat text
void multi_pxo_chat_blit();

// add a line of text
void multi_pxo_chat_add_line(const char *txt,int mode);

// process an incoming line of text
void multi_pxo_chat_process_incoming(const char *txt, int mode = CHAT_MODE_NORMAL);

// scroll to the very bottom of the chat area
void multi_pxo_goto_bottom();

// check whether we can scroll down or not
bool multi_pxo_can_scroll_down();

// scroll the text up
void multi_pxo_scroll_chat_up();

// scroll the text down
void multi_pxo_scroll_chat_down();

// process chat controls
void multi_pxo_chat_process();

// if the text is a private message, return a pointer to the beginning of the message, otherwise return NULL
const char *multi_pxo_chat_is_private(const char *txt);

// if the text came from the server
int multi_pxo_is_server_text(const char *txt);

// if the text is message of the day text
int multi_pxo_is_motd_text(const char *txt);

// if the text is the end of motd text
int multi_pxo_is_end_of_motd_text(const char *txt);

// if the text is a "has left message" from the server
int multi_pxo_chat_is_left_message(const char *txt);

// recalculate the chat start index, and adjust the slider properly
void multi_pxo_chat_adjust_start();


// motd stuff ---------------------------------------------------------
#define MAX_PXO_MOTD_LEN			1024
#define PXO_MOTD_BLINK_TIME		500
char Pxo_motd[1024] = "";
int Pxo_motd_end = 0;
int Pxo_motd_read = 0;
static UI_TIMESTAMP Pxo_motd_blink_stamp;
int Pxo_motd_blink_on = 0;
int Pxo_motd_blinked_already = 0;

// initialize motd when going into this screen
void multi_pxo_motd_init();

// set the motd text
void multi_pxo_motd_add_text(const char *text);

// set end of motd
void multi_pxo_set_end_of_motd();

// display the motd dialog
void multi_pxo_motd_dialog();

// call to maybe blink the motd button
void multi_pxo_motd_maybe_blit();


// common dialog stuff ------------------------------------------------
const char *Multi_pxo_com_fname[GR_NUM_RESOLUTIONS] = {
	"PXOPop",
	"2_PXOPop"
};
const char *Multi_pxo_com_mask_fname[GR_NUM_RESOLUTIONS] = {
	"PXOPop-m",
	"2_PXOPop-m"
};

// popup coords
int Multi_pxo_com_coords[GR_NUM_RESOLUTIONS][2] = {
	{ // GR_640
		38, 129
	},
	{ // GR_1024
		61, 207
	}
};

// input box coords
int Multi_pxo_com_input_coords[GR_NUM_RESOLUTIONS][4] = {
	{ // GR_640
		53, 233, 448, 25
	},
	{ // GR_1024
		85, 372, 716, 40
	}
};

#define MULTI_PXO_COM_NUM_BUTTONS		2
#define MULTI_PXO_COM_CANCEL				0
#define MULTI_PXO_COM_OK					1

ui_button_info Multi_pxo_com_buttons[GR_NUM_RESOLUTIONS][MULTI_PXO_COM_NUM_BUTTONS] = {
	{	// GR_640
		ui_button_info("PXP_00",		573,	192,	-1,	-1,	0),
		ui_button_info("PXP_01",		573,	226,	-1,	-1,	1)
	},
	{	// GR_1024
		ui_button_info("2_PXP_00",		917,	308,	-1,	-1,	0),
		ui_button_info("2_PXP_01",		917,	361,	-1,	-1,	1)
	}
};

#define MULTI_PXO_COM_NUM_TEXT			2
UI_XSTR Multi_pxo_com_text[GR_NUM_RESOLUTIONS][MULTI_PXO_COM_NUM_TEXT] = {
	{ // GR_640
		{ "&Cancel",			645,	510,	204,	UI_XSTR_COLOR_PINK,	-1,	&Multi_pxo_com_buttons[0][MULTI_PXO_COM_CANCEL].button },
		{ "&Ok",					669,	548,	233,	UI_XSTR_COLOR_GREEN,	-1,	&Multi_pxo_com_buttons[0][MULTI_PXO_COM_OK].button }
	},
	{ // GR_1024
		{ "&Cancel",			645,	847,	327,	UI_XSTR_COLOR_PINK,	-1,	&Multi_pxo_com_buttons[1][MULTI_PXO_COM_CANCEL].button },
		{ "&Ok",					669,	877,	372,	UI_XSTR_COLOR_GREEN,	-1,	&Multi_pxo_com_buttons[1][MULTI_PXO_COM_OK].button }
	}
};

int Multi_pxo_com_bitmap = -1;
UI_WINDOW Multi_pxo_com_window;
UI_INPUTBOX Multi_pxo_com_input;

// text on the "top" half of the dialog display area
char Multi_pxo_com_top_text[MAX_PXO_TEXT_LEN];

// text on the "middle" portion of the dialog display area
char Multi_pxo_com_middle_text[MAX_PXO_TEXT_LEN];

// text on the "bottom" half of the dialog display area
char Multi_pxo_com_bottom_text[MAX_PXO_TEXT_LEN];

int Multi_pxo_com_top_text_coords[GR_NUM_RESOLUTIONS][2] = {
	{ // GR_640
		58, 152
	},
	{ // GR_1024
		91, 227
	}
};
int Multi_pxo_com_middle_text_y[GR_NUM_RESOLUTIONS] = {
	172,		// GR_640
	280		// GR_1024
};
int Multi_pxo_com_bottom_text_y[GR_NUM_RESOLUTIONS] = {
	192,		// GR_640
	326		// GR_1024
};

// initialize the common dialog with the passed max input length
void multi_pxo_com_init();

// close down the common dialog
void multi_pxo_com_close();

// blit all text lines, top, middle, bottoms
void multi_pxo_com_blit_text();

// set the top text, shortening as necessary
void multi_pxo_com_set_top_text(const char *txt);

// set the middle text, shortening as necessary
void multi_pxo_com_set_middle_text(const char *txt);

// set the bottom text, shortening as necessary
void multi_pxo_com_set_bottom_text(const char *txt);


// private channel join stuff -----------------------------------------
#define MULTI_PXO_PRIV_MAX_TEXT_LEN		30

// max private channel name length
char Multi_pxo_priv_chan[MULTI_PXO_PRIV_MAX_TEXT_LEN+100];

// return code, set to something other than -1 if we're supposed to return
int Multi_pxo_priv_return_code = -1;

// initialize the popup
void multi_pxo_priv_init();

// close down the popup
void multi_pxo_priv_close();

// run the popup, 0 if still running, -1 if cancel, 1 if ok 
int multi_pxo_priv_popup();

// process button presses
void multi_pxo_priv_process_buttons();

// handle a button press
void multi_pxo_priv_button_pressed(int n);

// process the inputbox
void multi_pxo_priv_process_input();


// find player stuff -----------------------------------------

char Multi_pxo_find_channel[MAX_CHANNEL_NAME_LEN+1];

// return code, set to something other than -1 if we're supposed to return
int Multi_pxo_find_return_code = -1;

// initialize the popup
void multi_pxo_find_init();

// close down the popup
void multi_pxo_find_close();

// run the popup, 0 if still running, -1 if cancel, 1 if ok 
int multi_pxo_find_popup();

// process button presses
void multi_pxo_find_process_buttons();

// handle a button press
void multi_pxo_find_button_pressed(int n);

// process the inputbox
void multi_pxo_find_process_input();

// process search mode if applicable
void multi_pxo_find_search_process();


// player info stuff -----------------------------------------
const char *Multi_pxo_pinfo_fname[GR_NUM_RESOLUTIONS] = {
	"PilotInfo2",
	"2_PilotInfo2"
};
const char *Multi_pxo_pinfo_mask_fname[GR_NUM_RESOLUTIONS] = {
	"PilotInfo2-M",
	"2_PilotInfo2-M"
};

// medals
#define MULTI_PXO_PINFO_NUM_BUTTONS		2
#define MULTI_PXO_PINFO_MEDALS			0
#define MULTI_PXO_PINFO_OK					1

ui_button_info Multi_pxo_pinfo_buttons[GR_NUM_RESOLUTIONS][MULTI_PXO_PINFO_NUM_BUTTONS] = {
	{ // GR_640
		ui_button_info("PI2_00",	328,	446,	319,	433,	0),
		ui_button_info("PI2_01",	376,	446,	382,	433,	1),
	},
	{ // GR_1024
		ui_button_info("2_PI2_00",	525,	714,	510,	695,	0),
		ui_button_info("2_PI2_01",	601,	714,	611,	695,	1),
	}
};

// text
#define MULTI_PXO_PINFO_NUM_TEXT			2
UI_XSTR Multi_pxo_pinfo_text[GR_NUM_RESOLUTIONS][MULTI_PXO_PINFO_NUM_TEXT] = {
	{ // GR_640
		{ "Medals",		1037,		319,	433,	UI_XSTR_COLOR_GREEN,	-1, &Multi_pxo_pinfo_buttons[0][MULTI_PXO_PINFO_MEDALS].button },
		{ "Ok",			345,		382,	433,	UI_XSTR_COLOR_PINK,	-1, &Multi_pxo_pinfo_buttons[0][MULTI_PXO_PINFO_OK].button },
	},
	{ // GR_1024
		{ "Medals",		1037,		510,	695,	UI_XSTR_COLOR_GREEN,	-1, &Multi_pxo_pinfo_buttons[1][MULTI_PXO_PINFO_MEDALS].button },
		{ "Ok",			345,		611,	695,	UI_XSTR_COLOR_PINK,	-1, &Multi_pxo_pinfo_buttons[1][MULTI_PXO_PINFO_OK].button },
	}
};

int Multi_pxo_pinfo_bitmap = -1;
UI_WINDOW Multi_pxo_pinfo_window;

vmt_stats_struct Multi_pxo_pinfo;
player Multi_pxo_pinfo_player;

int Multi_pxo_retrieve_mode = -1;

char Multi_pxo_retrieve_name[MAX_PLAYER_NAME_LEN+1];
char Multi_pxo_retrieve_id[128];

// stats label stuff
#define MULTI_PXO_PINFO_NUM_LABELS			18

int Multi_pxo_pinfo_coords[GR_NUM_RESOLUTIONS][4] = {
	{ // GR_640
		37, 142, 377, 289
	},
	{ // GR_640
		54, 227, 602, 462
	},
};
int Multi_pxo_pinfo_val_x[GR_NUM_RESOLUTIONS] = {
	230,	// GR_640
	310	// GR_1024
};

char *Multi_pxo_pinfo_stats_labels[MULTI_PXO_PINFO_NUM_LABELS];

char Multi_pxo_pinfo_vals[MULTI_PXO_PINFO_NUM_LABELS][50];

int Multi_pxo_pinfo_stats_spacing[MULTI_PXO_PINFO_NUM_LABELS] = {
	10,20,10,10,20,10,10,20,10,10,20,10,10,20,10,20,10,0
};

// popup conditional functions, returns 10 on successful get of stats
int multi_pxo_pinfo_cond();

// return 1 if Multi_pxo_pinfo was successfully filled in, 0 otherwise
int multi_pxo_pinfo_get(const char *name);

// fire up the stats view popup
void multi_pxo_pinfo_show();

// build the stats labels values
void multi_pxo_pinfo_build_vals();

// initialize the popup
void multi_pxo_pinfo_init();

// do frame
int multi_pxo_pinfo_do();

// close
void multi_pxo_pinfo_close();

// blit all the stats on this screen
void multi_pxo_pinfo_blit();

// run the medals screen
void multi_pxo_run_medals();

// notify stuff stuff -----------------------------------------
#define MULTI_PXO_NOTIFY_TIME				4000
#define MULTI_PXO_NOTIFY_Y					435

char Multi_pxo_notify_text[MAX_PXO_TEXT_LEN];
UI_TIMESTAMP Multi_pxo_notify_stamp;

// add a notification string
void multi_pxo_notify_add(const char *txt);

// blit and process the notification string
void multi_pxo_notify_blit();


// help screen stuff -----------------------------------------
//XSTR:OFF
const char *Multi_pxo_help_fname[GR_NUM_RESOLUTIONS] = {
	"PXHelp",
	"2_PXHelp"
};
const char *Multi_pxo_help_mask_fname[GR_NUM_RESOLUTIONS] = {
	"PXOHelp-M",
	"2_PXOHelp-M"
};

#define MULTI_PXO_HELP_NUM_BUTTONS			3
#define MULTI_PXO_HELP_PREV					0
#define MULTI_PXO_HELP_NEXT					1
#define MULTI_PXO_HELP_CONTINUE				2

ui_button_info Multi_pxo_help_buttons[GR_NUM_RESOLUTIONS][MULTI_PXO_HELP_NUM_BUTTONS] = {
	{ // GR_640
		ui_button_info("PXH_00",	15,	389,	-1,	-1,	0),
		ui_button_info("PXH_01",	60,	389,	-1,	-1,	1),
		ui_button_info("PXH_02",	574,	431,	571,	413,	2),
	},
	{ // GR_1024
		ui_button_info("2_PXH_00",	24,	622,	-1,	-1,	0),
		ui_button_info("2_PXH_01",	96,	622,	-1,	-1,	1),
		ui_button_info("2_PXH_02",	919,	689,	928,	663,	2),
	}
};

#define MULTI_PXO_HELP_NUM_TEXT				1
UI_XSTR Multi_pxo_help_text[GR_NUM_RESOLUTIONS][MULTI_PXO_HELP_NUM_TEXT] = {
	{	// GR_640
		{"Continue",		1069,		571,	413,	UI_XSTR_COLOR_PINK, -1,	&Multi_pxo_help_buttons[0][MULTI_PXO_HELP_CONTINUE].button },
	},
	{	// GR_1024
		{"Continue",		1069,		928,	663,	UI_XSTR_COLOR_PINK, -1,	&Multi_pxo_help_buttons[1][MULTI_PXO_HELP_CONTINUE].button },
	},
};

// help text
#define MULTI_PXO_HELP_FILE			"pxohelp.txt"

int Multi_pxo_help_coords[GR_NUM_RESOLUTIONS][2] = {
	{ // GR_640
		40, 40
	},
	{ // GR_1024
		60, 40
	}
};

int Multi_pxo_chars_per_line[GR_NUM_RESOLUTIONS] = {
	130,		// GR_640
	130		// GR_1024
};

int Multi_pxo_lines_pp[GR_NUM_RESOLUTIONS] = {
	32,		// GR_640
	57			// GR_1024
};

help_page Multi_pxo_help_pages[MULTI_PXO_MAX_PAGES];

int Multi_pxo_help_num_pages = 0;

int Multi_pxo_help_bitmap = -1;
UI_WINDOW Multi_pxo_help_window;

// current page we're on
int Multi_pxo_help_cur = 0;

// blit the current page
void multi_pxo_help_blit_page();

// process button presses
void multi_pxo_help_process_buttons();

// button pressed
void multi_pxo_help_button_pressed(int n);


// http banner stuff ---------------------------------------------
InetGetFile *Multi_pxo_ban_get = NULL;

#define PXO_BANNER_SWAP_TIME			120000
static UI_TIMESTAMP Multi_pxo_ban_swap_stamp;

// banners file
#define PXO_BANNERS_CONFIG_FILE			"pxobanners.cfg"

// coords to display banners at
int Pxo_ban_coords[GR_NUM_RESOLUTIONS][4] = {
	{ // GR_640
		149, 3, 475, 75
	},
	{ // GR_1024
		524, 3, 475, 75
	}
};

// http modes
#define PXO_BAN_MODE_LIST_STARTUP		0		// start downloading list
#define PXO_BAN_MODE_LIST					1		// downloading list
#define PXO_BAN_MODE_IMAGES_STARTUP		2		// start downloading images
#define PXO_BAN_MODE_IMAGES				3		// downloading images
#define PXO_BAN_MODE_IMAGES_DONE			4		// done downloading everything - now maybe load an image
#define PXO_BAN_MODE_IDLE					5		// done with everything - doing nothing
#define PXO_BAN_MODE_CHOOSE_RANDOM		6		// choose a bitmap we've already downloaded at random

// interface button for detecting clicks
UI_BUTTON Multi_pxo_ban_button;

// active pxo banner
pxo_banner Multi_pxo_banner;

// mode
int Multi_pxo_ban_mode = PXO_BAN_MODE_LIST_STARTUP;

// init
void multi_pxo_ban_init();

// process http download details
void multi_pxo_ban_process(bool api_access);

// close
void multi_pxo_ban_close();

// parse the banners file and maybe fill in Multi_pxo_dl_file[]
void multi_pxo_ban_parse_banner_file();

// any bitmap or info or whatever
void multi_pxo_ban_draw();

// called when the URL button is clicked
void multi_pxo_ban_clicked();


// ----------------------------------------------------------------------------------------------------
// PXO FUNCTIONS
//

// initialize the PXO screen
void multi_pxo_init(int use_last_channel, bool api_access)
{
	if (!api_access) {
		// load the background bitmap
		Multi_pxo_bitmap = bm_load(Multi_pxo_bitmap_fname[gr_screen.res]);
		if (Multi_pxo_bitmap < 0) {
			// we failed to load the bitmap - this is very bad
			Int3();
		}

		// load up the private channel bitmap
		Multi_pxo_com_bitmap = bm_load(Multi_pxo_com_fname[gr_screen.res]);
		Assert(Multi_pxo_com_bitmap != -1);

		// create the interface window
		Multi_pxo_window.create(0, 0, gr_screen.max_w_unscaled, gr_screen.max_h_unscaled, 0);
		Multi_pxo_window.set_mask_bmap(Multi_pxo_mask_fname[gr_screen.res]);

		// create the interface buttons
		for (int idx = 0; idx < MULTI_PXO_NUM_BUTTONS; idx++) {
			// create the object
			Multi_pxo_buttons[gr_screen.res][idx].button.create(&Multi_pxo_window,
				"",
				Multi_pxo_buttons[gr_screen.res][idx].x,
				Multi_pxo_buttons[gr_screen.res][idx].y,
				1,
				1,
				0,
				1);

			// set the sound to play when highlighted
			Multi_pxo_buttons[gr_screen.res][idx].button.set_highlight_action(common_play_highlight_sound);

			// set the ani for the button
			Multi_pxo_buttons[gr_screen.res][idx].button.set_bmaps(Multi_pxo_buttons[gr_screen.res][idx].filename);

			// set the hotspot
			Multi_pxo_buttons[gr_screen.res][idx].button.link_hotspot(Multi_pxo_buttons[gr_screen.res][idx].hotspot);
		}

		// add all xstrs
		for (int idx = 0; idx < MULTI_PXO_NUM_TEXT; idx++) {
			Multi_pxo_window.add_XSTR(&Multi_pxo_text[gr_screen.res][idx]);
		}

		// make all scrolling buttons repeatable
		Multi_pxo_buttons[gr_screen.res][MULTI_PXO_TEXT_UP].button.repeatable(1);
		Multi_pxo_buttons[gr_screen.res][MULTI_PXO_TEXT_DOWN].button.repeatable(1);
		Multi_pxo_buttons[gr_screen.res][MULTI_PXO_CHAN_UP].button.repeatable(1);
		Multi_pxo_buttons[gr_screen.res][MULTI_PXO_CHAN_DOWN].button.repeatable(1);
		Multi_pxo_buttons[gr_screen.res][MULTI_PXO_PLIST_UP].button.repeatable(1);
		Multi_pxo_buttons[gr_screen.res][MULTI_PXO_PLIST_DOWN].button.repeatable(1);

		// set the mouseover cursor if it loaded ok
		if (Web_cursor != NULL) {
			Multi_pxo_buttons[gr_screen.res][MULTI_PXO_RANKINGS].button.set_custom_cursor(Web_cursor);
		}

		// create the channel list select button and hide it
		Multi_pxo_channel_button.create(&Multi_pxo_window, "", Multi_pxo_chan_coords[gr_screen.res][0], Multi_pxo_chan_coords[gr_screen.res][1], Multi_pxo_chan_coords[gr_screen.res][2], Multi_pxo_chan_coords[gr_screen.res][3], 0, 1);
		Multi_pxo_channel_button.hide();

		// create the player list select button and hide it
		Multi_pxo_player_button.create(&Multi_pxo_window, "", Multi_pxo_player_coords[gr_screen.res][0], Multi_pxo_player_coords[gr_screen.res][1], Multi_pxo_player_coords[gr_screen.res][2], Multi_pxo_player_coords[gr_screen.res][3], 0, 1);
		Multi_pxo_player_button.hide();

		// create the chat input box
		Multi_pxo_chat_input.create(&Multi_pxo_window, Multi_pxo_input_coords[gr_screen.res][0], Multi_pxo_input_coords[gr_screen.res][1], Multi_pxo_input_coords[gr_screen.res][2], MAX_CHAT_LINE_LEN + 1, "", UI_INPUTBOX_FLAG_INVIS | UI_INPUTBOX_FLAG_ESC_CLR | UI_INPUTBOX_FLAG_KEYTHRU | UI_INPUTBOX_FLAG_EAT_USED);
		Multi_pxo_chat_input.set_focus();

		// create the banner button and hide it
		Multi_pxo_ban_button.create(&Multi_pxo_window, "", Pxo_ban_coords[gr_screen.res][0], Pxo_ban_coords[gr_screen.res][1], Pxo_ban_coords[gr_screen.res][2], Pxo_ban_coords[gr_screen.res][3], 0, 1);
		Multi_pxo_ban_button.hide();

		// create the chat slider
		Multi_pxo_chat_slider.create(&Multi_pxo_window, Multi_pxo_chat_slider_coords[gr_screen.res][0], Multi_pxo_chat_slider_coords[gr_screen.res][1], Multi_pxo_chat_slider_coords[gr_screen.res][2], Multi_pxo_chat_slider_coords[gr_screen.res][3], 0, Multi_pxo_chat_slider_name[gr_screen.res], multi_pxo_scroll_chat_up, multi_pxo_scroll_chat_down, NULL);
	}

	if (use_last_channel && strlen(Multi_pxo_channel_last)) {
		Multi_pxo_use_last_channel = 1;
	} else {
		memset(Multi_pxo_channel_last, 0, MAX_CHANNEL_NAME_LEN + 1);
		Multi_pxo_use_last_channel = 0;
	}
	
	// set our connection status so that we do the right stuff next frame
	Multi_pxo_must_validate = 1;
	Multi_pxo_must_connect = 0;
	Multi_pxo_connected = 0;	

	// channel we're currently connected to
	memset(&Multi_pxo_channel_current,0,sizeof(pxo_channel));
	Multi_pxo_channel_current.num_users = -1;
	
	// channel we're currently trying to change to, or NULL if nont	
	memset(&Multi_pxo_channel_switch,0,sizeof(pxo_channel));	
	Multi_pxo_channel_switch.num_users = -1;	

	// last time clicked the url button (so we don't have repeats)
	Multi_pxo_ranking_last = -1.0f;

	// channel switching extra time delay stamp
	Multi_pxo_switch_delay = UI_TIMESTAMP::invalid();

	// our nick for this session		
	multi_pxo_underscore_nick(Player->callsign,Multi_pxo_nick);		

	// clear the channel list
	multi_pxo_clear_channels();	

	// clear the player list
	multi_pxo_clear_players();

	// initialize the chat system
	multi_pxo_chat_clear();

	// initialize http
	multi_pxo_ban_init();

	if (!api_access) {
		// load the animation up
		if (gr_screen.res == GR_1024) {
			char anim_filename[32] = "2_";
			strcat_s(anim_filename, MULTI_PXO_ANIM_FNAME);
			generic_anim_init(&Multi_pxo_anim, anim_filename);
			Multi_pxo_anim.ani.bg_type = bm_get_type(Multi_pxo_bitmap);

			// if hi-res is not there, fallback to low
			if (generic_anim_stream(&Multi_pxo_anim) == -1) {
				generic_anim_init(&Multi_pxo_anim, MULTI_PXO_ANIM_FNAME);
				generic_anim_stream(&Multi_pxo_anim);
			}
		} else {
			generic_anim_init(&Multi_pxo_anim, MULTI_PXO_ANIM_FNAME);
			Multi_pxo_anim.ani.bg_type = bm_get_type(Multi_pxo_bitmap);
			generic_anim_stream(&Multi_pxo_anim);
		}
	}

	// clear the status text
	multi_pxo_set_status_text("");

	// last refresh time
	Multi_pxo_channel_last_refresh = -1.0f;

	// server count last refresh time
	Multi_pxo_channel_server_refresh = -1.0f;

	// set our mode
	Multi_pxo_mode = MULTI_PXO_MODE_NORMAL;

	// init motd
	multi_pxo_motd_init();

	// make sure we autojoin
	Multi_pxo_must_autojoin = 1;

	// clear all tracker channel related strings
	memset(Multi_fs_tracker_channel, 0, MAX_PATH);
	memset(Multi_fs_tracker_filter, 0, MAX_PATH);

	if (!api_access) {
		main_hall_start_music();
	}
}

// do frame for the PXO screen
void multi_pxo_do()
{
	pxo_channel priv_chan;
	
	// run api stuff	
	if(Multi_pxo_connected) {
		multi_pxo_api_process();
	}

	// process common stuff
	multi_pxo_process_common();
	
	switch(Multi_pxo_mode){
	// private channel join mode
	case MULTI_PXO_MODE_PRIVATE:
		switch(multi_pxo_priv_popup()){
		// still running
		case 0:
			break;

		// user hit "cancel"
		case -1:
			// return to normal mode
			Multi_pxo_mode = MULTI_PXO_MODE_NORMAL;
			break;

		// user hit "ok"
		case 1 :
			// setup some information
			memset(&priv_chan, 0, sizeof(pxo_channel));
			priv_chan.num_users = 0;
			strcpy_s(priv_chan.name, Multi_pxo_priv_chan);

			// see if we know about this channel already
			multi_pxo_join_channel(&priv_chan);

			// return to normal mode
			Multi_pxo_mode = MULTI_PXO_MODE_NORMAL;
			break;
		}
		break;

	// find player mode
	case MULTI_PXO_MODE_FIND:
		switch(multi_pxo_find_popup()){
		// still running
		case 0:
			break;

		// user hit "cancel"
		case -1:
			// return to normal mode
			Multi_pxo_mode = MULTI_PXO_MODE_NORMAL;
			break;

		// user hit "ok"
		case 1 :			
			// return to normal mode
			Multi_pxo_mode = MULTI_PXO_MODE_NORMAL;

			// if there is a valid channel name try and join it
			if(strlen(Multi_pxo_find_channel) && !SWITCHING_CHANNELS()){
				pxo_channel join;

				// setup the info
				memset(&join,0,sizeof(pxo_channel));
				join.num_users = 0;
				strcpy_s(join.name,Multi_pxo_find_channel);

				// try and join
				multi_pxo_join_channel(&join);
			}
			break;
		}
		break;
	// normal mode
	case MULTI_PXO_MODE_NORMAL:	
		multi_pxo_do_normal();
		break;
	}
}
//XSTR:ON
// close the PXO screen
void multi_pxo_close(bool api_access)
{
	if (!api_access) {
		// unload any bitmaps
		bm_release(Multi_pxo_bitmap);
		bm_release(Multi_pxo_com_bitmap);
	}

	// record the last channel we were on, if any
	memset(Multi_fs_tracker_channel, 0, MAX_PATH);
	memset(Multi_fs_tracker_filter, 0, MAX_PATH);

	if ( ON_CHANNEL() && strlen(Multi_pxo_channel_current.name) ) {
		// channel name
		strcpy(Multi_fs_tracker_channel, Multi_pxo_channel_current.name);
    
		// filter name
		strcpy(Multi_fs_tracker_filter, Multi_pxo_channel_current.name);
	} 

	// disconnect from the server
	DisconnectFromChatServer();
	Multi_pxo_connected = 0;

	if (!api_access) {
		// unload the animation
		if (Multi_pxo_anim.num_frames > 0) {
			generic_anim_unload(&Multi_pxo_anim);
		}

		// unload the palette for this screen
		multi_pxo_unload_palette();

		// destroy the UI_WINDOW
		Multi_pxo_window.destroy();
	}

	// clear the channel list
	multi_pxo_clear_channels();

	// close the chat system
	multi_pxo_chat_clear();

	// close http stuff
	multi_pxo_ban_close();
}

// run normally (no popups)
void multi_pxo_do_normal(bool api_access)
{
	int validate_code;

	if (!api_access) {
		int k = Multi_pxo_window.process();

		// process any keypresses
		switch (k) {
		case KEY_ESC:
			gamesnd_play_iface(InterfaceSounds::USER_SELECT);
			gameseq_post_event(GS_EVENT_MAIN_MENU);
			break;
		}

		// check for button presses
		multi_pxo_check_buttons();

		// if we're not in a chatroom, disable and hide the chat input box
		if (!ON_CHANNEL()) {
			Multi_pxo_chat_input.hide();
			Multi_pxo_chat_input.disable();
		} else {
			Multi_pxo_chat_input.enable();
			Multi_pxo_chat_input.unhide();
		}

		// blit everything
		multi_pxo_blit_all();

		// flip the page
		gr_flip();
	}

	// if we need to get tracker info for ourselves, do so
	if (Multi_pxo_must_validate) {
		// initialize the master tracker API for Freespace
		multi_fs_tracker_init();

		// validate game data first, for initial game/mod ident
		if (multi_fs_tracker_validate_game_data() < 0) {
			// in the event of a connection failure we should just dump back to the mainhall
			popup(PF_USE_AFFIRMATIVE_ICON | PF_TITLE_BIG | PF_TITLE_RED, 1, POPUP_OK, XSTR("Failed to connect to Parallax Online!", 947));
			gameseq_post_event(GS_EVENT_MAIN_MENU);

			return;
		}

		// validate the current player with the master tracker (will create the pilot on the MT if necessary)
		validate_code = multi_fs_tracker_validate(0);

		if (validate_code != 1) {
			// show an error popup if it failed (not cancelled by the user)
			if (validate_code == 0) {
				switch (popup(PF_USE_AFFIRMATIVE_ICON | PF_WEB_CURSOR_1 | PF_WEB_CURSOR_2, 3, POPUP_CANCEL,XSTR("&Create Acct",936), XSTR("&Verify Acct",937), XSTR("PXO Login not accepted.  You may visit the Parallax Online website to create or verify your login.  Or you may click Cancel to play without using the Parallax Online service.  (You may switch back to Parallax Online from the Options Menu under the Multi tab.)",938))) {
					case 0:
						nprintf(("Network","PXO CANCEL\n"));

						// flip player "pxo" off and push to the join game screen,
						// they will have to follow the instructions above to toggle PXO back on
						// and since player is accepting, we need to persist changes
						Multi_options_g.pxo = false;
						options::OptionsManager::instance()->set_ingame_binary_option("Multi.TogglePXO", false);
						if (Using_in_game_options) {
							options::OptionsManager::instance()->persistChanges();
						}
						// Net_game_tcp_mode = NET_TCP;
						gameseq_post_event(GS_EVENT_MULTI_JOIN_GAME);
						break;

					case 1:
						nprintf(("Network","PXO CREATE\n"));
						// fire up the given URL
						multi_pxo_url(Multi_options_g.pxo_create_url);
						break;

					case 2:
						nprintf(("Network","PXO VERIFY\n"));
						// fire up the given URL
						multi_pxo_url(Multi_options_g.pxo_verify_url);
						break;
				}
			}

			Multi_pxo_must_connect = 0;
			Multi_pxo_must_validate = 0;
		}
		// now we have to connect to PXO
		else {
			Multi_pxo_must_connect = 1;
			Multi_pxo_must_validate = 0;
		}
	}

	// if we need to connect, do so now
	if (Multi_pxo_must_connect) {		
		SCP_string pxo_ident = "PXO: Game identified as " + Multi_fs_tracker_game_name;
		multi_pxo_chat_process_incoming(pxo_ident.c_str(), CHAT_MODE_CARRY);

		// for now, just try once
		Multi_pxo_connected = multi_pxo_connect();

		// if we successfully connected, send a request for a list of channels on the server
		if (Multi_pxo_connected) {
			multi_pxo_get_channels();

			// set our status
			multi_pxo_set_status_text(XSTR("Retrieving Public Channels",939));
		} else {
			// set our status
			multi_pxo_set_status_text(XSTR("Failed to connect to Parallax Online",940));
		}

		// no longer need to connect
		Multi_pxo_must_connect = 0;
	}
}

// blit everything on the "normal" screen
void multi_pxo_blit_all()
{
	// draw the background, etc
	gr_reset_clip();	
	// GR_MAYBE_CLEAR_RES(Multi_pxo_bitmap);
	int bmap = Multi_pxo_bitmap;
	do  { 
		int bmw = -1; 
		int bmh = -1; 
		if(bmap != -1){ 
			bm_get_info( bmap, &bmw, &bmh); 
			if((bmw != gr_screen.max_w_unscaled) || (bmh != gr_screen.max_h_unscaled)){
				gr_clear();
			} 
		} else {
			gr_clear();
		} 
	} while(false);
	if(Multi_pxo_bitmap != -1){
		gr_set_bitmap(Multi_pxo_bitmap);
		gr_bitmap(0,0,GR_RESIZE_MENU);
	}
	Multi_pxo_window.draw();

	// display the channel list
	multi_pxo_blit_channels();

	// display the player list
	multi_pxo_blit_players();

	// blit the chat text
	multi_pxo_chat_blit();

	// blit the status text
	multi_pxo_blit_status_text();		

	// blit and process the notification string
	multi_pxo_notify_blit();

	// any bitmap or info or whatever
	multi_pxo_ban_draw();

	// draw any motd stuff
	multi_pxo_motd_maybe_blit();

	// if we have a valid animation handle, play it
	// display the mission start countdown timer (if any)
	//anim_render_all(GS_STATE_MULTI_MISSION_SYNC,flFrametime);
	if(gameseq_get_state() == GS_STATE_PXO && Multi_pxo_anim.num_frames > 0)
		generic_anim_render(&Multi_pxo_anim, flFrametime, MULTI_PXO_ANIM_X, MULTI_PXO_ANIM_Y, true);
}

// process common stuff
void multi_pxo_process_common(bool api_access)
{
	// skip if we haven't logged into the tracker yet
	if (Multi_pxo_must_validate) {
		return;
	}

	// process the channel list (select, etc)
	multi_pxo_process_channels(api_access);

	// process the player list (select, etc)
	multi_pxo_process_players(api_access);

	// chat_process specifically handles checking the typed string in the
	// ui input box and sending it. When running through the UI, that input
	// box does not exist. So we do not need this one processor.
	if (!api_access) {
		// process chat controls
		multi_pxo_chat_process();
	}

	// process http download details
	multi_pxo_ban_process(api_access);
}

// get selected player information
void multi_pxo_get_data(char * /*name*/)
{
}

// handle being kicked
void multi_pxo_handle_kick()
{
	// remove ourselves from the room	
	memset(&Multi_pxo_channel_current,0,sizeof(pxo_channel));
	Multi_pxo_channel_current.num_users = -1;

	// clear text
	multi_pxo_chat_clear();

	// clear the old player list
	multi_pxo_clear_players();

	// add a notification string
	multi_pxo_notify_add(XSTR("You have been kicked",941));
}

// handle being disconnected
void multi_pxo_handle_disconnect()
{
	ml_printf("PXO:  Got DISCONNECT from server!");

	if ( popup_active() ) {
		popup_change_text( XSTR("You have been disconnected from the server", 942) );
	} else {
		popup(PF_USE_AFFIRMATIVE_ICON, 1, POPUP_OK, XSTR("You have been disconnected from the server", 942));
		gameseq_post_event(GS_EVENT_MAIN_MENU);
	}
}

// return string2, which is the first substring of string 1 without a space
// it is safe to pass the same pointer for both parameters
void multi_pxo_strip_space(char *string1,char *string2)
{
	char midway[MAX_PXO_TEXT_LEN];
	char *tok;

	// copy the original
	strcpy_s(midway,string1);
	tok = strtok(midway," ");
	if(tok != NULL){
		strcpy(string2,tok);
	} else {
		strcpy(string2,"");
	}
}

static int open_url(const char *url)
{
#if defined(_WIN32) || defined(__APPLE__)
	const char *open_cmd = "open";
#else
	const char *open_cmd = "xdg-open";
#endif

	char s_url[256];

	// make sure it's a valid web uri
	if ( !SDL_strncasecmp(url, "http://", 7) || !SDL_strncasecmp(url, "https://", 8) ) {
		SDL_strlcpy(s_url, url, SDL_arraysize(s_url));
	} else {
		SDL_snprintf(s_url, SDL_arraysize(s_url), "http://%s", url);
	}

#ifdef _WIN32
	intptr_t rval = (intptr_t) ShellExecuteA(NULL, open_cmd, s_url, NULL, NULL, SW_SHOWNORMAL);

	if (rval <= 32) {
		return -1;
	}

	return 0;
#else
	int statval = 0;
	pid_t mpid = fork();

	if (mpid < 0) {
		// nothing, will return error
	} else if (mpid == 0) {
		int rv = 0;

		rv = execlp(open_cmd, open_cmd, s_url, nullptr);

		exit(rv);
	} else {
		waitpid(mpid, &statval, 0);

		if ( WIFEXITED(statval) ) {
			if (WEXITSTATUS(statval) == 0) {
				return 0;
			} else {
				return -1;
			}
		}
	}

	return -1;
#endif
}

// fire up the given URL
void multi_pxo_url(const char *url)
{
	if ( !url || !strlen(url) ) {
		return;
	}

	static UI_TIMESTAMP click_timeout;

	if ( click_timeout.isValid() && !ui_timestamp_elapsed(click_timeout) ) {
		return;
	}

	if ( open_url(url) ) {
		popup(PF_USE_AFFIRMATIVE_ICON | PF_TITLE_RED | PF_TITLE_BIG,1,POPUP_OK,XSTR("Warning\nCould not locate/launch default Internet Browser",943));
	} else {
		// short delay before allowing another click
		click_timeout = ui_timestamp(750);
	}
}

/**
 * Unload the palette
 */
void multi_pxo_unload_palette()
{
	// unload the palette if it exists
	if(Multi_pxo_palette != -1){
		bm_release(Multi_pxo_palette);
		Multi_pxo_palette = -1;
	}
}

/**
 * If we're currently on a private channel
 */
int multi_pxo_on_private_channel()
{
	// if we're connected to a channel with the "+" symbol on front
	if(ON_CHANNEL() && (Multi_pxo_channel_current.name[0] == '+')){
		return 1;
	}

	// otherwise return falos
	return 0;
}

/**
 * Convert string 1 into string 2, substituting underscores for spaces
 */
void multi_pxo_underscore_nick(char *string1,char *string2)
{
	char nick_temp[512];
	char *tok;
	
	// don't do anything if we have bogus string
	if((string1 == NULL) || (string2 == NULL)){
		return;
	}

	// copy the nickname
	memset(nick_temp,0,512);
	strcpy_s(nick_temp,string1);

	// get the first token
	tok = strtok(nick_temp," ");
	if(tok != NULL){
		strcpy(string2,tok);

		// get the next token
		tok = strtok(NULL," ");
		while(tok != NULL){				
			if(tok != NULL){
				strcat(string2,"_");
				strcat(string2,tok);
			}

			tok = strtok(NULL," ");
		}
	} else {
		strcpy(string2,string1);
	}
}

/**
 * If the command is a potential "nick" command
 */
int multi_pxo_is_nick_command(char *msg)
{
	char *tok;
	char tmp[512];

	// get the first token in the message
	memset(tmp,0,512);
	strcpy_s(tmp,msg);
	tok = strtok(tmp," ");
	if(tok == NULL){
		// can't be a nick message
		return 0;
	}

	return !stricmp(tok,NOX("/nick"));
}

/**
 * Check for button presses
 */
void multi_pxo_check_buttons()
{
	int idx;

	// go through all buttons
	for(idx=0;idx<MULTI_PXO_NUM_BUTTONS;idx++){
		if(Multi_pxo_buttons[gr_screen.res][idx].button.pressed()){
			multi_pxo_button_pressed(idx);
			break;
		}
	}
}

/**
 * Handle a button press
 */
void multi_pxo_button_pressed(int n)
{
	switch(n){
	case MULTI_PXO_EXIT:
		gamesnd_play_iface(InterfaceSounds::USER_SELECT);
		gameseq_post_event(GS_EVENT_MAIN_MENU);
		break;

	case MULTI_PXO_CHAN_UP:
		multi_pxo_scroll_channels_up();
		break;

	case MULTI_PXO_CHAN_DOWN:
		multi_pxo_scroll_channels_down();
		break;

	case MULTI_PXO_TEXT_UP:
		multi_pxo_scroll_chat_up();
		break;

	case MULTI_PXO_TEXT_DOWN:
		multi_pxo_scroll_chat_down();
		break;

	case MULTI_PXO_PLIST_UP:
		multi_pxo_scroll_players_up();
		multi_pxo_chat_adjust_start();
		break;

	case MULTI_PXO_PLIST_DOWN:
		multi_pxo_scroll_players_down();		
		multi_pxo_chat_adjust_start();		
		break;

	case MULTI_PXO_JOIN:
		multi_pxo_maybe_join_channel(&Multi_pxo_channels[Multi_pxo_channel_select]);
		break;

	case MULTI_PXO_GAMES:
		// move to the join game screen as normally (temporary!)
		gameseq_post_event( GS_EVENT_MULTI_JOIN_GAME );
		break;

	case MULTI_PXO_JOIN_PRIV:
		// if we're not already trying to join, allow this
		if(!SWITCHING_CHANNELS()){
			gamesnd_play_iface(InterfaceSounds::USER_SELECT);

			// fire up the private join popup
			multi_pxo_priv_popup();
		} else {
			multi_pxo_notify_add(XSTR("Already trying to join a channel!",945));
			gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
		}		
		break;

	case MULTI_PXO_FIND:
		gamesnd_play_iface(InterfaceSounds::USER_SELECT);

		// fire up the find join popup
		multi_pxo_find_popup();
		break;

	case MULTI_PXO_HELP:
		gamesnd_play_iface(InterfaceSounds::USER_SELECT);
		gameseq_post_event(GS_EVENT_PXO_HELP);
		break;

	case MULTI_PXO_PINFO:
		char stats[MAX_PXO_TEXT_LEN];

		// if we have a guy selected, try and get his info
		if((Multi_pxo_player_select >= 0) && (Multi_pxo_player_select < static_cast<int>(Multi_pxo_players.size()))){
			// if we successfully got info for this guy
			if (multi_pxo_maybe_get_player(Multi_pxo_players[Multi_pxo_player_select].c_str())) {				
				// show the stats
				multi_pxo_pinfo_show();				
			}
			// if we didn't get stats for this guy.
			else {
				memset(stats,0,MAX_PXO_TEXT_LEN);
				sprintf(stats,
					XSTR("Could not get stats for %s\n(May not be a registered pilot)", 946),
					Multi_pxo_players[Multi_pxo_player_select].c_str());
				popup(PF_USE_AFFIRMATIVE_ICON,1,POPUP_OK,stats);
			}
		} else {
			gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
		}
		break;

	case MULTI_PXO_RANKINGS:		
		// make sure he doesn't click it too many times
		if((Multi_pxo_ranking_last < 0.0f) || ((f2fl(timer_get_fixed_seconds()) - Multi_pxo_ranking_last) > MULTI_PXO_RANK_TIME) ){
			gamesnd_play_iface(InterfaceSounds::USER_SELECT);
			
			// fire up the url
			multi_pxo_url(Multi_options_g.pxo_rank_url);

			// mark the time down
			Multi_pxo_ranking_last = f2fl(timer_get_fixed_seconds());
		} else {
			gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
		}
		break;

	case MULTI_PXO_MOTD:
		// maybe fire up the pxo motd dialog
		multi_pxo_motd_dialog();
		break;
	}
}

/**
 * Condition function for popup_do_with_condition for connected to Parallax Online
 */
int mpxo_failed = 0;
int multi_pxo_connect_do()
{
	int ret_code;		
	char id_string[MAX_PXO_TEXT_LEN] = "";
	char ip_string[MAX_PXO_TEXT_LEN] = "";	

	// if we already tried and failed, sit around until the user presses cancel
	if(!mpxo_failed){	
		// try and connect to the server	
		Assert(Player);		

		// build the tracker id string
		memset(id_string, 0, MAX_PXO_TEXT_LEN);
		sprintf_safe(id_string, "%s %s", Multi_tracker_id_string, Player->callsign);

		// build the ip string
		memset(ip_string, 0, MAX_PXO_TEXT_LEN);
		if (snprintf(ip_string, MAX_PXO_TEXT_LEN, "%s:%d", Multi_options_g.pxo_ip, PXO_CHAT_PORT) >= MAX_PXO_TEXT_LEN) {
			// Make sure the string is null terminated
			ip_string[MAX_PXO_TEXT_LEN - 1] = '\0';
		}

		// connect to the server
		ret_code = ConnectToChatServer(ip_string, Multi_pxo_nick, id_string);		

		// give some time to the pxo api.
		multi_pxo_api_process();	

		switch(ret_code){
		// already connected, return success
		case -2:
			return 10;

		// failed to connect, return fail
		case -1 :
			mpxo_failed = 1;
			return 1;

		// connected, return success
		case 1 :
			return 10;

		// still connecting
		case 0 :			
			return 0;
		}
	}

	return 0;
}

/**
 * Popup loop which does an autojoin of a public channel.
 *
 * Returns when the autojoin process is complete
 */
int multi_pxo_autojoin_do()
{
	pxo_channel last_channel;

	// if we need to autojoin, do so now
	if (Multi_pxo_must_autojoin) {
		Multi_pxo_must_autojoin = 0;

		// if we're supposed to be using a (valid) "last" channel, do so
		if ( Multi_pxo_use_last_channel && strlen(Multi_pxo_channel_last) ) {
			// setup the data
			memset(&last_channel, 0, sizeof(pxo_channel));
			last_channel.num_users = 0;
			strcpy_s(last_channel.name, Multi_pxo_channel_last);

			// join the channel
			multi_pxo_join_channel(&last_channel);

			nprintf(("Network","PXO : using last channel\n"));
		} else {
			multi_pxo_autojoin();

			nprintf(("Network","PXO : using autojoin channel\n"));
		}

		multi_pxo_get_channels();
	}

	// give some time to the pxo api.
	multi_pxo_api_process();	
	multi_pxo_process_common();

	// next value is not -1 when actually switching channels, so keep processing by returning 0.
	if ( SWITCHING_CHANNELS() )
		return 0;

	// couldn't switch channel for some reason.  bail out with -1
	if ( !ON_CHANNEL() )
		return -1;

	// return success
	return 1;
}

/**
 * Attempt to connect to Parallax Online, return success or fail
 */
int multi_pxo_connect()
{
	char join_str[256];	
	char join_fail_str[256];
	
	// intiialize chat api
	ChatInit();

	// set us to "must autojoin"
	Multi_pxo_must_autojoin = 1;

	// run the connect dialog/popup
	mpxo_failed = 0;

	if ( popup_till_condition(multi_pxo_connect_do, XSTR("&Cancel", 779), XSTR("Logging into Parallax Online", 949)) == 10 ) {
		int rval;

		memset(join_str, 0, 256);
		memset(join_fail_str, 0, 256);

		// if we're going to use the "last" channel
		if ( Multi_pxo_use_last_channel && strlen(Multi_pxo_channel_last) ) {			
			strcpy_s(join_str, XSTR("Joining last channel (", 982));
			strcat_s(join_str, Multi_pxo_channel_last + 1);
			strcat_s(join_str, ")");

			strcpy_s(join_fail_str, XSTR("Unable to join last channel", 983));
		} else {
			strcpy_s(join_str, XSTR("Autojoining public channel", 984));
			strcpy_s(join_fail_str, XSTR("Unable to autojoin public channel", 985));
		}

		// once connected, we should do an autojoin before allowing the guy to continue.
		rval = popup_till_condition( multi_pxo_autojoin_do, XSTR("&Cancel", 779), join_str );

		if ( rval == 1 )
			return 1;

		popup(PF_USE_AFFIRMATIVE_ICON, 1, XSTR("OK", 1492), join_fail_str);
	}

	// otherwise disconnect just to be safe
	DisconnectFromChatServer();

	// we failed to connect, so give a nice popup about that
	if (mpxo_failed) {
		popup(PF_USE_AFFIRMATIVE_ICON, 1, POPUP_OK, XSTR("Failed to connect to Parallax Online!", 947));
	}

	// if we are coming from the mainhall then fail to the join game screen rather
	// than keeping the user constantly at the mainhall
	if (gameseq_get_previous_state() == GS_STATE_MAIN_MENU) {
		gameseq_post_event(GS_EVENT_MULTI_JOIN_GAME);
	} else {
		gameseq_post_event(GS_EVENT_MAIN_MENU);
	}

	// did not successfully connect
	return 0;
}

/**
 * Run the networking functions for the PXO API
 */
void multi_pxo_api_process()
{
	char *p;
	char msg_str[512];
	Chat_command *cmd;	
	pxo_channel *lookup;

	// give some time to psnet
	PSNET_TOP_LAYER_PROCESS();

	// give some time to the game tracker API
	IdleGameTracker();

	// give some time to the user tracker API
	PollPTrackNet();
	
	// get any incoming text 
	do
	{
		p = GetChatText();

		if (p) {
			// process the chat line
			multi_pxo_chat_process_incoming(p);
		}
	} while(p);
	
	// get any incoming channel list stuff
	p = GetChannelList();

	if (p) {
		multi_pxo_make_channels(p);
	}	
	
	// process any chat commands
	cmd = GetChatCommand();

	while (cmd) {		
		switch (cmd->command)
		{			
			case CC_USER_JOINING:			
				// add a user, if he doesn't already exist
				if (multi_pxo_find_player(cmd->data) < 0)
					multi_pxo_add_player(cmd->data);

				// increase the player count
				if (ON_CHANNEL() ) {
					lookup = multi_pxo_find_channel(Multi_pxo_channel_current.name);

					if (lookup != nullptr)
						lookup->num_users++;
				}
				break;
		
			case CC_USER_LEAVING:			
				// delete a user
				multi_pxo_del_player(cmd->data);

				// add a text message
				memset(msg_str, 0, 512);
				sprintf(msg_str, XSTR("*** %s has left", 950), cmd->data);			
				multi_pxo_chat_process_incoming(msg_str);

				// decrease the player count
				if ( ON_CHANNEL() ) {
					lookup = multi_pxo_find_channel(Multi_pxo_channel_current.name);

					if (lookup != nullptr)
						lookup->num_users--;
				}
				break;
		
			case CC_DISCONNECTED:
				multi_pxo_handle_disconnect();
				break;
		
			case CC_KICKED:
				multi_pxo_handle_kick();
				break;

			case CC_NICKCHANGED:
				// process a nick change
				multi_pxo_process_nick_change(cmd->data);			
				break;

			case CC_YOURCHANNEL:
				// copy the current channel info, and unset the switching status
				memset( &Multi_pxo_channel_current, 0, sizeof(pxo_channel) );
				Multi_pxo_channel_switch.num_users = -1;			

				SetNewChatChannel(nullptr);

				strcpy_s(Multi_pxo_channel_current.name, cmd->data);

				// if we don't already have this guy on the list, add him
				lookup = multi_pxo_find_channel(Multi_pxo_channel_current.name);
	
				if (lookup == nullptr) {
					// create a new channel with the given name and place it on the channel list, return a pointer or NULL on fail
					lookup = multi_pxo_add_channel(Multi_pxo_channel_current.name);
				}

				// set the user count to be 0
				if (lookup != nullptr)
					lookup->num_users = 0;

				// set our "last" channel to be this one
				strcpy_s(Multi_pxo_channel_last, Multi_pxo_channel_current.name);

				// refresh current channel server count
				multi_pxo_channel_refresh_current();

				break;
		
			default:
				Int3();
		}

		cmd = GetChatCommand();
	}	

	// handle any processing details if we're currently trying to join a channel
	multi_pxo_handle_channel_change();
}

/**
 * Process a "nick" change event
 */
void multi_pxo_process_nick_change(char *data)
{
	char *from, *to;	
	
	// get the new string
	from = strtok(data," ");
	to = strtok(nullptr,"");
	if((from != nullptr) && (to != nullptr)){
		int ply = multi_pxo_find_player(from);
		if(ply > 0){
			Multi_pxo_players[ply] = to;

			// if this is also my nick, change it
			if(!stricmp(Multi_pxo_nick,from)){
				strcpy_s(Multi_pxo_nick,to);
			}
		}		
	}	
}

/**
 * Autojoin an appropriate channel
 */
void multi_pxo_autojoin()
{
	pxo_channel sw;

	memset( &sw, 0, sizeof(pxo_channel) );
	sw.num_users = 0;
	strcpy_s(sw.name,MULTI_PXO_AUTOJOIN_CHANNEL);

	// if we found a valid room, attempt to join it	
	multi_pxo_join_channel(&sw);		
}

/**
 * Does the string match the "autojoin" prefix
 */
int multi_pxo_is_autojoin(char *name)
{
	// check to see if the name is long enough
	if ( strlen(name) < strlen(MULTI_PXO_AUTOJOIN_PREFIX) )
		return 0;

	// check to see if the first n chars match
	return !strnicmp(name, MULTI_PXO_AUTOJOIN_PREFIX, strlen(MULTI_PXO_AUTOJOIN_PREFIX));
}

/**
 * Called from the game tracker API - server count update for a channel
 */
void multi_pxo_channel_count_update(char *name, int count)
{
	pxo_channel *lookup;
	
	// lookup the channel name on the normal list	
	lookup = multi_pxo_find_channel(name);

	if (lookup != nullptr) {
		lookup->num_servers = (ushort)count;

		nprintf(("Network","PXO : updated channel %s server count to %d\n",name,count));
		ml_printf("PXO : updated channel %s server count to %d", name, count);
	} else {
		ml_printf("PXO : unable to locate channel when trying to update count for %s", name);
	}
}

// status bar stuff -----------------------------------------------

/**
 * Set the status text
 */
void multi_pxo_set_status_text(const char *txt)
{
	// copy in the text
	memset(Multi_pxo_status_text, 0, MAX_PXO_TEXT_LEN);
	strncpy(Multi_pxo_status_text, txt, MAX_PXO_TEXT_LEN-1);

	// make sure it fits properly
	font::force_fit_string(Multi_pxo_status_text, MAX_PXO_TEXT_LEN-1, Multi_pxo_status_coords[gr_screen.res][2]);
}

/**
 * Blit the status text
 */
void multi_pxo_blit_status_text()
{
	int w;

	// center and draw the text
	if(strlen(Multi_pxo_status_text)) {
		gr_set_color_fast(&Color_bright);
		gr_get_string_size(&w, NULL, Multi_pxo_status_text);
		gr_string(Multi_pxo_status_coords[gr_screen.res][0] + ((Multi_pxo_status_coords[gr_screen.res][2] - w)/2), Multi_pxo_status_coords[gr_screen.res][1], Multi_pxo_status_text, GR_RESIZE_MENU);
	}
}


// channel related stuff -------------------------------------------

/**
 * Get a list of channels on the server
 */
void multi_pxo_get_channels()
{		
	SendChatString(NOX("/list"));
}

/**
 * Clear the old channel list
 */
void multi_pxo_clear_channels()
{
	// only clear a non-null list
	if(!Multi_pxo_channels.empty()){		
		Multi_pxo_channels.clear();
		Multi_pxo_channels.shrink_to_fit();

		// item we're going to start displaying at
		Multi_pxo_channel_start = 0;
		Multi_pxo_channel_start_index = -1;

		// items we've currently got selected
		Multi_pxo_channel_select = 0;
	}	
}

/**
 * Parse the input string and make a list of new channels
 */
void multi_pxo_make_channels(char *chan_str)
{	
	char *name_tok,*user_tok,*desc_tok;
	pxo_channel *res;
	pxo_channel *lookup;
	int num_users;
	
	nprintf(("Network","Making some channels!\n"));

	// set the last get time
	Multi_pxo_channel_last_refresh = f2fl(timer_get_fixed_seconds());

	name_tok = strtok(chan_str," ");
	if(name_tok == nullptr){
		return;
	} 
	name_tok += 1;
	do {
		// parse the user count token		
		user_tok = strtok(nullptr," ");

		// parse the channel description token
		desc_tok = strtok(nullptr,"$");

		// something invalid in the data, return here.....
		if((name_tok == nullptr) || (user_tok == nullptr) || (desc_tok == nullptr)){
			return;
		}

		// get the # of users
		num_users = (ubyte)atoi(user_tok);

		// if the # of users is > 0, or its not an autojoin, place it on the display list
		if((num_users > 0) || !multi_pxo_is_autojoin(name_tok)){
			// see if it exists already, and if so, just update the user count
			lookup = multi_pxo_find_channel(name_tok);
			
			if(lookup != nullptr){
				lookup->num_users = (short)num_users;
			}
			// add the channel
			else {
				res = multi_pxo_add_channel(name_tok);
				if(res != nullptr){
					res->num_users = (short)num_users;
					strcpy_s(res->desc,desc_tok);
				}		
			}
		}				

		// get the next name token
		name_tok = strtok(nullptr," ");
	} while(name_tok != nullptr);

	// refresh channels
	multi_pxo_set_status_text(XSTR("Connected to Parallax Online",951));	

	// if we haven't refreshed server counts yet, do it now
	if(Multi_pxo_channel_server_refresh < 0.0f){
		multi_pxo_channel_refresh_servers();
	}

	// if we don't already have this guy on the list, add him
	if(ON_CHANNEL()){
		lookup = multi_pxo_find_channel(Multi_pxo_channel_current.name);
		if(lookup == nullptr){
			// create a new channel with the given name and place it on the channel list, return a pointer or NULL on fail
			multi_pxo_add_channel(Multi_pxo_channel_current.name);
		}
	}
}

/**
 * Create a new channel with the given name and place it on the channel list, return a pointer or NULL on fail
 */
pxo_channel *multi_pxo_add_channel(char *name)
{
	pxo_channel channel;
	strcpy_s(channel.name, name);
	strcpy_s(channel.desc, "");
	channel.num_users = -1;
	channel.num_servers = 0;
	Multi_pxo_channels.push_back(channel);
	return &Multi_pxo_channels.back();
}

/**
 * Lookup a channel with the specified name
 */
pxo_channel *multi_pxo_find_channel(char *name)
{
	if(Multi_pxo_channels.size() == 0){
		return nullptr;
	} 

	for (size_t i = 0; i < Multi_pxo_channels.size(); i++) {
		pxo_channel channel = Multi_pxo_channels[i];
		if (!stricmp(name, channel.name)) {
			return &Multi_pxo_channels[i];
		}
	}

	return nullptr;
}

/**
 * Process the channel list (select, etc)
 */
void multi_pxo_process_channels(bool api_access)
{
	int item_index,my;
	int idx;
	
	// the first part of this function works under the assumption that the list has been initialized.
	if (Multi_pxo_channels.size() > 0) {

		// if we don't have a start item
		if (Multi_pxo_channel_start < 0) {
			Multi_pxo_channel_start = 0;
			Multi_pxo_channel_start_index = 0;
		} 

		// if we don't have a selected item
		if(Multi_pxo_channel_select < 0){
			Multi_pxo_channel_select = 0;

			// set the text
			multi_pxo_set_status_text(Multi_pxo_channels[Multi_pxo_channel_select].desc);
		}

		// if the "switch" delay timestamp is set, see if it has expired
		if(Multi_pxo_switch_delay.isValid() && ui_timestamp_elapsed(Multi_pxo_switch_delay)){
			Multi_pxo_switch_delay = UI_TIMESTAMP::invalid();
		}

		// see if we have a mouse click on the channel region
		if(!api_access && Multi_pxo_channel_button.pressed()){
			Multi_pxo_channel_button.get_mouse_pos(nullptr,&my);

			// index from the top
			item_index = my / (gr_get_font_height() + 1);

			// select the item if possible
			if((item_index + Multi_pxo_channel_start_index) < static_cast<int>(Multi_pxo_channels.size())){
				Multi_pxo_channel_select = Multi_pxo_channel_start;
				for(idx=0;idx<item_index;idx++){
					Multi_pxo_channel_select++;
				}

				// set the text
				multi_pxo_set_status_text(Multi_pxo_channels[Multi_pxo_channel_select].desc);
			}
		}
	}

	// last refresh time
	if((Multi_pxo_channel_last_refresh > 0.0f) && ((f2fl(timer_get_fixed_seconds()) - Multi_pxo_channel_last_refresh) > CHANNEL_REFRESH_TIME) ){
		// refresh channels
		multi_pxo_set_status_text(XSTR("Refreshing Public Channel List",952));				

		// get a list of channels on the server (clear any old list as well)
		multi_pxo_get_channels();

		// refresh
		Multi_pxo_channel_last_refresh = -1.0f;

		nprintf(("Network","Refreshing channels\n"));
	}

	// if we haven't updated our server channel counts in a while, do so again
	// last refresh time
	if((Multi_pxo_channel_server_refresh > 0.0f) && ((f2fl(timer_get_fixed_seconds()) - Multi_pxo_channel_server_refresh) > CHANNEL_SERVER_REFRESH_TIME) ){

		// do it _NOW_ I"M RIGHT HERE KILL ME WHAT ARE YOU WAITING FOR DO IT KILL ME DO IT NOW!
		multi_pxo_channel_refresh_servers();		
	}	
}

/**
 * Send a request to refresh our channel server counts
 */
void multi_pxo_channel_refresh_servers()
{
	filter_game_list_struct filter;

	if (Multi_pxo_channels.size() == 0) {
		return;
	}

	// traverse the list of existing channels we know about and query the game tracker about them
	for (auto &channel : Multi_pxo_channels) {
		if (strlen(channel.name)) {
			// copy in the info
			memset(&filter, 0, sizeof(filter_game_list_struct));
			SDL_strlcpy(filter.channel, channel.name, SDL_arraysize(filter.channel));

			// send the request
			RequestGameCountWithFilter(&filter);
		}
	}

	// record the time
	Multi_pxo_channel_server_refresh = f2fl(timer_get_fixed_seconds());
}

/**
 * Refresh current channel server count
 */
void multi_pxo_channel_refresh_current()
{
	// send a request for a server count on this channel
	if ( strlen(Multi_pxo_channel_current.name) ) {
		// fill in the data
		filter_game_list_struct filter;
		memset(&filter, 0, sizeof(filter_game_list_struct));
		SDL_strlcpy(filter.channel, Multi_pxo_channel_current.name, SDL_arraysize(filter.channel));

		// send the request
		RequestGameCountWithFilter(&filter);
	}		
}

/**
 * Display the channel list
 */
void multi_pxo_blit_channels()
{
	char chan_name[MAX_PXO_TEXT_LEN];
	char chan_users[15];
	char chan_servers[15];
	int user_w,server_w;
	int y_start;
	int line_height = gr_get_font_height() + 1;

	// blit as many channels as we can
	y_start = Multi_pxo_chan_coords[gr_screen.res][1];
	if (Multi_pxo_channel_start < 0) {
		return;
	}
	for (int i = 0; i < static_cast<int>(Multi_pxo_channels.size()); i++) {		
		// if this is the currently selected item, highlight it
		if(i == Multi_pxo_channel_select){
			gr_set_color_fast(&Color_bright);
		}
		// otherwise draw it normally
		else {
			gr_set_color_fast(&Color_normal);
		}

		pxo_channel *channel = &Multi_pxo_channels[i];

		// get the # of users on the channel
		memset(chan_users, 0, 15);
		sprintf(chan_users, "%d", channel->num_users);

		// get the width of the user count string
		gr_get_string_size(&user_w, nullptr, chan_users);

		// get the # of servers on the channel
		memset(chan_servers,0,15);
		sprintf(chan_servers, "%d", channel->num_servers);

		// get the width of the user count string
		gr_get_string_size(&server_w, nullptr, chan_servers);

		// make sure the name fits
		memset(chan_name, 0, MAX_PXO_TEXT_LEN);
		Assert(channel->name);
		strcpy_s(chan_name,channel->name);
		font::force_fit_string(chan_name, MAX_PXO_TEXT_LEN-1, Multi_pxo_chan_coords[gr_screen.res][2] - Multi_pxo_chan_column_offsets[gr_screen.res][CHAN_PLAYERS_COLUMN]);

		// blit the strings
		gr_string(Multi_pxo_chan_coords[gr_screen.res][0], y_start, chan_name + 1, GR_RESIZE_MENU);
		gr_string(Multi_pxo_chan_coords[gr_screen.res][0] + Multi_pxo_chan_coords[gr_screen.res][2] - Multi_pxo_chan_column_offsets[gr_screen.res][CHAN_PLAYERS_COLUMN], y_start, chan_users, GR_RESIZE_MENU);
		gr_set_color_fast(&Color_bright);
		gr_string(Multi_pxo_chan_coords[gr_screen.res][0] + Multi_pxo_chan_coords[gr_screen.res][2] - Multi_pxo_chan_column_offsets[gr_screen.res][CHAN_GAMES_COLUMN], y_start, chan_servers, GR_RESIZE_MENU);

		// increment the displayed count
		y_start += line_height;		

	};
}

/**
 * Scroll channel list up
 */
void multi_pxo_scroll_channels_up()
{		
	// if we're already at the head of the list, do nothing
	if(Multi_pxo_channel_start == 0){
		gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
		return;
	}
	
	// otherwise move up one
	Multi_pxo_channel_start--;
	Multi_pxo_channel_start_index--;
	gamesnd_play_iface(InterfaceSounds::USER_SELECT);
}

/**
 * Scroll channel list down
 */
void multi_pxo_scroll_channels_down()
{
	// if we're already at the tail of the list, do nothing
	if(Multi_pxo_channel_start == static_cast<int>(Multi_pxo_channels.size() -1)){
		gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
		return;
	}

	// if we can't scroll further without going past the end of the viewable list, don't
	if((Multi_pxo_channel_start_index + gr_get_dynamic_font_lines(Multi_pxo_max_chan_display[gr_screen.res]) >= static_cast<int>(Multi_pxo_channels.size()))){
		gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
		return;
	}

	// otherwise move down one
	Multi_pxo_channel_start++;
	Multi_pxo_channel_start_index++;
	gamesnd_play_iface(InterfaceSounds::USER_SELECT);
}

/**
 * Attempt to join a channel
 */
void multi_pxo_join_channel(pxo_channel *chan)
{	
	char switch_msg[256];
	
	// if we're already on this channel, do nothing
	if(ON_CHANNEL() && !stricmp(chan->name, Multi_pxo_channel_current.name)){
		return;
	}

	// if we're already trying to join a channel, do nothing
	if(SWITCHING_CHANNELS()){
		return;
	}

	// try and join the channel	
	switch(SetNewChatChannel(chan->name)){
	case -1 :
		Int3();
		break;
		
	case 0 :
		// decrement the count of our current channel
		pxo_channel *lookup;
		lookup = multi_pxo_find_channel(Multi_pxo_channel_current.name);
		if(lookup != nullptr){
			lookup->num_users--;
		}

		// set our current channel as none
		memset(&Multi_pxo_channel_current,0,sizeof(pxo_channel));
		Multi_pxo_channel_current.num_users = -1;

		multi_pxo_set_status_text(XSTR("Switching channels",953));

		// copy the channel
		memcpy(&Multi_pxo_channel_switch,chan,sizeof(pxo_channel));

		// clear the player list
		multi_pxo_clear_players();

		// display a line of text indicating that we're switching channels
		memset(switch_msg,0,256);

		if(strlen(Multi_pxo_channel_switch.name) > 1){
			sprintf(switch_msg, "[Switching to channel %s]", Multi_pxo_channel_switch.name + 1);
		} else {
			sprintf(switch_msg, "[Switching to channel %s]", Multi_pxo_channel_switch.name);
		}

		multi_pxo_chat_process_incoming(switch_msg, CHAT_MODE_CHANNEL_SWITCH);
		break;

	case 1 :
		Int3();		
	}	
}

void multi_pxo_maybe_join_channel(pxo_channel* chan)
{
	// if there are no channels to join, let the user know
	if (Multi_pxo_channels.size() == 0) {
		gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
		multi_pxo_notify_add(XSTR("No channels!", 944));
		return;
	}

	// if we're not already trying to join, allow this
	if (!SWITCHING_CHANNELS() && (Multi_pxo_channel_select >= 0)) {
		gamesnd_play_iface(InterfaceSounds::USER_SELECT);
		multi_pxo_join_channel(chan);
	} else {
		multi_pxo_notify_add(XSTR("Already trying to join a channel!", 945));
		gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
	}
}

/**
 * Handle any processing details if we're currently trying to join a channel
 */
void multi_pxo_handle_channel_change()
{			
	// if we're not switching channels, do nothing
	if(!SWITCHING_CHANNELS()){
		return;
	}

	// if we are, check the status
	switch(SetNewChatChannel(nullptr)){
	// failed to switch
	case -1 :
		// unset our switching struct
		memset(&Multi_pxo_channel_switch,0,sizeof(pxo_channel));
		Multi_pxo_channel_switch.num_users = -1;

		// notify of error
		multi_pxo_set_status_text(XSTR("No channel (error while switching)",954));
		break;

	// still switching
	case 0:
		break;

	// successfully changed
	case 1:
		// copy the current channel info, and unset the switching status
		memcpy(&Multi_pxo_channel_current,&Multi_pxo_channel_switch,sizeof(pxo_channel));
		Multi_pxo_channel_switch.num_users = -1;

		// set our "last" channel
		strcpy_s(Multi_pxo_channel_last, Multi_pxo_channel_current.name);

		// notify the user		
		multi_pxo_set_status_text(XSTR("Connected to Parallax Online",951));

		// if we don't already have this guy on the list, add him
		pxo_channel *lookup;
		lookup = multi_pxo_find_channel(Multi_pxo_channel_current.name);
		if(lookup == nullptr){
			// create a new channel with the given name and place it on the channel list, return a pointer or NULL on fail
			lookup = multi_pxo_add_channel(Multi_pxo_channel_current.name);
		}

		// set the user count to be 1 (just me)
		if(lookup != nullptr){
			lookup->num_users = 1;
		}

		// set the "switch" delay timestamp
		Multi_pxo_switch_delay = ui_timestamp(MULTI_PXO_SWITCH_DELAY_TIME);

		// refresh current channel server count
		multi_pxo_channel_refresh_current();		
		break;
	}
}


// player related stuff -------------------------------------------

/**
 * Clear the old player list
 */
void multi_pxo_clear_players()
{
	Multi_pxo_players.clear();
	Multi_pxo_players.shrink_to_fit();
	Multi_pxo_player_start = 0;	
	Multi_pxo_player_select = -1;
}

/**
 * Create a new player with the given name and place it on the player list, return a pointer or NULL on fail
 */
void multi_pxo_add_player(const char *name)
{
	SCP_string new_player = name;
	Multi_pxo_players.push_back(new_player);
}

/**
 * Remove a player with the given name
 */
void multi_pxo_del_player(const char *name)
{
	if (Multi_pxo_players.size() == 0) {
		return;
	}

	// find and erase the player from the list
	for (size_t i = 0; i < Multi_pxo_players.size(); i++) {
		if (!stricmp(Multi_pxo_players[i].c_str(), name)) {
			Multi_pxo_player_select = -1; // unselect
			Multi_pxo_player_start--;
			Multi_pxo_players.erase(Multi_pxo_players.begin() + i);
			break;
		}
	}

	return;
}

/**
 * Try and find a player with the given name, return a pointer to his entry (or NULL)
 */
int multi_pxo_find_player(const char *name)
{
	if (Multi_pxo_players.empty()) {
		return -1;
	}

	for (int i = 0; i < static_cast<int>(Multi_pxo_players.size()); i++) {
		if (!stricmp(Multi_pxo_players[i].c_str(), name)) {
			return i;
		}
	}

	return -1;
}

/**
 * Process the player list (select, etc)
 */
void multi_pxo_process_players(bool api_access)
{
	int item_index,my;
	
	// if we don't have a start item, but the list is non-null
	if(Multi_pxo_player_start_index < 0){
		Multi_pxo_player_start = 0;
		Multi_pxo_player_start_index = 0;
	}

	// see if we have a mouse click on the channel region
	if(!api_access && Multi_pxo_player_button.pressed()){
		Multi_pxo_player_button.get_mouse_pos(nullptr,&my);

		// index from the top
		item_index = my / (gr_get_font_height() + 1);

		// select the item if possible
		if ((item_index + Multi_pxo_player_start_index) < static_cast<int>(Multi_pxo_players.size())) {
			Multi_pxo_player_select = Multi_pxo_player_start_index;
			for (int idx = 0; idx < item_index; idx++) {
				Multi_pxo_player_select++;
			};
		}

		if ((Multi_pxo_player_select < 0) || (Multi_pxo_player_select >= static_cast<int>(Multi_pxo_players.size()))) {
			Multi_pxo_player_select = -1;
		}
	}
}

/**
 * Display the player list
 */
void multi_pxo_blit_players()
{
	int disp_count,y_start;
	int line_height = gr_get_font_height() + 1;

	// blit as many players as we can
	disp_count = 0;
	y_start = Multi_pxo_player_coords[gr_screen.res][1];
	if(Multi_pxo_players.size() == 0){
		return;
	}

	for (int i = 0; i < static_cast<int>(Multi_pxo_players.size()); i++) {
		if (disp_count < gr_get_dynamic_font_lines(Multi_pxo_max_player_display[gr_screen.res])) {
			if (i == Multi_pxo_player_select) {
				gr_set_color_fast(&Color_bright);
			} else {
				gr_set_color_fast(&Color_normal);
			}

			// make sure the string fits
			char player_name[MAX_PXO_TEXT_LEN];
			strcpy_s(player_name, Multi_pxo_players[i].c_str());
			font::force_fit_string(player_name, MAX_PXO_TEXT_LEN - 1, Multi_pxo_player_coords[gr_screen.res][2]);

			// blit the string
			gr_string(Multi_pxo_player_coords[gr_screen.res][0], y_start, player_name, GR_RESIZE_MENU);

			// increment the displayed count
			disp_count++;
			y_start += line_height;
		}
	}
}

/**
 * Scroll player list up
 */
void multi_pxo_scroll_players_up()
{
	// if we're already at the head of the list, do nothing
	if(Multi_pxo_player_start == 0){
		gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
		return;
	}
	
	// otherwise move up one
	Multi_pxo_player_start--;
	Multi_pxo_player_start_index--;	

	gamesnd_play_iface(InterfaceSounds::USER_SELECT);
}

/**
 * Scroll player list down
 */
void multi_pxo_scroll_players_down()
{
	
	// if we're already at the tail of the list, do nothing
	if (Multi_pxo_player_start == static_cast<int>(Multi_pxo_players.size() - 1)) {
		gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
		return;
	}

	// if we can't scroll further without going past the end of the viewable list, don't
	if ((Multi_pxo_player_start_index + gr_get_dynamic_font_lines(Multi_pxo_max_player_display[gr_screen.res]) >=
			static_cast<int>(Multi_pxo_players.size()))) {
		gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
		return;
	}

	// otherwise move down one
	Multi_pxo_player_start++;
	Multi_pxo_player_start_index++;
	gamesnd_play_iface(InterfaceSounds::USER_SELECT);	
}


// chat text stuff -----------------------------------------

/**
 * Clear all lines of chat text in the chat area
 */
void multi_pxo_chat_clear()
{
	// clear the text in all the lines
	Multi_pxo_chat.clear();
	Multi_pxo_chat_start = 0;
	Multi_pxo_chat_slider.set_numberItems(0);
}

/**
 * Add a line of text
 */
void multi_pxo_chat_add_line(const char *txt, int mode)
{
	chat_line temp;
	strcpy_s(temp.text, txt);
	temp.mode = mode;

	if (Multi_pxo_chat.size() >= MAX_CHAT_LINES) {
		Multi_pxo_chat.erase(Multi_pxo_chat.begin());
	}
	
	Multi_pxo_chat.push_back(temp);

	// if we've reached max chat lines, don't increment
	int count = static_cast<int>(Multi_pxo_chat.size());

	// set the count
	Multi_pxo_chat_slider.set_numberItems(count > gr_get_dynamic_font_lines(Multi_pxo_max_chat_display[gr_screen.res]) ? count : 0, 0);		// the 0 means don't reset

	multi_pxo_goto_bottom();
}

/**
 * Process an incoming line of text
 */
void multi_pxo_chat_process_incoming(const char *txt,int mode)
{
	char msg_total[MULTI_PXO_CHAT_LINE_LEN],line[MULTI_PXO_CHAT_LINE_LEN];
	int	n_lines,idx;
	int	n_chars[20];
	const char	*p_str[20];			//  the initial line (unindented)	
	const char *priv_ptr;	

	// filter out "has left" channel messages, when switching channels
	if((SWITCHING_CHANNELS() || (Multi_pxo_switch_delay.isValid() && !ui_timestamp_elapsed(Multi_pxo_switch_delay))) && 
		multi_pxo_chat_is_left_message(txt)){
		return;
	}
		
	// if the text is a private message, return a pointer to the beginning of the message, otherwise return NULL
	priv_ptr = multi_pxo_chat_is_private(txt);
	if(priv_ptr != NULL){		
		strcpy_s(msg_total, priv_ptr);
	} else {
		strcpy_s(msg_total, txt);
	}	

	// determine what mode to display this text in

	// if this is private chat
	if(priv_ptr != NULL){
		mode = CHAT_MODE_PRIVATE;
	}
	// all other chat
	else {
		// if this is a server message
		if(multi_pxo_is_server_text(txt)){
			mode = CHAT_MODE_SERVER;
		}
		// if this is a MOTD
		else if(multi_pxo_is_motd_text(txt)){
			multi_pxo_motd_add_text(txt);
			return;
		} 
		// if this is the end of motd text
		else if(multi_pxo_is_end_of_motd_text(txt)){
			multi_pxo_set_end_of_motd();
			return;
		}
	}

	// split the text up into as many lines as necessary
	n_lines = split_str(msg_total, Multi_pxo_chat_coords[gr_screen.res][2] - 5, n_chars, p_str, 3, MULTI_PXO_CHAT_LINE_LEN);
	Assert((n_lines != -1) && (n_lines <= 20));
	if((n_lines < 0) || (n_lines > 20)) {
		return;
	}

	// if the string fits on one line
	if(n_lines == 1) {
		multi_pxo_chat_add_line(msg_total,mode);
	}
	// if the string was split into multiple lines
	else {
		// add the first line		
		memcpy(line,p_str[0],n_chars[0]);
		line[n_chars[0]] = '\0';
		multi_pxo_chat_add_line(line,mode);

		// copy the rest of the lines
		for(idx=1; idx<n_lines; idx++){
			memcpy(line,p_str[idx],n_chars[idx]);
			line[n_chars[idx]] = '\0';			
			
			// unless the current mode is server or "switching channels", make all these CHAT_MODE_CARRY
			if((mode != CHAT_MODE_SERVER) && (mode != CHAT_MODE_CHANNEL_SWITCH)){
				mode = CHAT_MODE_CARRY;
			}			
			multi_pxo_chat_add_line(line, mode);
		}
	}	
}

/**
 * Blit the chat text
 */
void multi_pxo_chat_blit()
{
	int token_width;
	
	// blit the title line
	char title[MAX_PXO_TEXT_LEN];
	memset(title,0,MAX_PXO_TEXT_LEN);
	if(ON_CHANNEL()){
		if(strlen(Multi_pxo_channel_current.name) > 1){
			sprintf(title, XSTR("%s on %s", 955), Multi_pxo_nick, Multi_pxo_channel_current.name+1);  // [[ <who> on <channel> ]]
		} else {
			sprintf(title, XSTR("%s on %s", 955), Multi_pxo_nick, Multi_pxo_channel_current.name);	  // [[ <who> on <channel> ]]
		}
	} else {
		strcpy_s(title,XSTR("Parallax Online - No Channel", 956));
	}	
	font::force_fit_string(title, MAX_PXO_TEXT_LEN-1, Multi_pxo_chat_coords[gr_screen.res][2] - 10);
	gr_get_string_size(&token_width,nullptr,title);
	gr_set_color_fast(&Color_normal);
	gr_string(Multi_pxo_chat_coords[gr_screen.res][0] + ((Multi_pxo_chat_coords[gr_screen.res][2] - token_width)/2), Multi_pxo_chat_title_y[gr_screen.res], title, GR_RESIZE_MENU);	
	
	int disp_count, y_start;
	int line_height = gr_get_font_height() + 1;

	// blit as many chat lines as we can
	disp_count = 0;
	y_start = Multi_pxo_chat_coords[gr_screen.res][1];
	if (Multi_pxo_chat.size() == 0) {
		return;
	}

	for (int i = Multi_pxo_chat_start; i < static_cast<int>(Multi_pxo_chat.size()); i++) {
		if (disp_count < gr_get_dynamic_font_lines(Multi_pxo_max_chat_display[gr_screen.res])) {
			SCP_list<chat_line>::iterator line = Multi_pxo_chat.begin();
			std::advance(line, i);
			char* tok;
			switch (line->mode) {
			// if this is text from the server, display it all "bright"
			case CHAT_MODE_SERVER:
				gr_set_color_fast(&Color_bright);
				gr_string(Multi_pxo_chat_coords[gr_screen.res][0], y_start, line->text, GR_RESIZE_MENU);
				break;

			// if this is motd, display it all "bright"
			case CHAT_MODE_MOTD:
				gr_set_color_fast(&Color_bright_white);
				gr_string(Multi_pxo_chat_coords[gr_screen.res][0], y_start, line->text, GR_RESIZE_MENU);
				break;

			// normal mode, just highlight the server
			case CHAT_MODE_PRIVATE:
			case CHAT_MODE_NORMAL:
				char piece[MAX_CHAT_LINE_LEN + 1];
				strcpy_s(piece, line->text);
				tok = strtok(piece, " ");
				if (tok != nullptr) {
					// get the width of just the first "piece"
					gr_get_string_size(&token_width, nullptr, tok);

					// draw it brightly
					gr_set_color_fast(&Color_bright);
					gr_string(Multi_pxo_chat_coords[gr_screen.res][0], y_start, tok, GR_RESIZE_MENU);

					// draw the rest of the string normally
					tok = strtok(nullptr, "");
					if (tok != nullptr) {
						gr_set_color_fast(&Color_normal);
						gr_string(Multi_pxo_chat_coords[gr_screen.res][0] + token_width + 6,
							y_start,
							tok,
							GR_RESIZE_MENU);
					}
				}
				break;

			// carry mode, display with no highlight
			case CHAT_MODE_CARRY:
				gr_set_color_fast(&Color_normal);
				gr_string(Multi_pxo_chat_coords[gr_screen.res][0], y_start, line->text, GR_RESIZE_MENU);
				break;

			// "switching channels mode", display it bright
			case CHAT_MODE_CHANNEL_SWITCH:
				gr_set_color_fast(&Color_bright);
				gr_string(Multi_pxo_chat_coords[gr_screen.res][0], y_start, line->text, GR_RESIZE_MENU);
				break;
			}

			// increment the displayed count
			disp_count++;
			y_start += line_height;
		}
	}
}

/**
 * Scroll to the very bottom of the chat area
 */
void multi_pxo_goto_bottom()
{
	// if we have less than the displayable amount of lines, do nothing
	if(static_cast<int>(Multi_pxo_chat.size()) <= gr_get_dynamic_font_lines(Multi_pxo_max_chat_display[gr_screen.res])){
		// nothing to do for the slider
		Multi_pxo_chat_slider.set_numberItems(0);
		return;
	}

	if (multi_pxo_can_scroll_down())
	{
		// otherwise move back the right # of items
		Multi_pxo_chat_start = static_cast<int>(Multi_pxo_chat.size()) - gr_get_dynamic_font_lines(Multi_pxo_max_chat_display[gr_screen.res]);

		// fixup the start index
		multi_pxo_chat_adjust_start();	
	}
}

/**
 * Scroll the text up
 */
void multi_pxo_scroll_chat_up()
{
	// if we're already at the top of the list, don't do anything	
	if (Multi_pxo_chat_start == 0) {
		gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
		return;
	}

	// otherwise move up one
	Multi_pxo_chat_start--;	
	multi_pxo_chat_adjust_start();	
	gamesnd_play_iface(InterfaceSounds::USER_SELECT);
}

/**
 * Returns 1 if we can scroll down, 0 otherwise
 */
bool multi_pxo_can_scroll_down()
{
	// check if we can move down, return accordingly
	if (static_cast<int>(Multi_pxo_chat.size()) > gr_get_dynamic_font_lines(Multi_pxo_max_chat_display[gr_screen.res])) {
		if (Multi_pxo_chat_start < (static_cast<int>(Multi_pxo_chat.size()) - gr_get_dynamic_font_lines(Multi_pxo_max_chat_display[gr_screen.res]))){
			return true;
		}
	}
	return false;
}

/**
 * Scroll the text down
 */
void multi_pxo_scroll_chat_down()
{
	// if we can move down
	if (multi_pxo_can_scroll_down()) {
		Multi_pxo_chat_start++;		
		multi_pxo_chat_adjust_start();	
		gamesnd_play_iface(InterfaceSounds::USER_SELECT);
	} else {
		gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
	}
}

/**
* Send a chat message
*/
void multi_pxo_chat_send(const char* msg)
{
	// if we're connected to a channel, send the chat to the server
	if (ON_CHANNEL()) {
		const char* result = SendChatString(msg, 1);
		if (result != NULL) {
			multi_pxo_chat_process_incoming(result);
		}
	}
}

/**
 * Process chat controls
 */
void multi_pxo_chat_process()
{
	const char *remainder;
	char msg[512];
	int msg_pixel_width;

	// Bail if the input box is not ready to go
	if (!Multi_pxo_chat_input.is_valid()) {
		return;
	}

	// if the chat line is getting too long, fire off the message, putting the last
	// word on the next input line.
	memset(msg, 0, 512);
	Multi_pxo_chat_input.get_text(msg);
	remainder = "";

	// determine if the width of the string in pixels is > than the inputbox width -- if so,
	// then send the message
	gr_get_string_size(&msg_pixel_width, NULL, msg);
	if ( msg_pixel_width >= (Multi_pxo_input_coords[gr_screen.res][2])) {
		auto last_space = strrchr(msg, ' ');
		if ( last_space ) {
			*last_space = '\0';
			remainder = last_space + 1;
		} else {
			remainder = "";
		}	
		
		// if we're connected to a channel, send the chat to the server
		multi_pxo_chat_send(msg);
		
		// display any remainder of text on the next line
		if (ON_CHANNEL()) {
			Multi_pxo_chat_input.set_text(remainder);
		} else {
			Multi_pxo_chat_input.set_text("");
		}
	} else if((Multi_pxo_chat_input.pressed() && (msg[0] != '\0')) || (strlen(msg) >= MAX_CHAT_LINE_LEN)) { 
		// tack on the null terminator in the boundary case
		size_t x = strlen(msg);
		if(x >= MAX_CHAT_LINE_LEN){
			msg[MAX_CHAT_LINE_LEN-1] = '\0';
		}		

		// ignore "/nick" commands
		if(multi_pxo_is_nick_command(msg)){
			Multi_pxo_chat_input.set_text("");
			return;
		}
		
		// send the chat to the server  		
		// if we're connected to a channel, send the chat to the server
		multi_pxo_chat_send(msg);

		// display any remainder of text on the next line
		if (ON_CHANNEL()) {
			Multi_pxo_chat_input.set_text(remainder);
		} else {
			Multi_pxo_chat_input.set_text("");
		}
	}	
}

// if the text is a private message, return a pointer to the beginning of the message, otherwise return NULL
//XSTR:OFF

// NOTE : DO NOT LOCALIZE THESE STRINGS!!!! THEY ARE CONSTANTS WHICH ARE CHECKED AGAINST 
// PXO CHAT SERVER DATA. THEY CANNOT CHANGE!!!
#define PMSG_FROM			"private message from "
#define PMSG_TO			"private message to "
const char *multi_pxo_chat_is_private(const char *txt)
{
	// quick check
	size_t from_len = strlen( PMSG_FROM );
	if( strlen(txt) > from_len ){	
		// otherwise do a comparison
		if( !strnicmp(txt, PMSG_FROM, from_len) ){
			return &txt[from_len];
		}
	}

	// quick check
	size_t to_len = strlen( PMSG_TO );
	if( strlen(txt) > to_len ){	
		// otherwise do a comparison
		if( !strnicmp(txt, PMSG_TO, to_len) ){
			return &txt[to_len];
		}
	}
	
	return NULL;
}
//XSTR:ON

static const size_t pxo_prefix_len = strlen(MULTI_PXO_SERVER_PREFIX);

/**
 * If the text came from the server
 */
int multi_pxo_is_server_text(const char *txt)
{
	// if the message is prefaced by a ***
	if((strlen(txt) >= pxo_prefix_len) && !strncmp(txt, MULTI_PXO_SERVER_PREFIX, pxo_prefix_len)){
		return 1;
	}

	return 0;
}

static const size_t motd_prefix_len = strlen(PXO_CHAT_MOTD_PREFIX);

/**
 * If the text is message of the day text
 */
int multi_pxo_is_motd_text(const char *txt)
{
	// if we're not on a channel, and this is not a channel switching message assume its coming from a server
	if((strlen(txt) >= motd_prefix_len) && !strncmp(txt, PXO_CHAT_MOTD_PREFIX, motd_prefix_len)){
		return 1;
	}	
	
	return 0;
}

static const size_t end_motd_prefix_len = strlen(PXO_CHAT_END_OF_MOTD_PREFIX);

/**
 * If the text is the end of motd text
 */
int multi_pxo_is_end_of_motd_text(const char *txt)
{
	// if we're not on a channel, and this is not a channel switching message assume its coming from a server
	if((strlen(txt) >= end_motd_prefix_len) && !strncmp(txt, PXO_CHAT_END_OF_MOTD_PREFIX, end_motd_prefix_len)){
		return 1;
	}	
	
	return 0;
}

/**
 * If the text is a "has left message" from the server
 */
int multi_pxo_chat_is_left_message(const char *txt)
{
	// if the text is not server text
	if(!multi_pxo_is_server_text(txt)){
		return 0;
	}

	// check to see if the last portion is the correct wording
	if((strlen(txt) > strlen(MULTI_PXO_HAS_LEFT)) && !strcmp(&txt[strlen(txt) - strlen(MULTI_PXO_HAS_LEFT)], MULTI_PXO_HAS_LEFT)){
		return 1;
	}

	// check the end of the line
	return 0;
}

/**
 * Recalculate the chat start index, and adjust the slider properly
 */
void multi_pxo_chat_adjust_start()
{
	// if we have no chat
	if (static_cast<int>(Multi_pxo_chat.size()) <= gr_get_dynamic_font_lines(Multi_pxo_max_chat_display[gr_screen.res])) {
		Multi_pxo_chat_start = 0;		
		return;
	} else {
		Multi_pxo_chat_slider.force_currentItem(Multi_pxo_chat_start);
	}
}

// motd stuff ---------------------------------------------------------

/**
 * Initialize motd when going into this screen
 */
void multi_pxo_motd_init()
{
	// zero the motd string
	strcpy_s(Pxo_motd, "");

	// haven't gotten it yet
	Pxo_motd_end = 0;

	// haven't read it yet either
	Pxo_motd_read = 0;
}

/**
 * Set the motd text
 */
void multi_pxo_motd_add_text(const char *text)
{
	size_t cur_len = strlen(Pxo_motd);
	size_t new_len;

	// sanity
	if(text == NULL){
		return;
	}

	// make sure its motd text
	Assert(multi_pxo_is_motd_text(text));
	if(!multi_pxo_is_motd_text(text)){
		return;
	}
	
	// if its a 0 line motd
	if(strlen(text) <= motd_prefix_len){
		return;
	}

	// add text to the motd
	new_len = strlen(text + motd_prefix_len) - 1;
	if((cur_len + new_len + 1) < MAX_PXO_MOTD_LEN){
		strcat_s(Pxo_motd, text + motd_prefix_len + 1);
		strcat_s(Pxo_motd, "\n");
		mprintf(("MOTD ADD : %s\n", Pxo_motd));
	}
}

/**
 * Set end of motd
 */
void multi_pxo_set_end_of_motd()
{
	int blink = 1;

	Pxo_motd_end = 1;
	mprintf(("MOTD ALL : %s\n", Pxo_motd));
	
	Pxo_motd_read = 0;

	// do we have an old MOTD file laying around? If so, read it in and see if its the same
	uint old_chksum;
	uint new_chksum;

	// checksum the current motd		
	new_chksum = cf_add_chksum_long(0, (ubyte*)Pxo_motd, strlen(Pxo_motd));		

	// checksum the old motd if its lying around
	CFILE *in = cfopen("oldmotd.txt", "rb");
	if(in != NULL){
		// read the old checksum
		cfread(&old_chksum, sizeof(old_chksum), 1, in);
		cfclose(in);
		
		// same checksum? no blink
		if(new_chksum == old_chksum){
			blink = 0;
		}
	}	
	
	// write out the motd for next time
	if(strlen(Pxo_motd)){
		CFILE *out = cfopen("oldmotd.txt", "wb", CFILE_NORMAL, CF_TYPE_DATA);
		if(out != NULL){
			// write all the text
			cfwrite(&new_chksum, sizeof(new_chksum), 1, out);
			
			// close the outfile
			cfclose(out);
		}
	}
	
	// set the blink stamp
	Pxo_motd_blink_stamp = UI_TIMESTAMP::invalid();
	if(blink){		
		Pxo_motd_blink_on = 0;
		if(!Pxo_motd_blinked_already){
			Pxo_motd_blink_stamp = ui_timestamp(PXO_MOTD_BLINK_TIME);
			Pxo_motd_blink_on = 1;
		}
	}

	Pxo_motd_blinked_already = 1;
}

/**
 * Display the motd dialog
 */
void multi_pxo_motd_dialog()
{
	// mark the motd as read
	Pxo_motd_read = 1;

	// simple popup, with a slider
	popup(PF_USE_AFFIRMATIVE_ICON, 1, POPUP_OK, Pxo_motd);
}

/**
 * Call to maybe blink the motd button
 */
void multi_pxo_motd_maybe_blit()
{
	// if we got the end of the motd, and he hasn't read it yet
	if(Pxo_motd_end && !Pxo_motd_read && Pxo_motd_blink_stamp.isValid()){
		// if the timestamp elapsed, flip the blink flag
		if(ui_timestamp_elapsed(Pxo_motd_blink_stamp)){
			Pxo_motd_blink_on = !Pxo_motd_blink_on;
			Pxo_motd_blink_stamp = ui_timestamp(PXO_MOTD_BLINK_TIME);
		}

		// draw properly
		if(Pxo_motd_blink_on){
			Multi_pxo_buttons[gr_screen.res][MULTI_PXO_MOTD].button.draw_forced(2);
		}
	}	
}


// common dialog stuff ------------------------------------------------

int Multi_pxo_searching = 0; 

/**
 * Initialize the common dialog with the passed max input length
 */
void multi_pxo_com_init(int input_len)
{
	int idx;
	
	// create the interface window
	Multi_pxo_com_window.create(0, 0, gr_screen.max_w_unscaled,gr_screen.max_h_unscaled, 0);
	Multi_pxo_com_window.set_mask_bmap(Multi_pxo_com_mask_fname[gr_screen.res]);	

	// create the interface buttons
	for(idx=0; idx<MULTI_PXO_COM_NUM_BUTTONS; idx++){
		// create the object
		Multi_pxo_com_buttons[gr_screen.res][idx].button.create(&Multi_pxo_com_window, "", Multi_pxo_com_buttons[gr_screen.res][idx].x, Multi_pxo_com_buttons[gr_screen.res][idx].y, 1, 1, 0, 1);

		// set the sound to play when highlighted
		Multi_pxo_com_buttons[gr_screen.res][idx].button.set_highlight_action(common_play_highlight_sound);

		// set the ani for the button
		Multi_pxo_com_buttons[gr_screen.res][idx].button.set_bmaps(Multi_pxo_com_buttons[gr_screen.res][idx].filename);

		// set the hotspot
		Multi_pxo_com_buttons[gr_screen.res][idx].button.link_hotspot(Multi_pxo_com_buttons[gr_screen.res][idx].hotspot);
	}			

	// add xstrs
	for(idx=0; idx<MULTI_PXO_COM_NUM_TEXT; idx++){
		Multi_pxo_com_window.add_XSTR(&Multi_pxo_com_text[gr_screen.res][idx]);
	}

	// create the input box
	Multi_pxo_com_input.create(&Multi_pxo_com_window, Multi_pxo_com_input_coords[gr_screen.res][0], Multi_pxo_com_input_coords[gr_screen.res][1], Multi_pxo_com_input_coords[gr_screen.res][2], input_len, "", UI_INPUTBOX_FLAG_INVIS | UI_INPUTBOX_FLAG_ESC_CLR | UI_INPUTBOX_FLAG_KEYTHRU | UI_INPUTBOX_FLAG_EAT_USED);	
	Multi_pxo_com_input.set_focus();

	// clear all text lines
	memset(Multi_pxo_com_bottom_text, 0, MAX_PXO_TEXT_LEN);
	memset(Multi_pxo_com_middle_text, 0, MAX_PXO_TEXT_LEN);
	memset(Multi_pxo_com_top_text, 0, MAX_PXO_TEXT_LEN);
}

/**
 * Close down the common dialog
 */
void multi_pxo_com_close()
{
	// destroy the UI_WINDOW
	Multi_pxo_com_window.destroy();
}

/**
 * Blit all text lines, top, middle, bottoms
 */
void multi_pxo_com_blit_text()
{
	// blit top, middle and bottom text if possible
	if(Multi_pxo_com_top_text[0] != '\0'){
		gr_set_color_fast(&Color_bright);
		gr_string(Multi_pxo_com_top_text_coords[gr_screen.res][0], Multi_pxo_com_top_text_coords[gr_screen.res][1], Multi_pxo_com_top_text, GR_RESIZE_MENU);
	}
	if(Multi_pxo_com_middle_text[0] != '\0'){
		gr_set_color_fast(&Color_bright);
		gr_string(Multi_pxo_com_top_text_coords[gr_screen.res][0], Multi_pxo_com_middle_text_y[gr_screen.res], Multi_pxo_com_middle_text, GR_RESIZE_MENU);
	}
	if(Multi_pxo_com_bottom_text[0] != '\0'){
		gr_set_color_fast(&Color_bright);
		gr_string(Multi_pxo_com_top_text_coords[gr_screen.res][0], Multi_pxo_com_bottom_text_y[gr_screen.res], Multi_pxo_com_bottom_text, GR_RESIZE_MENU);
	}
}

/**
 * Set the top text, shortening as necessary
 */
void multi_pxo_com_set_top_text(const char *txt)
{	
	if((txt != NULL) && strlen(txt)){
		strcpy_s(Multi_pxo_com_top_text,txt);
		font::force_fit_string(Multi_pxo_com_top_text, MAX_PXO_TEXT_LEN-1, Multi_pxo_com_input_coords[gr_screen.res][2]);
	}	
}

/**
 * Set the middle text, shortening as necessary
 */
void multi_pxo_com_set_middle_text(const char *txt)
{
	if((txt != NULL) && strlen(txt)){
		strcpy_s(Multi_pxo_com_middle_text,txt);
		font::force_fit_string(Multi_pxo_com_middle_text, MAX_PXO_TEXT_LEN-1, Multi_pxo_com_input_coords[gr_screen.res][2]);
	}	
}

/**
 * Set the bottom text, shortening as necessary
 */
void multi_pxo_com_set_bottom_text(const char *txt)
{
	if((txt != NULL) && strlen(txt)){
		strcpy_s(Multi_pxo_com_bottom_text,txt);
		font::force_fit_string(Multi_pxo_com_bottom_text, MAX_PXO_TEXT_LEN-1, Multi_pxo_com_input_coords[gr_screen.res][2]);
	}	
}


// private channel join stuff -----------------------------------------

/**
 * Initialize the popup
 */
void multi_pxo_priv_init()
{
	Assert(Multi_pxo_mode != MULTI_PXO_MODE_PRIVATE);

	// initialize the common dialog with the passed max input length
	multi_pxo_com_init(MULTI_PXO_PRIV_MAX_TEXT_LEN);
	
	// initialize the return code
	Multi_pxo_priv_return_code = -1;	

	// mark us as running
	Multi_pxo_mode = MULTI_PXO_MODE_PRIVATE;

	// set some text
	multi_pxo_com_set_middle_text(XSTR("Type the name of the channel to join/create",961));	
}

/**
 * Close down the popup
 */
void multi_pxo_priv_close()
{	
	// close down the common dialog
	multi_pxo_com_close();

	// mark us as not running any more
	Multi_pxo_mode = MULTI_PXO_MODE_NORMAL;
}

/**
 * Run the popup, 0 if still running, -1 if cancel, 1 if ok
 */
int multi_pxo_priv_popup()
{
	int k;
	
	// if we're not already running, initialize stuff
	if(Multi_pxo_mode != MULTI_PXO_MODE_PRIVATE){
		// intialize
		multi_pxo_priv_init();

		// return "still running"
		return 0;
	}

	k = Multi_pxo_com_window.process();

	// process keypresses
	switch(k){
	// like hitting the cancel button
	case KEY_ESC:
		Multi_pxo_priv_return_code = 0;
		break;
	}

	// process button presses
	multi_pxo_priv_process_buttons();

	// process the inputbox
	multi_pxo_priv_process_input();

	// blit the background
	multi_pxo_blit_all();	

	// blit my stuff		
	gr_reset_clip();	
	gr_set_bitmap(Multi_pxo_com_bitmap);
	gr_bitmap(Multi_pxo_com_coords[gr_screen.res][0], Multi_pxo_com_coords[gr_screen.res][1], GR_RESIZE_MENU);
	Multi_pxo_com_window.draw();	

	// blit all text lines, top, middle, bottoms
	multi_pxo_com_blit_text();

	gr_flip();

	// check the return code
	switch(Multi_pxo_priv_return_code){
	// still in progress
	case -1 :
		return 0;

	// user hit cancel
	case 0 :
		multi_pxo_priv_close();
		return -1;

	// user hit ok
	case 1 :		
		multi_pxo_priv_close();
		return 1;
	}	

	return 0;
}

/**
 * Process button presses
 */
void multi_pxo_priv_process_buttons()
{
	int idx;

	// check all buttons
	for(idx=0;idx<MULTI_PXO_COM_NUM_BUTTONS;idx++){
		if(Multi_pxo_com_buttons[gr_screen.res][idx].button.pressed()){
			multi_pxo_priv_button_pressed(idx);
			return;
		}
	}
}

/**
 * Handle a button press
 */
void multi_pxo_priv_button_pressed(int n)
{
	char priv_chan_name[128];

	switch(n){	
	case MULTI_PXO_COM_CANCEL:
		Multi_pxo_priv_return_code = 0;
		break;
	
	case MULTI_PXO_COM_OK:
		Multi_pxo_com_input.get_text(priv_chan_name);
		multi_pxo_strip_space(priv_chan_name,priv_chan_name);

		// if its a 0 length string, interpret as a cancel
		if(strlen(priv_chan_name) <= 0){
			Multi_pxo_priv_return_code = 0;
			return;
		}

		Multi_pxo_priv_return_code = 1;
		break;
	}	
}

/**
 * Process the inputbox
 */
void multi_pxo_priv_process_input()
{
	char priv_chan_name[128];
	
	// see if the user has pressed enter
	if(Multi_pxo_com_input.pressed()){
		Multi_pxo_com_input.get_text(priv_chan_name);
		multi_pxo_strip_space(priv_chan_name,priv_chan_name);
		
		// if its a 0 length string, interpret as a cancel
		if(strlen(priv_chan_name) <= 0){
			Multi_pxo_priv_return_code = 0;
			return;
		}

		// otherwise interpret as "accept"
		Multi_pxo_priv_return_code = 1;

		// add in the "+" which indicates a private room
		strcpy_s(Multi_pxo_priv_chan,"+");
		strcat_s(Multi_pxo_priv_chan, priv_chan_name);
	}
}

// find player stuff -----------------------------------------

char name_lookup[MAX_PXO_TEXT_LEN];

/**
 * Initialize the popup
 */
void multi_pxo_find_init()
{
	Assert(Multi_pxo_mode != MULTI_PXO_MODE_FIND);

	// initialize the common dialog with the passed max input length
	multi_pxo_com_init(MAX_PLAYER_NAME_LEN);	

	// return code, set to something other than -1 if we're supposed to return
	Multi_pxo_find_return_code = -1;

	// mark us as running
	Multi_pxo_mode = MULTI_PXO_MODE_FIND;	

	// not searching yet
	Multi_pxo_searching = 0; 

	// set the top text
	multi_pxo_com_set_top_text(XSTR("Enter user to be found",962));	

	// 0 length
	strcpy_s(Multi_pxo_find_channel,"");

	// 0 length
	strcpy_s(name_lookup,"");
}

/**
 * Close down the popup
 */
void multi_pxo_find_close()
{
	// close down the common dialog
	multi_pxo_com_close();

	// mark us as not running any more
	Multi_pxo_mode = MULTI_PXO_MODE_NORMAL;
}

/**
 * Run the popup, 0 if still running, -1 if cancel, 1 if ok
 */
int multi_pxo_find_popup()
{
	int k;
	
	// if we're not already running, initialize stuff
	if(Multi_pxo_mode != MULTI_PXO_MODE_FIND){
		// intialize
		multi_pxo_find_init();

		// return "still running"
		return 0;
	}

	k = Multi_pxo_com_window.process();

	// process keypresses
	switch(k){
	// like hitting the cancel button
	case KEY_ESC:
		Multi_pxo_find_return_code = 0;
		break;
	}

	// process button presses
	multi_pxo_find_process_buttons();

	// process the inputbox
	multi_pxo_find_process_input();

	// process search mode if applicable
	multi_pxo_find_search_process();

	// blit the background
	multi_pxo_blit_all();	

	// blit my stuff		
	gr_reset_clip();	
	gr_set_bitmap(Multi_pxo_com_bitmap);
	gr_bitmap(Multi_pxo_com_coords[gr_screen.res][0], Multi_pxo_com_coords[gr_screen.res][1], GR_RESIZE_MENU);
	Multi_pxo_com_window.draw();	

	// blit any text lines
	multi_pxo_com_blit_text();
	
	gr_flip();

	// check the return code
	switch(Multi_pxo_find_return_code){
	// still in progress
	case -1 :
		return 0;

	// user hit cancel
	case 0 :
		// close the popup down
		multi_pxo_find_close();
		return -1;

	// user hit ok
	case 1 :		
		// close the popup down
		multi_pxo_find_close();

		// if we have a channel, join it now if possible
		if(Multi_pxo_find_channel[0] != '\0'){
			pxo_channel *lookup;
			lookup = multi_pxo_find_channel(Multi_pxo_find_channel);
			
			// if we couldn't find it, don't join
			if(lookup != nullptr){				
				multi_pxo_join_channel(lookup);
			}
		}
		return 1;
	}	

	return 0;
}

/**
 * Process button presses
 */
void multi_pxo_find_process_buttons()
{
	int idx;

	// check all buttons
	for(idx=0;idx<MULTI_PXO_COM_NUM_BUTTONS;idx++){
		if(Multi_pxo_com_buttons[gr_screen.res][idx].button.pressed()){
			multi_pxo_find_button_pressed(idx);
			return;
		}
	}
}

/**
 * Handle a button press
 */
void multi_pxo_find_button_pressed(int n)
{
	switch(n){	
	case MULTI_PXO_COM_CANCEL:
		Multi_pxo_find_return_code = 0;
		break;
	
	case MULTI_PXO_COM_OK:
		Multi_pxo_find_return_code = 1;
		break;
	}	
}

/**
 * Process the inputbox
 */
void multi_pxo_find_process_input()
{		
	// see if the user has pressed enter
	if(Multi_pxo_com_input.pressed()){
		// if we're not already in search mode
		if(!Multi_pxo_searching){
			// clear all text
			memset(Multi_pxo_com_middle_text,0,MAX_PXO_TEXT_LEN);
			memset(Multi_pxo_com_bottom_text,0,MAX_PXO_TEXT_LEN);

			Multi_pxo_com_input.get_text(name_lookup);
			multi_pxo_strip_space(name_lookup,name_lookup);

			// never search with a zero length string
			if(name_lookup[0] != '\0'){
				char search_text[512];

				// put us in search mode
				Multi_pxo_searching = 1;

				// look for the guy
				GetChannelByUser(name_lookup);			

				// set the top text
				memset(search_text,0,512);
				sprintf(search_text,XSTR("Searching for %s",963),name_lookup);
				multi_pxo_com_set_top_text(search_text);
			}
			// clear everything
			else {
				memset(Multi_pxo_com_top_text,0,MAX_PXO_TEXT_LEN);
			}
		}
	}
}

/**
 * Process search mode if applicable
 */
void multi_pxo_find_search_process()
{
	char *channel;
	
	// if we're not searching for anything, return
	if(!Multi_pxo_searching){
		return;
	}

	// otherwise check to see if we've found him
	channel = GetChannelByUser(NULL);
	
	// if we've got a result, let the user know
	if(channel){
		// if he couldn't be found
		if((ptr_s)channel == -1){
			multi_pxo_com_set_middle_text(XSTR("User not found",964));									
			strcpy_s(Multi_pxo_find_channel,"");
		} else {	
			if(channel[0] == '*'){
				multi_pxo_com_set_middle_text(XSTR("Player is logged in but is not on a channel",965));				
				strcpy_s(Multi_pxo_find_channel,"");
			} else {
				char p_text[512];
				memset(p_text,0,512);

				// if this guy is on a public channel, display which one
				if(channel[0] == '#'){			
					sprintf(p_text,XSTR("Found %s on :",966),name_lookup);

					// display the results								
					multi_pxo_com_set_middle_text(p_text);								
					multi_pxo_com_set_bottom_text(channel+1);

					// mark down the channel name so we know where to find him
					strcpy_s(Multi_pxo_find_channel,channel);		
					// strip out trailing whitespace
					if(Multi_pxo_find_channel[strlen(Multi_pxo_find_channel) - 1] == ' '){
						Multi_pxo_find_channel[strlen(Multi_pxo_find_channel) - 1] = '\0';
					}				
				}
				// if this is a private channel
				else if(channel[0] == '+'){
					sprintf(p_text,XSTR("Found %s on a private channel",967),name_lookup);
					multi_pxo_com_set_middle_text(p_text);

					strcpy_s(Multi_pxo_find_channel,"");
				}								
			}
		}

		// unset search mode
		Multi_pxo_searching = 0;

		// clear the inputbox
		Multi_pxo_com_input.set_text("");
	}
}


// player info stuff -----------------------------------------

/**
 * Popup conditional functions, returns 10 on successful get of stats
 */
int multi_pxo_pinfo_cond()
{
	// process common stuff
	multi_pxo_process_common();

	// run the networking functions for the PXO API
	multi_pxo_api_process();

	// process depending on what mode we're in
	switch (Multi_pxo_retrieve_mode)
	{
		// we don't need to do anything extra here, just move on to mode 1
		case 0:
		{
			char *ret_string;
			char temp_string[MAX_PXO_TEXT_LEN];
			char *tok;

			// if the thing is non-null, do something		
			ret_string = GetTrackerIdByUser(Multi_pxo_retrieve_name);

			if (ret_string != NULL) {
				// user not-online/not found
				if (reinterpret_cast<ptr_s>(ret_string) == -1) {
					return 1;
				} 

				// user not a tracker pilot
				if ( !stricmp(ret_string,"-1") ) {
					return 1;
				}

				// otherwise parse into his id and callsign
				strcpy_s(temp_string, ret_string);
				tok = strtok(temp_string, " ");
			
				// get tracker id
				if (tok != NULL) {
					strcpy_s(Multi_pxo_retrieve_id, tok);

					// get the callsign
					tok = strtok(NULL, "");

					if (tok != NULL) {
						strcpy_s(Multi_pxo_retrieve_name, tok);
					}
					// failure
					else {
						return 1;
					}
				}
				// failure of some kind or another
				else {
					return 1;
				}			

				Multi_pxo_retrieve_mode = 1;

				return 0;			
			}

			break;
		}

		// initial call to get his stats
		case 1:	
		{			
			// change the popup text
			popup_change_text(XSTR("Getting player stats",968));

			// fill in the data
			memset(&Multi_pxo_pinfo, 0, sizeof(Multi_pxo_pinfo));
			SDL_strlcpy(Multi_pxo_pinfo.pilot_name, Multi_pxo_retrieve_name, SDL_arraysize(Multi_pxo_pinfo.pilot_name));
			Multi_pxo_pinfo.tracker_id = SDL_atoi(Multi_pxo_retrieve_id);

			// make the initial call to the API
			GetFSPilotData(reinterpret_cast<vmt_stats_struct*>(static_cast<uintptr_t>(0xffffffff)), nullptr, nullptr, 0);

			if (GetFSPilotData(&Multi_pxo_pinfo, Multi_pxo_retrieve_name, Multi_pxo_retrieve_id, 0) != 0){
				return 2;
			}
			// if the call went through, set the mode to 2
			else {
				Multi_pxo_retrieve_mode = 2;
			}

			break;
		}
	
		// busy retrieving his stats
		case 2:
		{
			switch (GetFSPilotData(nullptr, nullptr, nullptr, 0)) {
				// timeout, fail, cancel
				case -1:
				case 3:
				case 2:
					return 2;

				// got the data
				case 1:
					return 10;

				// still busy
				case 0:
					break;
			}

			break;
		}
	}

	// return not done yet
	return 0;
}

/**
 * Return 1 if Multi_pxo_pinfo was successfully filled in, 0 otherwise
 */
int multi_pxo_pinfo_get(const char *name)
{
	// run the popup	
	Multi_pxo_retrieve_mode = 0;
	strcpy_s(Multi_pxo_retrieve_name, name);

	switch ( popup_till_condition(multi_pxo_pinfo_cond, XSTR("&Cancel", 779), XSTR("Retrieving player tracker id", 969)) )
	{
		// success
		case 10 :
			return 1;		

		// failed to get his tracker id
		case 1 :
			return 0;

		// failed to get his stats
		case 2 :
			return 0;	
	}

	// we didn't get the stats
	return 0;
}

bool multi_pxo_maybe_get_player(const char* name)
{
	if (multi_pxo_pinfo_get(name)) {
		// convert stats to player
		multi_stats_tracker_to_fs(&Multi_pxo_pinfo, &Multi_pxo_pinfo_player.stats);
		SDL_strlcpy(Multi_pxo_pinfo_player.callsign, Multi_pxo_pinfo.pilot_name, SDL_arraysize(Multi_pxo_pinfo_player.callsign));
		return true;
	}
	return false;
}

/**
 * Fire up the stats view popup
 */
void multi_pxo_pinfo_show()
{
	// initialize the popup
	multi_pxo_pinfo_init();
	
	// run the popup
	do {
		game_set_frametime(GS_STATE_PXO);
	} while ( !multi_pxo_pinfo_do() );

	// close down the popup
	multi_pxo_pinfo_close();
}

/**
 * Build the stats labels values
 */
void multi_pxo_pinfo_build_vals()
{
	player *fs = &Multi_pxo_pinfo_player;	
			
	// pilot name
	memset(Multi_pxo_pinfo_vals[0], 0, 50);
	strcpy_s(Multi_pxo_pinfo_vals[0], fs->callsign);
	font::force_fit_string(Multi_pxo_pinfo_vals[0], 49, Multi_pxo_pinfo_coords[gr_screen.res][2] - (Multi_pxo_pinfo_val_x[gr_screen.res] - Multi_pxo_pinfo_coords[gr_screen.res][0]));

	// rank
	memset(Multi_pxo_pinfo_vals[1], 0, 50);	
	multi_sg_rank_build_name(get_rank_display_name(&Ranks[verify_rank(fs->stats.rank)]).c_str(),
		Multi_pxo_pinfo_vals[1]);	
	font::force_fit_string(Multi_pxo_pinfo_vals[1], 49, Multi_pxo_pinfo_coords[gr_screen.res][2] - (Multi_pxo_pinfo_val_x[gr_screen.res] - Multi_pxo_pinfo_coords[gr_screen.res][0]));

	// kills
	memset(Multi_pxo_pinfo_vals[2], 0, 50);
	sprintf(Multi_pxo_pinfo_vals[2], "%d", fs->stats.kill_count);

	// assists
	memset(Multi_pxo_pinfo_vals[3], 0, 50);
	sprintf(Multi_pxo_pinfo_vals[3], "%d", fs->stats.assists);

	// friendly kills
	memset(Multi_pxo_pinfo_vals[4], 0, 50);
	sprintf(Multi_pxo_pinfo_vals[4], "%d", fs->stats.kill_count - fs->stats.kill_count_ok);

	// missions flown
	memset(Multi_pxo_pinfo_vals[5], 0, 50);
	sprintf(Multi_pxo_pinfo_vals[5], "%d", (int)fs->stats.missions_flown);	

	// flight time	
	memset(Multi_pxo_pinfo_vals[6], 0, 50);
	game_format_time( fl2f((float)fs->stats.flight_time), Multi_pxo_pinfo_vals[6] );	

	// last flown
	memset(Multi_pxo_pinfo_vals[7], 0, 50);
	if (fs->stats.last_flown == 0) {		
		strcpy_s(Multi_pxo_pinfo_vals[7], XSTR("No missions flown", 970) );
	} else {
		time_t tmp_lf = fs->stats.last_flown; // don't cast a pointer to a type that can be either 32 or 64bit
		tm *tmr = gmtime( &tmp_lf );

		if (tmr != NULL)
			strftime(Multi_pxo_pinfo_vals[7], 30, "%m/%d/%y %H:%M", tmr);	
		else
			strcpy_s(Multi_pxo_pinfo_vals[7], "");
	}		

	// primary shots fired
	memset(Multi_pxo_pinfo_vals[8], 0, 50);
	sprintf(Multi_pxo_pinfo_vals[8], "%d", (int)fs->stats.p_shots_fired);

	// primary shots hit
	memset(Multi_pxo_pinfo_vals[9],0,50);
	sprintf(Multi_pxo_pinfo_vals[9], "%d", (int)fs->stats.p_shots_hit);

	// primary hit pct
	memset(Multi_pxo_pinfo_vals[10], 0, 50);
	if (fs->stats.p_shots_fired > 0) {		
		sprintf(Multi_pxo_pinfo_vals[10], "%d%%", (int)((float)fs->stats.p_shots_hit / (float)fs->stats.p_shots_fired * 100.0f));
	} else {		
		strcpy_s(Multi_pxo_pinfo_vals[10], "0%");
	}

	// secondary shots fired
	memset(Multi_pxo_pinfo_vals[11], 0, 50);
	sprintf(Multi_pxo_pinfo_vals[11], "%d", (int)fs->stats.s_shots_fired);

	// secondary shots hit
	memset(Multi_pxo_pinfo_vals[12], 0, 50);
	sprintf(Multi_pxo_pinfo_vals[12], "%d", (int)fs->stats.s_shots_hit);

	// secondary hit pct
	memset(Multi_pxo_pinfo_vals[13], 0, 50);
	if (fs->stats.s_shots_fired > 0) {		
		sprintf(Multi_pxo_pinfo_vals[13], "%d%%", (int)((float)fs->stats.s_shots_hit / (float)fs->stats.s_shots_fired * 100.0f));
	} else {		
		strcpy_s(Multi_pxo_pinfo_vals[13], "0%");
	}

	// primary friendly hits
	memset(Multi_pxo_pinfo_vals[14], 0, 50);
	sprintf(Multi_pxo_pinfo_vals[14], "%u", fs->stats.p_bonehead_hits);

	// primary friendly hit %
	memset(Multi_pxo_pinfo_vals[15], 0, 50);
	if (fs->stats.p_shots_fired > 0) {		
	   sprintf(Multi_pxo_pinfo_vals[15], "%d%%", (int)((float)100.0f*((float)fs->stats.p_bonehead_hits/(float)fs->stats.p_shots_fired)));
	} else {		
		strcpy_s(Multi_pxo_pinfo_vals[15], "0%");
	}

	// secondary friendly hits
	memset(Multi_pxo_pinfo_vals[16], 0, 50);
	sprintf(Multi_pxo_pinfo_vals[16], "%u", fs->stats.s_bonehead_hits);

	// secondary friendly hit %
	memset(Multi_pxo_pinfo_vals[17], 0, 50);
	if (fs->stats.s_shots_fired > 0) {
	   sprintf(Multi_pxo_pinfo_vals[17], "%d%%", (int)((float)100.0f*((float)fs->stats.s_bonehead_hits/(float)fs->stats.s_shots_fired)));
	} else {		
		strcpy_s(Multi_pxo_pinfo_vals[17], "0%");
	}
}

/**
 * Initialize the popup
 */
void multi_pxo_pinfo_init()
{
	int idx;
	
	// create the interface window
	Multi_pxo_pinfo_window.create(0,0,gr_screen.max_w_unscaled,gr_screen.max_h_unscaled,0);
	Multi_pxo_pinfo_window.set_mask_bmap(Multi_pxo_pinfo_mask_fname[gr_screen.res]);	
	
	Multi_pxo_pinfo_bitmap = bm_load(Multi_pxo_pinfo_fname[gr_screen.res]);
	Assert(Multi_pxo_pinfo_bitmap != -1);

	// create the interface buttons
	for(idx=0; idx<MULTI_PXO_PINFO_NUM_BUTTONS; idx++){
		// create the object
		Multi_pxo_pinfo_buttons[gr_screen.res][idx].button.create(&Multi_pxo_pinfo_window, "", Multi_pxo_pinfo_buttons[gr_screen.res][idx].x, Multi_pxo_pinfo_buttons[gr_screen.res][idx].y, 1, 1, 0, 1);

		// set the sound to play when highlighted
		Multi_pxo_pinfo_buttons[gr_screen.res][idx].button.set_highlight_action(common_play_highlight_sound);

		// set the ani for the button
		Multi_pxo_pinfo_buttons[gr_screen.res][idx].button.set_bmaps(Multi_pxo_pinfo_buttons[gr_screen.res][idx].filename);

		// set the hotspot
		Multi_pxo_pinfo_buttons[gr_screen.res][idx].button.link_hotspot(Multi_pxo_pinfo_buttons[gr_screen.res][idx].hotspot);
	}				

	// add xstrs
	for(idx=0; idx<MULTI_PXO_PINFO_NUM_TEXT; idx++){
		Multi_pxo_pinfo_window.add_XSTR(&Multi_pxo_pinfo_text[gr_screen.res][idx]);
	}

	// set up the stats labels
	Multi_pxo_pinfo_stats_labels[0] = vm_strdup(XSTR("Name", 1532));
	Multi_pxo_pinfo_stats_labels[1] = vm_strdup(XSTR("Rank", 1533));
	Multi_pxo_pinfo_stats_labels[2] = vm_strdup(XSTR("Kills", 1534));
	Multi_pxo_pinfo_stats_labels[3] = vm_strdup(XSTR("Assists", 1535));
	Multi_pxo_pinfo_stats_labels[4] = vm_strdup(XSTR("Friendly kills", 1536));
	Multi_pxo_pinfo_stats_labels[5] = vm_strdup(XSTR("Missions flown", 1537));
	Multi_pxo_pinfo_stats_labels[6] = vm_strdup(XSTR("Flight time", 1538));
	Multi_pxo_pinfo_stats_labels[7] = vm_strdup(XSTR("Last flown", 1539));
	Multi_pxo_pinfo_stats_labels[8] = vm_strdup(XSTR("Primary shots fired", 1540));
	Multi_pxo_pinfo_stats_labels[9] = vm_strdup(XSTR("Primary shots hit", 1541));
	Multi_pxo_pinfo_stats_labels[10] = vm_strdup(XSTR("Primary hit %", 1542));
	Multi_pxo_pinfo_stats_labels[11] = vm_strdup(XSTR("Secondary shots fired", 1543));
	Multi_pxo_pinfo_stats_labels[12] = vm_strdup(XSTR("Secondary shots hit", 1544));
	Multi_pxo_pinfo_stats_labels[13] = vm_strdup(XSTR("Secondary hit %", 1545));
	Multi_pxo_pinfo_stats_labels[14] = vm_strdup(XSTR("Primary friendly hits", 1546));
	Multi_pxo_pinfo_stats_labels[15] = vm_strdup(XSTR("Primary friendly hit %", 1547));
	Multi_pxo_pinfo_stats_labels[16] = vm_strdup(XSTR("Secondary friendly hits", 1548));
	Multi_pxo_pinfo_stats_labels[17] = vm_strdup(XSTR("Secondary friendly hit %", 1549));

	// build the stats labels values
	multi_pxo_pinfo_build_vals();
}

/**
 * Do frame
 */
int multi_pxo_pinfo_do()
{
	int k = Multi_pxo_pinfo_window.process();

	// process common stuff
	multi_pxo_process_common();

	// run the networking functions for the PXO API
	multi_pxo_api_process();

	// check to see if he pressed escp
	if(k == KEY_ESC){
		return 1;
	}

	// if he pressed the ok button
	if(Multi_pxo_pinfo_buttons[gr_screen.res][MULTI_PXO_PINFO_OK].button.pressed()){
		return 1;
	}

	// if he pressed the medals buttons, run the medals screen
	if(Multi_pxo_pinfo_buttons[gr_screen.res][MULTI_PXO_PINFO_MEDALS].button.pressed()){
		multi_pxo_run_medals();
	}
	
	// draw stuff

	// blit everything on the "normal" screen
	multi_pxo_blit_all();

	// blit our own stuff
	gr_reset_clip();	
	gr_set_bitmap(Multi_pxo_pinfo_bitmap);
	gr_bitmap(0, 0, GR_RESIZE_MENU);
	Multi_pxo_pinfo_window.draw();	

	// blit the stats themselves
	multi_pxo_pinfo_blit();

	// flip the page
	gr_flip();

	// not done yet
	return 0;
}

/**
 * Close
 */
void multi_pxo_pinfo_close()
{
	int i;

	// destroy the UI_WINDOW
	Multi_pxo_pinfo_window.destroy();

	// unload the bitmap
	if(Multi_pxo_pinfo_bitmap != -1){
		bm_release(Multi_pxo_pinfo_bitmap);
	}

	// free the stats labels strings
	for (i=0; i<MULTI_PXO_PINFO_NUM_LABELS; i++) {
		vm_free(Multi_pxo_pinfo_stats_labels[i]);
	}
}

/**
 * Blit all the stats on this screen
 */
void multi_pxo_pinfo_blit()
{
	int idx;
	int y_start;
	
	// blit all the labels	
	y_start = Multi_pxo_pinfo_coords[gr_screen.res][1];
	for(idx=0; idx<MULTI_PXO_PINFO_NUM_LABELS; idx++){
		// blit the label
		gr_set_color_fast(&Color_bright);
		gr_string(Multi_pxo_pinfo_coords[gr_screen.res][0], y_start, Multi_pxo_pinfo_stats_labels[idx], GR_RESIZE_MENU);

		// blit the label's value
		gr_set_color_fast(&Color_normal);
		gr_string(Multi_pxo_pinfo_val_x[gr_screen.res], y_start, Multi_pxo_pinfo_vals[idx], GR_RESIZE_MENU);

		// spacing
		y_start += Multi_pxo_pinfo_stats_spacing[idx];
	}
}

/**
 * Run the medals screen
 */
void multi_pxo_run_medals()
{
	int ret_code;
	
	// process common stuff
	multi_pxo_process_common();

	// run the networking functions for the PXO API
	multi_pxo_api_process();

	// initialize the freespace data and the player struct
	multi_stats_tracker_to_fs(&Multi_pxo_pinfo, &Multi_pxo_pinfo_player.stats);
	SDL_strlcpy(Multi_pxo_pinfo_player.callsign, Multi_pxo_pinfo.pilot_name, SDL_arraysize(Multi_pxo_pinfo_player.callsign));

	// initialize the medals screen
	medal_main_init(&Multi_pxo_pinfo_player, MM_POPUP);

	// run the medals screen until it says that it should be closed
	do {
		// set frametime and run common functions
		game_set_frametime(-1);
		game_do_state_common( gameseq_get_state() );

		// run the medals screen
		ret_code = medal_main_do();		
	} while(ret_code);

	// close the medals screen down
	medal_main_close();
}


// notify stuff stuff -----------------------------------------

/**
 * Add a notification string
 */
void multi_pxo_notify_add(const char *txt)
{
	// copy the text
	strcpy_s(Multi_pxo_notify_text, txt);

	// set the timestamp
	Multi_pxo_notify_stamp = ui_timestamp(MULTI_PXO_NOTIFY_TIME);
}

/**
 * Blit and process the notification string
 */
void multi_pxo_notify_blit()
{
	int w;

	// if the timestamp is -1, do nothing
	if (!Multi_pxo_notify_stamp.isValid()){
		return;
	}

	// if it has expired, do nothing
	if (ui_timestamp_elapsed(Multi_pxo_notify_stamp)){
		Multi_pxo_notify_stamp = UI_TIMESTAMP::invalid();
	}

	// otherwise blit the text
	gr_set_color_fast(&Color_bright);
	gr_get_string_size(&w,NULL,Multi_pxo_notify_text);
	gr_string((gr_screen.max_w_unscaled - w)/2,MULTI_PXO_NOTIFY_Y,Multi_pxo_notify_text,GR_RESIZE_MENU);
}


/**
 * Initialize the PXO help screen
 */
void multi_pxo_help_init()
{
	int idx;
	
	// load the background bitmap
	Multi_pxo_help_bitmap = bm_load(Multi_pxo_help_fname[gr_screen.res]);
	if(Multi_pxo_help_bitmap < 0){
		// we failed to load the bitmap - this is very bad
		Int3();
	}
	
	// create the interface window
	Multi_pxo_help_window.create(0,0,gr_screen.max_w_unscaled,gr_screen.max_h_unscaled,0);
	Multi_pxo_help_window.set_mask_bmap(Multi_pxo_help_mask_fname[gr_screen.res]);

	// create the interface buttons
	for(idx=0; idx<MULTI_PXO_HELP_NUM_BUTTONS; idx++){
		// create the object
		Multi_pxo_help_buttons[gr_screen.res][idx].button.create(&Multi_pxo_help_window, "", Multi_pxo_help_buttons[gr_screen.res][idx].x, Multi_pxo_help_buttons[gr_screen.res][idx].y, 1, 1, 0, 1);

		// set the sound to play when highlighted
		Multi_pxo_help_buttons[gr_screen.res][idx].button.set_highlight_action(common_play_highlight_sound);

		// set the ani for the button
		Multi_pxo_help_buttons[gr_screen.res][idx].button.set_bmaps(Multi_pxo_help_buttons[gr_screen.res][idx].filename);

		// set the hotspot
		Multi_pxo_help_buttons[gr_screen.res][idx].button.link_hotspot(Multi_pxo_help_buttons[gr_screen.res][idx].hotspot);
	}	
	
	// add xstrs
	for(idx=0; idx<MULTI_PXO_HELP_NUM_TEXT; idx++){
		Multi_pxo_help_window.add_XSTR(&Multi_pxo_help_text[gr_screen.res][idx]);
	}

	multi_pxo_help_load();

	// set the current page to 0
	Multi_pxo_help_cur = 0;
}

/**
 * Do frame for PXO help
 */
void multi_pxo_help_do()
{
	// run api stuff	
	if(Multi_pxo_connected){
		multi_pxo_api_process();
	}

	// process common stuff
	multi_pxo_process_common();

	int k = Multi_pxo_help_window.process();

	// process any keypresses
	switch(k){
	case KEY_ESC:
		gamesnd_play_iface(InterfaceSounds::USER_SELECT);
		gameseq_post_event(GS_EVENT_PXO);
		break;
	}		

	// process button presses
	multi_pxo_help_process_buttons();

	// draw the background, etc
	gr_reset_clip();
	GR_MAYBE_CLEAR_RES(Multi_pxo_help_bitmap);
	if(Multi_pxo_help_bitmap != -1){
		gr_set_bitmap(Multi_pxo_help_bitmap);
		gr_bitmap(0,0,GR_RESIZE_MENU);
	}
	Multi_pxo_help_window.draw();

	// blit the current page
	multi_pxo_help_blit_page();

	// page flip
	gr_flip();
}

/**
 * Close the pxo screen
 */
void multi_pxo_help_close()
{

	// unload any bitmaps
	bm_release(Multi_pxo_help_bitmap);
	
	// destroy the UI_WINDOW
	Multi_pxo_help_window.destroy();

	multi_pxo_help_free();
}

/**
 * Load the help file up
 */
void multi_pxo_help_free()
{
	// free all pages
	for (int idx = 0; idx < Multi_pxo_help_num_pages; idx++) {
		for (int idx2 = 0; idx2 < Multi_pxo_help_pages[idx].num_lines; idx2++) {
			// maybe free
			if (Multi_pxo_help_pages[idx].text[idx2] != NULL) {
				vm_free(Multi_pxo_help_pages[idx].text[idx2]);
				Multi_pxo_help_pages[idx].text[idx2] = NULL;
			}
		}
	}
}

/**
 * Load the help file up
 */
void multi_pxo_help_load()
{
	CFILE *in;	
	help_page *cp;	

	// read in the text file
	in = NULL;
	in = cfopen(MULTI_PXO_HELP_FILE,"rt",CFILE_NORMAL,CF_TYPE_DATA);			
	Assert(in != NULL);
	if(in == NULL){
		return;
	}

	Multi_pxo_help_num_pages = 0;

	// blast all the help pages clear
	memset(Multi_pxo_help_pages, 0, sizeof(help_page) * MULTI_PXO_MAX_PAGES);	
	Multi_pxo_help_num_pages = 0;
	cp = &Multi_pxo_help_pages[0];

	while(!cfeof(in)){
		// malloc the line
		cp->text[cp->num_lines] = (char*)vm_malloc(Multi_pxo_chars_per_line[gr_screen.res]);
		if(cp->text[cp->num_lines] == NULL){
			break;
		}
		
		// read in the next line		
		cfgets(cp->text[cp->num_lines++], Multi_pxo_chars_per_line[gr_screen.res], in);

		// skip to the next page if necessary
		if(cp->num_lines == Multi_pxo_lines_pp[gr_screen.res]){			
			Multi_pxo_help_num_pages++;
			Assert(Multi_pxo_help_num_pages < MULTI_PXO_MAX_PAGES);
			if(Multi_pxo_help_num_pages >= MULTI_PXO_MAX_PAGES){
				Multi_pxo_help_num_pages--;
				break;
			}
			cp = &Multi_pxo_help_pages[Multi_pxo_help_num_pages];
		}
	}

	// close the file
	cfclose(in);
}

/**
 * Blit the current page
 */
void multi_pxo_help_blit_page()
{
	int idx;
	int start_pos;
	int y_start, line_height;
	help_page *cp = &Multi_pxo_help_pages[Multi_pxo_help_cur];
	
	// blit each line
	y_start = Multi_pxo_help_coords[gr_screen.res][1];
	line_height = gr_get_font_height() + 1;
	for(idx=0;idx<cp->num_lines;idx++){
		// if the first symbol is "@", highlight the line
		if(cp->text[idx][0] == '@'){
			gr_set_color_fast(&Color_bright);
			start_pos = 1;
		} else {
			gr_set_color_fast(&Color_normal);
			start_pos = 0;
		}

		// blit the line
		gr_string(Multi_pxo_help_coords[gr_screen.res][0], y_start, cp->text[idx] + start_pos, GR_RESIZE_MENU);

		// increment the y location
		y_start += line_height;
	}
}

/**
 * Process button presses
 */
void multi_pxo_help_process_buttons()
{
	int idx;

	// process all buttons
	for(idx=0;idx<MULTI_PXO_HELP_NUM_BUTTONS;idx++){
		if(Multi_pxo_help_buttons[gr_screen.res][idx].button.pressed()){
			multi_pxo_help_button_pressed(idx);
			return;
		}
	}
}

/**
 * Button pressed
 */
void multi_pxo_help_button_pressed(int n)
{	
	switch(n){
	case MULTI_PXO_HELP_PREV:
		// if we're already at page 0, do nothing
		if(Multi_pxo_help_cur == 0){
			gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
		} else {
			Multi_pxo_help_cur--;
			gamesnd_play_iface(InterfaceSounds::USER_SELECT);
		}
		break;

	case MULTI_PXO_HELP_NEXT:
		// if we're already at max pages, do nothing
		if(Multi_pxo_help_cur == Multi_pxo_help_num_pages){
			gamesnd_play_iface(InterfaceSounds::GENERAL_FAIL);
		} else {
			Multi_pxo_help_cur++;
			gamesnd_play_iface(InterfaceSounds::USER_SELECT);
		}
		break;

	case MULTI_PXO_HELP_CONTINUE:
		gamesnd_play_iface(InterfaceSounds::USER_SELECT);
		gameseq_post_event(GS_EVENT_PXO);
		break;
	}
}

// http banner stuff ---------------------------------------------

/**
 * Initialisation
 */
void multi_pxo_ban_init()
{
	// zero the active banner bitmap
	Multi_pxo_banner.ban_bitmap = -1;	

	Multi_pxo_ban_swap_stamp = UI_TIMESTAMP::invalid();

	// are we doing banners at all?
	if ( os_config_read_uint(nullptr, "PXOBanners", 1) && strlen(Multi_options_g.pxo_banner_url) ) {
		// if we're already in idle mode, we're done downloading for this instance of freespace. pick a random image we already have
		if(Multi_pxo_ban_mode == PXO_BAN_MODE_IDLE){
			Multi_pxo_ban_mode = PXO_BAN_MODE_CHOOSE_RANDOM;		
			return;
		}

		// set ourselves to startup mode	
		Multi_pxo_ban_mode = PXO_BAN_MODE_LIST_STARTUP;
		Multi_pxo_ban_get = NULL;
	} else {
		// set ourselves to idle mode
		Multi_pxo_ban_mode = PXO_BAN_MODE_IDLE;
		Multi_pxo_ban_get = NULL;
	}

	// zero the active banner bitmap
	Multi_pxo_banner.ban_bitmap = -1;	
	Multi_pxo_banner.ban_file.clear();
	Multi_pxo_banner.ban_file_url.clear();
	Multi_pxo_banner.ban_url.clear();
}

/**
 * Process http download details
 */
void multi_pxo_ban_process(bool api_access)
{
	SCP_string url_string;
	SCP_string local_file;

	// if they've been here long enough, maybe swap out the banner
	if ((Multi_pxo_ban_mode == PXO_BAN_MODE_IDLE) && ui_timestamp_elapsed(Multi_pxo_ban_swap_stamp)) {
		Multi_pxo_ban_mode = PXO_BAN_MODE_IMAGES_STARTUP;
		Multi_pxo_ban_swap_stamp = UI_TIMESTAMP::invalid();
	}

	// process stuff
	switch(Multi_pxo_ban_mode){
	// start downloading list
	case PXO_BAN_MODE_LIST_STARTUP:		
		// remote file
		url_string = Multi_options_g.pxo_banner_url;

		if (url_string.back() != '/') {
			url_string.push_back('/');
		}

		url_string += PXO_BANNERS_CONFIG_FILE;

		// add game tag if we've got one (for mod-specific banner config)
		if ( !Multi_fs_tracker_game_tag.empty() ) {
			url_string += "?tag=" + Multi_fs_tracker_game_tag;
		}

		// local file
		cf_create_default_path_string(local_file, CF_TYPE_MULTI_CACHE, PXO_BANNERS_CONFIG_FILE);

		// try creating the file get object
		Multi_pxo_ban_get = new InetGetFile(url_string.c_str(), local_file.c_str(), CF_TYPE_MULTI_CACHE);

		// bad
		if (Multi_pxo_ban_get == NULL) {
			Multi_pxo_ban_mode = PXO_BAN_MODE_IDLE;
			break;
		}

		// go to the downloading list mode
		Multi_pxo_ban_mode = PXO_BAN_MODE_LIST;
		break;

	// downloading list
	case PXO_BAN_MODE_LIST:
		// error
		if ( Multi_pxo_ban_get->IsFileError() ) {
			delete Multi_pxo_ban_get;
			Multi_pxo_ban_get = NULL;
			Multi_pxo_ban_mode = PXO_BAN_MODE_IDLE;
			break;
		} 

		// connecting, receiving
		if ( Multi_pxo_ban_get->IsConnecting() || Multi_pxo_ban_get->IsReceiving() )
			break;

		// done!
		if ( Multi_pxo_ban_get->IsFileReceived() ) {
			delete Multi_pxo_ban_get;
			Multi_pxo_ban_get = NULL;
			Multi_pxo_ban_mode = PXO_BAN_MODE_IMAGES_STARTUP;
		}
		break;

	// start downloading files
	case PXO_BAN_MODE_IMAGES_STARTUP:
		// first thing - parse the banners file and pick a file
		multi_pxo_ban_parse_banner_file();

		// if we have no active file, we're done
		if ( Multi_pxo_banner.ban_file.empty() || Multi_pxo_banner.ban_file_url.empty() ) {
			Multi_pxo_ban_mode = PXO_BAN_MODE_IDLE;
			break;
		}

		// if the file already exists, we're done
		if ( cf_exists(Multi_pxo_banner.ban_file.c_str(), CF_TYPE_MULTI_CACHE) ) {
			Multi_pxo_ban_mode = PXO_BAN_MODE_IMAGES_DONE;
			break;
		}

		// otherwise try and download it				
		cf_create_default_path_string(local_file, CF_TYPE_MULTI_CACHE, Multi_pxo_banner.ban_file.c_str());

		// try creating the file get object
		Multi_pxo_ban_get = new InetGetFile(Multi_pxo_banner.ban_file_url.c_str(), local_file.c_str(), CF_TYPE_MULTI_CACHE);

		// bad
		if (Multi_pxo_ban_get == NULL) {
			Multi_pxo_ban_mode = PXO_BAN_MODE_IDLE;
			break;
		}

		// go to the downloading images mode
		Multi_pxo_ban_mode = PXO_BAN_MODE_IMAGES;
		break;

	// downloading files
	case PXO_BAN_MODE_IMAGES:
		// error
		if ( Multi_pxo_ban_get->IsFileError() ) {
			delete Multi_pxo_ban_get;
			Multi_pxo_ban_get = NULL;
			Multi_pxo_ban_mode = PXO_BAN_MODE_IDLE;
			break;
		} 

		// connecting, receiving
		if ( Multi_pxo_ban_get->IsConnecting() || Multi_pxo_ban_get->IsReceiving() )
			break;

		// done!
		if ( Multi_pxo_ban_get->IsFileReceived() ) {
			delete Multi_pxo_ban_get;
			Multi_pxo_ban_get = NULL;
			Multi_pxo_ban_mode = PXO_BAN_MODE_IMAGES_DONE;
		}
		break;

	// done downloading - maybe load an image
	case PXO_BAN_MODE_IMAGES_DONE:
		// make sure we have a valid filename
		if ( !Multi_pxo_banner.ban_file.empty() ) {
			// if we have a loaded bitmap, unload it
			if (Multi_pxo_banner.ban_bitmap != -1) {
				bm_release(Multi_pxo_banner.ban_bitmap);
			}

			Multi_pxo_banner.ban_bitmap = bm_load(Multi_pxo_banner.ban_file);
		}

		// now we're idle
		Multi_pxo_ban_mode = PXO_BAN_MODE_IDLE;
		break;

	// idle (done with EVERYTHING)
	case PXO_BAN_MODE_IDLE:
		// if the banner button was clicked
		if ( !api_access && Multi_pxo_ban_button.pressed() ) {
			multi_pxo_ban_clicked();
		}

		// set time to choose a new banner to show
		if ( !Multi_pxo_ban_swap_stamp.isValid() ) {
			Multi_pxo_ban_swap_stamp = ui_timestamp(PXO_BANNER_SWAP_TIME);
		}
		break;

	case PXO_BAN_MODE_CHOOSE_RANDOM:
		// just bounce back to IMAGES_STARTUP, which will pick a new banner
		// image and download it if necessary
		Multi_pxo_ban_mode = PXO_BAN_MODE_IMAGES_STARTUP;
		break;
	}
}

/**
 * Close
 */
void multi_pxo_ban_close()
{
	// if we have a currently active transfer
	if(Multi_pxo_ban_get != NULL){
		Multi_pxo_ban_get->AbortGet();
		delete Multi_pxo_ban_get;
		Multi_pxo_ban_get = NULL;
	}

	// if we have a loaded bitmap, unload it
	if(Multi_pxo_banner.ban_bitmap != -1){
		bm_release(Multi_pxo_banner.ban_bitmap);
		Multi_pxo_banner.ban_bitmap = -1;
	}
}

/**
 * Parse the banners file and maybe fill in Multi_pxo_dl_file
 */
void multi_pxo_ban_parse_banner_file()
{
	char file_url[512] = "";
	char banners[10][512];
	char urls[10][512];
	int num_banners, idx;

	CFILE *in = cfopen(PXO_BANNERS_CONFIG_FILE, "rt", CFILE_NORMAL, CF_TYPE_MULTI_CACHE);

	// bad
	if(in == NULL){
		return;
	}

	// clear all strings
	for(idx=0; idx<10; idx++){
		strcpy_s(banners[idx], "");
		strcpy_s(urls[idx], "");
	}

	// get the global banner url
	if(cfgets(file_url, 254, in) == NULL){
		cfclose(in);
		cf_delete(PXO_BANNERS_CONFIG_FILE, CF_TYPE_MULTI_CACHE);
		return;
	}
	drop_leading_white_space(file_url);
	drop_trailing_white_space(file_url);

	// verify that it's a proper url
	if ( strncmp(file_url, "http://", 7) && strncmp(file_url, "ftp://", 6) ) {
		cfclose(in);
		cf_delete(PXO_BANNERS_CONFIG_FILE, CF_TYPE_MULTI_CACHE);
		return;
	}

	// otherwise read in 		
	num_banners = 0;
	while(num_banners < 10){
		// try and get the pcx
		if(cfgets(banners[num_banners], 254, in) == NULL){
			break;
		}
		// try and get the url
		if(cfgets(urls[num_banners], 254, in) == NULL){
			break;
		}

		// strip off trailing and leading whitespace
		drop_leading_white_space(banners[num_banners]);
		drop_trailing_white_space(banners[num_banners]);
		drop_leading_white_space(urls[num_banners]);
		drop_trailing_white_space(urls[num_banners]);

		// got one
		num_banners++;		
	}

	// close the file
	cfclose(in);

	// no banners
	if(num_banners <= 0){		
		return;
	}

	idx = 0;

	if (num_banners > 1) {
		do {
			// randomly pick a file for download
			idx = Random::next(num_banners);
		} while (Multi_pxo_banner.ban_file == banners[idx]);
	}

	// base filename
	Multi_pxo_banner.ban_file = banners[idx];

	// get the full file url
	Multi_pxo_banner.ban_file_url = file_url;

	if (Multi_pxo_banner.ban_file_url.back() != '/') {
		Multi_pxo_banner.ban_file_url.push_back('/');
	}

	Multi_pxo_banner.ban_file_url.append(banners[idx]);

	// url of where to go to when clicked
	Multi_pxo_banner.ban_url = urls[idx];
}

/**
 * Any bitmap or info or whatever
 */
void multi_pxo_ban_draw()
{	
	// if we have a valid bitmap
	if(Multi_pxo_banner.ban_bitmap >= 0){
		// if the mouse is over the banner button, highlight with a rectangle
		if (Multi_pxo_ban_button.is_mouse_on() && !Multi_pxo_banner.ban_url.empty()) {
			gr_set_color_fast(&Color_bright_blue);
			gr_rect(Pxo_ban_coords[gr_screen.res][0] - 1, Pxo_ban_coords[gr_screen.res][1] - 1, Pxo_ban_coords[gr_screen.res][2] + 2, Pxo_ban_coords[gr_screen.res][3] + 2, GR_RESIZE_MENU);
		}

		// draw the bitmap itself
		gr_set_bitmap(Multi_pxo_banner.ban_bitmap);
		gr_bitmap(Pxo_ban_coords[gr_screen.res][0], Pxo_ban_coords[gr_screen.res][1], GR_RESIZE_MENU);
	}
}

/**
 * Called when the URL button is clicked
 */
void multi_pxo_ban_clicked()
{
	// if we have a valid bitmap and URL, launch the URL
	if ( (Multi_pxo_banner.ban_bitmap >= 0) && !Multi_pxo_banner.ban_url.empty() ) {
		multi_pxo_url(Multi_pxo_banner.ban_url.c_str());
	}
}
