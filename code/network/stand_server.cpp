/*
 * Copyright (C) Volition, Inc. 1999.  All rights reserved.
 *
 * All source code herein is the property of Volition, Inc. You may not sell
 * or otherwise commercially exploit the source or things you created based on the
 * source.
 *
*/


#include "globalincs/pstypes.h"
#include "network/multi.h"
#include "network/stand_server.h"
#include "osapi/osregistry.h"
#include "network/multi_options.h"
#include "gamesequence/gamesequence.h"
#include "globalincs/version.h"
#include "network/multi_pmsg.h"
#include "network/multi_endgame.h"
#include "network/multimsgs.h"
#include "network/multiui.h"
#include "network/multiutil.h"
#include "freespace.h"
#include "mission/missiongoals.h"
#include "cmdline/cmdline.h"
#include "network/multi_kick.h"
#include "network/multi_fstracker.h"
#include "network/multi_log.h"
#include "playerman/player.h"
#include "ship/ship.h"

PUSH_SUPPRESS_WARNINGS
#include "nlohmann/json.hpp"
#include <libwebsockets.h>
POP_SUPPRESS_WARNINGS

#include <string>
#include <vector>
#include <list>
#include <thread>
#include <atomic>

#include <iostream>

using json = nlohmann::json;


#define STANDALONE_MAX_BAN		50
static std::vector<SCP_string> Standalone_ban_list;

enum UpdateTimes {
	stats	= 1500,
	netgame	= 2500,
	fps		= 250
};

struct Standalone_client {
	uint32_t m_id;
	struct lws *m_wsi;

	SCP_list<SCP_string> m_send_buffer;

	short m_active_player;
	bool m_multilog_enabled;

	uint32_t m_stats_timestamp;

	Standalone_client(uint32_t id, struct lws *wsi) :
		m_id(id), m_wsi(wsi), m_active_player(-1), m_multilog_enabled(false),
		m_stats_timestamp(0) {};

	~Standalone_client() {};
};

class StandaloneUI {
	private:
		lws_context *m_lws_context;
		SCP_string m_interface;
		time_t m_start_time;

		const struct lws_protocols m_lws_protocols[3] = {
			{ "http", lws_callback_http_dummy, 0, 0, 0, nullptr, 0 },
			{ "standalone", ext_callback_standalone, sizeof(uint32_t), 0, 1, this, 0 },
			{ nullptr, nullptr, 0, 0, 0, nullptr, 0 }		// terminator
		};

		struct lws_http_mount *m_lws_mounts;

		SCP_string m_title;
		SCP_string m_state_text;
		json m_popup;

		const uint MAX_MULTILOG_LINES = 100;
		SCP_list<SCP_string> m_multilog;

		uint32_t m_netgame_timestamp;

		uint32_t m_next_client_id;

		SCP_list<Standalone_client *> m_clients;
		const unsigned int MAX_STD_CLIENTS = 5;

		Standalone_client *m_active_client;

		uint32_t getNewClientId() {
			uint32_t next = m_next_client_id++;

			if ( !m_next_client_id ) {
				m_next_client_id = 1;
			}

			return next;
		}

		bool add_message(const json &msg);
		void update_connections();

		void do_frame();

		lws_callback_function callback_standalone;

		int msg_handler(const SCP_string &str, struct lws *wsi);
		void msg_handle_server_config(const json &msg);
		void msg_handle_player(const json &msg);

	public:
		StandaloneUI();
		~StandaloneUI();

		static int ext_callback_standalone(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len) {
			return reinterpret_cast<StandaloneUI*>(lws_get_protocol(wsi)->user)->callback_standalone(wsi, reason, user, in, len);
		}

		void process();

		void reset_all();
		void reset();
		void reset_timestamps();

		void server_set_state(const char *str);
		void server_update_settings();

		void netgame_set_name();
		void netgame_update();

		void player_add(const net_player *p);
		void player_update(const net_player *p);
		void player_info(const net_player *p);
		void player_remove(const net_player *p);

		void chat_add_text(const char *text, int player_index, int add_id);

		void multilog_add_line(const char *line);
		void multilog_refresh();

		void popup_open(const char *title);
		void popup_set_text(const char *str, int field_num);
		void popup_close();

		void mission_set_time(float mission_time);
		void mission_set_goals();
};


static StandaloneUI *Standalone = nullptr;
static std::thread Standalone_thread;
static std::atomic<bool> Standalone_terminate(false);

static void std_lws_logger(int level, const char *line)
{
	if (level & (LLL_WARN|LLL_ERR)) {
		mprintf(("STD: %s", line));
	} else if (level & LLL_NOTICE) {
		nprintf(("lws", "STD: %s", line));
	}
}


StandaloneUI::StandaloneUI()
{
	struct lws_context_creation_info info;

	SDL_zero(info);

	// only needs the single mount
	m_lws_mounts = new struct lws_http_mount;

	SDL_zerop(m_lws_mounts);

	m_lws_mounts->mountpoint = "/";
	m_lws_mounts->mountpoint_len = 1;
	m_lws_mounts->origin = "./standalone-web";
	m_lws_mounts->origin_protocol = LWSMPRO_FILE;
	m_lws_mounts->def = "index.html";

	if ( Multi_options_g.std_listen_addr.empty() ) {
		// this option is to get around a (since fixed) libwebsockets bug that prevented binding
		// to a IPv4 iface address properly
		info.options |= LWS_SERVER_OPTION_DISABLE_IPV6;
		info.iface = "127.0.0.1";
		m_interface = "127.0.0.1";
	} else {
		info.iface = Multi_options_g.std_listen_addr.c_str();
		m_interface = Multi_options_g.std_listen_addr;
	}

	m_interface += SCP_string(":") + std::to_string(Multi_options_g.port);

	info.port = Multi_options_g.port;
	info.protocols = m_lws_protocols;
	info.mounts = m_lws_mounts;

	info.gid = -1;
	info.uid = -1;

	lws_set_log_level(LLL_ERR|LLL_WARN|LLL_NOTICE, std_lws_logger);

	m_lws_context = lws_create_context(&info);

	if (m_lws_context == nullptr) {
		Error(LOCATION, "Unable to initialize standalone server!");
	}

	m_start_time = time(nullptr);

	char title[64];
	SDL_snprintf(title, SDL_arraysize(title), "%s %s", XSTR("FreeSpace Standalone", 935), FS_VERSION_FULL);
	m_title = title;

	m_next_client_id = 1;

	m_active_client = nullptr;

	m_netgame_timestamp = 0;
}

StandaloneUI::~StandaloneUI()
{
	if (m_lws_context) {
		lws_cancel_service(m_lws_context);
		lws_context_destroy(m_lws_context);
		m_lws_context = nullptr;
	}
}

int StandaloneUI::callback_standalone(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len)
{
	#define MAX_BUF_SIZE	2048
	unsigned char buf[LWS_PRE + MAX_BUF_SIZE];
	unsigned char *p = &buf[LWS_PRE];
	int rval;
	int size;
	int exit_val = 0;

	uint32_t *client_id = reinterpret_cast<uint32_t *>(user);

	if (client_id && *client_id) {
		for (auto &cl : m_clients) {
			if (cl->m_id == *client_id) {
				m_active_client = cl;
				break;
			}
		}
	}

	switch (reason) {
		case LWS_CALLBACK_ESTABLISHED: {
			SDL_assert(m_active_client == nullptr);

			uint32_t cid = getNewClientId();

			Standalone_client *new_client = new Standalone_client(cid, wsi);
			m_clients.push_back(new_client);

			*client_id = cid;
			m_active_client = new_client;

			reset();

			break;
		}

		case LWS_CALLBACK_CLOSED: {
			if ( !m_active_client ) {
				break;
			}

			for (auto it = m_clients.begin(); it != m_clients.end(); ++it) {
				if ((*it)->m_id == m_active_client->m_id) {
					m_clients.erase(it);
					break;
				}
			}

			break;
		}

		case LWS_CALLBACK_FILTER_PROTOCOL_CONNECTION: {
			if (m_clients.size() >= MAX_STD_CLIENTS) {
				exit_val = -1;
			}

			break;
		}

		case LWS_CALLBACK_SERVER_WRITEABLE: {
			if ( !m_active_client ) {
				break;
			}

			while ( !m_active_client->m_send_buffer.empty() ) {
				if (m_active_client->m_send_buffer.front().size() >= MAX_BUF_SIZE) {
					lwsl_warn("Message size (%zu) exceeds buffer size (%d)!  Discarding...\n", m_active_client->m_send_buffer.size(), MAX_BUF_SIZE);
					m_active_client->m_send_buffer.pop_front();

					continue;
				}

				size = SDL_strlcpy((char *)p, m_active_client->m_send_buffer.front().c_str(), MAX_BUF_SIZE);

				rval = lws_write(wsi, p, size, LWS_WRITE_TEXT);

				if (rval < size) {
					lwsl_err("ERROR sending buffer!\n");
					lws_close_reason(wsi, LWS_CLOSE_STATUS_UNEXPECTED_CONDITION, (unsigned char *)"write error", 11);

					exit_val = -1;
					break;
				}

				m_active_client->m_send_buffer.pop_front();

				if ( lws_send_pipe_choked(wsi) ) {
					lws_callback_on_writable(wsi);

					break;
				}
			}

			break;
		}

		case LWS_CALLBACK_RECEIVE: {
			if ( !len || !in ) {
				break;
			}

			SCP_string str(reinterpret_cast<const char *>(in), len);

			exit_val = msg_handler(str, wsi);

			break;
		}

		default:
			break;
	}

	m_active_client = nullptr;

	return exit_val;
}

bool StandaloneUI::add_message(const json &msg)
{
	// if no client then don't add messages
	if ( m_clients.empty() ) {
		return false;
	}

	const SCP_string msg_str = msg.dump();

	if (m_active_client) {
		m_active_client->m_send_buffer.push_back(msg_str);
	} else {
		for (auto &client : m_clients) {
			client->m_send_buffer.push_back(msg_str);
		}
	}

	return true;
}

int StandaloneUI::msg_handler(const SCP_string &str, struct lws *wsi)
{
	try {
		json msg = json::parse(str);

		// server control

		if (msg.contains("shutdown")) {
			lws_close_reason(wsi, LWS_CLOSE_STATUS_GOINGAWAY, (unsigned char *)"shutdown", 8);
			gameseq_post_event(GS_EVENT_QUIT_GAME);
			Standalone_terminate = true;

			return -1;
		}

		if (msg.contains("reset_all")) {
			multi_quit_game(PROMPT_NONE);
			reset_all();
		}

		if (msg.contains("validate")) {
			cf_delete(MULTI_VALID_MISSION_FILE, CF_TYPE_DATA);

			multi_update_valid_missions();
		}

		// server config

		auto server_config = msg.find("server");

		if (server_config != msg.end()) {
			msg_handle_server_config( *server_config );
		}

		// player

		auto player = msg.find("player");

		if (player != msg.end()) {
			msg_handle_player( *player );
		}

		// chat

		auto chat_key = msg.find("chat");

		if ( chat_key != msg.end() ) {
			SCP_string txt = *chat_key;

			if ( !txt.empty() ) {
				send_game_chat_packet(Net_player, txt.c_str(), MULTI_MSG_ALL, nullptr);

				std_add_chat_text(txt.c_str(), MY_NET_PLAYER_NUM, 1);
			}
		}

		// multilog

		auto multilog_key = msg.find("multilog");

		if ( multilog_key != msg.end() ) {
			bool enabled = *multilog_key;

			m_active_client->m_multilog_enabled = enabled ? true : false;

			// if we are enabling the multilog then send all that we have to the client
			multilog_refresh();
		}
	} catch (json::exception &e) {
		ml_printf("STD => Exception caught handling client message: %s", e.what());
	}

	return 0;
}

void StandaloneUI::msg_handle_server_config(const json &msg)
{
	// name
	auto name_key = msg.find("name");

	if ( name_key != msg.end() ) {
		SCP_string name = *name_key;

		if ( name.empty() ) {
			name = XSTR("Standalone Server", 916);
		}

		SDL_strlcpy(Multi_options_g.std_pname, name.c_str(), SDL_arraysize(Multi_options_g.std_pname));

		// if no host is connected then set as netgame name too
		if ( !Netgame.host ) {
			SDL_strlcpy(Netgame.name, name.c_str(), SDL_arraysize(Netgame.name));
		}

	}

	// password
	auto password_key = msg.find("password");

	if ( password_key != msg.end() ) {
		SCP_string pass = *password_key;
		SDL_strlcpy(Multi_options_g.std_passwd, pass.c_str(), SDL_arraysize(Multi_options_g.std_passwd));
	}

	// server update rate
	auto update_rate_key = msg.find("update_rate");

	if ( update_rate_key != msg.end() ) {
		int obj_update = *update_rate_key;

		if ( (obj_update >= 0) && (obj_update < MAX_OBJ_UPDATE_LEVELS) ) {
			Multi_options_g.std_datarate = obj_update;
			Net_player->p_info.options.obj_update_level = obj_update;
		}
	}

	// max players
	auto max_players_key = msg.find("max_players");

	if ( max_players_key != msg.end() ) {
		int max_players = *max_players_key;

		if ( (max_players == -1) || !((max_players < 1) || (max_players > MAX_PLAYERS)) ) {
			Multi_options_g.std_max_players = max_players;
		}
	}

	// framecap
	auto framecap_key = msg.find("framecap");

	if ( framecap_key != msg.end() ) {
		int framecap = *framecap_key;
		CAP(framecap, 15, 120);

		Multi_options_g.std_framecap = framecap;
	}

	// PXO
	auto pxo_key = msg.find("pxo");

	if ( pxo_key != msg.end() ) {
		bool pxo = *pxo_key;
		bool using_pxo = Multi_options_g.pxo == 1;

		Multi_options_g.pxo = pxo ? 1 : 0;

		if (pxo && !using_pxo) {
			if ( !multi_fs_tracker_inited() ) {
				multi_fs_tracker_init();
			}

			multi_fs_tracker_login_freespace();
		} else if ( !pxo && using_pxo ) {
			multi_fs_tracker_logout();
		}
	}

	// PXO chat channel
	auto pxo_channel_key = msg.find("pxo_channel");

	if ( pxo_channel_key != msg.end() ) {
		SCP_string pxo_channel = *pxo_channel_key;

		if (pxo_channel != Multi_fs_tracker_channel) {
			SDL_strlcpy(Multi_fs_tracker_channel, pxo_channel.c_str(), SDL_arraysize(Multi_fs_tracker_channel));

			// if pxo is enabled then relog server in new channel
			if (Multi_options_g.pxo) {
				multi_fs_tracker_logout();
				multi_fs_tracker_login_freespace();
			}
		}
	}

	// allow voice
	auto voice_key = msg.find("voice");

	if ( voice_key != msg.end() ) {
		bool voice = *voice_key;
		Multi_options_g.std_voice = voice ? 1 : 0;
	}
}

void StandaloneUI::msg_handle_player(const json &msg)
{
	// kick player
	auto kick_key = msg.find("kick");

	if ( kick_key != msg.end() ) {
		short player_id = *kick_key;
		int idx = find_player_index(player_id);

		multi_kick_player(idx, 0);
	}

	// player info/stats
	auto info_key = msg.find("info");

	if ( info_key != msg.end() ) {
		short player_id = *info_key;
		int idx = find_player_index(player_id);

		if (idx >= 0) {
			player_info(&Net_players[idx]);
			m_active_client->m_active_player = player_id;
		} else {
			m_active_client->m_active_player = -1;
		}
	}
}

void StandaloneUI::update_connections()
{
	for (int i = 0; i < MAX_PLAYERS; i++) {
		net_player *np = &Net_players[i];

		if ( MULTI_CONNECTED((*np)) && (Net_player != np) ) {
			player_add(np);
		}
	}
}

void StandaloneUI::process()
{
	do {
		do_frame();

		lws_service(m_lws_context, 0);

		SDL_Delay(1000/30);
	} while ( !Standalone_terminate );

	lws_cancel_service(m_lws_context);

	SDL_Delay(1000);
}

void StandaloneUI::do_frame()
{
	if ( m_clients.empty() ) {
		return;
	}

	uint32_t cur_time_ms = SDL_GetTicks();

	// maybe update netgame info
	if ( !m_netgame_timestamp || (cur_time_ms > m_netgame_timestamp) ) {
		m_netgame_timestamp = cur_time_ms + UpdateTimes::netgame;

		netgame_update();
	}

	// update client specific stuff
	for (auto &client : m_clients) {
		// maybe update selected player stats
		if ( (client->m_active_player != -1) && (!client->m_stats_timestamp || (cur_time_ms > client->m_stats_timestamp)) ) {
			client->m_stats_timestamp = cur_time_ms + UpdateTimes::stats;

			int player_idx = find_player_index(client->m_active_player);

			if (player_idx >= 0) {
				m_active_client = client;
				player_info(&Net_players[player_idx]);
				m_active_client = nullptr;
			}
		}

		if ( !client->m_send_buffer.empty() ) {
			lws_callback_on_writable(client->m_wsi);
		}
	}
}

void StandaloneUI::server_set_state(const char *str)
{
	m_state_text = str;

	json msg;

	msg["server_info"]["state"] = str;

	add_message(msg);
}

void StandaloneUI::server_update_settings()
{
	json msg;

	if ( SDL_strlen(Multi_options_g.std_pname) ) {
		msg["server"]["name"] = Multi_options_g.std_pname;
	} else {
		msg["server"]["name"] = XSTR("Standalone Server", 916);
	}

	msg["server"]["password"] = Multi_options_g.std_passwd;
	msg["server"]["update_rate"] = Multi_options_g.std_datarate;
	msg["server"]["max_players"] = Multi_options_g.std_max_players;
	msg["server"]["framecap"] = Multi_options_g.std_framecap;
	msg["server"]["voice"] = Multi_options_g.std_voice;
	msg["server"]["pxo"] = Multi_options_g.pxo;
	msg["server"]["pxo_channel"] = Multi_fs_tracker_channel;

	add_message(msg);
}

void StandaloneUI::netgame_set_name()
{
	json msg;

	// use netgame name if it's from host, otherwise trigger field reset
	msg["netgame"]["name"] = Netgame.host ? Netgame.name : "";

	add_message(msg);
}

void StandaloneUI::netgame_update()
{
	json msg;
	SCP_string mode;
	SCP_string type;
	SCP_string state;

	if ( !multi_num_connections() ) {
		return;
	}

	switch (Netgame.mode) {
		case NG_MODE_OPEN:
			mode = XSTR("Open", 1322);
			break;
		case NG_MODE_CLOSED:
			mode = XSTR("Closed", 1323);
			break;
		case NG_MODE_PASSWORD:
			mode = XSTR("Password Protected", 1325);
			break;
		case NG_MODE_RESTRICTED:
			mode = XSTR("Restricted", 1324);
			break;
		case NG_MODE_RANK_ABOVE:
		case NG_MODE_RANK_BELOW:
			mode = "Rank Limited";
			break;
	}

	if (Netgame.type_flags & NG_TYPE_COOP) {
		type = XSTR("Coop", 1257);
	} else if (Netgame.type_flags & NG_TYPE_TEAM) {
		type = XSTR("Team", 1258);

		if (Netgame.type_flags & NG_TYPE_SW) {
			type.append(" (SquadWar)");
		}
	} else if (Netgame.type_flags & NG_TYPE_DOGFIGHT) {
		type = XSTR("Dogfight", 1259);
	}

	switch (Netgame.game_state) {
		case NETGAME_STATE_FORMING:
			state = XSTR("Forming", 764);
			break;
		case NETGAME_STATE_BRIEFING:
			state = XSTR("Briefing", 765);
			break;
		case NETGAME_STATE_DEBRIEF:
		case NETGAME_STATE_ENDGAME:
			state = XSTR("Debrief", 766);
			break;
		case NETGAME_STATE_PAUSED:
			state = XSTR("Paused", 767);
			break;
		case NETGAME_STATE_IN_MISSION:
		case NETGAME_STATE_MISSION_SYNC:
			state = XSTR("Playing", 768);
			break;
		default:
			state = XSTR("Unknown", 769);
	}

	msg["netgame"]["mission_name"] = Netgame.mission_name;
	msg["netgame"]["mission_title"] = Netgame.title;

	if (Netgame.campaign_mode) {
		msg["netgame"]["campaign_name"] = Netgame.campaign_name;
	} else {
		msg["netgame"]["campaign_name"] = "";
	}

	msg["netgame"]["mode"] = mode;
	msg["netgame"]["type"] = type;
	msg["netgame"]["state"] = state;

	msg["netgame"]["max_players"] = Netgame.max_players;
	msg["netgame"]["max_observers"] = Netgame.options.max_observers;
	msg["netgame"]["max_respawns"] = Netgame.respawn;

	add_message(msg);
}

void StandaloneUI::player_add(const net_player *p)
{
	SDL_assert(p);

	char ip_address[INET6_ADDRSTRLEN];
	json msg;

	msg["player"]["add"]["id"] = p->player_id;
	msg["player"]["add"]["name"] = p->m_player->callsign;
	msg["player"]["add"]["ping"] = p->s_info.ping.ping_avg;

	msg["player"]["add"]["host"] = MULTI_HOST((*p)) != 0;
	msg["player"]["add"]["observer"] = MULTI_OBSERVER((*p)) != 0;

	psnet_addr_to_string(&p->p_info.addr, ip_address, SDL_arraysize(ip_address));

	SCP_string address = "[" + SCP_string(ip_address) + "]";
	address += SCP_string(":") + std::to_string(p->p_info.addr.port);

	msg["player"]["add"]["address"] = address;

	add_message(msg);
}

void StandaloneUI::player_update(const net_player *p)
{
	SDL_assert(p);

	json msg;

	msg["player"]["update"]["id"] = p->player_id;
	msg["player"]["update"]["ping"] = p->s_info.ping.ping_avg;

	msg["player"]["update"]["host"] = MULTI_HOST((*p)) != 0;
	msg["player"]["update"]["observer"] = MULTI_OBSERVER((*p)) != 0;

	add_message(msg);
}

void StandaloneUI::player_info(const net_player *p)
{
	json msg;
	json info;
	char temp_str[64];

	info["id"] = p->player_id;
	info["name"] = p->m_player->callsign;
	info["ping"] = p->s_info.ping.ping_avg;

	// ip address
	psnet_addr_to_string(&p->p_info.addr, temp_str, SDL_arraysize(temp_str));

	SCP_string address = "[" + SCP_string(temp_str) + "]";
	address += SCP_string(":") + std::to_string(p->p_info.addr.port);

	info["address"] = address;

	// ship type
	info["ship"] = Ship_info[p->p_info.ship_class].name;

	// rank
	multi_sg_rank_build_name(Ranks[p->m_player->stats.rank].name, temp_str, SDL_arraysize(temp_str));
	info["rank"] = temp_str;

	// flight time
	game_format_time(p->m_player->stats.flight_time, temp_str);
	info["flight_time"] = temp_str;

	// missions
	info["missions_flown"] = p->m_player->stats.missions_flown;

	// stats
	scoring_struct *ptr = &p->m_player->stats;
	std::vector<unsigned int> stats;

	stats.reserve(7);

	// all-time
	stats.push_back(ptr->kill_count);
	stats.push_back(ptr->kill_count - ptr->kill_count_ok);
	stats.push_back(ptr->assists);
	stats.push_back(ptr->p_shots_fired);
	stats.push_back(ptr->p_shots_fired ? (unsigned int)(100.0f * ((float)ptr->p_shots_hit / (float)ptr->p_shots_fired)) : 0);
	stats.push_back(ptr->s_shots_fired);
	stats.push_back(ptr->s_shots_fired ? (unsigned int)(100.0f * ((float)ptr->s_shots_hit / (float)ptr->s_shots_fired)) : 0);

	info["stats"]["all-time"] = stats;

	stats.clear();

	// mission
	stats.push_back(ptr->m_kill_count);
	stats.push_back(ptr->m_kill_count - ptr->m_kill_count_ok);
	stats.push_back(ptr->m_assists);
	stats.push_back(ptr->mp_shots_fired);
	stats.push_back(ptr->mp_shots_fired ? (unsigned int)(100.0f * ((float)ptr->mp_shots_hit / (float)ptr->mp_shots_fired)) : 0);
	stats.push_back(ptr->ms_shots_fired);
	stats.push_back(ptr->ms_shots_fired ? (unsigned int)(100.0f * ((float)ptr->ms_shots_hit / (float)ptr->ms_shots_fired)) : 0);

	info["stats"]["mission"] = stats;

	// final msg layout

	msg["player"]["info"] = info;

	add_message(msg);
}

void StandaloneUI::player_remove(const net_player *p)
{
	json msg;

	msg["player"]["remove"]["id"] = p->player_id;

	add_message(msg);

	// clear active player, client should reset if needed
	for (auto &client : m_clients) {
		if (client->m_active_player == p->player_id) {
			client->m_active_player = -1;
			client->m_stats_timestamp = 0;
		}
	}
}

void StandaloneUI::chat_add_text(const char *text, int player_index, int add_id)
{
	SDL_assert( (player_index >= 0) && (player_index < MAX_PLAYERS) );

	json msg;

	if (add_id) {
		SCP_string id;

		if ( MULTI_STANDALONE(Net_players[player_index]) ) {
			id = XSTR("<SERVER> %s", 924);

			size_t idx = id.find(">");

			if (idx != SCP_string::npos) {
				id.erase(idx+1, SCP_string::npos);
			}
		} else {
			id = Net_players[player_index].m_player->callsign;
		}

		msg["chat"]["id"] = id;
	}

	msg["chat"]["message"] = text;

	add_message(msg);
}

void StandaloneUI::reset_all()
{
	Standalone_client *prev_client = m_active_client;

	for (auto &client : m_clients) {
		m_active_client = client;
		reset();
	}

	m_active_client = prev_client;
}

void StandaloneUI::reset()
{
	if (m_active_client) {
		m_active_client->m_send_buffer.clear();
	}

	json msg;

	// send gui reset
	msg["reset_gui"] = true;

	add_message(msg);

	// server config
	server_update_settings();

	// basic server information
	msg.clear();

	msg["server_info"]["title"] = m_title;
	msg["server_info"]["build"] = gameversion::get_version_string();
	msg["server_info"]["multi_version"] = MULTI_FS_SERVER_VERSION;
	msg["server_info"]["start_time"] = m_start_time;
	msg["server_info"]["state"] = m_state_text;
	msg["server_info"]["address"] = m_interface;

	add_message(msg);

	// refresh various ui elements
	mission_set_time(0.0f);

	// refresh netgame data
	netgame_set_name();
	netgame_update();

	// refresh connections
	update_connections();

	// popup - only if active
	if ( !m_popup.empty() ) {
		msg.clear();
		msg["popup"] = m_popup;

		add_message(msg);
	}

	// multilog - only if active
	if (m_active_client) {
		multilog_refresh();
	}

	// refresh server-side state
	// m_netgame_timestamp = timestamp(UpdateTimes::netgame);

	// refresh client-side state
	if (m_active_client) {
		m_active_client->m_stats_timestamp = 0;
		m_active_client->m_active_player = -1;
	}
}

void StandaloneUI::reset_timestamps()
{
	m_netgame_timestamp = 0;

	for (auto &client : m_clients) {
		client->m_stats_timestamp = 0;
	}
}

void StandaloneUI::multilog_add_line(const char *line)
{
	SDL_assert(line);

	m_multilog.push_back(line);

	if (m_multilog.size() > MAX_MULTILOG_LINES) {
		m_multilog.pop_front();
	}

	Standalone_client *prev_client = m_active_client;
	json msg;

	for (auto &client : m_clients) {
		m_active_client = client;

		if (client->m_multilog_enabled) {
			msg["multilog"] = line;

			add_message(msg);
		}
	}

	m_active_client = prev_client;
}

void StandaloneUI::multilog_refresh()
{
	// active client only
	if ( !m_active_client || !m_active_client->m_multilog_enabled ) {
		return;
	}

	json msg;

	for (auto &line : m_multilog) {
		msg["multilog"] = line;

		add_message(msg);
	}
}

void StandaloneUI::popup_open(const char *title)
{
	SDL_assert(title);

	m_popup["title"] = title;

	m_popup["field1"] = "";
	m_popup["field2"] = "";

	json msg;

	msg["popup"] = m_popup;

	if ( add_message(msg) ) {
		// trigger write callback so we send this message quickly
		for (auto &client : m_clients) {
			lws_callback_on_writable(client->m_wsi);
		}

		lws_service(m_lws_context, 0);
	}
}

void StandaloneUI::popup_set_text(const char *str, int field_num)
{
	SDL_assert(str);

	switch (field_num) {
		case 0:
			m_popup["title"] = str;
			break;

		case 1:
			m_popup["field1"] = str;
			break;

		case 2:
			m_popup["field2"] = str;
			break;

		default:
			return;
	}


	json msg;

	msg["popup"] = m_popup;

	if ( add_message(msg) ) {
		// trigger write callback so we send this message quickly
		for (auto &client : m_clients) {
			lws_callback_on_writable(client->m_wsi);
		}

		lws_service(m_lws_context, 0);
	}
}

void StandaloneUI::popup_close()
{
	json msg = {{ "popup", false }};

	m_popup.clear();

	if ( add_message(msg) ) {
		// trigger write callback so we send this message quickly
		for (auto &client : m_clients) {
			lws_callback_on_writable(client->m_wsi);
		}

		lws_service(m_lws_context, 0);
	}
}

void StandaloneUI::mission_set_time(float mission_time)
{
	json msg;

	msg["mission"]["time"] = mission_time;

	add_message(msg);
}

void StandaloneUI::mission_set_goals()
{
	json primary;
	json secondary;
	json bonus;

	for (int i = 0; i < Num_goals; i++) {
		switch (Mission_goals[i].type & GOAL_TYPE_MASK) {
			case PRIMARY_GOAL: {
				primary.push_back({{ { "name", Mission_goals[i].name }, { "status", Mission_goals[i].satisfied } }});
				break;
			}

			case SECONDARY_GOAL: {
				secondary.push_back({{ { "name", Mission_goals[i].name }, { "status", Mission_goals[i].satisfied } }});
				break;
			}

			case BONUS_GOAL: {
				bonus.push_back({{ { "name", Mission_goals[i].name }, { "status", Mission_goals[i].satisfied } }});
				break;
			}

			default:
				break;
		}
	}

	json msg;

	if ( !primary.empty() ) {
		msg["mission"]["goals"]["primary"] = primary;
	}

	if ( !secondary.empty() ) {
		msg["mission"]["goals"]["secondary"] = secondary;
	}

	if ( !bonus.empty() ) {
		msg["mission"]["goals"]["bonus"] = bonus;
	}

	if ( !msg.empty() ) {
		add_message(msg);
	}
}


void std_deinit_standalone()
{
	Standalone_terminate = true;

	if ( Standalone_thread.joinable() ) {
		Standalone_thread.join();
	}

	if (Standalone) {
		delete Standalone;
		Standalone = nullptr;
	}
}

void std_init_standalone()
{
	if (Standalone) {
		return;
	}

	// turn off all sound and music
	Cmdline_freespace_no_sound = 1;
	Cmdline_freespace_no_music = 1;

	Standalone = new StandaloneUI();

	Standalone_thread = std::thread(&StandaloneUI::process, Standalone);

	atexit(std_deinit_standalone);
}


void std_do_gui_frame()
{
}

void std_debug_set_standalone_state_string(const char *str)
{
	if ( !Standalone ) {
		return;
	}

	Standalone->server_set_state(str);
}

void std_connect_set_gamename(const char *name)
{
	if (name == nullptr) {
		// if a	permanent name exists, use that instead of the default
		if ( SDL_strlen(Multi_options_g.std_pname) ) {
			SDL_strlcpy(Netgame.name, Multi_options_g.std_pname, SDL_arraysize(Netgame.name));
		} else {
			SDL_strlcpy(Netgame.name, XSTR("Standalone Server", 916), SDL_arraysize(Netgame.name));
		}
	} else {
		SDL_strlcpy(Netgame.name, name, SDL_arraysize(Netgame.name));
	}

	if ( !Standalone ) {
		return;
	}

	Standalone->netgame_set_name();
}

int std_connect_set_connect_count()
{
	return 0;
}

void std_add_player(net_player *p)
{
	if ( !p || !Standalone ) {
		return;
	}

	Standalone->player_add(p);
}

int std_remove_player(net_player *p)
{
	if ( !p || !Standalone ) {
		return 0;
	}

	Standalone->player_remove(p);

	// update the currently connected players
	int count = multi_num_connections();

	if (count == 0) {
		multi_quit_game(PROMPT_NONE);
		return 1;
	}

	return 0;
}

void std_update_player_ping(net_player *p)
{
	if ( !p || !Standalone ) {
		return;
	}

	Standalone->player_update(p);
}

void std_add_chat_text(const char *text, int player_index, int add_id)
{
	if ( (player_index < 0) || (player_index >= MAX_PLAYERS) ) {
		return;
	}

	if ( !Standalone ) {
		return;
	}

	Standalone->chat_add_text(text, player_index, add_id);
}

void std_reset_timestamps()
{
	if ( !Standalone ) {
		return;
	}

	Standalone->reset_timestamps();
}

void std_add_ban(const char *name)
{
	if ( (name == nullptr) || !SDL_strlen(name) ) {
		return;
	}

	if (Standalone_ban_list.size() >= STANDALONE_MAX_BAN) {
		return;
	}

	Standalone_ban_list.push_back(name);
}

int std_player_is_banned(const char *name)
{
	if ( Standalone_ban_list.empty() ) {
		return 0;
	}

	for (size_t i = 0; i < Standalone_ban_list.size(); i++) {
		if ( !SDL_strcasecmp(name, Standalone_ban_list[i].c_str()) ) {
			return 1;
		}
	}

	return 0;
}

int std_is_host_passwd()
{
	return (SDL_strlen(Multi_options_g.std_passwd) > 0) ? 1 : 0;
}

void std_multi_set_standalone_mission_name(const char * /*mission_name*/)
{
}

void std_multi_set_standalone_missiontime(float mission_time)
{
	if (Standalone) {
		Standalone->mission_set_time(mission_time);
	}
}

void std_multi_update_netgame_info_controls()
{
	if (Standalone) {
		Standalone->netgame_update();
	}
}

void std_set_standalone_fps(float /*fps*/)
{
}

void std_multi_setup_goal_tree()
{
	if (Standalone) {
		Standalone->mission_set_goals();
	}
}

void std_multi_add_goals()
{
	std_multi_setup_goal_tree();
}

void std_multi_update_goals()
{
	std_multi_setup_goal_tree();
}

void std_reset_standalone_gui()
{
	if ( !Standalone ) {
		return;
	}

	Standalone->reset_all();
}

void std_create_gen_dialog(const char *title)
{
	if ( !title || !Standalone ) {
		return;
	}

	Standalone->popup_open(title);
}

void std_destroy_gen_dialog()
{
	if (Standalone) {
		Standalone->popup_close();
	}
}

void std_gen_set_text(const char *str, int field_num)
{
	if ( !str || !Standalone ) {
		return;
	}

	Standalone->popup_set_text(str, field_num);
}

void std_tracker_notify_login_fail()
{
}

void std_tracker_login()
{
	if ( !Multi_options_g.pxo ) {
		return;
	}

	multi_fs_tracker_init();

	if ( !multi_fs_tracker_inited() ) {
		std_tracker_notify_login_fail();
		return;
	}

	multi_fs_tracker_login_freespace();
}

void std_connect_set_host_connect_status()
{
}

void std_multilog_add_line(const char *line)
{
	if ( !line || !line[0] ) {
		return;
	}

	if ( !Standalone ) {
		return;
	}

	Standalone->multilog_add_line(line);
}
