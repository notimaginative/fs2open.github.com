/*
 * Copyright (C) Volition, Inc. 2005.  All rights reserved.
 * 
 * All source code herein is the property of Volition, Inc. You may not sell 
 * or otherwise commercially exploit the source or things you created based on the 
 * source.
 *
*/


#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>
#include <netdb.h>
#endif

#include "globalincs/pstypes.h"
#include "io/timer.h"
#include "network/multi.h"
#include "network/multi_pxo.h"
#include "network/gtrack.h"
#include "network/ptrack.h"


// check structs for size compatibility
SDL_COMPILE_TIME_ASSERT(game_packet_header, sizeof(game_packet_header) == 529);
SDL_COMPILE_TIME_ASSERT(freespace2_net_game_data, sizeof(freespace2_net_game_data) == 120);
SDL_COMPILE_TIME_ASSERT(game_list, sizeof(game_list) == 384);
SDL_COMPILE_TIME_ASSERT(filter_game_list_struct, sizeof(filter_game_list_struct) == 40);



//Variables
SOCKADDR_IN	gtrackaddr;

game_list GameBuffer[MAX_GAME_BUFFERS];
int GameType;//d3 or fs

unsigned int LastTrackerUpdate;
unsigned int LastSentToTracker;
unsigned int TrackerAckdUs;
unsigned int TrackerGameIsRunning;

game_packet_header TrackerGameData;
game_packet_header GameListReq;
game_packet_header TrackAckPacket;
game_packet_header GameOverPacket;

freespace2_net_game_data	*FreeSpace2TrackerGameData;

//Start New 7-9-98
unsigned int LastGameOverPacket;
unsigned int FirstGameOverPacket;

int SendingGameOver;
//End New 7-9-98


static int SerializeGamePacket(const game_packet_header *gph, ubyte *data)
{
	int packet_size = 0;

	PXO_ADD_UINT(gph->len);
	PXO_ADD_DATA(gph->game_type);

	PXO_ADD_DATA(gph->junk); // not used, basically just padding for compatibility
	PXO_ADD_INT(gph->type);
	PXO_ADD_UINT(gph->sig);

	switch (gph->type) {
		// these have no other data
		case GNT_CLIENT_ACK:
		case GNT_GAMEOVER:
			break;

		// this one may or may not have extra data
		case GNT_GAMELIST_REQ: {
			if (gph->len > GAME_HEADER_ONLY_SIZE) {
				SDL_assert(gph->len == (GAME_HEADER_ONLY_SIZE+sizeof(filter_game_list_struct)));

				filter_game_list_struct *filter = (filter_game_list_struct *)&gph->data;

				PXO_ADD_INT(filter->rank);
				PXO_ADD_DATA(filter->channel);
				PXO_ADD_DATA(filter->pad);		// for sizing, so gph->len will match
			}

			break;
		}

		case GNT_GAMEUPDATE: {
			pxo_net_game_data *game_data = (pxo_net_game_data *)&gph->data;

			PXO_ADD_DATA(game_data->game_name);
			PXO_ADD_INT(game_data->difficulty);
			PXO_ADD_INT(game_data->type);
			PXO_ADD_INT(game_data->state);
			PXO_ADD_INT(game_data->max_players);
			PXO_ADD_INT(game_data->current_num_players);
			PXO_ADD_DATA(game_data->mission_name);

			PXO_ADD_DATA(game_data->channel);
			PXO_ADD_DATA(game_data->pad);		// for sizing, so gph->len will match

			break;
		}

		case GNT_GAME_COUNT_REQ: {
			SDL_assert(gph->len == (GAME_HEADER_ONLY_SIZE+sizeof(filter_game_list_struct)));

			filter_game_list_struct filter;

			SDL_zero(filter);

			memcpy(filter.channel, gph->data, sizeof(filter.channel));

			PXO_ADD_DATA(filter.channel);

			// add in junk data (ignored on server) to make packet size match
			PXO_ADD_INT(filter.rank);
			PXO_ADD_DATA(filter.pad);

			break;
		}

		// we shouldn't be sending any other packet types
		default:
			Int3();
			break;
	}

	SDL_assert(packet_size >= (int)GAME_HEADER_ONLY_SIZE);
	SDL_assert(packet_size == (int)gph->len);

	return packet_size;
}

static void DeserializeGamePacket(const ubyte *data, const int data_size, game_packet_header *gph)
{
	int offset = 0;
	int i;

	memset(gph, 0, sizeof(game_packet_header));

	// make sure we received a complete base packet
	if (data_size < (int)GAME_HEADER_ONLY_SIZE) {
		gph->len = 0;
		gph->type = 255;	// invalid = 0xff

		return;
	}

	PXO_GET_UINT(gph->len);
	PXO_GET_DATA(gph->game_type);

	PXO_GET_DATA(gph->junk); // not used, basically just padding for compatibility
	PXO_GET_INT(gph->type);
	PXO_GET_UINT(gph->sig);

	// sanity check data size to make sure we reveived all of the expected packet
	// (the -1 is because psnet2 pops off one byte)
	if ((int)gph->len-1 > data_size) {
		gph->len = 0;
		gph->type = -1;

		return;
	}

	switch (gph->type) {
		case GNT_SERVER_ACK:
			break;

		case GNT_GAMELIST_DATA: {
			game_list *games = (game_list *)&gph->data;

			PXO_GET_DATA(games->game_type);

			for (i = 0; i < MAX_GAME_LISTS_PER_PACKET; i++) {
				PXO_GET_DATA(games->game_name[i]);
			}

			PXO_GET_DATA(games->pad);	// padded bytes for alignment

			for (i = 0; i < MAX_GAME_LISTS_PER_PACKET; i++) {
				PXO_GET_UINT(games->game_server[i]);
			}

			for (i = 0; i < MAX_GAME_LISTS_PER_PACKET; i++) {
				PXO_GET_USHORT(games->port[i]);
			}

			break;
		}

		case GNT_GAME_COUNT_DATA: {
			int n_users = 0;
			char channel[512];

			PXO_GET_INT(n_users);

			SDL_strlcpy(channel, (char *)(data+offset), SDL_arraysize(channel));
			offset += (strlen(channel) + 1);

			memcpy(gph->data, &n_users, sizeof(int));
			memcpy(gph->data+sizeof(int), channel, strlen(channel)+1);

			break;
		}

		default:
			break;
	}

	SDL_assert(offset == data_size);
}


int InitGameTrackerClient(int gametype)
{
	SOCKADDR_IN sockaddr;
	unsigned long iaddr;

	GameType = gametype;
	LastTrackerUpdate = 0;
	switch(gametype)
	{
	case GT_FS2OPEN:
		TrackerGameData.len = GAME_HEADER_ONLY_SIZE+sizeof(freespace2_net_game_data);
		break;

	default:
		Int3();
		return 0;
	}
	TrackerGameData.game_type = (unsigned char)gametype;	//1==freespace (GT_FREESPACE), 2==D3, 3==tuberacer, etc.
	TrackerGameData.type = GNT_GAMEUPDATE;	//Used to specify what to do ie. Add a new net game (GNT_GAMESTARTED), remove a net game (game over), etc.

	FreeSpace2TrackerGameData = (freespace2_net_game_data *)&TrackerGameData.data;
	
	GameListReq.game_type = (unsigned char)gametype;
	GameListReq.type = GNT_GAMELIST_REQ;
	GameListReq.len = GAME_HEADER_ONLY_SIZE;

	TrackAckPacket.game_type = (unsigned char)gametype;
	TrackAckPacket.len = GAME_HEADER_ONLY_SIZE;
	TrackAckPacket.type = GNT_CLIENT_ACK;

	GameOverPacket.game_type = (unsigned char)gametype;
	GameOverPacket.len = GAME_HEADER_ONLY_SIZE;
	GameOverPacket.type = GNT_GAMEOVER;

	memset( &sockaddr, 0, sizeof(SOCKADDR_IN) );
	sockaddr.sin_family = AF_INET; 
	sockaddr.sin_addr.s_addr = INADDR_ANY; 
	sockaddr.sin_port = 0;//htons(GAMEPORT);

	iaddr = inet_addr ( Multi_options_g.game_tracker_ip ); 
	if ( iaddr == INADDR_NONE ) {
		// first try and resolve by name
		struct hostent *he;
		he = gethostbyname( Multi_options_g.game_tracker_ip );
		if(!he)
		{		
			return 0;
			/*
			// try and resolve by address		
			unsigned int n_order = inet_addr(Multi_game_tracker_ip_address);
			he = gethostbyaddr((char*)&n_order,4,PF_INET);		

			if(!he){
				return 0;
			}
			*/
		}

		iaddr = ((in_addr *)(he->h_addr))->s_addr;
	}

	// This would be a good place to resolve the IP based on a domain name
	gtrackaddr.sin_addr.s_addr = iaddr;
	gtrackaddr.sin_family = AF_INET; 
	gtrackaddr.sin_port = htons( GAMEPORT );

	//Start New 7-9-98
	SendingGameOver = 0;
	//End New 7-9-98

	return 1;
}

void IdleGameTracker()
{
	fd_set read_fds;	           
	struct timeval timeout;
	ubyte packet_data[sizeof(game_packet_header)];
	int packet_length = 0;

	PSNET_TOP_LAYER_PROCESS();

	timeout.tv_sec=0;            
	timeout.tv_usec=0;
	if((TrackerGameIsRunning) && ((timer_get_seconds()-LastTrackerUpdate)>TRACKER_UPDATE_INTERVAL) && !SendingGameOver)
	{
		//Time to update the tracker again
		packet_length = SerializeGamePacket(&TrackerGameData, packet_data);
		SENDTO(Unreliable_socket, (char *)&packet_data, packet_length, 0, (SOCKADDR *)&gtrackaddr, sizeof(SOCKADDR_IN), PSNET_TYPE_GAME_TRACKER);
		TrackerAckdUs = 0;
		LastTrackerUpdate = timer_get_seconds();
	}
	else if((TrackerGameIsRunning)&&(!TrackerAckdUs)&&((timer_get_milliseconds()-LastSentToTracker)>TRACKER_RESEND_TIME))
	{
		//We still haven't been acked by the last packet and it's time to resend.
		packet_length = SerializeGamePacket(&TrackerGameData, packet_data);
		SENDTO(Unreliable_socket, (char *)&packet_data, packet_length, 0, (SOCKADDR *)&gtrackaddr, sizeof(SOCKADDR_IN), PSNET_TYPE_GAME_TRACKER);
		TrackerAckdUs = 0;
		LastTrackerUpdate = timer_get_seconds();
		LastSentToTracker = timer_get_milliseconds();
	}

	//Start New 7-9-98
	if(SendingGameOver){
		if((timer_get_milliseconds()-LastGameOverPacket)>TRACKER_RESEND_TIME){
			//resend
			packet_length = SerializeGamePacket(&GameOverPacket, packet_data);
			LastGameOverPacket = timer_get_milliseconds();
			SENDTO(Unreliable_socket, (char *)&packet_data, packet_length, 0, (SOCKADDR *)&gtrackaddr, sizeof(SOCKADDR_IN), PSNET_TYPE_GAME_TRACKER);
		} 
		/*
		else if((timer_get_milliseconds()-FirstGameOverPacket)>NET_ACK_TIMEOUT) {
			//Giving up, it timed out.
			SendingGameOver = 2;
		}
		*/
	}
	//End New 7-9-98

	//Check for incoming
		
	FD_ZERO(&read_fds);
	FD_SET(Unreliable_socket, &read_fds);

	if(SELECT(Unreliable_socket+1,&read_fds,NULL,NULL,&timeout, PSNET_TYPE_GAME_TRACKER))
	{
		int bytesin;
		int addrsize;
		SOCKADDR_IN fromaddr;

		game_packet_header inpacket;

		SDL_zero(inpacket);
		addrsize = sizeof(SOCKADDR_IN);

		bytesin = RECVFROM(Unreliable_socket, (char *)&packet_data, sizeof(game_packet_header), 0, (SOCKADDR *)&fromaddr, &addrsize, PSNET_TYPE_GAME_TRACKER);

		if (bytesin > 0) {
			DeserializeGamePacket(packet_data, bytesin, &inpacket);

			// subtract one from the header
			inpacket.len--;
#ifndef NDEBUG
		} else {
			int wserr=WSAGetLastError();
			mprintf(("RECVFROM() failure. WSAGetLastError() returned %d\n",wserr));
#endif
		}

		//Check to make sure the packets ok
		if ( (bytesin > 0) && (bytesin == (int)inpacket.len) )
		{
			switch(inpacket.type)
			{
			case GNT_SERVER_ACK:
				//The server got our packet so we can stop sending now
				TrackerAckdUs = 1;				
				
				// 7/13/98 -- because of the FreeSpace iterative frame process -- set this value to 0, instead
				// of to 2 (as it originally was) since we call SendGameOver() only once.  Once we get the ack
				// from the server, we can assume that we are done.
				// need to mark this as 0
				SendingGameOver = 0;							
				break;
			case GNT_GAMELIST_DATA:
				int i;
				//Woohoo! Game data! put it in the buffer (if one's free)
				for(i=0;i<MAX_GAME_BUFFERS;i++)
				{
					if(GameBuffer[i].game_type==GT_UNUSED)
					{
						memcpy(&GameBuffer[i],&inpacket.data,sizeof(game_list));
						i=MAX_GAME_BUFFERS+1;
					}
				}
				break;

			case GNT_GAME_COUNT_DATA:
				//Here, inpacket.data contains the following structure
				//struct {
				//	int numusers;
				//	char channel[];//Null terminated
				//	}
				//You can add whatever code, or callback, etc. you need to deal with this data

				// let the PXO screen know about this data
				int num_servers;
				char channel[512];

				// get the user count
				memcpy(&num_servers,inpacket.data,sizeof(int));

				// copy the channel name
				SDL_strlcpy(channel, inpacket.data+sizeof(int), SDL_arraysize(channel));

				// send it to the PXO screen				
				multi_pxo_channel_count_update(channel,num_servers);
				break;
			}
			AckPacket(inpacket.sig);			
		}
	}
}

void UpdateGameData(void *buffer)
{
	SendingGameOver = 0;

	switch(GameType){
	case GT_FS2OPEN:
		memcpy(FreeSpace2TrackerGameData,buffer,sizeof(freespace2_net_game_data));
		break;

	default:
		Int3();
		break;
	}
}

game_list * GetGameList()
{
	static game_list gl;
	for(int i=0;i<MAX_GAME_BUFFERS;i++)
	{
		if(GameBuffer[i].game_type!=GT_UNUSED)
		{
			memcpy(&gl,&GameBuffer[i],sizeof(game_list));
			GameBuffer[i].game_type = GT_UNUSED;
			return &gl;
		}
	}
	return NULL;
}

void RequestGameList()
{
	ubyte packet_data[sizeof(game_packet_header)];
	int packet_length = 0;

	GameListReq.len = GAME_HEADER_ONLY_SIZE;

	packet_length = SerializeGamePacket(&GameListReq, packet_data);
	SENDTO(Unreliable_socket, (char *)&packet_data, packet_length, 0, (SOCKADDR *)&gtrackaddr, sizeof(SOCKADDR_IN), PSNET_TYPE_GAME_TRACKER);
}

void RequestGameListWithFilter(void *filter)
{
	ubyte packet_data[sizeof(game_packet_header)];
	int packet_length = 0;

	memcpy(&GameListReq.data,filter,sizeof(filter_game_list_struct));
	GameListReq.len = GAME_HEADER_ONLY_SIZE+sizeof(filter_game_list_struct);

	packet_length = SerializeGamePacket(&GameListReq, packet_data);
	SENDTO(Unreliable_socket, (char *)&packet_data, packet_length, 0, (SOCKADDR *)&gtrackaddr, sizeof(SOCKADDR_IN), PSNET_TYPE_GAME_TRACKER);
}


/* REPLACED BELOW
void SendGameOver()
{
	TrackerGameIsRunning = 0;
	sendto(gamesock,(const char *)&GameOverPacket,GameOverPacket.len,0,(SOCKADDR *)&gtrackaddr,sizeof(SOCKADDR_IN));
}
*/

//Start New 7-9-98
int SendGameOver()
{
	ubyte packet_data[sizeof(game_packet_header)];
	int packet_length = 0;

	if(SendingGameOver==2) 
	{
		SendingGameOver = 0;	
		return 1;
	}
	if(SendingGameOver==1) 
	{
		//Wait until it's sent.
		IdleGameTracker();
		return 0;
	}
	if(SendingGameOver==0)
	{
		LastGameOverPacket = timer_get_milliseconds();
		FirstGameOverPacket = timer_get_milliseconds();
		SendingGameOver = 1;
		TrackerGameIsRunning = 0;

		packet_length = SerializeGamePacket(&GameOverPacket, packet_data);
		SENDTO(Unreliable_socket, (char *)&packet_data, packet_length, 0, (SOCKADDR *)&gtrackaddr, sizeof(SOCKADDR_IN), PSNET_TYPE_GAME_TRACKER);

		return 0;
	}
	return 0;
}
//End New 7-9-98

void AckPacket(int sig)
{
	ubyte packet_data[sizeof(game_packet_header)];
	int packet_length = 0;

	TrackAckPacket.sig = sig;

	packet_length = SerializeGamePacket(&TrackAckPacket, packet_data);
	SENDTO(Unreliable_socket, (char *)&packet_data, packet_length, 0, (SOCKADDR *)&gtrackaddr, sizeof(SOCKADDR_IN), PSNET_TYPE_GAME_TRACKER);
}

void StartTrackerGame(void *buffer)
{
	SendingGameOver = 0;

	switch(GameType){
	case GT_FS2OPEN:
		memcpy(FreeSpace2TrackerGameData,buffer,sizeof(freespace2_net_game_data));
		break;

	default:
		Int3();
		break;
	}
	TrackerGameIsRunning = 1;
	LastTrackerUpdate = 0;	
}

//A new function
void RequestGameCountWithFilter(void *filter) 
{
	game_packet_header GameCountReq;
	ubyte packet_data[sizeof(game_packet_header)];
	int packet_length = 0;

	GameCountReq.game_type = GT_FS2OPEN;
	GameCountReq.type = GNT_GAME_COUNT_REQ;
	GameCountReq.len = GAME_HEADER_ONLY_SIZE+sizeof(filter_game_list_struct);
	memcpy(&GameCountReq.data, ((filter_game_list_struct*)filter)->channel, CHANNEL_LEN);

	packet_length = SerializeGamePacket(&GameCountReq, packet_data);
	SENDTO(Unreliable_socket, (char *)&packet_data, packet_length, 0, (SOCKADDR *)&gtrackaddr, sizeof(SOCKADDR_IN), PSNET_TYPE_GAME_TRACKER);
}
