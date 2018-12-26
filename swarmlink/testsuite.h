/** @file
 *    @brief MAVLink comm protocol testsuite generated from swarmlink.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef SWARMLINK_TESTSUITE_H
#define SWARMLINK_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_ardupilotmega(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_uAvionix(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_icarous(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_swarmlink(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_ardupilotmega(system_id, component_id, last_msg);
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_uAvionix(system_id, component_id, last_msg);
    mavlink_test_icarous(system_id, component_id, last_msg);
    mavlink_test_swarmlink(system_id, component_id, last_msg);
}
#endif

#include "../ardupilotmega/testsuite.h"
#include "../common/testsuite.h"
#include "../uAvionix/testsuite.h"
#include "../icarous/testsuite.h"


static void mavlink_test_swarm_beat(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SWARM_BEAT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_swarm_beat_t packet_in = {
        963497464,963497672,963497880,963498088,129.0,65
    };
    mavlink_swarm_beat_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.flightlevel = packet_in.flightlevel;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.groundspeed = packet_in.groundspeed;
        packet1.gpid = packet_in.gpid;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SWARM_BEAT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SWARM_BEAT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_swarm_beat_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_swarm_beat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_swarm_beat_pack(system_id, component_id, &msg , packet1.gpid , packet1.flightlevel , packet1.lat , packet1.lon , packet1.alt , packet1.groundspeed );
    mavlink_msg_swarm_beat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_swarm_beat_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.gpid , packet1.flightlevel , packet1.lat , packet1.lon , packet1.alt , packet1.groundspeed );
    mavlink_msg_swarm_beat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_swarm_beat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_swarm_beat_send(MAVLINK_COMM_1 , packet1.gpid , packet1.flightlevel , packet1.lat , packet1.lon , packet1.alt , packet1.groundspeed );
    mavlink_msg_swarm_beat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_buddy_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BUDDY_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_buddy_info_t packet_in = {
        963497464,963497672,963497880,963498088,129.0,157.0,77,144
    };
    mavlink_buddy_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.flightlevel = packet_in.flightlevel;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.groundspeed = packet_in.groundspeed;
        packet1.distance = packet_in.distance;
        packet1.gpid = packet_in.gpid;
        packet1.mav_id = packet_in.mav_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BUDDY_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BUDDY_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_buddy_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_buddy_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_buddy_info_pack(system_id, component_id, &msg , packet1.gpid , packet1.mav_id , packet1.flightlevel , packet1.lat , packet1.lon , packet1.alt , packet1.groundspeed , packet1.distance );
    mavlink_msg_buddy_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_buddy_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.gpid , packet1.mav_id , packet1.flightlevel , packet1.lat , packet1.lon , packet1.alt , packet1.groundspeed , packet1.distance );
    mavlink_msg_buddy_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_buddy_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_buddy_info_send(MAVLINK_COMM_1 , packet1.gpid , packet1.mav_id , packet1.flightlevel , packet1.lat , packet1.lon , packet1.alt , packet1.groundspeed , packet1.distance );
    mavlink_msg_buddy_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_buddy_position(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BUDDY_POSITION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_buddy_position_t packet_in = {
        963497464,963497672,963497880,101.0
    };
    mavlink_buddy_position_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.distance = packet_in.distance;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BUDDY_POSITION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BUDDY_POSITION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_buddy_position_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_buddy_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_buddy_position_pack(system_id, component_id, &msg , packet1.lat , packet1.lon , packet1.alt , packet1.distance );
    mavlink_msg_buddy_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_buddy_position_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lat , packet1.lon , packet1.alt , packet1.distance );
    mavlink_msg_buddy_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_buddy_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_buddy_position_send(MAVLINK_COMM_1 , packet1.lat , packet1.lon , packet1.alt , packet1.distance );
    mavlink_msg_buddy_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_buddy_groundspeed(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BUDDY_GROUNDSPEED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_buddy_groundspeed_t packet_in = {
        17.0,45.0
    };
    mavlink_buddy_groundspeed_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.groundspeed = packet_in.groundspeed;
        packet1.distance = packet_in.distance;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_buddy_groundspeed_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_buddy_groundspeed_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_buddy_groundspeed_pack(system_id, component_id, &msg , packet1.groundspeed , packet1.distance );
    mavlink_msg_buddy_groundspeed_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_buddy_groundspeed_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.groundspeed , packet1.distance );
    mavlink_msg_buddy_groundspeed_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_buddy_groundspeed_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_buddy_groundspeed_send(MAVLINK_COMM_1 , packet1.groundspeed , packet1.distance );
    mavlink_msg_buddy_groundspeed_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_global_poi(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GLOBAL_POI >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_global_poi_t packet_in = {
        963497464,963497672,963497880,41,108
    };
    mavlink_global_poi_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.gpid = packet_in.gpid;
        packet1.poi_n = packet_in.poi_n;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GLOBAL_POI_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GLOBAL_POI_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_poi_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_global_poi_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_poi_pack(system_id, component_id, &msg , packet1.gpid , packet1.poi_n , packet1.lat , packet1.lon , packet1.alt );
    mavlink_msg_global_poi_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_poi_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.gpid , packet1.poi_n , packet1.lat , packet1.lon , packet1.alt );
    mavlink_msg_global_poi_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_global_poi_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_poi_send(MAVLINK_COMM_1 , packet1.gpid , packet1.poi_n , packet1.lat , packet1.lon , packet1.alt );
    mavlink_msg_global_poi_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_swarmlink(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_swarm_beat(system_id, component_id, last_msg);
    mavlink_test_buddy_info(system_id, component_id, last_msg);
    mavlink_test_buddy_position(system_id, component_id, last_msg);
    mavlink_test_buddy_groundspeed(system_id, component_id, last_msg);
    mavlink_test_global_poi(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SWARMLINK_TESTSUITE_H
