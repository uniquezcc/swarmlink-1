/** @file
 *	@brief MAVLink comm testsuite protocol generated from swarmlink.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "swarmlink.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(swarmlink, SWARM_BEAT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::swarmlink::msg::SWARM_BEAT packet_in{};
    packet_in.gpid = 65;
    packet_in.flightlevel = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 963498088;
    packet_in.groundspeed = 129.0;

    mavlink::swarmlink::msg::SWARM_BEAT packet1{};
    mavlink::swarmlink::msg::SWARM_BEAT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.gpid, packet2.gpid);
    EXPECT_EQ(packet1.flightlevel, packet2.flightlevel);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.groundspeed, packet2.groundspeed);
}

#ifdef TEST_INTEROP
TEST(swarmlink_interop, SWARM_BEAT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_swarm_beat_t packet_c {
         963497464, 963497672, 963497880, 963498088, 129.0, 65
    };

    mavlink::swarmlink::msg::SWARM_BEAT packet_in{};
    packet_in.gpid = 65;
    packet_in.flightlevel = 963497464;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 963498088;
    packet_in.groundspeed = 129.0;

    mavlink::swarmlink::msg::SWARM_BEAT packet2{};

    mavlink_msg_swarm_beat_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.gpid, packet2.gpid);
    EXPECT_EQ(packet_in.flightlevel, packet2.flightlevel);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.groundspeed, packet2.groundspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
