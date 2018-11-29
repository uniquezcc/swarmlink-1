// MESSAGE SWARM_BEAT support class

#pragma once

namespace mavlink {
namespace swarmlink {
namespace msg {

/**
 * @brief SWARM_BEAT message
 *
 * The swarm beat message is the heartbeat analog for the swarm. It provides the nessessary information about position and speed of an agent.
 */
struct SWARM_BEAT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 271;
    static constexpr size_t LENGTH = 21;
    static constexpr size_t MIN_LENGTH = 21;
    static constexpr uint8_t CRC_EXTRA = 163;
    static constexpr auto NAME = "SWARM_BEAT";


    uint8_t gpid; /*<  Group ID */
    uint32_t flightlevel; /*< [m] Flight level */
    int32_t lat; /*< [degE7] Latitude, expressed */
    int32_t lon; /*< [degE7] Longitude, expressed */
    int32_t alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL. */
    float groundspeed; /*< [m/s] Current ground speed */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  gpid: " << +gpid << std::endl;
        ss << "  flightlevel: " << flightlevel << std::endl;
        ss << "  lat: " << lat << std::endl;
        ss << "  lon: " << lon << std::endl;
        ss << "  alt: " << alt << std::endl;
        ss << "  groundspeed: " << groundspeed << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << flightlevel;                   // offset: 0
        map << lat;                           // offset: 4
        map << lon;                           // offset: 8
        map << alt;                           // offset: 12
        map << groundspeed;                   // offset: 16
        map << gpid;                          // offset: 20
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> flightlevel;                   // offset: 0
        map >> lat;                           // offset: 4
        map >> lon;                           // offset: 8
        map >> alt;                           // offset: 12
        map >> groundspeed;                   // offset: 16
        map >> gpid;                          // offset: 20
    }
};

} // namespace msg
} // namespace swarmlink
} // namespace mavlink
