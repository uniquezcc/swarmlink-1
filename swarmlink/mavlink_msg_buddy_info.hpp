// MESSAGE BUDDY_INFO support class

#pragma once

namespace mavlink {
namespace swarmlink {
namespace msg {

/**
 * @brief BUDDY_INFO message
 *
 * This message contains information about boid.
 */
struct BUDDY_INFO : mavlink::Message {
    static constexpr msgid_t MSG_ID = 272;
    static constexpr size_t LENGTH = 26;
    static constexpr size_t MIN_LENGTH = 26;
    static constexpr uint8_t CRC_EXTRA = 151;
    static constexpr auto NAME = "BUDDY_INFO";


    uint8_t gpid; /*<  Group ID */
    uint8_t mav_id; /*<  MAV ID */
    uint32_t flightlevel; /*< [m] Flight level */
    int32_t lat; /*< [degE7] Latitude, expressed */
    int32_t lon; /*< [degE7] Longitude, expressed */
    int32_t alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL. */
    float groundspeed; /*< [m/s] Current ground speed */
    float distance; /*< [m/s] Distance to buddy */


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
        ss << "  mav_id: " << +mav_id << std::endl;
        ss << "  flightlevel: " << flightlevel << std::endl;
        ss << "  lat: " << lat << std::endl;
        ss << "  lon: " << lon << std::endl;
        ss << "  alt: " << alt << std::endl;
        ss << "  groundspeed: " << groundspeed << std::endl;
        ss << "  distance: " << distance << std::endl;

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
        map << distance;                      // offset: 20
        map << gpid;                          // offset: 24
        map << mav_id;                        // offset: 25
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> flightlevel;                   // offset: 0
        map >> lat;                           // offset: 4
        map >> lon;                           // offset: 8
        map >> alt;                           // offset: 12
        map >> groundspeed;                   // offset: 16
        map >> distance;                      // offset: 20
        map >> gpid;                          // offset: 24
        map >> mav_id;                        // offset: 25
    }
};

} // namespace msg
} // namespace swarmlink
} // namespace mavlink
