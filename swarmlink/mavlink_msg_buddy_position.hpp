// MESSAGE BUDDY_POSITION support class

#pragma once

namespace mavlink {
namespace swarmlink {
namespace msg {

/**
 * @brief BUDDY_POSITION message
 *
 * This message contains information about boid's position.
 */
struct BUDDY_POSITION : mavlink::Message {
    static constexpr msgid_t MSG_ID = 273;
    static constexpr size_t LENGTH = 16;
    static constexpr size_t MIN_LENGTH = 16;
    static constexpr uint8_t CRC_EXTRA = 17;
    static constexpr auto NAME = "BUDDY_POSITION";


    int32_t lat; /*< [degE7] Latitude, expressed */
    int32_t lon; /*< [degE7] Longitude, expressed */
    int32_t alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL. */
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
        ss << "  lat: " << lat << std::endl;
        ss << "  lon: " << lon << std::endl;
        ss << "  alt: " << alt << std::endl;
        ss << "  distance: " << distance << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << lat;                           // offset: 0
        map << lon;                           // offset: 4
        map << alt;                           // offset: 8
        map << distance;                      // offset: 12
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> lat;                           // offset: 0
        map >> lon;                           // offset: 4
        map >> alt;                           // offset: 8
        map >> distance;                      // offset: 12
    }
};

} // namespace msg
} // namespace swarmlink
} // namespace mavlink
