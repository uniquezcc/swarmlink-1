// MESSAGE GLOBAL_POI support class

#pragma once

namespace mavlink {
namespace swarmlink {
namespace msg {

/**
 * @brief GLOBAL_POI message
 *
 * This message contains information about boid.
 */
struct GLOBAL_POI : mavlink::Message {
    static constexpr msgid_t MSG_ID = 273;
    static constexpr size_t LENGTH = 14;
    static constexpr size_t MIN_LENGTH = 14;
    static constexpr uint8_t CRC_EXTRA = 57;
    static constexpr auto NAME = "GLOBAL_POI";


    uint8_t gpid; /*<  Group ID */
    uint8_t poi_n; /*<  Point num */
    int32_t lat; /*< [degE7] Latitude, expressed */
    int32_t lon; /*< [degE7] Longitude, expressed */
    int32_t alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL. */


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
        ss << "  poi_n: " << +poi_n << std::endl;
        ss << "  lat: " << lat << std::endl;
        ss << "  lon: " << lon << std::endl;
        ss << "  alt: " << alt << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << lat;                           // offset: 0
        map << lon;                           // offset: 4
        map << alt;                           // offset: 8
        map << gpid;                          // offset: 12
        map << poi_n;                         // offset: 13
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> lat;                           // offset: 0
        map >> lon;                           // offset: 4
        map >> alt;                           // offset: 8
        map >> gpid;                          // offset: 12
        map >> poi_n;                         // offset: 13
    }
};

} // namespace msg
} // namespace swarmlink
} // namespace mavlink
