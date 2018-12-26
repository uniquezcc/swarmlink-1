#pragma once
// MESSAGE BUDDY_INFO PACKING

#define MAVLINK_MSG_ID_BUDDY_INFO 272

MAVPACKED(
typedef struct __mavlink_buddy_info_t {
 uint32_t flightlevel; /*< [m] Flight level*/
 int32_t lat; /*< [degE7] Latitude, expressed*/
 int32_t lon; /*< [degE7] Longitude, expressed*/
 int32_t alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
 float groundspeed; /*< [m/s] Current ground speed*/
 float distance; /*< [m/s] Distance to buddy*/
 uint8_t gpid; /*<  Group ID*/
 uint8_t mav_id; /*<  MAV ID*/
}) mavlink_buddy_info_t;

#define MAVLINK_MSG_ID_BUDDY_INFO_LEN 26
#define MAVLINK_MSG_ID_BUDDY_INFO_MIN_LEN 26
#define MAVLINK_MSG_ID_272_LEN 26
#define MAVLINK_MSG_ID_272_MIN_LEN 26

#define MAVLINK_MSG_ID_BUDDY_INFO_CRC 151
#define MAVLINK_MSG_ID_272_CRC 151



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BUDDY_INFO { \
    272, \
    "BUDDY_INFO", \
    8, \
    {  { "gpid", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_buddy_info_t, gpid) }, \
         { "mav_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_buddy_info_t, mav_id) }, \
         { "flightlevel", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_buddy_info_t, flightlevel) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_buddy_info_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_buddy_info_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_buddy_info_t, alt) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_buddy_info_t, groundspeed) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_buddy_info_t, distance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BUDDY_INFO { \
    "BUDDY_INFO", \
    8, \
    {  { "gpid", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_buddy_info_t, gpid) }, \
         { "mav_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_buddy_info_t, mav_id) }, \
         { "flightlevel", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_buddy_info_t, flightlevel) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_buddy_info_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_buddy_info_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_buddy_info_t, alt) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_buddy_info_t, groundspeed) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_buddy_info_t, distance) }, \
         } \
}
#endif

/**
 * @brief Pack a buddy_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gpid  Group ID
 * @param mav_id  MAV ID
 * @param flightlevel [m] Flight level
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param groundspeed [m/s] Current ground speed
 * @param distance [m/s] Distance to buddy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_buddy_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t gpid, uint8_t mav_id, uint32_t flightlevel, int32_t lat, int32_t lon, int32_t alt, float groundspeed, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BUDDY_INFO_LEN];
    _mav_put_uint32_t(buf, 0, flightlevel);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_float(buf, 16, groundspeed);
    _mav_put_float(buf, 20, distance);
    _mav_put_uint8_t(buf, 24, gpid);
    _mav_put_uint8_t(buf, 25, mav_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BUDDY_INFO_LEN);
#else
    mavlink_buddy_info_t packet;
    packet.flightlevel = flightlevel;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.groundspeed = groundspeed;
    packet.distance = distance;
    packet.gpid = gpid;
    packet.mav_id = mav_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BUDDY_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BUDDY_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BUDDY_INFO_MIN_LEN, MAVLINK_MSG_ID_BUDDY_INFO_LEN, MAVLINK_MSG_ID_BUDDY_INFO_CRC);
}

/**
 * @brief Pack a buddy_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gpid  Group ID
 * @param mav_id  MAV ID
 * @param flightlevel [m] Flight level
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param groundspeed [m/s] Current ground speed
 * @param distance [m/s] Distance to buddy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_buddy_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t gpid,uint8_t mav_id,uint32_t flightlevel,int32_t lat,int32_t lon,int32_t alt,float groundspeed,float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BUDDY_INFO_LEN];
    _mav_put_uint32_t(buf, 0, flightlevel);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_float(buf, 16, groundspeed);
    _mav_put_float(buf, 20, distance);
    _mav_put_uint8_t(buf, 24, gpid);
    _mav_put_uint8_t(buf, 25, mav_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BUDDY_INFO_LEN);
#else
    mavlink_buddy_info_t packet;
    packet.flightlevel = flightlevel;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.groundspeed = groundspeed;
    packet.distance = distance;
    packet.gpid = gpid;
    packet.mav_id = mav_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BUDDY_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BUDDY_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BUDDY_INFO_MIN_LEN, MAVLINK_MSG_ID_BUDDY_INFO_LEN, MAVLINK_MSG_ID_BUDDY_INFO_CRC);
}

/**
 * @brief Encode a buddy_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param buddy_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_buddy_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_buddy_info_t* buddy_info)
{
    return mavlink_msg_buddy_info_pack(system_id, component_id, msg, buddy_info->gpid, buddy_info->mav_id, buddy_info->flightlevel, buddy_info->lat, buddy_info->lon, buddy_info->alt, buddy_info->groundspeed, buddy_info->distance);
}

/**
 * @brief Encode a buddy_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param buddy_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_buddy_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_buddy_info_t* buddy_info)
{
    return mavlink_msg_buddy_info_pack_chan(system_id, component_id, chan, msg, buddy_info->gpid, buddy_info->mav_id, buddy_info->flightlevel, buddy_info->lat, buddy_info->lon, buddy_info->alt, buddy_info->groundspeed, buddy_info->distance);
}

/**
 * @brief Send a buddy_info message
 * @param chan MAVLink channel to send the message
 *
 * @param gpid  Group ID
 * @param mav_id  MAV ID
 * @param flightlevel [m] Flight level
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param groundspeed [m/s] Current ground speed
 * @param distance [m/s] Distance to buddy
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_buddy_info_send(mavlink_channel_t chan, uint8_t gpid, uint8_t mav_id, uint32_t flightlevel, int32_t lat, int32_t lon, int32_t alt, float groundspeed, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BUDDY_INFO_LEN];
    _mav_put_uint32_t(buf, 0, flightlevel);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_float(buf, 16, groundspeed);
    _mav_put_float(buf, 20, distance);
    _mav_put_uint8_t(buf, 24, gpid);
    _mav_put_uint8_t(buf, 25, mav_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_INFO, buf, MAVLINK_MSG_ID_BUDDY_INFO_MIN_LEN, MAVLINK_MSG_ID_BUDDY_INFO_LEN, MAVLINK_MSG_ID_BUDDY_INFO_CRC);
#else
    mavlink_buddy_info_t packet;
    packet.flightlevel = flightlevel;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.groundspeed = groundspeed;
    packet.distance = distance;
    packet.gpid = gpid;
    packet.mav_id = mav_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_INFO, (const char *)&packet, MAVLINK_MSG_ID_BUDDY_INFO_MIN_LEN, MAVLINK_MSG_ID_BUDDY_INFO_LEN, MAVLINK_MSG_ID_BUDDY_INFO_CRC);
#endif
}

/**
 * @brief Send a buddy_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_buddy_info_send_struct(mavlink_channel_t chan, const mavlink_buddy_info_t* buddy_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_buddy_info_send(chan, buddy_info->gpid, buddy_info->mav_id, buddy_info->flightlevel, buddy_info->lat, buddy_info->lon, buddy_info->alt, buddy_info->groundspeed, buddy_info->distance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_INFO, (const char *)buddy_info, MAVLINK_MSG_ID_BUDDY_INFO_MIN_LEN, MAVLINK_MSG_ID_BUDDY_INFO_LEN, MAVLINK_MSG_ID_BUDDY_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_BUDDY_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_buddy_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t gpid, uint8_t mav_id, uint32_t flightlevel, int32_t lat, int32_t lon, int32_t alt, float groundspeed, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, flightlevel);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_float(buf, 16, groundspeed);
    _mav_put_float(buf, 20, distance);
    _mav_put_uint8_t(buf, 24, gpid);
    _mav_put_uint8_t(buf, 25, mav_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_INFO, buf, MAVLINK_MSG_ID_BUDDY_INFO_MIN_LEN, MAVLINK_MSG_ID_BUDDY_INFO_LEN, MAVLINK_MSG_ID_BUDDY_INFO_CRC);
#else
    mavlink_buddy_info_t *packet = (mavlink_buddy_info_t *)msgbuf;
    packet->flightlevel = flightlevel;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->groundspeed = groundspeed;
    packet->distance = distance;
    packet->gpid = gpid;
    packet->mav_id = mav_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_INFO, (const char *)packet, MAVLINK_MSG_ID_BUDDY_INFO_MIN_LEN, MAVLINK_MSG_ID_BUDDY_INFO_LEN, MAVLINK_MSG_ID_BUDDY_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE BUDDY_INFO UNPACKING


/**
 * @brief Get field gpid from buddy_info message
 *
 * @return  Group ID
 */
static inline uint8_t mavlink_msg_buddy_info_get_gpid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field mav_id from buddy_info message
 *
 * @return  MAV ID
 */
static inline uint8_t mavlink_msg_buddy_info_get_mav_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field flightlevel from buddy_info message
 *
 * @return [m] Flight level
 */
static inline uint32_t mavlink_msg_buddy_info_get_flightlevel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat from buddy_info message
 *
 * @return [degE7] Latitude, expressed
 */
static inline int32_t mavlink_msg_buddy_info_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from buddy_info message
 *
 * @return [degE7] Longitude, expressed
 */
static inline int32_t mavlink_msg_buddy_info_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from buddy_info message
 *
 * @return [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 */
static inline int32_t mavlink_msg_buddy_info_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field groundspeed from buddy_info message
 *
 * @return [m/s] Current ground speed
 */
static inline float mavlink_msg_buddy_info_get_groundspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field distance from buddy_info message
 *
 * @return [m/s] Distance to buddy
 */
static inline float mavlink_msg_buddy_info_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a buddy_info message into a struct
 *
 * @param msg The message to decode
 * @param buddy_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_buddy_info_decode(const mavlink_message_t* msg, mavlink_buddy_info_t* buddy_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    buddy_info->flightlevel = mavlink_msg_buddy_info_get_flightlevel(msg);
    buddy_info->lat = mavlink_msg_buddy_info_get_lat(msg);
    buddy_info->lon = mavlink_msg_buddy_info_get_lon(msg);
    buddy_info->alt = mavlink_msg_buddy_info_get_alt(msg);
    buddy_info->groundspeed = mavlink_msg_buddy_info_get_groundspeed(msg);
    buddy_info->distance = mavlink_msg_buddy_info_get_distance(msg);
    buddy_info->gpid = mavlink_msg_buddy_info_get_gpid(msg);
    buddy_info->mav_id = mavlink_msg_buddy_info_get_mav_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BUDDY_INFO_LEN? msg->len : MAVLINK_MSG_ID_BUDDY_INFO_LEN;
        memset(buddy_info, 0, MAVLINK_MSG_ID_BUDDY_INFO_LEN);
    memcpy(buddy_info, _MAV_PAYLOAD(msg), len);
#endif
}
