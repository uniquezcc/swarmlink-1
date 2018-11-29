#pragma once
// MESSAGE SWARM_BEAT PACKING

#define MAVLINK_MSG_ID_SWARM_BEAT 271

MAVPACKED(
typedef struct __mavlink_swarm_beat_t {
 uint32_t flightlevel; /*< [m] Flight level*/
 int32_t lat; /*< [degE7] Latitude, expressed*/
 int32_t lon; /*< [degE7] Longitude, expressed*/
 int32_t alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
 float groundspeed; /*< [m/s] Current ground speed*/
 uint8_t gpid; /*<  Group ID*/
}) mavlink_swarm_beat_t;

#define MAVLINK_MSG_ID_SWARM_BEAT_LEN 21
#define MAVLINK_MSG_ID_SWARM_BEAT_MIN_LEN 21
#define MAVLINK_MSG_ID_271_LEN 21
#define MAVLINK_MSG_ID_271_MIN_LEN 21

#define MAVLINK_MSG_ID_SWARM_BEAT_CRC 163
#define MAVLINK_MSG_ID_271_CRC 163



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SWARM_BEAT { \
    271, \
    "SWARM_BEAT", \
    6, \
    {  { "gpid", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_swarm_beat_t, gpid) }, \
         { "flightlevel", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_swarm_beat_t, flightlevel) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_swarm_beat_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_swarm_beat_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_swarm_beat_t, alt) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_swarm_beat_t, groundspeed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SWARM_BEAT { \
    "SWARM_BEAT", \
    6, \
    {  { "gpid", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_swarm_beat_t, gpid) }, \
         { "flightlevel", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_swarm_beat_t, flightlevel) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_swarm_beat_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_swarm_beat_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_swarm_beat_t, alt) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_swarm_beat_t, groundspeed) }, \
         } \
}
#endif

/**
 * @brief Pack a swarm_beat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gpid  Group ID
 * @param flightlevel [m] Flight level
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param groundspeed [m/s] Current ground speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_swarm_beat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t gpid, uint32_t flightlevel, int32_t lat, int32_t lon, int32_t alt, float groundspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_BEAT_LEN];
    _mav_put_uint32_t(buf, 0, flightlevel);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_float(buf, 16, groundspeed);
    _mav_put_uint8_t(buf, 20, gpid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SWARM_BEAT_LEN);
#else
    mavlink_swarm_beat_t packet;
    packet.flightlevel = flightlevel;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.groundspeed = groundspeed;
    packet.gpid = gpid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SWARM_BEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SWARM_BEAT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SWARM_BEAT_MIN_LEN, MAVLINK_MSG_ID_SWARM_BEAT_LEN, MAVLINK_MSG_ID_SWARM_BEAT_CRC);
}

/**
 * @brief Pack a swarm_beat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gpid  Group ID
 * @param flightlevel [m] Flight level
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param groundspeed [m/s] Current ground speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_swarm_beat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t gpid,uint32_t flightlevel,int32_t lat,int32_t lon,int32_t alt,float groundspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_BEAT_LEN];
    _mav_put_uint32_t(buf, 0, flightlevel);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_float(buf, 16, groundspeed);
    _mav_put_uint8_t(buf, 20, gpid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SWARM_BEAT_LEN);
#else
    mavlink_swarm_beat_t packet;
    packet.flightlevel = flightlevel;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.groundspeed = groundspeed;
    packet.gpid = gpid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SWARM_BEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SWARM_BEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SWARM_BEAT_MIN_LEN, MAVLINK_MSG_ID_SWARM_BEAT_LEN, MAVLINK_MSG_ID_SWARM_BEAT_CRC);
}

/**
 * @brief Encode a swarm_beat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param swarm_beat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_swarm_beat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_swarm_beat_t* swarm_beat)
{
    return mavlink_msg_swarm_beat_pack(system_id, component_id, msg, swarm_beat->gpid, swarm_beat->flightlevel, swarm_beat->lat, swarm_beat->lon, swarm_beat->alt, swarm_beat->groundspeed);
}

/**
 * @brief Encode a swarm_beat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param swarm_beat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_swarm_beat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_swarm_beat_t* swarm_beat)
{
    return mavlink_msg_swarm_beat_pack_chan(system_id, component_id, chan, msg, swarm_beat->gpid, swarm_beat->flightlevel, swarm_beat->lat, swarm_beat->lon, swarm_beat->alt, swarm_beat->groundspeed);
}

/**
 * @brief Send a swarm_beat message
 * @param chan MAVLink channel to send the message
 *
 * @param gpid  Group ID
 * @param flightlevel [m] Flight level
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param groundspeed [m/s] Current ground speed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_swarm_beat_send(mavlink_channel_t chan, uint8_t gpid, uint32_t flightlevel, int32_t lat, int32_t lon, int32_t alt, float groundspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SWARM_BEAT_LEN];
    _mav_put_uint32_t(buf, 0, flightlevel);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_float(buf, 16, groundspeed);
    _mav_put_uint8_t(buf, 20, gpid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_BEAT, buf, MAVLINK_MSG_ID_SWARM_BEAT_MIN_LEN, MAVLINK_MSG_ID_SWARM_BEAT_LEN, MAVLINK_MSG_ID_SWARM_BEAT_CRC);
#else
    mavlink_swarm_beat_t packet;
    packet.flightlevel = flightlevel;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.groundspeed = groundspeed;
    packet.gpid = gpid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_BEAT, (const char *)&packet, MAVLINK_MSG_ID_SWARM_BEAT_MIN_LEN, MAVLINK_MSG_ID_SWARM_BEAT_LEN, MAVLINK_MSG_ID_SWARM_BEAT_CRC);
#endif
}

/**
 * @brief Send a swarm_beat message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_swarm_beat_send_struct(mavlink_channel_t chan, const mavlink_swarm_beat_t* swarm_beat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_swarm_beat_send(chan, swarm_beat->gpid, swarm_beat->flightlevel, swarm_beat->lat, swarm_beat->lon, swarm_beat->alt, swarm_beat->groundspeed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_BEAT, (const char *)swarm_beat, MAVLINK_MSG_ID_SWARM_BEAT_MIN_LEN, MAVLINK_MSG_ID_SWARM_BEAT_LEN, MAVLINK_MSG_ID_SWARM_BEAT_CRC);
#endif
}

#if MAVLINK_MSG_ID_SWARM_BEAT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_swarm_beat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t gpid, uint32_t flightlevel, int32_t lat, int32_t lon, int32_t alt, float groundspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, flightlevel);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_float(buf, 16, groundspeed);
    _mav_put_uint8_t(buf, 20, gpid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_BEAT, buf, MAVLINK_MSG_ID_SWARM_BEAT_MIN_LEN, MAVLINK_MSG_ID_SWARM_BEAT_LEN, MAVLINK_MSG_ID_SWARM_BEAT_CRC);
#else
    mavlink_swarm_beat_t *packet = (mavlink_swarm_beat_t *)msgbuf;
    packet->flightlevel = flightlevel;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->groundspeed = groundspeed;
    packet->gpid = gpid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SWARM_BEAT, (const char *)packet, MAVLINK_MSG_ID_SWARM_BEAT_MIN_LEN, MAVLINK_MSG_ID_SWARM_BEAT_LEN, MAVLINK_MSG_ID_SWARM_BEAT_CRC);
#endif
}
#endif

#endif

// MESSAGE SWARM_BEAT UNPACKING


/**
 * @brief Get field gpid from swarm_beat message
 *
 * @return  Group ID
 */
static inline uint8_t mavlink_msg_swarm_beat_get_gpid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field flightlevel from swarm_beat message
 *
 * @return [m] Flight level
 */
static inline uint32_t mavlink_msg_swarm_beat_get_flightlevel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat from swarm_beat message
 *
 * @return [degE7] Latitude, expressed
 */
static inline int32_t mavlink_msg_swarm_beat_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from swarm_beat message
 *
 * @return [degE7] Longitude, expressed
 */
static inline int32_t mavlink_msg_swarm_beat_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from swarm_beat message
 *
 * @return [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 */
static inline int32_t mavlink_msg_swarm_beat_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field groundspeed from swarm_beat message
 *
 * @return [m/s] Current ground speed
 */
static inline float mavlink_msg_swarm_beat_get_groundspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a swarm_beat message into a struct
 *
 * @param msg The message to decode
 * @param swarm_beat C-struct to decode the message contents into
 */
static inline void mavlink_msg_swarm_beat_decode(const mavlink_message_t* msg, mavlink_swarm_beat_t* swarm_beat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    swarm_beat->flightlevel = mavlink_msg_swarm_beat_get_flightlevel(msg);
    swarm_beat->lat = mavlink_msg_swarm_beat_get_lat(msg);
    swarm_beat->lon = mavlink_msg_swarm_beat_get_lon(msg);
    swarm_beat->alt = mavlink_msg_swarm_beat_get_alt(msg);
    swarm_beat->groundspeed = mavlink_msg_swarm_beat_get_groundspeed(msg);
    swarm_beat->gpid = mavlink_msg_swarm_beat_get_gpid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SWARM_BEAT_LEN? msg->len : MAVLINK_MSG_ID_SWARM_BEAT_LEN;
        memset(swarm_beat, 0, MAVLINK_MSG_ID_SWARM_BEAT_LEN);
    memcpy(swarm_beat, _MAV_PAYLOAD(msg), len);
#endif
}
