#pragma once
// MESSAGE GLOBAL_POI PACKING

#define MAVLINK_MSG_ID_GLOBAL_POI 273

MAVPACKED(
typedef struct __mavlink_global_poi_t {
 int32_t lat; /*< [degE7] Latitude, expressed*/
 int32_t lon; /*< [degE7] Longitude, expressed*/
 int32_t alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
 uint8_t gpid; /*<  Group ID*/
 uint8_t poi_n; /*<  Point num*/
}) mavlink_global_poi_t;

#define MAVLINK_MSG_ID_GLOBAL_POI_LEN 14
#define MAVLINK_MSG_ID_GLOBAL_POI_MIN_LEN 14
#define MAVLINK_MSG_ID_273_LEN 14
#define MAVLINK_MSG_ID_273_MIN_LEN 14

#define MAVLINK_MSG_ID_GLOBAL_POI_CRC 57
#define MAVLINK_MSG_ID_273_CRC 57



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GLOBAL_POI { \
    273, \
    "GLOBAL_POI", \
    5, \
    {  { "gpid", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_global_poi_t, gpid) }, \
         { "poi_n", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_global_poi_t, poi_n) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_global_poi_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_global_poi_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_global_poi_t, alt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GLOBAL_POI { \
    "GLOBAL_POI", \
    5, \
    {  { "gpid", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_global_poi_t, gpid) }, \
         { "poi_n", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_global_poi_t, poi_n) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_global_poi_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_global_poi_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_global_poi_t, alt) }, \
         } \
}
#endif

/**
 * @brief Pack a global_poi message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gpid  Group ID
 * @param poi_n  Point num
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_poi_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t gpid, uint8_t poi_n, int32_t lat, int32_t lon, int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POI_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_uint8_t(buf, 12, gpid);
    _mav_put_uint8_t(buf, 13, poi_n);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POI_LEN);
#else
    mavlink_global_poi_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.gpid = gpid;
    packet.poi_n = poi_n;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POI_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POI;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POI_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POI_LEN, MAVLINK_MSG_ID_GLOBAL_POI_CRC);
}

/**
 * @brief Pack a global_poi message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gpid  Group ID
 * @param poi_n  Point num
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_poi_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t gpid,uint8_t poi_n,int32_t lat,int32_t lon,int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POI_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_uint8_t(buf, 12, gpid);
    _mav_put_uint8_t(buf, 13, poi_n);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POI_LEN);
#else
    mavlink_global_poi_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.gpid = gpid;
    packet.poi_n = poi_n;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POI_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POI;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POI_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POI_LEN, MAVLINK_MSG_ID_GLOBAL_POI_CRC);
}

/**
 * @brief Encode a global_poi struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_poi C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_poi_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_poi_t* global_poi)
{
    return mavlink_msg_global_poi_pack(system_id, component_id, msg, global_poi->gpid, global_poi->poi_n, global_poi->lat, global_poi->lon, global_poi->alt);
}

/**
 * @brief Encode a global_poi struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param global_poi C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_poi_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_global_poi_t* global_poi)
{
    return mavlink_msg_global_poi_pack_chan(system_id, component_id, chan, msg, global_poi->gpid, global_poi->poi_n, global_poi->lat, global_poi->lon, global_poi->alt);
}

/**
 * @brief Send a global_poi message
 * @param chan MAVLink channel to send the message
 *
 * @param gpid  Group ID
 * @param poi_n  Point num
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_poi_send(mavlink_channel_t chan, uint8_t gpid, uint8_t poi_n, int32_t lat, int32_t lon, int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POI_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_uint8_t(buf, 12, gpid);
    _mav_put_uint8_t(buf, 13, poi_n);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POI, buf, MAVLINK_MSG_ID_GLOBAL_POI_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POI_LEN, MAVLINK_MSG_ID_GLOBAL_POI_CRC);
#else
    mavlink_global_poi_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.gpid = gpid;
    packet.poi_n = poi_n;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POI, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POI_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POI_LEN, MAVLINK_MSG_ID_GLOBAL_POI_CRC);
#endif
}

/**
 * @brief Send a global_poi message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_global_poi_send_struct(mavlink_channel_t chan, const mavlink_global_poi_t* global_poi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_global_poi_send(chan, global_poi->gpid, global_poi->poi_n, global_poi->lat, global_poi->lon, global_poi->alt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POI, (const char *)global_poi, MAVLINK_MSG_ID_GLOBAL_POI_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POI_LEN, MAVLINK_MSG_ID_GLOBAL_POI_CRC);
#endif
}

#if MAVLINK_MSG_ID_GLOBAL_POI_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_global_poi_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t gpid, uint8_t poi_n, int32_t lat, int32_t lon, int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_uint8_t(buf, 12, gpid);
    _mav_put_uint8_t(buf, 13, poi_n);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POI, buf, MAVLINK_MSG_ID_GLOBAL_POI_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POI_LEN, MAVLINK_MSG_ID_GLOBAL_POI_CRC);
#else
    mavlink_global_poi_t *packet = (mavlink_global_poi_t *)msgbuf;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->gpid = gpid;
    packet->poi_n = poi_n;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POI, (const char *)packet, MAVLINK_MSG_ID_GLOBAL_POI_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POI_LEN, MAVLINK_MSG_ID_GLOBAL_POI_CRC);
#endif
}
#endif

#endif

// MESSAGE GLOBAL_POI UNPACKING


/**
 * @brief Get field gpid from global_poi message
 *
 * @return  Group ID
 */
static inline uint8_t mavlink_msg_global_poi_get_gpid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field poi_n from global_poi message
 *
 * @return  Point num
 */
static inline uint8_t mavlink_msg_global_poi_get_poi_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field lat from global_poi message
 *
 * @return [degE7] Latitude, expressed
 */
static inline int32_t mavlink_msg_global_poi_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lon from global_poi message
 *
 * @return [degE7] Longitude, expressed
 */
static inline int32_t mavlink_msg_global_poi_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt from global_poi message
 *
 * @return [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 */
static inline int32_t mavlink_msg_global_poi_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a global_poi message into a struct
 *
 * @param msg The message to decode
 * @param global_poi C-struct to decode the message contents into
 */
static inline void mavlink_msg_global_poi_decode(const mavlink_message_t* msg, mavlink_global_poi_t* global_poi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    global_poi->lat = mavlink_msg_global_poi_get_lat(msg);
    global_poi->lon = mavlink_msg_global_poi_get_lon(msg);
    global_poi->alt = mavlink_msg_global_poi_get_alt(msg);
    global_poi->gpid = mavlink_msg_global_poi_get_gpid(msg);
    global_poi->poi_n = mavlink_msg_global_poi_get_poi_n(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GLOBAL_POI_LEN? msg->len : MAVLINK_MSG_ID_GLOBAL_POI_LEN;
        memset(global_poi, 0, MAVLINK_MSG_ID_GLOBAL_POI_LEN);
    memcpy(global_poi, _MAV_PAYLOAD(msg), len);
#endif
}
