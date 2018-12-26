#pragma once
// MESSAGE BUDDY_POSITION PACKING

#define MAVLINK_MSG_ID_BUDDY_POSITION 273

MAVPACKED(
typedef struct __mavlink_buddy_position_t {
 int32_t lat; /*< [degE7] Latitude, expressed*/
 int32_t lon; /*< [degE7] Longitude, expressed*/
 int32_t alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
 float distance; /*< [m/s] Distance to buddy*/
}) mavlink_buddy_position_t;

#define MAVLINK_MSG_ID_BUDDY_POSITION_LEN 16
#define MAVLINK_MSG_ID_BUDDY_POSITION_MIN_LEN 16
#define MAVLINK_MSG_ID_273_LEN 16
#define MAVLINK_MSG_ID_273_MIN_LEN 16

#define MAVLINK_MSG_ID_BUDDY_POSITION_CRC 17
#define MAVLINK_MSG_ID_273_CRC 17



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BUDDY_POSITION { \
    273, \
    "BUDDY_POSITION", \
    4, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_buddy_position_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_buddy_position_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_buddy_position_t, alt) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_buddy_position_t, distance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BUDDY_POSITION { \
    "BUDDY_POSITION", \
    4, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_buddy_position_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_buddy_position_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_buddy_position_t, alt) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_buddy_position_t, distance) }, \
         } \
}
#endif

/**
 * @brief Pack a buddy_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param distance [m/s] Distance to buddy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_buddy_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lat, int32_t lon, int32_t alt, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BUDDY_POSITION_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_float(buf, 12, distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BUDDY_POSITION_LEN);
#else
    mavlink_buddy_position_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.distance = distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BUDDY_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BUDDY_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BUDDY_POSITION_MIN_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_CRC);
}

/**
 * @brief Pack a buddy_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param distance [m/s] Distance to buddy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_buddy_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lat,int32_t lon,int32_t alt,float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BUDDY_POSITION_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_float(buf, 12, distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BUDDY_POSITION_LEN);
#else
    mavlink_buddy_position_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.distance = distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BUDDY_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BUDDY_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BUDDY_POSITION_MIN_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_CRC);
}

/**
 * @brief Encode a buddy_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param buddy_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_buddy_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_buddy_position_t* buddy_position)
{
    return mavlink_msg_buddy_position_pack(system_id, component_id, msg, buddy_position->lat, buddy_position->lon, buddy_position->alt, buddy_position->distance);
}

/**
 * @brief Encode a buddy_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param buddy_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_buddy_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_buddy_position_t* buddy_position)
{
    return mavlink_msg_buddy_position_pack_chan(system_id, component_id, chan, msg, buddy_position->lat, buddy_position->lon, buddy_position->alt, buddy_position->distance);
}

/**
 * @brief Send a buddy_position message
 * @param chan MAVLink channel to send the message
 *
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param distance [m/s] Distance to buddy
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_buddy_position_send(mavlink_channel_t chan, int32_t lat, int32_t lon, int32_t alt, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BUDDY_POSITION_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_float(buf, 12, distance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_POSITION, buf, MAVLINK_MSG_ID_BUDDY_POSITION_MIN_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_CRC);
#else
    mavlink_buddy_position_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.distance = distance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_POSITION, (const char *)&packet, MAVLINK_MSG_ID_BUDDY_POSITION_MIN_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_CRC);
#endif
}

/**
 * @brief Send a buddy_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_buddy_position_send_struct(mavlink_channel_t chan, const mavlink_buddy_position_t* buddy_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_buddy_position_send(chan, buddy_position->lat, buddy_position->lon, buddy_position->alt, buddy_position->distance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_POSITION, (const char *)buddy_position, MAVLINK_MSG_ID_BUDDY_POSITION_MIN_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_BUDDY_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_buddy_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lat, int32_t lon, int32_t alt, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_float(buf, 12, distance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_POSITION, buf, MAVLINK_MSG_ID_BUDDY_POSITION_MIN_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_CRC);
#else
    mavlink_buddy_position_t *packet = (mavlink_buddy_position_t *)msgbuf;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->distance = distance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_POSITION, (const char *)packet, MAVLINK_MSG_ID_BUDDY_POSITION_MIN_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_LEN, MAVLINK_MSG_ID_BUDDY_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE BUDDY_POSITION UNPACKING


/**
 * @brief Get field lat from buddy_position message
 *
 * @return [degE7] Latitude, expressed
 */
static inline int32_t mavlink_msg_buddy_position_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lon from buddy_position message
 *
 * @return [degE7] Longitude, expressed
 */
static inline int32_t mavlink_msg_buddy_position_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt from buddy_position message
 *
 * @return [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 */
static inline int32_t mavlink_msg_buddy_position_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field distance from buddy_position message
 *
 * @return [m/s] Distance to buddy
 */
static inline float mavlink_msg_buddy_position_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a buddy_position message into a struct
 *
 * @param msg The message to decode
 * @param buddy_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_buddy_position_decode(const mavlink_message_t* msg, mavlink_buddy_position_t* buddy_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    buddy_position->lat = mavlink_msg_buddy_position_get_lat(msg);
    buddy_position->lon = mavlink_msg_buddy_position_get_lon(msg);
    buddy_position->alt = mavlink_msg_buddy_position_get_alt(msg);
    buddy_position->distance = mavlink_msg_buddy_position_get_distance(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BUDDY_POSITION_LEN? msg->len : MAVLINK_MSG_ID_BUDDY_POSITION_LEN;
        memset(buddy_position, 0, MAVLINK_MSG_ID_BUDDY_POSITION_LEN);
    memcpy(buddy_position, _MAV_PAYLOAD(msg), len);
#endif
}
