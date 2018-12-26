#pragma once
// MESSAGE BUDDY_GROUNDSPEED PACKING

#define MAVLINK_MSG_ID_BUDDY_GROUNDSPEED 274

MAVPACKED(
typedef struct __mavlink_buddy_groundspeed_t {
 float groundspeed; /*< [m/s] Current ground speed*/
 float distance; /*< [m/s] Distance to buddy*/
}) mavlink_buddy_groundspeed_t;

#define MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN 8
#define MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_MIN_LEN 8
#define MAVLINK_MSG_ID_274_LEN 8
#define MAVLINK_MSG_ID_274_MIN_LEN 8

#define MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_CRC 175
#define MAVLINK_MSG_ID_274_CRC 175



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BUDDY_GROUNDSPEED { \
    274, \
    "BUDDY_GROUNDSPEED", \
    2, \
    {  { "groundspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_buddy_groundspeed_t, groundspeed) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_buddy_groundspeed_t, distance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BUDDY_GROUNDSPEED { \
    "BUDDY_GROUNDSPEED", \
    2, \
    {  { "groundspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_buddy_groundspeed_t, groundspeed) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_buddy_groundspeed_t, distance) }, \
         } \
}
#endif

/**
 * @brief Pack a buddy_groundspeed message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param groundspeed [m/s] Current ground speed
 * @param distance [m/s] Distance to buddy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_buddy_groundspeed_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float groundspeed, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN];
    _mav_put_float(buf, 0, groundspeed);
    _mav_put_float(buf, 4, distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN);
#else
    mavlink_buddy_groundspeed_t packet;
    packet.groundspeed = groundspeed;
    packet.distance = distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BUDDY_GROUNDSPEED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_MIN_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_CRC);
}

/**
 * @brief Pack a buddy_groundspeed message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param groundspeed [m/s] Current ground speed
 * @param distance [m/s] Distance to buddy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_buddy_groundspeed_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float groundspeed,float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN];
    _mav_put_float(buf, 0, groundspeed);
    _mav_put_float(buf, 4, distance);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN);
#else
    mavlink_buddy_groundspeed_t packet;
    packet.groundspeed = groundspeed;
    packet.distance = distance;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BUDDY_GROUNDSPEED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_MIN_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_CRC);
}

/**
 * @brief Encode a buddy_groundspeed struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param buddy_groundspeed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_buddy_groundspeed_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_buddy_groundspeed_t* buddy_groundspeed)
{
    return mavlink_msg_buddy_groundspeed_pack(system_id, component_id, msg, buddy_groundspeed->groundspeed, buddy_groundspeed->distance);
}

/**
 * @brief Encode a buddy_groundspeed struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param buddy_groundspeed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_buddy_groundspeed_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_buddy_groundspeed_t* buddy_groundspeed)
{
    return mavlink_msg_buddy_groundspeed_pack_chan(system_id, component_id, chan, msg, buddy_groundspeed->groundspeed, buddy_groundspeed->distance);
}

/**
 * @brief Send a buddy_groundspeed message
 * @param chan MAVLink channel to send the message
 *
 * @param groundspeed [m/s] Current ground speed
 * @param distance [m/s] Distance to buddy
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_buddy_groundspeed_send(mavlink_channel_t chan, float groundspeed, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN];
    _mav_put_float(buf, 0, groundspeed);
    _mav_put_float(buf, 4, distance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED, buf, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_MIN_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_CRC);
#else
    mavlink_buddy_groundspeed_t packet;
    packet.groundspeed = groundspeed;
    packet.distance = distance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED, (const char *)&packet, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_MIN_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_CRC);
#endif
}

/**
 * @brief Send a buddy_groundspeed message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_buddy_groundspeed_send_struct(mavlink_channel_t chan, const mavlink_buddy_groundspeed_t* buddy_groundspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_buddy_groundspeed_send(chan, buddy_groundspeed->groundspeed, buddy_groundspeed->distance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED, (const char *)buddy_groundspeed, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_MIN_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_CRC);
#endif
}

#if MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_buddy_groundspeed_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float groundspeed, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, groundspeed);
    _mav_put_float(buf, 4, distance);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED, buf, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_MIN_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_CRC);
#else
    mavlink_buddy_groundspeed_t *packet = (mavlink_buddy_groundspeed_t *)msgbuf;
    packet->groundspeed = groundspeed;
    packet->distance = distance;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED, (const char *)packet, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_MIN_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_CRC);
#endif
}
#endif

#endif

// MESSAGE BUDDY_GROUNDSPEED UNPACKING


/**
 * @brief Get field groundspeed from buddy_groundspeed message
 *
 * @return [m/s] Current ground speed
 */
static inline float mavlink_msg_buddy_groundspeed_get_groundspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field distance from buddy_groundspeed message
 *
 * @return [m/s] Distance to buddy
 */
static inline float mavlink_msg_buddy_groundspeed_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a buddy_groundspeed message into a struct
 *
 * @param msg The message to decode
 * @param buddy_groundspeed C-struct to decode the message contents into
 */
static inline void mavlink_msg_buddy_groundspeed_decode(const mavlink_message_t* msg, mavlink_buddy_groundspeed_t* buddy_groundspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    buddy_groundspeed->groundspeed = mavlink_msg_buddy_groundspeed_get_groundspeed(msg);
    buddy_groundspeed->distance = mavlink_msg_buddy_groundspeed_get_distance(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN? msg->len : MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN;
        memset(buddy_groundspeed, 0, MAVLINK_MSG_ID_BUDDY_GROUNDSPEED_LEN);
    memcpy(buddy_groundspeed, _MAV_PAYLOAD(msg), len);
#endif
}
