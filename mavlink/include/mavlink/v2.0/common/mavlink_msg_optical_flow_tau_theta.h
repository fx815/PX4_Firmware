#pragma once
// MESSAGE OPTICAL_FLOW_TAU_THETA PACKING

#define MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA 555

MAVPACKED(
typedef struct __mavlink_optical_flow_tau_theta_t {
 float tau; /*< [1/s] V over D, approaching speed divided by distance to wall, equivalent as optical flow divergence*/
 float theta; /*< [degree] approaching angle w.r.t. the wall*/
}) mavlink_optical_flow_tau_theta_t;

#define MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN 8
#define MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_MIN_LEN 8
#define MAVLINK_MSG_ID_555_LEN 8
#define MAVLINK_MSG_ID_555_MIN_LEN 8

#define MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_CRC 39
#define MAVLINK_MSG_ID_555_CRC 39



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OPTICAL_FLOW_TAU_THETA { \
    555, \
    "OPTICAL_FLOW_TAU_THETA", \
    2, \
    {  { "tau", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_optical_flow_tau_theta_t, tau) }, \
         { "theta", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_optical_flow_tau_theta_t, theta) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OPTICAL_FLOW_TAU_THETA { \
    "OPTICAL_FLOW_TAU_THETA", \
    2, \
    {  { "tau", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_optical_flow_tau_theta_t, tau) }, \
         { "theta", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_optical_flow_tau_theta_t, theta) }, \
         } \
}
#endif

/**
 * @brief Pack a optical_flow_tau_theta message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param tau [1/s] V over D, approaching speed divided by distance to wall, equivalent as optical flow divergence
 * @param theta [degree] approaching angle w.r.t. the wall
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_optical_flow_tau_theta_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float tau, float theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN];
    _mav_put_float(buf, 0, tau);
    _mav_put_float(buf, 4, theta);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN);
#else
    mavlink_optical_flow_tau_theta_t packet;
    packet.tau = tau;
    packet.theta = theta;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_CRC);
}

/**
 * @brief Pack a optical_flow_tau_theta message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tau [1/s] V over D, approaching speed divided by distance to wall, equivalent as optical flow divergence
 * @param theta [degree] approaching angle w.r.t. the wall
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_optical_flow_tau_theta_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float tau,float theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN];
    _mav_put_float(buf, 0, tau);
    _mav_put_float(buf, 4, theta);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN);
#else
    mavlink_optical_flow_tau_theta_t packet;
    packet.tau = tau;
    packet.theta = theta;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_CRC);
}

/**
 * @brief Encode a optical_flow_tau_theta struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param optical_flow_tau_theta C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_optical_flow_tau_theta_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_optical_flow_tau_theta_t* optical_flow_tau_theta)
{
    return mavlink_msg_optical_flow_tau_theta_pack(system_id, component_id, msg, optical_flow_tau_theta->tau, optical_flow_tau_theta->theta);
}

/**
 * @brief Encode a optical_flow_tau_theta struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param optical_flow_tau_theta C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_optical_flow_tau_theta_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_optical_flow_tau_theta_t* optical_flow_tau_theta)
{
    return mavlink_msg_optical_flow_tau_theta_pack_chan(system_id, component_id, chan, msg, optical_flow_tau_theta->tau, optical_flow_tau_theta->theta);
}

/**
 * @brief Send a optical_flow_tau_theta message
 * @param chan MAVLink channel to send the message
 *
 * @param tau [1/s] V over D, approaching speed divided by distance to wall, equivalent as optical flow divergence
 * @param theta [degree] approaching angle w.r.t. the wall
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_optical_flow_tau_theta_send(mavlink_channel_t chan, float tau, float theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN];
    _mav_put_float(buf, 0, tau);
    _mav_put_float(buf, 4, theta);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA, buf, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_CRC);
#else
    mavlink_optical_flow_tau_theta_t packet;
    packet.tau = tau;
    packet.theta = theta;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA, (const char *)&packet, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_CRC);
#endif
}

/**
 * @brief Send a optical_flow_tau_theta message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_optical_flow_tau_theta_send_struct(mavlink_channel_t chan, const mavlink_optical_flow_tau_theta_t* optical_flow_tau_theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_optical_flow_tau_theta_send(chan, optical_flow_tau_theta->tau, optical_flow_tau_theta->theta);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA, (const char *)optical_flow_tau_theta, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_CRC);
#endif
}

#if MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_optical_flow_tau_theta_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float tau, float theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, tau);
    _mav_put_float(buf, 4, theta);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA, buf, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_CRC);
#else
    mavlink_optical_flow_tau_theta_t *packet = (mavlink_optical_flow_tau_theta_t *)msgbuf;
    packet->tau = tau;
    packet->theta = theta;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA, (const char *)packet, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_CRC);
#endif
}
#endif

#endif

// MESSAGE OPTICAL_FLOW_TAU_THETA UNPACKING


/**
 * @brief Get field tau from optical_flow_tau_theta message
 *
 * @return [1/s] V over D, approaching speed divided by distance to wall, equivalent as optical flow divergence
 */
static inline float mavlink_msg_optical_flow_tau_theta_get_tau(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field theta from optical_flow_tau_theta message
 *
 * @return [degree] approaching angle w.r.t. the wall
 */
static inline float mavlink_msg_optical_flow_tau_theta_get_theta(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a optical_flow_tau_theta message into a struct
 *
 * @param msg The message to decode
 * @param optical_flow_tau_theta C-struct to decode the message contents into
 */
static inline void mavlink_msg_optical_flow_tau_theta_decode(const mavlink_message_t* msg, mavlink_optical_flow_tau_theta_t* optical_flow_tau_theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    optical_flow_tau_theta->tau = mavlink_msg_optical_flow_tau_theta_get_tau(msg);
    optical_flow_tau_theta->theta = mavlink_msg_optical_flow_tau_theta_get_theta(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN? msg->len : MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN;
        memset(optical_flow_tau_theta, 0, MAVLINK_MSG_ID_OPTICAL_FLOW_TAU_THETA_LEN);
    memcpy(optical_flow_tau_theta, _MAV_PAYLOAD(msg), len);
#endif
}
