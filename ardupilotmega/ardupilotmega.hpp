/** @file
 *	@brief MAVLink comm protocol generated from ardupilotmega.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace ardupilotmega {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (trought @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 60> MESSAGE_ENTRIES {{ {150, 134, 42, 0, 0, 0}, {151, 219, 8, 3, 6, 7}, {152, 208, 4, 0, 0, 0}, {153, 188, 12, 0, 0, 0}, {154, 84, 15, 3, 6, 7}, {155, 22, 13, 3, 4, 5}, {156, 19, 6, 3, 0, 1}, {157, 21, 15, 3, 12, 13}, {158, 134, 14, 3, 12, 13}, {160, 78, 12, 3, 8, 9}, {161, 68, 3, 3, 0, 1}, {162, 189, 8, 0, 0, 0}, {163, 127, 28, 0, 0, 0}, {164, 154, 44, 0, 0, 0}, {165, 21, 3, 0, 0, 0}, {166, 21, 9, 0, 0, 0}, {167, 144, 22, 0, 0, 0}, {168, 1, 12, 0, 0, 0}, {169, 234, 18, 0, 0, 0}, {170, 73, 34, 0, 0, 0}, {171, 181, 66, 0, 0, 0}, {172, 22, 98, 0, 0, 0}, {173, 83, 8, 0, 0, 0}, {174, 167, 48, 0, 0, 0}, {175, 138, 19, 3, 14, 15}, {176, 234, 3, 3, 0, 1}, {177, 240, 20, 0, 0, 0}, {178, 47, 24, 0, 0, 0}, {179, 189, 29, 1, 26, 0}, {180, 52, 45, 1, 42, 0}, {181, 174, 4, 0, 0, 0}, {182, 229, 40, 0, 0, 0}, {183, 85, 2, 3, 0, 1}, {184, 159, 206, 3, 4, 5}, {185, 186, 7, 3, 4, 5}, {186, 72, 29, 3, 0, 1}, {191, 92, 27, 0, 0, 0}, {192, 36, 44, 0, 0, 0}, {193, 71, 22, 0, 0, 0}, {194, 98, 25, 0, 0, 0}, {195, 120, 37, 0, 0, 0}, {200, 134, 42, 3, 40, 41}, {201, 205, 14, 3, 12, 13}, {214, 69, 8, 3, 6, 7}, {215, 101, 3, 0, 0, 0}, {216, 50, 3, 3, 0, 1}, {217, 202, 6, 0, 0, 0}, {218, 17, 7, 3, 0, 1}, {219, 162, 2, 0, 0, 0}, {226, 207, 8, 0, 0, 0}, {11000, 134, 51, 3, 4, 5}, {11001, 15, 135, 0, 0, 0}, {11002, 234, 179, 3, 4, 5}, {11003, 64, 5, 0, 0, 0}, {11010, 46, 49, 0, 0, 0}, {11011, 106, 44, 0, 0, 0}, {11020, 205, 16, 0, 0, 0}, {11030, 144, 44, 0, 0, 0}, {11031, 133, 44, 0, 0, 0}, {11032, 85, 44, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 2;


// ENUM DEFINITIONS


/** @brief  */
enum class ACCELCAL_VEHICLE_POS
{
    LEVEL=1, /*  | */
    LEFT=2, /*  | */
    RIGHT=3, /*  | */
    NOSEDOWN=4, /*  | */
    NOSEUP=5, /*  | */
    BACK=6, /*  | */
    SUCCESS=16777215, /*  | */
    FAILED=16777216, /*  | */
};

//! ACCELCAL_VEHICLE_POS ENUM_END
constexpr auto ACCELCAL_VEHICLE_POS_ENUM_END = 16777217;

/** @brief  */
enum class MAV_CMD
{
    NAV_ALTITUDE_WAIT=83, /* Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up. |Altitude (m).| Descent speed (m/s).| Wiggle Time (s).| Empty.| Empty.| Empty.| Empty.|  */
    DO_GRIPPER=211, /* Mission command to operate EPM gripper. |Gripper number (a number from 1 to max number of grippers on the vehicle).| Gripper action (0=release, 1=grab. See GRIPPER_ACTIONS enum).| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    DO_AUTOTUNE_ENABLE=212, /* Enable/disable autotune. |Enable (1: enable, 0:disable).| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    POWER_OFF_INITIATED=42000, /* A system wide power-off event has been initiated. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    SOLO_BTN_FLY_CLICK=42001, /* FLY button has been clicked. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    SOLO_BTN_FLY_HOLD=42002, /* FLY button has been held for 1.5 seconds. |Takeoff altitude.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    SOLO_BTN_PAUSE_CLICK=42003, /* PAUSE button has been clicked. |1 if Solo is in a shot mode, 0 otherwise.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    FIXED_MAG_CAL=42004, /* Magnetometer calibration based on fixed position
        in earth field given by inclination, declination and intensity. |MagDeclinationDegrees.| MagInclinationDegrees.| MagIntensityMilliGauss.| YawDegrees.| Empty.| Empty.| Empty.|  */
    FIXED_MAG_CAL_FIELD=42005, /* Magnetometer calibration based on fixed expected field values in milliGauss. |FieldX.| FieldY.| FieldZ.| Empty.| Empty.| Empty.| Empty.|  */
    DO_START_MAG_CAL=42424, /* Initiate a magnetometer calibration. |uint8_t bitmask of magnetometers (0 means all).| Automatically retry on failure (0=no retry, 1=retry).| Save without user input (0=require input, 1=autosave).| Delay (seconds).| Autoreboot (0=user reboot, 1=autoreboot).| Empty.| Empty.|  */
    DO_ACCEPT_MAG_CAL=42425, /* Initiate a magnetometer calibration. |uint8_t bitmask of magnetometers (0 means all).| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    DO_CANCEL_MAG_CAL=42426, /* Cancel a running magnetometer calibration. |uint8_t bitmask of magnetometers (0 means all).| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    SET_FACTORY_TEST_MODE=42427, /* Command autopilot to get into factory test/diagnostic mode. |0 means get out of test mode, 1 means get into test mode.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    DO_SEND_BANNER=42428, /* Reply with the version banner. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    ACCELCAL_VEHICLE_POS=42429, /* Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle in. When sent to the vehicle says what position the vehicle is in. |Position, one of the ACCELCAL_VEHICLE_POS enum values.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    GIMBAL_RESET=42501, /* Causes the gimbal to reset and boot as if it was just powered on. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    GIMBAL_AXIS_CALIBRATION_STATUS=42502, /* Reports progress and success or failure of gimbal axis calibration procedure. |Gimbal axis we're reporting calibration progress for.| Current calibration progress for this axis, 0x64=100%.| Status of the calibration.| Empty.| Empty.| Empty.| Empty.|  */
    GIMBAL_REQUEST_AXIS_CALIBRATION=42503, /* Starts commutation calibration on the gimbal. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    GIMBAL_FULL_RESET=42505, /* Erases gimbal application and parameters. |Magic number.| Magic number.| Magic number.| Magic number.| Magic number.| Magic number.| Magic number.|  */
    DO_WINCH=42600, /* Command to operate winch. |Winch number (0 for the default winch, otherwise a number from 1 to max number of winches on the vehicle).| Action (0=relax, 1=relative length control, 2=rate control. See WINCH_ACTIONS enum.).| Release length (cable distance to unwind in meters, negative numbers to wind in cable).| Release rate (meters/second).| Empty.| Empty.| Empty.|  */
    FLASH_BOOTLOADER=42650, /* Update the bootloader |Empty| Empty| Empty| Empty| Magic number - set to 290876 to actually flash| Empty| Empty|  */
};

//! MAV_CMD ENUM_END
constexpr auto MAV_CMD_ENUM_END = 42651;

/** @brief  */
enum class LIMITS_STATE : uint8_t
{
    INIT=0, /* Pre-initialization. | */
    DISABLED=1, /* Disabled. | */
    ENABLED=2, /* Checking limits. | */
    TRIGGERED=3, /* A limit has been breached. | */
    RECOVERING=4, /* Taking action e.g. Return/RTL. | */
    RECOVERED=5, /* We're no longer in breach of a limit. | */
};

//! LIMITS_STATE ENUM_END
constexpr auto LIMITS_STATE_ENUM_END = 6;

/** @brief  */
enum class LIMIT_MODULE : uint8_t
{
    GPSLOCK=1, /* Pre-initialization. | */
    GEOFENCE=2, /* Disabled. | */
    ALTITUDE=4, /* Checking limits. | */
};

//! LIMIT_MODULE ENUM_END
constexpr auto LIMIT_MODULE_ENUM_END = 5;

/** @brief Flags in RALLY_POINT message. */
enum class RALLY_FLAGS : uint8_t
{
    FAVORABLE_WIND=1, /* Flag set when requiring favorable winds for landing. | */
    LAND_IMMEDIATELY=2, /* Flag set when plane is to immediately descend to break altitude and land without GCS intervention. Flag not set when plane is to loiter at Rally point until commanded to land. | */
};

//! RALLY_FLAGS ENUM_END
constexpr auto RALLY_FLAGS_ENUM_END = 3;

/** @brief  */
enum class PARACHUTE_ACTION
{
    DISABLE=0, /* Disable parachute release. | */
    ENABLE=1, /* Enable parachute release. | */
    RELEASE=2, /* Release parachute. | */
};

//! PARACHUTE_ACTION ENUM_END
constexpr auto PARACHUTE_ACTION_ENUM_END = 3;

/** @brief Gripper actions. */
enum class GRIPPER_ACTIONS
{
    ACTION_RELEASE=0, /* Gripper release cargo. | */
    ACTION_GRAB=1, /* Gripper grab onto cargo. | */
};

//! GRIPPER_ACTIONS ENUM_END
constexpr auto GRIPPER_ACTIONS_ENUM_END = 2;

/** @brief Winch actions. */
enum class WINCH_ACTIONS
{
    RELAXED=0, /* Relax winch. | */
    RELATIVE_LENGTH_CONTROL=1, /* Winch unwinds or winds specified length of cable optionally using specified rate. | */
    RATE_CONTROL=2, /* Winch unwinds or winds cable at specified rate in meters/seconds. | */
};

//! WINCH_ACTIONS ENUM_END
constexpr auto WINCH_ACTIONS_ENUM_END = 3;

/** @brief  */
enum class CAMERA_STATUS_TYPES : uint8_t
{
    TYPE_HEARTBEAT=0, /* Camera heartbeat, announce camera component ID at 1Hz. | */
    TYPE_TRIGGER=1, /* Camera image triggered. | */
    TYPE_DISCONNECT=2, /* Camera connection lost. | */
    TYPE_ERROR=3, /* Camera unknown error. | */
    TYPE_LOWBATT=4, /* Camera battery low. Parameter p1 shows reported voltage. | */
    TYPE_LOWSTORE=5, /* Camera storage low. Parameter p1 shows reported shots remaining. | */
    TYPE_LOWSTOREV=6, /* Camera storage low. Parameter p1 shows reported video minutes remaining. | */
};

//! CAMERA_STATUS_TYPES ENUM_END
constexpr auto CAMERA_STATUS_TYPES_ENUM_END = 7;

/** @brief  */
enum class CAMERA_FEEDBACK_FLAGS : uint8_t
{
    PHOTO=0, /* Shooting photos, not video. | */
    VIDEO=1, /* Shooting video, not stills. | */
    BADEXPOSURE=2, /* Unable to achieve requested exposure (e.g. shutter speed too low). | */
    CLOSEDLOOP=3, /* Closed loop feedback from camera, we know for sure it has successfully taken a picture. | */
    OPENLOOP=4, /* Open loop camera, an image trigger has been requested but we can't know for sure it has successfully taken a picture. | */
};

//! CAMERA_FEEDBACK_FLAGS ENUM_END
constexpr auto CAMERA_FEEDBACK_FLAGS_ENUM_END = 5;

/** @brief  */
enum class MAV_MODE_GIMBAL
{
    UNINITIALIZED=0, /* Gimbal is powered on but has not started initializing yet. | */
    CALIBRATING_PITCH=1, /* Gimbal is currently running calibration on the pitch axis. | */
    CALIBRATING_ROLL=2, /* Gimbal is currently running calibration on the roll axis. | */
    CALIBRATING_YAW=3, /* Gimbal is currently running calibration on the yaw axis. | */
    INITIALIZED=4, /* Gimbal has finished calibrating and initializing, but is relaxed pending reception of first rate command from copter. | */
    ACTIVE=5, /* Gimbal is actively stabilizing. | */
    RATE_CMD_TIMEOUT=6, /* Gimbal is relaxed because it missed more than 10 expected rate command messages in a row. Gimbal will move back to active mode when it receives a new rate command. | */
};

//! MAV_MODE_GIMBAL ENUM_END
constexpr auto MAV_MODE_GIMBAL_ENUM_END = 7;

/** @brief  */
enum class GIMBAL_AXIS
{
    YAW=0, /* Gimbal yaw axis. | */
    PITCH=1, /* Gimbal pitch axis. | */
    ROLL=2, /* Gimbal roll axis. | */
};

//! GIMBAL_AXIS ENUM_END
constexpr auto GIMBAL_AXIS_ENUM_END = 3;

/** @brief  */
enum class GIMBAL_AXIS_CALIBRATION_STATUS
{
    IN_PROGRESS=0, /* Axis calibration is in progress. | */
    SUCCEEDED=1, /* Axis calibration succeeded. | */
    FAILED=2, /* Axis calibration failed. | */
};

//! GIMBAL_AXIS_CALIBRATION_STATUS ENUM_END
constexpr auto GIMBAL_AXIS_CALIBRATION_STATUS_ENUM_END = 3;

/** @brief  */
enum class GIMBAL_AXIS_CALIBRATION_REQUIRED
{
    UNKNOWN=0, /* Whether or not this axis requires calibration is unknown at this time. | */
    TRUE=1, /* This axis requires calibration. | */
    FALSE=2, /* This axis does not require calibration. | */
};

//! GIMBAL_AXIS_CALIBRATION_REQUIRED ENUM_END
constexpr auto GIMBAL_AXIS_CALIBRATION_REQUIRED_ENUM_END = 3;

/** @brief  */
enum class GOPRO_HEARTBEAT_STATUS : uint8_t
{
    DISCONNECTED=0, /* No GoPro connected. | */
    INCOMPATIBLE=1, /* The detected GoPro is not HeroBus compatible. | */
    CONNECTED=2, /* A HeroBus compatible GoPro is connected. | */
    ERROR=3, /* An unrecoverable error was encountered with the connected GoPro, it may require a power cycle. | */
};

//! GOPRO_HEARTBEAT_STATUS ENUM_END
constexpr auto GOPRO_HEARTBEAT_STATUS_ENUM_END = 4;

/** @brief  */
enum class GOPRO_HEARTBEAT_FLAGS : uint8_t
{
    FLAG_RECORDING=1, /* GoPro is currently recording. | */
};

//! GOPRO_HEARTBEAT_FLAGS ENUM_END
constexpr auto GOPRO_HEARTBEAT_FLAGS_ENUM_END = 2;

/** @brief  */
enum class GOPRO_REQUEST_STATUS : uint8_t
{
    SUCCESS=0, /* The write message with ID indicated succeeded. | */
    FAILED=1, /* The write message with ID indicated failed. | */
};

//! GOPRO_REQUEST_STATUS ENUM_END
constexpr auto GOPRO_REQUEST_STATUS_ENUM_END = 2;

/** @brief  */
enum class GOPRO_COMMAND : uint8_t
{
    POWER=0, /* (Get/Set). | */
    CAPTURE_MODE=1, /* (Get/Set). | */
    SHUTTER=2, /* (___/Set). | */
    BATTERY=3, /* (Get/___). | */
    MODEL=4, /* (Get/___). | */
    VIDEO_SETTINGS=5, /* (Get/Set). | */
    LOW_LIGHT=6, /* (Get/Set). | */
    PHOTO_RESOLUTION=7, /* (Get/Set). | */
    PHOTO_BURST_RATE=8, /* (Get/Set). | */
    PROTUNE=9, /* (Get/Set). | */
    PROTUNE_WHITE_BALANCE=10, /* (Get/Set) Hero 3+ Only. | */
    PROTUNE_COLOUR=11, /* (Get/Set) Hero 3+ Only. | */
    PROTUNE_GAIN=12, /* (Get/Set) Hero 3+ Only. | */
    PROTUNE_SHARPNESS=13, /* (Get/Set) Hero 3+ Only. | */
    PROTUNE_EXPOSURE=14, /* (Get/Set) Hero 3+ Only. | */
    TIME=15, /* (Get/Set). | */
    CHARGING=16, /* (Get/Set). | */
};

//! GOPRO_COMMAND ENUM_END
constexpr auto GOPRO_COMMAND_ENUM_END = 17;

/** @brief  */
enum class GOPRO_CAPTURE_MODE : uint8_t
{
    VIDEO=0, /* Video mode. | */
    PHOTO=1, /* Photo mode. | */
    BURST=2, /* Burst mode, Hero 3+ only. | */
    TIME_LAPSE=3, /* Time lapse mode, Hero 3+ only. | */
    MULTI_SHOT=4, /* Multi shot mode, Hero 4 only. | */
    PLAYBACK=5, /* Playback mode, Hero 4 only, silver only except when LCD or HDMI is connected to black. | */
    SETUP=6, /* Playback mode, Hero 4 only. | */
    UNKNOWN=255, /* Mode not yet known. | */
};

//! GOPRO_CAPTURE_MODE ENUM_END
constexpr auto GOPRO_CAPTURE_MODE_ENUM_END = 256;

/** @brief  */
enum class GOPRO_RESOLUTION
{
    RESOLUTION_480p=0, /* 848 x 480 (480p). | */
    RESOLUTION_720p=1, /* 1280 x 720 (720p). | */
    RESOLUTION_960p=2, /* 1280 x 960 (960p). | */
    RESOLUTION_1080p=3, /* 1920 x 1080 (1080p). | */
    RESOLUTION_1440p=4, /* 1920 x 1440 (1440p). | */
    RESOLUTION_2_7k_17_9=5, /* 2704 x 1440 (2.7k-17:9). | */
    RESOLUTION_2_7k_16_9=6, /* 2704 x 1524 (2.7k-16:9). | */
    RESOLUTION_2_7k_4_3=7, /* 2704 x 2028 (2.7k-4:3). | */
    RESOLUTION_4k_16_9=8, /* 3840 x 2160 (4k-16:9). | */
    RESOLUTION_4k_17_9=9, /* 4096 x 2160 (4k-17:9). | */
    RESOLUTION_720p_SUPERVIEW=10, /* 1280 x 720 (720p-SuperView). | */
    RESOLUTION_1080p_SUPERVIEW=11, /* 1920 x 1080 (1080p-SuperView). | */
    RESOLUTION_2_7k_SUPERVIEW=12, /* 2704 x 1520 (2.7k-SuperView). | */
    RESOLUTION_4k_SUPERVIEW=13, /* 3840 x 2160 (4k-SuperView). | */
};

//! GOPRO_RESOLUTION ENUM_END
constexpr auto GOPRO_RESOLUTION_ENUM_END = 14;

/** @brief  */
enum class GOPRO_FRAME_RATE
{
    RATE_12=0, /* 12 FPS. | */
    RATE_15=1, /* 15 FPS. | */
    RATE_24=2, /* 24 FPS. | */
    RATE_25=3, /* 25 FPS. | */
    RATE_30=4, /* 30 FPS. | */
    RATE_48=5, /* 48 FPS. | */
    RATE_50=6, /* 50 FPS. | */
    RATE_60=7, /* 60 FPS. | */
    RATE_80=8, /* 80 FPS. | */
    RATE_90=9, /* 90 FPS. | */
    RATE_100=10, /* 100 FPS. | */
    RATE_120=11, /* 120 FPS. | */
    RATE_240=12, /* 240 FPS. | */
    RATE_12_5=13, /* 12.5 FPS. | */
};

//! GOPRO_FRAME_RATE ENUM_END
constexpr auto GOPRO_FRAME_RATE_ENUM_END = 14;

/** @brief  */
enum class GOPRO_FIELD_OF_VIEW
{
    WIDE=0, /* 0x00: Wide. | */
    MEDIUM=1, /* 0x01: Medium. | */
    NARROW=2, /* 0x02: Narrow. | */
};

//! GOPRO_FIELD_OF_VIEW ENUM_END
constexpr auto GOPRO_FIELD_OF_VIEW_ENUM_END = 3;

/** @brief  */
enum class GOPRO_VIDEO_SETTINGS_FLAGS
{
    TV_MODE=1, /* 0=NTSC, 1=PAL. | */
};

//! GOPRO_VIDEO_SETTINGS_FLAGS ENUM_END
constexpr auto GOPRO_VIDEO_SETTINGS_FLAGS_ENUM_END = 2;

/** @brief  */
enum class GOPRO_PHOTO_RESOLUTION
{
    RESOLUTION_5MP_MEDIUM=0, /* 5MP Medium. | */
    RESOLUTION_7MP_MEDIUM=1, /* 7MP Medium. | */
    RESOLUTION_7MP_WIDE=2, /* 7MP Wide. | */
    RESOLUTION_10MP_WIDE=3, /* 10MP Wide. | */
    RESOLUTION_12MP_WIDE=4, /* 12MP Wide. | */
};

//! GOPRO_PHOTO_RESOLUTION ENUM_END
constexpr auto GOPRO_PHOTO_RESOLUTION_ENUM_END = 5;

/** @brief  */
enum class GOPRO_PROTUNE_WHITE_BALANCE
{
    AUTO=0, /* Auto. | */
    BALANCE_3000K=1, /* 3000K. | */
    BALANCE_5500K=2, /* 5500K. | */
    BALANCE_6500K=3, /* 6500K. | */
    RAW=4, /* Camera Raw. | */
};

//! GOPRO_PROTUNE_WHITE_BALANCE ENUM_END
constexpr auto GOPRO_PROTUNE_WHITE_BALANCE_ENUM_END = 5;

/** @brief  */
enum class GOPRO_PROTUNE_COLOUR
{
    STANDARD=0, /* Auto. | */
    NEUTRAL=1, /* Neutral. | */
};

//! GOPRO_PROTUNE_COLOUR ENUM_END
constexpr auto GOPRO_PROTUNE_COLOUR_ENUM_END = 2;

/** @brief  */
enum class GOPRO_PROTUNE_GAIN
{
    GAIN_400=0, /* ISO 400. | */
    GAIN_800=1, /* ISO 800 (Only Hero 4). | */
    GAIN_1600=2, /* ISO 1600. | */
    GAIN_3200=3, /* ISO 3200 (Only Hero 4). | */
    GAIN_6400=4, /* ISO 6400. | */
};

//! GOPRO_PROTUNE_GAIN ENUM_END
constexpr auto GOPRO_PROTUNE_GAIN_ENUM_END = 5;

/** @brief  */
enum class GOPRO_PROTUNE_SHARPNESS
{
    LOW=0, /* Low Sharpness. | */
    MEDIUM=1, /* Medium Sharpness. | */
    HIGH=2, /* High Sharpness. | */
};

//! GOPRO_PROTUNE_SHARPNESS ENUM_END
constexpr auto GOPRO_PROTUNE_SHARPNESS_ENUM_END = 3;

/** @brief  */
enum class GOPRO_PROTUNE_EXPOSURE
{
    NEG_5_0=0, /* -5.0 EV (Hero 3+ Only). | */
    NEG_4_5=1, /* -4.5 EV (Hero 3+ Only). | */
    NEG_4_0=2, /* -4.0 EV (Hero 3+ Only). | */
    NEG_3_5=3, /* -3.5 EV (Hero 3+ Only). | */
    NEG_3_0=4, /* -3.0 EV (Hero 3+ Only). | */
    NEG_2_5=5, /* -2.5 EV (Hero 3+ Only). | */
    NEG_2_0=6, /* -2.0 EV. | */
    NEG_1_5=7, /* -1.5 EV. | */
    NEG_1_0=8, /* -1.0 EV. | */
    NEG_0_5=9, /* -0.5 EV. | */
    ZERO=10, /* 0.0 EV. | */
    POS_0_5=11, /* +0.5 EV. | */
    POS_1_0=12, /* +1.0 EV. | */
    POS_1_5=13, /* +1.5 EV. | */
    POS_2_0=14, /* +2.0 EV. | */
    POS_2_5=15, /* +2.5 EV (Hero 3+ Only). | */
    POS_3_0=16, /* +3.0 EV (Hero 3+ Only). | */
    POS_3_5=17, /* +3.5 EV (Hero 3+ Only). | */
    POS_4_0=18, /* +4.0 EV (Hero 3+ Only). | */
    POS_4_5=19, /* +4.5 EV (Hero 3+ Only). | */
    POS_5_0=20, /* +5.0 EV (Hero 3+ Only). | */
};

//! GOPRO_PROTUNE_EXPOSURE ENUM_END
constexpr auto GOPRO_PROTUNE_EXPOSURE_ENUM_END = 21;

/** @brief  */
enum class GOPRO_CHARGING
{
    DISABLED=0, /* Charging disabled. | */
    ENABLED=1, /* Charging enabled. | */
};

//! GOPRO_CHARGING ENUM_END
constexpr auto GOPRO_CHARGING_ENUM_END = 2;

/** @brief  */
enum class GOPRO_MODEL
{
    UNKNOWN=0, /* Unknown gopro model. | */
    HERO_3_PLUS_SILVER=1, /* Hero 3+ Silver (HeroBus not supported by GoPro). | */
    HERO_3_PLUS_BLACK=2, /* Hero 3+ Black. | */
    HERO_4_SILVER=3, /* Hero 4 Silver. | */
    HERO_4_BLACK=4, /* Hero 4 Black. | */
};

//! GOPRO_MODEL ENUM_END
constexpr auto GOPRO_MODEL_ENUM_END = 5;

/** @brief  */
enum class GOPRO_BURST_RATE
{
    RATE_3_IN_1_SECOND=0, /* 3 Shots / 1 Second. | */
    RATE_5_IN_1_SECOND=1, /* 5 Shots / 1 Second. | */
    RATE_10_IN_1_SECOND=2, /* 10 Shots / 1 Second. | */
    RATE_10_IN_2_SECOND=3, /* 10 Shots / 2 Second. | */
    RATE_10_IN_3_SECOND=4, /* 10 Shots / 3 Second (Hero 4 Only). | */
    RATE_30_IN_1_SECOND=5, /* 30 Shots / 1 Second. | */
    RATE_30_IN_2_SECOND=6, /* 30 Shots / 2 Second. | */
    RATE_30_IN_3_SECOND=7, /* 30 Shots / 3 Second. | */
    RATE_30_IN_6_SECOND=8, /* 30 Shots / 6 Second. | */
};

//! GOPRO_BURST_RATE ENUM_END
constexpr auto GOPRO_BURST_RATE_ENUM_END = 9;

/** @brief  */
enum class LED_CONTROL_PATTERN
{
    OFF=0, /* LED patterns off (return control to regular vehicle control). | */
    FIRMWAREUPDATE=1, /* LEDs show pattern during firmware update. | */
    CUSTOM=255, /* Custom Pattern using custom bytes fields. | */
};

//! LED_CONTROL_PATTERN ENUM_END
constexpr auto LED_CONTROL_PATTERN_ENUM_END = 256;

/** @brief Flags in EKF_STATUS message. */
enum class EKF_STATUS_FLAGS : uint16_t
{
    ATTITUDE=1, /* Set if EKF's attitude estimate is good. | */
    VELOCITY_HORIZ=2, /* Set if EKF's horizontal velocity estimate is good. | */
    VELOCITY_VERT=4, /* Set if EKF's vertical velocity estimate is good. | */
    POS_HORIZ_REL=8, /* Set if EKF's horizontal position (relative) estimate is good. | */
    POS_HORIZ_ABS=16, /* Set if EKF's horizontal position (absolute) estimate is good. | */
    POS_VERT_ABS=32, /* Set if EKF's vertical position (absolute) estimate is good. | */
    POS_VERT_AGL=64, /* Set if EKF's vertical position (above ground) estimate is good. | */
    CONST_POS_MODE=128, /* EKF is in constant position mode and does not know it's absolute or relative position. | */
    PRED_POS_HORIZ_REL=256, /* Set if EKF's predicted horizontal position (relative) estimate is good. | */
    PRED_POS_HORIZ_ABS=512, /* Set if EKF's predicted horizontal position (absolute) estimate is good. | */
};

//! EKF_STATUS_FLAGS ENUM_END
constexpr auto EKF_STATUS_FLAGS_ENUM_END = 513;

/** @brief  */
enum class PID_TUNING_AXIS : uint8_t
{
    ROLL=1, /*  | */
    PITCH=2, /*  | */
    YAW=3, /*  | */
    ACCZ=4, /*  | */
    STEER=5, /*  | */
    LANDING=6, /*  | */
};

//! PID_TUNING_AXIS ENUM_END
constexpr auto PID_TUNING_AXIS_ENUM_END = 7;

/** @brief  */
enum class MAG_CAL_STATUS : uint8_t
{
    NOT_STARTED=0, /*  | */
    WAITING_TO_START=1, /*  | */
    RUNNING_STEP_ONE=2, /*  | */
    RUNNING_STEP_TWO=3, /*  | */
    SUCCESS=4, /*  | */
    FAILED=5, /*  | */
    BAD_ORIENTATION=6, /*  | */
};

//! MAG_CAL_STATUS ENUM_END
constexpr auto MAG_CAL_STATUS_ENUM_END = 7;

/** @brief Special ACK block numbers control activation of dataflash log streaming. */
enum class MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS : uint32_t
{
    STOP=2147483645, /* UAV to stop sending DataFlash blocks. | */
    START=2147483646, /* UAV to start sending DataFlash blocks. | */
};

//! MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS ENUM_END
constexpr auto MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_ENUM_END = 2147483647;

/** @brief Possible remote log data block statuses. */
enum class MAV_REMOTE_LOG_DATA_BLOCK_STATUSES : uint8_t
{
    NACK=0, /* This block has NOT been received. | */
    ACK=1, /* This block has been received. | */
};

//! MAV_REMOTE_LOG_DATA_BLOCK_STATUSES ENUM_END
constexpr auto MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_ENUM_END = 2;

/** @brief Bus types for device operations. */
enum class DEVICE_OP_BUSTYPE : uint8_t
{
    I2C=0, /* I2C Device operation. | */
    SPI=1, /* SPI Device operation. | */
};

//! DEVICE_OP_BUSTYPE ENUM_END
constexpr auto DEVICE_OP_BUSTYPE_ENUM_END = 2;

/** @brief Deepstall flight stage. */
enum class DEEPSTALL_STAGE : uint8_t
{
    FLY_TO_LANDING=0, /* Flying to the landing point. | */
    ESTIMATE_WIND=1, /* Building an estimate of the wind. | */
    WAIT_FOR_BREAKOUT=2, /* Waiting to breakout of the loiter to fly the approach. | */
    FLY_TO_ARC=3, /* Flying to the first arc point to turn around to the landing point. | */
    ARC=4, /* Turning around back to the deepstall landing point. | */
    APPROACH=5, /* Approaching the landing point. | */
    LAND=6, /* Stalling and steering towards the land point. | */
};

//! DEEPSTALL_STAGE ENUM_END
constexpr auto DEEPSTALL_STAGE_ENUM_END = 7;

/** @brief A mapping of plane flight modes for custom_mode field of heartbeat. */
enum class PLANE_MODE
{
    MANUAL=0, /*  | */
    CIRCLE=1, /*  | */
    STABILIZE=2, /*  | */
    TRAINING=3, /*  | */
    ACRO=4, /*  | */
    FLY_BY_WIRE_A=5, /*  | */
    FLY_BY_WIRE_B=6, /*  | */
    CRUISE=7, /*  | */
    AUTOTUNE=8, /*  | */
    AUTO=10, /*  | */
    RTL=11, /*  | */
    LOITER=12, /*  | */
    AVOID_ADSB=14, /*  | */
    GUIDED=15, /*  | */
    INITIALIZING=16, /*  | */
    QSTABILIZE=17, /*  | */
    QHOVER=18, /*  | */
    QLOITER=19, /*  | */
    QLAND=20, /*  | */
    QRTL=21, /*  | */
};

//! PLANE_MODE ENUM_END
constexpr auto PLANE_MODE_ENUM_END = 22;

/** @brief A mapping of copter flight modes for custom_mode field of heartbeat. */
enum class COPTER_MODE
{
    STABILIZE=0, /*  | */
    ACRO=1, /*  | */
    ALT_HOLD=2, /*  | */
    AUTO=3, /*  | */
    GUIDED=4, /*  | */
    LOITER=5, /*  | */
    RTL=6, /*  | */
    CIRCLE=7, /*  | */
    LAND=9, /*  | */
    DRIFT=11, /*  | */
    SPORT=13, /*  | */
    FLIP=14, /*  | */
    AUTOTUNE=15, /*  | */
    POSHOLD=16, /*  | */
    BRAKE=17, /*  | */
    THROW=18, /*  | */
    AVOID_ADSB=19, /*  | */
    GUIDED_NOGPS=20, /*  | */
    SMART_RTL=21, /*  | */
};

//! COPTER_MODE ENUM_END
constexpr auto COPTER_MODE_ENUM_END = 22;

/** @brief A mapping of sub flight modes for custom_mode field of heartbeat. */
enum class SUB_MODE
{
    STABILIZE=0, /*  | */
    ACRO=1, /*  | */
    ALT_HOLD=2, /*  | */
    AUTO=3, /*  | */
    GUIDED=4, /*  | */
    CIRCLE=7, /*  | */
    SURFACE=9, /*  | */
    POSHOLD=16, /*  | */
    MANUAL=19, /*  | */
};

//! SUB_MODE ENUM_END
constexpr auto SUB_MODE_ENUM_END = 20;

/** @brief A mapping of rover flight modes for custom_mode field of heartbeat. */
enum class ROVER_MODE
{
    MANUAL=0, /*  | */
    ACRO=1, /*  | */
    STEERING=3, /*  | */
    HOLD=4, /*  | */
    LOITER=5, /*  | */
    AUTO=10, /*  | */
    RTL=11, /*  | */
    SMART_RTL=12, /*  | */
    GUIDED=15, /*  | */
    INITIALIZING=16, /*  | */
};

//! ROVER_MODE ENUM_END
constexpr auto ROVER_MODE_ENUM_END = 17;

/** @brief A mapping of antenna tracker flight modes for custom_mode field of heartbeat. */
enum class TRACKER_MODE
{
    MANUAL=0, /*  | */
    STOP=1, /*  | */
    SCAN=2, /*  | */
    SERVO_TEST=3, /*  | */
    AUTO=10, /*  | */
    INITIALIZING=16, /*  | */
};

//! TRACKER_MODE ENUM_END
constexpr auto TRACKER_MODE_ENUM_END = 17;


} // namespace ardupilotmega
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_sensor_offsets.hpp"
#include "./mavlink_msg_set_mag_offsets.hpp"
#include "./mavlink_msg_meminfo.hpp"
#include "./mavlink_msg_ap_adc.hpp"
#include "./mavlink_msg_digicam_configure.hpp"
#include "./mavlink_msg_digicam_control.hpp"
#include "./mavlink_msg_mount_configure.hpp"
#include "./mavlink_msg_mount_control.hpp"
#include "./mavlink_msg_mount_status.hpp"
#include "./mavlink_msg_fence_point.hpp"
#include "./mavlink_msg_fence_fetch_point.hpp"
#include "./mavlink_msg_fence_status.hpp"
#include "./mavlink_msg_ahrs.hpp"
#include "./mavlink_msg_simstate.hpp"
#include "./mavlink_msg_hwstatus.hpp"
#include "./mavlink_msg_radio.hpp"
#include "./mavlink_msg_limits_status.hpp"
#include "./mavlink_msg_wind.hpp"
#include "./mavlink_msg_data16.hpp"
#include "./mavlink_msg_data32.hpp"
#include "./mavlink_msg_data64.hpp"
#include "./mavlink_msg_data96.hpp"
#include "./mavlink_msg_rangefinder.hpp"
#include "./mavlink_msg_airspeed_autocal.hpp"
#include "./mavlink_msg_rally_point.hpp"
#include "./mavlink_msg_rally_fetch_point.hpp"
#include "./mavlink_msg_compassmot_status.hpp"
#include "./mavlink_msg_ahrs2.hpp"
#include "./mavlink_msg_camera_status.hpp"
#include "./mavlink_msg_camera_feedback.hpp"
#include "./mavlink_msg_battery2.hpp"
#include "./mavlink_msg_ahrs3.hpp"
#include "./mavlink_msg_autopilot_version_request.hpp"
#include "./mavlink_msg_remote_log_data_block.hpp"
#include "./mavlink_msg_remote_log_block_status.hpp"
#include "./mavlink_msg_led_control.hpp"
#include "./mavlink_msg_mag_cal_progress.hpp"
#include "./mavlink_msg_mag_cal_report.hpp"
#include "./mavlink_msg_ekf_status_report.hpp"
#include "./mavlink_msg_pid_tuning.hpp"
#include "./mavlink_msg_deepstall.hpp"
#include "./mavlink_msg_gimbal_report.hpp"
#include "./mavlink_msg_gimbal_control.hpp"
#include "./mavlink_msg_gimbal_torque_cmd_report.hpp"
#include "./mavlink_msg_gopro_heartbeat.hpp"
#include "./mavlink_msg_gopro_get_request.hpp"
#include "./mavlink_msg_gopro_get_response.hpp"
#include "./mavlink_msg_gopro_set_request.hpp"
#include "./mavlink_msg_gopro_set_response.hpp"
#include "./mavlink_msg_rpm.hpp"
#include "./mavlink_msg_device_op_read.hpp"
#include "./mavlink_msg_device_op_read_reply.hpp"
#include "./mavlink_msg_device_op_write.hpp"
#include "./mavlink_msg_device_op_write_reply.hpp"
#include "./mavlink_msg_adap_tuning.hpp"
#include "./mavlink_msg_vision_position_delta.hpp"
#include "./mavlink_msg_aoa_ssa.hpp"
#include "./mavlink_msg_esc_telemetry_1_to_4.hpp"
#include "./mavlink_msg_esc_telemetry_5_to_8.hpp"
#include "./mavlink_msg_esc_telemetry_9_to_12.hpp"

// base include
#include "../common/common.hpp"
#include "../uAvionix/uAvionix.hpp"
#include "../icarous/icarous.hpp"
