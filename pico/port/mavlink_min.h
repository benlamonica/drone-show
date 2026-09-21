/*------------------------------------------------------------------------------
* mavlink_min.h : the small slice of MAVLink 2 this bridge actually needs
*
* The generated MAVLink C library is thousands of headers; we send two message
* types and only need frame boundaries for the rest. So this is a hand-rolled
* framer whose constants (CRC_EXTRA, field offsets) were extracted from
* pymavlink rather than transcribed by eye -- see tools/mavlink_wire.py.
*
* Handles v1 (0xFE) and v2 (0xFD) framing on receive, because ArduPilot may
* emit v1 heartbeats before the link negotiates v2. Transmits v2 only.
*-----------------------------------------------------------------------------*/
#ifndef MAVLINK_MIN_H
#define MAVLINK_MIN_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAVLINK_STX_V1        0xFE
#define MAVLINK_STX_V2        0xFD
#define MAVLINK_MAX_FRAME     280   /* v2: 12 header/crc + 255 payload + 13 sig */

#define MAVLINK_MSG_HEARTBEAT   0
#define MAVLINK_MSG_GPS_INPUT 232

/* CRC_EXTRA, from pymavlink (common.xml) */
#define MAVLINK_CRC_HEARTBEAT  50
#define MAVLINK_CRC_GPS_INPUT 151

/* GPS_INPUT.ignore_flags */
#define GPS_INPUT_IGNORE_ALT        1
#define GPS_INPUT_IGNORE_HDOP       2
#define GPS_INPUT_IGNORE_VDOP       4
#define GPS_INPUT_IGNORE_VEL_HORIZ  8
#define GPS_INPUT_IGNORE_VEL_VERT  16
#define GPS_INPUT_IGNORE_SPEED_ACC 32
#define GPS_INPUT_IGNORE_HORIZ_ACC 64
#define GPS_INPUT_IGNORE_VERT_ACC 128

/* ArduPilot reads GPS_INPUT.fix_type straight into AP_GPS_FixType. */
#define MAV_FIX_NONE      1
#define MAV_FIX_2D        2
#define MAV_FIX_3D        3
#define MAV_FIX_DGPS      4
#define MAV_FIX_RTK_FLOAT 5
#define MAV_FIX_RTK_FIXED 6

/* --- incremental frame parser ---------------------------------------------
 * Feed bytes; returns true when `frame`/`len` hold one complete, CRC-unchecked
 * frame. We deliberately do not verify CRC on forwarded traffic: the endpoints
 * do that, and a bridge that silently drops frames it mis-parses is worse than
 * one that forwards them. CRC is only computed for messages we originate. */
typedef struct {
    uint8_t  buf[MAVLINK_MAX_FRAME];
    uint16_t got;        /* bytes accumulated */
    uint16_t need;       /* total frame length once the header is in */
    uint8_t  stx;
    bool     in_frame;
    uint32_t frames;     /* complete frames seen */
    uint32_t resyncs;    /* bytes discarded looking for a start byte */
} mav_parser_t;

void mav_parser_reset(mav_parser_t *p);
bool mav_parse_byte(mav_parser_t *p, uint8_t b,
                    const uint8_t **frame, uint16_t *len);

/* Message id / sysid / compid of a complete frame (either version). */
uint32_t mav_frame_msgid(const uint8_t *f);
uint8_t  mav_frame_sysid(const uint8_t *f);
uint8_t  mav_frame_compid(const uint8_t *f);

/* --- transmit -------------------------------------------------------------- */
typedef struct {
    uint8_t sysid, compid;
    uint8_t seq;
} mav_tx_t;

/* Frame `payload` as MAVLink 2 into `out`. Trailing zero bytes are truncated,
 * as the v2 spec allows and every receiver zero-fills. Returns frame length. */
uint16_t mav_pack(mav_tx_t *tx, uint8_t *out, uint32_t msgid, uint8_t crc_extra,
                  const uint8_t *payload, uint16_t payload_len);

/* GPS_INPUT payload builder. All units are MAVLink's, not RTKLIB's. */
typedef struct {
    uint32_t time_week_ms;
    uint16_t time_week;
    int32_t  lat_e7, lon_e7;     /* degrees * 1e7                  */
    float    alt_msl;            /* metres, MSL (not ellipsoidal)  */
    float    hdop, vdop;
    float    vn, ve, vd;         /* m/s, NED                       */
    float    speed_acc, horiz_acc, vert_acc;   /* m/s, m, m        */
    uint16_t ignore_flags;
    uint8_t  gps_id;             /* must match the FC's GPS instance */
    uint8_t  fix_type;
    uint8_t  sats;
} mav_gps_input_t;

uint16_t mav_build_gps_input(uint8_t *payload, const mav_gps_input_t *g);

/* HEARTBEAT payload (9 bytes). */
uint16_t mav_build_heartbeat(uint8_t *payload, uint8_t type, uint8_t autopilot,
                             uint8_t base_mode, uint32_t custom_mode,
                             uint8_t system_status);

#ifdef __cplusplus
}
#endif
#endif /* MAVLINK_MIN_H */
