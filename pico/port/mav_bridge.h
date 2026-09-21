/*------------------------------------------------------------------------------
* mav_bridge.h : MAVLink bridge between the ground network and the flight
* controller, plus RTK position injection.
*
*   ground MAVLink server  <--UDP/Wi-Fi-->  Pico  <--UART, MAVLink2-->  ArduPilot
*
* One UART carries everything: GPS_INPUT (so the FC needs GPS_TYPE=14 and
* SERIAL<n>_PROTOCOL=2) and all mission/command/telemetry traffic, which is
* forwarded verbatim. The bridge never interprets mission or command
* messages -- the mission protocol is stateful with its own retries, and a
* middlebox that second-guesses it causes exactly the failures it is trying to
* prevent. It only frames traffic (to send whole messages per datagram) and
* learns the FC's system id from its heartbeats.
*-----------------------------------------------------------------------------*/
#ifndef MAV_BRIDGE_H
#define MAV_BRIDGE_H
#include <stdint.h>
#include <stdbool.h>
#include "rtklib.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *server_ip;    /* MAVLink server (mavlink-router/MAVProxy)     */
    uint16_t    server_port;  /* usually 14550                                */
    uint16_t    local_port;   /* our bind port; the server learns us from it  */
    uint8_t     gps_id;       /* must match the FC's GPS instance (0 = first) */
    uint8_t     compid;       /* our component id; 220 = MAV_COMP_ID_GPS      */
    /* Ellipsoidal-to-MSL offset for the flying site, metres: alt_msl =
     * h_ellipsoidal - geoid_sep. RTK gives ellipsoidal height but GPS_INPUT
     * wants MSL. Over a show-sized area this is a constant to well under a
     * centimetre; carrying a full geoid grid would cost ~60 KB of flash for
     * no benefit. Leave 0 to send ellipsoidal height and accept the offset
     * (harmless if you fly relative to home). */
    double      geoid_sep;
} mavbridge_cfg_t;

typedef struct {
    bool     net_up;
    uint8_t  fc_sysid;        /* learned from the FC's heartbeats, 0 = unseen */
    uint32_t fc_frames;       /* frames parsed from the FC                    */
    uint32_t to_net;          /* datagrams sent to the ground                 */
    uint32_t from_net;        /* datagrams received                           */
    uint32_t to_fc_bytes;     /* bytes queued toward the FC                   */
    uint32_t gps_sent;        /* GPS_INPUT messages injected                  */
    uint32_t dropped_to_fc;   /* ring full: FC link too slow                  */
    uint32_t resyncs;         /* framing resyncs on the FC link               */
} mavbridge_stats_t;

bool mavbridge_init(const mavbridge_cfg_t *cfg);

/* Feed one byte received from the flight controller. Complete frames are
 * forwarded to the ground; heartbeats also teach us the FC's system id. */
void mavbridge_fc_byte(uint8_t b);

/* Pull one byte destined for the flight controller's UART; 0 if none. */
int  mavbridge_next_fc_byte(uint8_t *b);

/* Build and queue a GPS_INPUT for the FC from the current RTK solution.
 * Returns false (and sends nothing) if the solution is not usable. */
bool mavbridge_send_gps_input(const rtk_t *rtk);

const mavbridge_stats_t *mavbridge_stats(void);

/* Observe complete frames arriving from the flight controller. Called before
 * the frame is forwarded to the ground, and must not modify it -- this is how
 * the light program is picked up without the bridge knowing about LEDs. */
typedef void (*mavbridge_frame_hook_t)(const uint8_t *frame, uint16_t len);
void mavbridge_set_frame_hook(mavbridge_frame_hook_t hook);

#ifdef __cplusplus
}
#endif
#endif /* MAV_BRIDGE_H */
