/*------------------------------------------------------------------------------
* mav_bridge.c : MAVLink bridge + RTK position injection (see mav_bridge.h)
*-----------------------------------------------------------------------------*/
#include "mav_bridge.h"
#include "mavlink_min.h"
#include <string.h>
#include <math.h>

#ifdef PICO_BUILD
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#endif

#define FC_TX_RING 4096              /* power of two */

static mavbridge_cfg_t   cfg_;
static mavbridge_stats_t st_;
static mav_parser_t      fc_parser_;
static mav_parser_t      net_parser_;   /* observes ground -> FC traffic */
static mav_tx_t          tx_ = { .sysid = 1, .compid = 220, .seq = 0 };
static mavbridge_frame_hook_t frame_hook_;

/* Bytes waiting to go out of the UART to the flight controller. Filled from
 * the lwIP callback (an alarm IRQ) and from GPS_INPUT injection; drained by
 * the main loop. Single producer per side is not true here -- both the IRQ and
 * the main loop push -- so pushes are done with interrupts masked. */
static uint8_t  fc_tx_buf[FC_TX_RING];
static volatile uint32_t fc_tx_head, fc_tx_tail;

#ifdef PICO_BUILD
static struct udp_pcb *pcb_;
static ip_addr_t       server_ip_;
#endif

static void fc_tx_push(const uint8_t *d, uint16_t n)
{
#ifdef PICO_BUILD
    uint32_t save = save_and_disable_interrupts();
#endif
    for (uint16_t i = 0; i < n; i++) {
        uint32_t nx = (fc_tx_head + 1) & (FC_TX_RING - 1);
        if (nx == fc_tx_tail) { st_.dropped_to_fc++; break; }
        fc_tx_buf[fc_tx_head] = d[i];
        fc_tx_head = nx;
    }
    st_.to_fc_bytes += n;
#ifdef PICO_BUILD
    restore_interrupts(save);
#endif
}

int mavbridge_next_fc_byte(uint8_t *b)
{
    const uint8_t *f;
    uint16_t len;
    uint32_t t = fc_tx_tail;

    if (t == fc_tx_head) return 0;
    *b = fc_tx_buf[t];
    fc_tx_tail = (t + 1) & (FC_TX_RING - 1);

    /* Watch what we are handing the flight controller as well as what it sends
     * us. Skybrush's show firmware emits light commands outbound, but a ground
     * station can also drive colours directly, and this is where those arrive.
     * Parsing here rather than in the lwIP callback keeps the hook on the main
     * loop, so it cannot race the FC-direction parse. The forwarded bytes are
     * untouched -- this only observes. */
    if (frame_hook_ && mav_parse_byte(&net_parser_, *b, &f, &len)) {
        frame_hook_(f, len);
    }
    return 1;
}

/* --- network --------------------------------------------------------------- */
#ifdef PICO_BUILD
static void on_udp(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                   const ip_addr_t *addr, u16_t port)
{
    (void)arg; (void)pcb; (void)addr; (void)port;
    if (!p) return;
    /* Everything from the ground goes to the flight controller untouched: a
     * datagram may hold several MAVLink frames and the FC will reframe. */
    for (struct pbuf *q = p; q; q = q->next) {
        fc_tx_push((const uint8_t *)q->payload, q->len);
    }
    st_.from_net++;
    pbuf_free(p);
}

static void net_send(const uint8_t *d, uint16_t n)
{
    struct pbuf *p;
    if (!pcb_) return;
    cyw43_arch_lwip_begin();
    p = pbuf_alloc(PBUF_TRANSPORT, n, PBUF_RAM);
    if (p) {
        memcpy(p->payload, d, n);
        udp_sendto(pcb_, p, &server_ip_, cfg_.server_port);
        pbuf_free(p);
        st_.to_net++;
    }
    cyw43_arch_lwip_end();
}
#else
/* Host build: hand outbound frames to the test harness instead of a socket,
 * so the framing and forwarding logic is exercised without a network. */
__attribute__((weak)) void mavbridge_test_on_net(const uint8_t *d, uint16_t n)
{
    (void)d; (void)n;
}
static void net_send(const uint8_t *d, uint16_t n)
{
    mavbridge_test_on_net(d, n);
    st_.to_net++;
}
#endif

bool mavbridge_init(const mavbridge_cfg_t *cfg)
{
    memset(&st_, 0, sizeof(st_));
    cfg_ = *cfg;
    if (!cfg_.server_port) cfg_.server_port = 14550;
    if (!cfg_.local_port)  cfg_.local_port  = 14555;
    if (!cfg_.compid)      cfg_.compid      = 220;   /* MAV_COMP_ID_GPS */
    tx_.compid = cfg_.compid;
    tx_.sysid  = 1;
    mav_parser_reset(&fc_parser_);
    mav_parser_reset(&net_parser_);
    fc_tx_head = fc_tx_tail = 0;

#ifdef PICO_BUILD
    if (!ipaddr_aton(cfg_.server_ip, &server_ip_)) return false;
    cyw43_arch_lwip_begin();
    pcb_ = udp_new_ip_type(IPADDR_TYPE_V4);
    if (pcb_) {
        if (udp_bind(pcb_, IP_ANY_TYPE, cfg_.local_port) != ERR_OK) {
            udp_remove(pcb_); pcb_ = NULL;
        } else {
            udp_recv(pcb_, on_udp, NULL);
        }
    }
    cyw43_arch_lwip_end();
    st_.net_up = pcb_ != NULL;
    return st_.net_up;
#else
    st_.net_up = true;
    return true;
#endif
}

void mavbridge_fc_byte(uint8_t b)
{
    const uint8_t *f;
    uint16_t len;

    if (!mav_parse_byte(&fc_parser_, b, &f, &len)) return;

    st_.fc_frames = fc_parser_.frames;
    st_.resyncs   = fc_parser_.resyncs;

    /* Learn who the flight controller is, so injected GPS_INPUT carries the
     * same system id and the ground sees one coherent vehicle. */
    if (mav_frame_msgid(f) == MAVLINK_MSG_HEARTBEAT) {
        uint8_t sys = mav_frame_sysid(f);
        if (sys && sys != st_.fc_sysid) {
            st_.fc_sysid = sys;
            tx_.sysid    = sys;
        }
    }
    if (frame_hook_) frame_hook_(f, len);
    net_send(f, len);            /* one MAVLink frame per datagram */
}

/* --- RTK solution -> GPS_INPUT --------------------------------------------- */

/* Rotate the ECEF position covariance into ENU and take per-axis sigmas. */
static void enu_sigma(const sol_t *sol, const double *pos,
                      double *sh, double *sv)
{
    double P[9], Q[9], E[9];
    P[0] = sol->qr[0]; P[4] = sol->qr[1]; P[8] = sol->qr[2];
    P[1] = P[3] = sol->qr[3];
    P[5] = P[7] = sol->qr[4];
    P[2] = P[6] = sol->qr[5];
    xyz2enu(pos, E);
    matmul("NN", 3, 3, 3, 1.0, E, P, 0.0, Q);
    matmul("NT", 3, 3, 3, 1.0, Q, E, 0.0, P);
    *sh = sqrt((P[0] > 0 ? P[0] : 0) + (P[4] > 0 ? P[4] : 0));
    *sv = sqrt(P[8] > 0 ? P[8] : 0);
}

static uint8_t fix_type_of(int stat)
{
    switch (stat) {
        case SOLQ_FIX:    return MAV_FIX_RTK_FIXED;
        case SOLQ_FLOAT:  return MAV_FIX_RTK_FLOAT;
        case SOLQ_DGPS:   return MAV_FIX_DGPS;
        case SOLQ_SBAS:   return MAV_FIX_DGPS;
        case SOLQ_SINGLE: return MAV_FIX_3D;
    }
    return MAV_FIX_NONE;
}

bool mavbridge_send_gps_input(const rtk_t *rtk)
{
    const sol_t *sol = &rtk->sol;
    mav_gps_input_t g;
    uint8_t pay[80], frame[MAVLINK_MAX_FRAME];
    double pos[3], enu[3], azel[2 * MAXSAT], dop[4], sh, sv, tow;
    int i, ns = 0, week;
    uint16_t pl, fl;

    if (sol->stat == SOLQ_NONE || sol->time.time == 0) return false;

    memset(&g, 0, sizeof(g));
    tow = time2gpst(sol->time, &week);
    g.time_week    = (uint16_t)week;
    g.time_week_ms = (uint32_t)(tow * 1000.0 + 0.5);

    ecef2pos(sol->rr, pos);
    g.lat_e7  = (int32_t)(pos[0] * R2D * 1e7);
    g.lon_e7  = (int32_t)(pos[1] * R2D * 1e7);
    g.alt_msl = (float)(pos[2] - cfg_.geoid_sep);

    /* Real DOP from the geometry actually used, rather than a flattering
     * constant: ArduPilot's arming checks look at HDOP. */
    for (i = 0; i < MAXSAT && ns < MAXSAT; i++) {
        if (!rtk->ssat[i].vs) continue;
        azel[ns * 2]     = rtk->ssat[i].azel[0];
        azel[ns * 2 + 1] = rtk->ssat[i].azel[1];
        ns++;
    }
    if (ns >= 4) {
        dops(ns, azel, rtk->opt.elmin, dop);
        g.hdop = (float)dop[2];
        g.vdop = (float)dop[3];
    } else {
        g.ignore_flags |= GPS_INPUT_IGNORE_HDOP | GPS_INPUT_IGNORE_VDOP;
    }

    enu_sigma(sol, pos, &sh, &sv);
    g.horiz_acc = (float)sh;
    g.vert_acc  = (float)sv;

    /* Velocity comes from pntpos()'s Doppler solution: relpos() only
     * overwrites rr[0..2], so rr[3..5] is this epoch's estimate. estvel() can
     * leave it stale if it failed to converge, and feeding the EKF a wrong
     * velocity is far worse than feeding it none -- so sanity-check the
     * magnitude and drop the fields rather than guess. */
    ecef2enu(pos, sol->rr + 3, enu);
    if (norm(sol->rr + 3, 3) < 100.0) {
        g.vn = (float)enu[1];
        g.ve = (float)enu[0];
        g.vd = (float)(-enu[2]);
        g.speed_acc = 0.2f;              /* typical L1 Doppler, conservative */
    } else {
        g.ignore_flags |= GPS_INPUT_IGNORE_VEL_HORIZ | GPS_INPUT_IGNORE_VEL_VERT
                        | GPS_INPUT_IGNORE_SPEED_ACC;
    }

    g.gps_id   = cfg_.gps_id;
    g.fix_type = fix_type_of(sol->stat);
    g.sats     = sol->ns;

    pl = mav_build_gps_input(pay, &g);
    fl = mav_pack(&tx_, frame, MAVLINK_MSG_GPS_INPUT, MAVLINK_CRC_GPS_INPUT, pay, pl);
    fc_tx_push(frame, fl);
    st_.gps_sent++;
    return true;
}

const mavbridge_stats_t *mavbridge_stats(void) { return &st_; }

void mavbridge_set_frame_hook(mavbridge_frame_hook_t hook) { frame_hook_ = hook; }
