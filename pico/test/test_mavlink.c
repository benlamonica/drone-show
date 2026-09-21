/*------------------------------------------------------------------------------
* test_mavlink.c : MAVLink bridge tests
*
* The important one is end-to-end: simulated constellation -> UBX bytes ->
* RTKLIB -> fixed RTK solution -> GPS_INPUT. The resulting frame is printed as
* hex and checked by pymavlink against the simulator's own truth position, so
* a field-mapping or unit error cannot pass silently.
*
* Run standalone for the C-side checks, or via tools/check_mavlink.py for the
* pymavlink cross-validation.
*-----------------------------------------------------------------------------*/
#include "sim_scenario.h"
#include "mav_bridge.h"
#include "mavlink_min.h"
#include "test_util.h"
#include <string.h>
#include <stdlib.h>

/* captures frames the bridge would put on the wire */
static uint8_t  net_buf[4096];
static int      net_len = 0, net_frames = 0;
void mavbridge_test_on_net(const uint8_t *d, uint16_t n)
{
    if (net_len + n <= (int)sizeof(net_buf)) { memcpy(net_buf + net_len, d, n); net_len += n; }
    net_frames++;
}

static sim_scenario_t s;
static rtkeng_t eng;

/* --- 1. FC -> ground forwarding -------------------------------------------- */
static void test_forwarding(void)
{
    mavbridge_cfg_t cfg = {0};
    /* Genuine pymavlink output: HEARTBEAT(sys=7), COMMAND_LONG, HEARTBEAT.
     * Two heartbeats because the junk prefix below ends in a valid-looking
     * zero-length frame, which legitimately swallows the bytes after it --
     * exactly what a real link does after line noise. Recovery by the next
     * message is the property worth asserting. */
    static const uint8_t stream[] = {
        0xfd,0x09,0x00,0x00,0x00,0x07,0x01,0x00,0x00,0x00,0x05,0x00,
        0x00,0x00,0x02,0x03,0x51,0x04,0x03,0xb1,0x3a,0xfd,0x20,0x00,
        0x00,0x00,0x07,0x01,0x4c,0x00,0x00,0x00,0x00,0x80,0x3f,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x90,
        0x01,0x07,0x01,0x61,0xc3,0xfd,0x09,0x00,0x00,0x00,0x07,0x01,
        0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x02,0x03,0x51,0x04,0x03,
        0xb1,0x3a,
    };
    const mavbridge_stats_t *st;
    unsigned i;

    TEST_BEGIN("flight controller -> ground forwarding");
    cfg.server_ip = "127.0.0.1";
    CHECK(mavbridge_init(&cfg), "bridge init");
    net_len = net_frames = 0;

    /* junk, including a false start byte, must not break framing */
    mavbridge_fc_byte(0x11); mavbridge_fc_byte(0xFD); mavbridge_fc_byte(0x00);
    for (i = 0; i < sizeof(stream); i++) mavbridge_fc_byte(stream[i]);

    st = mavbridge_stats();
    CHECK(net_frames >= 2, "forwarded %d frames after junk", net_frames);
    CHECK(st->fc_sysid == 7, "learned the FC system id from HEARTBEAT (got %d)",
          st->fc_sysid);
    CHECK(st->resyncs > 0, "resynchronised past the junk prefix (%lu bytes)",
          (unsigned long)st->resyncs);
}

/* --- 2. RTK solution -> GPS_INPUT ------------------------------------------ */
static void test_gps_input(void)
{
    sim_scenario_cfg_t scfg;
    rtkeng_cfg_t ecfg;
    mavbridge_cfg_t mcfg = {0};
    uint8_t frame[512];
    int n = 0, i, fixed = 0;
    uint8_t b;
    double truth[3], pos[3];

    TEST_BEGIN("RTK solution -> GPS_INPUT");

    sim_scenario_default(&scfg);
    /* Moving rover: a static one cannot distinguish a real Doppler velocity
     * from stale zeros, and publishing a wrong velocity to the EKF is the
     * most dangerous thing this bridge could do. */
    scfg.motion = SIM_MOTION_CIRCLE;
    rtkeng_default_cfg(&ecfg);
    ecfg.mode = PMODE_KINEMA;
    sim_scenario_init(&s, &scfg);
    memcpy(ecfg.base_ecef, s.base.pos, sizeof(double) * 3);
    CHECK(rtkeng_init(&eng, &ecfg), "engine init");

    mcfg.server_ip = "127.0.0.1";
    mcfg.gps_id = 0;
    mavbridge_init(&mcfg);

    for (i = 0; i < 120; i++) {
        sim_scenario_step(&s, &eng);
        if (eng.rtk.sol.stat == SOLQ_FIX) { fixed = 1; break; }
    }
    CHECK(fixed, "reached a fixed RTK solution by epoch %d", i);

    memcpy(truth, s.true_rover_ecef, sizeof(truth));
    ecef2pos(truth, pos);

    CHECK(mavbridge_send_gps_input(&eng.rtk), "GPS_INPUT built and queued");
    while (n < (int)sizeof(frame) && mavbridge_next_fc_byte(&b)) frame[n++] = b;
    CHECK(n > 12, "queued %d bytes toward the flight controller", n);
    CHECK(frame[0] == 0xFD, "MAVLink 2 start byte");
    CHECK(mav_frame_msgid(frame) == MAVLINK_MSG_GPS_INPUT,
          "msgid %u == GPS_INPUT(232)", mav_frame_msgid(frame));
    CHECK(mav_frame_compid(frame) == 220, "compid 220 (MAV_COMP_ID_GPS)");

    /* Hand the frame and the truth to the Python checker. */
    printf("\nGPS_INPUT_HEX ");
    for (i = 0; i < n; i++) printf("%02x", frame[i]);
    printf("\nTRUTH_LLH %.9f %.9f %.4f\n", pos[0]*R2D, pos[1]*R2D, pos[2]);
    {   /* truth velocity in NED, for comparison against what we published */
        double venu[3];
        ecef2enu(pos, s.rover.vel, venu);
        printf("TRUTH_VEL_NED %.4f %.4f %.4f\n", venu[1], venu[0], -venu[2]);
    }
    printf("SOL_STAT %d  NS %d\n", eng.rtk.sol.stat, eng.rtk.sol.ns);

    rtkeng_free(&eng);
}

/* --- 3. a solution we must NOT send ---------------------------------------- */
static void test_rejects_bad_solution(void)
{
    rtk_t r;
    prcopt_t opt = prcopt_default;
    mavbridge_cfg_t cfg = {0};

    TEST_BEGIN("refuses to publish an unusable solution");
    cfg.server_ip = "127.0.0.1";
    mavbridge_init(&cfg);
    rtkinit(&r, &opt);

    r.sol.stat = SOLQ_NONE;
    r.sol.time.time = 1000;
    CHECK(!mavbridge_send_gps_input(&r), "SOLQ_NONE is not published");

    r.sol.stat = SOLQ_FIX;
    r.sol.time.time = 0;
    CHECK(!mavbridge_send_gps_input(&r), "a solution with no time is not published");

    rtkfree(&r);
}

int main(void)
{
    printf("RTKLIB Pico port - MAVLink bridge tests\n");
    test_forwarding();
    test_gps_input();
    test_rejects_bad_solution();
    TEST_SUMMARY();
    return g_tests_failed ? 1 : 0;
}
