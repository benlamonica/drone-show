/*------------------------------------------------------------------------------
* test_ubxcfg.c : UBX configuration payload layouts
*
* Checked field by field against the u-blox 8/M8 protocol spec. These messages
* fail silently on real hardware -- a malformed CFG-GNSS just means the
* constellation never appears -- so they are worth pinning down here.
*-----------------------------------------------------------------------------*/
#include "ubx_cfg.h"
#include "test_util.h"
#include <string.h>
#include <stdio.h>

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}

static void test_prt(void)
{
    uint8_t p[UBX_CFG_PRT_LEN];

    TEST_BEGIN("CFG-PRT (0x06 0x00)");
    ubx_cfg_prt(p, 230400);

    CHECK(sizeof(p) == 20, "payload is 20 bytes as the spec requires");
    CHECK(p[0] == 1, "portID 1 = UART1");
    CHECK(le32(p+4) == 0x000008D0, "mode 0x%08X = 8 bits, no parity, 1 stop",
          le32(p+4));
    CHECK(le32(p+8) == 230400, "baudRate %u", le32(p+8));
    /* UBX + RTCM3. RTCM3 input is what lets the receiver's own RTK engine see
     * the corrections we tee to it; with UBX alone the port silently discarded
     * every RTCM byte, so hardware RTK could never have worked. */
    CHECK(p[12] == (0x01 | 0x20) && p[13] == 0x00, "inProtoMask  = UBX + RTCM3");
    CHECK(p[14] == 0x01 && p[15] == 0x00, "outProtoMask = UBX only (no NMEA)");
    ubx_cfg_prt(p, 9600);
    CHECK(le32(p+8) == 9600, "baud field follows the argument");
}

static void test_gnss(void)
{
    uint8_t p[UBX_CFG_GNSS_LEN];
    int i, enabled = 0;

    TEST_BEGIN("CFG-GNSS (0x06 0x3E), GPS + Galileo");
    ubx_cfg_gnss(p, 1, 0, 0);

    CHECK(sizeof(p) == 60, "payload is 4 + 8*7 = 60 bytes");
    CHECK(p[0] == 0, "msgVer 0");
    CHECK(p[2] == 0xFF, "numTrkChUse 0xFF = use all hardware channels");
    CHECK(p[3] == 7, "7 configuration blocks");

    for (i = 0; i < 7; i++) {
        const uint8_t *b = p + 4 + 8*i;
        CHECK(b[0] == i, "block %d is gnssId %d", i, i);
        CHECK(b[6] == 0x01, "block %d signal mask set (flags bits 16-23)", i);
        if (b[4] & 1) enabled++;
    }
    {
        const uint8_t *gps = p+4, *sbas = p+12, *gal = p+20,
                      *bds = p+28, *qzss = p+44, *glo = p+52;
        CHECK(gps[4]  & 1, "GPS enabled");
        CHECK(gal[4]  & 1, "Galileo enabled - off by default on the M8T");
        CHECK(qzss[4] & 1, "QZSS enabled alongside GPS, as the spec requires");
        CHECK(!(sbas[4] & 1), "SBAS disabled");
        CHECK(!(bds[4]  & 1), "BeiDou disabled");
        CHECK(!(glo[4]  & 1), "GLONASS disabled");
        CHECK(gps[2] == 16 && gal[2] == 8,
              "channel reservations GPS=%d Galileo=%d", gps[2], gal[2]);
    }
    /* The M8 tracks at most three constellations at once. */
    CHECK(enabled <= 3, "%d constellations enabled, M8 hardware limit is 3", enabled);

    ubx_cfg_gnss(p, 0, 1, 0);
    CHECK(!(p[20+4] & 1), "Galileo off when not requested");
    CHECK(p[52+4] & 1,    "GLONASS on when requested");
    /* GLONASS carries half the solution on the M8P pairing this port targets
     * (MON-VER: "GPS;GLO;BDS" -- no Galileo), so it gets GPS-class channel
     * reservations rather than Galileo's 4/8, which would cap it below the
     * 8-10 satellites usually in view. */
    CHECK(p[52+1] == 8 && p[52+2] == 14,
          "GLONASS reserved 8, max 14 channels (got %d/%d)", p[52+1], p[52+2]);
}

int main(void)
{
    printf("RTKLIB Pico port - UBX configuration tests\n");
    test_prt();
    test_gnss();
    TEST_SUMMARY();
    return g_tests_failed ? 1 : 0;
}
