/*------------------------------------------------------------------------------
* ubx_cfg.c : UBX configuration payload builders (see ubx_cfg.h)
*-----------------------------------------------------------------------------*/
#include "ubx_cfg.h"
#include <string.h>

void ubx_cfg_prt(uint8_t *p, uint32_t baud)
{
    memset(p, 0, UBX_CFG_PRT_LEN);
    p[0]  = 1;                                   /* portID: UART1 */
    p[4]  = 0xD0; p[5] = 0x08;                   /* mode: 8 bits, no parity, 1 stop */
    p[8]  = (uint8_t)(baud);
    p[9]  = (uint8_t)(baud >> 8);
    p[10] = (uint8_t)(baud >> 16);
    p[11] = (uint8_t)(baud >> 24);
    /* UBX | RTCM3 (bit 5). The receiver needs RTCM3 in to run its own RTK
     * engine; without bit 5 the corrections are dropped at the port with no
     * error of any kind, and the receiver simply never leaves 3D-fix. */
    p[12] = 0x01 | 0x20;                         /* inProtoMask:  UBX + RTCM3 */
    p[14] = 0x01;                                /* outProtoMask: UBX */
}

void ubx_cfg_gnss(uint8_t *p, int gal, int glo, int bds)
{
    /* gnssId, resTrkCh, maxTrkCh, enable. The M8 tracks at most three
     * constellations concurrently, so the caller must not ask for more. */
    const uint8_t blk[7][4] = {
        {0, 8, 16, 1},                  /* GPS     */
        {1, 0,  0, 0},                  /* SBAS    */
        {2, 4,  8, (uint8_t)(gal != 0)},/* Galileo */
        {3, 4,  8, (uint8_t)(bds != 0)},/* BeiDou  */
        {4, 0,  0, 0},                  /* IMES    */
        {5, 0,  3, 1},                  /* QZSS: paired with GPS per the spec */
        /* 8/14, not Galileo's 4/8: when GLONASS is the second constellation it
         * carries half the solution, and an 8-channel ceiling would cap it
         * below the 8-10 satellites usually in view. These are u-blox's own
         * defaults for a GPS+GLONASS receiver. */
        {6, 8, 14, (uint8_t)(glo != 0)},/* GLONASS */
    };
    int i;

    memset(p, 0, UBX_CFG_GNSS_LEN);
    p[0] = 0;        /* msgVer                                  */
    p[1] = 0;        /* numTrkChHw, read-only                   */
    p[2] = 0xFF;     /* numTrkChUse: as many as the receiver has */
    p[3] = 7;        /* numConfigBlocks                          */

    for (i = 0; i < 7; i++) {
        uint8_t *b = p + 4 + 8 * i;
        b[0] = blk[i][0];
        b[1] = blk[i][1];
        b[2] = blk[i][2];
        b[4] = blk[i][3];    /* flags bit 0: enable            */
        b[6] = 0x01;         /* flags bits 16-23: signal mask  */
    }
}
