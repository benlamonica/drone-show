/*------------------------------------------------------------------------------
* ubx_cfg.h : UBX configuration payload builders
*
* Separated from the firmware so the byte layouts can be checked on the host
* against the u-blox protocol spec. A wrong CFG-GNSS is silent -- the receiver
* simply never reports the constellation you expected -- so these are worth
* testing rather than eyeballing.
*-----------------------------------------------------------------------------*/
#ifndef UBX_CFG_H
#define UBX_CFG_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UBX_CFG_PRT_LEN   20
#define UBX_CFG_GNSS_LEN  (4 + 8 * 7)

/* CFG-PRT (0x06 0x00) for UART1: 8N1, UBX in and out only, at `baud`.
 * NMEA is disabled because we consume only UBX and it is otherwise pure
 * bandwidth. Writes UBX_CFG_PRT_LEN bytes. */
void ubx_cfg_prt(uint8_t *p, uint32_t baud);

/* CFG-GNSS (0x06 0x3E). `gal` enables Galileo, which the M8T leaves off by
 * default even on FW 3.01. GPS and QZSS are configured together as the
 * protocol spec requires. Writes UBX_CFG_GNSS_LEN bytes. */
void ubx_cfg_gnss(uint8_t *p, int gal, int glo, int bds);

#ifdef __cplusplus
}
#endif
#endif
