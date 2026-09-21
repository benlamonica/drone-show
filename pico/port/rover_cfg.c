/*------------------------------------------------------------------------------
* rover_cfg.c : boot-time configuration of the u-blox rover receiver
*
* Moved verbatim out of app/main.c so app/gps_bench.c can share it. See
* rover_cfg.h for why sharing matters rather than copying.
*-----------------------------------------------------------------------------*/
#include <string.h>
#include "rover_cfg.h"
#include "ubx_cfg.h"
#include "pico/stdlib.h"

void ubx_send(uart_inst_t *u, uint8_t cls, uint8_t id,
              const uint8_t *payload, uint16_t len)
{
    uint8_t hdr[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint8_t cka = 0, ckb = 0, ck[2];
    int i;
    for (i = 2; i < 6; i++)   { cka += hdr[i];     ckb += cka; }
    for (i = 0; i < len; i++) { cka += payload[i]; ckb += cka; }
    ck[0] = cka; ck[1] = ckb;
    uart_write_blocking(u, hdr, 6);
    if (len) uart_write_blocking(u, payload, len);
    uart_write_blocking(u, ck, 2);
}

/* Baud rates to try when hunting for the receiver. A NEO-M8T leaves the
 * factory at 9600, so a fleet would otherwise need every unit configured by
 * hand in u-center before it would say anything at the target baud. */
static const uint32_t ROVER_BAUD_CANDIDATES[] = {230400, 9600, 38400, 115200, 57600, 460800};

static void ubx_set_port(uart_inst_t *u, uint32_t baud)
{
    uint8_t p[UBX_CFG_PRT_LEN];
    ubx_cfg_prt(p, baud);
    ubx_send(u, 0x06, 0x00, p, sizeof(p));
}

/* Both ends are NEO-M8P-2 / HPG 1.43REF, and MON-VER reports their capability
 * as "GPS;GLO;BDS" -- no Galileo. The base backs that up on the wire: it sends
 * 1077 (GPS MSM7), 1087 (GLONASS MSM7) and 1230 (GLONASS code-phase biases).
 *
 * So GLONASS, not Galileo, is the second constellation available here, and it
 * is already being broadcast by the base and thrown away by this rover. Asking
 * for Galileo cost a silent no-op on the receiver and 36 reserved satellite
 * slots in RTKLIB (MAXSAT 68 vs 59) for satellites that can never arrive.
 *
 * Switching needs BOTH sides: this flag *and* a build with RTKPICO_ENABLE_GLO
 * and RTKPICO_ENABLE_GAL=OFF, because rtkpico_config.h gates ENAGLO. Setting
 * one without the other either hands satno() satellites it cannot number
 * (silently dropped) or reserves state for a constellation that never appears.
 * Measured: GPS+GLO is 249 KB live, against 285 KB for the GPS+GAL pairing it
 * replaces -- the swap frees memory rather than costing it. Enabling GLONASS
 * *alongside* Galileo is what exhausted the heap previously, at 402 KB. */
static void ubx_set_gnss(uart_inst_t *u)
{
    uint8_t p[UBX_CFG_GNSS_LEN];
    ubx_cfg_gnss(p, 0 /* Galileo */, 1 /* GLONASS */, 0 /* BeiDou */);
    ubx_send(u, 0x06, 0x3E, p, sizeof(p));
}

void rover_cfg_apply(uart_inst_t *u, uint32_t baud, unsigned solution_rate_hz)
{
    const uint8_t rawx[3]  = {0x02, 0x15, 1};   /* CFG-MSG: RXM-RAWX  rate 1 */
    const uint8_t sfrbx[3] = {0x02, 0x13, 1};   /* CFG-MSG: RXM-SFRBX rate 1 */
    /* The receiver's own RTK result, for comparison against our RTKLIB one.
     * NAV-RELPOSNED is the direct analogue of what the engine computes: the
     * base->rover vector, with its own carrier-solution status and accuracy
     * estimate. NAV-PVT comes along for the satellite count the hardware is
     * actually using, which RELPOSNED does not carry. */
    const uint8_t relpos[3] = {0x01, 0x3C, 1};  /* CFG-MSG: NAV-RELPOSNED     */
    const uint8_t pvt[3]    = {0x01, 0x07, 1};  /* CFG-MSG: NAV-PVT           */
    /* The receiver's acknowledgement that our RTCM actually arrived and was
     * used. Without it, a correction path that is silently dropping frames is
     * indistinguishable from one that simply has no satellites to apply them
     * to -- which is precisely the failure that wastes a trip to the field. */
    const uint8_t rxmrtcm[3] = {0x02, 0x32, 1};/* CFG-MSG: RXM-RTCM          */
    uint16_t ms = (uint16_t)(1000u / (solution_rate_hz ? solution_rate_hz : 1u));
    uint8_t rate[6] = {(uint8_t)(ms & 0xFF), (uint8_t)(ms >> 8), 1, 0, 1, 0};
    unsigned i;

    for (i = 0; i < count_of(ROVER_BAUD_CANDIDATES); i++) {
        uart_set_baudrate(u, ROVER_BAUD_CANDIDATES[i]);
        sleep_ms(10);

        /* Terminate any RTCM3 frame the receiver is part-way through before
         * saying anything in UBX. Once RTCM3 is enabled on this port, a frame
         * header commits the receiver to consuming its declared length -- up to
         * 1023 payload bytes -- and if we reset mid-frame it will swallow the
         * whole configuration burst below as payload and answer nothing. That
         * is not hypothetical: it is why a Pico reset alone could not recover
         * the receiver while a reflash could. Flashing halts the core for
         * seconds with the line idle, which lets the parser resync; a reset
         * resumes instantly and does not. 0xFF is not a sync byte for any
         * protocol on this port, so the padding is inert once consumed. */
        {
            uint8_t pad[1030];   /* > max RTCM3 frame: 3 + 1023 + 3 */
            memset(pad, 0xFF, sizeof(pad));
            uart_write_blocking(u, pad, sizeof(pad));
            uart_tx_wait_blocking(u);
            sleep_ms(20);
        }

        ubx_set_port(u, baud);
        uart_tx_wait_blocking(u);
        sleep_ms(60);            /* the receiver reconfigures its port */
    }
    uart_set_baudrate(u, baud);
    sleep_ms(150);

    ubx_set_gnss(u);  uart_tx_wait_blocking(u);
    sleep_ms(1200);              /* CFG-GNSS restarts the GNSS subsystem */

    ubx_send(u, 0x06, 0x01, rawx,  3); sleep_ms(20);
    ubx_send(u, 0x06, 0x01, sfrbx, 3); sleep_ms(20);
    ubx_send(u, 0x06, 0x01, relpos,3); sleep_ms(20);
    ubx_send(u, 0x06, 0x01, pvt,   3); sleep_ms(20);
    ubx_send(u, 0x06, 0x01, rxmrtcm,3);sleep_ms(20);
    ubx_send(u, 0x06, 0x08, rate,  6); sleep_ms(20);
}
