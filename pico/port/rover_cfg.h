/*------------------------------------------------------------------------------
* rover_cfg.h : boot-time configuration of the u-blox rover receiver
*
* Lifted out of main.c so the bench (app/gps_bench.c) drives the receiver
* through exactly the same sequence the firmware does. A bench that configured
* the receiver even slightly differently would be measuring a receiver state
* the firmware never actually sees, which is the one thing a bench must not do.
*-----------------------------------------------------------------------------*/
#ifndef ROVER_CFG_H
#define ROVER_CFG_H
#include <stdint.h>
#include "hardware/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Send one UBX frame: sync, header, payload, Fletcher-8 checksum. */
void ubx_send(uart_inst_t *u, uint8_t cls, uint8_t id,
              const uint8_t *payload, uint16_t len);

/* Walk the plausible baud rates telling whoever is listening to move to `baud`,
 * then enable Galileo, RXM-RAWX and RXM-SFRBX and set the measurement rate.
 * Leaves the UART at `baud`. Harmless on an already-configured receiver. */
void rover_cfg_apply(uart_inst_t *u, uint32_t baud, unsigned solution_rate_hz);

#ifdef __cplusplus
}
#endif
#endif /* ROVER_CFG_H */
