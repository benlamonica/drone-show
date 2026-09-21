/*------------------------------------------------------------------------------
* ubx_encode.h : UBX message generator for the GNSS simulator
*
* Produces byte-exact UBX-RXM-RAWX and UBX-RXM-SFRBX messages so tests drive
* the same decode path the firmware uses on real hardware: sync_ubx() ->
* decode_rxmrawx()/decode_rxmsfrbx() -> save_subfrm() -> decode_frame().
* Nothing is injected behind the decoder's back.
*-----------------------------------------------------------------------------*/
#ifndef UBX_ENCODE_H
#define UBX_ENCODE_H
#include "rtklib.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UBX_MAX_MSG 2048

/* Per-satellite transmit-side detail that obsd_t does not carry. */
typedef struct {
    uint16_t locktime_ms;   /* carrier lock time; 0 or a decrease => cycle slip */
    uint8_t  cno;           /* dBHz                                            */
    uint8_t  pr_stdev;      /* UBX code:   0.01*2^n metres                     */
    uint8_t  cp_stdev;      /* UBX code:   0.004*n cycles; >5 marks cp invalid */
    uint8_t  do_stdev;
    uint8_t  trk_stat;      /* b0 pr valid, b1 cp valid, b2 half-cyc valid     */
    uint8_t  half_cyc_sub;  /* b3 of trkStat: half cycle subtracted            */
} ubx_meas_aux_t;

/* Encode one UBX-RXM-RAWX. Returns message length, or 0 if it would overflow.
 * aux may be NULL, in which case healthy defaults are used. */
int ubx_encode_rawx(unsigned char *buf, int size, gtime_t time,
                    const obsd_t *obs, int n, const ubx_meas_aux_t *aux);

/* Encode one UBX-RXM-SFRBX carrying GPS LNAV subframe `id` (1..3) for `eph`.
 * `tow` is the transmission time of the NEXT subframe, as broadcast in HOW. */
int ubx_encode_sfrbx_gps(unsigned char *buf, int size, int prn, int id,
                         const eph_t *eph, double tow);

/* Encode one UBX-RXM-SFRBX carrying GLONASS navigation string `m` (1..4) for
 * `slot`. Only what decode_glostr() checks is real -- string number, tb in
 * string 2, slot in string 4, Hamming bits -- so the ephemeris it yields is
 * zero-valued: enough to test framing and buffering, not orbit maths. */
int ubx_encode_sfrbx_glo(unsigned char *buf, int size, int slot, int m, int tb);

/* Pack a 30-byte GPS LNAV subframe (10 words x 24 data bits, no parity).
 * Exposed so tests can check the encoder against decode_frame() directly. */
void gps_lnav_subframe(unsigned char *sf, int id, const eph_t *eph, double tow);

#ifdef __cplusplus
}
#endif
#endif /* UBX_ENCODE_H */
