/*------------------------------------------------------------------------------
* rtkpico_config.h : compile-time configuration for the RTKLIB embedded port
*
* Included by rtklib.h when RTKLIB_EMBEDDED is defined. Everything here is
* either (a) an override of an RTKLIB #ifndef-guarded default, or (b) a feature
* switch consumed by the RTKLIB_EMBEDDED guards added to rtklib.h.
*
* Sizing rationale (RP2350 / Pico 2 W: 520 KB SRAM, 4 MB flash):
*   the stock RTKLIB configuration needs ~550 KB of RAM for a rover+base RTK
*   session, which does not fit. The dominant consumers are all features this
*   port does not use (SBAS iono grids, satellite antenna PCV tables, SSR/
*   PPP-RTK corrections, QZSS LEX). Removing them plus dropping to L1-only
*   brings a GPS+Galileo rover+base session to ~120 KB.
*-----------------------------------------------------------------------------*/
#ifndef RTKPICO_CONFIG_H
#define RTKPICO_CONFIG_H

/* --- signal / constellation configuration --------------------------------- */

/* L1 C/A + E1 only, matching the raw-capable but non-RTK u-blox parts
 * (NEO/LEA-M8T class).
 *
 * NFREQ must be >= 2 even though we only ever process L1. In 2.4.3 prange()
 * no longer refuses NFREQ<2 outright (2.4.2 did), but it still reads
 * obs->P[1] and obs->code[1] unconditionally before the ionoopt test. At
 * NFREQ=1 those index one past `double P[NFREQ+NEXOBS]`, so an NFREQ==1 build
 * is undefined behaviour even though the values go unused outside
 * IONOOPT_IFLC. Not worth 12 KB when 340 KB is free.
 * The frequency count that actually drives processing (and the EKF state
 * size) is prcopt_t.nf, which the engine pins to 1: NFREQ=2 sizes the
 * per-satellite arrays while nf=1 keeps the filter single-frequency, the
 * standard RTKLIB configuration for an L1-only receiver.                     */
#ifndef NFREQ
#define NFREQ       2
#endif
#ifndef NEXOBS
#define NEXOBS      0
#endif

/* Max observations per epoch. M8T tracks 32 channels concurrently; 48 leaves
 * headroom for a 3-constellation configuration without wasting much.          */
#ifndef MAXOBS
#define MAXOBS      48
#endif

/* IRNSS is new in 2.4.3 and off by default; naming it here records that the
 * omission is deliberate. */
#ifdef RTKPICO_ENABLE_IRN
#define ENAIRN
#endif

/* Galileo E1 shares L1 hardware on M8-class receivers: nearly free extra
 * satellites, and E1 ambiguity resolution behaves well. Enabled by default.
 * GLONASS is deliberately NOT enabled: it is FDMA, so single-frequency
 * GLONASS AR needs per-receiver inter-channel bias calibration, and it costs
 * 24 more EKF states. Define RTKPICO_ENABLE_GLO to opt in anyway.             */
#ifndef RTKPICO_NO_GAL
#define ENAGAL
#endif
#ifdef RTKPICO_ENABLE_GLO
#define ENAGLO
#endif
#ifdef RTKPICO_ENABLE_BDS
#define ENACMP
#endif

/* SBAS satellites occupy MAXSAT slots (and therefore EKF states, ~28 KB of
 * covariance) even when SBAS is unused. An empty PRN range removes them while
 * keeping satno()/satsys() arithmetic correct and keeping the
 * "MINPRNSBS<=prn" tests in the receiver decoders false for GPS PRNs.         */
#define MINPRNSBS   120
#define MAXPRNSBS   119                 /* -> NSATSBS == 0 (2.4.3 defaults to
                                           120..158: 39 wasted slots) */

/* --- buffer sizing -------------------------------------------------------- */

/* Largest UBX message we care about is RXM-RAWX: 8 + 16 + 32*numMeas bytes,
 * i.e. 1560 bytes at 48 satellites. 2048 covers it with margin.               */
#ifndef MAXRAWLEN
#define MAXRAWLEN   2048
#endif

/* GPS LNAV needs 5 subframes x 30 bytes = 150; Galileo I/NAV uses [0..112].
 * GLONASS keeps its frame-id at [150..151] (ublox.c decode_gnav), so 150 put
 * it in the next satellite's buffer: adjacent slots corrupted each other's
 * string 1 and no GLONASS ephemeris ever decoded. BeiDou D1/D2 would need 418. */
#ifndef MAXSUBFRMLEN
#define MAXSUBFRMLEN 152
#endif

/* RTKLIB's 4 KB per-rtk_t error ring is a debugging aid; 256 is plenty. */
#ifndef MAXERRMSG
#define MAXERRMSG   256
#endif

/* --- feature removal (drives the RTKLIB_EMBEDDED guards in rtklib.h) ------- */
#define RTKLIB_NO_SBAS          /* SBAS ranging/iono grid  (~83 KB in nav_t)  */
#define RTKLIB_NO_PCV           /* antenna phase-centre variation tables       */
#define RTKLIB_NO_SSR           /* SSR / PPP-RTK corrections                   */
#define RTKLIB_NO_DGPS          /* RTCM2 DGPS corrections                      */
/* RTKLIB_NO_LEX is unnecessary: 2.4.3 removed QZSS LEX entirely.             */
#define RTKLIB_NO_PPP           /* precise point positioning + ambiguity ctl   */

/* --- platform ------------------------------------------------------------- */

/* RTKLIB resolves the 10-bit GPS week in the LNAV subframe against the system
 * clock (adjgpsweek -> timeget). A bare Pico has no RTC at boot, so seed the
 * era. The engine calls timeset() from the first RXM-RAWX, after which the
 * real time is used. Only the 1024-week era matters here.                     */
#ifndef RTKPICO_SEED_GPS_WEEK
#define RTKPICO_SEED_GPS_WEEK 2300      /* 2024-01-07; valid until ~2043       */
#endif

#endif /* RTKPICO_CONFIG_H */
