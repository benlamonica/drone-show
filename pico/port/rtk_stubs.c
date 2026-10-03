/*------------------------------------------------------------------------------
* rtk_stubs.c : stubs for RTKLIB subsystems excluded from the embedded port
*
* The embedded build compiles only rtkcmn/rtkpos/pntpos/ephemeris/lambda/
* rcvraw/ublox (+ optionally rtcm/rtcm3). Those files still reference a handful
* of symbols from modules we deliberately leave out. Each stub below is the
* "feature absent" answer, chosen so RTKLIB's own fallback paths take over.
*
* Nothing here is reachable in the default configuration: the engine forces
* EPHOPT_BRDC, PMODE_KINEMA/STATIC and IONOOPT_BRDC/OFF, which never dispatch
* to SBAS, SSR, precise-ephemeris, LEX or PPP. They exist to satisfy the linker
* and to fail safe if an option is ever mis-set at runtime.
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

/* --- SBAS (sbas.c) -------------------------------------------------------- */
/* nav_t carries no SBAS correction tables in this build; report "no data" so
 * satpos()/ionocorr() fall back to broadcast ephemeris and Klobuchar.        */
extern int sbssatcorr(gtime_t time, int sat, const nav_t *nav, double *rs,
                      double *dts, double *var)
{
    (void)time; (void)sat; (void)nav; (void)rs; (void)dts; (void)var;
    return 0;
}
extern int sbsioncorr(gtime_t time, const nav_t *nav, const double *pos,
                      const double *azel, double *delay, double *var)
{
    (void)time; (void)nav; (void)pos; (void)azel;
    if (delay) *delay=0.0;
    if (var) *var=5.0*5.0;   /* ERR_ION: unmodelled -> full iono variance */
    return 0;
}
extern double sbstropcorr(gtime_t time, const double *pos, const double *azel,
                          double *var)
{
    (void)time; (void)pos; (void)azel;
    if (var) *var=3.0*3.0;      /* ERR_TROP from pntpos.c */
    return 0.0;
}
extern int sbsdecodemsg(gtime_t time, int prn, const uint32_t *words,
                        sbsmsg_t *sbsmsg)
{
    (void)time; (void)prn; (void)words; (void)sbsmsg;
    return 0;   /* ublox.c decode_snav(): drop SBAS nav pages */
}

/* --- precise ephemeris / antenna offsets (preceph.c) ---------------------- */
extern int peph2pos(gtime_t time, int sat, const nav_t *nav, int opt,
                    double *rs, double *dts, double *var)
{
    (void)time; (void)sat; (void)nav; (void)opt; (void)rs; (void)dts; (void)var;
    return 0;
}
extern void satantoff(gtime_t time, const double *rs, int sat, const nav_t *nav,
                      double *dant)
{
    (void)time; (void)rs; (void)sat; (void)nav;
    if (dant) dant[0]=dant[1]=dant[2]=0.0;
}

/* --- ionosphere TEC grid (ionex.c) ---------------------------------------- */
extern int iontec(gtime_t time, const nav_t *nav, const double *pos,
                  const double *azel, int opt, double *delay, double *var)
{
    (void)time; (void)nav; (void)pos; (void)azel; (void)opt;
    if (delay) *delay=0.0;
    if (var) *var=5.0*5.0;      /* ERR_ION from pntpos.c */
    return 0;
}

/* --- QZSS LEX (qzslex.c) -------------------------------------------------- */
extern int lexeph2pos(gtime_t time, int sat, const nav_t *nav, double *rs,
                      double *dts, double *var)
{
    (void)time; (void)sat; (void)nav; (void)rs; (void)dts; (void)var;
    return 0;
}
extern int lexioncorr(gtime_t time, const nav_t *nav, const double *pos,
                      const double *azel, double *delay, double *var)
{
    (void)time; (void)nav; (void)pos; (void)azel;
    if (delay) *delay=0.0;
    if (var) *var=5.0*5.0;      /* ERR_ION from pntpos.c */
    return 0;
}

/* --- PPP (ppp.c) ---------------------------------------------------------- */
/* rtkpos() dispatches here only for PMODE_PPP_*; the engine rejects those
 * modes at init, so reaching pppos() means a mis-configured prcopt_t.        */
extern int pppnx(const prcopt_t *opt)
{
    (void)opt;
    return 0;
}
extern void pppos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    (void)obs; (void)n; (void)nav;
    if (rtk) rtk->sol.stat=SOLQ_NONE;
}
/* 2.4.3 renamed pppoutsolstat(rtk,level,FILE*) to pppoutstat(rtk,char*),
 * which now returns the number of bytes written into the caller's buffer. */
extern int pppoutstat(rtk_t *rtk, char *buff)
{
    (void)rtk;
    if (buff) buff[0]='\0';
    return 0;
}

/* --- earth tides (ppp.c) -------------------------------------------------- */
/* Only called when prcopt_t.tidecorr != 0. Sub-decimetre effect on a baseline
 * of a few km it largely cancels anyway, and it needs erp/ocean-loading data
 * we do not carry.                                                           */
extern void tidedisp(gtime_t tutc, const double *rr, int opt, const erp_t *erp,
                     const double *odisp, double *dr)
{
    (void)tutc; (void)rr; (void)opt; (void)erp; (void)odisp;
    if (dr) dr[0]=dr[1]=dr[2]=0.0;
}

/* --- other receiver formats (rcvraw.c input_raw/input_rawf dispatchers) ----
 * Only UBX is linked in. These keep the format dispatcher resolvable without
 * dragging in ten more decoders. 2.4.3 dropped gw10 and QZSS LEX, added
 * Septentrio SBF, and gave RT17 its own init/free because receiver state now
 * lives in raw->rcv_data.                                                   */
#define STUB_RCV(name) \
    extern int input_##name(raw_t *raw, uint8_t data) \
        { (void)raw; (void)data; return 0; } \
    extern int input_##name##f(raw_t *raw, FILE *fp) \
        { (void)raw; (void)fp; return -2; }

STUB_RCV(oem4)
STUB_RCV(oem3)
STUB_RCV(ss2)
STUB_RCV(cres)
STUB_RCV(stq)
STUB_RCV(javad)
STUB_RCV(nvs)
STUB_RCV(bnx)
STUB_RCV(rt17)
STUB_RCV(sbf)

/* init_raw()/free_raw() call these unconditionally in 2.4.3. */
extern int  init_rt17(raw_t *raw) { (void)raw; return 1; }
extern void free_rt17(raw_t *raw) { (void)raw; }

/* --- RTCM --------------------------------------------------------------- */
#ifdef RTKPICO_NO_RTCM
/* RTCM3 base support compiled out entirely: the base link must be UBX. */
extern int  input_rtcm3(rtcm_t *rtcm, unsigned char data)
    { (void)rtcm; (void)data; return 0; }
extern int  init_rtcm(rtcm_t *rtcm)  { (void)rtcm; return 0; }
extern void free_rtcm(rtcm_t *rtcm)  { (void)rtcm; }
#else
/* rtcm.c's dispatchers reference the RTCM2 decoder and the RTCM3 *encoder*.
 * The firmware only ever consumes RTCM3, so neither is linked in. The host
 * test build does link the encoder (RTKPICO_HAVE_RTCM3E), so the simulated
 * base can send MSM7; there the stub would shadow it and every message would
 * silently encode to nothing. */
extern int decode_rtcm2(rtcm_t *rtcm) { (void)rtcm; return -1; }
#ifndef RTKPICO_HAVE_RTCM3E
extern int encode_rtcm3(rtcm_t *rtcm, int type, int subtype, int sync)
    { (void)rtcm; (void)type; (void)subtype; (void)sync; return 0; }
#endif
#endif
