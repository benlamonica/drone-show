/*------------------------------------------------------------------------------
* test_ubx.c : UBX encode/decode round-trip and framing robustness
*
* Validates that the simulator produces bytes RTKLIB's own decoders accept, and
* that everything RTKLIB needs survives the LNAV bit packing. If these pass,
* any failure further up the stack is in the filter, not the wire format.
*-----------------------------------------------------------------------------*/
#include "gnss_sim.h"
#include "ubx_encode.h"
#include "test_util.h"
#include <string.h>
#include <stdlib.h>


/* Feed a buffer byte-by-byte, returning the last non-zero decoder status. */
static int feed(raw_t *raw, const unsigned char *buf, int n)
{
    int i,r,last=0;
    for (i=0;i<n;i++) if ((r=input_ubx(raw,buf[i]))) last=r;
    return last;
}

static void test_rawx_roundtrip(void)
{
    sim_constellation_t con;
    sim_station_t st;
    sim_errmodel_t err;
    obsd_t obs[SIM_MAXSAT];
    ubx_meas_aux_t aux[SIM_MAXSAT];
    unsigned char buf[UBX_MAX_MSG];
    double llh[3]={35.68,139.76,40.0};
    gtime_t t;
    raw_t raw;
    int n,len,i,j,ret,matched=0;

    TEST_BEGIN("UBX-RXM-RAWX round trip");

    sim_default_errmodel(&err);
    /* 24 GPS, not 12+10: the simulator's second constellation is Galileo and is
     * compiled out of the M8P build (ENAGAL off, ENAGLO on), which left only 4
     * satellites above the mask and failed the check below for a reason that
     * had nothing to do with the round trip under test. GPS alone carries the
     * count so this stays honest whatever constellations are enabled.
     * NOTE: the simulator cannot model GLONASS -- that needs geph_t PZ-90 state
     * vectors rather than Keplerian eph_t -- so the two-constellation decode
     * path is currently not covered by any test in the shipping config. */
    sim_make_constellation(&con,2300,302400.0,24,10);
    sim_station_init(&st,llh,777);
    t=gpst2time(2300,302400.0);

    n=sim_observe(&con,&st,&err,t,llh,obs,aux,SIM_MAXSAT);
    CHECK(n>=6,"simulator produced %d satellites above the mask",n);

    len=ubx_encode_rawx(buf,sizeof(buf),t,obs,n,aux);
    CHECK(len==24+32*n,"encoded length %d == 24+32*%d",len,n);
    CHECK(buf[0]==0xB5&&buf[1]==0x62,"sync bytes B5 62");
    CHECK(buf[2]==0x02&&buf[3]==0x15,"class/id 02 15 (RXM-RAWX)");

    CHECK(init_raw(&raw,STRFMT_UBX)==1,"init_raw");
    ret=feed(&raw,buf,len);
    CHECK(ret==1,"decode_rxmrawx accepted the message (ret=%d)",ret);
    CHECK(raw.obs.n==n,"decoded %d observations, expected %d",raw.obs.n,n);
    CHECK_NEAR(timediff(raw.time,t),0.0,1E-6,"epoch time preserved");

    /* obs order is preserved by the encoder, but match on satellite anyway */
    for (i=0;i<raw.obs.n;i++) {
        for (j=0;j<n;j++) if (obs[j].sat==raw.obs.data[i].sat) break;
        if (j>=n) continue;
        matched++;
        if (fabs(raw.obs.data[i].P[0]-obs[j].P[0])>1E-6||
            fabs(raw.obs.data[i].L[0]-obs[j].L[0])>1E-6) {
            printf("  sat %d: P %.6f vs %.6f  L %.6f vs %.6f\n",obs[j].sat,
                   raw.obs.data[i].P[0],obs[j].P[0],
                   raw.obs.data[i].L[0],obs[j].L[0]);
        }
    }
    CHECK(matched==n,"all %d satellites matched by number",n);

    /* R8 fields must survive exactly; SNR is quantised to 0.25 dBHz */
    {
        int exact_p=1,exact_l=1,snr_ok=1,lli_ok=1;
        for (i=0;i<raw.obs.n;i++) {
            for (j=0;j<n;j++) if (obs[j].sat==raw.obs.data[i].sat) break;
            if (j>=n) continue;
            if (raw.obs.data[i].P[0]!=obs[j].P[0]) exact_p=0;
            if (raw.obs.data[i].L[0]!=obs[j].L[0]) exact_l=0;
            /* SNR_UNIT is 0.001 dBHz in 2.4.3; allow 1 dBHz of rounding
             * through the UBX cno field, which is whole dBHz. */
            if (abs((int)raw.obs.data[i].SNR[0]-(int)obs[j].SNR[0])>1000) snr_ok=0;
            /* lockTime is 0 on the first measurement of a satellite, so
             * decode_rxmrawx() must flag a slip: that is correct behaviour. */
            if (!(raw.obs.data[i].LLI[0]&LLI_SLIP)) lli_ok=0;
        }
        CHECK(exact_p,"pseudoranges are bit-exact through R8");
        CHECK(exact_l,"carrier phases are bit-exact through R8");
        CHECK(snr_ok ,"SNR within 1 dBHz through the UBX cno field");
        CHECK(lli_ok ,"LLI_SLIP set on first lock (lockTime==0)");
    }
    free_raw(&raw);
}

/* Loss of carrier lock must surface as LLI_SLIP, which is what drives the
 * filter to reset that ambiguity. */
static void test_slip_flag(void)
{
    sim_constellation_t con;
    sim_station_t st;
    sim_errmodel_t err;
    obsd_t obs[SIM_MAXSAT];
    ubx_meas_aux_t aux[SIM_MAXSAT];
    unsigned char buf[UBX_MAX_MSG];
    double llh[3]={35.68,139.76,40.0};
    gtime_t t;
    raw_t raw;
    int n,len,i,slip_seen=0,slip_sat;

    TEST_BEGIN("cycle slip -> LLI_SLIP");

    sim_default_errmodel(&err);
    sim_make_constellation(&con,2300,302400.0,12,0);
    sim_station_init(&st,llh,4242);
    init_raw(&raw,STRFMT_UBX);
    t=gpst2time(2300,302400.0);

    /* two clean epochs to establish lock */
    for (i=0;i<2;i++) {
        n=sim_observe(&con,&st,&err,t,llh,obs,aux,SIM_MAXSAT);
        len=ubx_encode_rawx(buf,sizeof(buf),t,obs,n,aux);
        feed(&raw,buf,len);
        t=timeadd(t,1.0);
    }
    /* by the second epoch lock is established, so no satellite should be
     * flagged any more */
    {
        int clean=1;
        for (i=0;i<raw.obs.n;i++) if (raw.obs.data[i].LLI[0]&LLI_SLIP) clean=0;
        CHECK(clean,"no slip flags once carrier lock is continuous");
    }

    slip_sat=obs[0].sat;
    sim_force_slip(&st,slip_sat);

    n=sim_observe(&con,&st,&err,t,llh,obs,aux,SIM_MAXSAT);
    len=ubx_encode_rawx(buf,sizeof(buf),t,obs,n,aux);
    feed(&raw,buf,len);

    for (i=0;i<raw.obs.n;i++) {
        if (raw.obs.data[i].sat==slip_sat&&(raw.obs.data[i].LLI[0]&LLI_SLIP)) {
            slip_seen=1;
        }
    }
    CHECK(slip_seen,"LLI_SLIP raised on sat %d after lock reset",slip_sat);
    free_raw(&raw);
}

static void test_lnav_roundtrip(void)
{
    sim_constellation_t con;
    unsigned char frm[90];      /* subframes 1..3, 30 bytes each */
    eph_t out={0};
    const eph_t *in;
    int id,r;

    TEST_BEGIN("GPS LNAV subframe encode -> decode_frame()");

    sim_make_constellation(&con,2300,302400.0,12,0);
    in=&con.eph[0];

    /* 2.4.3 changed decode_frame(): it now consumes all three subframes at
     * once and returns 1/0, where 2.4.2 took one subframe and returned its
     * id. It also dropped the trailing `int *leaps` argument. */
    for (id=1;id<=3;id++) gps_lnav_subframe(frm+(id-1)*30,id,in,302400.0);
    r=decode_frame(frm,&out,NULL,NULL,NULL);
    CHECK(r==1,"decode_frame() accepted subframes 1-3 (got %d)",r);

    /* Tolerances are one quantisation step of each LNAV field. */
    CHECK_NEAR(out.A   ,in->A   ,2E-2 ,"A     (semi-major axis, m)");
    CHECK_NEAR(out.e   ,in->e   ,1E-9 ,"e     (eccentricity)");
    CHECK_NEAR(out.i0  ,in->i0  ,1E-8 ,"i0    (rad)");
    CHECK_NEAR(out.OMG0,in->OMG0,1E-8 ,"OMG0  (rad)");
    CHECK_NEAR(out.omg ,in->omg ,1E-8 ,"omg   (rad)");
    CHECK_NEAR(out.M0  ,in->M0  ,1E-8 ,"M0    (rad)");
    CHECK_NEAR(out.deln,in->deln,1E-12,"deln  (rad/s)");
    CHECK_NEAR(out.OMGd,in->OMGd,1E-12,"OMGd  (rad/s)");
    CHECK_NEAR(out.idot,in->idot,1E-12,"idot  (rad/s)");
    CHECK_NEAR(out.crs ,in->crs ,2E-2 ,"crs   (m)");
    CHECK_NEAR(out.crc ,in->crc ,2E-2 ,"crc   (m)");
    CHECK_NEAR(out.cuc ,in->cuc ,1E-9 ,"cuc   (rad)");
    CHECK_NEAR(out.cus ,in->cus ,1E-9 ,"cus   (rad)");
    CHECK_NEAR(out.cic ,in->cic ,1E-9 ,"cic   (rad)");
    CHECK_NEAR(out.cis ,in->cis ,1E-9 ,"cis   (rad)");
    CHECK_NEAR(out.f0  ,in->f0  ,1E-9 ,"f0    (s)");
    CHECK_NEAR(out.f1  ,in->f1  ,1E-12,"f1    (s/s)");
    CHECK_NEAR(out.tgd[0],in->tgd[0],1E-9,"tgd   (s)");
    CHECK_NEAR(out.toes,in->toes,1E-6 ,"toes  (s)");
    CHECK(out.iode==in->iode,"iode %d preserved",in->iode);
    CHECK(out.week==in->week,"week %d resolved from 10 bits (got %d)",
          in->week,out.week);
    CHECK(out.svh ==in->svh ,"svh preserved");
}

static void test_sfrbx_ephemeris(void)
{
    sim_constellation_t con;
    unsigned char buf[UBX_MAX_MSG];
    raw_t raw;
    int id,len,ret=0,prn,sat;
    double dpos;

    TEST_BEGIN("UBX-RXM-SFRBX -> ephemeris in nav_t");

    sim_make_constellation(&con,2300,302400.0,12,0);
    sat=con.sat[0];
    satsys(sat,&prn);

    /* adjgpsweek() resolves the 10-bit week against the wall clock */
    timeset(gpst2utc(gpst2time(2300,302400.0)));

    CHECK(init_raw(&raw,STRFMT_UBX)==1,"init_raw");
    for (id=1;id<=3;id++) {
        len=ubx_encode_sfrbx_gps(buf,sizeof(buf),prn,id,&con.eph[0],302400.0);
        CHECK(len==8+8+40,"SFRBX subframe %d length %d",id,len);
        ret=feed(&raw,buf,len);
    }
    CHECK(ret==2,"decode_ephem() accepted the ephemeris (ret=%d)",ret);
    CHECK(raw.ephsat==sat,"ephsat==%d (got %d)",sat,raw.ephsat);
    CHECK(raw.nav.eph[sat-1].sat==sat,"ephemeris stored for sat %d",sat);
    CHECK(raw.nav.eph[sat-1].iode==con.eph[0].iode,"iode matches");

    /* The real check: does the decoded ephemeris put the satellite in the
     * same place as the truth ephemeris? */
    {
        double rs1[6]={0},rs2[6]={0},dts1[2],dts2[2],var;
        gtime_t t=gpst2time(2300,302400.0);
        eph2pos(t,&con.eph[0],rs1,dts1,&var);
        eph2pos(t,&raw.nav.eph[sat-1],rs2,dts2,&var);
        dpos=sqrt(pow(rs1[0]-rs2[0],2)+pow(rs1[1]-rs2[1],2)+pow(rs1[2]-rs2[2],2));
        CHECK(dpos<0.05,"satellite position agrees within %.4f m (< 5 cm)",dpos);
        CHECK_NEAR(dts1[0],dts2[0],1E-9,"satellite clock (s)");
    }
    free_raw(&raw);
}

/* The receiver link is a UART: the decoder must resynchronise after noise,
 * partial messages and corrupted checksums without losing the next good one. */
static void test_framing_robustness(void)
{
    sim_constellation_t con;
    sim_station_t st;
    sim_errmodel_t err;
    obsd_t obs[SIM_MAXSAT];
    ubx_meas_aux_t aux[SIM_MAXSAT];
    unsigned char buf[UBX_MAX_MSG],junk[64];
    double llh[3]={35.68,139.76,40.0};
    gtime_t t;
    raw_t raw;
    int n,len,i,ret;

    TEST_BEGIN("UBX framing robustness");

    sim_default_errmodel(&err);
    sim_make_constellation(&con,2300,302400.0,12,0);
    sim_station_init(&st,llh,99);
    t=gpst2time(2300,302400.0);
    init_raw(&raw,STRFMT_UBX);

    n=sim_observe(&con,&st,&err,t,llh,obs,aux,SIM_MAXSAT);
    len=ubx_encode_rawx(buf,sizeof(buf),t,obs,n,aux);

    /* leading garbage, including bytes that look like a partial sync */
    for (i=0;i<(int)sizeof(junk);i++) junk[i]=(unsigned char)(i*37+1);
    junk[10]=0xB5; junk[11]=0x00;      /* false sync */
    junk[40]=0xB5; junk[41]=0x62;      /* sync then garbage length */
    feed(&raw,junk,sizeof(junk));
    ret=feed(&raw,buf,len);
    CHECK(ret==1,"good message decoded after %zu bytes of junk",sizeof(junk));

    /* corrupted checksum must be rejected, and must not wedge the decoder */
    raw.obs.n=0;
    buf[len-1]^=0xFF;
    ret=feed(&raw,buf,len);
    CHECK(ret==-1,"checksum error reported (ret=%d)",ret);
    buf[len-1]^=0xFF;
    ret=feed(&raw,buf,len);
    CHECK(ret==1,"decoder recovered on the next good message");

    /* A truncated message leaves the framer mid-frame, so it swallows the
     * head of the next one while completing the bad frame. The guarantee is
     * that it resynchronises by the message after that. */
    raw.obs.n=0;
    feed(&raw,buf,len/2);
    ret=feed(&raw,buf,len);
    CHECK(ret!=1,"truncated frame consumes the next message (ret=%d)",ret);
    ret=feed(&raw,buf,len);
    CHECK(ret==1,"resynchronised one message after a truncation");

    free_raw(&raw);
}

#if NSATGLO>0
static void test_sfrbx_glonass_adjacent(void)
{
    unsigned char buf[UBX_MAX_MSG];
    raw_t raw;
    int m,k,len,ret[2]={0},sat[2];

    /* decode_gnav() keeps each satellite's frame-id at subfrm[sat-1]+150. With
     * MAXSUBFRMLEN trimmed to 150 that was the next satellite's string 1, so
     * adjacent slots -- routinely both in view -- corrupted each other and no
     * GLONASS ephemeris ever decoded. The receiver interleaves strings across
     * satellites exactly like this. */
    TEST_BEGIN("UBX-RXM-SFRBX GLONASS, adjacent slots");

    sat[0]=satno(SYS_GLO,1);
    sat[1]=satno(SYS_GLO,2);
    CHECK(init_raw(&raw,STRFMT_UBX)==1,"init_raw");
    raw.time=gpst2time(2300,302400.0);

    for (m=1;m<=4;m++) for (k=0;k<2;k++) {
        len=ubx_encode_sfrbx_glo(buf,sizeof(buf),k+1,m,1);
        if (m==1&&k==0) CHECK(len==8+8+16,"SFRBX length %d",len);
        ret[k]=feed(&raw,buf,len);
    }
    CHECK(ret[0]==2&&raw.nav.geph[0].sat==sat[0],
          "slot 1 ephemeris decoded (ret=%d sat=%d)",ret[0],raw.nav.geph[0].sat);
    CHECK(ret[1]==2&&raw.nav.geph[1].sat==sat[1],
          "slot 2 ephemeris decoded (ret=%d sat=%d)",ret[1],raw.nav.geph[1].sat);
    free_raw(&raw);
}
#endif

#ifdef RTKPICO_TARGET_SUITE
#define main test_ubx_main
#endif
int main(void)
{
    printf("RTKLIB Pico port - UBX layer tests\n");
    printf("MAXSAT=%d NFREQ=%d MAXOBS=%d\n",MAXSAT,NFREQ,MAXOBS);
    test_rawx_roundtrip();
    test_slip_flag();
    test_lnav_roundtrip();
    test_sfrbx_ephemeris();
#if NSATGLO>0
    test_sfrbx_glonass_adjacent();
#endif
    test_framing_robustness();
    TEST_SUMMARY();
    return g_tests_failed?1:0;
}
