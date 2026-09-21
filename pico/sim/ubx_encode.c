/*------------------------------------------------------------------------------
* ubx_encode.c : UBX message generator (see ubx_encode.h)
*
* Bit layouts are the exact inverse of RTKLIB's decoders:
*   RXM-RAWX  : ublox.c decode_rxmrawx()
*   RXM-SFRBX : ublox.c decode_rxmsfrbx() -> decode_nav() -> save_subfrm()
*   LNAV 1..3 : rcvraw.c decode_subfrm1/2/3()
*   GLO string: ublox.c decode_gnav() -> rcvraw.c decode_glostr()
*-----------------------------------------------------------------------------*/
#include "ubx_encode.h"
#include <string.h>

#define UBXSYNC1 0xB5
#define UBXSYNC2 0x62

/* --- little-endian field writers (mirror ublox.c U1/U2/U4/R4/R8) ---------- */
static void setU1(unsigned char *p, uint8_t  v) { *p=v; }
static void setI1(unsigned char *p, int8_t   v) { *p=(unsigned char)v; }
static void setU2(unsigned char *p, uint16_t v) { memcpy(p,&v,2); }
static void setU4(unsigned char *p, uint32_t v) { memcpy(p,&v,4); }
static void setR4(unsigned char *p, float    v) { memcpy(p,&v,4); }
static void setR8(unsigned char *p, double   v) { memcpy(p,&v,8); }

/* Header + checksum. `paylen` is the payload length; returns total length. */
static int ubx_finish(unsigned char *buf, int cls, int id, int paylen)
{
    unsigned char cka=0,ckb=0;
    int i,len=paylen+8;
    buf[0]=UBXSYNC1; buf[1]=UBXSYNC2;
    buf[2]=(unsigned char)cls; buf[3]=(unsigned char)id;
    setU2(buf+4,(uint16_t)paylen);
    for (i=2;i<len-2;i++) { cka+=buf[i]; ckb+=cka; }
    buf[len-2]=cka; buf[len-1]=ckb;
    return len;
}

/* RTKLIB's ubx_sys() indicator, inverted. */
static int sys2gnssid(int sys)
{
    switch (sys) {
        case SYS_GPS: return 0;
        case SYS_SBS: return 1;
        case SYS_GAL: return 2;
        case SYS_CMP: return 3;
        case SYS_QZS: return 5;
        case SYS_GLO: return 6;
    }
    return -1;
}

int ubx_encode_rawx(unsigned char *buf, int size, gtime_t time,
                    const obsd_t *obs, int n, const ubx_meas_aux_t *aux)
{
    unsigned char *p;
    double tow;
    int i,week,nsat=0,paylen;

    if (n<0) n=0;
    paylen=16+32*n;
    if (size<paylen+8) return 0;

    tow=time2gpst(time,&week);

    memset(buf,0,paylen+8);
    p=buf+6;
    setR8(p   ,tow);
    setU2(p+ 8,(uint16_t)week);
    setI1(p+10,(int8_t)18);        /* leapS */
    /* p+11 numMeas is filled once we know how many satellites we emitted */
    setU1(p+12,0x01);              /* recStat: leap seconds known           */
    setU1(p+13,0x01);              /* version                               */

    for (i=0;i<n;i++) {
        unsigned char *q=buf+6+16+32*nsat;
        int sys,prn,gnssid;
        ubx_meas_aux_t a;

        if (!(sys=satsys(obs[i].sat,&prn))) continue;
        if ((gnssid=sys2gnssid(sys))<0) continue;

        if (aux) a=aux[i];
        else {
            a.locktime_ms=64000; a.cno=45; a.pr_stdev=5; a.cp_stdev=1;
            a.do_stdev=1; a.trk_stat=0x07; a.half_cyc_sub=0;
        }
        if (sys==SYS_QZS) prn-=192;   /* decode_rxmsfrbx() adds it back */

        setR8(q   ,obs[i].P[0]);
        setR8(q+ 8,obs[i].L[0]);
        setR4(q+16,obs[i].D[0]);
        setU1(q+20,(uint8_t)gnssid);
        setU1(q+21,(uint8_t)prn);
        setU1(q+22,0);                                  /* sigId            */
        setU1(q+23,7);                                  /* freqId (fcn+7)   */
        setU2(q+24,a.locktime_ms);
        setU1(q+26,a.cno);
        setU1(q+27,a.pr_stdev);
        setU1(q+28,(uint8_t)(a.cp_stdev&0x0F));
        setU1(q+29,a.do_stdev);
        setU1(q+30,(uint8_t)(a.trk_stat|(a.half_cyc_sub?0x08:0)));
        setU1(q+31,0);
        nsat++;
    }
    paylen=16+32*nsat;
    setU1(buf+6+11,(uint8_t)nsat);
    return ubx_finish(buf,0x02,0x15,paylen);
}

/* --- GPS LNAV subframe packing -------------------------------------------- */

/* Inverse of getbits(): store a signed value in `len` bits. */
static void setbits_(unsigned char *buff, int pos, int len, int32_t v)
{
    setbitu(buff,pos,len,(uint32_t)v&(len>=32?0xFFFFFFFFu:((1u<<len)-1)));
}
/* Round to nearest, away from zero: matches how the control segment quantises. */
static int32_t q(double v, double lsb)
{
    double x=v/lsb;
    return (int32_t)(x>=0.0?x+0.5:x-0.5);
}
static uint32_t qu(double v, double lsb)
{
    return (uint32_t)(v/lsb+0.5);
}

void gps_lnav_subframe(unsigned char *sf, int id, const eph_t *eph, double tow)
{
    int i;
    double toc,toes;
    int week;

    memset(sf,0,30);

    /* word 1: TLM preamble. Not inspected by RTKLIB, but keep it realistic. */
    setbitu(sf,0,8,0x8B);
    /* word 2 (HOW): TOW-count at bit 24 (17 b, 6 s units), subframe id at 43 */
    setbitu(sf,24,17,(uint32_t)(tow/6.0));
    setbitu(sf,43,3,(uint32_t)id);

    toc =time2gpst(eph->toc,&week);
    toes=eph->toes;

    if (id==1) {
        setbitu (sf, 48,10,(uint32_t)(eph->week%1024));
        setbitu (sf, 58, 2,(uint32_t)eph->code);
        setbitu (sf, 60, 4,(uint32_t)eph->sva);
        setbitu (sf, 64, 6,(uint32_t)eph->svh);
        setbitu (sf, 70, 2,(uint32_t)((eph->iodc>>8)&0x3));
        setbitu (sf, 72, 1,(uint32_t)eph->flag);
        /* bits 73..159 are reserved/spare in subframe 1 */
        setbits_(sf,160, 8,q(eph->tgd[0],P2_31));
        setbitu (sf,168, 8,(uint32_t)(eph->iodc&0xFF));
        setbitu (sf,176,16,qu(toc,16.0));
        setbits_(sf,192, 8,q(eph->f2,P2_55));
        setbits_(sf,200,16,q(eph->f1,P2_43));
        setbits_(sf,216,22,q(eph->f0,P2_31));
    }
    else if (id==2) {
        setbitu (sf, 48, 8,(uint32_t)eph->iode);
        setbits_(sf, 56,16,q(eph->crs ,P2_5));
        setbits_(sf, 72,16,q(eph->deln,P2_43*SC2RAD));
        setbits_(sf, 88,32,q(eph->M0  ,P2_31*SC2RAD));
        setbits_(sf,120,16,q(eph->cuc ,P2_29));
        setbitu (sf,136,32,qu(eph->e  ,P2_33));
        setbits_(sf,168,16,q(eph->cus ,P2_29));
        setbitu (sf,184,32,qu(sqrt(eph->A),P2_19));
        setbitu (sf,216,16,qu(toes,16.0));
        setbitu (sf,232, 1,eph->fit>0.0?0:1);  /* 0 => 4 h fit interval */
    }
    else if (id==3) {
        setbits_(sf, 48,16,q(eph->cic ,P2_29));
        setbits_(sf, 64,32,q(eph->OMG0,P2_31*SC2RAD));
        setbits_(sf, 96,16,q(eph->cis ,P2_29));
        setbits_(sf,112,32,q(eph->i0  ,P2_31*SC2RAD));
        setbits_(sf,144,16,q(eph->crc ,P2_5));
        setbits_(sf,160,32,q(eph->omg ,P2_31*SC2RAD));
        setbits_(sf,192,24,q(eph->OMGd,P2_43*SC2RAD));
        /* decode_subfrm3() rejects the frame unless this matches subframe 2's
         * IODE and the low byte of IODC. */
        setbitu (sf,216, 8,(uint32_t)eph->iode);
        setbits_(sf,224,14,q(eph->idot,P2_43*SC2RAD));
    }
    (void)i;
}

int ubx_encode_sfrbx_gps(unsigned char *buf, int size, int prn, int id,
                         const eph_t *eph, double tow)
{
    unsigned char sf[30];
    int i,paylen=8+4*10;

    if (size<paylen+8||id<1||id>3) return 0;

    gps_lnav_subframe(sf,id,eph,tow);

    memset(buf,0,paylen+8);
    setU1(buf+6+0,0);                    /* gnssId: GPS                     */
    setU1(buf+6+1,(uint8_t)prn);         /* svId                            */
    setU1(buf+6+2,0);                    /* reserved                        */
    setU1(buf+6+3,0);                    /* freqId                          */
    setU1(buf+6+4,10);                   /* numWords                        */
    setU1(buf+6+5,0);                    /* chn                             */
    setU1(buf+6+6,2);                    /* version                         */
    setU1(buf+6+7,0);                    /* reserved                        */

    /* decode_nav() reads each word as U4(p)>>6, i.e. the 24 data bits sit at
     * bits 29..6 with the 6 parity bits zeroed. */
    for (i=0;i<10;i++) {
        uint32_t w=getbitu(sf,i*24,24);
        setU4(buf+6+8+4*i,w<<6);
    }
    return ubx_finish(buf,0x02,0x13,paylen);
}

/* --- GLONASS navigation string -------------------------------------------- */

int ubx_encode_sfrbx_glo(unsigned char *buf, int size, int slot, int m, int tb)
{
    unsigned char s[16]={0};
    int i,paylen=8+4*4;
    uint32_t c;

    if (size<paylen+8||m<1||m>4) return 0;

    setbitu(s,1,4,(uint32_t)m);                 /* string number           */
    if (m==2) setbitu(s, 9,7,(uint32_t)tb);
    if (m==4) setbitu(s,70,5,(uint32_t)slot);
    /* Check bits 77..84: search rather than re-derive the code, so the string
     * is valid by the decoder's own definition. */
    for (c=0;c<256;c++) {
        setbitu(s,77,8,c);
        if (test_glostr(s)) break;
    }
    s[12]=0x12; s[13]=0x01;                     /* frame id, same for all SVs */

    memset(buf,0,paylen+8);
    setU1(buf+6+0,6);                    /* gnssId: GLONASS                 */
    setU1(buf+6+1,(uint8_t)slot);        /* svId: slot                      */
    setU1(buf+6+3,7);                    /* freqId: fcn+7                   */
    setU1(buf+6+4,4);                    /* numWords                        */
    setU1(buf+6+6,2);                    /* version                         */

    /* decode_gnav() reverses each little-endian word into big-endian bytes */
    for (i=0;i<4;i++) {
        setU4(buf+6+8+4*i,((uint32_t)s[4*i]<<24)|((uint32_t)s[4*i+1]<<16)|
                          ((uint32_t)s[4*i+2]<<8)|s[4*i+3]);
    }
    return ubx_finish(buf,0x02,0x13,paylen);
}
