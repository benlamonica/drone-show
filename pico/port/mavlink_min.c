/*------------------------------------------------------------------------------
* mavlink_min.c : minimal MAVLink 2 framer (see mavlink_min.h)
*-----------------------------------------------------------------------------*/
#include "mavlink_min.h"
#include <string.h>

/* --- CRC-16/MCRF4XX, the MAVLink checksum -------------------------------- */
static void crc_acc(uint8_t d, uint16_t *crc)
{
    uint8_t t = d ^ (uint8_t)(*crc & 0xFF);
    t ^= (uint8_t)(t << 4);
    *crc = (uint16_t)((*crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3)
                      ^ ((uint16_t)t >> 4));
}

/* --- parser ---------------------------------------------------------------- */
void mav_parser_reset(mav_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

bool mav_parse_byte(mav_parser_t *p, uint8_t b,
                    const uint8_t **frame, uint16_t *len)
{
    if (!p->in_frame) {
        if (b != MAVLINK_STX_V1 && b != MAVLINK_STX_V2) { p->resyncs++; return false; }
        p->in_frame = true;
        p->stx = b;
        p->got = 0;
        p->need = 0;
        p->buf[p->got++] = b;
        return false;
    }

    if (p->got < MAVLINK_MAX_FRAME) p->buf[p->got++] = b;
    else { p->in_frame = false; return false; }      /* runaway: resynchronise */

    /* Once the length byte is in we know the whole frame size. v1 is
     * 6 header + payload + 2 CRC; v2 is 10 + payload + 2 CRC, plus a 13 byte
     * signature when incompat_flags bit 0 is set. */
    if (p->need == 0 && p->got >= 2) {
        uint16_t payload = p->buf[1];
        if (p->stx == MAVLINK_STX_V1) {
            p->need = 6 + payload + 2;
        } else if (p->got >= 3) {
            p->need = 10 + payload + 2 + ((p->buf[2] & 0x01) ? 13 : 0);
        }
    }

    if (p->need && p->got >= p->need) {
        *frame = p->buf;
        *len   = p->need;
        p->in_frame = false;
        p->frames++;
        return true;
    }
    return false;
}

uint32_t mav_frame_msgid(const uint8_t *f)
{
    if (f[0] == MAVLINK_STX_V1) return f[5];
    return (uint32_t)f[7] | ((uint32_t)f[8] << 8) | ((uint32_t)f[9] << 16);
}
uint8_t mav_frame_sysid(const uint8_t *f)
{
    return f[0] == MAVLINK_STX_V1 ? f[3] : f[5];
}
uint8_t mav_frame_compid(const uint8_t *f)
{
    return f[0] == MAVLINK_STX_V1 ? f[4] : f[6];
}

/* --- transmit -------------------------------------------------------------- */
uint16_t mav_pack(mav_tx_t *tx, uint8_t *out, uint32_t msgid, uint8_t crc_extra,
                  const uint8_t *payload, uint16_t payload_len)
{
    uint16_t crc = 0xFFFF, i, n;

    /* v2 allows trailing zero bytes to be dropped; receivers zero-fill. At
     * least one payload byte must remain. */
    n = payload_len;
    while (n > 1 && payload[n - 1] == 0) n--;
    if (payload_len == 0) n = 0;

    out[0] = MAVLINK_STX_V2;
    out[1] = (uint8_t)n;
    out[2] = 0;                 /* incompat_flags: unsigned */
    out[3] = 0;                 /* compat_flags               */
    out[4] = tx->seq++;
    out[5] = tx->sysid;
    out[6] = tx->compid;
    out[7] = (uint8_t)(msgid & 0xFF);
    out[8] = (uint8_t)((msgid >> 8) & 0xFF);
    out[9] = (uint8_t)((msgid >> 16) & 0xFF);
    memcpy(out + 10, payload, n);

    /* CRC covers everything from the length byte through the payload, then
     * the message-specific CRC_EXTRA. */
    for (i = 1; i < 10 + n; i++) crc_acc(out[i], &crc);
    crc_acc(crc_extra, &crc);

    out[10 + n] = (uint8_t)(crc & 0xFF);
    out[11 + n] = (uint8_t)(crc >> 8);
    return (uint16_t)(12 + n);
}

/* --- payload builders ------------------------------------------------------
 * Offsets are MAVLink wire order (fields sorted by size, extensions last),
 * taken from pymavlink; see tools/mavlink_wire.py to regenerate. */
static void put_u16(uint8_t *p, uint16_t v) { memcpy(p, &v, 2); }
static void put_u32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }
static void put_i32(uint8_t *p, int32_t  v) { memcpy(p, &v, 4); }
static void put_u64(uint8_t *p, uint64_t v) { memcpy(p, &v, 8); }
static void put_f32(uint8_t *p, float    v) { memcpy(p, &v, 4); }

uint16_t mav_build_gps_input(uint8_t *b, const mav_gps_input_t *g)
{
    memset(b, 0, 65);
    put_u64(b +  0, 0);                  /* time_usec: 0 = use time_week_ms */
    put_u32(b +  8, g->time_week_ms);
    put_i32(b + 12, g->lat_e7);
    put_i32(b + 16, g->lon_e7);
    put_f32(b + 20, g->alt_msl);
    put_f32(b + 24, g->hdop);
    put_f32(b + 28, g->vdop);
    put_f32(b + 32, g->vn);
    put_f32(b + 36, g->ve);
    put_f32(b + 40, g->vd);
    put_f32(b + 44, g->speed_acc);
    put_f32(b + 48, g->horiz_acc);
    put_f32(b + 52, g->vert_acc);
    put_u16(b + 56, g->ignore_flags);
    put_u16(b + 58, g->time_week);
    b[60] = g->gps_id;
    b[61] = g->fix_type;
    b[62] = g->sats;
    put_u16(b + 63, 0);                  /* yaw extension: 0 = not available */
    return 65;
}

uint16_t mav_build_heartbeat(uint8_t *b, uint8_t type, uint8_t autopilot,
                             uint8_t base_mode, uint32_t custom_mode,
                             uint8_t system_status)
{
    memset(b, 0, 9);
    put_u32(b + 0, custom_mode);
    b[4] = type;
    b[5] = autopilot;
    b[6] = base_mode;
    b[7] = system_status;
    b[8] = 3;                            /* MAVLink version */
    return 9;
}
