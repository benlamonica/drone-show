/*------------------------------------------------------------------------------
* ntrip_client.c : Wi-Fi + NTRIP client (see ntrip_client.h)
*-----------------------------------------------------------------------------*/
#include "ntrip_client.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"

#define WIFI_JOIN_TIMEOUT_MS   20000   /* abandon one join attempt after this */
#define WIFI_JOIN_RETRY_MS      2000   /* pause before the next attempt        */
#define LINK_POLL_MS             250   /* how often we ask the driver          */
#define BACKOFF_MIN_MS          2000
#define BACKOFF_MAX_MS         30000
#define HDR_SCAN_MAX             512   /* give up looking for a header after this */

static ntrip_config_t  cfg_;
static ntrip_status_t  st_;
static ntrip_data_cb_t cb_;
static void           *cb_arg_;

static struct tcp_pcb *pcb_;
static uint32_t backoff_ms_   = BACKOFF_MIN_MS;
static uint32_t retry_at_ms_  = 0;
static uint32_t rssi_at_ms_   = 0;
static uint32_t link_at_ms_   = 0;
static uint32_t join_start_ms_ = 0;
static int      link_status_  = CYW43_LINK_DOWN;
static bool     header_done_;
static uint16_t header_seen_;
static char     header_buf_[HDR_SCAN_MAX + 1];

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

static void set_status(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(st_.status, sizeof(st_.status), fmt, ap);
    va_end(ap);
}

/* --- base64, for the Authorization header --------------------------------- */
static int b64_encode(const char *in, char *out, int outsz)
{
    static const char T[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int n = (int)strlen(in), i, o = 0;
    for (i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)(uint8_t)in[i] << 16;
        int rem = n - i;
        if (rem > 1) v |= (uint32_t)(uint8_t)in[i + 1] << 8;
        if (rem > 2) v |= (uint32_t)(uint8_t)in[i + 2];
        if (o + 4 >= outsz) return 0;
        out[o++] = T[(v >> 18) & 0x3F];
        out[o++] = T[(v >> 12) & 0x3F];
        out[o++] = rem > 1 ? T[(v >> 6) & 0x3F] : '=';
        out[o++] = rem > 2 ? T[v & 0x3F] : '=';
    }
    out[o] = '\0';
    return o;
}

/* --- connection teardown / retry scheduling ------------------------------- */
static void schedule_retry(const char *why)
{
    if (why) set_status("%s", why);
    st_.state = NTRIP_STATE_BACKOFF;
    retry_at_ms_ = now_ms() + backoff_ms_;
    backoff_ms_ *= 2;
    if (backoff_ms_ > BACKOFF_MAX_MS) backoff_ms_ = BACKOFF_MAX_MS;
}

/* `locked` says whether we are already inside the lwIP lock -- true when
 * called from an lwIP callback (which runs in the alarm IRQ), false when
 * called from the main loop. Taking the lock recursively from the callback
 * would deadlock; omitting it from the main loop corrupts lwIP state. */
static void close_conn_ex(const char *why, bool locked)
{
    if (pcb_) {
        struct tcp_pcb *p = pcb_;
        pcb_ = NULL;
        if (!locked) cyw43_arch_lwip_begin();
        tcp_arg(p, NULL);
        tcp_recv(p, NULL);
        tcp_err(p, NULL);
        tcp_poll(p, NULL, 0);
        if (tcp_close(p) != ERR_OK) tcp_abort(p);
        if (!locked) cyw43_arch_lwip_end();
    }
    schedule_retry(why);
}
/* from lwIP callback context */
static void close_conn(const char *why) { close_conn_ex(why, true); }

/* --- response handling ----------------------------------------------------
 * Decide, from the first bytes the far end sends, whether we are talking to a
 * caster (header then RTCM) or a plain TCP server (RTCM straight away), and
 * find where the payload starts. */
static int header_offset(const char *buf, uint16_t len)
{
    const char *p;
    if (len >= 1 && (uint8_t)buf[0] == 0xD3) return 0;      /* raw RTCM3 */

    /* NTRIP v1 replies "ICY 200 OK" and then a CRLF. Whether a *blank* line
     * follows is caster-dependent: str2str sends one, others go straight into
     * RTCM3. Handle the status line explicitly and consume however many CRLFs
     * follow it, because the searches below cannot help here -- RTCM3 payload
     * contains no blank line, and it contains NUL bytes, so strstr() stops at
     * the first one and would never find a terminator that came after it.
     * Getting this wrong looks like a caster fault: the connection succeeds,
     * 512 bytes of good corrections are scanned, discarded, and retried. */
    if (len >= 10 && !strncmp(buf, "ICY 200 OK", 10)) {
        uint16_t i = 10;
        while ((uint16_t)(i + 1) < len && buf[i] == '\r' && buf[i + 1] == '\n') i += 2;
        if (i > 10) return (int)i;
        return -1;                                          /* CRLF not in yet */
    }

    if ((p = strstr(buf, "\r\n\r\n"))) return (int)(p - buf) + 4;
    if ((p = strstr(buf, "\n\n")))     return (int)(p - buf) + 2;
    return -1;                                              /* need more */
}

/* A caster answers with its source table when the mountpoint is unknown.
 * Surface the names it does offer -- that is almost always the fix. */
static void report_sourcetable(const char *buf)
{
    const char *p = buf;
    char names[64] = "";
    int n = 0;
    while ((p = strstr(p, "STR;")) && n < (int)sizeof(names) - 12) {
        const char *s = p + 4, *e = strchr(s, ';');
        if (!e) break;
        int l = (int)(e - s);
        if (l > 0 && l < 20) {
            n += snprintf(names + n, sizeof(names) - n, "%s%.*s",
                          n ? "," : "", l, s);
        }
        p = e;
    }
    set_status("mountpoint '%s' rejected; caster offers: %s",
               cfg_.mountpoint, n ? names : "(none)");
}

static void handle_header(const char *buf, uint16_t len)
{
    if (strstr(buf, "SOURCETABLE")) {
        report_sourcetable(buf);
        return;
    }
    if (strstr(buf, "ICY 200") || strstr(buf, "200 OK")) {
        set_status("caster accepted /%s", cfg_.mountpoint);
        return;
    }
    if (strstr(buf, "401")) {
        set_status("caster rejected credentials (401)");
        return;
    }
    set_status("unexpected caster reply: %.40s", buf);
    (void)len;
}

/* --- lwIP callbacks (alarm IRQ context under threadsafe_background) -------- */
static err_t on_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    struct pbuf *q;
    (void)arg;

    if (!p) { close_conn("caster closed the connection"); return ERR_OK; }
    if (err != ERR_OK) { pbuf_free(p); close_conn("recv error"); return ERR_OK; }

    for (q = p; q; q = q->next) {
        const uint8_t *d = (const uint8_t *)q->payload;
        uint16_t n = q->len, off = 0;

        if (!header_done_) {
            /* Accumulate until we can locate the end of the header (or decide
             * there is not one). */
            uint16_t take = n;
            if (header_seen_ + take > HDR_SCAN_MAX) take = HDR_SCAN_MAX - header_seen_;
            memcpy(header_buf_ + header_seen_, d, take);
            header_seen_ += take;
            header_buf_[header_seen_] = '\0';

            int ofs = header_offset(header_buf_, header_seen_);
            if (ofs < 0) {
                if (header_seen_ >= HDR_SCAN_MAX) {
                    /* No header we recognise. Before dropping what may be a
                     * perfectly good stream -- a reconnect that landed
                     * mid-message, or a caster greeting we do not know --
                     * look for an RTCM3 sync byte and start there. Closing
                     * costs a full reconnect cycle and loses corrections;
                     * resynchronising costs at most one framing error. */
                    uint16_t i;
                    for (i = 0; i < header_seen_; i++) {
                        if ((uint8_t)header_buf_[i] == 0xD3) break;
                    }
                    if (i < header_seen_) {
                        ofs = (int)i;
                        set_status("unrecognised header; resynchronised on RTCM3");
                    } else {
                        pbuf_free(p);
                        close_conn("no usable response header");
                        return ERR_OK;
                    }
                } else {
                    continue;                   /* wait for more */
                }
            }
            if (ofs > 0) handle_header(header_buf_, header_seen_);
            else         set_status("raw RTCM3 stream (plain TCP server)");

            header_done_ = true;
            st_.state = NTRIP_STATE_STREAMING;

            /* Replay whatever payload arrived alongside the header. */
            if (header_seen_ > ofs && cb_) {
                cb_((const uint8_t *)header_buf_ + ofs,
                    (uint16_t)(header_seen_ - ofs), cb_arg_);
                st_.bytes_received += header_seen_ - ofs;
                st_.last_data_ms = now_ms();
            }
            /* Bytes of this pbuf beyond what we copied are still unconsumed. */
            off = take;
            n   = q->len;
            if (off >= n) continue;
            d  += off;
            n  -= off;
        }

        if (cb_ && n) {
            cb_(d, n, cb_arg_);
            st_.bytes_received += n;
            st_.last_data_ms = now_ms();
        }
    }

    /* The ring buffer downstream is far larger than one epoch of RTCM, so we
     * acknowledge immediately rather than applying TCP backpressure. */
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    backoff_ms_ = BACKOFF_MIN_MS;    /* a healthy stream resets the backoff */
    return ERR_OK;
}

static void on_err(void *arg, err_t err)
{
    (void)arg;
    pcb_ = NULL;                      /* lwIP has already freed it */
    schedule_retry(err == ERR_ABRT ? "connection aborted" : "connection error");
}

static err_t send_request(struct tcp_pcb *pcb)
{
    char req[320], auth[128], cred[96];
    int n;

    auth[0] = '\0';
    if (cfg_.user && cfg_.user[0]) {
        snprintf(cred, sizeof(cred), "%s:%s", cfg_.user,
                 cfg_.password ? cfg_.password : "");
        char enc[128];
        if (b64_encode(cred, enc, sizeof(enc))) {
            snprintf(auth, sizeof(auth), "Authorization: Basic %s\r\n", enc);
        }
    }
    /* NTRIP v1 request. A v2 caster accepts it, and a plain TCP server simply
     * discards it, which is what makes the same client work against both. */
    n = snprintf(req, sizeof(req),
                 "GET /%s HTTP/1.0\r\n"
                 "User-Agent: NTRIP RTKLIB-Pico/1.0\r\n"
                 "Accept: */*\r\n"
                 "%s"
                 "Connection: close\r\n"
                 "\r\n",
                 cfg_.mountpoint ? cfg_.mountpoint : "", auth);

    err_t e = tcp_write(pcb, req, (u16_t)n, TCP_WRITE_FLAG_COPY);
    if (e == ERR_OK) e = tcp_output(pcb);
    return e;
}

static err_t on_connected(void *arg, struct tcp_pcb *pcb, err_t err)
{
    (void)arg;
    if (err != ERR_OK) { close_conn("TCP connect failed"); return ERR_OK; }

    header_done_ = false;
    header_seen_ = 0;

    if (send_request(pcb) != ERR_OK) {
        close_conn("failed to send NTRIP request");
        return ERR_OK;
    }
    st_.state = NTRIP_STATE_REQUEST_SENT;
    st_.last_data_ms = now_ms();
    set_status("requested /%s from %s:%u", cfg_.mountpoint, cfg_.host, cfg_.port);
    return ERR_OK;
}

static void start_connect(void)
{
    ip_addr_t ip;

    if (!ipaddr_aton(cfg_.host, &ip)) {
        set_status("bad caster address '%s'", cfg_.host);
        st_.state = NTRIP_STATE_BACKOFF;
        retry_at_ms_ = now_ms() + BACKOFF_MAX_MS;
        return;
    }

    cyw43_arch_lwip_begin();
    pcb_ = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (!pcb_) {
        cyw43_arch_lwip_end();
        schedule_retry("out of TCP pcbs");
        return;
    }
    tcp_arg(pcb_, NULL);
    tcp_recv(pcb_, on_recv);
    tcp_err(pcb_, on_err);
    err_t e = tcp_connect(pcb_, &ip, cfg_.port, on_connected);
    cyw43_arch_lwip_end();

    if (e != ERR_OK) { close_conn_ex("tcp_connect refused", false); return; }

    st_.state = NTRIP_STATE_TCP_CONNECTING;
    st_.last_data_ms = now_ms();
    set_status("connecting to %s:%u", cfg_.host, cfg_.port);
}

/* --- Wi-Fi ---------------------------------------------------------------- */
/* The join is asynchronous on purpose. cyw43_arch_wifi_connect_timeout_ms()
 * blocks for up to 20 s, and this runs in the same loop that feeds GPS_INPUT
 * to the flight controller -- so a Wi-Fi dropout would stop position updates
 * for 20 s at a time while the EKF coasted on the IMU alone. Starting the join
 * and polling the link costs microseconds per pass instead. */
static void wifi_start_join(void)
{
    int r;

    st_.wifi_up = false;
    st_.ip[0] = '\0';
    st_.state = NTRIP_STATE_WIFI_CONNECTING;
    join_start_ms_ = now_ms();
    link_status_ = CYW43_LINK_DOWN;

    cyw43_arch_lwip_begin();
    r = cyw43_arch_wifi_connect_async(cfg_.wifi_ssid, cfg_.wifi_password,
                                      CYW43_AUTH_WPA2_AES_PSK);
    cyw43_arch_lwip_end();

    if (r) {
        st_.state = NTRIP_STATE_WIFI_FAILED;
        retry_at_ms_ = now_ms() + WIFI_JOIN_RETRY_MS;
        set_status("could not start Wi-Fi join (%d)", r);
        return;
    }
    st_.joins++;
    set_status("joining Wi-Fi '%s'", cfg_.wifi_ssid);
}

/* Advance an in-flight join using the link status sampled in the poll loop.
 * Returns true the moment the link is usable. */
static bool wifi_check_join(uint32_t t)
{
    if (link_status_ == CYW43_LINK_UP) {
        st_.wifi_up = true;
        snprintf(st_.ip, sizeof(st_.ip), "%s",
                 ip4addr_ntoa(netif_ip4_addr(netif_default)));
        set_status("Wi-Fi up, IP %s", st_.ip);
        return true;
    }
    if (link_status_ < 0) {                    /* FAIL, NONET or BADAUTH */
        st_.state = NTRIP_STATE_WIFI_FAILED;
        retry_at_ms_ = t + WIFI_JOIN_RETRY_MS;
        set_status("%s", link_status_ == CYW43_LINK_BADAUTH ? "Wi-Fi auth rejected"
                       : link_status_ == CYW43_LINK_NONET   ? "Wi-Fi network not found"
                                                            : "Wi-Fi join failed");
        return false;
    }
    if ((uint32_t)(t - join_start_ms_) > WIFI_JOIN_TIMEOUT_MS) {
        st_.state = NTRIP_STATE_WIFI_FAILED;
        retry_at_ms_ = t + WIFI_JOIN_RETRY_MS;
        set_status("Wi-Fi join timed out");
        return false;
    }
    return false;                              /* DOWN, JOIN or NOIP: keep waiting */
}

/* --- public API ----------------------------------------------------------- */
bool ntrip_client_init(const ntrip_config_t *cfg, ntrip_data_cb_t cb, void *arg)
{
    memset(&st_, 0, sizeof(st_));
    cfg_ = *cfg;
    cb_ = cb;
    cb_arg_ = arg;
    if (!cfg_.port) cfg_.port = 2101;
    /* Matches rtkeng_cfg_t.max_age: below that a resumed stream is still
     * usable, so reconnecting early only loses corrections. */
    if (!cfg_.stall_timeout_ms) cfg_.stall_timeout_ms = 30000;
    st_.stall_ms = cfg_.stall_timeout_ms;

    if (cyw43_arch_init()) {
        set_status("CYW43 init failed");
        return false;
    }
    cyw43_arch_enable_sta_mode();
    cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);   /* latency over power */

    /* Returns as soon as the radio is up; the join finishes in the background
     * and ntrip_client_poll() picks it up. */
    wifi_start_join();
    return true;
}

void ntrip_client_poll(void)
{
    uint32_t t = now_ms();

    /* Refresh the link indicator once a second. This is a CYW43 driver call,
     * so it needs the lwIP lock; calling it unlocked on every main-loop
     * iteration races the alarm IRQ that drives lwIP and will tear down a
     * healthy TCP connection within seconds. */
    if ((uint32_t)(t - link_at_ms_) >= LINK_POLL_MS) {
        link_at_ms_ = t;
        cyw43_arch_lwip_begin();
        link_status_ = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        if (st_.wifi_up && (uint32_t)(t - rssi_at_ms_) >= 1000) {
            int32_t r;
            rssi_at_ms_ = t;
            if (cyw43_wifi_get_rssi(&cyw43_state, &r) == 0) {
                st_.rssi = r;
                st_.rssi_valid = 1;
            }
        }
        cyw43_arch_lwip_end();
    }

    /* A link that drops from under an established session has to be noticed
     * here: lwIP will not say so, and the socket would sit there until the
     * stall timeout. */
    if (st_.wifi_up && link_status_ != CYW43_LINK_UP) {
        close_conn("Wi-Fi link lost");
        wifi_start_join();
        return;
    }

    switch (st_.state) {
    case NTRIP_STATE_WIFI_CONNECTING:
        if (wifi_check_join(t)) start_connect();
        break;

    case NTRIP_STATE_WIFI_FAILED:
        if ((int32_t)(t - retry_at_ms_) < 0) break;
        wifi_start_join();
        break;

    case NTRIP_STATE_BACKOFF:
        if ((int32_t)(t - retry_at_ms_) < 0) break;
        st_.reconnects++;
        if (!st_.wifi_up) { wifi_start_join(); break; }
        start_connect();
        break;

    case NTRIP_STATE_TCP_CONNECTING:
    case NTRIP_STATE_REQUEST_SENT:
    case NTRIP_STATE_STREAMING:
        /* A caster that stops sending is indistinguishable from a dead link
         * at the TCP level until keepalive fires, so time it out ourselves. */
        /* last_data_ms is written from lwIP's callback, which under
         * threadsafe_background runs from an alarm IRQ and can fire between the
         * `t = now_ms()` at the top of this function and this comparison. When
         * it does, last_data_ms is *ahead* of t, the unsigned subtraction wraps
         * to ~4.29e9, and the connection is torn down one millisecond after it
         * delivered data. Sampling the clock here shrinks the window, and the
         * signed comparison closes it: a timestamp from the future now reads as
         * a small negative age instead of an enormous positive one. */
        {
            int32_t age = (int32_t)(now_ms() - st_.last_data_ms);
            st_.data_age_ms = age > 0 ? (uint32_t)age : 0u;
            if (age > (int32_t)cfg_.stall_timeout_ms) {
                close_conn_ex("no corrections received; reconnecting", false);
            }
        }
        break;

    default:
        break;
    }
}

const ntrip_status_t *ntrip_client_status(void) { return &st_; }

uint16_t ntrip_client_header(const char **buf)
{
    if (buf) *buf = header_buf_;
    return header_seen_;
}

bool ntrip_client_streaming(void)
{
    return st_.state == NTRIP_STATE_STREAMING;
}

void ntrip_client_led(bool on)
{
    cyw43_arch_lwip_begin();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
    cyw43_arch_lwip_end();
}
