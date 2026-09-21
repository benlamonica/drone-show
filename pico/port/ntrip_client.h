/*------------------------------------------------------------------------------
* ntrip_client.h : Wi-Fi + NTRIP client for the base correction link
*
* Joins a Wi-Fi network, opens a TCP connection to an NTRIP caster, and hands
* every received RTCM3 byte to a callback. Runs on lwIP's raw API with
* pico_cyw43_arch_lwip_threadsafe_background, so the connection keeps being
* serviced from an alarm IRQ while rtkpos() blocks the main loop.
*
* The reader is deliberately tolerant about what answers on the far end:
*   - an NTRIP v1 caster replying "ICY 200 OK"
*   - an NTRIP v2 caster replying "HTTP/1.1 200 OK"
*   - a plain TCP server (RTKLIB `str2str ... tcpsvr://:2101`) that starts
*     sending RTCM3 immediately and ignores our request line
* A SOURCETABLE reply -- what a caster sends when the mountpoint is wrong -- is
* reported with the mountpoint names it offers, because that is the single
* most likely thing to be misconfigured.
*
* All state is recovered automatically: link loss, caster restart and RTCM
* silence each trigger a reconnect with backoff.
*-----------------------------------------------------------------------------*/
#ifndef NTRIP_CLIENT_H
#define NTRIP_CLIENT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NTRIP_STATE_IDLE = 0,
    NTRIP_STATE_WIFI_CONNECTING,
    NTRIP_STATE_WIFI_FAILED,
    NTRIP_STATE_TCP_CONNECTING,
    NTRIP_STATE_REQUEST_SENT,
    NTRIP_STATE_STREAMING,       /* corrections are flowing */
    NTRIP_STATE_BACKOFF,
} ntrip_state_t;

typedef struct {
    const char *wifi_ssid;
    const char *wifi_password;
    const char *host;            /* caster IP, dotted quad */
    uint16_t    port;            /* 2101 is the registered NTRIP port */
    const char *mountpoint;      /* without a leading '/' */
    const char *user;            /* NULL or "" for no authentication */
    const char *password;
    uint32_t    stall_timeout_ms;/* reconnect if no data for this long */
} ntrip_config_t;

typedef struct {
    ntrip_state_t state;
    bool     wifi_up;
    uint32_t bytes_received;
    uint32_t reconnects;
    uint32_t joins;              /* Wi-Fi join attempts started              */
    uint32_t last_data_ms;
    uint32_t rssi_valid;
    int32_t  rssi;
    char     ip[16];             /* our DHCP address: if this is not on the
                                    caster's subnet, nothing else will work  */
    char     status[96];         /* last notable event, for the health line */
    uint32_t stall_ms;           /* configured stall timeout, as actually used */
    uint32_t data_age_ms;        /* now - last_data_ms at the last poll        */
} ntrip_status_t;

/* Called from lwIP's callback context (an alarm IRQ under
 * threadsafe_background), once per received chunk. Keep it short: push the
 * bytes into a ring buffer and return. */
typedef void (*ntrip_data_cb_t)(const uint8_t *data, uint16_t len, void *arg);

/* Brings up the CYW43 driver and starts the Wi-Fi join. Non-blocking beyond
 * the driver init; returns false only if the radio itself fails to start. */
bool ntrip_client_init(const ntrip_config_t *cfg, ntrip_data_cb_t cb, void *arg);

/* Drive reconnection and timeouts. Call from the main loop; cheap when idle. */
void ntrip_client_poll(void);

const ntrip_status_t *ntrip_client_status(void);
bool ntrip_client_streaming(void);

/* The first bytes of the caster's reply, exactly as received. When a header is
 * rejected the counters only say "no usable response header"; the bytes
 * themselves say whether it was a sourcetable, an auth failure, an HTML error
 * page or a raw stream we joined mid-message. Returns the length and points
 * *buf at the buffer. */
uint16_t ntrip_client_header(const char **buf);

/* Pico 2 W's LED hangs off the CYW43 module, so it is only drivable once the
 * radio is up. Exposed here because that is where the driver lives. */
void ntrip_client_led(bool on);

#ifdef __cplusplus
}
#endif
#endif /* NTRIP_CLIENT_H */
