/*------------------------------------------------------------------------------
* wifi_bridge.c : Wi-Fi MAVLink bridge for a Pico W / Pico 2 W
*
*   ArduPilot <--UART, MAVLink--> Pico <--UDP 14550, Wi-Fi--> ground station
*
*   GP0  UART0 TX -> flight controller RX
*   GP1  UART0 RX <- flight controller TX
*
* These are the light controller's flight-controller pins, so this firmware is
* the first stage of that board's firmware. On the FC set SERIAL<n>_PROTOCOL=2
* and SERIAL<n>_BAUD to match FC_BAUD.
*
* Until a ground station is heard, FC traffic is broadcast to UDP 14550, which
* QGroundControl and Mission Planner listen on. The first ground packet teaches
* us its address and traffic becomes unicast; if it goes quiet for
* PEER_TIMEOUT_MS we go back to broadcasting so another station can connect.
*
* Status LED: slow blink = Wi-Fi down, solid = connected, flicker = traffic.
* USB serial prints a status line every few seconds.
*-----------------------------------------------------------------------------*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "lwip/udp.h"
#include "lwip/ip.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "cyw43.h"

#include "mavlink_min.h"

/* Set at configure time; see the wifi_bridge section of pico/README.md. */
#if !defined(WIFI_SSID) || !defined(WIFI_PASSWORD)
#error "Configure with -DRTKPICO_WIFI_SSID=... -DRTKPICO_WIFI_PASSWORD=..."
#endif

#define FC_UART          uart0
#define FC_UART_IRQ      UART0_IRQ
#define FC_TX_PIN        0
#define FC_RX_PIN        1
#ifndef FC_BAUD
#define FC_BAUD          460800
#endif

#define MAVLINK_UDP_PORT 14550
#define PEER_TIMEOUT_MS  10000      /* ground stations heartbeat at 1 Hz */

#define WIFI_JOIN_TIMEOUT_MS 20000
#define WIFI_BACKOFF_MIN_MS  1000
#define WIFI_BACKOFF_MAX_MS  16000
#define LINK_POLL_MS         250
#define STATUS_PERIOD_MS     3000
#define TRAFFIC_LED_MS       200

#define RING_SIZE 4096              /* power of two */

/* One producer and one consumer per ring, so head and tail need no lock. */
typedef struct {
    uint8_t buf[RING_SIZE];
    volatile uint32_t head, tail;
} ring_t;

static inline uint32_t ring_used(const ring_t *r)
{
    return (r->head - r->tail) & (RING_SIZE - 1);
}
static inline bool ring_put(ring_t *r, uint8_t b)
{
    uint32_t nx = (r->head + 1) & (RING_SIZE - 1);
    if (nx == r->tail) return false;
    r->buf[r->head] = b;
    r->head = nx;
    return true;
}
static inline bool ring_get(ring_t *r, uint8_t *b)
{
    uint32_t t = r->tail;
    if (t == r->head) return false;
    *b = r->buf[t];
    r->tail = (t + 1) & (RING_SIZE - 1);
    return true;
}

static ring_t from_fc;              /* UART IRQ -> main loop */
static ring_t to_fc;                /* lwIP callback -> main loop */

typedef enum { WIFI_JOINING, WIFI_UP, WIFI_BACKOFF } wifi_state_t;

static struct udp_pcb *pcb;
static ip_addr_t peer_ip;           /* peer_* are guarded by the lwIP lock */
static u16_t     peer_port;
static bool      have_peer;
static uint32_t  peer_rx_ms;

static wifi_state_t wifi_state = WIFI_BACKOFF;
static int          link_status = CYW43_LINK_DOWN;
#define NO_FAILURE 100     /* outside the CYW43_LINK_* values */
static int          last_failure = NO_FAILURE;
static uint32_t     join_start_ms, retry_at_ms, backoff_ms = WIFI_BACKOFF_MIN_MS;
static uint32_t     joins;

static mav_parser_t parser;
static uint8_t  fc_sysid;
static volatile uint32_t last_traffic_ms;

/* counters */
static volatile uint32_t fc_rx_bytes, fc_rx_dropped;
static volatile uint32_t net_rx_bytes, net_rx_datagrams, net_rx_ignored, net_rx_dropped;
static uint32_t fc_frames, net_tx_datagrams, net_tx_failed;

/* --- flight controller UART ------------------------------------------------ */
static void on_uart_rx(void)
{
    while (uart_is_readable(FC_UART)) {
        uint8_t c = (uint8_t)uart_get_hw(FC_UART)->dr;
        fc_rx_bytes++;
        if (!ring_put(&from_fc, c)) fc_rx_dropped++;
    }
}

static void fc_uart_init(void)
{
    uart_init(FC_UART, FC_BAUD);
    gpio_set_function(FC_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(FC_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(FC_RX_PIN);        /* quiet line while the FC is unplugged */
    uart_set_hw_flow(FC_UART, false, false);
    uart_set_format(FC_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(FC_UART, true);
    irq_set_exclusive_handler(FC_UART_IRQ, on_uart_rx);
    irq_set_enabled(FC_UART_IRQ, true);
    uart_set_irq_enables(FC_UART, true, false);
}

/* --- UDP ------------------------------------------------------------------- */
/* Runs in lwIP's context, with the lock already held. */
static void on_udp(void *arg, struct udp_pcb *p_, struct pbuf *p,
                   const ip_addr_t *addr, u16_t port)
{
    (void)arg; (void)p_;
    if (!p) return;

    /* Every bridge on the network broadcasts to this port. Taking another
     * drone's broadcast for a ground station would feed its telemetry to our
     * FC and send ours to it, so only unicast counts. */
    uint8_t first = p->len ? ((const uint8_t *)p->payload)[0] : 0;
    if (ip_addr_isbroadcast(ip_current_dest_addr(), ip_current_netif()) ||
        (first != MAVLINK_STX_V1 && first != MAVLINK_STX_V2)) {
        net_rx_ignored++;
        pbuf_free(p);
        return;
    }

    ip_addr_copy(peer_ip, *addr);
    peer_port  = port;
    have_peer  = true;
    peer_rx_ms = to_ms_since_boot(get_absolute_time());
    net_rx_datagrams++;
    last_traffic_ms = peer_rx_ms;

    /* All or nothing: a partly queued message would corrupt the next one. */
    if (RING_SIZE - 1 - ring_used(&to_fc) < p->tot_len) {
        net_rx_dropped += p->tot_len;
    } else {
        for (struct pbuf *q = p; q; q = q->next) {
            for (u16_t i = 0; i < q->len; i++) {
                ring_put(&to_fc, ((const uint8_t *)q->payload)[i]);
            }
        }
        net_rx_bytes += p->tot_len;
    }
    pbuf_free(p);
}

static bool udp_init_pcb(void)
{
    cyw43_arch_lwip_begin();
    pcb = udp_new_ip_type(IPADDR_TYPE_V4);
    if (pcb) {
        ip_set_option(pcb, SOF_BROADCAST);
        if (udp_bind(pcb, IP_ANY_TYPE, MAVLINK_UDP_PORT) != ERR_OK) {
            udp_remove(pcb);
            pcb = NULL;
        } else {
            udp_recv(pcb, on_udp, NULL);
        }
    }
    cyw43_arch_lwip_end();
    return pcb != NULL;
}

static void net_send(const uint8_t *d, uint16_t n, uint32_t now)
{
    cyw43_arch_lwip_begin();
    if (have_peer && (uint32_t)(now - peer_rx_ms) > PEER_TIMEOUT_MS) have_peer = false;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, n, PBUF_RAM);
    if (p) {
        pbuf_take(p, d, n);
        err_t e;
        if (have_peer) {
            e = udp_sendto(pcb, p, &peer_ip, peer_port);
        } else {
            /* The subnet's broadcast address: on the bench, a Mac on the same
             * network received these but not 255.255.255.255. */
            ip_addr_t bcast;
            ip_addr_set_ip4_u32(&bcast, ip4_addr_get_u32(netif_ip4_addr(netif_default)) |
                                        ~ip4_addr_get_u32(netif_ip4_netmask(netif_default)));
            e = udp_sendto(pcb, p, &bcast, MAVLINK_UDP_PORT);
        }
        pbuf_free(p);
        if (e == ERR_OK) net_tx_datagrams++; else net_tx_failed++;
    } else {
        net_tx_failed++;
    }
    cyw43_arch_lwip_end();
}

/* --- Wi-Fi ----------------------------------------------------------------- */
static void wifi_backoff(uint32_t now, int reason)
{
    last_failure = reason;
    wifi_state   = WIFI_BACKOFF;
    retry_at_ms  = now + backoff_ms;
    backoff_ms   = backoff_ms * 2 > WIFI_BACKOFF_MAX_MS ? WIFI_BACKOFF_MAX_MS : backoff_ms * 2;
}

static void wifi_start_join(uint32_t now)
{
    cyw43_arch_lwip_begin();
    int r = cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);
    cyw43_arch_lwip_end();
    joins++;
    if (r) { wifi_backoff(now, r); return; }
    wifi_state    = WIFI_JOINING;
    join_start_ms = now;
    link_status   = CYW43_LINK_DOWN;
}

/* The join is asynchronous so a dropout never stalls the UART. */
static void wifi_poll(uint32_t now)
{
    static uint32_t polled_at;

    if ((uint32_t)(now - polled_at) >= LINK_POLL_MS) {
        polled_at = now;
        cyw43_arch_lwip_begin();
        link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        cyw43_arch_lwip_end();
    }

    switch (wifi_state) {
    case WIFI_UP:
        if (link_status != CYW43_LINK_UP) wifi_backoff(now, link_status);
        break;
    case WIFI_JOINING:
        if (link_status == CYW43_LINK_UP) {
            wifi_state   = WIFI_UP;
            backoff_ms   = WIFI_BACKOFF_MIN_MS;
            last_failure = NO_FAILURE;
        } else if (link_status < 0) {       /* FAIL, NONET or BADAUTH */
            wifi_backoff(now, link_status);
        } else if ((uint32_t)(now - join_start_ms) > WIFI_JOIN_TIMEOUT_MS) {
            wifi_backoff(now, CYW43_LINK_DOWN);
        }
        break;
    case WIFI_BACKOFF:
        if ((int32_t)(now - retry_at_ms) >= 0) wifi_start_join(now);
        break;
    }
}

/* --- status ---------------------------------------------------------------- */
static void led_update(uint32_t now)
{
    static bool shown = false, first = true;
    bool on;

    if (wifi_state != WIFI_UP) {
        on = (now / 500) & 1;
    } else {
        on = !((uint32_t)(now - last_traffic_ms) < TRAFFIC_LED_MS && (now % 200) < 40);
    }
    if (on == shown && !first) return;
    first = false;
    shown = on;
    cyw43_arch_lwip_begin();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
    cyw43_arch_lwip_end();
}

static const char *failure_name(int s)
{
    switch (s) {
    case NO_FAILURE:         return "-";
    case CYW43_LINK_BADAUTH: return "bad password";
    case CYW43_LINK_NONET:   return "network not found";
    case CYW43_LINK_FAIL:    return "join failed";
    default:                 return "timed out or link lost";
    }
}

static void print_status(void)
{
    char ip[16] = "-", peer[24] = "broadcast";
    int32_t rssi = 0;

    cyw43_arch_lwip_begin();
    if (wifi_state == WIFI_UP) {
        ip4addr_ntoa_r(netif_ip4_addr(netif_default), ip, sizeof(ip));
        cyw43_wifi_get_rssi(&cyw43_state, &rssi);
    }
    if (have_peer) {
        char a[16];
        ip4addr_ntoa_r(ip_2_ip4(&peer_ip), a, sizeof(a));
        snprintf(peer, sizeof(peer), "%s:%u", a, peer_port);
    }
    cyw43_arch_lwip_end();

    if (wifi_state == WIFI_UP) {
        printf("[bridge] wifi=up ssid=%s ip=%s rssi=%ld ground=%s", WIFI_SSID, ip, (long)rssi, peer);
    } else {
        printf("[bridge] wifi=%s ssid=%s joins=%lu last_failure=\"%s\"",
               wifi_state == WIFI_JOINING ? "joining" : "retrying", WIFI_SSID,
               (unsigned long)joins, failure_name(last_failure));
    }
    printf(" | fc_sysid=%u fc->net bytes=%lu frames=%lu sent=%lu failed=%lu resync=%lu drop=%lu"
           " | net->fc bytes=%lu datagrams=%lu ignored=%lu drop=%lu\n",
           fc_sysid, (unsigned long)fc_rx_bytes, (unsigned long)fc_frames,
           (unsigned long)net_tx_datagrams, (unsigned long)net_tx_failed,
           (unsigned long)parser.resyncs, (unsigned long)fc_rx_dropped,
           (unsigned long)net_rx_bytes, (unsigned long)net_rx_datagrams,
           (unsigned long)net_rx_ignored, (unsigned long)net_rx_dropped);
}

int main(void)
{
    bi_decl(bi_program_description("Wi-Fi MAVLink bridge: FC UART <-> UDP 14550"));
    bi_decl(bi_2pins_with_func(FC_RX_PIN, FC_TX_PIN, GPIO_FUNC_UART));

    stdio_init_all();
    mav_parser_reset(&parser);
    fc_uart_init();

    if (cyw43_arch_init()) {
        while (true) { printf("[bridge] CYW43 init failed: is this a W board?\n"); sleep_ms(2000); }
    }
    cyw43_arch_enable_sta_mode();
    /* No power saving: it delays every inbound packet by a sleep interval. */
    cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM);

    while (!udp_init_pcb()) { printf("[bridge] UDP bind failed\n"); sleep_ms(2000); }

    uint32_t status_at = 0;
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        uint8_t b;
        const uint8_t *frame;
        uint16_t len;

        while (uart_is_writable(FC_UART) && ring_get(&to_fc, &b)) {
            uart_putc_raw(FC_UART, b);
        }

        while (ring_get(&from_fc, &b)) {
            if (!mav_parse_byte(&parser, b, &frame, &len)) continue;
            fc_frames = parser.frames;
            if (mav_frame_msgid(frame) == MAVLINK_MSG_HEARTBEAT &&
                mav_frame_compid(frame) == 1) {     /* MAV_COMP_ID_AUTOPILOT1 */
                fc_sysid = mav_frame_sysid(frame);
            }
            if (wifi_state == WIFI_UP) {
                net_send(frame, len, now);
                last_traffic_ms = now;
            }
        }

        wifi_poll(now);
        led_update(now);

        if ((uint32_t)(now - status_at) >= STATUS_PERIOD_MS) {
            status_at = now;
            print_status();
        }
    }
}
