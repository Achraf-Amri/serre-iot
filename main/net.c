#include <string.h>
#include <stdio.h>
#include <time.h>
#include "net.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "mqtt_client.h"

/* ---------- CONFIGURATION ---------- */
#define SIMULATION 1   /* 1 = Wokwi (broker public + pont) ; 0 = montage reel (broker du PC) */

#if SIMULATION
  #define WIFI_SSID     "Wokwi-GUEST"
  #define WIFI_PASS     ""
  #define BROKER_URI    "mqtt://broker.hivemq.com:1883"
  #define TOPIC_PREFIX  "iot/achraf-privat/groupe1/serre"   /* = prefixe distant du pont */
  #define CLIENT_ID     "serre-achraf-7391"                 /* unique : evite les collisions Wokwi */
#else
  #define WIFI_SSID     "TON_WIFI"
  #define WIFI_PASS     "TON_MOT_DE_PASSE"
  #define BROKER_URI    "mqtt://192.168.1.XX:1883"           /* IP du PC (ipconfig) */
  #define TOPIC_PREFIX  "iot/groupe1/serre"
  #define CLIENT_ID     "serre-esp32-groupe1"
#endif
/* ----------------------------------- */

static const char *TAG = "NET";
static volatile bool s_wifi = false;
static volatile bool s_mqtt = false;
static volatile bool s_new_conn = false;
static esp_mqtt_client_handle_t s_client = NULL;
static net_rx_cb_t s_rx_cb = NULL;
static bool s_sntp_started = false;

void net_set_rx_cb(net_rx_cb_t cb) { s_rx_cb = cb; }
bool net_wifi_ok(void) { return s_wifi; }
bool net_mqtt_ok(void) { return s_mqtt; }

bool net_take_new_connection(void)
{
    bool r = s_new_conn;
    s_new_conn = false;
    return r;
}

long long net_epoch(void)
{
    time_t now = time(NULL);
    return (now > 1700000000) ? (long long)now : 0;
}

int net_publish(const char *suffix, const char *payload, int qos, bool retain)
{
    if (!s_client || !s_mqtt) return -1;
    char topic[128];
    snprintf(topic, sizeof(topic), "%s%s", TOPIC_PREFIX, suffix);
    return esp_mqtt_client_publish(s_client, topic, payload, 0, qos, retain);
}

static void mqtt_evt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        s_mqtt = true;
        s_new_conn = true;
        ESP_LOGI(TAG, "MQTT connecte au broker");
        net_publish("/status", "online", 1, true);     /* presence, retenue par le broker */
        esp_mqtt_client_subscribe(s_client, TOPIC_PREFIX "/cmd", 1);
        esp_mqtt_client_subscribe(s_client, TOPIC_PREFIX "/config/set", 1);
        break;
    case MQTT_EVENT_DISCONNECTED:
        s_mqtt = false;
        ESP_LOGW(TAG, "MQTT deconnecte (reconnexion automatique)");
        break;
    case MQTT_EVENT_DATA: {
        char topic[128];
        char payload[256];
        if (e->current_data_offset != 0 || e->data_len != e->total_data_len) {
            ESP_LOGW(TAG, "message fragmente ignore");
            break;
        }
        if (e->topic_len <= 0 || e->topic_len >= (int)sizeof(topic) ||
            e->data_len >= (int)sizeof(payload)) {
            ESP_LOGW(TAG, "message trop long ignore");
            break;
        }
        memcpy(topic, e->topic, e->topic_len);
        topic[e->topic_len] = 0;
        memcpy(payload, e->data, e->data_len);
        payload[e->data_len] = 0;
        ESP_LOGI(TAG, "RECU  %s  ->  %s", topic, payload);
        size_t pl = strlen(TOPIC_PREFIX);
        if (s_rx_cb && strncmp(topic, TOPIC_PREFIX, pl) == 0) {
            s_rx_cb(topic + pl, payload, e->data_len);
        }
        break;
    }
    case MQTT_EVENT_ERROR:
        ESP_LOGW(TAG, "Erreur MQTT");
        break;
    default:
        break;
    }
}

static void mqtt_start(void)
{
    if (s_client) return;
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = BROKER_URI,
        .credentials.client_id = CLIENT_ID,
        .session.keepalive = 15,   /* le Last Will part ~20 s apres une coupure brutale */
        .session.last_will = {
            .topic   = TOPIC_PREFIX "/status",
            .msg     = "offline",
            .msg_len = 7,
            .qos     = 1,
            .retain  = 1,
        },
    };
    s_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_evt, NULL);
    esp_mqtt_client_start(s_client);
}

static void sntp_start_once(void)
{
    if (s_sntp_started) return;
    s_sntp_started = true;
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
}

static void wifi_evt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi = false;
        ESP_LOGW(TAG, "Wi-Fi perdu, nouvelle tentative...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        s_wifi = true;
        ESP_LOGI(TAG, "Wi-Fi connecte, IP " IPSTR, IP2STR(&ev->ip_info.ip));
        sntp_start_once();
        mqtt_start();
    }
}

void net_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_evt, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_evt, NULL));

    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, WIFI_PASS, sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = (strlen(WIFI_PASS) > 0) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
}