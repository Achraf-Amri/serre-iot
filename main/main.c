#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "net.h"

#define I2C_SDA      GPIO_NUM_21
#define I2C_SCL      GPIO_NUM_22
#define BMP_ADDR     0x77
#define OLED_ADDR    0x3C
#define SOL_CHANNEL  ADC_CHANNEL_6   // GPIO 34

static const char *TAG = "SERRE";

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t bmp_dev;
static i2c_master_dev_handle_t oled_dev;
static adc_oneshot_unit_handle_t adc1;

/* =====================  MINI PILOTE OLED SSD1306  ===================== */

static uint8_t fb[128 * 8];       // framebuffer 128x64 (8 pages de 128 octets)

static const char CHARSET[] = " %-./:0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static const uint8_t FONT[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x23,0x13,0x08,0x64,0x62}, // %
    {0x08,0x08,0x08,0x08,0x08}, // -
    {0x00,0x60,0x60,0x00,0x00}, // .
    {0x20,0x10,0x08,0x04,0x02}, // /
    {0x00,0x36,0x36,0x00,0x00}, // :
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
    {0x7E,0x11,0x11,0x11,0x7E}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x09,0x01}, // F
    {0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x00,0x41,0x7F,0x41,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01}, // J
    {0x7F,0x08,0x14,0x22,0x41}, // K
    {0x7F,0x40,0x40,0x40,0x40}, // L
    {0x7F,0x02,0x0C,0x02,0x7F}, // M
    {0x7F,0x04,0x08,0x10,0x7F}, // N
    {0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x7F,0x09,0x09,0x09,0x06}, // P
    {0x3E,0x41,0x51,0x21,0x5E}, // Q
    {0x7F,0x09,0x19,0x29,0x46}, // R
    {0x46,0x49,0x49,0x49,0x31}, // S
    {0x01,0x01,0x7F,0x01,0x01}, // T
    {0x3F,0x40,0x40,0x40,0x3F}, // U
    {0x1F,0x20,0x40,0x20,0x1F}, // V
    {0x3F,0x40,0x38,0x40,0x3F}, // W
    {0x63,0x14,0x08,0x14,0x63}, // X
    {0x07,0x08,0x70,0x08,0x07}, // Y
    {0x61,0x51,0x49,0x45,0x43}, // Z
};

static void oled_cmds(const uint8_t *cmds, size_t n)
{
    uint8_t buf[16];
    buf[0] = 0x00;                       // octet de controle : commandes
    memcpy(&buf[1], cmds, n);
    i2c_master_transmit(oled_dev, buf, n + 1, 100);
}

static void oled_init(void)
{
    static const uint8_t init[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF,
        0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
    size_t i = 0;
    while (i < sizeof(init)) {
        size_t n = sizeof(init) - i;
        if (n > 15) n = 15;
        oled_cmds(&init[i], n);
        i += n;
    }
    memset(fb, 0, sizeof(fb));
}

static void oled_flush(void)
{
    static uint8_t tx[129];
    for (uint8_t page = 0; page < 8; page++) {
        uint8_t c[3] = { (uint8_t)(0xB0 | page), 0x00, 0x10 };
        oled_cmds(c, 3);
        tx[0] = 0x40;                    // octet de controle : donnees
        memcpy(&tx[1], &fb[page * 128], 128);
        i2c_master_transmit(oled_dev, tx, 129, 200);
    }
}

static void oled_clear_line(int line)
{
    memset(&fb[line * 128], 0, 128);
}

static void oled_text(int line, const char *s)
{
    oled_clear_line(line);
    int x = 0;
    for (; *s && x + 5 <= 128; s++, x += 6) {
        char ch = (char)toupper((unsigned char)*s);
        const char *p = (ch != 0) ? strchr(CHARSET, ch) : NULL;
        int idx = p ? (int)(p - CHARSET) : 0;
        memcpy(&fb[line * 128 + x], FONT[idx], 5);
    }
}

/* ========================  PILOTE BMP180  ============================== */

static int16_t  AC1, AC2, AC3, B1, B2, MB, MC, MD;
static uint16_t AC4, AC5, AC6;

static esp_err_t bmp_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(bmp_dev, &reg, 1, buf, len, 100);
}

static esp_err_t bmp_write(uint8_t reg, uint8_t val)
{
    uint8_t d[2] = { reg, val };
    return i2c_master_transmit(bmp_dev, d, 2, 100);
}

static esp_err_t bmp180_init(void)
{
    uint8_t id = 0;
    esp_err_t err = bmp_read(0xD0, &id, 1);
    if (err != ESP_OK) return err;
    if (id != 0x55) {
        ESP_LOGE(TAG, "ID BMP180 inattendu : 0x%02X (attendu 0x55)", id);
        return ESP_FAIL;
    }
    uint8_t c[22];
    err = bmp_read(0xAA, c, 22);
    if (err != ESP_OK) return err;

    #define S16(i) ((int16_t)((c[i] << 8) | c[(i) + 1]))
    #define U16(i) ((uint16_t)((c[i] << 8) | c[(i) + 1]))
    AC1 = S16(0);  AC2 = S16(2);  AC3 = S16(4);
    AC4 = U16(6);  AC5 = U16(8);  AC6 = U16(10);
    B1  = S16(12); B2  = S16(14);
    MB  = S16(16); MC  = S16(18); MD  = S16(20);
    return ESP_OK;
}

/* t en degres C, p en hPa. Retourne true si les lectures sont plausibles. */
static bool bmp180_measure(float *t, float *p)
{
    uint8_t b[3];

    if (bmp_write(0xF4, 0x2E) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    if (bmp_read(0xF6, b, 2) != ESP_OK) return false;
    int32_t UT = (b[0] << 8) | b[1];

    if (bmp_write(0xF4, 0x34) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    if (bmp_read(0xF6, b, 3) != ESP_OK) return false;
    int32_t UP = ((b[0] << 16) | (b[1] << 8) | b[2]) >> 8;

    int32_t X1 = ((UT - (int32_t)AC6) * (int32_t)AC5) >> 15;
    if (X1 + MD == 0) return false;
    int32_t X2 = ((int32_t)MC << 11) / (X1 + MD);
    int32_t B5 = X1 + X2;
    *t = ((B5 + 8) >> 4) / 10.0f;

    int32_t B6 = B5 - 4000;
    X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = (((int32_t)AC1 * 4 + X3) + 2) >> 2;
    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = (uint32_t)AC4 * (uint32_t)(X3 + 32768) >> 15;
    if (B4 == 0) return false;
    uint32_t B7 = ((uint32_t)UP - B3) * 50000;
    int32_t pa = (B7 < 0x80000000) ? (int32_t)((B7 * 2) / B4)
                                   : (int32_t)((B7 / B4) * 2);
    X1 = (pa >> 8) * (pa >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * pa) >> 16;
    pa = pa + ((X1 + X2 + 3791) >> 4);
    *p = pa / 100.0f;

    return (*t > -40.0f && *t < 85.0f && *p > 300.0f && *p < 1100.0f);
}

/* ===================  SOL : FILTRAGE + CALIBRATION  ==================== */

#define BTN_SEC      GPIO_NUM_32
#define BTN_HUM      GPIO_NUM_33
#define EMA_ALPHA    0.2f
#define ADC_MIN_OK   20      // en dessous : sonde debranchee / court-circuit
#define ADC_MAX_OK   4075    // au dessus  : circuit ouvert
#define CAL_MIN_GAP  200     // ecart minimal sec/humide pour une calibration valide

static int   adc_sec    = 0;     // calibration sol sec
static int   adc_humide = 0;     // calibration sol humide
static float soil_f     = -1.0f; // valeur brute filtree (-1 = pas encore initialisee)
static char  status_msg[24] = "";
static int   status_ticks   = 0;

static void nvs_load_cal(void)
{
    /* Valeurs par defaut (mesures du groupe) : utilisees si la NVS est vide,
       ce qui est le cas a chaque lancement en simulation Wokwi. */
    adc_sec = 3170;
    adc_humide = 668;

    nvs_handle_t h;
    if (nvs_open("cal", NVS_READONLY, &h) == ESP_OK) {
        int32_t v;
        if (nvs_get_i32(h, "sec", &v) == ESP_OK) adc_sec = v;
        if (nvs_get_i32(h, "hum", &v) == ESP_OK) adc_humide = v;
        nvs_close(h);
    }
}

static void nvs_save_cal(void)
{
    nvs_handle_t h;
    if (nvs_open("cal", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "sec", adc_sec);
        nvs_set_i32(h, "hum", adc_humide);
        nvs_commit(h);
        nvs_close(h);
    }
}

static bool cal_valid(void)
{
    return adc_sec >= ADC_MIN_OK && adc_humide >= ADC_MIN_OK
        && abs(adc_sec - adc_humide) >= CAL_MIN_GAP;
}

/* Moyenne de n lectures ADC. Retourne -1 si toutes les lectures echouent. */
static int soil_read_avg(int n)
{
    long sum = 0;
    int ok = 0;
    for (int i = 0; i < n; i++) {
        int r;
        if (adc_oneshot_read(adc1, SOL_CHANNEL, &r) == ESP_OK) {
            sum += r;
            ok++;
        }
    }
    return ok ? (int)(sum / ok) : -1;
}

static void set_status(const char *msg)
{
    snprintf(status_msg, sizeof(status_msg), "%s", msg);
    status_ticks = 30;   // affiche pendant 3 s
}

static void calibrate(bool dry)
{
    int v = soil_read_avg(64);
    if (v < ADC_MIN_OK || v > ADC_MAX_OK) {
        ESP_LOGW(TAG, "Calibration refusee : lecture ADC invalide (%d)", v);
        set_status("CAL REFUSEE");
        return;
    }
    if (dry) adc_sec = v; else adc_humide = v;
    nvs_save_cal();
    ESP_LOGI(TAG, "Calibration %s enregistree : ADC=%d  (sec=%d, humide=%d)",
             dry ? "SEC" : "HUMIDE", v, adc_sec, adc_humide);
    set_status(dry ? "CAL SEC OK" : "CAL HUM OK");
}

/* ===============  ETAT APPLIQUE, COMMANDES, ALERTES, PUBLICATIONS  ============== */

#define LED_PUMP    GPIO_NUM_26     // LED verte : pompe d'arrosage simulee
#define LED_VENT    GPIO_NUM_27     // LED jaune : ventilation simulee
#define SOIL_HYST   10.0f           // pompe OFF quand indice >= seuil + 10
#define TEMP_HYST   2.0f            // ventilation OFF quand T <= seuil - 2

typedef struct {
    bool  manual;      // false = AUTO, true = MANUEL
    bool  pump;        // etat reellement applique a la LED verte
    bool  vent;        // etat reellement applique a la LED jaune
    float soil_thr;    // seuil de sol sec (%)
    float temp_thr;    // seuil de temperature (C)
    int   period_s;    // periode de publication de la telemetrie (s)
} app_state_t;

static app_state_t st = { .manual = false, .pump = false, .vent = false,
                          .soil_thr = 30, .temp_thr = 30, .period_s = 5 };
static SemaphoreHandle_t st_mx;
static volatile bool state_dirty = true;   // /state a (re)publier

typedef struct { char type[24]; char cause[80]; char state[10]; } alert_t;
static QueueHandle_t alert_q;

static void push_alert(const char *type, const char *cause, const char *state)
{
    alert_t a;
    snprintf(a.type, sizeof(a.type), "%s", type);
    snprintf(a.cause, sizeof(a.cause), "%s", cause);
    snprintf(a.state, sizeof(a.state), "%s", state);
    xQueueSend(alert_q, &a, 0);
}

static void reject(const char *type, const char *cause)
{
    ESP_LOGW(TAG, "%s : %s", type, cause);
    push_alert(type, cause, "active");
}

/* Appelee par le client MQTT pour /cmd et /config/set : valide, applique, confirme. */
static void on_rx(const char *suffix, const char *data, int len)
{
    (void)len;
    bool is_cmd = (strcmp(suffix, "/cmd") == 0);
    bool is_cfg = (strcmp(suffix, "/config/set") == 0);
    if (!is_cmd && !is_cfg) return;

    const char *rej = is_cmd ? "cmd_rejected" : "config_rejected";
    cJSON *root = cJSON_Parse(data);
    if (!cJSON_IsObject(root)) {
        reject(rej, "message non JSON");
        state_dirty = true;
        cJSON_Delete(root);
        return;
    }

    xSemaphoreTake(st_mx, portMAX_DELAY);
    app_state_t n = st;
    const char *err = NULL;

    if (is_cmd) {
        cJSON *m = cJSON_GetObjectItemCaseSensitive(root, "mode");
        cJSON *p = cJSON_GetObjectItemCaseSensitive(root, "pump");
        cJSON *v = cJSON_GetObjectItemCaseSensitive(root, "vent");
        if (m) {
            if (cJSON_IsString(m) && m->valuestring && strcmp(m->valuestring, "AUTO") == 0) {
                n.manual = false; n.pump = false; n.vent = false;
            } else if (cJSON_IsString(m) && m->valuestring && strcmp(m->valuestring, "MANUEL") == 0) {
                n.manual = true;
            } else {
                err = "mode invalide";
            }
        }
        if (!err && (p || v)) {
            if (!n.manual) {
                err = "commande LED refusee en mode AUTO";
            } else {
                if (p) { if (cJSON_IsBool(p)) n.pump = cJSON_IsTrue(p); else err = "pump doit etre booleen"; }
                if (!err && v) { if (cJSON_IsBool(v)) n.vent = cJSON_IsTrue(v); else err = "vent doit etre booleen"; }
            }
        }
    } else {
        cJSON *a = cJSON_GetObjectItemCaseSensitive(root, "soil_dry_threshold");
        cJSON *b = cJSON_GetObjectItemCaseSensitive(root, "temp_threshold");
        cJSON *c = cJSON_GetObjectItemCaseSensitive(root, "publish_period_s");
        if (a) {
            if (cJSON_IsNumber(a) && a->valuedouble >= 0 && a->valuedouble <= 100) n.soil_thr = (float)a->valuedouble;
            else err = "soil_dry_threshold hors plage (0..100)";
        }
        if (!err && b) {
            if (cJSON_IsNumber(b) && b->valuedouble >= 0 && b->valuedouble <= 80) n.temp_thr = (float)b->valuedouble;
            else err = "temp_threshold hors plage (0..80)";
        }
        if (!err && c) {
            if (cJSON_IsNumber(c) && c->valuedouble >= 1 && c->valuedouble <= 3600) n.period_s = (int)c->valuedouble;
            else err = "publish_period_s hors plage (1..3600)";
        }
    }

    if (!err) st = n;                  // rien n'est applique si une seule valeur est refusee
    xSemaphoreGive(st_mx);

    if (err) reject(rej, err);
    state_dirty = true;                // dans tous les cas, /state confirme l'etat reel
    cJSON_Delete(root);
}

static void publish_state(void)
{
    state_dirty = false;
    app_state_t s;
    xSemaphoreTake(st_mx, portMAX_DELAY);
    s = st;
    xSemaphoreGive(st_mx);

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"ts\":%lld,\"mode\":\"%s\",\"pump\":%s,\"vent\":%s,"
        "\"params\":{\"soil_dry_threshold\":%g,\"temp_threshold\":%g,\"publish_period_s\":%d}}",
        net_epoch(), s.manual ? "MANUEL" : "AUTO",
        s.pump ? "true" : "false", s.vent ? "true" : "false",
        s.soil_thr, s.temp_thr, s.period_s);
    if (net_publish("/state", buf, 1, true) < 0) state_dirty = true;   // on reessaiera
}

static void publish_telemetry(bool bv, float t, float p, bool iv, float idx, float adc)
{
    static uint32_t seq = 0;
    char ts_[16], ps_[16], is_[16], buf[384];

    if (bv) {
        snprintf(ts_, sizeof(ts_), "%.1f", t);
        snprintf(ps_, sizeof(ps_), "%.1f", p);
    } else {
        strcpy(ts_, "null");
        strcpy(ps_, "null");
    }
    if (iv) snprintf(is_, sizeof(is_), "%.0f", idx);
    else    strcpy(is_, "null");

    seq++;
    snprintf(buf, sizeof(buf),
        "{\"seq\":%lu,\"ts\":%lld,\"uptime_s\":%lld,"
        "\"temperature\":{\"value\":%s,\"unit\":\"\xC2\xB0" "C\",\"valid\":%s},"
        "\"pressure\":{\"value\":%s,\"unit\":\"hPa\",\"valid\":%s},"
        "\"soil_index\":{\"value\":%s,\"unit\":\"%%\",\"valid\":%s,\"adc\":%.0f}}",
        (unsigned long)seq, net_epoch(), (long long)(esp_timer_get_time() / 1000000),
        ts_, bv ? "true" : "false",
        ps_, bv ? "true" : "false",
        is_, iv ? "true" : "false", adc);
    net_publish("/telemetry", buf, 0, false);
}

static int publish_alert(const alert_t *a)
{
    char buf[220];
    snprintf(buf, sizeof(buf),
        "{\"ts\":%lld,\"type\":\"%s\",\"cause\":\"%s\",\"state\":\"%s\"}",
        net_epoch(), a->type, a->cause, a->state);
    return net_publish("/alerts", buf, 1, false);
}

/* ==============================  TACHE PRINCIPALE  ======================== */

static void app_task(void *arg)
{
    st_mx   = xSemaphoreCreateMutex();
    alert_q = xQueueCreate(8, sizeof(alert_t));

    /* LED : eteintes au demarrage, jusqu'aux premieres mesures valides */
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << LED_PUMP) | (1ULL << LED_VENT),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&led_cfg);
    gpio_set_level(LED_PUMP, 0);
    gpio_set_level(LED_VENT, 0);

    /* NVS (stockage de la calibration) */
    esp_err_t nerr = nvs_flash_init();
    if (nerr == ESP_ERR_NVS_NO_FREE_PAGES || nerr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nerr = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nerr);
    nvs_load_cal();

    /* Wi-Fi + MQTT (apres la NVS) */
    net_set_rx_cb(on_rx);
    net_start();

    /* Boutons de calibration (actifs a l'etat bas) */
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BTN_SEC) | (1ULL << BTN_HUM),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn_cfg);

    /* I2C */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t bmp_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &bmp_cfg, &bmp_dev));

    i2c_device_config_t oled_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &oled_cfg, &oled_dev));

    oled_init();
    oled_text(0, "SERRE CONNECTEE");
    oled_flush();

    bool bmp_ok = (bmp180_init() == ESP_OK);
    if (!bmp_ok) ESP_LOGE(TAG, "BMP180 non initialise : lectures invalides");

    /* ADC */
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1));
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, SOL_CHANNEL, &chan_cfg));

    ESP_LOGI(TAG, "Calibration chargee : sec=%d humide=%d (%s)",
             adc_sec, adc_humide, cal_valid() ? "valide" : "ABSENTE");

    char  line[40];
    float t_last = 0, p_last = 0;
    bool  have_last = false;
    int   bmp_fail = 0;
    int   tick = 0;
    int   tel_ms = 0;
    bool  have_meas = false;
    float cur_idx = 0;
    bool  cur_idx_valid = false, cur_bmp_valid = false;
    bool  soil_cond = false, temp_cond = false;            // conditions avec hysteresis
    bool  soil_cond_prev = false, temp_cond_prev = false;  // pour detecter les changements
    bool  soil_inv_prev = false, bmp_inv_prev = false;

    while (1) {
        /* ---- Acquisition du sol toutes les 100 ms ---- */
        int avg = soil_read_avg(16);
        bool soil_valid = (avg >= ADC_MIN_OK && avg <= ADC_MAX_OK);
        if (soil_valid) {
            soil_f = (soil_f < 0) ? (float)avg
                                  : EMA_ALPHA * avg + (1.0f - EMA_ALPHA) * soil_f;
        }

        /* ---- Boutons de calibration ---- */
        if (gpio_get_level(BTN_SEC) == 0) {
            calibrate(true);
            while (gpio_get_level(BTN_SEC) == 0) vTaskDelay(pdMS_TO_TICKS(20));
        } else if (gpio_get_level(BTN_HUM) == 0) {
            calibrate(false);
            while (gpio_get_level(BTN_HUM) == 0) vTaskDelay(pdMS_TO_TICKS(20));
        }

        if (status_ticks > 0) status_ticks--;

        /* ---- Mesures, decisions locales, affichage : une fois par seconde ---- */
        if (++tick >= 10) {
            tick = 0;

            /* BMP180 : on garde la derniere valeur valide pendant 2 echecs */
            float t, p;
            if (bmp_ok && bmp180_measure(&t, &p)) {
                t_last = t; p_last = p;
                have_last = true;
                bmp_fail = 0;
            } else if (bmp_fail < 255) {
                bmp_fail++;
            }
            bool bmp_valid = have_last && bmp_fail < 3;

            /* Indice du sol : valide seulement si lecture ET calibration valides */
            bool idx_valid = soil_valid && soil_f >= 0 && cal_valid();
            float idx = 0;
            if (idx_valid) {
                idx = 100.0f * (adc_sec - soil_f) / (float)(adc_sec - adc_humide);
                if (idx < 0.0f)   idx = 0.0f;
                if (idx > 100.0f) idx = 100.0f;
            }
            cur_idx = idx;
            cur_idx_valid = idx_valid;
            cur_bmp_valid = bmp_valid;
            have_meas = true;

            app_state_t s0;
            xSemaphoreTake(st_mx, portMAX_DELAY);
            s0 = st;
            xSemaphoreGive(st_mx);

            /* Conditions avec hysteresis (evaluees dans tous les modes) */
            if (idx_valid) {
                float off = s0.soil_thr + SOIL_HYST;
                if (off > 100.0f) off = 100.0f;
                if (idx < s0.soil_thr)  soil_cond = true;
                else if (idx >= off)    soil_cond = false;
            }
            if (bmp_valid) {
                if (t_last > s0.temp_thr)                 temp_cond = true;
                else if (t_last <= s0.temp_thr - TEMP_HYST) temp_cond = false;
            }

            /* Alertes : apparition et retour a la normale */
            char cause[80];
            if (soil_cond != soil_cond_prev) {
                soil_cond_prev = soil_cond;
                if (soil_cond) snprintf(cause, sizeof(cause), "indice sol %.0f %% < seuil %.0f %%", idx, s0.soil_thr);
                else           snprintf(cause, sizeof(cause), "indice sol %.0f %% revenu a la normale", idx);
                push_alert("soil_dry", cause, soil_cond ? "active" : "cleared");
            }
            if (temp_cond != temp_cond_prev) {
                temp_cond_prev = temp_cond;
                if (temp_cond) snprintf(cause, sizeof(cause), "temperature %.1f C > seuil %.0f C", t_last, s0.temp_thr);
                else           snprintf(cause, sizeof(cause), "temperature %.1f C revenue a la normale", t_last);
                push_alert("temp_high", cause, temp_cond ? "active" : "cleared");
            }
            if ((!soil_valid) != soil_inv_prev) {
                soil_inv_prev = !soil_valid;
                push_alert("sensor_soil",
                           soil_inv_prev ? "lecture ADC sol invalide" : "lecture ADC sol revenue valide",
                           soil_inv_prev ? "active" : "cleared");
            }
            if ((!bmp_valid) != bmp_inv_prev) {
                bmp_inv_prev = !bmp_valid;
                push_alert("sensor_bmp",
                           bmp_inv_prev ? "lecture BMP180 invalide" : "lecture BMP180 revenue valide",
                           bmp_inv_prev ? "active" : "cleared");
            }

            /* Mode AUTO : les LED suivent les conditions ; mesure invalide = LED eteinte */
            if (!s0.manual) {
                bool np = idx_valid && soil_cond;
                bool nv = bmp_valid && temp_cond;
                xSemaphoreTake(st_mx, portMAX_DELAY);
                if (!st.manual && (st.pump != np || st.vent != nv)) {
                    st.pump = np;
                    st.vent = nv;
                    state_dirty = true;
                }
                xSemaphoreGive(st_mx);
            }

            app_state_t s1;
            xSemaphoreTake(st_mx, portMAX_DELAY);
            s1 = st;
            xSemaphoreGive(st_mx);

            /* OLED */
            snprintf(line, sizeof(line), "%s P:%s V:%s",
                     s1.manual ? "MANUEL" : "AUTO", s1.pump ? "ON" : "OFF", s1.vent ? "ON" : "OFF");
            oled_text(1, line);

            if (bmp_valid) {
                snprintf(line, sizeof(line), "T: %.1f C", t_last);
                oled_text(2, line);
                snprintf(line, sizeof(line), "P: %.1f HPA", p_last);
                oled_text(3, line);
            } else {
                oled_text(2, "T: ---");
                oled_text(3, "P: ---");
            }

            if (idx_valid) {
                snprintf(line, sizeof(line), "SOL:%3.0f%% RAW:%4.0f", idx, soil_f);
            } else if (!soil_valid) {
                snprintf(line, sizeof(line), "SOL: INVALIDE");
            } else {
                snprintf(line, sizeof(line), "SOL: CALIBRER");
            }
            oled_text(4, line);
            oled_text(5, status_ticks > 0 ? status_msg : "");
            snprintf(line, sizeof(line), "WIFI:%s MQTT:%s",
                     net_wifi_ok() ? "OK" : "--", net_mqtt_ok() ? "OK" : "--");
            oled_text(6, line);
            oled_flush();

            /* Terminal : une ligne par seconde */
            char tl[12], pl[12], il[12];
            if (bmp_valid) { snprintf(tl, sizeof(tl), "%.1f", t_last); snprintf(pl, sizeof(pl), "%.1f", p_last); }
            else           { strcpy(tl, "---"); strcpy(pl, "---"); }
            if (idx_valid) snprintf(il, sizeof(il), "%.0f", idx); else strcpy(il, "---");
            ESP_LOGI(TAG, "T=%s C P=%s hPa | sol=%s %% (ADC %.0f) | %s pompe=%d vent=%d | wifi=%d mqtt=%d",
                     tl, pl, il, soil_f < 0 ? 0.0f : soil_f,
                     s1.manual ? "MANUEL" : "AUTO", s1.pump, s1.vent,
                     net_wifi_ok(), net_mqtt_ok());
        }

        /* ---- LED : appliquent l'etat courant (commande MANUEL immediate) ---- */
        bool lp, lv;
        xSemaphoreTake(st_mx, portMAX_DELAY);
        lp = st.pump;
        lv = st.vent;
        int period_s = st.period_s;
        xSemaphoreGive(st_mx);
        gpio_set_level(LED_PUMP, lp);
        gpio_set_level(LED_VENT, lv);

        /* ---- Publications ---- */
        if (net_take_new_connection()) state_dirty = true;   // resynchroniser apres (re)connexion

        if (net_mqtt_ok()) {
            if (state_dirty) publish_state();

            alert_t a;
            while (xQueueReceive(alert_q, &a, 0) == pdTRUE) {
                if (publish_alert(&a) < 0) {                 // echec : on garde l'alerte
                    xQueueSendToFront(alert_q, &a, 0);
                    break;
                }
            }

            tel_ms += 100;
            if (have_meas && tel_ms >= period_s * 1000) {
                tel_ms = 0;
                publish_telemetry(cur_bmp_valid, t_last, p_last, cur_idx_valid, cur_idx,
                                  soil_f < 0 ? 0.0f : soil_f);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    xTaskCreate(app_task, "app", 8192, NULL, 5, NULL);
}