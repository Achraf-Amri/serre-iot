#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "nvs.h"

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

/* ==============================  MAIN  ================================= */

void app_main(void)
{
    /* NVS (stockage de la calibration) */
    esp_err_t nerr = nvs_flash_init();
    if (nerr == ESP_ERR_NVS_NO_FREE_PAGES || nerr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nerr = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nerr);
    nvs_load_cal();

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

    char  line[32];
    float t_last = 0, p_last = 0;
    bool  have_last = false;
    int   bmp_fail = 0;
    int   tick = 0;

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

        /* ---- Affichage + logs une fois par seconde ---- */
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

            /* OLED */
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
            oled_text(6, "WIFI: --  MQTT: --");
            oled_flush();

            /* Terminal */
            ESP_LOGI(TAG, "T=%s | P=%s | ADC brut=%d filtre=%.0f | idx=%s | cal sec=%d hum=%d",
                     bmp_valid ? "ok" : "INVALIDE",
                     bmp_valid ? "ok" : "INVALIDE",
                     avg, soil_f < 0 ? 0.0f : soil_f,
                     idx_valid ? "ok" : "INVALIDE", adc_sec, adc_humide);
            if (bmp_valid) {
                ESP_LOGI(TAG, "   -> %.1f C, %.1f hPa", t_last, p_last);
            }
            if (idx_valid) {
                ESP_LOGI(TAG, "   -> indice sol = %.0f %%", idx);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}