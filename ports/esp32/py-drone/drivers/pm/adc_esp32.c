/**
 * ESP-Drone Firmware — ADC driver, ported to IDF 5.x oneshot API.
 *
 * Original: driver/adc.h + esp_adc_cal.h (IDF 4.x, removed in IDF 5.0).
 * Replacement: esp_adc/adc_oneshot.h + esp_adc/adc_cali.h (IDF 5.x).
 */

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#include "adc_esp32.h"
#include "config.h"
#include "pm_esplane.h"

#define TAG "ADC"

#define ADC_CHANNEL     ADC_CHANNEL_1   /* GPIO2 on ADC1 */
#define ADC_ATTEN       ADC_ATTEN_DB_0
#define ADC_BITWIDTH    ADC_BITWIDTH_12
#define NO_OF_SAMPLES   30

static bool isInit = false;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t cali_handle = NULL;
static bool do_calibration = false;

void adcInit(void)
{
    if (isInit) {
        return;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &chan_cfg));

    /* Calibration — curve fitting available on ESP32-S3 */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = ADC_CHANNEL,
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle) == ESP_OK) {
        do_calibration = true;
        ESP_LOGI(TAG, "calibration OK (curve fitting)");
    } else {
        ESP_LOGW(TAG, "calibration not available, using linear approximation");
    }

    isInit = true;
}

bool adcTest(void)
{
    return isInit;
}

float analogReadVoltage(uint32_t pin)
{
    int sum = 0, raw = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_oneshot_read(adc1_handle, ADC_CHANNEL, &raw);
        sum += raw;
    }
    raw = sum / NO_OF_SAMPLES;

    if (do_calibration) {
        int mv = 0;
        adc_cali_raw_to_voltage(cali_handle, raw, &mv);
        return mv / 1000.0f;
    }
    /* Fallback: 0 dB atten on ESP32-S3, Vref ~1.1 V, 12-bit */
    return (raw / 4095.0f) * 1.1f;
}

/* Stub functions — not used in pyDrone flight path */
float adcConvertToVoltageFloat(uint16_t v, uint16_t vref) { return 0.0f; }
void adcDmaStart(void) {}
void adcDmaStop(void) {}
void adcInterruptHandler(void) {}
void adcTask(void *param) {}
