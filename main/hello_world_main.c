#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
#include "esp_log.h"
#include "sdkconfig.h"
 
#include "esp_adc/adc_oneshot.h"
 
// Thermistor parameter
#define BETA 3435
#define R0_NTC   10000
#define T0   298.15
#define R 10000
#define UVersorger 3.3
 
// ADC config (GPIO34 -> ADC1_CH6)
#define EXAMPLE_ADC_UNIT    ADC_UNIT_1
#define EXAMPLE_ADC_CHANNEL ADC_CHANNEL_6
#define EXAMPLE_ADC_ATTEN   ADC_ATTEN_DB_11
#define EXAMPLE_SAMPLES     20
 
static const char *TAG = "THERMISTOR_ADC";
 
// Initialisiert die ADC-Oneshot-Einheit und gibt das Handle zurück.
static adc_oneshot_unit_handle_t init_adc(void)
{
    adc_oneshot_unit_handle_t unit = NULL;
 
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = EXAMPLE_ADC_UNIT,
        .ulp_mode = false,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &unit));
 
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(unit, EXAMPLE_ADC_CHANNEL, &chan_cfg));
 
    return unit;
}

static bool init_calibration(adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id      = EXAMPLE_ADC_UNIT,
        .atten        = EXAMPLE_ADC_ATTEN,
        .bitwidth     = ADC_BITWIDTH_DEFAULT,
        .default_vref = 0,  // nicht benötigt
    };

    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC-Kalibrierung erfolgreich initialisiert (Line Fitting).");
        *out_handle = handle;
        return true;
    } else {
        ESP_LOGW(TAG, "ADC-Kalibrierung NICHT verfügbar. Es wird ohne Kalibration gearbeitet.");
        return false;
    }
}
 
/*static double berechneEingangsspannung(int adc_value)
{
    return ((double)adc_value / 4095.0) * 6;
}
*/ 
static double berechnenvonntcthermistor(double u_ntc)
{
    if (u_ntc <= 0.0) return 1e9;
    return R * (UVersorger / u_ntc - 1.0);
}
 
static double berechnentemperatur(double r_ntc)
{
    double temp_kelvin = 1.0 / ((1.0 / T0) + (1.0 / BETA) * log(r_ntc / R0_NTC));
    return temp_kelvin - 273.15;
}
 
static int berechnungdurchschnitt (int *samples, uint32_t n)
{
    if (n == 0) return 0;
    int64_t sum = 0;
    for (uint32_t i = 0; i < n; ++i) sum += samples[i];
    return (int)(sum / (int64_t)n);
}
 
void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32 Thermistor Temperature Measurement (oneshot ADC)");
 
    adc_oneshot_unit_handle_t adc = init_adc();

    adc_cali_handle_t cali_handle = NULL;
    bool cal_enabled = init_calibration(&cali_handle);
 
    int samples[EXAMPLE_SAMPLES];
 
    while (1)
    {
        for (int i = 0; i < EXAMPLE_SAMPLES; ++i)
        {
            int raw = 0;
            esp_err_t r = adc_oneshot_read(adc, EXAMPLE_ADC_CHANNEL, &raw);
            if (r != ESP_OK)
            {
                ESP_LOGE(TAG, "adc_oneshot_read failed: %s", esp_err_to_name(r));
                raw = 0;
            }
            samples[i] = raw;
            vTaskDelay(pdMS_TO_TICKS(30));
        }
 
        int adc_value = berechnungdurchschnitt(samples, EXAMPLE_SAMPLES);
        int voltage_mv;
        //double u_ntc = berechnenEinagnagsspannung(adc_value);

        if (cal_enabled)
        {
            adc_cali_raw_to_voltage(cali_handle, adc_value, &voltage_mv);
        }
        else
        {
            // einfache Formel ohne Kalibrierung
            voltage_mv = (adc_value * 3300) / 4095;
        }

        double u_ntc = voltage_mv / 1000.0;
 
        //ESP_LOGI(TAG, "ADC: %d, U_NTC: %.3f V", adc_value, u_ntc);
 
        double r_ntc = berechnenvonntcthermistor(u_ntc);
        double temp = berechnentemperatur(r_ntc);
 
        ESP_LOGI(TAG, "ADC: %4d | U(T): %.3f V | R_NTC: %.0f Ω | Temp: %.2f °C",
         adc_value, u_ntc, r_ntc, temp);
 
    }
 
   
}