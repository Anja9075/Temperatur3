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
    #define R0   10000
    #define T0   298.15
    #define R_FIXED 10000
    #define U_SUPPLY 3.3

    // ADC config (GPIO34 -> ADC1_CH6)
    #define EXAMPLE_ADC_UNIT    ADC_UNIT_1
    #define EXAMPLE_ADC_CHANNEL ADC_CHANNEL_6
    #define EXAMPLE_ADC_ATTEN   ADC_ATTEN_DB_11
    #define EXAMPLE_SAMPLES     20

    static const char *TAG = "THERMISTOR_ADC";

    static adc_oneshot_unit_handle_t init_adc(void);

 

    static double berechnenWiderstand(int adc_value)
    {
        return ((double)adc_value / 4095.0) * U_SUPPLY;
    }

    static double  berechnenvonntcthermistor(double u_ntc)
    {
        if (u_ntc <= 0.0) return 1e9;
        return R_FIXED * (U_SUPPLY / u_ntc - 1.0);
    }

    static double berechnentemperatur(double r_ntc)
    {
        double temp_kelvin = 1.0 / ((1.0 / T0) + (1.0 / BETA) * log(r_ntc / R0));
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

    static adc_oneshot_unit_handle_t adc(void)
    {
    
        // Handle für die ADC-Einheit
        adc_oneshot_unit_handle_t adc_handle = NULL;
 
        // 1. ADC-Einheit konfigurieren
        adc_oneshot_unit_init_cfg_t init_cfg = 
        {
            .unit_id = EXAMPLE_ADC_UNIT,
            .ulp_mode = false, // kein Ultra-Low-Power-Modus
        };
         ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));
 
        // 2. ADC-Kanal konfigurieren
        adc_oneshot_chan_cfg_t chan_cfg = 
        {
           .bitwidth = ADC_BITWIDTH_DEFAULT, // Standard: 12 Bit
          .atten = EXAMPLE_ADC_ATTEN,       // 11 dB Dämpfung (0–3,3 V)
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg,&unit));
 
        // 3. Rückgabe des Handles zur weiteren Nutzung
        return unit;
    }
    int samples[EXAMPLE_SAMPLES];

    while (1) 
    {
        for (int i = 0; i < EXAMPLE_SAMPLES; ++i)
        {
            int raw = 0; // adc_oneshot_read expects int*
            esp_err_t r = adc_oneshot_read(adc, EXAMPLE_ADC_CHANNEL, &raw);
            if (r != ESP_OK) 
            {
                ESP_LOGE(TAG, "adc_oneshot_read failed: %s", esp_err_to_name(r));
                raw = 0;
            }
            samples[i] = raw;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        int adc_value = berechnungdurchschnitt(samples, EXAMPLE_SAMPLES);
        double u_ntc = berechnenWiderstand(adc_value);
        double r_ntc = berechnenvonntcthermistor(u_ntc);
        double temp = berechnentemperatur(r_ntc);

            ESP_LOGI(TAG, "ADC: %4d | U(T): %.3f V | R_NTC: %.0f Ω | Temp: %.2f °C", adc_value, u_ntc, r_ntc, temp);

            vTaskDelay(pdMS_TO_TICKS(1000));
            

        ESP_ERROR_CHECK(adc_oneshot_del_unit(adc));
    }

    return 0;
}




