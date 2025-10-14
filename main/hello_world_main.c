/*

* SPDX-FileCopyrightText: 2010-2025 Espressif Systems (Shanghai) CO LTD

* SPDX-License-Identifier: Apache-2.0

*

* Angepasstes ESP32-Projekt: Temperaturmessung mit NTC 10k/3435 über continuous ADC.

* Verfeinert mit modernem ADC-API (aus Beispiel), Hello World-Start.

*/
 
#include <string.h>

#include <stdio.h>

#include <math.h>  // Für log()

#include "esp_log.h"

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"

#include "freertos/task.h"

#include "freertos/semphr.h"

#include "esp_adc/adc_oneshot.h"
 
// -------------------------------------------------------------------

// Parameter aus dem Datenblatt des Thermistors (unverändert)

// -------------------------------------------------------------------

/*
 * ESP32 Temperaturmessung mit NTC 10k/3435
 * Aktualisiert auf moderne ESP-IDF ADC-API (oneshot)
 *
 * SPDX-FileCopyrightText: 2010-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "sdkconfig.h"

#include "esp_adc/adc_oneshot.h"
/*
 * ESP32 Temperaturmessung mit NTC 10k/3435
 * Aktualisiert auf moderne ESP-IDF ADC-API (oneshot)
 *
 * SPDX-FileCopyrightText: 2010-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

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

static double calculate_voltage(int adc_value)
{
    return ((double)adc_value / 4095.0) * U_SUPPLY;
}

static double calculate_ntc_resistance(double u_ntc)
{
    if (u_ntc <= 0.0) return 1e9;
    return R_FIXED * (U_SUPPLY / u_ntc - 1.0);
}

static double calculate_temperature(double r_ntc)
{
    double temp_kelvin = 1.0 / ((1.0 / T0) + (1.0 / BETA) * log(r_ntc / R0));
    return temp_kelvin - 273.15;
}

static void init_adc(void)
{
    AD
}

static int calc_average(int *samples, uint32_t n)
{
    if (n == 0) return 0;
    int64_t sum = 0;
    for (uint32_t i = 0; i < n; ++i) sum += samples[i];
    return (int)(sum / (int64_t)n);
}

static adc_oneshot_unit_handle_t init_adc(void) //init_adc
{
    adc_oneshot_unit_handle_t unit = NULL;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = EXAMPLE_ADC_UNIT,
        .ulp_mode = false,
    };
    /*
     * ESP32 Temperaturmessung mit NTC 10k/3435
     * Aktualisiert auf moderne ESP-IDF ADC-API (oneshot)
     *
     * SPDX-FileCopyrightText: 2010-2025 Espressif Systems (Shanghai) CO LTD
     * SPDX-License-Identifier: Apache-2.0
     */

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

    static double berechnenWiderstand (int adc_value)
    {
        return ((double)adc_value / 4095.0) * U_SUPPLY;
    }

    static double  berechnenvonntcthermistor (double u_ntc)
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

    static adc_oneshot_unit_handle_t (void) //init_adc
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
    {
        adc_oneshot_unit_handle_t unit = NULL;
        adc_oneshot_unit_init_cfg_t init_cfg = {
            .unit_id = EXAMPLE_ADC_UNIT,
            .ulp_mode = false,
        };
        /*
         * ESP32 Temperaturmessung mit NTC 10k/3435
         * Aktualisiert auf moderne ESP-IDF ADC-API (oneshot)
         *
         * SPDX-FileCopyrightText: 2010-2025 Espressif Systems (Shanghai) CO LTD
         * SPDX-License-Identifier: Apache-2.0
         */

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

        static double berechnungwiderstand2(int adc_value)
        {
            return ((double)adc_value / 4095.0) * U_SUPPLY;
        }

        static double calculate_ntc_resistance(double u_ntc)
        {
            if (u_ntc <= 0.0) return 1e9;
            return R_FIXED * (U_SUPPLY / u_ntc - 1.0);
        }

        static double calculate_temperature(double r_ntc)
        {
            double temp_kelvin = 1.0 / ((1.0 / T0) + (1.0 / BETA) * log(r_ntc / R0));
            return temp_kelvin - 273.15;
        }

        static int calc_average(int *samples, uint32_t n)
        {
            if (n == 0) return 0;
            int64_t sum = 0;
            for (uint32_t i = 0; i < n; ++i) sum += samples[i];
            return (int)(sum / (int64_t)n);
        }

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

        void app_main(void)
        {
            ESP_LOGI(TAG, "Starting ESP32 Thermistor Temperature Measurement (oneshot ADC)");

            adc_oneshot_unit_handle_t adc = init_adc();
            int samples[EXAMPLE_SAMPLES];

            while (1) {
                for (int i = 0; i < EXAMPLE_SAMPLES; ++i) {
                    int raw = 0; // adc_oneshot_read expects int*
                    esp_err_t r = adc_oneshot_read(adc, EXAMPLE_ADC_CHANNEL, &raw);
                    if (r != ESP_OK) {
                        ESP_LOGE(TAG, "adc_oneshot_read failed: %s", esp_err_to_name(r));
                        raw = 0;
                    }
                    samples[i] = raw;
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                int adc_value = calc_average(samples, EXAMPLE_SAMPLES);
                double u_ntc = calculate_voltage(adc_value);
                double r_ntc = calculate_ntc_resistance(u_ntc);
                double temp = calculate_temperature(r_ntc);

                ESP_LOGI(TAG, "ADC: %4d | U(T): %.3f V | R_NTC: %.0f Ω | Temp: %.2f °C",
                         adc_value, u_ntc, r_ntc, temp);

                vTaskDelay(pdMS_TO_TICKS(1000));
            }

            ESP_ERROR_CHECK(adc_oneshot_del_unit(adc));
        }


