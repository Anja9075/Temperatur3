/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_adc/adc_continuous.h"

// -------------------------------------------------------------------
// Parameter aus dem Datenblatt des Thermistors
// -------------------------------------------------------------------
#define BETA 3435          // Beta-Wert des NTC (10k/3435)
#define R0   10000         // NTC-Widerstand bei 25°C
#define T0   298.15        // 25°C in Kelvin
#define R_FIXED 10000      // Serienwiderstand im Spannungsteiler (10kΩ)
#define U_SUPPLY 3.3       // Versorgungsspannung (Volt)

// -------------------------------------------------------------------
// Pin-Zuweisung: GPIO34 = ADC1_CHANNEL_6
// -------------------------------------------------------------------
#define ADC_CHANNEL ADC1_CHANNEL_6  

// -------------------------------------------------------------------
// Funktion: ADC-Wert lesen (Rohwert 0...4095)
// -------------------------------------------------------------------
int read_adc()
{
    return adc1_get_raw(ADC_CHANNEL); // Einen Messwert vom ADC einlesen
}

// -------------------------------------------------------------------
// Funktion: Mehrfach messen und Mittelwert bilden (stabilere Messung)
// -------------------------------------------------------------------
int read_adc_average(int samples)
{
    long sum = 0;
    for (int i = 0; i < samples; i++)     // Mehrfach messen
    {
        sum += read_adc();                // ADC-Wert addieren
        vTaskDelay(pdMS_TO_TICKS(10));    // Kurze Pause (10ms)
    }
    return (int)(sum / samples);          // Mittelwert berechnen
}

// -------------------------------------------------------------------
// Funktion: Spannung am ADC-Pin berechnen (U(T))
// -------------------------------------------------------------------
double calculate_voltage(int adc_value)
{
    // 4095 entspricht 3.3V → lineare Umrechnung
    return ((double)adc_value / 4095.0) * U_SUPPLY;
}

// -------------------------------------------------------------------
// Funktion: Thermistor-Widerstand berechnen (aus Spannungsteiler)
// -------------------------------------------------------------------
double calculate_ntc_resistance(double u_ntc)
{
    // Formel: R_NTC = R_FIXED * (U/U(T) - 1)
    return R_FIXED * (U_SUPPLY / u_ntc - 1.0);
}

// -------------------------------------------------------------------
// Funktion: Temperatur berechnen (Beta-Formel)
// -------------------------------------------------------------------
double calculate_temperature(double r_ntc)
{
    // Berechnung nach der Steinhart-Gleichung (vereinfachte Beta-Version)
    double temp_kelvin = 1.0 / ((1.0 / T0) + (1.0 / BETA) * log(r_ntc / R0));

    // Umrechnung in °C
    return temp_kelvin - 273.15;
}

// -------------------------------------------------------------------
// Hauptprogramm: Wird automatisch nach dem Start ausgeführt
// -------------------------------------------------------------------
void app_main(void)
{
    // ADC konfigurieren
    adc1_config_width(ADC_WIDTH_BIT_12);                 // 12-Bit-Auflösung
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11); // Messbereich bis 3.3V

    // Startmeldung auf der Konsole
    printf("\n=============================================\n");
    printf("   ESP32 Temperaturmessung mit NTC 10k/3435   \n");
    printf("   Eingang: GPIO34 (ADC1_CHANNEL_6)           \n");
    printf("=============================================\n\n");

    // Endlosschleife: Wiederholt Messung jede Sekunde
    while (1)
    {
        // 1️⃣ ADC-Wert lesen (mehrfach und gemittelt)
        int adc_value = read_adc_average(20);

        // 2️⃣ Spannung am ADC berechnen (U(T))
        double u_ntc = calculate_voltage(adc_value);

        // 3️⃣ NTC-Widerstand berechnen
        double r_ntc = calculate_ntc_resistance(u_ntc);

        // 4️⃣ Temperatur in °C berechnen
        double temperature = calculate_temperature(r_ntc);

        // 5️⃣ Ergebnisse in der Konsole anzeigen
        printf("ADC-Wert: %4d | U(T): %.3f V | Temperatur: %.2f °C\n",
               adc_value, u_ntc, temperature);

        // 1 Sekunde warten bis zur nächsten Messung
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
