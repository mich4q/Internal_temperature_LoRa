#include <Arduino.h>
#include "LoRaRadio.h"

HardwareSerial SerialLora(D2, D8);


bool isSender = false;


int timer = 0;
#define SEND_PERIOD_MS 1000  


void enableTemperatureSensor() {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  // Włącz zegar dla ADC1
    ADC->CCR |= ADC_CCR_TSVREFE;        // Włącz sensor temperatury
}

void configureADC() {
    ADC1->SMPR2 |= ADC_SMPR2_SMP16;     // Ustaw czas próbkowania dla kanału 16
    ADC1->SQR5 |= 16;                   // Ustaw kanał 16 (sensor temperatury) w kolejce
    ADC1->CR2 |= ADC_CR2_ADON;          // Włącz ADC
    delay(1);                           // Poczekaj na stabilizację ADC
}

uint16_t readTemperatureADC() {
    ADC1->CR2 |= ADC_CR2_SWSTART;       // Rozpocznij konwersję
    while (!(ADC1->SR & ADC_SR_EOC));   // Czekaj na zakończenie konwersji
    return ADC1->DR;                    // Pobierz wynik konwersji
}

float calculateTemperature(uint16_t adcValue) {
    const uint16_t *TS_CAL1 = (uint16_t *)0x1FF800FA; // Kalibracja przy 30°C
    const uint16_t *TS_CAL2 = (uint16_t *)0x1FF800FE; // Kalibracja przy 110°C

    // Interpolacja liniowa na podstawie danych kalibracyjnych
    float temperature = ((float)(adcValue - *TS_CAL1)) * (110.0 - 30.0) /
                        (*TS_CAL2 - *TS_CAL1) + 30.0;
    return temperature;
}

void temperatureToBytes(float temperature, uint8_t *data) {
    int tempInt = (int)temperature;
    int tempFrac = (int)((temperature - tempInt) * 100);  // Część dziesiętna
    data[0] = (uint8_t)(tempInt >> 8);   // Najbardziej znaczący bajt całkowitej części
    data[1] = (uint8_t)(tempInt & 0xFF); // Mniej znaczący bajt całkowitej części
    data[2] = (uint8_t)(tempFrac >> 8);  // Najbardziej znaczący bajt części dziesiętnej
    data[3] = (uint8_t)(tempFrac & 0xFF); // Mniej znaczący bajt części dziesiętnej
}

void setup() {
    Serial.begin(115200);
    Serial.println("-- LoRa Sender/Receiver --");


    while (!loraRadio.begin(&SerialLora)) {
        Serial.println("LoRa module not ready");
        delay(1000);
    }
    Serial.println("LoRa module ready");

    if (isSender) {
        enableTemperatureSensor();
        configureADC();
    }
}

void loop() {
    uint8_t rcvData[64];  // Tablica do przechowywania danych

    // Nadajnik
    if (isSender) {
        uint16_t adcValue = readTemperatureADC();
        float temperature = calculateTemperature(adcValue);

        uint8_t temperatureData[4];
        temperatureToBytes(temperature, temperatureData); // Zamiana temperatury na dane do wysłania

        loraRadio.write(temperatureData, 4); // Wyślij temperaturę jako 4 bajty
        Serial.print("Sent Temperature: ");
        Serial.println(temperature);  // Wypisz temperaturę na serialu
        timer = millis();

        // Sprawdź, czy minęło 1 sekunda
        if (millis() - timer >= SEND_PERIOD_MS) {
            // Reset timer i ponów wysyłanie
            timer = millis();
        }
    }

    // Odbiornik
    else {
        int packetSize = loraRadio.read(rcvData);  // Odczyt danych z LoRa

        if (packetSize > 0) {  // Jeśli odebrano dane
            float receivedTemperature;
            // Zamiana odebranych bajtów na temperaturę
            int tempInt = (rcvData[0] << 8) | rcvData[1];
            int tempFrac = (rcvData[2] << 8) | rcvData[3];
            receivedTemperature = (float)tempInt + (float)tempFrac / 100.0;

            Serial.print("Received Temperature: ");
            Serial.println(receivedTemperature);  // Wypisz odebraną temperaturę
        }
    }
}
