#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <LoRaRadio.h>

// Konfiguracja trybu pracy (nadawca/odbiornik)
#define IS_SENDER true // Ustaw na `false`, aby włączyć tryb odbiornika

// Konfiguracja DHT22 (dla nadajnika)
#define DHTPIN D7       // Pin, do którego podłączony jest DHT22
#define DHTTYPE DHT22   // Typ czujnika (DHT22)
DHT dht(DHTPIN, DHTTYPE);

void handleSender();
void handleReceiver();
void encodeToBytes(float value, uint8_t *data);
float decodeToFloat(uint8_t *data);


// Konfiguracja LoRa
HardwareSerial SerialLora(D2, D8); // Serial dla modułu LoRa (RX, TX)

// Zmienna do mierzenia czasu w trybie nadawania
#define SEND_PERIOD_MS 2000 // Czas między wysyłaniem wiadomości (2 sekundy)
unsigned long lastSendTime = 0;
bool sendTemperatureNext = true; // Przełącznik wiadomości (temperatura/wilgotność)

void setup() {
    Serial.begin(115200);
    Serial.println("-- LoRa Universal Sender/Receiver --");

    // Inicjalizacja LoRa
    Serial.println("Initializing LoRa...");
    while (!loraRadio.begin(&SerialLora)) {
        Serial.println("LoRa module not ready, retrying...");
        delay(1000);
    }
    Serial.println("LoRa module ready");

    // Inicjalizacja DHT (tylko dla nadawcy)
    if (IS_SENDER) {
        dht.begin();
        Serial.println("DHT22 Initialized");
    }
}

void loop() {
    if (IS_SENDER) {
        handleSender();
    } else {
        handleReceiver();
    }
}

void handleSender() {
    // Sprawdź, czy nadszedł czas na wysyłanie danych
    if (millis() - lastSendTime >= SEND_PERIOD_MS) {
        float temperature = dht.readTemperature(); // W °C
        float humidity = dht.readHumidity();       // W %

        // Obsługa błędów odczytu
        if (isnan(temperature) || isnan(humidity)) {
            Serial.println("Failed to read from DHT sensor!");
            return;
        }

        uint8_t data[4];
        if (sendTemperatureNext) {
            encodeToBytes(temperature, data);
            loraRadio.write(data, 4);
            Serial.print("Sent Temperature: ");
            Serial.print(temperature);
            Serial.println(" °C");
        } else {
            encodeToBytes(humidity, data);
            loraRadio.write(data, 4);
            Serial.print("Sent Humidity: ");
            Serial.print(humidity);
            Serial.println(" %");
        }

        // Przełącz wiadomość i zaktualizuj czas
        sendTemperatureNext = !sendTemperatureNext;
        lastSendTime = millis();
    }
}

void handleReceiver() {
    uint8_t receivedData[4];
    int packetSize = loraRadio.read(receivedData); // Odczyt danych

    if (packetSize > 0) {
        // Dekodowanie danych
        float value = decodeToFloat(receivedData);

        // Wyświetlenie odebranych danych
        static bool expectingTemperature = true;
        if (expectingTemperature) {
            Serial.print("Received Temperature: ");
            Serial.print(value);
            Serial.println(" °C");
        } else {
            Serial.print("Received Humidity: ");
            Serial.print(value);
            Serial.println(" %");
        }

        // Przełącz oczekiwanie na następną wiadomość
        expectingTemperature = !expectingTemperature;
    }
}

void encodeToBytes(float value, uint8_t *data) {
    int32_t valueInt = (int32_t)(value * 100); // Konwersja na int z 2 miejscami po przecinku
    data[0] = (uint8_t)(valueInt >> 24);
    data[1] = (uint8_t)(valueInt >> 16);
    data[2] = (uint8_t)(valueInt >> 8);
    data[3] = (uint8_t)valueInt;
}

float decodeToFloat(uint8_t *data) {
    int32_t value = (data[0] << 24) |
                    (data[1] << 16) |
                    (data[2] << 8) |
                    data[3];
    return (float)value / 100.0; // Przywracanie oryginalnej wartości
}
