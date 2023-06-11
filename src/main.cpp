#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <MQTT.h>
#include <ZMPT101B.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

unsigned long lastMillis = 0;

// Definisikan sensitivitas sensor zmpt101b
#define SENSITIVITY1 765.25f
#define SENSITIVITY2 650.25f
#define SENSITIVITY3 600.25f

// Mendefinisikan pin sensor voltage dengan frekuensi 50hz
ZMPT101B voltageSensor0(34, 50.0);
ZMPT101B voltageSensor1(35, 50.0);
ZMPT101B voltageSensor2(32, 50.0);

// Definisikan pin relay
int relayPins[] = {23, 22, 14, 21, 19, 18, 5, 17, 16, 4, 2, 15};

// Definisikan pin sensor arus ACS712 dan tegangan XMPT
int currentSensorPins[] = {27, 25, 26};

// deklarasi nilai current
float current0 = 0.12;
float current1 = 0.17;
float current2 = 0.17;

// Konfigurasi WiFi
const char *ssid = "pascalance";
const char *password = "wisuda2023";

// Definisikan objek MQTT dan WiFi
MQTTClient client;
WiFiClient net;

// Definisikan topik
String topic_kontrol = "kontrol_relay_phase_changer";
String topic_data = "data_sensor_phase_changer";

// Deklarasi fungsi
float *tegangan();
void connect();
void messageReceived(String &topic, String &payload);

// deklarasi fungsi freertos
void serialTask(void *pvParameters);
void publishTask(void *pvParameters);
void teganganTask(void *pvParameters);
void mqttTask(void *pvParameters);

// Definisi antrean untuk tegangan
QueueHandle_t teganganQueue;

void setup()
{
  Serial.begin(115200);

  for (int i = 0; i < 12; i++)
  {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
  }

  WiFi.begin(ssid, password);
  client.begin("broker.emqx.io", net);
  client.onMessage(messageReceived);

  connect();

  voltageSensor0.setSensitivity(SENSITIVITY1);
  voltageSensor1.setSensitivity(SENSITIVITY2);
  voltageSensor2.setSensitivity(SENSITIVITY3);

  teganganQueue = xQueueCreate(1, sizeof(float[3]));

  xTaskCreatePinnedToCore(serialTask, "Serial Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(publishTask, "Publish Task", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(teganganTask, "Tegangan Task", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 4096, NULL, 4, NULL, 1);
}

void loop()
{
}

void serialTask(void *pvParameters)
{
  while (1)
  {
    float receivedVoltages[3];

    if (xQueueReceive(teganganQueue, &receivedVoltages, portMAX_DELAY) == pdTRUE)
    {
      float voltage0 = receivedVoltages[0];
      float voltage1 = receivedVoltages[1];
      float voltage2 = receivedVoltages[2];

      Serial.print("Tegangan R: ");
      Serial.println(voltage0);
      Serial.print("Tegangan S: ");
      Serial.println(voltage1);
      Serial.print("Tegangan T: ");
      Serial.println(voltage2);
      Serial.println();
    }

    delay(5000);
  }
}

void publishTask(void *pvParameters)
{
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t interval = 5000; // Interval waktu (dalam milidetik)

  while (1)
  {
    float receivedVoltages[3];

    if (xQueueReceive(teganganQueue, &receivedVoltages, portMAX_DELAY) == pdTRUE)
    {
      float voltage0 = receivedVoltages[0];
      float voltage1 = receivedVoltages[1];
      float voltage2 = receivedVoltages[2];

      // Publish data tegangan jika sudah melewati interval waktu
      if (xTaskGetTickCount() - lastWakeTime >= interval)
      {
        StaticJsonDocument<1024> jsonDocument;
        jsonDocument["Tegangan_R"] = String(voltage0, 2);
        jsonDocument["Tegangan_S"] = String(voltage1, 2);
        jsonDocument["Tegangan_T"] = String(voltage2, 2);
        jsonDocument["Arus_R"] = String(current0);
        jsonDocument["Arus_S"] = String(current1);
        jsonDocument["Arus_T"] = String(current2);

        char jsonBuffer[1024];
        serializeJson(jsonDocument, jsonBuffer);

        client.publish(topic_data, jsonBuffer);

        lastWakeTime = xTaskGetTickCount();
      }
    }
  }
}

void teganganTask(void *pvParameters)
{
  while (1)
  {
    // Pengukuran tegangan menggunakan sensor ZMPT101B
    float voltage0 = voltageSensor0.getRmsVoltage();
    float voltage1 = voltageSensor1.getRmsVoltage();
    float voltage2 = voltageSensor2.getRmsVoltage();

    float voltages[3] = {voltage0, voltage1, voltage2};

    // Mengirim data tegangan ke antrean
    xQueueSend(teganganQueue, &voltages, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay selama 1 detik menggunakan FreeRTOS
  }
}

void mqttTask(void *pvParameters)
{
  while (1)
  {
    if (!client.connected())
    {
      connect();
    }

    client.loop();
    delay(10); // <- Fixes some issues with WiFi stability
  }
}

void connect()
{
  Serial.print("Checking WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nWiFi connected!");
  Serial.print("Connecting to MQTT broker...");
  while (!client.connect("device_1"))
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nConnected to MQTT broker!");
  client.subscribe(topic_kontrol);
  client.subscribe(topic_data);
  Serial.println("Subscribed to topics: " + topic_kontrol + ", " + topic_data);
}

void messageReceived(String &topic, String &payload)
{
  Serial.println("Incoming message: " + topic + " - " + payload);

  if (topic == topic_kontrol)
  {
    int relayPin;
    int relayState;

    int colonIndex = payload.indexOf(':'); // Mencari indeks dari tanda titik dua

    if (colonIndex != -1)
    {
      relayPin = payload.substring(0, colonIndex).toInt();    // Mengambil karakter sebelum tanda titik dua sebagai relay pin
      relayState = payload.substring(colonIndex + 1).toInt(); // Mengambil karakter setelah tanda titik dua sebagai relay state
    }
    else
    {
      // Mengatur nilai default jika tanda titik dua tidak ditemukan
      relayPin = -1;   // Misalnya, -1 untuk menandakan relay pin tidak valid
      relayState = -1; // Misalnya, -1 untuk menandakan relay state tidak valid
    }

    if (relayPin >= 0 && relayState >= 0)
    {
      // Memastikan relay pin dan relay state valid sebelum melakukan aksi
      if (relayState == 1)
      {
        digitalWrite(relayPins[relayPin], HIGH);
      }
      else
      {
        digitalWrite(relayPins[relayPin], LOW);
      }
    }
    else
    {
      // Menangani situasi jika relay pin atau relay state tidak valid
      // Misalnya, memberikan pesan kesalahan atau melakukan tindakan tertentu
    }
  }
}
