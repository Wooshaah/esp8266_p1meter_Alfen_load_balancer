#include <FS.h>
#include <ESP8266WiFi.h>
#include <ModbusIP_ESP8266.h>
#include "settings.h"
#include "secrets.h"

IPAddress charger = IPAddress();
ModbusIP mb;

#pragma region SETUP

void connectToWiFi()
{
  WiFi.begin(SSID, PASS);

  long startTime = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    if (millis() - startTime > WIFI_TIMEOUT)
    {
      Serial.println("Failed to connect to WiFi. Restarting...");
      ESP.restart();
    }
  }
  Serial.println("Connected to WiFi");
}

void connectToCharger()
{
  charger.fromString(CHARGER_ADDRESS);
  mb.client();

  long startTime = millis();
  Serial.println("Connecting to charger...");
  auto connected = mb.connect(charger, CHARGER_MODBUS_PORT);

  if (!connected)
  {
    Serial.println("failed to connect to charger. restarting...");
    ESP.restart();
  }

  Serial.println("Connected to charger");
}

void setup()
{
  Serial.begin(115200);
  connectToWiFi();
  connectToCharger();

  Serial.println("Setup completed");
}

#pragma endregion SETUP

#pragma region HELPERS

float toFloat(uint16_t high, uint16_t low)
{
  uint32_t combined = (static_cast<uint32_t>(high) << 16) | low;
  float result;
  memcpy(&result, &combined, sizeof(result));
  return result;
}

#pragma endregion HELPERS

#pragma region MODBUS

bool cb(Modbus::ResultCode event, uint16_t transactionId, void *data)
{
  if (event != Modbus::EX_SUCCESS)                       // If transaction got an error
    Serial.printf("Modbus error result: %02X\n", event); // Display Modbus error code
  if (event == Modbus::EX_TIMEOUT)
  {                         // If Transaction timeout took place
    mb.disconnect(charger); // Close connection to slave and
    mb.dropTransactions();  // Cancel all waiting transactions
  }
  return true;
}

float readActualChargerConsumption()
{
  uint16_t actual_consumption[2];
  if (!mb.isConnected(charger))
  {
    Serial.println("Lost connection to charger. Reconnecting...");
    ESP.restart();
    return -1;
  }

  auto transaction = mb.readHreg(charger, 344, actual_consumption, 2, cb, CHARGER_UNIT_ID);
  while (mb.isTransaction(transaction))
  {
    mb.task();
    delay(10);
  }

  return toFloat(actual_consumption[0], actual_consumption[1]);
}

#pragma endregion MODBUS

void loop()
{
  auto charger_consumption = readActualChargerConsumption();

  Serial.println("Charger consumption: " + String(charger_consumption) + " W");
  Serial.println("sleeping 10 seconds...");
  delay(10000);
}