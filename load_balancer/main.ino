#include <FS.h>
#include <ESP8266WiFi.h>
#include <ModbusIP_ESP8266.h>
#include <Ticker.h>
#include "settings.h"
#include "secrets.h"

Ticker ticker;
IPAddress charger;
ModbusIP mb;

#pragma region TICKER
// **********************************
// * Ticker (System LED Blinker)    *
// **********************************

// * Blink on-board Led
void tick()
{
  // * Toggle state
  int state = digitalRead(LED_BUILTIN); // * Get the current state of GPIO1 pin
  digitalWrite(LED_BUILTIN, !state);    // * Set pin to the opposite state
}
#pragma endregion TICKER

#pragma region SETUP

void setupSerialPortForP1MeterAndLogging()
{
  Serial.begin(BAUD_RATE, SERIAL_8N1, SERIAL_FULL);
  Serial.setRxBufferSize(1024);
  Serial.println("");
  Serial.println("Swapping UART0 RX to inverted");
  Serial.flush();

  // Invert the RX serialport by setting a register value, this way the TX might continue normally allowing the serial monitor to read println's
  USC0(UART0) = USC0(UART0) | BIT(UCRXI);
  Serial.println("Serial port is ready to recieve.");
}

void startTickingLed()
{
  pinMode(LED_BUILTIN, OUTPUT);
  ticker.attach(0.5, tick);
}

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

  Serial.println("Connecting to charger...");
  auto connected = mb.connect(charger, CHARGER_MODBUS_PORT);

  if (!connected)
  {
    Serial.println("failed to connect to charger. restarting...");
    ESP.restart();
  }

  Serial.println("Connected to charger");
}

void stopTickingLed()
{
  ticker.detach();
}

void enableLed()
{
  digitalWrite(LED_BUILTIN, LOW);
}

void setup()
{
  setupSerialPortForP1MeterAndLogging();
  startTickingLed();
  connectToWiFi();
  connectToCharger();

  stopTickingLed();
  enableLed();
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

void toUint16Array(int value, uint16_t result[2])
{
  result[0] = (value >> 16) & 0xFFFF; // High 16 bits
  result[1] = value & 0xFFFF;         // Low 16 bits
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
    Serial.println("Lost connection to charger. Restarting...");
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

void setChargerCurrent(int set_current)
{
  if (!mb.isConnected(charger))
  {
    Serial.println("Lost connection to charger. Restarting...");
    ESP.restart();
    return;
  }

  uint16_t current[2];
  toUint16Array(set_current, current);

  // auto transaction = mb.writeHreg(charger, 1210, current, 2, cb, CHARGER_UNIT_ID);
  // while (mb.isTransaction(transaction))
  // {
  //   mb.task();
  //   delay(10);
  // }
}

#pragma endregion MODBUS

#pragma region P1_METER

unsigned int CRC16(unsigned int crc, unsigned char *buf, int len)
{
  for (int pos = 0; pos < len; pos++)
  {
    crc ^= (unsigned int)buf[pos]; // * XOR byte into least sig. byte of crc
                                   // * Loop over each bit
    for (int i = 8; i != 0; i--)
    {
      // * If the LSB is set
      if ((crc & 0x0001) != 0)
      {
        // * Shift right and XOR 0xA001
        crc >>= 1;
        crc ^= 0xA001;
      }
      // * Else LSB is not set
      else
        // * Just shift right
        crc >>= 1;
    }
  }
  return crc;
}

bool isNumber(char *res, int len)
{
  for (int i = 0; i < len; i++)
  {
    if (((res[i] < '0') || (res[i] > '9')) && (res[i] != '.' && res[i] != 0))
      return false;
  }
  return true;
}

int FindCharInArrayRev(char array[], char c, int len)
{
  for (int i = len - 1; i >= 0; i--)
  {
    if (array[i] == c)
      return i;
  }
  return -1;
}

long getValue(char *buffer, int maxlen, char startchar, char endchar)
{
  int s = FindCharInArrayRev(buffer, startchar, maxlen - 2);
  int l = FindCharInArrayRev(buffer, endchar, maxlen - 2) - s - 1;

  char res[16];
  memset(res, 0, sizeof(res));

  if (strncpy(res, buffer + s + 1, l))
  {
    if (endchar == '*')
    {
      if (isNumber(res, l))
        // * Lazy convert float to long
        return (1000 * atof(res));
    }
    else if (endchar == ')')
    {
      if (isNumber(res, l))
        return atof(res);
    }
  }
  return 0;
}

bool decode_telegram(int len)
{
  int startChar = FindCharInArrayRev(telegram, '/', len);
  int endChar = FindCharInArrayRev(telegram, '!', len);
  bool validCRCFound = false;

  for (int cnt = 0; cnt < len; cnt++)
  {
    Serial.print(telegram[cnt]);
  }
  Serial.print("\n");

  if (startChar >= 0)
  {
    // * Start found. Reset CRC calculation
    currentCRC = CRC16(0x0000, (unsigned char *)telegram + startChar, len - startChar);
  }
  else if (endChar >= 0)
  {
    // * Add to crc calc
    currentCRC = CRC16(currentCRC, (unsigned char *)telegram + endChar, 1);

    char messageCRC[5];
    strncpy(messageCRC, telegram + endChar + 1, 4);

    messageCRC[4] = 0; // * Thanks to HarmOtten (issue 5)
    validCRCFound = (strtol(messageCRC, NULL, 16) == currentCRC);

    if (validCRCFound)
      Serial.println(F("CRC Valid!"));
    else
      Serial.println(F("CRC Invalid!"));

    currentCRC = 0;
  }
  else
  {
    currentCRC = CRC16(currentCRC, (unsigned char *)telegram, len);
  }

  // 1-0:1.8.1(000992.992*kWh)
  // 1-0:1.8.1 = Elektra verbruik laag tarief (DSMR v4.0)
  if (strncmp(telegram, "1-0:1.8.1", strlen("1-0:1.8.1")) == 0)
  {
    // CONSUMPTION_LOW_TARIF = getValue(telegram, len, '(', '*');
  }

  // 1-0:1.8.2(000560.157*kWh)
  // 1-0:1.8.2 = Elektra verbruik hoog tarief (DSMR v4.0)
  if (strncmp(telegram, "1-0:1.8.2", strlen("1-0:1.8.2")) == 0)
  {
    // CONSUMPTION_HIGH_TARIF = getValue(telegram, len, '(', '*');
  }

  // 1-0:2.8.1(000560.157*kWh)
  // 1-0:2.8.1 = Elektra teruglevering laag tarief (DSMR v4.0)
  if (strncmp(telegram, "1-0:2.8.1", strlen("1-0:2.8.1")) == 0)
  {
    // RETURNDELIVERY_LOW_TARIF = getValue(telegram, len, '(', '*');
  }

  // 1-0:2.8.2(000560.157*kWh)
  // 1-0:2.8.2 = Elektra teruglevering hoog tarief (DSMR v4.0)
  if (strncmp(telegram, "1-0:2.8.2", strlen("1-0:2.8.2")) == 0)
  {
    // RETURNDELIVERY_HIGH_TARIF = getValue(telegram, len, '(', '*');
  }

  // 1-0:1.7.0(00.424*kW) Actueel verbruik
  // 1-0:1.7.x = Electricity consumption actual usage (DSMR v4.0)
  if (strncmp(telegram, "1-0:1.7.0", strlen("1-0:1.7.0")) == 0)
  {
    ACTUAL_CONSUMPTION = getValue(telegram, len, '(', '*');
  }

  // 1-0:2.7.0(00.000*kW) Actuele teruglevering (-P) in 1 Watt resolution
  if (strncmp(telegram, "1-0:2.7.0", strlen("1-0:2.7.0")) == 0)
  {
    ACTUAL_RETURNDELIVERY = getValue(telegram, len, '(', '*');
  }

  // 1-0:21.7.0(00.378*kW)
  // 1-0:21.7.0 = Instantaan vermogen Elektriciteit levering L1
  if (strncmp(telegram, "1-0:21.7.0", strlen("1-0:21.7.0")) == 0)
  {
    // L1_INSTANT_POWER_USAGE = getValue(telegram, len, '(', '*');
  }

  // 1-0:41.7.0(00.378*kW)
  // 1-0:41.7.0 = Instantaan vermogen Elektriciteit levering L2
  if (strncmp(telegram, "1-0:41.7.0", strlen("1-0:41.7.0")) == 0)
  {
    // L2_INSTANT_POWER_USAGE = getValue(telegram, len, '(', '*');
  }

  // 1-0:61.7.0(00.378*kW)
  // 1-0:61.7.0 = Instantaan vermogen Elektriciteit levering L3
  if (strncmp(telegram, "1-0:61.7.0", strlen("1-0:61.7.0")) == 0)
  {
    // L3_INSTANT_POWER_USAGE = getValue(telegram, len, '(', '*');
  }

  // 1-0:31.7.0(002*A)
  // 1-0:31.7.0 = Instantane stroom Elektriciteit L1
  if (strncmp(telegram, "1-0:31.7.0", strlen("1-0:31.7.0")) == 0)
  {
    // L1_INSTANT_POWER_CURRENT = getValue(telegram, len, '(', '*');
  }
  // 1-0:51.7.0(002*A)
  // 1-0:51.7.0 = Instantane stroom Elektriciteit L2
  if (strncmp(telegram, "1-0:51.7.0", strlen("1-0:51.7.0")) == 0)
  {
    // L2_INSTANT_POWER_CURRENT = getValue(telegram, len, '(', '*');
  }
  // 1-0:71.7.0(002*A)
  // 1-0:71.7.0 = Instantane stroom Elektriciteit L3
  if (strncmp(telegram, "1-0:71.7.0", strlen("1-0:71.7.0")) == 0)
  {
    // L3_INSTANT_POWER_CURRENT = getValue(telegram, len, '(', '*');
  }

  // 1-0:32.7.0(232.0*V)
  // 1-0:32.7.0 = Voltage L1
  if (strncmp(telegram, "1-0:32.7.0", strlen("1-0:32.7.0")) == 0)
  {
    // L1_VOLTAGE = getValue(telegram, len, '(', '*');
  }
  // 1-0:52.7.0(232.0*V)
  // 1-0:52.7.0 = Voltage L2
  if (strncmp(telegram, "1-0:52.7.0", strlen("1-0:52.7.0")) == 0)
  {
    // L2_VOLTAGE = getValue(telegram, len, '(', '*');
  }
  // 1-0:72.7.0(232.0*V)
  // 1-0:72.7.0 = Voltage L3
  if (strncmp(telegram, "1-0:72.7.0", strlen("1-0:72.7.0")) == 0)
  {
    // L3_VOLTAGE = getValue(telegram, len, '(', '*');
  }

  // 0-1:24.2.1(150531200000S)(00811.923*m3)
  // 0-1:24.2.1 = Gas (DSMR v4.0) on Kaifa MA105 meter
  if (strncmp(telegram, "0-1:24.2.1", strlen("0-1:24.2.1")) == 0)
  {
    // GAS_METER_M3 = getValue(telegram, len, '(', '*');
  }

  // 0-0:96.14.0(0001)
  // 0-0:96.14.0 = Actual Tarif
  if (strncmp(telegram, "0-0:96.14.0", strlen("0-0:96.14.0")) == 0)
  {
    // ACTUAL_TARIF = getValue(telegram, len, '(', ')');
  }

  // 0-0:96.7.21(00003)
  // 0-0:96.7.21 = Aantal onderbrekingen Elektriciteit
  if (strncmp(telegram, "0-0:96.7.21", strlen("0-0:96.7.21")) == 0)
  {
    // SHORT_POWER_OUTAGES = getValue(telegram, len, '(', ')');
  }

  // 0-0:96.7.9(00001)
  // 0-0:96.7.9 = Aantal lange onderbrekingen Elektriciteit
  if (strncmp(telegram, "0-0:96.7.9", strlen("0-0:96.7.9")) == 0)
  {
    // LONG_POWER_OUTAGES = getValue(telegram, len, '(', ')');
  }

  // 1-0:32.32.0(00000)
  // 1-0:32.32.0 = Aantal korte spanningsdalingen Elektriciteit in fase 1
  if (strncmp(telegram, "1-0:32.32.0", strlen("1-0:32.32.0")) == 0)
  {
    // SHORT_POWER_DROPS = getValue(telegram, len, '(', ')');
  }

  // 1-0:32.36.0(00000)
  // 1-0:32.36.0 = Aantal korte spanningsstijgingen Elektriciteit in fase 1
  if (strncmp(telegram, "1-0:32.36.0", strlen("1-0:32.36.0")) == 0)
  {
    // SHORT_POWER_PEAKS = getValue(telegram, len, '(', ')');
  }

  return validCRCFound;
}

void read_p1_hardwareserial()
{
  if (Serial.available())
  {
    memset(telegram, 0, sizeof(telegram));

    while (Serial.available())
    {
      ESP.wdtDisable();
      int len = Serial.readBytesUntil('\n', telegram, P1_MAXLINELENGTH);
      ESP.wdtEnable(1);

      processLine(len);
    }
  }
}

void processLine(int len)
{
  telegram[len] = '\n';
  telegram[len + 1] = 0;
  yield();

  bool result = decode_telegram(len + 1);

  if (!result)
  {
    Serial.println("Telegram decode failed");
    ESP.restart();
  }
}

float readActualP1MeterConsumption()
{
  // read_p1_hardwareserial();
  // auto actual_consumption_in_watts = ACTUAL_CONSUMPTION * 1000.0;
  // auto actual_returndelivery_in_watts = ACTUAL_RETURNDELIVERY * 1000.0;
  // return actual_consumption_in_watts - actual_returndelivery_in_watts;
  return 1000;
}

#pragma endregion P1_METER

#pragma region LOAD_BALANCER

float calculatePowerForCurrent(float current)
{
  return sqrt(3) * current * 230.0;
}

float calculateCurrentForPower(float power)
{
  return power / (sqrt(3) * 230.0);
}

int getAmpsForAvailablePower(float available_power)
{
  auto min_power = calculatePowerForCurrent(CHARGER_MIN_CURRENT_IN_AMPERE);
  if (available_power <= 0)
    return 0;
  if (available_power < min_power)
    return 0;
  float amps = calculateCurrentForPower(available_power);
  int floored_amps = floor(amps);
  return min(floored_amps, CHARGER_MAX_CURRENT_IN_AMPERE);
}

void balanceLoad()
{
  auto average_consumption = 0.0;
  for (int i = 0; i < LOAD_BALANCE_INTERVAL_IN_SECONDS; i++)
  {
    average_consumption += p1meter_consumptions_in_watt[i];
  }
  average_consumption /= LOAD_BALANCE_INTERVAL_IN_SECONDS;
  Serial.println("Average consumption from grid over last " + String(LOAD_BALANCE_INTERVAL_IN_SECONDS) + " seconds: " + String(average_consumption) + " W");

  auto charger_consumption = readActualChargerConsumption();
  Serial.println("Charger consumption: " + String(charger_consumption) + " W");

  auto power_consumption_without_charger = average_consumption - charger_consumption;
  Serial.println("Power consumption without charger: " + String(power_consumption_without_charger) + " W");

  auto available_power = CAPACITEITSTARIEF_IN_WATT - power_consumption_without_charger;
  Serial.println("Available power for charger: " + String(available_power) + " W");

  auto amps = getAmpsForAvailablePower(available_power);
  Serial.println("Amps for available power: " + String(amps) + " A");

  // If amps is below MIN_CHARGER_CURRENT_IN_AMPERE, set to 4, because 0 unlocks charging port
  auto set_amps = max(amps, 4);
  Serial.println("Setting charger to " + String(set_amps) + " A");

  setChargerCurrent(set_amps);
}

#pragma endregion LOAD_BALANCER

void loop()
{
  auto p1meter_consumption = readActualP1MeterConsumption();
  Serial.println("P1 meter consumption: " + String(p1meter_consumption) + " W");

  p1meter_consumptions_in_watt[measurement_index] = p1meter_consumption;
  measurement_index++;
  if (measurement_index >= LOAD_BALANCE_INTERVAL_IN_SECONDS)
  {
    measurement_index = 0;
    balanceLoad();
  }

  Serial.println("sleeping 10 seconds...");
  delay(100);
}