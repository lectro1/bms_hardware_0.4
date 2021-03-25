#include <Arduino.h>
#include <HardwareSerial.h>
#include <bq769x0.h> // Library for Texas Instruments bq76920 battery management IC

#define BMS_ALERT_PIN PB_12 // attached to interrupt INT0
#define BMS_BOOT_PIN PC_13  // connected to TS1 input
#define BMS_I2C_ADDRESS 0x18

#define LED PB_13

bq769x0 BMS(bq76920, BMS_I2C_ADDRESS); // battery management system object
HardwareSerial Serial3(PB_11, PB_10);

int count = 0;
int currentMillis = 0;

// #define BT_TEST
// #define DEBUG

void setup()
{
  Serial.begin(19200);
  Serial3.begin(9600);
  pinMode(LED, OUTPUT);

#ifdef BT_TEST
  while (1)
  {
    digitalWrite(LED, LOW);
    Serial3.println("3200,3210,3222,40,3301,13321,27.90,-1204");
    delay(100);
    digitalWrite(LED, HIGH);
    delay(1900);
  }
#endif

  while (BMS.begin(BMS_ALERT_PIN, BMS_BOOT_PIN) == 1)
  {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
    count++;
    if (count > 10)
    {
      while (1)
      {
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);
      }
    }
  }

  digitalWrite(LED, HIGH);

  BMS.setTemperatureLimits(-20, 45, 0, 45);
  BMS.setShuntResistorValue(1);
  BMS.setShortCircuitProtection(200000, 1000);        // delay in us
  BMS.setOvercurrentChargeProtection(8000, 200);      // delay in ms
  BMS.setOvercurrentDischargeProtection(100000, 320); // delay in ms
  BMS.setCellUndervoltageProtection(2800, 2);         // delay in s
  BMS.setCellOvervoltageProtection(3500, 2);          // delay in s

  BMS.setBalancingThresholds(0, 3300, 20); // minIdleTime_min, minCellV_mV, maxVoltageDiff_mV
  BMS.setIdleCurrentThreshold(300);
  BMS.enableAutoBalancing();
  Serial.println("Cell_1,Cell_2,Cell_3,Cell_4,Cell_5,Total,Temp,Current");
  BMS.update();
}

bool loadStatus = true;
bool chargeStatus = true;

void loop()
{
  if (Serial3.available())
  {
    String inputSerial = Serial3.readStringUntil(';');
    Serial3.print("- ");
    if (inputSerial == "LD_0")
    {
      Serial3.println("LOAD OFF!");
      loadStatus = false;
    }
    else if (inputSerial == "LD_1")
    {
      Serial3.println("LOAD ON!");
      loadStatus = true;
    }
    else if (inputSerial == "CG_1")
    {
      Serial3.println("CHARGE ON!");
      chargeStatus = true;
    }
    else if (inputSerial == "CG_0")
    {
      Serial3.println("CHARGE OFF!");
      chargeStatus = false;
    }
  }

  if (loadStatus)
  {
    BMS.enableDischarging();
  }
  else
  {
    BMS.disableDischarging();
  }

  if (chargeStatus)
  {
    BMS.enableCharging();
  }
  else
  {
    BMS.disableCharging();
  }

  if (millis() - currentMillis >= 2500)
  {
    digitalWrite(LED, LOW);
    delay(100);
    digitalWrite(LED, HIGH);
    BMS.update();
// BMS.printRegisters();
#ifndef DEBUG
    Serial3.println("Cell Voltage:");
    for (int i = 1; i <= 5; i++)
    {
      if (i == 4)
      {
        continue;
      }
      Serial3.print("   Cell ");
      if (i == 5)
      {

        Serial3.print("4");
      }
      else
      {

        Serial3.print(i);
      }
      Serial3.print(": ");
      Serial3.print(float(BMS.getCellVoltage(i)) / 1000);
      Serial3.println(" V");
      /* code */
    }
    Serial3.print("Total Voltage: ");
    Serial3.print(float(BMS.getBatteryVoltage()) / 1000);
    Serial3.println(" V");
    Serial3.print("Temperature: ");
    Serial3.print(BMS.getTemperatureDegC());
    Serial3.println(" °C");
    Serial3.print("Current: ");
    Serial3.print(float(BMS.getBatteryCurrent()) / 1000);
    Serial3.println(" A");
    Serial3.println("");
#endif

#ifdef DEBUG
    for (int i = 1; i <= 5; i++)
    {
      Serial.print(BMS.getCellVoltage(i));
      Serial.print(",");
      Serial3.print(BMS.getCellVoltage(i));
      Serial3.print(",");
      // Serial.print("Cell ");
      // Serial.print(i);
      // Serial.print(": ");
      // Serial.println(BMS.getCellVoltage(i));
    }

    // Serial.println(BMS.getBatteryVoltage());
    Serial.print(BMS.getBatteryVoltage());
    Serial.print(",");
    Serial3.print(BMS.getBatteryVoltage());
    Serial3.print(",");

    // Serial.print("Temperture: ");
    // Serial.println(BMS.getTemperatureDegC());
    Serial.print(BMS.getTemperatureDegC());
    Serial.print(",");
    Serial3.print(BMS.getTemperatureDegC());
    Serial3.print(",");
    // Serial.print("Current: ");
    Serial.println(BMS.getBatteryCurrent());
    Serial3.println(BMS.getBatteryCurrent());
    // Serial.print("Status: ");
    // Serial.println(BMS.checkStatus());
#endif
    currentMillis = millis();
  }
}
