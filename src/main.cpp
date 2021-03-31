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
int batteryCounter = 0;
float soc = 0;
float currentDelta = 0;
float delta = 0;

int cellVoltages[255];

void setup()
{
  Serial.begin(19200);
  Serial3.begin(9600);
  pinMode(LED, OUTPUT);

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
  BMS.setCellUndervoltageProtection(3000, 2);         // delay in s
  BMS.setCellOvervoltageProtection(3400, 2);          // delay in s

  BMS.setBalancingThresholds(0, 3300, 20); // minIdleTime_min, minCellV_mV, maxVoltageDiff_mV
  BMS.setIdleCurrentThreshold(300);
  BMS.enableAutoBalancing();
  Serial.println("Cell_1,Cell_2,Cell_3,Cell_4,Cell_5,Total,Temp,Current");
}

void loop()
{

  digitalWrite(LED, LOW);
  delay(100);
  digitalWrite(LED, HIGH);
  BMS.update();
  // BMS.printRegisters();
  BMS.enableDischarging();
  BMS.enableCharging();

  for (int i = 1; i <= 5; i++)
  {
    int cellVoltage = BMS.getCellVoltage(i);
    if (cellVoltage <= 50)
    {
      continue;
    }
    if (currentDelta == 0)
    {

      currentDelta = cellVoltage;
    }
    else
    {
      delta += abs(currentDelta - cellVoltage);
      currentDelta = cellVoltage;
    }
    cellVoltages[batteryCounter] = cellVoltage;
    batteryCounter += 1;
  }

  Serial3.print(batteryCounter);
  Serial3.print(",");

  for (int i = 0; i < batteryCounter; i++)
  {
    // Serial.print(cellVoltages[i] / 1000.0f, 3);
    // Serial.print(",");
    Serial3.print(cellVoltages[i] / 1000.0f, 3);
    Serial3.print(",");
  }

  batteryCounter = 0;

  delta = delta / 4;
  currentDelta = 0;

  float totalBatteryVoltage = BMS.getBatteryVoltage() / 1000.0f;

  Serial3.print(totalBatteryVoltage, 3);
  Serial3.print(",");
  soc = totalBatteryVoltage >= 13.4f ? 100.00 : ((totalBatteryVoltage / 13.4f) * 100.0f);
  Serial3.print(soc, 2);
  Serial3.print("%,");
  Serial3.print(delta / 1000.0f, 3);
  Serial3.print(",");
  Serial3.print(BMS.getMinCellVoltage() / 1000.0f, 3);
  Serial3.print(",");
  Serial3.print(BMS.getMaxCellVoltage() / 1000.0f, 3);
  Serial3.print(",");
  Serial3.print(BMS.getTemperatureDegC());
  Serial3.print(",");

  float batteryCurrent = BMS.getBatteryCurrent() / 1000.0f;
  Serial3.print(abs(batteryCurrent), 2);
  Serial3.print(",");
  float power = abs(batteryCurrent * totalBatteryVoltage);
  Serial3.print(power, 2);
  Serial3.print(",");
  Serial3.println(batteryCurrent > 0 ? 1 : batteryCurrent == 0 ? 0
                                                               : -1);

  // Serial.print(BMS.getBatteryVoltage()/ 1000.0f, 3);
  // Serial.print(",");
  // Serial.print(BMS.getTemperatureDegC());
  // Serial.print(",");
  // Serial.println(BMS.getBatteryCurrent());

  delay(2500);
}
