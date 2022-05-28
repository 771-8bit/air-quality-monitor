#include <TimerTC3.h>
int32_t ms_counter[7];

#include"TFT_eSPI.h"
#include"Free_Fonts.h"
#define LCD_BACKLIGHT (72Ul)
TFT_eSPI tft;


uint16_t TIME_RANGE[7] = {1, 5, 30, 60, 720, 1440, 4320}; //320個のデータをとるのに何分かけるか
uint16_t time_range = 0;
bool time_range_change = false;
int32_t history[5][7][320];

byte ReadCO2[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
byte SelfCalOn[9]  = {0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6};
byte SelfCalOff[9] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86};
byte retval[9];


#include "Zanshin_BME680.h"  // Include the BME680 Sensor library
BME680_Class BME680;  ///< Create an instance of the BME680 class
float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel) {

  static float Altitude;
  Altitude =
    44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}  // of method altitude()

void setup() {


  pinMode(WIO_KEY_A, INPUT_PULLUP);
  pinMode(WIO_KEY_B, INPUT_PULLUP);
  pinMode(WIO_KEY_C, INPUT_PULLUP);
  attachInterrupt(WIO_KEY_A, power_control, FALLING );
  attachInterrupt(WIO_KEY_B, range_increment, FALLING );
  attachInterrupt(WIO_KEY_C, range_decrement, FALLING );

  tft.begin();
  tft.setRotation(3);

  tft.fillScreen(TFT_BLACK);

  tft.drawLine(0,   0, 319,   0, TFT_WHITE);
  tft.drawLine(0,  50, 319,  50, TFT_WHITE);
  tft.drawLine(0, 100, 319, 100, TFT_WHITE);
  tft.drawLine(  0, 0,   0, 100, TFT_WHITE);
  tft.drawLine(160, 0, 160, 100, TFT_WHITE);
  tft.drawLine(319, 0, 319, 100, TFT_WHITE);

  delay(2000);

  Serial1.begin(9600);
  Serial1.write(SelfCalOn, sizeof SelfCalOn);

  Serial.print(F("Starting I2CDemo example program for BME680\n"));
  Serial.print(F("- Initializing BME680 sensor\n"));
  while (!BME680.begin(I2C_STANDARD_MODE)) {
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(1000);
  }
  Serial.print(F("- Setting 16x oversampling for all sensors\n"));//測定しすぎると発熱するので4回だけにした
  BME680.setOversampling(TemperatureSensor, Oversample4);
  BME680.setOversampling(HumiditySensor, Oversample4);
  BME680.setOversampling(PressureSensor, Oversample4);
  Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  BME680.setIIRFilter(IIR4);
  Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));
  BME680.setGas(320, 150);

  for (int i = 0; i < 7; i++) {
    ms_counter[i] = 0;
  }
  TimerTc3.initialize(1000);
  TimerTc3.attachInterrupt(timerIsr);

  for (int idata = 0; idata < 5; idata++) {
    for (int irange = 0; irange < 7; irange++) {
      for (int itime = 0; itime < 320; itime++) {
        history[idata][irange][itime] = 0;
      }
    }
  }

}

void loop() {
  //センサ読み取り
  static int32_t readdata[5]={400,2000,100000,50000};
  //co2,temp,Pressure,Humidity

  bool refresh = false;
  for (int irange = 0; irange < 7; irange++) {
    
    if (ms_counter[irange] > TIME_RANGE[irange] * 60 * 1000 / 320) {
      ms_counter[irange] = 0;

      //CO2は1minペースで取得
      if (irange == 0) {
        //UARTでデータ取得
        Serial1.write(ReadCO2, sizeof ReadCO2);
        delay(1);
        Serial1.readBytes((char *)retval, sizeof retval);
        
        if (retval[0] == 134) {
          readdata[0] = ((int32_t)retval[1] * 256 + (int32_t)retval[2]);
          uint16_t uartco2 = retval[1] * 256 + retval[2];
          Serial.println(uartco2);
        } else if (retval[8] == 134) {
          readdata[0] = ((int32_t)retval[0] * 256 + (int32_t)retval[1]);
          uint16_t uartco2 = retval[0] * 256 + retval[1];
          Serial.println(uartco2);
        } else {
          readdata[0] = ((int32_t)retval[2] * 256 + (int32_t)retval[3]);
          uint16_t uartco2 = retval[2] * 256 + retval[3];
          Serial.println(uartco2);
        }

        /*
          for (int i = 0; i < 9; i++) {
          Serial.print(retval[i]);
          Serial.print(",");
          }
        */
      }

      //BME680は30minペースで取得
      if (irange == 2) {
        //temp, humidity, pressure, gas;
        BME680.getSensorData(readdata[1], readdata[3], readdata[2], readdata[4]);
      }
      
      //画面更新用フラグ
      if (irange == time_range)refresh = true;

      //グラフ用データ処理
      for (int idata = 0; idata < 5; idata++) {
        for (int itime = 0; itime < 319; itime++) {
          history[idata][irange][itime] = history[idata][irange][itime + 1];
        }
        history[idata][irange][319] = readdata[idata];
      }
    }
  }

  //描画開始
  if (refresh) {
    //グラフ描画
    for (int idata = 0; idata < 5; idata++) {
      for (int itime = 0; itime < 319; itime++) {
        tft.drawLine(itime,   101, itime,   239, TFT_BLACK);
        tft.drawPixel(itime, 230 - ((history[0][time_range][itime] - 400) / 10), 0xE38E); //CO2
        tft.drawPixel(itime, 240 - ((history[1][time_range][itime] / 10 - 100) / 2), TFT_YELLOW); //temp
        tft.drawPixel(itime, 170 - (((history[2][time_range][itime] / 100) - 1000) * 7 / 10), TFT_WHITE); //pressure
        tft.drawPixel(itime, 240 - (((history[3][time_range][itime] / 100) - 300) * 14 / 40), 0x4BB); //humidity
      }
    }

    //文字描画
    tft.setTextColor(TFT_BLACK);
    tft.setFreeFont(FSS18);
    String co2_str;
    if (readdata[0] < 1000) {
      co2_str = String(" ") + readdata[0] + String("ppm");
    } else {
      co2_str = readdata[0] + String("ppm");
    }
    if (readdata[0] > 1000) {
      tft.fillRect(1, 1, 159, 49, TFT_RED);
    } else {
      tft.fillRect(1, 1, 159, 49, 0xE38E);
    }
    tft.drawString(co2_str, 9, 9);

    String temp_str     = String("   ") + (readdata[1] /  100) + String(".") + (readdata[1] %  10) + String(" c");
    if (readdata[1] /  100 > 29 || readdata[1] /  100 < 15) {
      tft.fillRect(161, 1, 318, 49, TFT_YELLOW);
    } else {
      tft.fillRect(161, 1, 318, 49, 0xFDA9);
    }
    tft.drawString(temp_str, 169, 9);
    tft.drawCircle(270, 15, 3, TFT_BLACK);
    tft.drawCircle(270, 15, 4, TFT_BLACK);

    String pressure_str = (readdata[2] /  100) + String("hPa");
    tft.fillRect(1, 51, 159, 49, 0xE71C);
    tft.drawString(pressure_str, 9, 59);

    String humidity_str = String("   ") + (readdata[3] / 1000) + String(".") + (readdata[3] % 10) + String("%");
    if (readdata[3] /  1000 > 60 || readdata[3] /  1000 < 40) {
      tft.fillRect(161, 51, 318, 49, TFT_BLUE);
    } else {
      tft.fillRect(161, 51, 318, 49, 0x86BF);
    }
    tft.drawString(humidity_str, 169, 59);


    if (time_range_change) {
      time_range_change = false;
      ms_counter[time_range] = (TIME_RANGE[time_range] * 60 * 1000 / 320) - 1000;

      tft.setTextColor(TFT_WHITE);
      tft.setFreeFont(FSS18);

      String div_str;
      if (TIME_RANGE[time_range] > 60) {
        div_str = (TIME_RANGE[time_range] / 60) + String("h ") + (TIME_RANGE[time_range] * 60 / 320) + 1 + String("s/div");
      } else {
        div_str = (TIME_RANGE[time_range]) + String("min ") + (TIME_RANGE[time_range] * 60 / 320) + 1 + String("s/div");
      }
      tft.drawString(div_str, 70, 150);
    }
  }
}

void timerIsr()
{
  for (int i = 0; i < 7; i++) {
    ms_counter[i]++;
  }
}

void range_increment() {
  if (time_range < 6) {
    time_range++;
    ms_counter[time_range] = TIME_RANGE[time_range] * 60 * 1000 / 320;
  }
  time_range_change = true;
}

void range_decrement() {
  if (time_range > 0) {
    time_range--;
    ms_counter[time_range] = TIME_RANGE[time_range] * 60 * 1000 / 320;
  }
  time_range_change = true;

}

void power_control() {
  if (digitalRead(LCD_BACKLIGHT)) {
    digitalWrite(LCD_BACKLIGHT, LOW);
  } else {
    digitalWrite(LCD_BACKLIGHT, HIGH);
  }
}
