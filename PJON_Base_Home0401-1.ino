/*
   I2C (A4 - SDA, A5 - SCL)
   DS3231 BMP180

   LCD 1602 4-Pin
   D4-D7 -> D4 - D7
   D8       RS
   D9       E
   D10      Light
//----------------------
   D15(A1)      PJON-Line
//----------------------
   D3       BARIER_PIN
   D12      BEEP_PIN
   D13      AUX_BEEP_PIN
   D16(A2)  SENSOR1_PIN
*/
#define FOR_i(from, to) for(int i = (from); i < (to); i++)
#define FOR(x, from, to) for (int (x) = (from); (x) < (to); (x)++)
//===========================
#define EVERY_MS(x) \
  static uint32_t tmr;\
  bool flag = millis() - tmr >= (x);\
  if (flag) tmr += (x);\
  if (flag)
//===========================

#define BARIER_PIN 3
#define BEEP_PIN   12
#define AUX_BEEP_PIN   13
#include<Streaming.h>
//
#include <RCSwitch.h>
RCSwitch mySwitch = RCSwitch();
//                                                                     ---------- Timestamp ----------
#include "timestamp32bits.h"
timestamp32bits stamp = timestamp32bits();
//                                                                     ---------- LiquidCrystal ----------
#include <LiquidCrystal.h>
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
//                                                                     ---------- 18B20 ----------
#include <microDS18B20.h>
#define SENSOR1_PIN 16   // пины для термометров
MicroDS18B20 sensor1(SENSOR1_PIN);  // Создаем термометры без адресации

#include <Wire.h>
//                                                                     ---------- BMP085 ----------
#include <BMP085.h>
BMP085 bmp085 = BMP085();
float Temperature18B20;
int32_t Temperature = 0,
        Pressure = 0,
        Altitude = 0;
int32_t buff[10],
        avg_alt;
//                                                                     ---------- 1DS3231 ----------
#include <DS3231_Simple.h>
DS3231_Simple Clock;
DateTime MyDateAndTime;
float MyFloatTemperature;

//                                                                     ---------- PJON ----------
#define PJON_PIN 15
// АДРЕСА В СЕТИ СВОЙ И ГЛАВНЫЙ
#define MAIN_DEV_ADR       10
#define TEPL_DEV_ADR       15
#define TEPL_DEV_ADR_DUBLE 16
#define BARIER_DEV_ADR     20

#include <PJONSoftwareBitBang.h>

PJONSoftwareBitBang bus(MAIN_DEV_ADR);

struct {// Структурав для передачи времени по сети
  char id;
  uint32_t UTS;
}  now_time;

struct {// Структура СОСТОЯНИЕ ТЕПЛИЦЫ
  const char id = 'T';
  int temperature1;
  int temperature2;
  int temperature3;
  int hmd1;
  int hmd2;
  int hmd3;
  char state = 'N';
}  Teplica;
//-----------------------------------------------------------
struct {
  uint8_t f = 0;
  uint8_t count = 0;
  uint8_t numPattern = 0;
  uint16_t pattern[5] = {0B1101101010101101,
                         0B1010101010101010,
                         0B1001010010010100,
                         0B1111000011110000,
                         0B0000000000110011
                        };

} Sbeep;
uint32_t beepTimer;
bool barierF;

int8_t beep(int8_t);
//-----------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------- SETUP ---------
//-----------------------------------------------------------------------------------------------
void setup() {
  pinMode(BARIER_PIN,INPUT_PULLUP );
  pinMode(BEEP_PIN, OUTPUT);
  pinMode(AUX_BEEP_PIN, OUTPUT);
  PJON_init();

  Wire.begin();
  delay(500);
  bmp085.init();
//
  Clock.begin();
// LCD LightЗшт
  digitalWrite(10, 100);
  lcd.begin(16, 2);
//
  sensor1.setResolution(12);
  sensor1.requestTemp();
//
  mySwitch.enableTransmit(11);
  mySwitch.setPulseLength(320);
  mySwitch.setRepeatTransmit(6);
//
  Serial.begin(115200);
}

//-----------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------- LOOP ----------
//-----------------------------------------------------------------------------------------------
#define PERIOD 10000
void loop() {
  { // изолируем в блоке, макрос EVERY_MS
    EVERY_MS(PERIOD) {
      Temperature18B20 = sensor1.getTemp();    
      sensor1.requestTemp();// Запрашиваем преобразование температуры

      bmp085.getPressure(&Pressure);

      MyDateAndTime = Clock.read();
      MyFloatTemperature   = Clock.getTemperatureFloat(); // температура в корпусе (кристалл часов)

      TimeMeteoToLCD();
    }
  }
  //
  {// изолируем в блоке, макрос EVERY_MS
#define BARIER_PERIOD 50 // период опроса барьера
#define TRESHOLD 4 // сколько опросов должны дать положительный результат (200-250 миллисекунд)
 EVERY_MS(BARIER_PERIOD) { testBarier(); }
  }
  // Музыкальный автомат
  playBeep();
  // работа с шиной
  bus.update();
  bus.receive(1000);
}
/*
   -------------------------------------------------------------------
   -------------------------------------------------------------------
   -------------------------------------------------------------------
*/
/*
// Опрашиваем с частотой периода
static bool flag = true/false;
static uint32_t tmr;
if(state && flag && millis() - tmr > PERIOD)
{
  flag = false;
  tmr = millis();
}
*/
//---------------------------------------------------------------------------------------------------
void testBarier() {
  static byte barierTst = 0;
  static uint32_t tmr;
  if(millis() - tmr > BARIER_PERIOD) {
    tmr = millis();
    if(digitalRead(BARIER_PIN) && barierF == false) {
      if(++barierTst >= TRESHOLD){
        barierF = true;
        barierTst = 0;
        mySwitch.send(15755001, 24);
        beep(4);
      }
    }
    if(!digitalRead(BARIER_PIN)) {
      barierTst = 0;
    }
  }
}
//---------------------------------------------------------------------------------------------------
void PJON_init() {
  bus.set_receiver(receiver_function);
  bus.strategy.set_pin(PJON_PIN);
  bus.begin();
  //
  Serial.print("PJON - Sender's device id: ");
  Serial.print(bus.device_id());
  Serial.println(" DS18B20 tempC cyclical record sending...");
};
//---------------------------------------------------------------------------------------------------
/****************************************
    ПП ЧТЕНИЯ ДАННЫХ ИЗ СЕТИ PJON
 ****************************************/
void receiver_function(uint8_t *payload, uint16_t length, const PJON_Packet_Info &info) {
  switch ((char)payload[0]) {
    case 'R': {
        now_time.UTS = stamp.timestamp(MyDateAndTime.Year, MyDateAndTime.Month, MyDateAndTime.Day, MyDateAndTime.Hour, MyDateAndTime.Minute, MyDateAndTime.Second);
        bus.reply(&now_time, sizeof(now_time));
        Serial.println("Request Time");
        Serial.print(" Time Sinhronization from UTS   ");
        Serial.print(now_time.UTS);
        Serial.println();
        break;
      }

    case 'T': {
        memcpy(&Teplica, payload, sizeof(Teplica));
        Serial << endl << "Teplica_temp " << (float)Teplica.temperature1 / 10 << " state " << Teplica.state << endl;
        break;
      }

    default:
      Serial.print("TX id: ");
      Serial.print(info.tx.id);
      Serial.print(" RX id: ");
      Serial.print(info.rx.id);
      Serial.print(" Payload[0] = ");
      Serial.println((char)payload[0]);
      break;
  }
}
//---------------------------------------------------------------------------------------------------
void LCD2Digit(int d) {
  if (d < 10) {
    lcd.print("0");
  }
  lcd.print(d);
}
//---------------------------------------------------------------------------------------------------
void serial2Digit(int d) {
  if (d < 10) {
    Serial.print("0");
  }
  Serial.print(d);
};
//---------------------------------------------------------------------------------------------------
int8_t beep(int8_t sp = 0) {
  if (Sbeep.count) {
    return Sbeep.count;
  } else {
    Sbeep.count = 16;
    Sbeep.numPattern = sp;
    Sbeep.f = 1;
    return 0;
  }
};
//---------------------------------------------------------------------------------------------------
void playBeep() {
  if (millis() >= beepTimer && Sbeep.f) {
    beepTimer = millis() + 100;

    if (Sbeep.count--) {
      if ((Sbeep.pattern[Sbeep.numPattern] >> Sbeep.count) & 1) {
        digitalWrite(BEEP_PIN, 1);
        Serial.print("+");
      } else {
        digitalWrite(BEEP_PIN, 0);
        Serial.print("-");
      }
    } else {
      barierF = false;
      Sbeep.count = Sbeep.f = 0;  // Счётчик тактов пуст. Отыграли своё
      digitalWrite(BEEP_PIN, 0);
        Serial.println();
    }
  }
}
//---------------------------------------------------------------------------------------------------
/*
void playAuxBeep() {
  if (millis() >= beepTimer && Sbeep.f) {
    beepTimer = millis() + 100;

    if (Sbeep.count--) {
      if ((Sbeep.pattern[Sbeep.numPattern] >> Sbeep.count) & 1) {
        digitalWrite(BEEP_PIN, 1);
        Serial.print("+");
      } else {
        digitalWrite(BEEP_PIN, 0);
        Serial.print("-");
      }
    } else {
      barierF = false;
      Sbeep.count = Sbeep.f = 0;  // Счётчик тактов пуст. Отыграли своё
      digitalWrite(BEEP_PIN, 0);
        Serial.println();
    }
  }
}
*/
//---------------------------------------------------------------------------------------------------
void TimeMeteoToLCD()
{
 // line ONE
 // TIME
  lcd.setCursor(0, 0);
  LCD2Digit(MyDateAndTime.Hour);
  lcd.print(":");
  LCD2Digit(MyDateAndTime.Minute);
  lcd.print(":");
  LCD2Digit(MyDateAndTime.Second);

  lcd.setCursor(11, 0);
  if (Temperature18B20 < 10) {
    lcd.print(" ");
  }
  lcd.print(Temperature18B20 + 0.05, 1);
  lcd.print("C");

// line TWO
  lcd.setCursor(0, 1);
  lcd.println("   Hp   s      ");
  lcd.setCursor(0, 1);
  lcd.print(Pressure / 100);

  lcd.setCursor(9, 1);
  lcd.print(Teplica.state);

  lcd.setCursor(11, 1);
  if (Teplica.temperature1 < 100) {
    lcd.print(" ");
  }
  lcd.print((float)Teplica.temperature1 / 10, 1);
  lcd.print("C");
}

