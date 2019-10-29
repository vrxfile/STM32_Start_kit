#define BLYNK_PRINT Serial

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750FVI.h>
#include <TLC59108.h>
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <BlynkSimpleEthernet.h>

char auth[] = "kTql2zkTiKn37jtE5M97I_t0wu1aooFG";
IPAddress blynk_ip(139, 59, 206, 133);

// Встроенный светодиод
const uint8_t LED_PIN = LED_BUILTIN;

// Выходы реле
#define RELAY_PIN_1   D2
#define RELAY_PIN_2   D3

// Таймер
#define TIMER_INTERVAL 5000L
BlynkTimer timer;

// Константы для I2C хаба
#define I2C_HUB_ADDR        0x70
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08

// I2C порт 0x07 - выводы D4 (SDA), D5 (SCL)
// I2C порт 0x06 - выводы D6 (SDA), D7 (SCL)
// I2C порт 0x05 - выводы D8 (SDA), D9 (SCL)
// I2C порт 0x04 - выводы D10 (SDA), D11 (SCL)
// I2C порт 0x03 - выводы D12 (SDA), D13 (SCL)

// Датчик освещенности
BH1750FVI bh1750;

// Датчик температуры/влажности и атмосферного давления
Adafruit_BME280 bme280;

// Датчик влажности почвы емкостной
const float air_value    = 795.0;
const float water_value  = 457.0;
const float moisture_0   = 0.0;
const float moisture_100 = 100.0;

// Входы датчика влажности почвы
#define SOIL_HUM_PIN    A0
#define SOIL_TEMP_PIN   A1

// Модуль RGB светодиода
#define HW_RESET_PIN 0 // Только програмнный сброс
#define I2C_ADDR TLC59108::I2C_ADDR::BASE
TLC59108 leds(I2C_ADDR + 7); // Без перемычек добавляется 3 бита адреса

void myTimerEvent()
{
  Serial.println();
  Serial.println("Timer event: " + String(millis()));

  // Считывание датчика освещенности
  delay(10); setBusChannel(0x07); delay(10);
  float light = bh1750.getAmbientLight();
  Serial.println("Light = " + String(light, 0) + " lx");
  Blynk.virtualWrite(V3, String(light, 0)); delay(10);

  // Считывание датчика температуры/влажности/давления
  delay(10); setBusChannel(0x06); delay(10);
  float air_temp = bme280.readTemperature();
  float air_hum = bme280.readHumidity();
  float air_press = bme280.readPressure() * 7.5006F / 1000.0F;
  Serial.println("Air temperature = " + String(air_temp, 1) + " *C");
  Serial.println("Air humidity = " + String(air_hum, 1) + " %");
  Serial.println("Air pressure = " + String(air_press, 1) + " mm Hg");
  Blynk.virtualWrite(V0, String(air_temp, 1)); delay(10);
  Blynk.virtualWrite(V1, String(air_hum, 1)); delay(10);
  Blynk.virtualWrite(V2, String(air_press, 1)); delay(10);

  // Считывание датчика почвы
  float adc1_1 = analogRead(SOIL_HUM_PIN);
  float adc1_2 = analogRead(SOIL_TEMP_PIN);
  float soil_hum = map(adc1_1, air_value, water_value, moisture_0, moisture_100);
  float soil_temp = adc1_2 / 10.0;
  Serial.println("Soil temperature = " + String(soil_temp, 1) + " *C");
  Serial.println("Soil moisture = " + String(soil_hum, 1) + " %");
  Blynk.virtualWrite(V4, String(soil_temp, 1)); delay(10);
  Blynk.virtualWrite(V5, String(soil_hum, 1)); delay(10);
}

void setup()
{
  // Инициализация последовательного порта
  Serial.begin(115200);
  while (!Serial) {
  }

  // Инициализация I2C и I2C хаба
  Wire.begin();
  Wire.setClock(100000L);

  // Инициализация датчика BH1750
  delay(10); setBusChannel(0x07); delay(10);
  bh1750.begin();
  bh1750.setMode(Continuously_High_Resolution_Mode);

  // Инициализация датчика BME280
  delay(10); setBusChannel(0x06); delay(10);
  bool bme_status = bme280.begin();
  if (!bme_status)
    Serial.println("Could not find a valid BME280 sensor, check wiring!");

  // Инициализация RGB модуля
  delay(10); setBusChannel(0x05); delay(10);
  leds.init(HW_RESET_PIN);
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);

  // Инициализация встроенного светодиода
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Инициализация выходов реле
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  digitalWrite(RELAY_PIN_1, LOW);
  digitalWrite(RELAY_PIN_2, LOW);

  // Инициализация Ethernet и подключение к серверу Blynk
  Serial.println("Connecting...");
  Blynk.begin(auth, blynk_ip, 8442);
  Serial.println("Connected!");
  Serial.println();

  // Инициализация таймера
  timer.setInterval(TIMER_INTERVAL, myTimerEvent);
}

void loop()
{
  Blynk.run();
  timer.run();
}

// Управление реле #1 с Blynk
BLYNK_WRITE(V10)
{
  int relay_ctl = param.asInt();
  Serial.println("Relay power #1: " + String(relay_ctl));
  digitalWrite(RELAY_PIN_1, relay_ctl);
}

// Управление реле #2 с Blynk
BLYNK_WRITE(V11)
{
  int relay_ctl = param.asInt();
  Serial.println("Relay power #2: " + String(relay_ctl));
  digitalWrite(RELAY_PIN_2, relay_ctl);
}

// Управление RGB модулем с Blynk (белые светодиоды)
BLYNK_WRITE(V12)
{
  int led_ctl = param.asInt();
  byte pwm = 0x00;
  Serial.println("White LED power: " + String(led_ctl));
  if (led_ctl)
    pwm = 0xFE;
  else
    pwm = 0x00;
  delay(10); setBusChannel(0x05); delay(10);
  leds.setBrightness(0, pwm);
  leds.setBrightness(6, pwm);
}

// Управление RGB модулем с Blynk (УФ светодиоды)
BLYNK_WRITE(V13)
{
  int led_ctl = param.asInt();
  byte pwm = 0x00;
  Serial.println("UV LED power: " + String(led_ctl));
  if (led_ctl)
    pwm = 0xFE;
  else
    pwm = 0x00;
  delay(10); setBusChannel(0x05); delay(10);
  leds.setBrightness(1, pwm);
  leds.setBrightness(4, pwm);
}

// Управление RGB модулем с Blynk (RGB светодиод)
BLYNK_WRITE(V14)
{
  byte r = param[0].asInt();
  byte g = param[1].asInt();
  byte b = param[2].asInt();
  Serial.println("Red: " + String(r));
  Serial.println("Green: " + String(g));
  Serial.println("Blue: " + String(b));
  delay(10); setBusChannel(0x05); delay(10);
  leds.setBrightness(3, r);
  leds.setBrightness(2, g);
  leds.setBrightness(5, b);
}

// Функция установки нужного выхода I2C
bool setBusChannel(uint8_t i2c_channel)
{
  if (i2c_channel >= MAX_CHANNEL)
  {
    return false;
  }
  else
  {
    Wire.beginTransmission(I2C_HUB_ADDR);
    Wire.write(i2c_channel | EN_MASK);
    Wire.endTransmission();
    return true;
  }
}
