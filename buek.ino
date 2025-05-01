#include <WiFi.h>
#include <PubSubClient.h>

// Датчики
#define muddy 15             // D15 на esp32
#define TDS_PIN 13           // D13
#define VREF 3.3             // Напряжение питания ESP32
#define SCOUNT 30            // кол-во усредняемых значений
#define TEMPERATURE 25.0     // Температура по умолчани
#define K_VALUE 1.0          // коэфицент для калибровки tds

// Глобальные переменные
float muddyValue = 0;         // значение мутности
int analogBuffer[SCOUNT];     //буфер для усреднения значений ppm
int analogBufferIndex = 0;
float voltage = 0;
float tdsValue = 0;           //значения ppm

//данные для  Wi-Fi
const char *ssid = "DECO222";
const char *pass = "66065401!";

// данные для MQTT
const char *mqtt_server = "m7.wqtt.ru";
const int mqtt_port = 15128;
const char *mqtt_user = "u_QPTT9R";
const char *mqtt_pass = "fAhauOpC";
//инициализация
WiFiClient wclient;
PubSubClient client(wclient);

void setup() {
  Serial.begin(9600);
  pinMode(muddy, INPUT);
  Serial.println("Muddy OK!!!");
  pinMode(TDS_PIN, INPUT);
  Serial.println("TDS OK!!!");
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}
//чистый луп это круто
void loop() {
  wifiConnect();
  if (WiFi.status() == WL_CONNECTED) {
    mqttConnect();
    
    float ntu = muddySensor();
    tdsSensor();
    
    if(client.connected()) {
      client.publish("buek/sensors/muddy", String(ntu).c_str());
      client.publish("buek/sensors/tds", String(tdsValue).c_str());
    }
    
    client.loop();
  }
  delay(1000);
}

float muddySensor() {
  muddyValue = analogRead(muddy);                 //RAW
  float voltage = muddyValue * (VREF / 4095.0);   //Voltagee
  float ntu = voltage * 1000;                     //NTU
  //вывод NTU
  printf("NTU: %0.f | Voltage: %2.fV",ntu,voltage);
  return ntu;//зачем я так сделал ведь все равно через глобальные переменные делаю, и я знаю что есть указатели
}

void tdsSensor() {
  static unsigned long Timepoint = millis();//получаем кол-во миллисекунд с момента начала выполнения
  static unsigned long printTimepoint = millis();
  // Каждые 40 мс считываем значение с датчика
  if(millis() - Timepoint > 40) {
    Timepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);
    analogBufferIndex = (analogBufferIndex + 1) % SCOUNT;//двигаемся вперед по буферу
  }
 // Каждые 500 мс вычисляем и выводим значения
  if(millis() - printTimepoint > 500) {
    printTimepoint = millis();
    
    voltage = 0;
    // находим средние значения для вывода
    for(int i = 0; i < SCOUNT; i++) {
      voltage += analogBuffer[i] * VREF / 4095.0;
    }
    voltage /= SCOUNT;
    // вычетаем температуру
    float compensationVoltage = voltage / (1.0 + 0.02*(TEMPERATURE-25.0));
      // считаем TDS
    tdsValue = (133.42*pow(compensationVoltage,3) + 255.86*pow(compensationVoltage,2) + 857.39*compensationVoltage) * K_VALUE;
    //вывод ppm
    printf("TDS: %0.f ppm | Voltage: %2.fV",tdsValue,voltage);
  }
}
//Подключение к Интернету
void wifiConnect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Подключение к ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    
    Serial.println("WiFi connected");
    Serial.print("Ваш IP: ");//потому что могу
    Serial.println(WiFi.localIP());
  }
}
//подключение к MQTT
void mqttConnect() {
  if (!client.connected()) {
    Serial.print("Подключение к MQTT...");
    
    if (client.connect("Buek", mqtt_user, mqtt_pass)) {
      Serial.println("OK");
      client.subscribe("buek/sensor");
    } else {
      Serial.print("Ошибка, rc=");
      Serial.println(client.state());
    }
  }
}
//обработка всех сообщений
void callback(char* topic, byte* payload, unsigned int length) {
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  //вывод приходящих сообщений
  Serial.print("Message [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if(String(topic) == "buek/message" && String(message) == "45") {
    //оброботка сообщений в топике
  }
}
