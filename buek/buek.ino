#include <WiFi.h>
#include <PubSubClient.h>

// Конфигурация датчиков
#define muddy 13            //D15
#define TDS_PIN 15          //D13
#define VREF 3.3            //напряжение
#define SCOUNT 30           
#define TEMPERATURE 25.0    //температура воды
#define K_VALUE 1.0

// Глобальные переменные
float ntu = 0;                  // Значение мутности
float tdsValue = 0;             // Значение TDS
int analogBuffer[SCOUNT];       // Буфер для усреднения
int analogBufferIndex = 0;

// Настройки сети
const char *ssid = "";
const char *pass = "";

// Настройки MQTT
const char *mqtt_server = "";
const int mqtt_port =       ;
const char *mqtt_user =   "";
const char *mqtt_pass =   "";

WiFiClient wclient;
PubSubClient client(wclient);

void setup() {
  Serial.begin(9600);
  pinMode(muddy, INPUT);
  Serial.println("Muddy OK!!");
  pinMode(TDS_PIN, INPUT);
  Serial.println("TDS OK!!!");
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
}

void loop() {
  if (!wifiConnect()) {
    delay(1000); //если не смогли подключиться 
    return;
  }
  
  if (!mqttConnect()) {
    delay(1000);//если не смогли подключиться 
    return;
  }
  
  readSensors();            //считываем
  printSensors();        //выводим
  publishSensors();          //отправляем
  
  client.loop();
  delay(2000); // Интервал отправки данных
}

void readSensors() {
  readMuddySensor();//читаем мутность
  readTDSSensor();//читаем проводимость
}

void readMuddySensor() {
  int raw = analogRead(muddy);            //считываем с датчика
  float voltage = raw * (VREF / 4095.0);  //считаем напряжение 
  ntu = voltage * 1000;                   //присвоить к глобальной переменной
}

void readTDSSensor() {
  //TDS я устал писать, слишком сложно
  static unsigned long lastSample = 0;
  static unsigned long lastPrint = 0;
  int raw = analogRead(TDS_PIN);
  //каждые 40 мс считываем данные
  if(millis() - lastSample > 40) {
    lastSample = millis();
    analogBuffer[analogBufferIndex] = raw;
    analogBufferIndex = (analogBufferIndex + 1) % SCOUNT; //двигаемся вперед по буферу 
  }
  //каждые 500 мс выводим
  if(millis() - lastPrint > 500) {
    lastPrint = millis();
    
    float avgVoltage = 0;
    for(int i = 0; i < SCOUNT; i++) {
      avgVoltage += analogBuffer[i] * VREF / 4095.0;
    }
    avgVoltage /= SCOUNT;

    float compensation = avgVoltage / (1.0 + 0.02*(TEMPERATURE-25.0));
    tdsValue = (133.42 * pow(compensation,3) + 
               255.86 * pow(compensation,2) + 
               857.39 * compensation) * K_VALUE;
  }
}
//вывод всего (модульная конструкция, будб ты не ладна)
void printSensors() {
  Serial.print("NTU: ");
  Serial.print(ntu);
  Serial.print(" | TDS: ");
  Serial.print(tdsValue);
  Serial.println(" ppm");
}
//отправляем в mqtt
void publishSensors() {
  if(client.connected()) {
    char payload[20];
    
    // Отправка данных мутности
    dtostrf(ntu, 5, 2, payload);
    if(client.publish("buek/sensors/muddy", payload)) {
      Serial.println("NTU отправлено");
    } else {
      Serial.println("Ошибка отправки NTU");
    }
    
    // Отправка данных TDS
    dtostrf(tdsValue, 5, 2, payload);
    if(client.publish("buek/sensors/tds", payload)) {
      Serial.println("TDS отправлено");
    } else {
      Serial.println("Ошибка отправки TDS");
    }
  }
}
//подключаем инет
bool wifiConnect() {
  if (WiFi.status() == WL_CONNECTED) return true;//ну а вдруг
  
  Serial.print("Подключение к ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  
  unsigned long start = millis();
  //выводим а4нимацию из точек если долго подключаемся(......)
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    Serial.print(".");
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi подключен");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());//потому что могу
    return true;
  } else {
    Serial.println("\nОшибка подключения WiFi!");
    return false;
  }
}
//подключаемся к mqtt
bool mqttConnect() {
  if(client.connected()) return true;//ну а вдруг
  
  Serial.print("Подключение к MQTT...");
  if(client.connect("Buek", mqtt_user, mqtt_pass)) {
    Serial.println("OK");
    return true;
  } else {
    Serial.print("Ошибка: ");
    Serial.println(client.state());
    return false;
  }
}
//нагло украденая функция,тк от скуда не подошла
void callback(char* topic, byte* payload, unsigned int length) {
  // Обработка входящих 
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  
  Serial.print("Получено сообщение [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);
}
