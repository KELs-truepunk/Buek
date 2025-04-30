#define muddy 15                // D15 на esp32
#define TDS_PIN 13              // D13
#define VREF 3.3                // Напряжение питания ESP32
#define SCOUNT 30               // кол-во усредняемых значений
#define TEMPERATURE 25.0        // Температура по умолчанию
#define K_VALUE 1.0             // коэфицент для калибровки tds
          //глобальные переменные
float muddyValue = 0;           // переменная для значения мутности
int analogBuffer[SCOUNT];       // буфер для усредняемых значениЙ
int analogBufferIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;

void setup() {
  //включаем все тихонько 
  Serial.begin(9600);
  pinMode(muddy, INPUT);
  Serial.println("Muddy OK !!!");
  pinMode(TDS_PIN, INPUT);
  Serial.println("TDS OK !!!");
}
//чистый луп это круто
void loop(){
  muddySensor();
  tdsSensor();
}

void muddySensor() {
  muddyValue = analogRead(muddy);                   //  Raw
  float voltage = muddyValue * (3.3 / 4095.0);     // Voltage
  float ntu = muddyValue * (3.3 / 4095.0) * 1000; // NTU
  printf("NTU: %.0f   | Voltage: %.2f   | Raw ADC: %.0f \n",ntu, voltage, muddyValue);
  delay(1200);
}
void tdsSensor() {
  static unsigned long analogSampleTimepoint = millis(); //получаем кол-во миллисекунд с момента начала выполнения
  
  // Каждые 40 мс считываем значение с датчика
  if(millis() - analogSampleTimepoint > 40) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);
    analogBufferIndex++;//двигаемся вперед по буферу
    if(analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long time = millis();
  // Каждые 500 мс вычисляем и выводим значения
  if( millis() - time> 500) {
    time = millis();
    
    // находим средние значения для вывода
    for(int i = 0; i < SCOUNT; i++) {
      averageVoltage += analogBuffer[i] * VREF / 4095.0;
    }
    averageVoltage /= SCOUNT;

    // вычетаем температуру
    float compensationCoefficient = 1.0 + 0.02 * (TEMPERATURE - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;

    // считаем TDS
    tdsValue = (133.42 * pow(compensationVoltage, 3) +
               (255.86 * pow(compensationVoltage, 2)) +
               (857.39 * compensationVoltage) * K_VALUE);

    printf("TDS: %.2f ppm  | TDS voltage: %.0f \n",averageVoltage,tdsValue);//вывод
    averageVoltage = 0; // Сброс для следующего измерения
    delay(1200);
  }
}
