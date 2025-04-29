#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>

// WiFi credentials
const char* ssid = "LineFollowerCar";
const char* password = "pass.123";

// Sensor pins
#define SENSOR_IZQ 34
#define SENSOR_CEN 36
#define SENSOR_DER 35
#define DHT_PIN 4
#define DHT_TYPE DHT11

// Motor pins
#define ENA 25
#define IN1 26
#define IN2 27
#define ENB 32
#define IN3 33
#define IN4 14

// I2C pins
#define MPU_SDA 21
#define MPU_SCL 22
#define BMP_SDA 18
#define BMP_SCL 19

// PWM configuration
const int freq = 1000;
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int resolution = 8;

// PID parameters
float Kp = 10.0;
float Ki = 0.005;
float Kd = 5.0;
float integral = 0;
float lastError = 0;

// Motor speeds
int baseSpeed = 80;
int maxSpeed = 100;

// Sensor calibration
int minIzq = 4095, maxIzq = 0;
int minCen = 4095, maxCen = 0;
int minDer = 4095, maxDer = 0;

// Moving average filter
const int filterSize = 5;
int izqBuffer[filterSize] = {0};
int cenBuffer[filterSize] = {0};
int derBuffer[filterSize] = {0};
int bufferIndex = 0;

// Path log
String pathLog = "INICIO\n";

// Sensor objects
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
bool mpuInitialized = false;
bool bmpInitialized = false;

// Two I2C buses
TwoWire I2C_MPU = TwoWire(0);
TwoWire I2C_BMP = TwoWire(1);

// Gyroscope drift correction
float gyroZOffset = 0;
const int calibrationSamples = 100;

// Server
AsyncWebServer server(80);

// Timing
unsigned long lastLogTime = 0;
unsigned long lastSensorRead = 0;
const long logInterval = 100; // Registro cada 100 ms
const long sensorInterval = 5000; // Sensores cada 5s

void calibrarSensores() {
  Serial.println("Calibrando sensores infrarrojos...");
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    int valIzq = analogRead(SENSOR_IZQ);
    int valCen = analogRead(SENSOR_CEN);
    int valDer = analogRead(SENSOR_DER);
    minIzq = min(minIzq, valIzq);
    maxIzq = max(maxIzq, valIzq);
    minCen = min(minCen, valCen);
    maxCen = max(maxCen, valCen);
    minDer = min(minDer, valDer);
    maxDer = max(maxDer, valDer);
    delay(5);
  }
  if (maxIzq - minIzq < 100) maxIzq = minIzq + 100;
  if (maxCen - minCen < 100) maxCen = minCen + 100;
  if (maxDer - minDer < 100) maxDer = minDer + 100;
  Serial.print("minIzq: "); Serial.print(minIzq);
  Serial.print(", maxIzq: "); Serial.print(maxIzq);
  Serial.print(", minCen: "); Serial.print(minCen);
  Serial.print(", maxCen: "); Serial.print(maxCen);
  Serial.print(", minDer: "); Serial.print(minDer);
  Serial.print(", maxDer: "); Serial.println(maxDer);
  Serial.println("Calibración completa.");
}

void calibrarGiroscopio() {
  if (!mpuInitialized) return;
  Serial.println("Calibrando giroscopio...");
  float sumGyroZ = 0;
  for (int i = 0; i < calibrationSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumGyroZ += g.gyro.z;
    delay(10);
  }
  gyroZOffset = sumGyroZ / calibrationSamples;
  Serial.print("Offset giroscopio Z: "); Serial.println(gyroZOffset, 4);
}

void moverMotor(int velIzq, int velDer) {
  velIzq = constrain(velIzq, -maxSpeed, maxSpeed);
  velDer = constrain(velDer, -maxSpeed, maxSpeed);
  digitalWrite(IN1, velIzq >= 0 ? HIGH : LOW);
  digitalWrite(IN2, velIzq >= 0 ? LOW : HIGH);
  digitalWrite(IN3, velDer >= 0 ? HIGH : LOW);
  digitalWrite(IN4, velDer >= 0 ? LOW : HIGH);
  ledcWrite(pwmChannelA, abs(velIzq));
  ledcWrite(pwmChannelB, abs(velDer));
  Serial.print("Vel Izq: "); Serial.print(velIzq);
  Serial.print(", Vel Der: "); Serial.println(velDer);
}

void logMovement(String movement) {
  unsigned long startTime = micros();
  File file = LittleFS.open("/movements.txt", "a");
  if (file) {
    file.println(movement);
    file.close();
  }
  pathLog += movement + "\n";
  if (pathLog.length() > 5000) {
    pathLog = pathLog.substring(pathLog.length() - 5000);
  }
  Serial.print("Tiempo escritura LittleFS: "); Serial.print(micros() - startTime); Serial.println(" us");
}

String escapeJson(String input) {
  String output = "";
  for (unsigned int i = 0; i < input.length(); i++) {
    char c = input[i];
    switch (c) {
      case '"': output += "\\\""; break;
      case '\\': output += "\\\\"; break;
      case '\n': output += "\\n"; break;
      case '\r': output += "\\r"; break;
      case '\t': output += "\\t"; break;
      default: output += c; break;
    }
  }
  return output;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando...");

  if (!LittleFS.begin()) {
    Serial.println("Error al montar LittleFS");
    return;
  }
  Serial.println("LittleFS montado correctamente");

  File file = LittleFS.open("/movements.txt", "w");
  if (file) {
    file.println("INICIO");
    file.close();
  }

  dht.begin();
  Serial.println("DHT11 inicializado");

  // Inicializar I2C para MPU-6050 (GPIO 21, 22)
  I2C_MPU.begin(MPU_SDA, MPU_SCL, 100000);
  if (!mpu.begin(0x68, &I2C_MPU)) {
    Serial.println("No se pudo inicializar MPU-6050");
    mpuInitialized = false;
  } else {
    mpuInitialized = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU-6050 inicializado");
    calibrarGiroscopio();
  }

  // Inicializar I2C para BMP180 (GPIO 18, 19)
  I2C_BMP.begin(BMP_SDA, BMP_SCL, 100000);
  if (!bmp.begin(0, &I2C_BMP)) {
    Serial.println("No se pudo inicializar BMP180");
    bmpInitialized = false;
  } else {
    bmpInitialized = true;
    Serial.println("BMP180 inicializado");
  }

  pinMode(SENSOR_IZQ, INPUT);
  pinMode(SENSOR_CEN, INPUT);
  pinMode(SENSOR_DER, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(pwmChannelA, freq, resolution);
  ledcAttachPin(ENA, pwmChannelA);
  ledcSetup(pwmChannelB, freq, resolution);
  ledcAttachPin(ENB, pwmChannelB);

  WiFi.softAP(ssid, password);
  IPAddress local_IP(192, 168, 1, 120);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  Serial.println("Punto de acceso WiFi creado");
  Serial.print("IP: "); Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{\"path\":\"" + escapeJson(pathLog) + "\",";
    unsigned long startTime = micros();
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    json += "\"temperature\":" + String(temp) + ",";
    json += "\"humidity\":" + String(hum) + ",";
    Serial.print("Tiempo lectura DHT11: "); Serial.print(micros() - startTime); Serial.println(" us");
    if (bmpInitialized) {
      startTime = micros();
      float pressure = bmp.readPressure() / 100.0; // Convertir Pa a hPa
      json += "\"pressure\":" + String(pressure, 1) + ",";
      Serial.print("Tiempo lectura BMP180: "); Serial.print(micros() - startTime); Serial.println(" us");
    } else {
      json += "\"pressure\":0,";
    }
    if (mpuInitialized) {
      sensors_event_t a, g, temp_mpu;
      startTime = micros();
      mpu.getEvent(&a, &g, &temp_mpu);
      Serial.print("Tiempo lectura MPU-6050: "); Serial.print(micros() - startTime); Serial.println(" us");
      json += "\"accelX\":" + String(a.acceleration.x) + ",";
      json += "\"accelY\":" + String(a.acceleration.y) + ",";
      json += "\"accelZ\":" + String(a.acceleration.z) + ",";
      json += "\"gyroX\":" + String(g.gyro.x) + ",";
      json += "\"gyroY\":" + String(g.gyro.y) + ",";
      json += "\"gyroZ\":" + String(g.gyro.z);
    } else {
      json += "\"accelX\":0,\"accelY\":0,\"accelZ\":0,\"gyroX\":0,\"gyroY\":0,\"gyroZ\":0";
    }
    json += "}";
    request->send(200, "application/json", json);
  });

  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/movements.txt", "text/plain", true);
  });

  server.on("/clear", HTTP_GET, [](AsyncWebServerRequest *request) {
    pathLog = "INICIO\n";
    File file = LittleFS.open("/movements.txt", "w");
    if (file) {
      file.println("INICIO");
      file.close();
      request->send(200, "text/plain", "Datos reiniciados");
    } else {
      request->send(500, "text/plain", "Error al reiniciar movements.txt");
    }
  });

  server.begin();
  Serial.println("Servidor iniciado");

  calibrarSensores();
  Serial.println("Setup completo");
}

void loop() {
  unsigned long startLoop = micros();

  int valIzq = analogRead(SENSOR_IZQ);
  int valCen = analogRead(SENSOR_CEN);
  int valDer = analogRead(SENSOR_DER);

  izqBuffer[bufferIndex] = valIzq;
  cenBuffer[bufferIndex] = valCen;
  derBuffer[bufferIndex] = valDer;
  bufferIndex = (bufferIndex + 1) % filterSize;

  long sumIzq = 0, sumCen = 0, sumDer = 0;
  for (int i = 0; i < filterSize; i++) {
    sumIzq += izqBuffer[i];
    sumCen += cenBuffer[i];
    sumDer += derBuffer[i];
  }
  valIzq = sumIzq / filterSize;
  valCen = sumCen / filterSize;
  valDer = sumDer / filterSize;

  int normIzq = map(valIzq, minIzq, maxIzq, 1000, 0);
  int normCen = map(valCen, minCen, maxCen, 1000, 0);
  int normDer = map(valDer, minDer, maxDer, 1000, 0);

  float error = normDer - normIzq;
  integral += error;
  if (abs(integral) > 1000) integral = 0;
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int velIzq = baseSpeed - correction;
  int velDer = baseSpeed + correction;
  velIzq = constrain(velIzq, 0, maxSpeed);
  velDer = constrain(velDer, 0, maxSpeed);

  unsigned long currentTime = millis();
  float gyroZ = 0;
  if (mpuInitialized) {
    sensors_event_t a, g, temp_mpu;
    mpu.getEvent(&a, &g, &temp_mpu);
    gyroZ = (g.gyro.z - gyroZOffset) * 1.2; // Amplificar giros
  }

  float timeDelta = (currentTime - lastLogTime) / 1000.0; // En segundos
  float distance = baseSpeed * timeDelta * 0.03; // Reducido para menor escala
  float angleChange = gyroZ * timeDelta;

  if (currentTime - lastLogTime >= logInterval) {
    String movement = "MOVE:DIST:" + String(distance, 2) + ",ANGLE:" + String(angleChange, 2);
    logMovement(movement);
    Serial.println("--- Estado ---");
    Serial.print("Raw Izq: "); Serial.print(valIzq);
    Serial.print(" | Raw Cen: "); Serial.print(valCen);
    Serial.print(" | Raw Der: "); Serial.println(valDer);
    Serial.print("Norm Izq: "); Serial.print(normIzq);
    Serial.print(" | Norm Cen: "); Serial.print(normCen);
    Serial.print(" | Norm Der: "); Serial.println(normDer);
    Serial.print("Error: "); Serial.println(error);
    Serial.print("Correction: "); Serial.println(correction);
    Serial.print("Vel Izq: "); Serial.print(velIzq);
    Serial.print(" | Vel Der: "); Serial.println(velDer);
    Serial.print("Distancia: "); Serial.print(distance); Serial.print(" cm");
    Serial.print(" | Ángulo: "); Serial.print(angleChange); Serial.println(" rad");
    lastLogTime = currentTime;
  }

  if (currentTime - lastSensorRead >= sensorInterval) {
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    float pressure = bmpInitialized ? bmp.readPressure() / 100.0 : 0;
    String sensorData = "TEMP:" + String(temp) + ",HUM:" + String(hum) + ",PRES:" + String(pressure, 1);
    if (mpuInitialized) {
      sensors_event_t a, g, temp_mpu;
      mpu.getEvent(&a, &g, &temp_mpu);
      sensorData += ",ACCX:" + String(a.acceleration.x) + 
                    ",ACCY:" + String(a.acceleration.y) + 
                    ",ACCZ:" + String(a.acceleration.z) + 
                    ",GYRX:" + String(g.gyro.x) + 
                    ",GYRY:" + String(g.gyro.y) + 
                    ",GYRZ:" + String(g.gyro.z);
    } else {
      sensorData += ",ACCX:0,ACCY:0,ACCZ:0,GYRX:0,GYRY:0,GYRZ:0";
    }
    logMovement(sensorData);
    lastSensorRead = currentTime;
  }

  moverMotor(velIzq, velDer);

  Serial.print("Tiempo loop: "); Serial.print(micros() - startLoop); Serial.println(" us");
}