// Pines sensores
#define SENSOR_IZQ 34     // Pin analógico del sensor izquierdo
#define SENSOR_DER 35     // Pin analógico del sensor derecho

// Pines motores
#define ENA 25            // Pin de PWM para motor izquierdo
#define IN1 26            // Dirección 1 del motor izquierdo
#define IN2 27            // Dirección 2 del motor izquierdo

#define ENB 32            // Pin de PWM para motor derecho
#define IN3 33            // Dirección 1 del motor derecho
#define IN4 14            // Dirección 2 del motor derecho

// PWM
#define CH_A 0            // Canal PWM para el motor izquierdo
#define CH_B 1            // Canal PWM para el motor derecho

// Variables sensores para calibración
int minIzq = 4095, maxIzq = 0; // Valores extremos para normalizar sensor izquierdo
int minDer = 4095, maxDer = 0; // Valores extremos para normalizar sensor derecho

// Constantes y variables del controlador PID
float Kp = 6;                 // Ganancia proporcional
float Ki = 0;                  // Ganancia integral (no usada en este caso)
float Kd = 5;                  // Ganancia derivativa
float error = 0;               // Error actual
float integral = 0;           // Integral acumulada del error
float lastError = 0;          // Error del ciclo anterior

// Velocidades base y máxima de los motores
int baseSpeed = 54;           // Velocidad base del robot
int maxSpeed = 57;           // Velocidad máxima permitida

void setup() {
  Serial.begin(115200);       // Inicializa la comunicación serial

  // Configura los pines de sensores como entrada
  pinMode(SENSOR_IZQ, INPUT);
  pinMode(SENSOR_DER, INPUT);

  // Configura los pines de dirección de los motores como salida
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configura los canales PWM con frecuencia de 1kHz y resolución de 8 bits
  ledcSetup(CH_A, 1000, 8);         // Canal A
  ledcAttachPin(ENA, CH_A);        // Asocia ENA al canal A

  ledcSetup(CH_B, 1000, 8);         // Canal B
  ledcAttachPin(ENB, CH_B);         // Asocia ENB al canal B

  // Calibración de sensores durante 5 segundos
  Serial.println("🔧 Calibrando sensores (5 segundos)...");
  calibrarSensores();
  Serial.println("✅ Calibración completa.");
}

void loop() {
  // Lecturas analógicas de los sensores
  int valIzq = analogRead(SENSOR_IZQ);
  int valDer = analogRead(SENSOR_DER);

  // Normalización de los valores (0 a 1000)
  int normIzq = map(valIzq, minIzq, maxIzq, 1000, 0);
  int normDer = map(valDer, minDer, maxDer, 1000, 0);

  // Cálculo del error como diferencia entre sensores
  error = normDer - normIzq;

  // Cálculo del término integral y derivativo
  integral += error;
  float derivative = error - lastError;

  // Cálculo de la corrección PID
  float correction = Kp * error + Ki * integral + Kd * derivative;

  // Almacena el error actual para el siguiente ciclo
  lastError = error;

  // Calcula la velocidad corregida para cada motor
  int velIzq = baseSpeed - correction;
  int velDer = baseSpeed + correction;

  // Limita las velocidades a los valores permitidos
  velIzq = constrain(velIzq, 0, maxSpeed);
  velDer = constrain(velDer, 0, maxSpeed);

  // Aplica la velocidad a los motores
  moverMotor(velIzq, velDer);

  // Pequeña pausa para evitar sobrecarga
  delay(10);
}

// Función para calibrar sensores durante 5 segundos
void calibrarSensores() {
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    int valIzq = analogRead(SENSOR_IZQ);
    int valDer = analogRead(SENSOR_DER);

    // Actualiza los valores mínimos y máximos detectados
    if (valIzq < minIzq) minIzq = valIzq;
    if (valIzq > maxIzq) maxIzq = valIzq;

    if (valDer < minDer) minDer = valDer;
    if (valDer > maxDer) maxDer = valDer;

    // Hace girar el robot lentamente para captar el entorno
    moverMotor(60, 60);
    delay(5);
  }
  // Detiene el robot después de calibrar
  moverMotor(0, 0);
}

// Función para mover los motores con velocidades específicas
void moverMotor(int velIzq, int velDer) {
  // Configura dirección hacia adelante en ambos motores
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Aplica velocidad con PWM
  ledcWrite(CH_A, velIzq);
  ledcWrite(CH_B, velDer);
}