#include <Servo.h>
Servo servoMotor;

// Definición de pines digitales a usar para los motores DC
const int numMotores = 2;
int ENCA[numMotores] = {18, 19}; //c1_m1 and c1_m2
int ENCB[numMotores] = {44, 45}; //c2_m1 and c2_m2
int DIR[numMotores] = {40, 42};
int PWM[numMotores] = {41, 43};
volatile int posi[numMotores] = {0,0}; // Posición del motor según el encoder

// Variables a usar en el control PID
long prevT[numMotores] = {0};
float eprev[numMotores] = {0};
float eintegral[numMotores] = {0};

// Rango de ángulos y valores de encoder
int rangoGrados = 90; // Rango en grados (-90 a 90)
int rangoEncoder = 2300; // Rango en valores del encoder (-2300 a 2300)

// Constantes PID (ajústalas según sea necesario)
float kp = 1.5;
float kd = 0.024;
float ki = 0.005;

// Vector de posiciones angulares objetivos de los motores DC
int targets[numMotores] = {0, 0};

void setup() {
  Serial.begin(9600);

  // Configuración de los pines de los motores como salida
  for (int i = 0; i < numMotores; i++) {
    pinMode(DIR[i], OUTPUT);
    pinMode(PWM[i], OUTPUT);
  }
  for (int i = 0; i < numMotores; i++) {
    pinMode(ENCA[i], INPUT_PULLUP);
    pinMode(ENCB[i], INPUT);
  }

  // Configuración de las interrupciones para el conteo de pulsos
  attachInterrupt(digitalPinToInterrupt(ENCA[0]), doEncodeA0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[1]), doEncodeA1, RISING);
}

void loop() {
  // Chequear si hay una entrada desde el monitor serial
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int angulo = input.toInt(); // Convierte la entrada en un valor entero de grados
    if (angulo >= -rangoGrados && angulo <= rangoGrados) {
      // Convertir el ángulo a un valor del encoder
      targets[0] = map(angulo, -rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
      targets[1] = map(angulo, -rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
      Serial.print("Objetivo para el motor 1: ");
      Serial.println(targets[0]);
      Serial.print("Objetivo para el motor 2: ");
      Serial.println(targets[1]);
    }
  }

  // Aplicar control PID a ambos motores
  for (int i = 0; i < numMotores; i++) {
    long currT = micros();
    float deltaT = ((float)(currT - prevT[i])) / (1.0e6);

    prevT[i] = currT;
    int pos = 0;
    noInterrupts(); // Desactivar las interrupciones temporalmente durante la lectura
    pos = posi[i];
    interrupts(); // Volver a activar las interrupciones

    int e = targets[i] - pos;
    float dedt = (e - eprev[i]) / deltaT;
    eintegral[i] += e * deltaT;
    float u = kp * e + kd * dedt + ki * eintegral[i];

    // Limitar el valor de pwm a 255
    float pwr = fabs(u);
    if (pwr > 255) {
      pwr = 255;
    }

    // Dirección del motor
    int dir = 1;
    if (u < 0) {
      dir = -1;
    }
    setMotor(i, dir, pwr);
    eprev[i] = e;

    // Monitoreo continuo de la posición actual del encoder
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" Posición: ");
    Serial.println(posi[i]);
  }

  delay(100); // Espera 100 ms antes de la siguiente actualización
}

void setMotor(int motorIndex, int dir, int pwmVal) {
  analogWrite(PWM[motorIndex], pwmVal);
  if (dir == 1) {
    digitalWrite(DIR[motorIndex], HIGH);
  } else if (dir == -1) {
    digitalWrite(DIR[motorIndex], LOW);
  } else {
    digitalWrite(DIR[motorIndex], LOW);
  }
}

void doEncodeA0() {
  if (digitalRead(ENCB[0]) == 1) {
    posi[0]--;
  } else {
    posi[0]++;
  }
}

void doEncodeA1() {
  if (digitalRead(ENCB[1]) == 1) {
    posi[1]++;
  } else {
    posi[1]--;
  }
}
