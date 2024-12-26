#include <Servo.h>
Servo servoMotor;

// Definición de sensores de presión
const int numSensores = 4;
const int fsr_pins[numSensores] = {A0, A1, A2, A3};
const int fsr_limits[numSensores] = {5, 5, 5, 5};
int fsrReadings[4];                 // Array para almacenar las lecturas de los 4 sensores

// Definición de pines digitales a usar para los motores DC
const int numMotores = 6;
int ENCA[numMotores] = {20, 21, 18, 19, 2, 3};
int ENCB[numMotores] = {38, 39, 49, 51, 29, 31};
int DIR[numMotores] = {22, 23, 48, 50, 28, 30}; 
int PWM[numMotores] = {4, 5, 6, 7, 8, 9};
volatile int posi[numMotores] = {0, 0, 0, 0,0,0}; // Posición del motor según el encoder

// Variables a usar en el control PID
long prevT[numMotores] = {0,0,0,0,0,0};
float eprev[numMotores] = {0,0,0,0,0,0};
float eintegral[numMotores] = {0,0,0,0,0,0};

// Rango de ángulos y valores de encoder
int rangoGrados = 90; // Rango en grados (-90 a 90)
int rangoEncoder = 2300; // Rango en valores del encoder (-2300 a 2300)

// Constantes PID (ajústalas según sea necesario)
float kp = 1;
float kd = 0.024;
float ki = 0.005;
float target_servo = 90;

// Vector de posiciones angulares objetivos de los motores DC
int targets[numMotores] = {0, 0,0,0,0,0};

void setup() {
  Serial.begin(9600);

  servoMotor.attach(10); //Servomotor al pin 10 PWM
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
  attachInterrupt(digitalPinToInterrupt(ENCA[2]), doEncodeA2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[3]), doEncodeA3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[4]), doEncodeA4, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[5]), doEncodeA5, RISING);
}

void loop() {
  if (Serial.available()) {
    char option = Serial.read();
    if (option == 's') {
    Serial.print("Ingrese un angulo del servo: ");
    //Espera hasta que el usuario envié su ángulo
    while (!Serial.available());
    int angulo_s = Serial.parseInt(); // Lee el número enviado por el monitor serial
    if (angulo_s >= 0 && angulo_s <= 90){
      int target_servo = map(angulo_s, 0, 90, 35, 130);
      servoMotor.write(target_servo); // Mueve el servomotor al ángulo especificado
    //delay(2000);
    Serial.print("Servo posicionado en el angulo: ");
    Serial.println(angulo_s);} 
    else {
    Serial.println("Angulo no valido (debe estar entre 0 y 180).");}}
    if (option == 'f') {
    for (int i = 0; i < 4; i++) {
      fsrReadings[i] = analogRead(fsr_pins[i]);  // Leer el valor analógico de cada sensor
      Serial.println(fsrReadings[i]);
      }
    }
    if (option == 'm') {
      Serial.println("Digita los ángulos de los motores de la siguiente forma: 10,10,10,10,10,10");
      // Esperar a que el usuario ingrese los datos con timeout
      long startTime = millis();
      while (!Serial.available()) {
        if (millis() - startTime > 5000) { // Timeout de 5 segundos
          Serial.println("Timeout: No se recibieron datos.");
          return; // Salir de la función loop para evitar bloqueo
        }
      }

      // Leer los valores ingresados
      String post_data = Serial.readString();
      
      // Separar y convertir los valores
      int commaIndex1 = post_data.indexOf(',');
      int commaIndex2 = post_data.indexOf(',', commaIndex1 + 1);
      int commaIndex3 = post_data.indexOf(',', commaIndex2 + 1);
      int commaIndex4 = post_data.indexOf(',', commaIndex3 + 1);
      int commaIndex5 = post_data.indexOf(',', commaIndex4 + 1);

      if (commaIndex1 > 0 && commaIndex2 > commaIndex1 && commaIndex3 > commaIndex2) {
        int anguloMotor1 = post_data.substring(0, commaIndex1).toInt();
        int anguloMotor2 = post_data.substring(commaIndex1 + 1, commaIndex2).toInt();
        int anguloMotor3 = post_data.substring(commaIndex2 + 1, commaIndex3).toInt();
        int anguloMotor4 = post_data.substring(commaIndex3 + 1, commaIndex4).toInt();
        int anguloMotor5 = post_data.substring(commaIndex4 + 1, commaIndex5).toInt();
        int anguloMotor6 = post_data.substring(commaIndex5 + 1).toInt();

        // Convertir los ángulos a valores del encoder
        targets[0] = map(anguloMotor1, -rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
        targets[1] = map(anguloMotor2, -rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
        targets[2] = map(anguloMotor3, -rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
        targets[3] = map(anguloMotor4, -rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
        targets[4] = map(anguloMotor5, -rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
        targets[5] = map(anguloMotor6, -rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
        
        
      }
    }
  }
  // Aplicar control PID a los motores
  for (int i = 0; i < numMotores; i++) {
    long currT = micros();
    float deltaT = ((float)(currT - prevT[i])) / (1.0e6);

    prevT[i] = currT;
    int pos = 0;
    noInterrupts(); 
    pos = posi[i];
    interrupts(); 

    int e = targets[i] - pos;
    float dedt = (e - eprev[i]) / deltaT;
    eintegral[i] += e * deltaT;
    float u = kp * e + kd * dedt + ki * eintegral[i];

    float pwr = fabs(u);
    if (pwr > 255){
      pwr = 255;
    }

    if (eintegral[i] > 1000) eintegral[i] = 1000;
    if (eintegral[i] < -1000) eintegral[i] = -1000;

    int dir = 1;
    if (u < 0){
      dir = -1;
    }
    setMotor(i, dir, pwr);
    eprev[i] = e;

    Serial.print("Motor: ");
    Serial.print(i);
    Serial.print(" Error: ");
    Serial.print(e);
    Serial.print(" Output: ");
    Serial.println(pwr);

  }
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
//Para el motor 1 se cambia el la direccion de manera que funcione de igual manera que el 2
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
void doEncodeA2() {
  if (digitalRead(ENCB[2]) == 1) {
    posi[2]--;
  } else {
    posi[2]++;
  }
}

void doEncodeA3() {
  if (digitalRead(ENCB[3]) == 1) {
    posi[3]++;
  } else {
    posi[3]--;
  }
}
void doEncodeA4() {
  if (digitalRead(ENCB[4]) == 1) {
    posi[4]--;
  } else {
    posi[4]++;
  }
}

void doEncodeA5() {
  if (digitalRead(ENCB[5]) == 1) {
    posi[5]++;
  } else {
    posi[5]--;
  }
}
