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
float kp = 1.5;
float kd = 0.024;
float ki = 0.005;

// Vector de posiciones angulares objetivos de los motores DC
int targets[numMotores] = {0, 0,0,0,0,0};
int target_servo =45;
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
    if (option == '1') {
      moveGripperPencil('o');
      delay(2000);
    }

    if (option == 'f') {
    for (int i = 0; i < 4; i++) {
      fsrReadings[i] = analogRead(fsr_pins[i]);  // Leer el valor analógico de cada sensor
      Serial.println(fsrReadings[i]);
      }
    }

    if (option == '2') {
      moveGripperPencil('c');
      delay(2000);
      }
    if (option == '3') {
      moveGripperCan('o');
      delay(2000);
    }
    
    if (option == '4') {
      moveGripperCan('c');
      delay(2000);
    }
  }
  
  servoMotor.write(map(target_servo, 0, 90, 10, 115));

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

    int dir = 1;
    if (u < 0){
      dir = -1;
    }
    setMotor(i, dir, pwr);
    eprev[i] = e;
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" - Posición: ");
    Serial.print(posi[i]);
    Serial.print(" - Target: ");
    Serial.print(targets[i]);
    Serial.print(" - PWM: ");
    Serial.println(analogRead(pwr));
  }
}

// Mostrar menú principal
void showMainMenu() {
  Serial.println("Seleccione el tipo de movimiento:");
  Serial.println("1. Pencil");
  Serial.println("2. Can");
  Serial.println("3. Flat");
  Serial.println("4. Cylindrical");
  Serial.println("5. Pincer");
}

// Funciones de movimiento para cada tipo
void moveGripperPencil(char state) {
  // Definir posiciones y acciones para Pencil
  if (state == 'o') {
    int valores[] = {0,0,-10,10,-10,10};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 90;
  } else {
    int valores[] = {0,0,-25,20,-25,20};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 90;
  }
}

void moveGripperCan(char state) {
  // Definir posiciones y acciones para Can
  if (state == 'o') {
    int valores[] = {0,0,0,0,0,0};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 45;
  } else {
    int valores[] = {-10,10,-10,10,-10,10};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 45;
  }
}

void moveGripperFlat(char state) {
  // Definir posiciones y acciones para Flat
  if (state == 'o') {
    int valores[] = {20,-10,20,-10,20,-10};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 0;
    }else {
    int valores[] = {0,0,0,0,0,0};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 0;
  }    
}

void moveGripperCylindrical(char state) {
  // Definir posiciones y acciones para Cylindrical
  if (state == 'o'){
    int valores[] = {20,-10,20,-10,20,-10};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }    
    target_servo = 45;
  } else {
    int valores[] = {0,0,0,0,0,0};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 45;
  }
}

void moveGripperPincer(char state) {
  // Definir posiciones y acciones para Pincer
  if (state == 'o'){
    int valores[] = {0,0,20,-10,20,-10};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 90;
  } else {
    int valores[] = {0,0,-25,20,-25,20};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 90;
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

