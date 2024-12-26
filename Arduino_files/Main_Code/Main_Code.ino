#include <Servo.h>
Servo servoMotor;

// Definición de sensores de presión
const int numSensores = 4;
const int fsr_pins[numSensores] = {A0, A1, A2, A3};
const int fsr_limits[numSensores] = {5, 5, 5, 5};
int fsrReadings[4];

// Definición de pines digitales a usar para los motores DC
const int numMotores = 6;
int ENCA[numMotores] = {20, 21, 18, 19, 2, 3};
int ENCB[numMotores] = {38, 39, 49, 51, 29, 31};
int DIR[numMotores] = {22, 23, 48, 50, 28, 30}; 
int PWM[numMotores] = {4, 5, 6, 7, 8, 9};
volatile int posi[numMotores] = {0, 0, 0, 0, 0, 0};
int target_servo = 45;
// Variables de control PID
long prevT[numMotores] = {0, 0, 0, 0, 0, 0};
float eprev[numMotores] = {0, 0, 0, 0, 0, 0};
float eintegral[numMotores] = {0, 0, 0, 0, 0, 0};

// Rango de ángulos y valores de encoder
int rangoGrados = 90;
int rangoEncoder = 2300;

// Constantes PID
float kp = 1.5;
float kd = 0.024;
float ki = 0.005;


// Vector de posiciones objetivo
int targets[numMotores] = {0, 0, 0, 0, 0, 0};

// Definir estado del gripper
enum GripperState {OPEN, CLOSE};
GripperState gripperState;

void setup() {
  Serial.begin(9600);

  // Configuración del servo
  servoMotor.attach(10);

  // Configuración de los motores DC
  for (int i = 0; i < numMotores; i++) {
    pinMode(DIR[i], OUTPUT);
    pinMode(PWM[i], OUTPUT);
    pinMode(ENCA[i], INPUT_PULLUP);
    pinMode(ENCB[i], INPUT);
  }

  // Configuración de las interrupciones para los encoders
  attachInterrupt(digitalPinToInterrupt(ENCA[0]), doEncodeA0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[1]), doEncodeA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[2]), doEncodeA2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[3]), doEncodeA3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[4]), doEncodeA4, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[5]), doEncodeA5, RISING);

  // Mostrar el menú principal
  showMainMenu();
}

void loop() {
  if (Serial.available()) {
    char option = Serial.read();
    handleMenuOption(option);
  }

  servoMotor.write(map(target_servo, 0, 90, 30, 130));

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
    if (pwr > 255) {
      pwr = 255;
    }
    int dir = 1;
    if (u < 0) {
      dir = -1;
    }
    setMotor(i, dir, pwr);
    eprev[i] = e;
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
  Serial.println("6. Set cero");
}

// Manejar la opción seleccionada en el menú
void handleMenuOption(char option) {
  switch (option) {
    case '1': handleGripperMovement("Pencil"); break;
    case '2': handleGripperMovement("Can"); break;
    case '3': handleGripperMovement("flat"); break;
    case '4': handleGripperMovement("Cylindrical"); break;
    case '5': handleGripperMovement("Pincer"); break;
    case '6': ceroMovement(); break;
    default: Serial.println("Opción no válida. Intente de nuevo."); showMainMenu(); break;
  }
}

// Manejar la configuración de movimiento seleccionada
void handleGripperMovement(String movementType) {
  Serial.print("Seleccionaste: ");
  Serial.println(movementType);
  Serial.println("Ingrese 'o' para abrir (open) o 'c' para cerrar (close):");

  // Esperar hasta que se reciba una entrada válida
  char stateOption = '\0';  // Inicializamos el estado

  // Ciclo de espera no bloqueante
  while (true) {
    if (Serial.available() > 0) {
      stateOption = Serial.read();  // Lee el carácter ingresado
      if (stateOption == 'o' || stateOption == 'c') {
        break;  // Si es una opción válida ('o' o 'c'), salimos del ciclo
      } else {
        Serial.println("Opción no válida. Ingrese 'o' para abrir o 'c' para cerrar.");
      }
    }
    // Aquí podrías poner un pequeño delay si quieres evitar que se ejecute demasiado rápido.
  }

  // Después de recibir una opción válida
  if (stateOption == 'o') {
    gripperState = OPEN;
    Serial.println("Abriendo el gripper...");
  } else if (stateOption == 'c') {
    gripperState = CLOSE;
    Serial.println("Cerrando el gripper...");
  }

  // Aplicar el movimiento al gripper
  applyGripperMovement(movementType, gripperState);
}


// Aplicar el movimiento del gripper (open/close)
void applyGripperMovement(String movementType, GripperState state) {
  if (state == OPEN) {
    Serial.print("Abriendo en configuración: ");
  } else {
    Serial.print("Cerrando en configuración: ");
  }
  Serial.println(movementType);

  // Aquí puedes definir diferentes comportamientos para cada tipo de movimiento
  if (movementType == "Pencil") {
    moveGripperPencil(state);
  } else if (movementType == "Can") {
    moveGripperCan(state);
  } else if (movementType == "Cereal") {
    moveGripperCereal(state);
  } else if (movementType == "Cylindrical") {
    moveGripperCylindrical(state);
  } else if (movementType == "Pincer") {
    moveGripperPincer(state);
  }
}

// Funciones de movimiento para cada tipo
void moveGripperPencil(GripperState state) {
  // Definir posiciones y acciones para Pencil
  if (state == OPEN) {
    int valores[] = {20,-10,-10,10,-10,10};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 90;
  } else {
    int valores[] = {20,-10,-40,10,-40,10};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 90;
  }
}
void moveGripperCereal(GripperState state) {
  // Definir posiciones y acciones para Pencil
  if (state == OPEN) {
    int valores[] = {10,-10,10,-10,10,-10};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 0;
  } else {
    int valores[] = {-10,-10,-10,-10,-10,-10};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 0;
  }
}

void moveGripperCan(GripperState state) {
  // Definir posiciones y acciones para Can
  if (state == OPEN) {
    int valores[] = {0,0,0,0,0,0};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 45;
  } else {
    int valores[] = {-30,10,-30,10,-30,10};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 45;
  }
}

void moveGripperFlat(GripperState state) {
  // Definir posiciones y acciones para Flat
  if (state == OPEN) {
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

void moveGripperCylindrical(GripperState state) {
  // Definir posiciones y acciones para Cylindrical
  if (state == OPEN) {
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

void moveGripperPincer(GripperState state) {
  // Definir posiciones y acciones para Pincer
  if (state == OPEN) {
    int valores[] = {0,0,30,-15,30,-15};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 90;
  } else {
    int valores[] = {0,0,20,-10,20,-10};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 90;
  }
}

void ceroMovement(){
  int valores[] = {0,0,0,0,0,0};
    for (int i = 0; i < numMotores; i++) {
      targets[i] = map(valores[i],-rangoGrados, rangoGrados, -rangoEncoder, rangoEncoder);
    }
    target_servo = 45;
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
