#include <Servo.h>
Servo servoMotor;
// Definición de pines digitales a usar para los motores DC
const int numMotores = 2;
int ENCA[numMotores] = {2, 3};
int ENCB[numMotores] = {38, 39};
int IN1[numMotores] = {22, 24};
int IN2[numMotores] = {23 , 25};
int PWM[numMotores] = {4, 5};
volatile int counter[numMotores] = {0};
int encoderPin2Value[numMotores] = {0};
bool IsCW = true;
// Variables a usar en el control PID
volatile int posi[numMotores] = {0}; // Especificar posi como volatile
long prevT[numMotores] = {0};
float eprev[numMotores] = {0};
float eintegral[numMotores] = {0};
// Vector de posiciones angulares objetivos de los motores DC
int targets[numMotores] = {0, 0};


void setup() {
  Serial.begin(9600);

  // Configuración de los pines de los motores como salida
  for (int i = 0; i < numMotores; i++) {
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
    pinMode(PWM[i], OUTPUT);
  }
  for (int i = 0; i < numMotores; i++) {
    pinMode(ENCA[i], INPUT_PULLUP);
    pinMode(ENCB[i], INPUT);
  }
}
void loop() {
  Serial.println("Digita los ángulos de los seis motores de la siguiente forma: 0,0");
  while(!Serial.available()) {
    String post_data = Serial.readString();
    int prepos = 0;
    int pos = 0;
    for (int i = 0; i < numMotores; i++) { //Asigna los angulos a los micro motores en orden
      pos = post_data.indexOf(',', prepos);
      String preconteo = post_data.substring(prepos, pos);
      targets[i] += preconteo.toInt();
      prepos = pos + 1;
    }
    Serial.println(targets[1]);
  }
}

void setMotor(int motorIndex, int dir, int pwmVal) {
  analogWrite(PWM[motorIndex], pwmVal);
  if (dir == 1) {
    digitalWrite(IN1[motorIndex], HIGH);
    digitalWrite(IN2[motorIndex], LOW);
  }
  else if (dir == -1) {
    digitalWrite(IN1[motorIndex], LOW);
    digitalWrite(IN2[motorIndex], HIGH);
  }
  else {
    digitalWrite(IN1[motorIndex], LOW);
    digitalWrite(IN2[motorIndex], LOW);
  }
}
void doEncodeA0() {
  encoderPin2Value[0] = digitalRead(ENCB[0]);
  if (encoderPin2Value[0] == 1) //Clock Wise direction
  {
    posi[0]++;
  }
  else //else, it is zero... -> Counter Clock Wise direction
  {
    posi[0]--;
  }
}
void doEncodeA1() {
  encoderPin2Value[1] = digitalRead(ENCB[1]);
  if (encoderPin2Value[1] == 1)
  {
    posi[1]++;
  }
  else {
    posi[1]--;
  }
}
