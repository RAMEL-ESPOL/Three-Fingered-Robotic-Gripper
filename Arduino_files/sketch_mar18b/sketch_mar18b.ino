#include <Servo.h>
Servo servoMotor;
// Definición de pines Analógicos a usar para los sensores de fuerza
const int numSensores = 4;
const int fsr_pins[numSensores] = {A0, A1, A2, A3};
const int fsr_limits[numSensores] = {5, 5, 5, 5};
// Definición de pines digitales a usar para los motores DC
const int numMotores = 6;
int ENCA[numMotores] = {2, 3, 18, 19, 20, 21};
int ENCB[numMotores] = {38, 39, 40, 41, 42, 43};
int IN1[numMotores] = {22, 24, 26, 28, 30, 32};
int IN2[numMotores] = {23, 25, 27, 29, 31, 33};
int PWM[numMotores] = {4, 5, 6, 7, 8, 9};
volatile int counter[numMotores] = {0};
int encoderPin2Value[numMotores] = {0};
bool IsCW = true;
// Variables a usar en el control PID
volatile int posi[numMotores] = {0}; // Especificar posi como volatile
long prevT[numMotores] = {0};
float eprev[numMotores] = {0};
float eintegral[numMotores] = {0};
// Vector de posiciones angulares objetivos de los motores DC
int targets[numMotores] = {0, 0, 0, 0, 0, 0};
void setup()
{
    Serial.begin(9600);

    servoMotor.attach(10); // Servomotor al pin 10 PWM

    // Configuración de los pines de los motores como salida
    for (int i = 0; i < numMotores; i++)
    {
        pinMode(IN1[i], OUTPUT);
        pinMode(IN2[i], OUTPUT);
        pinMode(PWM[i], OUTPUT);
    }
    for (int i = 0; i < numMotores; i++)
    {
        pinMode(ENCA[i], INPUT_PULLUP);
        pinMode(ENCB[i], INPUT);
    }
    // Configuración de las interrupciones para el conteo de pulsos
    attachInterrupt(digitalPinToInterrupt(ENCA[0]), doEncodeA0, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA[1]), doEncodeA1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA[3]), doEncodeA2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA[4]), doEncodeA3, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA[5]), doEncodeA4, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA[6]), doEncodeA5, RISING);
}
void loop()
{
    char option;
    // Solo se activa cuando se envía alguna opción ("s" para servomotor y "m" para los
micromotores DC)
 if (Serial.available())
{
    option = Serial.read();
    // Servomotor
    if (option == 's')
    {
        Serial.print("Ingrese un angulo del servo: ");
        // Espera hasta que el usuario envié su ángulo
        while (!Serial.available())
            ;
        int angulo = Serial.parseInt(); // Lee el número enviado por el monitor serial
        if (angulo >= 0 && angulo <= 180)
        {
            servoMotor.write(angulo); // Mueve el servomotor al ángulo especificado
            // delay(2000);
            Serial.print("Servo posicionado en el angulo: ");
            Serial.println(angulo);
        }
        else
        {
            Serial.println("Angulo no valido (debe estar entre 0 y 180).");
        }
    }
    // Micro motores DC
    else if (option == 'm')
    {
        int number;
        Serial.println("Digita los ángulos de los seis motores de la siguiente forma: 0,0,0,0,0,0");
        while (!Serial.available())
            ; // Espera hasta que el usuario envié los ángulos
        String post_data = Serial.readString();
        int prepos = 0;
        int pos = 0;
        for (int i = 0; i < numMotores; i++)
        { // Asigna los angulos a los micro motores en orden
            pos = post_data.indexOf(',', prepos);
            String preconteo = post_data.substring(prepos, pos);
            targets[i] += preconteo.toInt();
            prepos = pos + 1;
        }
    }
    else
    {
        Serial.println("Invalid option. Please enter 'a' or 'b'.");
    }
    while (Serial.available())
    {
        Serial.read(); // Limpia el buffer de la comunicación serial
    }
}
// Define los valores obtenidos por los sensores de fuerza
int valoresSensores[numSensores];
for (int i = 0; i < numSensores; i++)
{
    valoresSensores[i] = analogRead(fsr_pins[i]);
    if (i == 3)
    {
        valoresSensores[i] -= 180;
    }
}
// Inicio del control PID
// Definición de constantes PID
float kp = 1.5;
float kd = 0.024;
float ki = 0.005;
// Aplicación de ajuste de ángulo por PID en cada motor
for (int i = 0; i < numMotores; i++)
{
    // time difference
    long currT = micros();
    float deltaT = ((float)(currT - prevT[i])) / (1.0e6);

    prevT[i] = currT;
    int pos = 0;
    noInterrupts(); // Desactivar las interrupciones temporalmente durante la lectura
    pos = posi[i];
    interrupts(); // Volver a activar las interrupciones
    int e = pos - targets[i];
    float dedt = (e - eprev[i]) / deltaT;
    eintegral[i] += e * deltaT;
    float u = kp * e + kd * dedt + ki * eintegral[i];

    // Potencia del motor
    float pwr = fabs(u);
    if (pwr > 255)
    {
        pwr = 255;
    }
    // Dirección del motor
    int dir = 1;
    if (u < 0)
    {
        dir = -1;
    }
    setMotor(i, dir, pwr);
    eprev[i] = e;
}
}
void setMotor(int motorIndex, int dir, int pwmVal)
{
    analogWrite(PWM[motorIndex], pwmVal);
    if (dir == 1)
    {
        digitalWrite(IN1[motorIndex], HIGH);
        digitalWrite(IN2[motorIndex], LOW);
    }
    else if (dir == -1)
    {
        digitalWrite(IN1[motorIndex], LOW);
        digitalWrite(IN2[motorIndex], HIGH);
    }
    else
    {
        digitalWrite(IN1[motorIndex], LOW);
        digitalWrite(IN2[motorIndex], LOW);
    }
}
void doEncodeA0() // Es una función por motor ya que deben especificarse de forma individual
{
    encoderPin2Value[0] = digitalRead(ENCB[0]);
    if (encoderPin2Value[0] == 1) // Clock Wise direction
    {
        posi[0]++;
    }
    else // else, it is zero... -> Counter Clock Wise direction
    {
        posi[0]--;
    }
}
void doEncodeA1()
{
    encoderPin2Value[1] = digitalRead(ENCB[1]);
    if (encoderPin2Value[1] == 1)
    {
        posi[1]++;
    }
    else
    {
        posi[1]--;
    }
}
void doEncodeA2()
{
    encoderPin2Value[3] = digitalRead(ENCB[3]);
    if (encoderPin2Value[3] == 1)
    {
        posi[3]++;
    }
    else
    {
        posi[3]--;
    }
}
void doEncodeA3()
{
    encoderPin2Value[4] = digitalRead(ENCB[4]);
    if (encoderPin2Value[4] == 1)
    {
        posi[4]++;
    }
    else
    {
        posi[4]--;
    }
}
void doEncodeA4()
{
    encoderPin2Value[5] = digitalRead(ENCB[5]);
    if (encoderPin2Value[5] == 1)
    {
        posi[5]++;
    }
    else
    {
        posi[5]--;
    }
}
void doEncodeA5()
{
    encoderPin2Value[6] = digitalRead(ENCB[6]);
    if (encoderPin2Value[6] == 1)
    {
        posi[6]++;
    }
    else
    {
        posi[6]--;
    }
}
