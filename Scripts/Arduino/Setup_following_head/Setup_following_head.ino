#include <Servo.h>

// Servos
Servo head;
Servo right_shoulder;
Servo left_shoulder;
Servo right_ankle;
Servo left_ankle;

// Rango del servo
const int SERVO_MIN = 55;
const int SERVO_MAX = 120;
const int POS_CENTER = 80;

float servoPos = POS_CENTER;    // Posición actual de la cabeza como float
char currentCommand = 'C';      // Último comando recibido
const float step = 0.2;         // Paso más fino
const int delayTime = 20;

void setup() {
  // Inicialización de la cabeza
  head.attach(3);
  head.write(80);   // Posición inicial al centro

  // Inicialización de hombros y tobillos
  //right_shoulder.attach(4);
  //left_shoulder.attach(5);
  right_ankle.attach(6);
  left_ankle.attach(7);

  // Posición inicial de cada servo
  //right_shoulder.write(100);  // hombro derecho
  //left_shoulder.write(130);   // hombro izquierdo
  right_ankle.write(80);      // tobillo derecho
  left_ankle.write(80);       // tobillo izquierdo

  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char zone = Serial.read();
    currentCommand = zone;
  }

  // Mover el servo de la cabeza progresivamente
  if (currentCommand == 'L' && servoPos < SERVO_MAX) {
    servoPos += step;
    if (servoPos > SERVO_MAX) servoPos = SERVO_MAX;
  } 
  else if (currentCommand == 'R' && servoPos > SERVO_MIN) {
    servoPos -= step;
    if (servoPos < SERVO_MIN) servoPos = SERVO_MIN;
  }
  // 'C' no hace nada

  head.write(int(servoPos));   // Convertir a entero para el servo
  delay(delayTime);
}
