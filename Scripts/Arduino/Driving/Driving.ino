#include <Servo.h>

Servo M1;
Servo M2;
Servo M4;
Servo M5;
Servo leg;

int direccionActual = 0;
int speedActual = 0;

void setup() {
  Serial.begin(9600);

  M1.attach(5, 1000, 2000);  // Cambia por tus pines
  M2.attach(6, 1000, 2000);
  M4.attach(10, 1000, 2000);
  M5.attach(11, 1000, 2000);

  leg.attach(9, 1000, 2000);

  leg.write(160);
  delay(500);

  setNeutro();

  Serial.println("Envía comandos en formato: (direccion,speed) ejemplo: (3,20)");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int dir, spd;
    if (parseInput(input, dir, spd)) {
      if (dir == 0) {
        direccionActual = 0;
        speedActual = 0;
        setNeutro();
        Serial.println("Posición neutra");
      }
      else if (dir >= 1 && dir <= 15) {
        direccionActual = dir;
        speedActual = spd;
        Serial.print("Comando actualizado: dir=");
        Serial.print(direccionActual);
        Serial.print(", speed=");
        Serial.println(speedActual);
      }
      else {
        Serial.println("Direccion fuera de rango (0-15)");
      }

      // Confirmación de recepción
      Serial.print("Recibido: ");
      Serial.println(input);
    } else {
      Serial.println("Formato inválido. Usa (dir,speed)");
    }
  }

  if (direccionActual != 0) {
    moverServos(direccionActual, speedActual);
  }

  delay(20);
}

void setNeutro() {
  M1.write(90);
  M2.write(90);
  M4.write(90);
  M5.write(90);
}

bool parseInput(String s, int &direccion, int &speed) {
  if (s.length() < 5) return false;

  if (s.charAt(0) != '(' || s.charAt(s.length() - 1) != ')') return false;

  int comaIndex = s.indexOf(',');
  if (comaIndex == -1) return false;

  String dirStr = s.substring(1, comaIndex);
  String speedStr = s.substring(comaIndex + 1, s.length() - 1);

  dirStr.trim();
  speedStr.trim();

  direccion = dirStr.toInt();
  speed = speedStr.toInt();

  if (dirStr.length() == 0 || speedStr.length() == 0) return false;

  return true;
}

void moverServos(int dir, int spd) {
  if (spd < 0) spd = 0;
  if (spd > 45) spd = 45;

  switch (dir) {
    case 1:
      M1.write(90 + spd);
      M2.write(90 + spd);
      M4.write(90 + spd);
      M5.write(90 + spd);
      Serial.println("Forward");
      break;
    case 2:
      M1.write(90 - spd);
      M2.write(90 - spd);
      M4.write(90 - spd);
      M5.write(90 - spd);
      Serial.println("Backward");
      break;
    case 3:
      M1.write(90 + spd);
      M2.write(90 - spd);
      M4.write(90 - spd);
      M5.write(90 + spd);
      Serial.println("Right");
      break;
    case 4:
      M1.write(90 - spd);
      M2.write(90 + spd);
      M4.write(90 + spd);
      M5.write(90 - spd);
      Serial.println("Left");
      break;
    case 5:
      M1.write(90 + spd);
      M2.write(90);
      M4.write(90);
      M5.write(90 + spd);
      Serial.println("Diag Forward Right");
      break;
    case 6:
      M1.write(90);
      M2.write(90 + spd);
      M4.write(90 + spd);
      M5.write(90);
      Serial.println("Diag Forward Left");
      break;
    case 7:
      M1.write(90);
      M2.write(90 - spd);
      M4.write(90 - spd);
      M5.write(90);
      Serial.println("Diag Backward Right");
      break;
    case 8:
      M1.write(90 - spd);
      M2.write(90);
      M4.write(90);
      M5.write(90 - spd);
      Serial.println("Diag Backward Left");
      break;
    case 9:
      M1.write(90 - spd);
      M2.write(90 - spd);
      M4.write(90 + spd);
      M5.write(90 + spd);
      Serial.println("Turn Right");
      break;
    case 10:
      M1.write(90 + spd);
      M2.write(90 + spd);
      M4.write(90 - spd);
      M5.write(90 - spd);
      Serial.println("Turn Left");
      break;
    case 11:
      M1.write(90);
      M2.write(90 + spd);
      M4.write(90 - spd);
      M5.write(90);
      Serial.println("A");
      break;
    case 12:
      M1.write(90 - spd);
      M2.write(90 + spd);
      M4.write(90);
      M5.write(90);
      Serial.println("B");
      break;
    case 13:
      M1.write(90 + spd);
      M2.write(90);
      M4.write(90 + spd);
      M5.write(90);
      Serial.println("C");
      break;
    case 14:
      M1.write(90 - spd / 2);
      M2.write(90 + spd / 2);
      M4.write(90 - spd / 2);
      M5.write(90 + spd / 2);
      Serial.println("D");
      break;
    case 15:
      M1.write(90);
      M2.write(90);
      M4.write(90);
      M5.write(90);
      Serial.println("E");
      break;
  }
}
