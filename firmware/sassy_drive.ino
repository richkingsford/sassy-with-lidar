// Sassy motor controller for Arduino Uno.
// Source: /home/richkingsford/Sketch - only the LR motors on the chassis - serial.txt
// Protocol: C,throttle,steering | D,left,right | S | H, newline terminated.

const uint8_t LEFT_FORWARD_PIN = 5, LEFT_REVERSE_PIN = 6;
const uint8_t RIGHT_FORWARD_PIN = 9, RIGHT_REVERSE_PIN = 10;
const uint32_t SERIAL_BAUD = 115200;
const uint16_t WATCHDOG_MS = 250;
const uint8_t RX_BUFFER_SIZE = 32;
const bool INVERT_LEFT_TREAD = false, INVERT_RIGHT_TREAD = false;

char rxBuffer[RX_BUFFER_SIZE]; uint8_t rxIndex = 0;
uint32_t lastCommandTime = 0; bool motorsActive = false;

void setup() {
  pinMode(LEFT_FORWARD_PIN, OUTPUT); pinMode(LEFT_REVERSE_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT); pinMode(RIGHT_REVERSE_PIN, OUTPUT);
  stopMotors(); Serial.begin(SERIAL_BAUD); lastCommandTime = millis();
}
void loop() { readSerial(); if (motorsActive && millis() - lastCommandTime > WATCHDOG_MS) stopMotors(); }

void readSerial() {
  while (Serial.available()) { char c = Serial.read(); if (c == '\r') continue;
    if (c == '\n') { if (rxIndex) { rxBuffer[rxIndex] = '\0'; processCommand(rxBuffer); rxIndex = 0; } }
    else if (rxIndex < RX_BUFFER_SIZE - 1) rxBuffer[rxIndex++] = c; else rxIndex = 0;
  }
}
void processCommand(char *cmd) {
  if (cmd[0] == 'S' || cmd[0] == 's') { stopMotors(); lastCommandTime = millis(); return; }
  if (cmd[0] == 'H' || cmd[0] == 'h') { lastCommandTime = millis(); return; }
  if (cmd[0] != 'C' && cmd[0] != 'c' && cmd[0] != 'D' && cmd[0] != 'd') return;
  char *p = cmd + 1; if (*p++ != ',') return; char *endPtr;
  long a = strtol(p, &endPtr, 10); if (endPtr == p || *endPtr++ != ',') return;
  long b = strtol(endPtr, &p, 10); if (p == endPtr) return;
  a = constrain(a, -255, 255); b = constrain(b, -255, 255);
  if (cmd[0] == 'C' || cmd[0] == 'c') driveMixed(a, b); else { setLeftTread(a); setRightTread(b); motorsActive = a || b; }
  lastCommandTime = millis();
}
void driveMixed(int16_t throttle, int16_t steering) {
  int32_t l = throttle + steering, r = throttle - steering, m = max(abs(l), abs(r));
  if (m > 255) { l = l * 255L / m; r = r * 255L / m; }
  setLeftTread(l); setRightTread(r); motorsActive = l || r;
}
void setLeftTread(int16_t s) { if (INVERT_LEFT_TREAD) s = -s; setMotor(LEFT_FORWARD_PIN, LEFT_REVERSE_PIN, s); }
void setRightTread(int16_t s) { if (INVERT_RIGHT_TREAD) s = -s; setMotor(RIGHT_FORWARD_PIN, RIGHT_REVERSE_PIN, s); }
void setMotor(uint8_t f, uint8_t r, int16_t s) { s = constrain(s, -255, 255); analogWrite(f, s > 0 ? s : 0); analogWrite(r, s < 0 ? -s : 0); }
void stopMotors() { analogWrite(LEFT_FORWARD_PIN, 0); analogWrite(LEFT_REVERSE_PIN, 0); analogWrite(RIGHT_FORWARD_PIN, 0); analogWrite(RIGHT_REVERSE_PIN, 0); motorsActive = false; }
