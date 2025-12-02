#include <Arduino.h>

// Pinos dos motores
#define TDIR 11
#define FDIR 10
#define TESQ 9
#define FESQ 6

// Pinos dos sensores
#define LD A2 // lateral direito
#define CD A3 // central direito
#define CE A4 // central esquerdo
#define LE A5 // lateral esquerdo

// LDR (sensor de luz ambiente)
#define LDR A1

// Botão
#define BUT 7

// Buzzer
#define BUZZER 8

// LED externo
#define LED 12

// Display 7 segmentos (BCD)
#define BCD_A 3
#define BCD_B 5
#define BCD_C 4
#define BCD_D 2

// Configuração da pista(mudar no campeonato)
const int TOTAL_LINHAS = 3;

// Velocidades
const int BASE_SPEED = 180;
const int CURVE_SPEED = 90;
const int SHARP_TURN_SPEED = 80;
const int MIN_MOTOR_SPEED = 70;
const int TURN_SPEED = 30;
const int AJUSTE_ESQUERDO = 14;
const int SENSIBILIDADE = 850;
const int LDR_THRESHOLD = 420;
const unsigned long DEBOUNCE_MS = 50;

// Constantes PID
const float Kp = 40.0;
const float Ki = 0.1;
const float Kd = 25.0;

// Estado do robô
bool powered = false;
int lastReading = LOW;
int stableState = LOW;
int initialIdle = LOW;
unsigned long lastDebounce = 0;
int lastLeftSpeed = BASE_SPEED;
int lastRightSpeed = BASE_SPEED;
int turnFactor = 0;
int lastTurnFactor = 0;

// Estado PID
float integral = 0;
int lastError = 0;

// Recuperação
unsigned long lostLineTime = 0;
const unsigned long RECOVERY_DELAY = 50;
const unsigned long REVERSE_TIME = 150;
int recoveryDirection = 0;

// Contador de linhas horizontais
int contadorFitas = 0;
bool ultimoEstadoHorizontal = false;
unsigned long tempoUltimaDeteccao = 0;
const unsigned long DEBOUNCE_FITA = 500;

void setMotors(int left, int right) {
  int l = left;
  int r = right;
  if (l >= 0) l += AJUSTE_ESQUERDO;
  l = constrain(l, -255, 255);
  r = constrain(r, -255, 255);

  if (l >= 0) { analogWrite(FESQ, l); analogWrite(TESQ, 0); } else { analogWrite(FESQ, 0); analogWrite(TESQ, abs(l)); }
  if (r >= 0) { analogWrite(FDIR, r); analogWrite(TDIR, 0); } else { analogWrite(FDIR, 0); analogWrite(TDIR, abs(r)); }
}

void stopMotors() {
  analogWrite(FESQ, 0); analogWrite(TESQ, 0);
  analogWrite(FDIR, 0); analogWrite(TDIR, 0);
}

bool detectarLinhaHorizontal(bool le, bool ce, bool cd, bool ld) {
  return (le && ce && cd && ld);
}

void displayDigit(int digit) {
  switch (digit) {
    case 0: digitalWrite(BCD_A, LOW); digitalWrite(BCD_B, LOW); digitalWrite(BCD_C, LOW); digitalWrite(BCD_D, LOW); break;
    case 1: digitalWrite(BCD_A, HIGH); digitalWrite(BCD_B, LOW); digitalWrite(BCD_C, LOW); digitalWrite(BCD_D, LOW); break;
    case 2: digitalWrite(BCD_A, LOW); digitalWrite(BCD_B, HIGH); digitalWrite(BCD_C, LOW); digitalWrite(BCD_D, LOW); break;
    case 3: digitalWrite(BCD_A, HIGH); digitalWrite(BCD_B, HIGH); digitalWrite(BCD_C, LOW); digitalWrite(BCD_D, LOW); break;
    case 4: digitalWrite(BCD_A, LOW); digitalWrite(BCD_B, LOW); digitalWrite(BCD_C, HIGH); digitalWrite(BCD_D, LOW); break;
    case 5: digitalWrite(BCD_A, HIGH); digitalWrite(BCD_B, LOW); digitalWrite(BCD_C, HIGH); digitalWrite(BCD_D, LOW); break;
    case 6: digitalWrite(BCD_A, LOW); digitalWrite(BCD_B, HIGH); digitalWrite(BCD_C, HIGH); digitalWrite(BCD_D, LOW); break;
    case 7: digitalWrite(BCD_A, HIGH); digitalWrite(BCD_B, HIGH); digitalWrite(BCD_C, HIGH); digitalWrite(BCD_D, LOW); break;
    case 8: digitalWrite(BCD_A, LOW); digitalWrite(BCD_B, LOW); digitalWrite(BCD_C, LOW); digitalWrite(BCD_D, HIGH); break;
    case 9: digitalWrite(BCD_A, HIGH); digitalWrite(BCD_B, LOW); digitalWrite(BCD_C, LOW); digitalWrite(BCD_D, HIGH); break;
  }
}

void setup() {
  pinMode(FESQ, OUTPUT); pinMode(TESQ, OUTPUT); pinMode(FDIR, OUTPUT); pinMode(TDIR, OUTPUT);
  pinMode(LE, INPUT); pinMode(CE, INPUT); pinMode(CD, INPUT); pinMode(LD, INPUT);
  pinMode(LDR, INPUT);
  pinMode(BUT, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(BCD_A, OUTPUT); pinMode(BCD_B, OUTPUT); pinMode(BCD_C, OUTPUT); pinMode(BCD_D, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED, LOW);
  digitalWrite(BUZZER, LOW);
  displayDigit(0);
  delay(10);
  initialIdle = digitalRead(BUT);
  stableState = initialIdle;
  lastReading = initialIdle;
}

void loop() {
  // Debounce do botão
  int reading = digitalRead(BUT);
  if (reading != lastReading) lastDebounce = millis();
  if (millis() - lastDebounce > DEBOUNCE_MS) {
    if (reading != stableState) {
      stableState = reading;
      if (stableState != initialIdle) {
        powered = !powered;
        if (!powered) stopMotors();
        digitalWrite(LED_BUILTIN, powered ? HIGH : LOW);
      }
    }
  }
  lastReading = reading;

  if (!powered) { delay(20); return; }

  // Leitura dos sensores
  int vLE = analogRead(LE);
  int vCE = analogRead(CE);
  int vCD = analogRead(CD);
  int vLD = analogRead(LD);

  // Detecção de túnel (escuro)
  int ldrValue = analogRead(LDR);
  if (ldrValue < LDR_THRESHOLD) {
    digitalWrite(LED, HIGH);
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(LED, LOW);
    digitalWrite(BUZZER, LOW);
  }

  bool le = (vLE < SENSIBILIDADE);
  bool ce = (vCE < SENSIBILIDADE);
  bool cd = (vCD < SENSIBILIDADE);
  bool ld = (vLD < SENSIBILIDADE);

  // Modo de recuperação: nenhum sensor detecta linha
  if (!le && !ce && !cd && !ld) {
    if (lostLineTime == 0) {
      lostLineTime = millis();
      if (lastError < 0) recoveryDirection = -1;
      else if (lastError > 0) recoveryDirection = 1;
      else recoveryDirection = (lastLeftSpeed > lastRightSpeed) ? -1 : 1;
    }
    
    unsigned long timeLost = millis() - lostLineTime;
    
    if (timeLost < RECOVERY_DELAY) {
      setMotors(lastLeftSpeed, lastRightSpeed);
    }
    else if (timeLost < RECOVERY_DELAY + REVERSE_TIME) {
      setMotors(-80, -80);
    }
    else {
      if (recoveryDirection < 0) {
        setMotors(50, 120);
      } else {
        setMotors(120, 50);
      }
    }
    delay(10);
    return;
  }
  
  lostLineTime = 0;
  
  // Curvas fechadas 90°
  bool curvaFechadaEsquerda = (le && ce && !cd && !ld);
  bool curvaFechadaDireita = (!le && !ce && cd && ld);
  
  if (curvaFechadaEsquerda) {
    setMotors(0, 180);
    delay(1);
    return;
  }
  
  if (curvaFechadaDireita) {
    setMotors(180, 0);
    delay(1);
    return;
  }
  
  // Detecção de linhas horizontais
  bool linhaHorizontalAtual = detectarLinhaHorizontal(le, ce, cd, ld);
  
  if (linhaHorizontalAtual && !ultimoEstadoHorizontal && (millis() - tempoUltimaDeteccao > DEBOUNCE_FITA)) {
    contadorFitas++;
    tempoUltimaDeteccao = millis();
    displayDigit(contadorFitas % 10);
    
    if (contadorFitas >= TOTAL_LINHAS) {
      powered = false;
      stopMotors();
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      return;
    }
    
    delay(50);
  }
  ultimoEstadoHorizontal = linhaHorizontalAtual;

  // Controle PID
  int position = 0;
  int sensorCount = 0;
  
  if (le) { position += -2; sensorCount++; }
  if (ce) { position += -1; sensorCount++; }
  if (cd) { position += -1; sensorCount++; }
  if (ld) { position += 2; sensorCount++; }
  
  if (sensorCount > 0) {
    position = position / sensorCount;
  }
  
  int error = position;
  
  float P = error * Kp;
  integral += error;
  integral = constrain(integral, -100, 100);
  float I = integral * Ki;
  float D = (error - lastError) * Kd;
  
  float correction = P + I + D;
  correction = constrain(correction, -50, 50);
  
  int leftMotorSpeed = BASE_SPEED + correction;
  int rightMotorSpeed = BASE_SPEED - correction;
  
  leftMotorSpeed = constrain(leftMotorSpeed, MIN_MOTOR_SPEED, 200);
  rightMotorSpeed = constrain(rightMotorSpeed, MIN_MOTOR_SPEED, 200);
  
  lastLeftSpeed = leftMotorSpeed;
  lastRightSpeed = rightMotorSpeed;
  lastError = error;
  lastTurnFactor = error;
  
  setMotors(leftMotorSpeed, rightMotorSpeed);
  delay(1);
}