#define TDIR 10  // trás direita
#define FDIR 11  // frente direita
#define TESQ 9   // trás esquerda
#define FESQ 6   // frente esquerda
#define LD A2    // sensor lateral direito
#define CD A3    // sensor central direito
#define CE A4    // sensor central esquerdo
#define LE A5    // sensor lateral esquerdo
#define BUT 7    // botão
#define LED 12   // LED
#define BUZZ 8   // Buzzer

// Pinos para o decodificador BCD
#define BCD_A 3
#define BCD_B 5  
#define BCD_C 4
#define BCD_D 2

#include <Arduino.h>
#include <math.h>
#include "AcksenButton.h"

// ENUMERAÇÕES E STRUCTS
struct controlIO
{
  enum Bobinas
  {
    TDIR_,
    FDIR_,
    TESQ_,
    FESQ_
  };

  enum Sensores
  {
    LD_,
    CD_,
    CE_,
    LE_
  };
  
  int bobinas[4] = {TDIR, FDIR, TESQ, FESQ};
  int sensores[4] = {LD, CD, CE, LE};
  char debugsensores[4][3] = {"LD", "CD", "CE", "LE"};
  int readings[4];
  bool digitalReadings[4];
} typedef cIO;

// CONSTANTES
const int SENSIBILIDADE = 700;
const int BASE_SPEED = 200;
const int AJUSTE_ESQUERDO = 14;
const int AJUSTE_DIREITO = 0;
const int TURN_SPEED = 180;
const int WIGGLE_DURATION = 3000;
const float WAVE_FREQUENCY = 0.8;

// BUZZER DESLIGADO POR PADRÃO - MUDE PARA true SE QUISER BARULHO
const bool BUZZER_ATIVO = false;

// VARIÁVEIS GLOBAIS
unsigned long lastDebugTime = 0;
unsigned long debugInterval = 500;
bool ligado = false;
bool isWiggling = false;
unsigned long sineWaveStartTime = 0;
const unsigned long debounce = 50;
int turnFactor = 0;
int lastTurnFactor = 0;

// VARIÁVEIS PARA CONTADOR
int contadorFitais = 0;
bool ultimoEstadoHorizontal = false;
unsigned long lastBuzzerTime = 0;
const unsigned long BUZZER_DURATION = 300;

// DECLARAÇÕES DE FUNÇÕES
void setMotors(int leftSpeed, int rightSpeed);
void parar();
void leiturasSensores();
void atualizarLeiturasDigitais();
void debugSensores();
bool verificarBotao();
void beep(int duration);
void seguirlinhaCorrigida();
bool detectarLinhaHorizontal();
void displayDigit(int digit);
void atualizarDisplay();

// INSTÂNCIAS
cIO robo;
AcksenButton button(BUT, ACKSEN_BUTTON_MODE_NORMAL, debounce, INPUT);

void setMotors(int leftSpeed, int rightSpeed)
{
  // APLICAR AJUSTE DE BALANCEAMENTO APENAS PARA VELOCIDADES POSITIVAS (MOTOR DIREITO)
  int adjustedLeftSpeed = leftSpeed;
  int adjustedRightSpeed = rightSpeed;
  
  if (leftSpeed >= 0) {
    adjustedLeftSpeed = leftSpeed + AJUSTE_ESQUERDO;
  }
  if (rightSpeed >= 0) {
    adjustedRightSpeed = rightSpeed + AJUSTE_DIREITO;
  }

  // Garantir que as velocidades estejam dentro dos limites
  adjustedLeftSpeed = constrain(adjustedLeftSpeed, -255, 255);
  adjustedRightSpeed = constrain(adjustedRightSpeed, -255, 255);
  
  // Motor esquerdo
  if (adjustedLeftSpeed >= 0) {
    analogWrite(FESQ, adjustedLeftSpeed);
    analogWrite(TESQ, 0);
  } else {
    analogWrite(FESQ, 0);
    analogWrite(TESQ, abs(adjustedLeftSpeed));
  }
  
  // Motor direito
  if (adjustedRightSpeed >= 0) {
    analogWrite(FDIR, adjustedRightSpeed);
    analogWrite(TDIR, 0);
  } else {
    analogWrite(FDIR, 0);
    analogWrite(TDIR, abs(adjustedRightSpeed));
  }
}

void parar()
{
  analogWrite(FESQ, 0);
  analogWrite(TESQ, 0);
  analogWrite(FDIR, 0);
  analogWrite(TDIR, 0);
}

void leiturasSensores()
{
  robo.readings[cIO::LD_] = analogRead(LD);
  robo.readings[cIO::CD_] = analogRead(CD);
  robo.readings[cIO::CE_] = analogRead(CE);
  robo.readings[cIO::LE_] = analogRead(LE);
}

void atualizarLeiturasDigitais()
{
  for (int i = 0; i < 4; i++) {
    robo.digitalReadings[i] = (robo.readings[i] < SENSIBILIDADE);
  }
}

bool detectarLinhaHorizontal()
{
  for (int i = 0; i < 4; i++) {
    if (!robo.digitalReadings[i]) {
      return false;
    }
  }
  return true;
}

void displayDigit(int digit)
{
  switch (digit) {
    case 0:
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, LOW);
      break;
    case 1:
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, LOW);
      break;
    case 2:
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, HIGH);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, LOW);
      break;
    case 3:
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, HIGH);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, LOW);
      break;
    case 4:
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, HIGH);
      digitalWrite(BCD_D, LOW);
      break;
    case 5:
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, HIGH);
      digitalWrite(BCD_D, LOW);
      break;
    case 6:
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, HIGH);
      digitalWrite(BCD_C, HIGH);
      digitalWrite(BCD_D, LOW);
      break;
    case 7:
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, HIGH);
      digitalWrite(BCD_C, HIGH);
      digitalWrite(BCD_D, LOW);
      break;
    case 8:
      digitalWrite(BCD_A, LOW);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, HIGH);
      break;
    case 9:
      digitalWrite(BCD_A, HIGH);
      digitalWrite(BCD_B, LOW);
      digitalWrite(BCD_C, LOW);
      digitalWrite(BCD_D, HIGH);
      break;
  }
}

void atualizarDisplay()
{
  displayDigit(contadorFitais % 10);
}

void debugSensores()
{
  if (millis() - lastDebugTime >= debugInterval) {
    lastDebugTime = millis();
    
    Serial.println("=== DEBUG SENSORES ===");
    Serial.print("LIGADO: "); Serial.println(ligado ? "SIM" : "NAO");
    Serial.print("WIGGLING: "); Serial.println(isWiggling ? "SIM" : "NAO");
    Serial.print("CONTADOR FITAS: "); Serial.println(contadorFitais);
    Serial.print("BUZZER: "); Serial.println(BUZZER_ATIVO ? "LIGADO" : "DESLIGADO");
    Serial.print("LINHA HORIZONTAL: "); Serial.println(detectarLinhaHorizontal() ? "SIM" : "NAO");
    Serial.print("TURNFACTOR: "); Serial.println(turnFactor);
    
    for (size_t i = 0; i < 4; i++) {
      Serial.print(robo.debugsensores[i]);
      Serial.print(": "); Serial.print(robo.readings[i]);
      Serial.print(" -> "); Serial.print(robo.digitalReadings[i] ? "LINHA" : "FORA");
      Serial.print(" | ");
    }
    Serial.println();
    Serial.println("=====================");
  }
}

bool verificarBotao() 
{
  button.refreshStatus();
  return button.onPressed();
}

// BUZZER SÓ FUNCIONA SE BUZZER_ATIVO = true
void beep(int duration) 
{
  if (BUZZER_ATIVO) {
    digitalWrite(BUZZ, HIGH);
    delay(duration);
    digitalWrite(BUZZ, LOW);
  }
}

void seguirlinhaCorrigida()
{
  bool LE_ativo = robo.digitalReadings[cIO::LE_];
  bool CE_ativo = robo.digitalReadings[cIO::CE_];
  bool CD_ativo = robo.digitalReadings[cIO::CD_];
  bool LD_ativo = robo.digitalReadings[cIO::LD_];
  
  if (LE_ativo && CE_ativo && !CD_ativo && !LD_ativo) {
    setMotors(BASE_SPEED - 180, BASE_SPEED + 80);
    Serial.println("CURVA DIREITA AGUDA");
    return;
  }
  
  if (LD_ativo && CD_ativo && !CE_ativo && !LE_ativo) {
    setMotors(BASE_SPEED + 80, BASE_SPEED - 180);
    Serial.println("CURVA ESQUERDA AGUDA");
    return;
  }
  
  if (LE_ativo && !CE_ativo && !CD_ativo && !LD_ativo) {
    setMotors(BASE_SPEED - 160, BASE_SPEED + 60);
    Serial.println("CURVA DIREITA MUITO FECHADA");
    return;
  }
  
  if (LD_ativo && !CD_ativo && !CE_ativo && !LE_ativo) {
    setMotors(BASE_SPEED + 60, BASE_SPEED - 160);
    Serial.println("CURVA ESQUERDA MUITO FECHADA");
    return;
  }
  
  if (LE_ativo && CE_ativo && CD_ativo && !LD_ativo) {
    setMotors(BASE_SPEED - 100, BASE_SPEED + 40);
    return;
  }
  
  if (LD_ativo && CD_ativo && CE_ativo && !LE_ativo) {
    setMotors(BASE_SPEED + 40, BASE_SPEED - 100);
    return;
  }
  
  turnFactor = 0;
  if (LE_ativo) turnFactor -= 2;
  if (CE_ativo) turnFactor -= 1;
  if (CD_ativo) turnFactor += 1;
  if (LD_ativo) turnFactor += 2;
  
  if (abs(turnFactor - lastTurnFactor) > 3) {
    turnFactor = lastTurnFactor + ((turnFactor > lastTurnFactor) ? 1 : -1);
  }
  
  int leftMotorSpeed = BASE_SPEED + (turnFactor * TURN_SPEED);
  int rightMotorSpeed = BASE_SPEED - (turnFactor * TURN_SPEED);
  
  setMotors(leftMotorSpeed, rightMotorSpeed);
  lastTurnFactor = turnFactor;
}

void setup()
{
  Serial.begin(115200);
  
  for (size_t i = 0; i < 4; i++) {
    pinMode(robo.bobinas[i], OUTPUT);
    pinMode(robo.sensores[i], INPUT);
  }
  
  pinMode(BCD_A, OUTPUT);
  pinMode(BCD_B, OUTPUT);
  pinMode(BCD_C, OUTPUT);
  pinMode(BCD_D, OUTPUT);
  
  pinMode(BUT, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  
  digitalWrite(LED, LOW);
  digitalWrite(BUZZ, LOW);
  
  displayDigit(0);
  
  Serial.println("=== SISTEMA INICIADO ===");
  Serial.print("BUZZER: ");
  Serial.println(BUZZER_ATIVO ? "ATIVO (COM BARULHO)" : "DESATIVADO (SILENCIOSO)");
  Serial.print("BASE_SPEED: "); Serial.println(BASE_SPEED);
  Serial.println("WIGGLING USANDO SETMOTORS NORMAL");
}

void loop()
{
  // 1. Ler sensores
  leiturasSensores();
  atualizarLeiturasDigitais();
  
  // 2. Verificar e contar fitas horizontais
  bool linhaHorizontalAtual = detectarLinhaHorizontal();
  
  if (linhaHorizontalAtual && !ultimoEstadoHorizontal && ligado && !isWiggling) {
    contadorFitais++;
    Serial.print("*** FITA HORIZONTAL DETECTADA - CONTADOR: ");
    Serial.println(contadorFitais);
    
    // Só ativa o buzzer se BUZZER_ATIVO = true
    if (BUZZER_ATIVO) {
      digitalWrite(BUZZ, HIGH);
      lastBuzzerTime = millis();
    }
    
    atualizarDisplay();
  }
  ultimoEstadoHorizontal = linhaHorizontalAtual;
  
  // Desligar buzzer após 300ms (se estiver ativo)
  if (BUZZER_ATIVO && digitalRead(BUZZ) == HIGH && (millis() - lastBuzzerTime >= BUZZER_DURATION)) {
    digitalWrite(BUZZ, LOW);
  }
  
  // 3. Debug
  debugSensores();
  
  // 4. Verificar botão (apenas para ligar/desligar o robô)
  if (verificarBotao()) {
    ligado = !ligado;
    if (ligado) {
      Serial.println(">>> ROBO LIGADO <<<");
      digitalWrite(LED, HIGH);
      beep(100); delay(100); beep(100);
    } else {
      Serial.println(">>> ROBO DESLIGADO <<<");
      digitalWrite(LED, LOW);
      parar();
      isWiggling = false;
      beep(300);
    }
  }

  // 5. Lógica principal se estiver ligado
  if (ligado) {
    bool todosNaLinha = true;
    for (int i = 0; i < 4; i++) {
      if (!robo.digitalReadings[i]) {
        todosNaLinha = false;
        break;
      }
    }
    
    if (todosNaLinha) {
      Serial.println("* COLUNA DETECTADA - INICIANDO WIGGLE *");
      isWiggling = true;
      sineWaveStartTime = millis();
      beep(300);
    }
    
    if (isWiggling) {
      bool algumSensorNaLinha = false;
      for (int i = 0; i < 4; i++) {
        if (robo.digitalReadings[i]) {
          algumSensorNaLinha = true;
          break;
        }
      }
      
      if (algumSensorNaLinha && !todosNaLinha) {
        Serial.println("Linha encontrada durante busca!");
        isWiggling = false;
        beep(150); delay(80); beep(150);
      }
      else if (millis() - sineWaveStartTime < WIGGLE_DURATION) {
        float time = (millis() - sineWaveStartTime) / 1000.0;
        float sineValue = sin(time * 2.0 * PI * WAVE_FREQUENCY);
        
        int leftMotorSpeed = 220 + (sineValue * 35);  // Entre 185 e 255
        int rightMotorSpeed = 220 - (sineValue * 35); // Entre 185 e 255
        
        // USAR A FUNÇÃO SETMOTORS NORMAL (COM AJUSTES)
        setMotors(leftMotorSpeed, rightMotorSpeed);
      }
      else {
        Serial.println("Tempo de busca esgotado - PARANDO");
        parar();
        ligado = false;
        isWiggling = false;
        digitalWrite(LED, LOW);
      }
    }
    else {
      seguirlinhaCorrigida();
    }
  } else {
    parar();
  }
  
  delay(25);
}