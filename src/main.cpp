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

// CONSTANTES - VELOCIDADE DE COMPETIÇÃO
const int SENSIBILIDADE = 750;    // VALOR CRÍTICO - TESTE NO SERIAL
const int LONGPRESS = 2000;
const int BASE_SPEED = 220;       // Velocidade base alta
const int TURN_SPEED = 150;       // AUMENTADO: correções mais agressivas para curvas
const int SHARP_TURN_SPEED = 200; // NOVO: velocidade específica para curvas fechadas
const int WIGGLE_DURATION = 3000;
const float WAVE_FREQUENCY = 0.8;

// VARIÁVEIS GLOBAIS
unsigned long lastDebugTime = 0;
unsigned long debugInterval = 500;
bool ligado = false;
bool isWiggling = false;
bool linhaHorizontalDetectada = false; // NOVO: flag para linha horizontal
unsigned long sineWaveStartTime = 0;
const unsigned long debounce = 50;
int turnFactor = 0;
int lastTurnFactor = 0;

// DECLARAÇÕES DE FUNÇÕES
void controleDirecao(int direitaFrente, int direitaTras, int esquerdaFrente, int esquerdaTras);
void setMotors(int leftSpeed, int rightSpeed);
void parar();
void leiturasSensores();
void atualizarLeiturasDigitais();
void debugSensores();
bool verificarBotao();
void beep(int duration);
void seguirlinhaCompeticao();
bool detectarLinhaHorizontal(); // NOVA FUNÇÃO

// INSTÂNCIAS
cIO robo;
AcksenButton button(BUT, ACKSEN_BUTTON_MODE_LONGPRESS, debounce, INPUT);

// DEFINIÇÕES DAS FUNÇÕES
void controleDirecao(int direitaFrente, int direitaTras, int esquerdaFrente, int esquerdaTras)
{
  analogWrite(FDIR, direitaFrente);
  analogWrite(TDIR, direitaTras);
  analogWrite(FESQ, esquerdaFrente);
  analogWrite(TESQ, esquerdaTras);
}

void setMotors(int leftSpeed, int rightSpeed)
{
  // Garantir que as velocidades estão dentro do range 0-255
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Motor esquerdo: FESQ para frente, TESQ para trás
  if (leftSpeed >= 0) {
    analogWrite(FESQ, leftSpeed);
    analogWrite(TESQ, 0);
  } else {
    analogWrite(FESQ, 0);
    analogWrite(TESQ, abs(leftSpeed));
  }
  
  // Motor direito: FDIR para frente, TDIR para trás
  if (rightSpeed >= 0) {
    analogWrite(FDIR, rightSpeed);
    analogWrite(TDIR, 0);
  } else {
    analogWrite(FDIR, 0);
    analogWrite(TDIR, abs(rightSpeed));
  }
}

void parar()
{
  controleDirecao(0, 0, 0, 0);
}

void leiturasSensores()
{
  // Ler valores analógicos dos sensores
  robo.readings[cIO::LD_] = analogRead(LD);
  robo.readings[cIO::CD_] = analogRead(CD);
  robo.readings[cIO::CE_] = analogRead(CE);
  robo.readings[cIO::LE_] = analogRead(LE);
}

void atualizarLeiturasDigitais()
{
  // Converter leituras analógicas para digitais baseado na sensibilidade
  for (int i = 0; i < 4; i++) {
    robo.digitalReadings[i] = (robo.readings[i] < SENSIBILIDADE);
  }
}

// NOVA FUNÇÃO: Detectar linha horizontal (todos os sensores pretos)
bool detectarLinhaHorizontal()
{
  for (int i = 0; i < 4; i++) {
    if (!robo.digitalReadings[i]) {
      return false;
    }
  }
  return true;
}

void debugSensores()
{
  if (millis() - lastDebugTime >= debugInterval) {
    lastDebugTime = millis();
    
    Serial.println("=== DEBUG SENSORES ===");
    Serial.print("SENSIBILIDADE: ");
    Serial.println(SENSIBILIDADE);
    Serial.print("LIGADO: ");
    Serial.println(ligado ? "SIM" : "NAO");
    Serial.print("WIGGLING: ");
    Serial.println(isWiggling ? "SIM" : "NAO");
    Serial.print("LINHA HORIZONTAL: ");
    Serial.println(linhaHorizontalDetectada ? "SIM" : "NAO");
    
    for (size_t i = 0; i < 4; i++) {
      Serial.print(robo.debugsensores[i]);
      Serial.print(": ");
      Serial.print(robo.readings[i]);
      Serial.print(" -> ");
      Serial.print(robo.digitalReadings[i] ? "LINHA" : "FORA");
      Serial.print(" | ");
    }
    Serial.println();
    
    Serial.print("TurnFactor: ");
    Serial.println(turnFactor);
    Serial.println("=====================");
  }
}

bool verificarBotao() 
{
  button.refreshStatus();
  bool pressed = button.onPressed();
  if (pressed) {
    Serial.println("Botao pressionado!");
  }
  return pressed;
}

void beep(int duration) 
{
  digitalWrite(BUZZ, HIGH);
  delay(duration);
  digitalWrite(BUZZ, LOW);
}

// NOVA FUNÇÃO - LÓGICA AVANÇADA PARA SEGUIR LINHA EM COMPETIÇÃO
void seguirlinhaCompeticao()
{
  // LÓGICA MELHORADA PARA CURVAS FECHADAS
  bool LE_ativo = robo.digitalReadings[cIO::LE_];
  bool CE_ativo = robo.digitalReadings[cIO::CE_];
  bool CD_ativo = robo.digitalReadings[cIO::CD_];
  bool LD_ativo = robo.digitalReadings[cIO::LD_];
  
  // CASOS ESPECIAIS PARA CURVAS FECHADAS
  if (LE_ativo && !CE_ativo && !CD_ativo && !LD_ativo) {
    // CURVA FECHADA PARA DIREITA - apenas sensor LE ativo
    Serial.println("CURVA FECHADA DIREITA DETECTADA");
    setMotors(BASE_SPEED - SHARP_TURN_SPEED, BASE_SPEED + 50);
    return;
  }
  
  if (LD_ativo && !CD_ativo && !CE_ativo && !LE_ativo) {
    // CURVA FECHADA PARA ESQUERDA - apenas sensor LD ativo
    Serial.println("CURVA FECHADA ESQUERDA DETECTADA");
    setMotors(BASE_SPEED + 50, BASE_SPEED - SHARP_TURN_SPEED);
    return;
  }
  
  if (LE_ativo && CE_ativo && !CD_ativo && !LD_ativo) {
    // CURVA SUAVE PARA DIREITA - sensores LE e CE ativos
    setMotors(BASE_SPEED - TURN_SPEED, BASE_SPEED);
    return;
  }
  
  if (LD_ativo && CD_ativo && !CE_ativo && !LE_ativo) {
    // CURVA SUAVE PARA ESQUERDA - sensores LD e CD ativos
    setMotors(BASE_SPEED, BASE_SPEED - TURN_SPEED);
    return;
  }
  
  if (LE_ativo && CE_ativo && CD_ativo && !LD_ativo) {
    // LINHA À DIREITA - virar moderadamente
    setMotors(BASE_SPEED - 80, BASE_SPEED + 40);
    return;
  }
  
  if (LD_ativo && CD_ativo && CE_ativo && !LE_ativo) {
    // LINHA À ESQUERDA - virar moderadamente
    setMotors(BASE_SPEED + 40, BASE_SPEED - 80);
    return;
  }
  
  // LÓGICA PADRÃO PARA CASOS NORMAIS
  turnFactor = 0;
  if (LE_ativo) turnFactor -= 2;
  if (CE_ativo) turnFactor -= 1;
  if (CD_ativo) turnFactor += 1;
  if (LD_ativo) turnFactor += 2;
  
  // Aplicar correção com histerese para evitar oscilações
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
  
  // Inicialização dos pinos
  for (size_t i = 0; i < 4; i++) {
    pinMode(robo.bobinas[i], OUTPUT);
    pinMode(robo.sensores[i], INPUT);
  }
  
  pinMode(BUT, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  
  digitalWrite(LED, LOW);
  digitalWrite(BUZZ, LOW);
  
  button.setLongPressInterval(LONGPRESS);
  
  Serial.println("=== SISTEMA INICIADO ===");
  Serial.println("MODO COMPETICAO - COM PARADA POR LINHA HORIZONTAL");
  Serial.print("BASE_SPEED: ");
  Serial.println(BASE_SPEED);
  Serial.print("TURN_SPEED: ");
  Serial.println(TURN_SPEED);
  Serial.print("SHARP_TURN_SPEED: ");
  Serial.println(SHARP_TURN_SPEED);
  Serial.print("Sensibilidade atual: ");
  Serial.println(SENSIBILIDADE);
}

void loop()
{
  // 1. Ler sensores
  leiturasSensores();
  
  // 2. Processar leituras
  atualizarLeiturasDigitais();
  
  // 3. Debug
  debugSensores();
  
  // 4. Verificar botão
  if (verificarBotao()) {
    if (linhaHorizontalDetectada) {
      // Se estava parado por linha horizontal, reiniciar
      linhaHorizontalDetectada = false;
      ligado = true;
      Serial.println(">>> REINICIANDO APOS LINHA HORIZONTAL <<<");
      digitalWrite(LED, HIGH);
      beep(100);
      delay(100);
      beep(100);
    } else {
      // Botão normal - ligar/desligar
      ligado = !ligado;
      if (ligado) {
        Serial.println(">>> ROBO LIGADO <<<");
        digitalWrite(LED, HIGH);
        beep(100);
        delay(100);
        beep(100);
      } else {
        Serial.println(">>> ROBO DESLIGADO <<<");
        digitalWrite(LED, LOW);
        parar();
        isWiggling = false;
        linhaHorizontalDetectada = false;
        beep(300);
      }
    }
  }

  // 5. Se linha horizontal foi detectada, ficar parado até botão
  if (linhaHorizontalDetectada) {
    parar();
    digitalWrite(LED, LOW);
    return; // Não faz mais nada até reiniciar com botão
  }

  // 6. Lógica principal se estiver ligado
  if (ligado) {
    
    // Verificar se detectou linha horizontal
    if (detectarLinhaHorizontal()) {
      Serial.println("*** LINHA HORIZONTAL DETECTADA - PARANDO ***");
      parar();
      linhaHorizontalDetectada = true;
      ligado = false;
      isWiggling = false;
      digitalWrite(LED, LOW);
      beep(500);
      delay(300);
      beep(500);
      return;
    }
    
    // Verificar se todos os sensores estão na linha (coluna) - lógica antiga
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
    
    // Modo Wiggling (busca)
    if (isWiggling) {
      Serial.println("Modo Wiggling Ativo");
      
      // Verificar se encontrou linha durante a busca
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
        beep(150);
        delay(80);
        beep(150);
      }
      // Continuar busca se não esgotou o tempo
      else if (millis() - sineWaveStartTime < WIGGLE_DURATION) {
        float time = (millis() - sineWaveStartTime) / 1000.0;
        float sineValue = sin(time * 2.0 * PI * WAVE_FREQUENCY);
        
        int leftMotorSpeed = BASE_SPEED + (sineValue * TURN_SPEED);
        int rightMotorSpeed = BASE_SPEED - (sineValue * TURN_SPEED);
        
        setMotors(leftMotorSpeed, rightMotorSpeed);
      }
      // Tempo esgotado
      else {
        Serial.println("Tempo de busca esgotado - PARANDO");
        parar();
        ligado = false;
        isWiggling = false;
        digitalWrite(LED, LOW);
      }
    }
    // Modo Seguidor de Linha Normal - USANDO A NOVA LÓGICA
    else {
      seguirlinhaCompeticao();
    }
  } else {
    // Robô desligado - garantir que está parado
    parar();
  }
  
  delay(25); // Delay reduzido para resposta ainda mais rápida
}