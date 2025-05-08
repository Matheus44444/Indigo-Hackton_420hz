// Código para o ESP32 da Bandana (Receptor ESP-NOW) - MODIFICADO para Sensor IR
#include <esp_now.h>
#include <WiFi.h>

// --- Definições de Pinos (GPIO Indiretos) ---
#define MOTOR_ESQUERDA_PIN 14
#define MOTOR_DIREITA_PIN 26 
#define MOTOR_INFERIOR_IR_PIN 37 // Pino para o motor/vibrador do sensor IR inferior

#define FREQ_MIN_ALERT 500  // Frequência em Hz para distância DIST_MAX_PROPORTIONAL_ALERT (alerta menos urgente)
#define FREQ_MAX_ALERT 2500 // Frequência em Hz para distância DIST_MIN_PROPORTIONAL_ALERT (alerta mais urgente)

#define DIST_MAX_PROPORTIONAL_ALERT 150 // Distância máxima (cm) para iniciar alerta proporcional
#define DIST_MIN_PROPORTIONAL_ALERT 5  // Distância mínima (cm) para alerta máximo proporcional

// --- Estrutura de Dados para ESP-NOW ---
// Deve ser a MESMA no transmissor e no receptor
typedef struct struct_message {
    float distancia_frente;
    float distancia_esquerda;
    float distancia_direita;
    // float distancia_tras; // Removido, pois não é mais enviado pelo transmissor
    bool obstaculo_frente;
    bool obstaculo_esquerda;
    bool obstaculo_direita;
    bool obstaculo_inferior_ir; // Campo para o sensor IR inferior
} struct_message;

struct_message dadosRecebidos;

// --- Função de Callback para ESP-NOW ---
// ATENÇÃO: A assinatura desta função mudou nas versões mais recentes da biblioteca ESP32
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  Serial.print("Dados recebidos de: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(info->src_addr[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  memcpy(&dadosRecebidos, incomingData, sizeof(dadosRecebidos));
  Serial.println("Conteúdo dos dados:");
  Serial.print("Frente: "); Serial.print(dadosRecebidos.distancia_frente); Serial.print(" cm, Obstáculo: "); Serial.println(dadosRecebidos.obstaculo_frente);
  Serial.print("Esquerda: "); Serial.print(dadosRecebidos.distancia_esquerda); Serial.print(" cm, Obstáculo: "); Serial.println(dadosRecebidos.obstaculo_esquerda);
  Serial.print("Direita: "); Serial.print(dadosRecebidos.distancia_direita); Serial.print(" cm, Obstáculo: "); Serial.println(dadosRecebidos.obstaculo_direita);
  // Serial.print("Trás/Baixo: "); Serial.print(dadosRecebidos.distancia_tras); // Removido
  Serial.print("IR Inferior Obstáculo: "); Serial.println(dadosRecebidos.obstaculo_inferior_ir);
  Serial.println("-------------------------------------");

  // Lógica para acionar os motores/vibradores
if (dadosRecebidos.obstaculo_frente) { // obstaculo_frente é true se distancia_frente < 50cm (definido no transmissor) e > 0
    float current_dist_cm = dadosRecebidos.distancia_frente;
    long frequency;

    // Mapeia a distância para a frequência: quanto menor a distância, maior a frequência.
    // Multiplicamos por 10 para usar uma casa decimal da distância no map, melhorando a suavidade da transição.
    frequency = map((long)(current_dist_cm * 10),
                    (long)(DIST_MAX_PROPORTIONAL_ALERT * 10),
                    (long)(DIST_MIN_PROPORTIONAL_ALERT * 10),
                    FREQ_MIN_ALERT,
                    FREQ_MAX_ALERT);
    
    // Garante que a frequência calculada esteja dentro dos limites definidos, caso a distância esteja fora do esperado pelo map.
    frequency = constrain(frequency, FREQ_MIN_ALERT, FREQ_MAX_ALERT);
    
    tone(MOTOR_DIREITA_PIN, frequency);
    delay(100);
    Serial.print("ALERTA FRONTAL PROPORCIONAL! Dist: ");
    Serial.print(current_dist_cm);
    Serial.print(" cm, Freq: ");
    Serial.println(frequency);
  } else {
    noTone(MOTOR_DIREITA_PIN); // Desliga o tom se não houver obstáculo frontal

  }

  if (dadosRecebidos.obstaculo_esquerda) { // obstaculo_frente é true se distancia_frente < 50cm (definido no transmissor) e > 0
    float current_dist_cm = dadosRecebidos.distancia_esquerda;
    long frequency;

    // Mapeia a distância para a frequência: quanto menor a distância, maior a frequência.
    // Multiplicamos por 10 para usar uma casa decimal da distância no map, melhorando a suavidade da transição.
    frequency = map((long)(current_dist_cm * 10),
                    (long)(DIST_MAX_PROPORTIONAL_ALERT * 10),
                    (long)(DIST_MIN_PROPORTIONAL_ALERT * 10),
                    FREQ_MIN_ALERT,
                    FREQ_MAX_ALERT);
    
    // Garante que a frequência calculada esteja dentro dos limites definidos, caso a distância esteja fora do esperado pelo map.
    frequency = constrain(frequency, FREQ_MIN_ALERT, FREQ_MAX_ALERT);
    
    tone(MOTOR_ESQUERDA_PIN, frequency);
    Serial.print("ALERTA ESQUERDO PROPORCIONAL! Dist: ");
    Serial.print(current_dist_cm);
    Serial.print(" cm, Freq: ");
    Serial.println(frequency);
  } else {
    noTone(MOTOR_ESQUERDA_PIN); // Desliga o tom se não houver obstáculo frontal
  }

    if (dadosRecebidos.obstaculo_direita) { // obstaculo_frente é true se distancia_frente < 50cm (definido no transmissor) e > 0
    float current_dist_cm = dadosRecebidos.distancia_direita;
    long frequency;

    // Mapeia a distância para a frequência: quanto menor a distância, maior a frequência.
    // Multiplicamos por 10 para usar uma casa decimal da distância no map, melhorando a suavidade da transição.
    frequency = map((long)(current_dist_cm * 10),
                    (long)(DIST_MAX_PROPORTIONAL_ALERT * 10),
                    (long)(DIST_MIN_PROPORTIONAL_ALERT * 10),
                    FREQ_MIN_ALERT,
                    FREQ_MAX_ALERT);
    
    // Garante que a frequência calculada esteja dentro dos limites definidos, caso a distância esteja fora do esperado pelo map.
    frequency = constrain(frequency, FREQ_MIN_ALERT, FREQ_MAX_ALERT);
    
    tone(MOTOR_DIREITA_PIN, frequency);
    Serial.print("ALERTA DIREITA PROPORCIONAL! Dist: ");
    Serial.print(current_dist_cm);
    Serial.print(" cm, Freq: ");
    Serial.println(frequency);
  } else {
    noTone(MOTOR_DIREITA_PIN); // Desliga o tom se não houver obstáculo frontal
  }


  
  // Lógica para o motor/vibrador do sensor IR inferior
  if (dadosRecebidos.obstaculo_inferior_ir) {
    digitalWrite(MOTOR_INFERIOR_IR_PIN, HIGH); 
    Serial.println("ALERTA: Obstáculo INFERIOR (IR)!");
  } else {
    digitalWrite(MOTOR_INFERIOR_IR_PIN, LOW);  
  }
}
 
void setup() {
  Serial.begin(115200);
  
  pinMode(MOTOR_ESQUERDA_PIN, OUTPUT);
  pinMode(MOTOR_DIREITA_PIN, OUTPUT);
  pinMode(MOTOR_INFERIOR_IR_PIN, OUTPUT); // Configura o pino do motor IR

  digitalWrite(MOTOR_ESQUERDA_PIN, LOW);
  digitalWrite(MOTOR_DIREITA_PIN, LOW);
  digitalWrite(MOTOR_INFERIOR_IR_PIN, LOW); // Garante que comece desligado
  
  WiFi.mode(WIFI_STA);
  Serial.print("Endereço MAC deste ESP32 (Receptor): ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP32 Bandana (Receptor com IR) configurado e aguardando dados...");
}
 
void loop() {
  delay(100);
}

