#include <SPI.h>
#include <LoRa.h>

#define CS_LORA 5 // Chip Select LoRa
#define D1O_LORA 17
#define RESET_LORA 2
#define DEFAULT_LORA_FREQ 915E6

// ### LORA CONFIGURATION ###
#define LORA_SPREADING_FACTOR 7     // SF7 (range 6-12)
#define LORA_BANDWIDTH 125E3        // 125 kHz bandwidth
#define LORA_CODING_RATE 5          // 4/5 coding rate
#define LORA_PREAMBLE_LENGTH 6      // Minimum preamble length
#define LORA_SYNC_WORD 0x12         // Network sync word
#define LORA_TX_POWER 10            // 17 dBm transmission power
#define LORA_PA_BOOST PA_OUTPUT_PA_BOOST_PIN // Power amplifier pin
#define LORA_DATA_RESOLUTION 1
#define MAX_INC 8 // maximo de caracteres que ele pode receber (evitar ficar preso no while)
#define MIN_LORA_FREQ 443E6 // (TBD)
#define MAX_LORA_FREQ 445E6 // (TBD)

#define BTN_FPV_ON 13
#define BTN_FPV_OFF 33

// Variaveis para controle da camera 
int fpv_on_state = HIGH; 
int fpv_off_state = HIGH; // será???

void configureLoRa() {
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.setTxPower(LORA_TX_POWER, LORA_PA_BOOST);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(BTN_FPV_ON, INPUT_PULLUP);
  pinMode(BTN_FPV_OFF, INPUT_PULLUP);

  Serial.println("LoRa Receiver");
  LoRa.setPins(CS_LORA, RESET_LORA, D1O_LORA);
  if (!LoRa.begin(DEFAULT_LORA_FREQ)) {
    Serial.println("Falha ao iniciar LoRa!");
    while(1);
  }
  configureLoRa();
}

void loop() {
  
  fpv_on_state = digitalRead(BTN_FPV_ON);
  fpv_off_state = digitalRead(BTN_FPV_OFF);

  if(fpv_on_state == LOW) // Botão Pressionado (PULLUP -> LOW = Pressionado)
  {
    LoRa.beginPacket();
    LoRa.print("TAU1TAU");
    LoRa.endPacket(); 
    Serial.println("TAU1TAU - Habilitando imagens");
  }
  if(fpv_off_state == LOW) // Botão Pressionado (PULLUP -> LOW = Pressionado)
  {
    LoRa.beginPacket();
    LoRa.print("TAU2TAU");
    LoRa.endPacket(); 
    Serial.println("TAU2TAU - Desabilitando imagens");
  }
  else // Botão Solto (HIGH)
  {
    // Tenta receber pacote (Lógica de Receiver)
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      Serial.print("Recebido: '");

      // read packet
      while (LoRa.available()) {
        Serial.print((char)LoRa.read());
      }

      // print RSSI of packet
      Serial.print("' com RSSI ");
      Serial.println(LoRa.packetRssi());
    }
  }
}
