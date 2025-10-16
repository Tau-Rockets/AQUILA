// FIRMWARE - ÁQUILA -
// TBD = To be defined/done

/* 
FALTA ADICIONAR:
 2. RECEBER COMANDOS DO INFO_BTN;
 3. ENVIAR MSGS DO SETUP VIA LORA
*/

#include <Adafruit_BME280.h>
#include <Adafruit_CCS811.h>
#include <LoRa.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// ### PINOS E ENDEREÇOS ###
#define DIG_GATE 32 // Chavear alimentação do transmissor de imagens
#define ANAG_POT 33 // Posição do trimpot para alterar frequência de transmissão
#define ANAG_TXPOWER 25 // Medir tensão das baterias
#define INFO_BTN 26 // Botão para trocar informações no display
#define CS_LORA 5 // Chip Select LoRa
#define D1O_LORA 17
#define RESET_LORA 2
#define CS_SD 4 // Chip Select SD 
#define BME_ADDRESS 0x76 // Endereço I2C do BME-280
#define CCS811_ADDRESS 0x5A // Endereço I2C do CCS811
#define DISPLAY_ADDRES 0x3C

// ### RATES ###
#define BAUDRATE 115200 
#define MIN_TXPOWER 5 // Tensão crítica nas baterias em Volts (MUDAR PARA 10 MAIS TARDE)
#define OPERATING_FREQUENCY 10 // Frequência de amostragem em HZ
#define FREQUENCY_SENDING 5 // Frequência de envio de dados em Hz
#define FREQUENCY_LISTENING 80 // Frequência em Hz para ouvir dados
#define TIME2LOOK_AT_POWER 10 // Tempo para medir a tensão na bateria em Segundos
#define LOOK_CO2_MS 500 // Tempo para medir dados de CO2 em milissegundos

// ### LORA CONFIGURATION ###
#define LORA_SPREADING_FACTOR 7       // SF7 (range 6-12)
#define LORA_BANDWIDTH 125E3         // 125 kHz bandwidth
#define LORA_CODING_RATE 5           // 4/5 coding rate
#define LORA_PREAMBLE_LENGTH 6       // Minimum preamble length
#define LORA_SYNC_WORD 0x12          // Network sync word
#define LORA_TX_POWER 10             // 17 dBm transmission power
#define LORA_PA_BOOST PA_OUTPUT_PA_BOOST_PIN  // Power amplifier pin
#define LORA_DATA_RESOLUTION 1
#define MAX_INC 8 // maximo de caracteres que ele pode receber (evitar ficar preso no while)
#define MIN_LORA_FREQ 443E6 // (TBD)
#define MAX_LORA_FREQ 445E6 // (TBD)
#define DEFAULT_LORA_FREQ 915E6

// ### DISPLAY CONFIGURATION ###
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 

// ### CONSTANTES ###
#define RESOLUTION 4095
#define VREF 3.3
#define LOGGING_PATH "DATALOG.TXT"
#define R1 1 // Ohms (TBD)
#define R2 1 // Ohms (TBD)
#define POT_TOLERANCE 20 // (TBD)
#define PRESSURE_TOLERANCE 0.1 // hpa (TBD)
#define TIME_STOP_TOLERANCE 10 // segundos
#define COMMAND_TO_SEND_IMAGES "TAU1TAU" // (TBD)
#define COMMAND_TO_STOP_SENDING_IMAGES "TAU2TAU" // (TBD)
#define COMMAND_TO_LOG_DATA "TAU3TAU"
#define COMMAND_TO_STOP_LOGGING_DATA "TAU4TAU"

// ### FLAGS ###
bool sending_images = false;
bool flying = false;
bool on_mission = false;
bool log_file_open = false;
bool logging_data = false;
bool lora_ok = false;
bool bme_ok = false;
bool ccs_ok = false;
bool sd_ok = true;
bool critical_battery = false;

// Define temporizadores
const unsigned long sampling = 1000 / OPERATING_FREQUENCY; // Taxa de amostragem em ms dos dados
const unsigned long listening_sampling = 1000 / FREQUENCY_LISTENING;
const unsigned long sending_sampling = 1000/ FREQUENCY_SENDING;
const unsigned long look_power = 1000 * TIME2LOOK_AT_POWER;  
const unsigned long look_co2_level = 220;
unsigned long currentMillis = 0; // Variável para armazenar tempo decorrido em ms
unsigned long lastUpdateTime = 0;
unsigned long lastUpdatePowerTime = 0;
unsigned long lastco2_measure = 0;
unsigned long lastListen = 0;
unsigned long stop_start_time = 0;
unsigned long time_stopped = 0;
unsigned long lastSend = 0;

// Define variáveis úteis
Adafruit_BME280 bme; 
Adafruit_CCS811 ccs;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
File logFile;

float battery_level = 0;
float temperature = 0;
float pressure = 0;
float humidity = 0;
float co2 = 0;
float tvoc = 0;
float last_pressure = 0;
int last_pot_value = 0;
int pot_value = 0;
int cnt = 0;
long freq = 0;
String command = "";

void setup() {
  
  Serial.begin(BAUDRATE);
  Serial.println("Código Iniciado");
  analogSetAttenuation(ADC_11db); // Permite ler valores até 3.3 V

  // Inicializa o display OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRES)) {
    Serial.println(F("Falha ao inicializar display SSD1306"));
    while(1);
  }
  Serial.println("Display OLED inicializado com sucesso!");

  // Mede tensão na bateria
//  battery_level = measure_power();
//  // Verificação crítica da bateria
//  if(battery_level < MIN_TXPOWER) {
//    Serial.println("Nível de bateria crítico!");
//    critical_battery = true;
//    display_info("", false); // Exibe mensagem de bateria crítica
//    while(1);
//  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Iniciando Aquila");
  display.display();
  delay(2000);

  // Inicializa o LoRa
  display_info("LORA", false);
  LoRa.setPins(CS_LORA, RESET_LORA, D1O_LORA);
  if (!LoRa.begin(DEFAULT_LORA_FREQ)) {
    display_info("LORA", false);
    Serial.println("Falha ao iniciar LoRa!");
    while(1);
  }
  configureLoRa();
  display_info("LORA", true);
  Serial.println("LoRa inicializado com sucesso!");

    // Inicializa CCS811
  display_info("CCS", false);
  if(!ccs.begin(CCS811_ADDRESS)){
    display_info("CCS", false);
    Serial.println("Falha ao iniciar CCS811! Verifique conexões.");
    while(1);
  }
  while(!ccs.available());
  display_info("CCS", true);
  Serial.println("CCS811 inicializado com sucesso!");
  ccs.setEnvironmentalData(bme.readHumidity(), bme.readTemperature());

  // Inicializa BME280
  display_info("BME", false);
  if (!bme.begin(BME_ADDRESS)) {
    display_info("BME", false);
    Serial.println("Não foi possível encontrar o sensor BME280!");
    while(1);
  }
  display_info("BME", true);
  Serial.println("BME280 inicializado com sucesso!");

//   // Inicializa Cartão SD
//   if (!SD.begin(CS_SD)) {
//     Serial.println("Erro ao inicializar cartão SD!");
//     display_info("SD", false);
//     while(1);
//   }
//   display_info("SD", true);
//   Serial.println("Cartão SD inicializado com sucesso.");

  // Mostra status final com todos os módulos OK
  display_info("", true);
  delay(1000); // Delay antes de entrar no main operation
}

void loop() {
  currentMillis = millis();
  // on_mission = flying || time_stopped < TIME_STOP_TOLERANCE;

  if(!on_mission){
    // measure_pot_position();
    update_data();
    display_data();
    Serial.println("Display");
  } 

  if (currentMillis - lastUpdateTime >= sampling) {
    update_data();
    Serial.println("Update");
    lastUpdateTime = currentMillis;

    // manage_logging();
    // check_flying();
    
    if(currentMillis - lastSend >= sending_sampling){
      send_data();
      Serial.println("envio");
      
      lastSend = currentMillis;
    }
  }

  if(currentMillis - lastco2_measure >= LOOK_CO2_MS){
    if(ccs.available()){
      if(!ccs.readData()){
        co2 = ccs.geteCO2();
        tvoc = ccs.getTVOC();
      }
      ccs.setEnvironmentalData(humidity, temperature);
    }
  }

  // if (currentMillis - lastUpdatePowerTime >= look_power) {
  //   battery_level = measure_power();
    
  //   lastUpdatePowerTime = currentMillis;
  // }

   if (currentMillis - lastListen >= listening_sampling){
     listen_command();
     aquila_eye();

     lastListen = currentMillis;
   }

  // LISTEN FOR COMMANDS AT INFO BTN (TBD)
}


float measure_power(){
  // Retorna tensão das baterias em Volts
  return analogRead(ANAG_TXPOWER)*(VREF/RESOLUTION)*(R1 + R2)/R2;
}

void measure_pot_position(){
  // Atualiza posição do trimpot e a frequência de transmissão com base no valor
  pot_value = analogRead(ANAG_POT);
  if(abs(pot_value - last_pot_value) > POT_TOLERANCE){
    frequency_adjust();
    pot_value = last_pot_value;
  }
}

void update_data(){
  // Atualiza os dados dos sensores
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100; //hPa
  co2 = ccs.geteCO2();
  tvoc = ccs.getTVOC();

  Serial.println(temperature);
  Serial.println(co2);
  Serial.println(tvoc);
}

void send_data(){
  // Envia dados dos sensores via LoRa
  LoRa.beginPacket();

  LoRa.print("TAU");
  LoRa.print(temperature, LORA_DATA_RESOLUTION);
  LoRa.print("T");
  LoRa.print(pressure, LORA_DATA_RESOLUTION);
  LoRa.print("P");
  LoRa.print(humidity, LORA_DATA_RESOLUTION);
  LoRa.print("H");
  LoRa.print(co2, LORA_DATA_RESOLUTION);
  LoRa.print("C");
  LoRa.print(tvoc, LORA_DATA_RESOLUTION);
  LoRa.print("V");
  LoRa.print("TAU");

  LoRa.endPacket();
}

void send_info(String msg){
  // Envia informação do setup via LoRa
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  delay(500);
}

void check_flying(){
  // Avalia se o Áquila está voando ou não, e atualiza tempo que está parado
  if (last_pressure > 0) {
    flying = (abs(pressure - last_pressure) >= PRESSURE_TOLERANCE);
  }

  if (!flying) {
    if (stop_start_time == 0) stop_start_time = currentMillis;
    time_stopped = (currentMillis - stop_start_time) / 1000; // em segundos
  } 
  else {
    stop_start_time = 0;
    time_stopped = 0;
  }

  last_pressure = pressure;
}

void manage_logging(){
  // Gerenciar abertura e fechamento do arquivo de log
  logging_data = on_mission;

  if (!log_file_open && logging_data) {
    logFile = SD.open(LOGGING_PATH, FILE_WRITE);
    if (!logFile) {
      Serial.println("Erro ao abrir o arquivo de log");
    } else {
      Serial.println("Arquivo de log aberto para escrita!");
      log_file_open = true;
    }
  }

  if(log_file_open && !logging_data){
    logFile.close();
    Serial.println("Arquivo de log fechado (parado por tempo excessivo).");
    log_file_open = false;
  }

  if(logFile && log_file_open){
    log_data();
  }
}

void log_data(){
  // Salva os dados no cartão SD
  logFile.print(lastUpdateTime); logFile.print(",");
  logFile.print(temperature);   logFile.print(",");
  logFile.print(pressure);      logFile.print(",");
  logFile.print(humidity);      logFile.print(",");
  logFile.print(tvoc);          logFile.print(",");
  logFile.print(co2);           logFile.print("\n");
  logFile.flush();
}

void listen_command(){
  Serial.println("estou ouvindo");
  delay(3000);
  int packetSize = LoRa.parsePacket();
  // Ouve comandos para abrir ou fechar os olhos do Áquila
  if (packetSize)
  {
    command = "";
    while (LoRa.available()) {
        command += (char)LoRa.read();
    }
  }
    Serial.println(command);
    delay(3000);
  
    if(command == COMMAND_TO_SEND_IMAGES){
      sending_images = true;
    }
    else if(command == COMMAND_TO_STOP_SENDING_IMAGES) sending_images = false;
    else if (command == COMMAND_TO_LOG_DATA) logging_data = true;
    else if (command == COMMAND_TO_STOP_LOGGING_DATA) logging_data = false;
}

void aquila_eye(){
  Serial.println("entrei nos olhos");
  delay(3000);
  // Abre ou fecha os olhos do Áquila com base no comando recebido ou se ele está voando ou não
  Serial.println(sending_images);
  delay(3000);

  if(sending_images){
    digitalWrite(DIG_GATE, HIGH); // Liga transmissão de imagens
  }
  else{
    digitalWrite(DIG_GATE, LOW); // Desliga transmissão de imagens
  }
}

void frequency_adjust(){
  freq = map(pot_value, 0, RESOLUTION, MIN_LORA_FREQ, MAX_LORA_FREQ);

  LoRa.end();
  delay(100);
  if(!LoRa.begin(freq)){
    Serial.println("Falha ao ajustar frequência LoRa!");
    delay(100);
    if(!LoRa.begin(DEFAULT_LORA_FREQ)){
      Serial.println("Crítico! Nao foi possível voltar à frequência padrão!");
    }
    else{
      Serial.println("Retornado a frequência padrão!");
    }
  }
  else{
    Serial.println("Frequência LoRa ajustada para: " + String(freq/1.0E6) + " Mhz");
  }
  configureLoRa();
}

void display_data() {  
  display.clearDisplay();
  
  // Configurações do texto
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  // Cabeçalho
  display.setCursor(0,0);
  display.print("AQUILA - DADOS   ");
  display.print(battery_level, 1); 
  display.println("V");
  display.drawLine(0, 10, 128, 10, WHITE);
  
  // Dados dos sensores
  display.setCursor(0,12);
  display.print("Temp: "); display.print(temperature); display.println(" C");
  
  display.setCursor(0,22);
  display.print("Press: "); display.print(pressure); display.println(" hPa");
  
  display.setCursor(0,32);
  display.print("Umid: "); display.print(humidity); display.println(" %");
  
  display.setCursor(0,42);
  display.print("CO2: "); display.print(co2); display.println(" ppm");
  
  display.setCursor(0,52);
  display.print("TVOC: "); display.print(tvoc); display.println(" ppb");
  
  // Exibe no display
  display.display();
}

void display_info(const String& module, bool status) {

  // AJEITAR ISSO 
  display.clearDisplay();
  
  // Configurações do texto
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  // Cabeçalho com bateria (em vermelho se crítica)
  display.setCursor(0,0);
  display.print("AQUILA - START   ");
  if(critical_battery) {
    display.setTextColor(BLACK, WHITE); // Texto invertido para alerta
    display.print(battery_level, 1); 
    display.print("V");
    display.setTextColor(WHITE); // Volta ao normal
    display.setCursor(0,52);
    display.setTextColor(BLACK, WHITE); // Texto invertido
    display.print("BATERIA CRITICA!");
    display.setTextColor(WHITE); // Volta ao normal
    return;

  } else {
    display.print(battery_level, 1); 
    display.print("V");
  }
  
  display.drawLine(0, 10, 128, 10, WHITE);
  
  // Atualiza o status do módulo atual
  if (module == "LORA") lora_ok = status;
  else if (module == "BME") bme_ok = status;
  else if (module == "CCS") ccs_ok = status;
  else if (module == "SD") sd_ok = status;
  
  // Mostra status de todos os módulos
  display.setCursor(0,12);
  display.print("LORA -> ");
  display.println(lora_ok ? "OK" : "--");
  
  display.setCursor(0,22);
  display.print("BME-280 -> ");
  display.println(bme_ok ? "OK" : "--");
  
  display.setCursor(0,32);
  display.print("CCS-811 -> ");
  display.println(ccs_ok ? "OK" : "--");

  display.setCursor(0,42);
  display.print("SD reader -> ");
  display.println(sd_ok ? "OK" : "--");
  
  // Mensagem do módulo atual sendo inicializado
  if(!module.isEmpty()) {
    display.setCursor(0,52);
    display.print("Iniciando ");
    display.print(module);
    display.print("...");
  }
  
  display.display();
  delay(500); // Pequeno delay para visualização
}

void configureLoRa() {
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.setTxPower(LORA_TX_POWER, LORA_PA_BOOST);
}
