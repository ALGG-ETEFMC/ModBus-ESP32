/*
  Exemplo: servidor Modbus TCP com debug no Serial Monitor
  Mostra conexao/desconexao de clientes e, para cada pacote recebido:
  funcao, endereco do registrador e dados (e excecoes, se houver).
*/
#ifdef ESP8266
 #include <ESP8266WiFi.h>
#else
 #include <WiFi.h>
#endif
#include <ModbusIP_ESP8266.h>

const char* ssid = "SUA_REDE";
const char* pass = "SUA_SENHA";

ModbusIP mb;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  mb.server();            // porta 502, debug ATIVO (padrao)
  // mb.server(502, false); // porta 502, debug desligado
  // mb.setDebug(false);    // tambem pode ligar/desligar a qualquer momento

  mb.addHreg(0, 123);     // Holding Registers 0 e 1
  mb.addHreg(1, 456);
  mb.addCoil(0, false);   // Coil 0
}

void loop() {
  mb.task();
  delay(10);
}
