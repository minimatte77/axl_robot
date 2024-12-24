#include <Arduino.h>
#include <ESP32Servo.h>
#include "axlrobot.h"
#include <WiFi.h>
#include <WebServer.h> // Libreria per il server HTTP
#include <PubSubClient.h>
#include <time.h>

// Credenziali per la rete Wi-Fi esistente (modalità STA)
const char* ssid = "Piano_Terra_WIFI";       // Nome della rete Wi-Fi esistente
const char* password = "monster600"; // Password della rete Wi-Fi esistente
// Credenziali per l'Access Point
const char* ap_ssid = "axl_robot";  // Nome dell'hotspot
const char* ap_password = "monster600";   // Password dell'hotspot (minimo 8 caratteri)
/* Time*/
// Server NTP per sincronizzazione
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600; // Offset GMT (es. Italia: GMT+1 => 3600 secondi)
const int daylightOffset_sec = 3600; // Ora legale (1 ora)

/* PER MESSAGGI MQTT*/
#define MAX_MESSAGES 20
String messages[MAX_MESSAGES];
int messageIndex = 0;
/* LED */
#define LED_PIN GPIO_NUM_48  // Cambia questo valore con il GPIO corretto se necessario

/*
Servo servo1;
Servo servo2;
Servo servo11;
Servo servo22;
const int servoPin1 = 13; // Pin collegato al segnale del servo
const int servoPin2 = 14; // Pin collegato al segnale del servo
const int servoPin11 = 12; // Pin collegato al segnale del servo
const int servoPin22 = 11; // Pin collegato al segnale del servo
*/
const int pwmChannel = 0; // Canale PWM
const int freq = 50;      // Frequenza in Hz (tipica per i servomotori)
const int resolution = 8; // Risoluzione a 8 bit

// Configurazione IP statico per AP
IPAddress local_ip(192, 168, 1, 170);     // IP statico dell'AP ESP32
IPAddress gateway(192, 168, 1, 1);       // Gateway della rete
IPAddress subnet(255, 255, 255, 0);      // Subnet mask


// Funzione per inizializzare la connessione Wi-Fi in modalità STA
const unsigned long connectionTimeout = 10000; // 10 secondi

// Crea un'istanza del server HTTP sulla porta 80
WebServer server(80);

/* AXL */
AxlRobot robot;

/* MQTT */
// Configurazione MQTT
const char* mqttServer = "192.168.1.58"; // Sostituisci con l'IP del Raspberry Pi
const int mqttPort = 1883;
const char* mqttTopic = "axl/topic";
const char* mqttMove = "axl/move";

WiFiClient espClient;
PubSubClient client(espClient);

/* FUNZIONE PER Time*/
void setupTime() {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("Sincronizzazione con NTP...");
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Errore nella sincronizzazione dell'orario.");
        return;
    }
    Serial.println("Sincronizzazione completata.");
}

// Funzione per gestire i comandi ricevuti
void handleCommand() {
    // Controlla se è stato passato un parametro "cmd"
    if (server.hasArg("cmd")) {
        String command = server.arg("cmd");
        Serial.print("Comando ricevuto: ");
        Serial.println(command);

        // Rispondi con una conferma
        server.send(200, "text/plain", "Comando ricevuto: " + command);

        // Esegui azioni in base al comando
        if (command == "LED_ON") {
            Serial.println("Accensione LED!");
            // Aggiungi qui il codice per accendere un LED
        } else if (command == "LED_OFF") {
            Serial.println("Spegnimento LED!");
            // Aggiungi qui il codice per spegnere un LED
        } else {
            Serial.println("Comando non riconosciuto.");
        }
    } else {
        server.send(400, "text/plain", "Parametro 'cmd' mancante.");
    }
}

// Funzione per gestire la pagina principale
void handleRoot() {
    String html = R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
            <title>AXL-ESP32 Server</title>
            <meta http-equiv="refresh" content="5">
            <style>
                body { font-family: Arial, sans-serif; margin: 20px; }
                h1 { color: #333; }
                ul { list-style-type: none; padding: 0; }
                li { background: #f9f9f9; margin: 5px 0; padding: 10px; border: 1px solid #ddd; border-radius: 5px; }
            </style>
        </head>
        <body>
            <h1>AXL-ESP32 Server HTTP</h1>
            <p>Invia un comando usando l'URL: <code>?cmd=LED_ON</code> o <code>?cmd=LED_OFF</code>.</p>
            <h1>MQTT Messages</h1>
            <ul>
            )rawliteral";
    // Aggiungi i messaggi ricevuti alla pagina
    for (int i = 0; i < MAX_MESSAGES; i++) {
        int index = (messageIndex + i) % MAX_MESSAGES;
        if (messages[index] != "") {
            html += "<li>" + messages[index] + "</li>";
        }
    }

    html += R"rawliteral(
                </ul>
            </body>
            </html>
        )rawliteral";

    server.send(200, "text/html", html);
}

void setupWiFiAPSTA() {
    WiFi.mode(WIFI_AP_STA);
    Serial.print("Tentativo di connessione alla rete Wi-Fi: ");
    Serial.println(ssid);
/*
    Serial.print("Configurando l'Access Point con SSID: ");
    Serial.println(ap_ssid);
    WiFi.softAPConfig(local_ip, gateway, subnet);
*/
    WiFi.softAP(ap_ssid, ap_password);
    //
    WiFi.begin(ssid, password); // Inizia la connessione alla rete Wi-Fi principale

    unsigned long startAttemptTime = millis();

    // Aspetta fino a che non si connette o scade il timeout
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < connectionTimeout) {
        delay(500);
        Serial.print(".");
        neopixelWrite(LED_PIN,RGB_BRIGHTNESS,0,0); // Red
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnesso alla rete principale!");
        Serial.print("Indirizzo IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("Indirizzo IP AP: ");
        Serial.println(WiFi.softAPIP());
        neopixelWrite(LED_PIN,0,0,RGB_BRIGHTNESS); // Blue
        delay(1000);
    } else {
        Serial.println("\nConnessione fallita. Attivazione solo Access Point.");
        neopixelWrite(LED_PIN,RGB_BRIGHTNESS,0,0); // Red
        delay(1000);
        Serial.print("Indirizzo IP AP: ");
        Serial.println(WiFi.softAPIP());
        neopixelWrite(LED_PIN,0,RGB_BRIGHTNESS,0); // Blue
        delay(1000);
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    message += (char)payload[i];
  }
  // Recupera e stampa data e ora
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
      char timeString[30];
      strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
      // Salva il messaggio nel buffer
      message = "[" + String(timeString) + "] " + message;
      messages[messageIndex] = message;
      messageIndex = (messageIndex + 1) % MAX_MESSAGES;

      Serial.print(" [");
      Serial.print(timeString);
      Serial.println("]");
  } else {
      Serial.println(" [Orario non disponibile]");
      // Salva il messaggio nel buffer
      message = "[NO TIME] " + message;
      messages[messageIndex] = message;
      messageIndex = (messageIndex + 1) % MAX_MESSAGES;
  }
}

void setupMQTT()
{
    // Connessione al broker MQTT
    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback);

    while (!client.connected()) {
      Serial.println("Connecting to MQTT...");
      if (client.connect("ESP32Client")) {
        Serial.println("Connected to MQTT broker");
        client.subscribe(mqttTopic);
        client.subscribe(mqttMove);
      } else {
        Serial.print("Failed with state ");
        Serial.println(client.state());
        delay(2000);
      }
    }
}

void setup() {
    /* SERIAL*/
    Serial.begin(115200);
    delay(100);
    Serial.println("Avvia WiFi");
    delay(100);
    setupWiFiAPSTA();
    // Configura il Wi-Fi client (AP+STA) e l'Access Point
    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());
    delay(1000);
    // Configura le route del server HTTP
    server.on("/", handleRoot);        // Pagina principale
    server.on("/command", handleCommand); // Endpoint per i comandi

    // Avvia il server
    server.begin();
    Serial.println("Server HTTP avviato.");
    /* AXL */ 
    //robot.init();
    //robot.setSpeed(100);
    /* Led */
    // No need to initialize the RGB LED
    pinMode(LED_PIN , OUTPUT);      // Set the RGB_PWR pin to OUTPUT
    digitalWrite(LED_PIN , HIGH);   // Turn on the RGB_PWR (LDO2)
/* SERVI */    
/*
    servo1.attach(servoPin1);
    servo2.attach(servoPin2);
    servo11.attach(servoPin11);
    servo22.attach(servoPin22);
*/
    /* */
    neopixelWrite(LED_PIN,RGB_BRIGHTNESS,0,0); // Red
    /* SETUP MQTT */
    setupMQTT();
    //INIZIALIZZA OROLOGIO 
    setupTime();
}

// the loop function runs over and over again forever
void loop() {
  // Gestisce MQTT
  client.loop();
  // Gestisci le richieste HTTP
  server.handleClient();
  //
  #ifdef LED_PIN
    /*
    digitalWrite(LED_PIN, HIGH);   // Turn the RGB LED white
    delay(1000);
    digitalWrite(LED_PIN, LOW);    // Turn the RGB LED off
    delay(1000);

    neopixelWrite(LED_PIN,RGB_BRIGHTNESS,0,0); // Red
    delay(1000);
    neopixelWrite(LED_PIN,0,RGB_BRIGHTNESS,0); // Green
    delay(1000);
    neopixelWrite(LED_PIN,0,0,RGB_BRIGHTNESS); // Blue
    delay(1000);
    neopixelWrite(LED_PIN,0,0,0); // Off / black
    delay(1000);
    */

  #endif
  //
  // Pubblicazione di un messaggio
  // client.publish(mqttTopic, "Hello from ESP32!");

//  servo1.write(0);
//  delay(1000);
//  servo1.write(45);
/*
  delay(1000);
  servo1.write(90);
  delay(1000);
  servo2.write(90);
  delay(1000);
  servo11.write(50);
  delay(1000);
  servo22.write(90);
  delay(1000);
*/
//  servo11.write(90);
//  servo22.write(120);
//  delay(1000);
//  servo1.write(135);
//  delay(1000);
//  servo1.write(90);
//  delay(1000);

  /*
  for(int i = 30; i <= 150; i++) {
      servo1.write(i);
      delay(22);
  }
  for(int i = 150; i >= 30; i--) {
      servo1.write(i);
      delay(22);
  }  delay(1000);  
  */
  /*
  Serial.println("0 Gradi");
  // Muovi il servo a 90 gradi
  Serial.println("90 Gradi");

  // Muovi il servo a 180 gradi
  Serial.println("180 Gradi");
  */

}

