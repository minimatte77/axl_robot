import paho.mqtt.client as mqtt

# Configurazione del broker
BROKER = "192.168.1.58"  # Indirizzo IP del broker
PORT = 1883              # Porta MQTT standard
TOPIC = "esp32/topic"    # Il topic usato dall'ESP32

# Funzioni callback
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connesso al broker!")
        # Sottoscrivi al topic
        client.subscribe(TOPIC)
    else:
        print(f"Errore di connessione: {rc}")

def on_message(client, userdata, msg):
    print(f"Messaggio ricevuto su {msg.topic}: {msg.payload.decode()}")

def on_publish(client, userdata, mid):
    print(f"Messaggio pubblicato con ID: {mid}")

# Creazione del client MQTT
client = mqtt.Client()

# Imposta le callback
client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish

# Connessione al broker
try:
    client.connect(BROKER, PORT, 60)
    print(f"Connesso al broker {BROKER} sulla porta {PORT}")
except Exception as e:
    print(f"Errore di connessione: {e}")
    exit(1)

# Simulazione di invio di un messaggio
client.loop_start()
client.publish(TOPIC, "Simulazione ESP32")

# Mantieni il client attivo per ascoltare i messaggi
try:
    while True:
        pass
except KeyboardInterrupt:
    print("\nDisconnessione...")
    client.loop_stop()
    client.disconnect()
