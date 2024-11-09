import time
import board
import busio
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw

# Impostazioni del display OLED SSD1306
WIDTH = 128
HEIGHT = 64

# Inizializza I2C e display
i2c = busio.I2C(board.SCL, board.SDA)
oled = SSD1306_I2C(WIDTH, HEIGHT, i2c)

# Funzione per disegnare la bocca
def draw_mouth(open_level):
    # Crea un'immagine vuota
    image = Image.new("1", (WIDTH, HEIGHT))
    draw = ImageDraw.Draw(image)

    # Coordinate della bocca
    mouth_x = WIDTH // 2
    mouth_y = HEIGHT // 2 + 10  # Posiziona la bocca un po' più in basso

    # Altezza della bocca variabile in base a quanto è aperta
    mouth_height = int(open_level * 20)

    # Disegna la bocca come un'ellisse variabile in altezza
    draw.ellipse(
        (mouth_x - 20, mouth_y - mouth_height, mouth_x + 20, mouth_y + mouth_height),
        outline=255,
        fill=255,
    )

    # Mostra l'immagine sul display
    oled.image(image)
    oled.show()

# Animazione di apertura e chiusura della bocca
try:
    while True:
        # Bocca si apre gradualmente
        for i in range(0, 11):
            draw_mouth(i / 10)
            time.sleep(0.05)

        # Pausa con bocca aperta
        time.sleep(0.3)

        # Bocca si chiude gradualmente
        for i in range(10, -1, -1):
            draw_mouth(i / 10)
            time.sleep(0.05)

        # Pausa con bocca chiusa
        time.sleep(0.5)

except KeyboardInterrupt:
    # Cancella lo schermo alla fine
    oled.fill(0)
    oled.show()


#pip install adafruit-circuitpython-ssd1306 pillow

