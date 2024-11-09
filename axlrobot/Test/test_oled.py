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

# Funzione per disegnare l'occhio
def draw_eye(open_level):
    # Crea un'immagine vuota
    image = Image.new("1", (WIDTH, HEIGHT))
    draw = ImageDraw.Draw(image)

    # Centra l'occhio
    eye_x = WIDTH // 2
    eye_y = HEIGHT // 2

    # Raggio dell'occhio in base a quanto è aperto
    radius = int(open_level * 20)

    # Disegna l'occhio come un cerchio con un raggio variabile
    draw.ellipse(
        (eye_x - radius, eye_y - radius, eye_x + radius, eye_y + radius),
        outline=255,
        fill=255,
    )

    # Mostra l'immagine sul display
    oled.image(image)
    oled.show()

# Animazione di apertura e chiusura dell'occhio
try:
    while True:
        # Occhio si chiude gradualmente
        for i in range(10, -1, -1):
            draw_eye(i / 10)
            time.sleep(0.01)

        # Pausa ad occhio chiuso
        time.sleep(0.2)

        # Occhio si apre gradualmente
        for i in range(0, 11):
            draw_eye(i / 10)
            time.sleep(0.01)

        # Pausa ad occhio aperto
        time.sleep(1)

except KeyboardInterrupt:
    # Cancella lo schermo alla fine
    oled.fill(0)
    oled.show()


#pip install adafruit-circuitpython-ssd1306 pillow
