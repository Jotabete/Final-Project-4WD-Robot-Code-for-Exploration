import RPi.GPIO as GPIO
import threading
import time
import math

counter_x = 0
counter_y = 0
previousState = GPIO.HIGH  # Status sebelumnya dari sensor
direction = 1 # 1 : untuk N/R sumbu positif ; -1 : untuk W/L sumbu negatif 
orientation = 0 # 0 : untuk vertikal (N/W) ; 1 : untuk horizontal (R/L)
jarakTempuh_x = 0.0
jarakTempuh_y = 0.0
posisi_x = 0
posisi_y = 0
count = 0 
jarakTempuh = 0


# Fungsi untuk membaca sensor dalam thread terpisah
def tempuh_to_posgrid(pause_event, sensorPin):
    global counter_x, counter_y, jarakTempuh_x, jarakTempuh_y, posisi_x, posisi_y, previousState, direction, orientation, count, jarakTempuh
    # Konstanta untuk perhitungan jarak tempuh
    kelilingRoda = 21.2  # Keliling roda dalam cm
    jumlahLubangPerPutaran = 20  # Jumlah lubang per putaran encoder
    while True:
        # Mengecek apakah harus menjeda
        while pause_event.is_set():
            time.sleep(0.1)  # Menunggu sebelum memeriksa lagi
            
        # Set mode pin GPIO
        GPIO.setmode(GPIO.BCM)
        
        # Konfigurasi GPIO
        GPIO.setup(sensorPin, GPIO.IN)

        # Membaca nilai dari sensor
        currentState = GPIO.input(sensorPin)

        # Jika terjadi perubahan dari LOW ke HIGH (lubang terdeteksi)
        if currentState == GPIO.HIGH and previousState == GPIO.LOW:
            if orientation == 0 : # hitung sumbu y
                # Menambahkan satu pada hitungan
                counter_y += direction
                #print(counter_y)
                
                # Menghitung dan mencetak jarak tempuh
                jarakTempuh_y = (counter_y * kelilingRoda) / jumlahLubangPerPutaran
                
                #print("Jarak Tempuh:", jarakTempuh_y, "cm")
                if jarakTempuh_y >= 0:
                    posisi_y = math.floor(jarakTempuh_y/50)
                else:
                    posisi_y = math.ceil(jarakTempuh_y/50)
            
            elif orientation == 1 : # hitung sumbu x
                # Menambahkan satu pada hitungan
                counter_x += direction
                #print(counter_x)
                
                # Menghitung dan mencetak jarak tempuh
                jarakTempuh_x = (counter_x * kelilingRoda) / jumlahLubangPerPutaran
                
                #print("Jarak Tempuh:", jarakTempuh_x, "cm")
                if jarakTempuh_x >= 0:
                    posisi_x = math.floor(jarakTempuh_x/50)
                else:
                    posisi_x = math.ceil(jarakTempuh_x/50)
            
            count += direction
            jarakTempuh = (count * kelilingRoda) / jumlahLubangPerPutaran
            if jarakTempuh >= 50:
                count = 0

        # Memperbarui status sebelumnya
        previousState = currentState
        
        # Memberi sedikit jeda sebelum membaca sensor lagi
        time.sleep(0.001)
