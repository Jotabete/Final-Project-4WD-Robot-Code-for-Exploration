import RPi.GPIO as GPIO
import time

def measure_distance(trig_pin, echo_pin):
    # Bersihkan trigger pin sebelum pengukuran
    GPIO.output(trig_pin, GPIO.LOW)
    time.sleep(0.1)

    # Kirim sinyal ultrasonik dengan menyalakan trigger pin
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)
    
    pulse_start = time.time()
    pulse_end = time.time()

    timeout = 0.1  # Atur timeout maksimum dalam detik
    start_time = time.time()

    while GPIO.input(echo_pin) == 0 and time.time() - start_time < timeout:
        pulse_start = time.time()

    start_time = time.time()
    while GPIO.input(echo_pin) == 1 and time.time() - start_time < timeout:
        pulse_end = time.time()

    # Setelah loop pembacaan
    if pulse_start == pulse_end:
        # Tidak ada respons dari sensor, tangani kesalahan di sini
        # Misalnya, keluar dari loop atau mencatat pesan kesalahan
        print("Sensor tidak memberikan respons yang diharapkan.")
        return None

    
    # Hitung durasi sinyal perjalanan
    pulse_duration = pulse_end - pulse_start

    # Hitung jarak berdasarkan durasi sinyal dan kecepatan suara
    speed_of_sound = 34300  # Kecepatan suara dalam cm/s
    distance = (pulse_duration * speed_of_sound) / 2
    
    # Lowpass filter
    global previous_distance
    if 'previous_distance' not in globals():
        previous_distance = distance
    
    ALPHA = 0.99
    filtered_distance = (1 - ALPHA) * previous_distance + ALPHA * distance
    previous_distance = filtered_distance
    
    if filtered_distance < 5: #minimum distance
        filtered_distance = 5
    
    return filtered_distance
