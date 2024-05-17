# main.py
# Main Control of Robot
# TA 08

import RPi.GPIO as GPIO
import time
import smbus
import threading
import math
import logging
import sys

from sensor_ultrasonik import measure_distance
from motor_driver import forward, backward, left, right, stop
from BSA_Robot import nextmove, searchzero

import sensor_imu
import sensor_jarak_tempuh

# Set mode pin GPIO
GPIO.setmode(GPIO.BCM)

############################################## IMU ####################################################
# Inisialisasi alamat I2C dan register MPU6050
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B  # Register untuk konfigurasi gyro
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

# Inisialisasi untuk akeselerometer
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

# Konfigurasi bus I2C
bus = smbus.SMBus(1)

# Variable global untuk menyimpan sudut rotasi z
rotation_angle_z = 0.0

# Variable global untuk menyimpan nilai offset yang dihasilkan dari kalibrasi
gyro_offset_z = 0.0

# Variabel global untuk menyimpan offset kalibrasi akselerometer
accel_offset_x = 0
accel_offset_y = 0
accel_offset_z = 0

############################################## ULTRASONIC ############################################
# Tentukan pin GPIO untuk masing-masing sensor ultrasonic
FRONT_TRIG_PIN = 10
FRONT_ECHO_PIN = 9
RIGHT_TRIG_PIN = 23 #27
RIGHT_ECHO_PIN = 24 #22
LEFT_TRIG_PIN = 27
LEFT_ECHO_PIN = 22
BACK_TRIG_PIN = 4
BACK_ECHO_PIN = 18

# Setup pin GPIO untuk setiap sensor
GPIO.setup(FRONT_TRIG_PIN, GPIO.OUT)
GPIO.setup(FRONT_ECHO_PIN, GPIO.IN)
GPIO.setup(BACK_TRIG_PIN, GPIO.OUT)
GPIO.setup(BACK_ECHO_PIN, GPIO.IN)
GPIO.setup(RIGHT_TRIG_PIN, GPIO.OUT)
GPIO.setup(RIGHT_ECHO_PIN, GPIO.IN)
GPIO.setup(LEFT_TRIG_PIN, GPIO.OUT)
GPIO.setup(LEFT_ECHO_PIN, GPIO.IN)

############################################## MOTOR DRIVER ##########################################
# Define GPIO pins for motor driver
ENA = 13
ENB = 19
IN1 = 25
IN2 = 8
IN3 = 7
IN4 = 11

# Setup GPIO pins for motor driver
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Set initial motor speed (PWM)
pwm_a = GPIO.PWM(ENA, 1000000)
pwm_b = GPIO.PWM(ENB, 1000000)
pwm_a.start(100)
pwm_b.start(100)

#Global Variable
front_distance = 0.0
back_distance = 0.0
right_distance = 0.0
left_distance  = 0.0

############################################## ODOMETRI #############################################
# Deklarasi pin
sensorPin = 5  # Pin input sensor optokopler
# previousState = GPIO.LOW  # Status sebelumnya dari sensor
# counter = 0  # Variabel untuk menghitung lubang

# Konstanta untuk perhitungan jarak tempuh
# kelilingRoda = 21.2  # Keliling roda dalam cm
# jumlahLubangPerPutaran = 20  # Jumlah lubang per putaran encoder
# direction = 1 # 1 : maju, -1 : mundur

############################################## Main Control ##########################################
# Variabel Event untuk memberi sinyal kepada thread untuk berhenti atau melanjutkan
pause_event = threading.Event()
pause_event_tempuh = threading.Event()

myarena = [[0 for i in range(15)] for i in range(9)]
# keterangan array di myarena : 
# - angka 1 = untuk yang sudah dilewat
# - angka 6 = untuk obstacle

xpos = 0 #inisialisasi posisi x
ypos = 1 #inisialisasi posisi y
xmax = 15
ymax = 9
myarena[ypos][xpos] = 1
arah = ["atas","kanan","bawah","kiri"]
pair = [2, 3, 0, 1] # ket : 0 = Atas , 1 = Kanan, 2 = Bawah, 3 = kiri ; relatif terhadap posisi awal robot; pair adalah varible yang berlawanan dengan keadaannya
current_move = 1

def move_forward():
    if sensor_imu.pitch >= 20: # cek kemiringan pitch robot (jalan menanjak)
        stop(IN1, IN2, IN3, IN4)
        time.sleep(1)
        x_naik = xpos
        y_naik = ypos
        move_backward()
        # lalu set arah tersebut menjadi obstacle
        myarena[y_naik][x_naik] = 6
    else:
        forward(IN1, IN2, IN3, IN4)
        time.sleep(0.8)
#         while sensor_jarak_tempuh.jarakTempuh < 50:
#             forward(IN1, IN2, IN3, IN4)
#             time.sleep(0.0001)
    
def move_right():
    pause_event_tempuh.set()
    while sensor_imu.rotation_angle_z > -65: #harusnya 90
        right(IN1, IN2, IN3, IN4)
        time.sleep(0.1)
    stop(IN1, IN2, IN3, IN4)
    time.sleep(1)
    sensor_imu.rotation_angle_z = 0
    pause_event_tempuh.clear()
    
def move_left():
    pause_event_tempuh.set()
    while sensor_imu.rotation_angle_z < 65: #harusnya 90
        left(IN1, IN2, IN3, IN4)
        time.sleep(0.1)
    stop(IN1, IN2, IN3, IN4)
    time.sleep(1)
    sensor_imu.rotation_angle_z = 0
    pause_event_tempuh.clear()

def move_backward(): 
    backward(IN1, IN2, IN3, IN4)
    time.sleep(1)
#     while sensor_jarak_tempuh.jarakTempuh > -50: #50 cm
#         backward(IN1, IN2, IN3, IN4)
#         time.sleep(0.1)
        
def turn_back():
    pause_event_tempuh.set()
    while sensor_imu.rotation_angle_z < 180: #perlu disesuaikan lagi dengan keadaan riil (apakah tepat 180 atau tidak, seperti saat belok)
        left(IN1, IN2, IN3, IN4)
        time.sleep(0.1)
    stop(IN1, IN2, IN3, IN4)
    time.sleep(1)
    sensor_imu.rotation_angle_z = 0

def read_front(FRONT_TRIG_PIN, FRONT_ECHO_PIN):
    global front_distance, myarena, ypos, xpos, xmax, ymax
    front_distance  = measure_distance(FRONT_TRIG_PIN, FRONT_ECHO_PIN)
    #print("depan :", front_distance)
    if front_distance <= 12:
        if current_move == 0 and ypos-1 != -1:
            myarena[ypos-1][xpos] = 6  # Tandai rintangan 
        elif current_move == 1 and xpos+1 != xmax:
            myarena[ypos][xpos+1] = 6  # Tandai rintangan 
        elif current_move == 2 and ypos+1 != ymax:
            myarena[ypos+1][xpos] = 6  # Tandai rintangan 
        elif current_move == 3 and xpos-1 != -1:
            myarena[ypos][xpos-1] = 6  # Tandai rintangan 

def read_right(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN):
    global right_distance, myarena, ypos, xpos, xmax, ymax
    right_distance  = measure_distance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN)
    #print("kanan:", right_distance)
    if right_distance <= 16:
        if current_move == 0 and xpos+1 != xmax:
            myarena[ypos][xpos+1] = 6  # Tandai rintangan
        elif current_move == 1 and ypos+1 != ymax:
            myarena[ypos+1][xpos] = 6  # Tandai rintangan
        elif current_move == 2 and xpos-1 != -1:
            myarena[ypos][xpos-1] = 6  # Tandai rintangan
        elif current_move == 3 and ypos-1 != -1:
            myarena[ypos-1][xpos] = 6  # Tandai rintangan
        
def read_left(LEFT_TRIG_PIN, LEFT_ECHO_PIN):    
    global left_distance, myarena, ypos, xpos, xmax, ymax
    left_distance   = measure_distance(LEFT_TRIG_PIN, LEFT_ECHO_PIN)
    #print("kiri :", left_distance)
    if left_distance <= 16:
        if current_move == 0 and xpos-1 != -1:
            myarena[ypos][xpos-1] = 6  # Tandai rintangan
        elif current_move == 1 and ypos-1 != -1:
            myarena[ypos-1][xpos] = 6  # Tandai rintangan
        elif current_move == 2 and xpos+1 != xmax:
            myarena[ypos][xpos+1] = 6  # Tandai rintangan
        elif current_move == 3 and ypos+1 != ymax:
            myarena[ypos+1][xpos] = 6  # Tandai rintangan
    
def read_back(BACK_TRIG_PIN, BACK_ECHO_PIN):    
    global back_distance, myarena, ypos, xpos, xmax, ymax
    back_distance   = measure_distance(BACK_TRIG_PIN, BACK_ECHO_PIN)
    #print("blkg :",back_distance)
    if back_distance <=12:
        if current_move == 0 and ypos+1 != ymax:
            myarena[ypos+1][xpos] = 6  # Tandai rintangan
        elif current_move == 1 and xpos-1 != -1:
            myarena[ypos][xpos-1] = 6  # Tandai rintangan
        elif current_move == 2 and ypos-1 != -1:
            myarena[ypos-1][xpos] = 6  # Tandai rintangan
        elif current_move == 3 and xpos+1 != xmax:
            myarena[ypos][xpos+1] = 6  # Tandai rintangan
    
def check_around():
    global myarena, xpos, ypos, current_move
    read_front(FRONT_TRIG_PIN, FRONT_ECHO_PIN)
    read_back(BACK_TRIG_PIN, BACK_ECHO_PIN)
    read_right(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN)
    read_left(LEFT_TRIG_PIN, LEFT_ECHO_PIN)
        
def pergerakan_robot():
    global current_move, next_move, pair, xpos, ypos, arah
    #fungsi untuk pergerakan robot
    #fungsi untuk cek keadaan sekitar (menandai obstale & mencari jalan yang bisa dilalui)
    if current_move == next_move:  
        move_forward()
    else:
        if current_move != pair[next_move]:
            if current_move == 3 and next_move == 0:
                move_right()
                move_forward()
            elif next_move > current_move:
                move_right()
                move_forward()
            else:
                move_left()
                move_forward()
        else:
            move_backward()
#             turn_back()
#             move_forward()
    
    xpos = sensor_jarak_tempuh.posisi_x
    ypos = sensor_jarak_tempuh.posisi_y
    print(f"arah = {arah[current_move]} ; (x,y) = ({xpos},{ypos})")
    
    for i in myarena:
        print(i)
    
    current_move = next_move
    
    stop(IN1, IN2, IN3, IN4)
    time.sleep(1)

def cleanup():
    # Matikan GPIO
    GPIO.cleanup()

if __name__ == "__main__":
    try:
        # Inisialisasi MPU6050
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
        # Konfigurasi gyro ; ket : 0b00 untuk 250dps
        sensor_imu.set_gyro_scale(0b00)  
        # 0 untuk skala +-2g
        sensor_imu.set_accel_fsr(0)

        print("Kalibrasi...")
        # Kalibrasi gyro
        sensor_imu.calibrate_gyro()
        print("Kalibrasi selesai")

        # Buat thread untuk mengukur sudut rotasi z secara kontinu
        dt = 0.1  # Interval waktu untuk pengukuran (misalnya, 0.1 detik)
        gyro_thread = threading.Thread(target=sensor_imu.integrate_gyro_data, args=(pause_event, dt,))
        gyro_thread.daemon = True
        gyro_thread.start()

        # Mulai thread untuk mencetak sudut pitch dan roll secara kontinu
        print_thread = threading.Thread(target=sensor_imu.get_pitch_and_roll, args=(pause_event, ACCEL_XOUT_H,ACCEL_YOUT_H,ACCEL_ZOUT_H,))
        print_thread.daemon = True
        print_thread.start()
        
        # Membuat thread baru untuk pembacaan sensor jarak tempuh secara kontinu
        sensor_thread = threading.Thread(target=sensor_jarak_tempuh.tempuh_to_posgrid, args=(pause_event_tempuh, sensorPin,))
        sensor_thread.daemon = True 
        sensor_thread.start()
        
        while True:
            #fungsi menentukan gerak selanjutnya
            check_around()
            time.sleep(2)
            next_move = nextmove(current_move, myarena, xpos, ypos)
            # Lokalisasi
            if next_move == 0: # atas y--
                #sebelum gerak, ubah parameter fungsi di sensor_jarak_tempuh
                sensor_jarak_tempuh.orientation = 0
                sensor_jarak_tempuh.direction = -1
                myarena[ypos][xpos] = 1
                pergerakan_robot()
            elif next_move == 1: # kanan x++
                sensor_jarak_tempuh.orientation = 1
                sensor_jarak_tempuh.direction = 1
                myarena[ypos][xpos] = 1
                pergerakan_robot()
            elif next_move == 2: # belakang y++
                sensor_jarak_tempuh.orientation = 0
                sensor_jarak_tempuh.direction = 1
                myarena[ypos][xpos] = 1
                pergerakan_robot()
            elif next_move == 3: # kiri x--
                sensor_jarak_tempuh.orientation = 1
                sensor_jarak_tempuh.direction = -1
                myarena[ypos][xpos] = 1
                pergerakan_robot()
            else:
                rlmove = searchzero(myarena, xpos, ypos)
                if (rlmove == "reject"):
                    break
                else:
                    for i in rlmove:
                        if (i == 0): #atas y--              
                            sensor_jarak_tempuh.orientation = 0
                            sensor_jarak_tempuh.direction = -1
                            myarena[ypos][xpos] = 1
                        elif (i == 1): #kanan x++
                            sensor_jarak_tempuh.orientation = 1
                            sensor_jarak_tempuh.direction = 1
                            myarena[ypos][xpos] = 1
                        elif (i == 2):# belakang y++
                            sensor_jarak_tempuh.orientation = 0
                            sensor_jarak_tempuh.direction = 1
                            myarena[ypos][xpos] = 1
                        else:# kiri x--
                            sensor_jarak_tempuh.orientation = 1
                            sensor_jarak_tempuh.direction = -1
                            myarena[ypos][xpos] = 1
                        next_move = i
                        pergerakan_robot()
                        
    except KeyboardInterrupt:
        # Tangkap KeyboardInterrupt (Ctrl+C) untuk membersihkan GPIO
        pause_event_tempuh.set()
        pause_event.set()
        cleanup()

