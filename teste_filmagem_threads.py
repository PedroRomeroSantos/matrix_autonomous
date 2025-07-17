import cv2
import numpy as np
import threading
import RPi.GPIO as GPIO
import time

IN1, IN2 = 27, 17
IN3, IN4 = 22, 23
ENA = 19
ENB = 13

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for pin in [IN1, IN2, IN3, IN4, ENA, ENB]:
    GPIO.setup(pin, GPIO.OUT)

pwm_esq = GPIO.PWM(ENA, 100)
pwm_dir = GPIO.PWM(ENB, 100)
pwm_esq.start(0)
pwm_dir.start(0)

def mover_motor(v_esq, v_dir):
    v_esq = max(min(v_esq, 100), 0)
    v_dir = max(min(v_dir, 100), 0)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_esq.ChangeDutyCycle(v_esq)
    pwm_dir.ChangeDutyCycle(v_dir)

def parar():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_esq.ChangeDutyCycle(0)
    pwm_dir.ChangeDutyCycle(0)

pipeline = (
    "libcamerasrc ! "
    "video/x-raw, width=1280, height=720, framerate=30/1 ! "
    "videoconvert ! appsink"
)
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Não foi possível abrir a câmera")
    exit()

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = None  # Inicializa após primeiro frame
recording = True

def gravar_video():
    global out, recording
    while recording:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue
        if not isinstance(frame, np.ndarray):
            continue
        if frame.ndim != 3 or frame.shape[2] != 3:
            continue
        if out is None:
            height, width = frame.shape[:2]
            out = cv2.VideoWriter('video_saida_threads.mp4', fourcc, 20.0, (width, height))
            if not out.isOpened():
                print("Não foi possível abrir o VideoWriter.")
                recording = False
                break
        out.write(frame)
        # Opcional: exibir preview
        # frame_exibicao = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        # frame_exibicao = cv2.resize(frame_exibicao, (640, 480))
        # cv2.imshow("Camera", frame_exibicao)
        # cv2.waitKey(1)

print("Comandos: [F]rente, [E]squerda, [D]ireita, [S]top, [Q]uit")

video_thread = threading.Thread(target=gravar_video)
video_thread.start()

try:
    while True:
        comando = input("Comando: ").strip().upper()
        if comando == 'F':
            mover_motor(60, 60)
        elif comando == 'E':
            mover_motor(10, 60)
        elif comando == 'D':
            mover_motor(60, 10)
        elif comando == 'S':
            parar()
        elif comando == 'Q':
            break
        else:
            print("Comando inválido. Use F, E, D, S ou Q.")
            print("Parando carrinho")
            break
except KeyboardInterrupt:
    print("Interrompido pelo usuário")
finally:
    recording = False
    video_thread.join()
    parar()
    pwm_esq.stop()
    pwm_dir.stop()
    GPIO.cleanup()
    cap.release()
    if out is not None:
        out.release()
    cv2.destroyAllWindows()
