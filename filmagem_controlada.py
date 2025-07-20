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
    "video/x-raw, width=640, height=480, framerate=30/1 ! "
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

        segmentado, centro = segmentar_pista(frame)

        if out is None:
            height, width = segmentado.shape[:2]
            out = cv2.VideoWriter('video_roi_autonomo.mp4', fourcc, 20.0, (width, height))

        out.write(segmentado)

def segmentar_pista(frame):
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    h, s, v = cv2.split(hsv)
    v_blur = cv2.GaussianBlur(v, (9, 9), 0)
    _, mask = cv2.threshold(v_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # contornos da região clara (maior área = chão/pista)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centro = None
    if contours:
        maior = max(contours, key=cv2.contourArea)
        if cv2.contourArea(maior) > 500:
            cv2.drawContours(frame, [maior], -1, (0, 255, 0), -1)
            M = cv2.moments(maior)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centro = (cx, cy)
                cv2.circle(frame, centro, 5, (0, 0, 255), -1)

    return frame, centro

video_thread = threading.Thread(target=gravar_video)
video_thread.start()

print("Comandos: [F]rente, [E]squerda, [D]ireita, [S]top, [Q]uit")
try:
    while True:
        comando = input("Comando: ").strip().upper()
        if comando == 'F':
            mover_motor(100, 100)
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
except KeyboardInterrupt:
    print("Interrupção")
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


