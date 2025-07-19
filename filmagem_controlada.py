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
            out = cv2.VideoWriter('video_roi.mp4', fourcc, 20.0, (width, height))
            if not out.isOpened():
                print("Não foi possível abrir o VideoWriter.")
                recording = False
                break
        filtros(frame)

def filtros(frame):
    imagem_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    imagem_blur = cv2.GaussianBlur(imagem_gray, (5, 5), 0)
    imagem_canny = cv2.Canny(imagem_blur, 50, 150)
    edges = cv2.Canny(imagem_gray, 50, 150)
    linhas = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=50)
    if linhas is not None:
        for linha in linhas:
            x1, y1, x2, y2 = linha[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    #cv2.imshow("ROI com Linhas", frame)
    out.write(frame)

def roi(frame):
    altura, largura = frame.shape[:2]
    topo = int(altura * 0.15)  #quanto menor maior a altura do triângulo
    base = altura
    polygon = np.array([[
        (0, base),
        (largura, base),
        (largura // 2, topo)
    ]], dtype=np.int32)
    mask = np.zeros_like(frame)
    cv2.fillPoly(mask, polygon, (255, 255, 255))
    masked_frame = cv2.bitwise_and(frame, mask)
    out.write(masked_frame)

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
