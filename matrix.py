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
    v_esq = max(min(v_esq, 100), -100)
    v_dir = max(min(v_dir, 100), -100)

    GPIO.output(IN1, GPIO.HIGH if v_esq >= 0 else GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW if v_esq >= 0 else GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH if v_dir >= 0 else GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW if v_dir >= 0 else GPIO.HIGH)

    pwm_esq.ChangeDutyCycle(abs(v_esq))
    pwm_dir.ChangeDutyCycle(abs(v_dir))

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
alternar_busca = True

# Velocidades reduzidas para comportamento mais suave
vel_avanco = 45
vel_giro = 30

def gravar_video():
    global out, recording

    if not hasattr(gravar_video, "em_avanco"):
        gravar_video.em_avanco = False

    estado_busca = 0
    tempo_inicio_giro = None
    tempos_rotacao = [1.2, 2.4, 1.2]  # esquerda, direita, volta esquerda

    while recording:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue
        if not isinstance(frame, np.ndarray):
            continue
        if frame.ndim != 3 or frame.shape[2] != 3:
            continue

        segmentado, centro, media_y = segmentar_pista(frame)

        if out is None:
            height, width = segmentado.shape[:2]
            out = cv2.VideoWriter('video_pista_segmentada.mp4', fourcc, 20.0, (width, height))

        out.write(segmentado)

        centro_frame = width // 2
        margem = 20
        altura_limite = int(height * 0.75)

        if centro and media_y <= altura_limite:
            estado_busca = 0
            tempo_inicio_giro = None
            erro = centro[0] - centro_frame

            if abs(erro) <= margem:
                if not gravar_video.em_avanco:
                    parar()
                    time.sleep(1)
                    gravar_video.em_avanco = True
                mover_motor(vel_avanco, vel_avanco)
            else:
                gravar_video.em_avanco = False
                parar()
                time.sleep(0.5)
                if erro < 0:
                    mover_motor(-vel_giro, vel_giro)  # esquerda
                else:
                    mover_motor(vel_giro, -vel_giro)  # direita

        else:
            gravar_video.em_avanco = False

            if tempo_inicio_giro is None:
                tempo_inicio_giro = time.time()

            tempo_giro = time.time() - tempo_inicio_giro

            if estado_busca == 0:
                if tempo_giro < tempos_rotacao[0]:
                    mover_motor(-vel_giro, vel_giro)  # esquerda
                else:
                    estado_busca = 1
                    tempo_inicio_giro = time.time()
                    parar()
                    time.sleep(0.5)

            elif estado_busca == 1:
                if tempo_giro < tempos_rotacao[1]:
                    mover_motor(vel_giro, -vel_giro)  # direita
                else:
                    estado_busca = 2
                    tempo_inicio_giro = time.time()
                    parar()
                    time.sleep(0.5)

            elif estado_busca == 2:
                if tempo_giro < tempos_rotacao[2]:
                    mover_motor(-vel_giro, vel_giro)  # esquerda
                else:
                    parar()
                    print("Não encontrou pista. Recuando...")
                    mover_motor(-40, -40)
                    time.sleep(1)
                    parar()
                    estado_busca = 0
                    tempo_inicio_giro = None

def segmentar_pista(frame):
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    h, s, v = cv2.split(hsv)
    v_blur = cv2.GaussianBlur(v, (9, 9), 0)
    _, mask = cv2.threshold(v_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centro = None
    media_y = 0
    if contours:
        maior = max(contours, key=cv2.contourArea)
        if cv2.contourArea(maior) > 500:
            cv2.drawContours(frame, [maior], -1, (0, 255, 0), -1)
            M = cv2.moments(maior)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centro = (cx, cy)
                media_y = cy
                cv2.circle(frame, centro, 5, (0, 0, 255), -1)

    return frame, centro, media_y

video_thread = threading.Thread(target=gravar_video)
video_thread.start()

try:
    while video_thread.is_alive():
        time.sleep(0.1)
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




