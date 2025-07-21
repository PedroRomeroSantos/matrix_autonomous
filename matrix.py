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

def girar_esquerda():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_esq.ChangeDutyCycle(40)
    pwm_dir.ChangeDutyCycle(40)

def girar_direita():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_esq.ChangeDutyCycle(40)
    pwm_dir.ChangeDutyCycle(40)

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

cx_img = 640 // 2  # centro da imagem assumindo 1280x720
Kp = 0.2  # constante de controle proporcional


def processamento(frame):
    altura, largura = frame.shape[:2]
    inicio = int(altura * 0.5)  # agora ROI começa na metade da imagem
    roi = frame[inicio:, :]

    # Converte para escala de cinza e aplica suavização
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Detecta bordas para estimar regiões com variação (ruído, bordas, objetos)
    edges = cv2.Canny(blur, 30, 60)

    # A região uniforme é onde não há bordas (bitwise NOT)
    mask_uniforme = cv2.bitwise_not(edges)

    # Prepara sobreposição verde onde a pista é uniforme
    verde = np.zeros_like(roi)
    verde[:] = (0, 255, 0)
    area_verde = cv2.bitwise_and(verde, verde, mask=mask_uniforme)
    sobreposicao = cv2.addWeighted(roi, 1.0, area_verde, 0.4, 0)
    frame[inicio:, :] = sobreposicao

    # Desenha o retângulo da ROI
    cv2.rectangle(frame, (0, inicio), (largura, altura), (0, 255, 0), 2)

    # Cálculo do centro da máscara de uniformidade
    M = cv2.moments(mask_uniforme)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"]) + inicio
        erro = cx - (largura // 2)

        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.line(frame, (largura // 2, cy), (cx, cy), (255, 255, 0), 2)
        cv2.putText(frame, f"Erro de alinhamento: {erro}", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return frame, erro
    else:
        return frame, None

def gravar_video():
    global out, recording
    while recording:
        ret, frame = cap.read()
        if not ret:
            continue

        if out is None:
            height, width = frame.shape[:2]
            out = cv2.VideoWriter('video_roi_autonomo.mp4', fourcc, 20.0, (width, height))

        frame_segmentado, erro = processamento(frame.copy())
        out.write(frame_segmentado)

        if erro is not None:
            if abs(erro) < 20:
                mover_motor(40, 40)
            elif erro > 0:
                girar_direita()
            else:
                girar_esquerda()
        else:
            parar()

        time.sleep(0.05)

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
