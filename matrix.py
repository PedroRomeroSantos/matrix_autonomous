import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

IN1, IN2 = 17, 27
IN3, IN4 = 22, 23
ENA = 19
ENB = 13

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Configura GPIO
for pin in [IN1, IN2, IN3, IN4, ENA, ENB]:
    GPIO.setup(pin, GPIO.OUT)

pwm_esq = GPIO.PWM(ENA, 100)
pwm_dir = GPIO.PWM(ENB, 100)

pwm_esq.start(0)
pwm_dir.start(0)

#Função de controle dos motores
def mover_motor(v_esq, v_dir):
    # Limita entre 0 e 100
    v_esq = max(min(v_esq, 100), 0)
    v_dir = max(min(v_dir, 100), 0)

    # Sempre pra frente (ajusta se quiser incluir marcha ré)
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

# Pipeline da câmera
pipeline = (
    "libcamerasrc ! "
    "video/x-raw, width=1280, height=720, framerate=30/1 ! "
    "videoconvert ! appsink"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Não foi possível abrir a câmera")
    exit()

# Controle proporcional
Kp = 0.15  # → AJUSTE FINO AQUI

# Loop principal
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Sem imagem")
            break

        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        frame_resized = cv2.resize(frame, (640, 480))

        roi = frame_resized[220:480, 0:640]  # Região de interesse (parte de baixo)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 120, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)

                #Controle proporcional
                centro_frame = 320  # → metade de 640
                erro = cx - centro_frame
                correcao = Kp * erro

                v_base = 50  # → velocidade base

                v_esq = v_base + correcao
                v_dir = v_base - correcao

                mover_motor(v_esq, v_dir)

                print(f'Erro: {erro} | Esq: {v_esq:.1f} | Dir: {v_dir:.1f}')

            else:
                print('Linha não detectada')
                mover_motor(30, 30)
        else:
            print('Linha não detectada')
            mover_motor(30, 30)

        # Debug visual
        cv2.imshow('Frame', frame_resized) # regiao de interesse
        #cv2.imshow('Thresh', thresh)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print('Parando')

finally:
    parar()
    pwm_esq.stop()
    pwm_dir.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()


