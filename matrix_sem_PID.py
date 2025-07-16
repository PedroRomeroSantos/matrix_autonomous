import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# 🛠️ Configuração dos pinos dos motores
IN1, IN2 = 17, 27   # Motor esquerdo
IN3, IN4 = 22, 23   # Motor direito
ENA = 19            # PWM esquerdo
ENB = 13            # PWM direito

# 🚗 Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

motor_pins = [IN1, IN2, IN3, IN4, ENA, ENB]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

pwm_esq = GPIO.PWM(ENA, 100)
pwm_dir = GPIO.PWM(ENB, 100)

pwm_esq.start(0)
pwm_dir.start(0)

# 🚗 Funções dos motores
def frente(vel=60):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_esq.ChangeDutyCycle(vel)
    pwm_dir.ChangeDutyCycle(vel)

def tras(vel=60):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_esq.ChangeDutyCycle(vel)
    pwm_dir.ChangeDutyCycle(vel)

def parar():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_esq.ChangeDutyCycle(0)
    pwm_dir.ChangeDutyCycle(0)

def esquerda(vel=60):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_esq.ChangeDutyCycle(vel)
    pwm_dir.ChangeDutyCycle(vel)

def direita(vel=60):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_esq.ChangeDutyCycle(vel)
    pwm_dir.ChangeDutyCycle(vel)

# 📸 Pipeline da câmera
pipeline = (
    "libcamerasrc ! "
    "video/x-raw, width=1280, height=720, framerate=30/1 ! "
    "videoconvert ! appsink"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Não foi possível abrir a câmera")
    exit()

# 🧠 Loop principal
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Sem imagem")
            break

        #Pré-processamento
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)  # Rotaciona se necessário
        frame_resized = cv2.resize(frame, (640, 480))

        # 🎯 Região de interesse → só a parte de baixo
        roi = frame_resized[220:480, 0:640]

        # 🔳 Convertendo para escala de cinza e binarizando
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 120, 255, cv2.THRESH_BINARY_INV)  # Use THRESH_BINARY se a linha for preta, INV se for branca

        # 🧠 Calcula centro da linha
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)

                # 🏁 Decisão → base na linha da direita
                if cx >= 400:
                    print('→ Direita')
                    direita(40)
                elif cx < 240:
                    print('← Esquerda')
                    esquerda(40)
                else:
                    print('↑ Frente')
                    frente(50)
            else:
                print('Linha não detectada')
                frente(30)
        else:
            print('Linha não detectada')
            frente(30)

        #Mostrar para debug
        cv2.imshow('Frame', frame_resized)
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
