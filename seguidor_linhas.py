#!/usr/bin/env python3
# line_follow_and_record.py
import cv2, numpy as np, time, sys, signal
import RPi.GPIO as GPIO

# =========================
# GPIO (BCM) — conforme seu setup
# =========================
IN1, IN2 = 27, 17      # motor esquerdo (direção)
IN3, IN4 = 22, 23      # motor direito  (direção)
ENA, ENB = 19, 13      # enables (PWM)

# =========================
# Vídeo (entrada e gravação)
# =========================
FRAME_W, FRAME_H = 640, 480
FPS = 20
SHOW_WINDOW = False          # True para visualizar no HDMI; False para headless
FOURCC = cv2.VideoWriter_fourcc(*'mp4v')  # use 'H264' ou 'avc1' se suportado
OUT_PATH = "debug_follow.mp4"

GST_PIPE = (
    "libcamerasrc ! "
    f"video/x-raw, width={FRAME_W}, height={FRAME_H}, framerate={FPS}/1 ! "
    "videoconvert ! appsink"
)

# =========================
# Visão
# =========================
ROI_BOTTOM = (0.60, 0.95)    # ROI baixa para controle
ROI_TOP    = (0.35, 0.55)    # ROI alta para antecipar bifurcação
BLUR_K = 5
MIN_AREA = 800
SPLIT_RATIO = 0.5
MIN_PIXELS_TOP = 1200
MARGEM_PIX = 6

# =========================
# Controle (P)
# =========================
Kp = 0.004
BASE_SPEED = 55      # duty 0..100
MAX_ADJ = 30
LOST_SPEED = 42
SCAN_ADJ = 22
SMOOTH_ALPHA = 0.45
TURN_BIAS = "LEFT"   # preferência em bifurcação: "LEFT" ou "RIGHT"

# =========================
# GPIO Setup
# =========================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for p in [IN1, IN2, IN3, IN4, ENA, ENB]:
    GPIO.setup(p, GPIO.OUT)

pwm_esq = GPIO.PWM(ENA, 1000)
pwm_dir = GPIO.PWM(ENB, 1000)
pwm_esq.start(0)
pwm_dir.start(0)

def mover_motor(v_esq, v_dir):
    v_esq = max(min(v_esq, 100), -100)
    v_dir = max(min(v_dir, 100), -100)

    GPIO.output(IN1, GPIO.HIGH if v_esq >= 0 else GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW  if v_esq >= 0 else GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH if v_dir >= 0 else GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW  if v_dir >= 0 else GPIO.HIGH)

    pwm_esq.ChangeDutyCycle(abs(float(v_esq)))
    pwm_dir.ChangeDutyCycle(abs(float(v_dir)))

def parar():
    pwm_esq.ChangeDutyCycle(0)
    pwm_dir.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.LOW)

def cleanup_and_exit(code=0):
    try:
        parar()
    finally:
        pwm_esq.stop()
        pwm_dir.stop()
        GPIO.cleanup()
        sys.exit(code)

def handle_sigint(sig, frame):
    print("\n[INFO] Encerrando...")
    cleanup_and_exit(0)

signal.signal(signal.SIGINT, handle_sigint)

# =========================
# Visão — utilitários
# =========================
def binariza(frame_bgr):
    blur = cv2.GaussianBlur(frame_bgr, (BLUR_K|1, BLUR_K|1), 0)
    hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    v    = hsv[:, :, 2]
    v_blur = cv2.GaussianBlur(v, (9, 9), 0)
    _, mask = cv2.threshold(v_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return mask

def get_roi(img, frac):
    h, w = img.shape[:2]
    y0 = int(frac[0]*h); y1 = int(frac[1]*h)
    return img[y0:y1, :], y0, y1, w

def centroide_maior_contorno(mask_roi):
    contours, _ = cv2.findContours(mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None
    maior = max(contours, key=cv2.contourArea)
    if cv2.contourArea(maior) < MIN_AREA:
        return None, None
    M = cv2.moments(maior)
    if M["m00"] == 0:
        return None, None
    cx = int(M["m10"]/M["m00"]); cy = int(M["m01"]/M["m00"])
    return (cx, cy), maior

def split_energy(mask_roi):
    h, w = mask_roi.shape[:2]
    cut = int(SPLIT_RATIO * w)
    left  = int(np.sum(mask_roi[:, :cut] == 255))
    right = int(np.sum(mask_roi[:, cut:] == 255))
    return left, right

def escolhe_bifurcacao(le, re):
    if le > MIN_PIXELS_TOP and re > MIN_PIXELS_TOP:
        return TURN_BIAS
    return "LEFT" if le >= re else "RIGHT"

# =========================
# Main
# =========================
def main():
    cap = cv2.VideoCapture(GST_PIPE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("[ERRO] Não foi possível abrir a câmera via GStreamer.")
        cleanup_and_exit(1)

    writer = cv2.VideoWriter(OUT_PATH, FOURCC, FPS, (FRAME_W, FRAME_H))
    if not writer.isOpened():
        print("[ERRO] VideoWriter não abriu. Verifique codec/permissões.")
        cap.release()
        cleanup_and_exit(1)

    last_err = 0.0
    last_turn = TURN_BIAS
    lost_since = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            mask = binariza(frame)
            overlay = frame.copy()

            # ROI baixa (controle)
            roi_b, yb0, yb1, W = get_roi(mask, ROI_BOTTOM)
            res_b = centroide_maior_contorno(roi_b)
            cv2.rectangle(overlay, (0, yb0), (W, yb1), (0,255,0), 2)

            # ROI alta (bifurcação)
            roi_t, yt0, yt1, _ = get_roi(mask, ROI_TOP)
            le, re = split_energy(roi_t)
            branch_hint = escolhe_bifurcacao(le, re)
            cv2.rectangle(overlay, (0, yt0), (W, yt1), (255,0,0), 2)
            # linha central de referência (na ROI baixa)
            cv2.line(overlay, (W//2, yb0), (W//2, yb1), (0,255,255), 1)

            if res_b is not None:
                (cx, cy), cnt = res_b
                cnt_disp = cnt + np.array([[[0, yb0]]])
                cv2.drawContours(overlay, [cnt_disp], -1, (0,255,0), 2)
                cv2.circle(overlay, (cx, cy+yb0), 5, (0,0,255), -1)

                # erro lateral (px)
                err_raw = cx - (W/2)
                if abs(err_raw) <= MARGEM_PIX:
                    err_raw = 0.0
                err = SMOOTH_ALPHA*err_raw + (1-SMOOTH_ALPHA)*last_err
                last_err = err

                # controle P + viés de bifurcação forte
                adj = float(np.clip(Kp * err, -MAX_ADJ, MAX_ADJ))
                bias = 5 if (le > MIN_PIXELS_TOP and re > MIN_PIXELS_TOP and branch_hint == "LEFT") else \
                       -5 if (le > MIN_PIXELS_TOP and re > MIN_PIXELS_TOP and branch_hint == "RIGHT") else 0
                if bias == 0:
                    last_turn = branch_hint

                L = BASE_SPEED - adj + bias
                R = BASE_SPEED + adj - bias
                mover_motor(L, R)
                lost_since = 0.0
            else:
                # linha perdida → varredura no último sentido
                lost_since += 1.0/FPS
                if last_turn == "LEFT":
                    mover_motor(LOST_SPEED + SCAN_ADJ, LOST_SPEED - SCAN_ADJ)
                else:
                    mover_motor(LOST_SPEED - SCAN_ADJ, LOST_SPEED + SCAN_ADJ)

            # grava overlay com todos os desenhos para debug posterior
            writer.write(overlay)

            if SHOW_WINDOW:
                cv2.imshow("debug_follow", overlay)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

    except KeyboardInterrupt:
        pass
    finally:
        parar()
        writer.release()
        cap.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        cleanup_and_exit(0)

if __name__ == "__main__":
    main()
