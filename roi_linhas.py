import cv2
import numpy as np

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
    return frame

def video_roi_real_time():
    pipeline = (
        "libcamerasrc ! "
        "video/x-raw, width=1280, height=720, framerate=30/1 ! "
        "videoconvert ! appsink"
    )
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret or frame is None:
            continue
        altura, largura = frame.shape[:2]
        topo = int(altura * 0.15)
        base = altura
        polygon = np.array([
            [
                (0, base),
                (largura, base),
                (largura // 2, topo)
            ]
        ], dtype=np.int32)
        mask = np.zeros_like(frame)
        cv2.fillPoly(mask, polygon, (255, 255, 255))
        roi_frame = cv2.bitwise_and(frame, mask)
        resultado = filtros(roi_frame)
        cv2.imshow("ROI com Linhas - Tempo Real", resultado)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

def video_roi_file(video_path):
    cap = cv2.VideoCapture(video_path)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret or frame is None:
            break
        altura, largura = frame.shape[:2]
        topo = int(altura * 0.15)
        base = altura
        polygon = np.array([
            [
                (0, base),
                (largura, base),
                (largura // 2, topo)
            ]
        ], dtype=np.int32)
        mask = np.zeros_like(frame)
        cv2.fillPoly(mask, polygon, (255, 255, 255))
        roi_frame = cv2.bitwise_and(frame, mask)
        resultado = filtros(roi_frame)
        cv2.imshow("ROI com Linhas - Vídeo", resultado)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

# Para analisar vídeo pronto, chame:
video_roi_file(r'matrix_autonomous\video_saida_threads.mp4')

# Para ver em tempo real, chame:
# video_roi_real_time()