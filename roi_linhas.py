import cv2
import numpy as np

import cv2
import numpy as np

def segmentar_pista(frame):
    altura, largura = frame.shape[:2]
    inicio = int(altura * 0.4) # define a partir de onde começa a análise
    area_analisada = frame[inicio:, :]

    # Converte para HSV
    hsv = cv2.cvtColor(area_analisada, cv2.COLOR_BGR2HSV)

    # Detecta pista escura (chão preto mais sensível)
    lower = np.array([0, 0, 0])
    upper = np.array([180, 255, 100])
    mask_pista = cv2.inRange(hsv, lower, upper)

    # Cria máscara trapezoidal da região analisada (usando toda a base da imagem)
    altura_roi = mask_pista.shape[0]
    largura_roi = mask_pista.shape[1]
    topo_y = int(altura_roi * 0.2)  # sobe mais alto
    base_y = altura_roi

    trapezio = np.array([[
        (0, base_y),
        (largura_roi, base_y),
        (int(largura_roi * 0.65), topo_y),
        (int(largura_roi * 0.35), topo_y)
    ]], dtype=np.int32)

    mask_roi = np.zeros_like(mask_pista)
    cv2.fillPoly(mask_roi, trapezio, 255)

    # Aplica ROI à máscara da pista
    mask_final = cv2.bitwise_and(mask_pista, mask_roi)

    # Cria imagem verde para representar a área onde o carrinho pode andar
    verde = np.zeros_like(area_analisada)
    verde[:, :] = (0, 255, 0)
    area_verde = cv2.bitwise_and(verde, verde, mask=mask_final)

    sobreposicao = cv2.addWeighted(area_analisada, 1.0, area_verde, 0.4, 0)
    frame[inicio:, :] = sobreposicao

    # Desenha a borda do trapézio de ROI
    trapezio_absoluto = trapezio + np.array([0, inicio])  # ajusta para o frame completo
    cv2.polylines(frame, [trapezio_absoluto], isClosed=True, color=(0, 255, 0), thickness=2)

    # Calcula centro da área de pista (ponto vermelho)
    M = cv2.moments(mask_final)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"]) + inicio
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        # Calcula o erro entre centro da pista e centro da imagem
        cx_img = largura // 2
        erro = cx - cx_img

        # Opcional: desenha linha entre centro da imagem e centro da pista
        cv2.line(frame, (cx_img, cy), (cx, cy), (255, 255, 0), 2)

        # Opcional: imprime erro
        texto = f"Erro de alinhamento: {erro}"
        cv2.putText(frame, texto, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    return frame

def video_segmentacao_real_time():
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
        segmentado = segmentar_pista(frame)
        cv2.imshow("Segmentacao da Pista - Tempo Real", segmentado)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

def video_segmentacao_file(video_path):
    cap = cv2.VideoCapture(video_path)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret or frame is None:
            break
        segmentado = segmentar_pista(frame)
        cv2.imshow("Segmentacao da Pista - Video", segmentado)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

# Para analisar vídeo pronto, chame:
video_segmentacao_file(r"C:\Users\galag\OneDrive\DV\matrix_autonomous\videoplayback.mp4")

# Para ver em tempo real, chame:
# video_segmentacao_real_time()
