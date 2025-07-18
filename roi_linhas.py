import cv2
import numpy as np

cap = cv2.VideoCapture(r'matrix_autonomous\video_saida_threads.mp4')

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

def vídeoc_roi(cap):
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        altura, largura = frame.shape[:2]
        topo = int(altura * 0.15) #quando maior, menor a altura do triângulo
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

        #aplica os filtros e desenha as linhas sobre o ROI
        resultado = filtros(roi_frame)

        cv2.imshow("ROI com Linhas", resultado)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

vídeoc_roi(cap)
cap.release()
cv2.destroyAllWindows()