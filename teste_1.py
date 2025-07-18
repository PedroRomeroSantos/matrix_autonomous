import cv2
import numpy as np

frame = cv2.imread('matrix_autonomous\WhatsApp Image 2025-07-18 at 01.16.32.jpeg')
altura, largura = frame.shape[:2]
# Define os pontos do trapézio 
topo = int(altura * 0.6)
base = altura
margem = int(largura * 0.2)
polygon = np.array([[
    (margem, base),                # canto inferior esquerdo
    (largura - margem, base),      # canto inferior direito
    (int(largura * 0.7), topo),   # topo direito
    (int(largura * 0.3), topo)    # topo esquerdo
]], dtype=np.int32)
mask = np.zeros_like(frame)
cv2.fillPoly(mask, polygon, (255, 255, 255))
masked_frame = cv2.bitwise_and(frame, mask)
print(masked_frame.shape)