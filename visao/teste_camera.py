import cv2

pipeline = (
    "libcamerasrc ! "
    "video/x-raw, width=1280, height=720, framerate=30/1 ! "
    "videoconvert ! appsink"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Pipeline não abriu.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame não recebido.")
        break

    #Rotaciona -90 graus (anti-horário)
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    cv2.imshow('Camera CSI', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()




