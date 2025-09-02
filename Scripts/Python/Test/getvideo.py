import cv2

cap = cv2.VideoCapture("http://127.0.0.1:5000/video_feed")

while True:
    ret, frame = cap.read()
    if not ret:
        continue
    cv2.imshow("CamVirtual", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

