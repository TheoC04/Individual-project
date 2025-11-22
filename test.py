import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open camera")
    exit()
else:
    print("Camera opened successfully")

cap.release()
cv2.destroyAllWindows()