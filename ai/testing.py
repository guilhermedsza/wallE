import cv2
cap = cv2.VideoCapture(0)                       # default backend
cap.set(cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*'MJPG'))        # tell it we want MJPG
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)         # values the bridge likes
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

ret, frame = cap.read()
print("ret:", ret, "| shape:", None if frame is None else frame.shape)
cap.release()