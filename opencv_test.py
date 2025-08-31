import cv2
import numpy as np

# ---- Object Detection: MobileNet SSD ----
net = cv2.dnn.readNetFromCaffe("deploy.prototxt",
                               "mobilenet_iter_73000.caffemodel")


# Classes we care about
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow",
           "diningtable", "dog", "horse", "motorbike", "person",
           "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

# Initialize Pi camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # ---- Object Detection ----
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (320,240), 127.5)
    net.setInput(blob)
    detections = net.forward()

    for i in range(detections.shape[2]):
        confidence = detections[0,0,i,2]
        idx = int(detections[0,0,i,1])
        if confidence > 0.5 and CLASSES[idx] != "background":
            box = detections[0,0,i,3:7] * np.array([frame.shape[1], frame.shape[0],
                                                   frame.shape[1], frame.shape[0]])
            (startX, startY, endX, endY) = box.astype("int")
            cv2.rectangle(frame, (startX, startY), (endX, endY), (0,255,0), 2)
            cv2.putText(frame, f"{CLASSES[idx]}:{confidence:.2f}",
                        (startX, startY-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    # ---- Edge Detection (wall-floor boundary) ----
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(blur, 50, 150)

    # Hough lines
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)
    if lines is not None:
        for line in lines:
            x1,y1,x2,y2 = line[0]
            cv2.line(frame, (x1,y1),(x2,y2),(0,0,255),2)

    # ---- Display combined frame ----
    cv2.imshow("Object + Wall Edges", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
