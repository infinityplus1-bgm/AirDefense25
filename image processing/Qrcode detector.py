import cv2 as cv
import numpy as np
video_source = 'video.mp4'
image = cv.imread('qr.jpeg')



def detect_qr_code(frame):
    qcd = cv.QRCodeDetector()

    retval, decoded_info, points, straight_qrcode = qcd.detectAndDecodeMulti(frame)
    # decoded_info  is the decoded information from the QR code
    # points is the coordinates of the corners of the QR code in the image


    pt1 = tuple(map(int, points[0][0]))
    pt2 = tuple(map(int, points[0][2]))
    cv.rectangle(frame, pt1, pt2, (255, 0, 0), 2)
    cv.putText(frame,decoded_info[0],pt1,cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv.LINE_AA)
    print(decoded_info[0])

def detect_shapes(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gray, (5, 5), 0)
    edges = cv.Canny(blurred, 50, 150)
    contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    detected_shapes = []

    for contour in contours:
        if cv.contourArea(contour) < 30:
            continue
        epsilon = 0.04 * cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)
        x, y, w, h = cv.boundingRect(contour)
        vertices = len(approx)
        shape_type = ""

        if vertices == 3:
            shape_type = "triangle"
        elif vertices == 4:
            aspect_ratio = float(w) / h
            shape_type = "square" if 0.95 <= aspect_ratio <= 1.05 else "rectangle"
        elif vertices > 4:
            area = cv.contourArea(contour)
            perimeter = cv.arcLength(contour, True)
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            shape_type = "circle" if circularity > 0.8 else "polygon"

        if shape_type:
            detected_shapes.append({
                "type": shape_type,
                "bbox": [x, y, x + w, y + h],
                "contour": contour
            })

    return detected_shapes

detect_qr_code(image)
detected_shapes = detect_shapes(image)
for shape in detected_shapes:
    x1, y1, x2, y2 = shape["bbox"]
    if shape["type"] == "triangle":
        cv.putText(image, f"red {shape['type']}", (x1, y1 - 5), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        cv.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

cv.imshow('QR Code', image)
cv.waitKey(0)