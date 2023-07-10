import cv2
import numpy as np
import imutils

# Kamera ile tespit edilen cismin kameraya gözüken
# alanından kameraya olan uzaklığını bulmaya çalışma

cap = cv2.VideoCapture(0)

_, frame = cap.read()

lower_blue = np.array([90,60,0],np.uint64)
upper_blue = np.array([121,255,255],np.uint64)

hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

mask = cv2.inRange(hsv_img, lower_blue, upper_blue)

contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contours = imutils.grab_contours(contours)

# Bu örnekte mavi cisimlerin kontorlarını aldık
for i in contours:
	area = cv2.contourArea(i)
	# cv2'nin hesapladıgı alan ile aşağıdaki 
	# formülü kullanarak cismin uzaklığını bul
	distance = 2*(10**(-7))* (area**2) - (0.0067 * area) + 83.487

	cv2.drawContours(frame, [i], -1, (0,255,0), 3)

	M = cv2.moments(i)
	cx = int(M["m10"] / M["m00"])
	cy = int(M["m01"] / M["m00"])

	S = 'Cismin Uzakligi = ' + str(distance)

	cv2.circle(frame, (cx, cy), 7, (255,255,255), -1)
	cv2.putText(frame, S , (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255,255,255), 3)

cv2.imshow("Uzaklik  Tespiti", frame)