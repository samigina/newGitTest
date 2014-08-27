import cv2

cap = cv2.VideoCapture(0)

while 1:

	flag, image = cap.read()
	cv2.imshow("frame", image)
	cv2.waitKey(25)


