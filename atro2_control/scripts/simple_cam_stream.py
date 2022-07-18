#! /usr/bin/python3

import cv2

cam = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

while True:
	ret, frame = cam.read()
	# resize the frame
	frame = cv2.resize(frame, (300, 300))
	frame = cv2.flip(frame, 0)
	cv2.imshow("Stream", frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cam.release()
cv2.destroyAllWindows()
