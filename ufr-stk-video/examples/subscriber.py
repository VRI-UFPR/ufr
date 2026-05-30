import ufr
import cv2

video = ufr.Subscriber("@new video @@new mqtt @@coder msgpack @@topic camera1")

# Main Loop
while True:
    print("opa")
    # get image and detect objects
    img = video.recv_cv_image()
    cv2.imshow("opa", img)
    cv2.waitKey(1)
