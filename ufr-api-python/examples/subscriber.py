import ufr
import time
import numpy as np
import ctypes
import cv2

def get_cv_image(link):
    msg = link.get("iiip")
    # print(msg)

    im_type = msg[0]
    im_rows = msg[1]
    im_cols = msg[2]
    im_data = msg[3]
    if im_type == 16:
        im_canal = 3
        image = np.ctypeslib.as_array(
            ctypes.cast(im_data, ctypes.POINTER(ctypes.c_ubyte)),
            shape=(im_rows, im_cols, im_canal)
        )
    elif im_type == 2:
        im_canal = 1
        image = np.ctypeslib.as_array(
            ctypes.cast(im_data, ctypes.POINTER(ctypes.c_ushort)),
            shape=(im_rows, im_cols, im_canal)
        )

    return image

def recv_cv_image(link):
    link.recv()
    return get_cv_image(link)



# sub = ufr.Subscriber("@new video @id 0")
# sub = ufr.Subscriber("@new video @file video.mp4")
# sub = ufr.Subscriber("@new video @@new mqtt @@coder msgpack @@host 185.159.82.136 @@topic camera")


# nao estao funcionando
# sub = ufr.Subscriber("@new video @device /dev/video0")
# sub = ufr.Subscriber("@new video @directory teste/dataset/")
# sub = ufr.Subscriber("@new video @@new ros_humble @@coder ros_humble:image @@topic camera1")
# sub = ufr.Subscriber("@new video @@new ros_noetic @@coder ros_noetic:image @@topic camera1")

#sub1 = ufr.Subscriber("@new video @@new mqtt @@coder msgpack @@host 10.0.0.6 @@topic camera1")
sub2 = ufr.Subscriber("@new video @@new mqtt @@coder msgpack @@host 10.0.0.6 @@topic camera2")
while True:
    #imagem1 = recv_cv_image(sub1)
    imagem2 = recv_cv_image(sub2)
    # cv2.imshow('Minha Imagem1', imagem1)
    cv2.imshow('Minha Imagem2', imagem2)
    cv2.waitKey(1)

sub.close()