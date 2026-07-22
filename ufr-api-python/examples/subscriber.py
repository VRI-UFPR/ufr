# =======================================================================================
#  Header
# =======================================================================================

import ufr
import time
import numpy as np
import ctypes
import cv2

# =======================================================================================
#  Main
# =======================================================================================

link = ufr.subscriber("@new mqtt @coder msgpack")
for i in range(3):
    res = link.get("> %d %f %s %a:d")
    print(res)
link.close()


"""
link = ufr.client("@new cuda")
for i in range(3):
    link.put("#help %s nome=%s idade=%d\n")
    link.get(">!")
    # link.get("> %a:f:10")
    print(res)
link.close()
"""