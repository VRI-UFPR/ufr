import ufr
import time

pub = ufr.Publisher("@new zmq @coder msgpack")
for i in range(5):
    pub.put("iiis\n", 10,20,30, "opa")
    time.sleep(1)
pub.close()

