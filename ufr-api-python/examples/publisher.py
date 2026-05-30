import ufr
import time

link = ufr.Publisher("@new mqtt @coder msgpack")
for i in range(5):
    link.put("%d %f %s\n", 10+i, 1.125*i, "opa"+str(i))
    time.sleep(1)
link.close()

