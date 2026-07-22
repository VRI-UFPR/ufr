import ufr
import time

link = ufr.Publisher("@new mqtt @coder msgpack")
for i in range(3):
    link.put("%d %f %s %a:d\n", 10+i, 1.125*i, "opa"+str(i), [10, 11, 13])
    time.sleep(1)
link.close()

