import sys
sys.path.append('./tiev2020')

import lcm
import threading
import time
import icumsg_lcm
from icumsg_lcm import structNAVINFO
_navinfo = structNAVINFO()
_tunnel = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
xiqumap_origin_utmXY = [328919.83836515166, 3463348.7826909614]
_navinfo.utmX = xiqumap_origin_utmXY[0]
_navinfo.utmY = xiqumap_origin_utmXY[1]
_navinfo.mHeading = 0


def pub_navinfo_loop(interval):
    while True:
        start = time.process_time()
        _tunnel.publish('NAVINFO', _navinfo.encode())
        _navinfo.utmX += 0.1
        end = time.process_time()
        sleep_time = interval - end + start
        if sleep_time > 0:
            time.sleep(sleep_time)

_threads = []
t_pub_navinfo = threading.Thread(target=pub_navinfo_loop, args=(1,))
_threads.append(t_pub_navinfo)
for thread in _threads:
    thread.start()