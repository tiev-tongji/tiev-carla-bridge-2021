import threading
import numpy as np
import sys
import random
import time
sys.path.append('C:\\Users\\autolab\\Desktop\\Carla-0.9.9.4\\PythonAPI\\tiev-carla-sim\\fusionmap_usingcpp\\x64\\Release')
import fusionmap_usingcpp as fmcpp
sys.path.append('C:\\Users\\autolab\\Desktop\\temp\\x64\\Release')
import temp


# import time
# start = time.process_time()
# x = list(range(9))
# y = []
# # random.shuffle(x)
# fmcpp.foo(x,y)
# end = time.process_time()
# print(x)
# print(y)
# print('spend ',end-start)

a = np.random.rand(100000).astype('f4')*20
# a = [100]*1000000
b = []


for i in range(1000):
    start = time.process_time()
    b = temp.numpy(a)
    end = time.process_time()
    print(a[:5])
    print('spend ',end-start)


# for i in range(100000):
#     start = time.process_time()
#     b = fmcpp.foo(a)
#     end = time.process_time()
#     print(np.sum(np.array(b)))
#     print('spend ',end-start)


# def fusionmap_process(useful_points, height_points, pointstmp):
#     for point in pointstmp:
#         point_tuple = (point[0],point[1])
#         if point_tuple in useful_points:
#             continue
#         if point_tuple not in height_points:
#             height_points[point_tuple] = point[2]
#             continue
#         if abs(height_points[point_tuple] - point[2]) > 0.1:
#             useful_points.add(point_tuple)

# useful_points = set()
# height_points = dict()


# pointstmp = np.array([[1,1,1],[2,2,2],[3,3,3],[4,4,4]])
# sub1 = threading.Thread(target=fusionmap_process, args=(useful_points, height_points, pointstmp[:2]))
# sub2 = threading.Thread(target=fusionmap_process, args=(useful_points, height_points, pointstmp[2:]))
# sub1.start()
# sub2.start()
# sub1.join()
# sub2.join()

# print(useful_points)
# print(height_points)
