# Lidar模块使用指南
## 程序运行需要的修改
为保证代码在git中测试通过，运行模式为离线模式，在线运行需要进行如下修改。
1. 修改param.json文件中的test_mode为false
2. 修改/src/object_detection.cpp中第8行代码`PyRun_SimpleString("sys.path.append(\'/home/docker/tiev-plus/modules/src/lidar/second.pytorch/second/pytorch\')");`为`PyRun_SimpleString("sys.path.append(\'/home/tiev-plus/second.pytorch/second/pytorch\')");`
3. 编译运行
## 功能解释
 - ***_lidar_processing 雷达解析
 该代码主要接收由激光雷达发送的udp数据包，从中解析出点所有点，并进行一帧完整点云的拼接。
 - data_association 数据关联
 该代码主要对利用一个矩阵（前后帧物体关联可能性分数）进行物体关联，生成关联结果。
 - drivable_area_detection 可行使区域检测
 该代码进行可行使区域的检测，利用多分辨率栅格地图剔除地面，生成可行使区域。
 - kalman_multitracker 卡尔曼多物体追踪
 该代码主要对多个物体使用卡尔曼算法进行追踪，包括前后帧物体关联概率计算，新物体添加追踪与旧物体剔除追踪列表。
 - laser_module_zcm 雷达模块ZCM
 该代码主要解决雷达模块所有通信问题，包括消息的构造、发送、接收、解析。
 - lidar 主函数
 该代码包含了激光模块的main函数，负责读取外部的参数，启动多个线程接收并处理点云，接收发送消息、追踪等。
 - lidar_fusion 雷达融合
 该代码主要融合多个雷达点云到一个坐标系下，包括外参的校准以及多帧之间时间戳的校准。
 - lidar_visualization 激光可视化
 该代码主要对激光模块产生的各种结果进行可视化，包括栅格图、物体检测结果、追踪结果。
 - linear_kalman_filter 卡尔曼滤波
 该代码主要实现了卡尔曼算法。
 - object_detection 物体检测
 该代码调用second深度学习模型进行点云物体检测。