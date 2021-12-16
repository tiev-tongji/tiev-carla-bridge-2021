#ifndef __STRUCTS_H__
#define __STRUCTS_H__
#include <vector>
#include <stdint.h>
#include <iostream>
#include <string>
//#include <Eigen/Core>
//#include <Eigen/Geometry>

struct LidarPoint {
  float x;
  float y;
  float z;
  float intensity;
  int32_t time_offset;
  LidarPoint() {
    x = 0;
    y = 0;
    z = 0;
    intensity = 0;
    time_offset = 0;
  }
  LidarPoint(float _x, float _y, float _z, float _intensity) {
    x = _x;
    y = _y;
    z = _z;
    intensity = _intensity;
    time_offset = 0;
  }
  LidarPoint(float _x, float _y, float _z, float _intensity,
             int32_t _time_offset) {
    x = _x;
    y = _y;
    z = _z;
    intensity = _intensity;
    time_offset = _time_offset;
  }
};

struct OneFrame {
  int64_t timestamp;
  std::vector<LidarPoint> frame_data;
};

struct GridMapPoint {
  bool obstacle;
  LidarPoint point;
  GridMapPoint(LidarPoint p, bool o) {
    point = p;
    obstacle = o;
  }
};

struct GridMapCell {
  unsigned short obstacle;
  float min_z;
  float max_z;
  int point_num;
  bool is_noise;
  GridMapCell() {
    obstacle = 0x00;
    min_z = 0;
    max_z = 0;
    point_num=0;
    is_noise=false;
  }
};

struct GridMap {
  float resolution;
  int rows, cols;
  int origin_row, origin_col;
  int cell_num;
  GridMapCell* cells;
  GridMap() {
    resolution = 0;
    rows = 0;
    cols = 0;
    origin_row = 0;
    origin_col = 0;
    cell_num = 0;
  }
};

struct LidarObstacle{
  int type;
  double x;
  double y;
  double z;
  double length;
  double width;
  double heigh;
  double heading;
  LidarObstacle(int _type,double _x,double _y,double _z,double _length,double _width,double _heigh,double _heading){
    type=_type;
    x=_x;
    y=_y;
    z=_z;
    length=_length;
    width=_width;
    heigh=_heigh;
    heading=_heading;
  }
  LidarObstacle(){
    type=0;
    x=0;
    y=0;
    z=0;
    length=0;
    width=0;
    heigh=0;
    heading=0;
  }
};
//
//struct TFrame{
//  OneFrame frame;
//  Eigen::Isometry3d T;
//  int64_t time_stamp;
//  /**
//   * @brief Construct a new TFrame object
//   * 
//   * @param _time_stamp 
//   * @param _frame 
//   * @param euler_angle RPY(X,Y,Z)
//   * @param tran 
//   */
//  TFrame(int64_t _time_stamp,OneFrame _frame, Eigen::Vector3d euler_angle,Eigen::Vector3d tran){
//    frame=_frame;
//    time_stamp=_time_stamp;
//    T=Eigen::Isometry3d::Identity();
//    Eigen::Matrix3d rotation_matrix3;
//    rotation_matrix3 = Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitZ()) * 
//                       Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) * 
//                       Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitX());
//    T.rotate(rotation_matrix3);
//    T.pretranslate(tran);
//  }
//  TFrame(){
//    OneFrame empty_frame;
//    frame=empty_frame;
//    time_stamp=0;
//    T=Eigen::Isometry3d::Identity();
//    Eigen::Matrix3d rotation_matrix3;
//    rotation_matrix3 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) * 
//                       Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
//                       Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
//    T.rotate(rotation_matrix3);
//    T.pretranslate(Eigen::Vector3d(0,0,0));
//  }
//};
//
enum ObstacleType{
  OBSTACLE_CAR=0,
  OBSTACLE_BICYCLIST = 1,
  OBSTACLE_PEDESTRIAN = 2,
  OBSTACLE_UNKNOWN = 127
} ;

struct Parameter{
    bool display_source_point_cloud;

    bool lidar_setting_test_mode;
    double lidar_setting_pitch_compensation;
    std::string lidar_setting_ruby_lidar_ip;
    int lidar_setting_ruby_lidar_port;
    std::string lidar_setting_rs16_left_lidar_ip;
    int lidar_setting_rs16_left_lidar_port;
    std::string lidar_setting_rs16_right_lidar_ip;
    int lidar_setting_rs16_right_lidar_port;
    std::string lidar_setting_blind_front_lidar_ip;
    int lidar_setting_blind_front_lidar_port;
    std::string lidar_setting_blind_back_lidar_ip;
    int lidar_setting_blind_back_lidar_port;

    double grid_map_setting_map_size_x;
    double grid_map_setting_map_size_y;
    double grid_map_setting_origin_in_map_x;
    double grid_map_setting_origin_in_map_y;
    double grid_map_setting_grid_map_z_resolution;
    double grid_map_setting_grid_map_resolution;
    double grid_map_setting_grid_map_obstacle_threshold;
    double grid_map_setting_car_top_to_pandar;
    double grid_map_setting_cat_bottom_to_pandar;
    double grid_map_setting_grid_map_point_to_origin_min;
    int grid_map_setting_noise_threshold;
    int grid_map_setting_noise_search_window_expend;

    double kalmax_filter_setting_correspondence_threshold;
    double kalmax_filter_setting_pruning_threshold;
    double kalmax_filter_setting_measurement_variance;
    double kalmax_filter_setting_position_variance;
    double kalmax_filter_setting_velocity_variance;
    double kalmax_filter_setting_initial_position_variance;
    double kalmax_filter_setting_initial_velocity_variance;
  
};



inline void Transform(double x, double y, double theta, double tx, double ty,
                      double& x1, double& y1) {
  x1 = cos(theta) * x - sin(theta) * y + tx;
  y1 = sin(theta) * x + cos(theta) * y + ty;
}

#endif