/**
 * @file drivable_area_detection.h
 * @author tianxuebo (940023303@qq.com)
 * @brief 
 * @version 0.1
 * @date 2020-06-17
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __DRIVABLE_AREA_DETECTION_H__
#define __DRIVABLE_AREA_DETECTION_H__

#include <float.h>
#include <math.h>
#include <common/structs.h>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Python.h>
using std::cout;
using std::endl;
namespace py = pybind11;
using std::vector;



class DrivableAreaDetector {
 private:
  /**
   * @brief this grid map's resolution is heigh. it's finnal grid map
   *
   */
  GridMap grid_map;

  /**
   * @brief this grid map's resolution is low.it is used to label point whether
   * is a obstacle.
   *
   */
  GridMap grid_map_z;

  /**
   * @brief fusion three grid map into one
   * 
   */
  GridMap fusion_grid_map;
  /**
   * @brief this variable is used to sava lidar frame date
   * 
   */
  std::vector<GridMapPoint> frame_data;
  bool processed;



  /**
   * @brief Get the Cell object
   * 
   * @param grid_map:the cell come from which grid map
   * @param x : point's x position
   * @param y : point's y position
   * @return GridMapCell* :the cell containing the point(x,y)
   */
  GridMapCell* GetCell(GridMap* grid_map, float x, float y);

  /**
   * @brief detect which point is obstacle using low-resolution grid map
   * 
   */
  void LabelPoint();

  /**
   * @brief detect which cell is obstacle in high-resolution grid map
   * 
   */
  void LabelCell();
  
  /**
   * @brief clean dynamic obstable in gridmap
   * 
   */
	void CleanDynamicObs(vector<vector<float>> object_list, bool eraseObs, GridMap *map);

  /**
   * @brief clean noise around the car
   * 
   */
  void CleanCarAround();

  /**
   * @brief clean grid maps' information
   * 
   */
  void CleanGridInfo();
 public:
  /**
   * @brief Construct a new Drivable Area Detector object
   * 
   */
  DrivableAreaDetector();
  ~DrivableAreaDetector();

  /**
 * @brief initialize the grid maps
 *
 */
  void GridMapInitialize();

  /**
   * @brief Get the Grid Map object
   *  in this function,all number variabel will be initialized or clean
   * @param frame  : this frame are used to detect where are drivabel 
   * @return GridMap : return grid map which has been detected drivable area
   */
  void GetGridMap(const py::array_t<float> &points_1d, py::list &fusionmap, 
      int show_info, vector<vector<float>> object_list, bool eraseObs);

  GridMap GetGridMapFromMultiFusionFrame(std::vector<OneFrame> fusion_frames);

  Parameter parameters;
};

#endif