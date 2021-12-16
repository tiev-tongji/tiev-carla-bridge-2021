#include <drivable_area_detection.h>

void DrivableAreaDetector::GridMapInitialize()
{
  // Initialize grid_map_z
  grid_map_z.resolution = parameters.grid_map_setting_grid_map_z_resolution;
  grid_map_z.rows = parameters.grid_map_setting_map_size_x / parameters.grid_map_setting_grid_map_z_resolution + 1;
  grid_map_z.cols = parameters.grid_map_setting_map_size_y / parameters.grid_map_setting_grid_map_z_resolution + 1;
  grid_map_z.origin_row = parameters.grid_map_setting_origin_in_map_x / parameters.grid_map_setting_grid_map_z_resolution + 1;
  grid_map_z.origin_col = parameters.grid_map_setting_origin_in_map_y / parameters.grid_map_setting_grid_map_z_resolution + 1;
  grid_map_z.cell_num = grid_map_z.rows * grid_map_z.cols;
  grid_map_z.cells = new GridMapCell[grid_map_z.cell_num];
  for (int i = 0; i < grid_map_z.cell_num; i++)
  {
    (grid_map_z.cells + i)->max_z = -FLT_MAX;
    (grid_map_z.cells + i)->min_z = FLT_MAX;
    (grid_map_z.cells + i)->obstacle = 0x00;
    (grid_map_z.cells + i)->point_num = 0;
    (grid_map_z.cells + i)->is_noise = false;
  }
  // Initialize grid_map
  grid_map.resolution = parameters.grid_map_setting_grid_map_resolution;
  grid_map.rows = parameters.grid_map_setting_map_size_x / parameters.grid_map_setting_grid_map_resolution + 1;
  grid_map.cols = parameters.grid_map_setting_map_size_y / parameters.grid_map_setting_grid_map_resolution + 1;
  grid_map.origin_row = parameters.grid_map_setting_origin_in_map_x / parameters.grid_map_setting_grid_map_resolution + 1;
  grid_map.origin_col = parameters.grid_map_setting_origin_in_map_y / parameters.grid_map_setting_grid_map_resolution + 1;
  grid_map.cell_num = grid_map.rows * grid_map.cols;
  grid_map.cells = new GridMapCell[grid_map.cell_num];
  for (int i = 0; i < grid_map.cell_num; i++)
  {
    (grid_map.cells + i)->max_z = -FLT_MAX;
    (grid_map.cells + i)->min_z = FLT_MAX;
    (grid_map.cells + i)->obstacle = 0x00;
    (grid_map.cells + i)->point_num = 0;
    (grid_map.cells + i)->is_noise = false;
  }

  // Initialize fusion_grid_map
  fusion_grid_map.resolution = parameters.grid_map_setting_grid_map_resolution;
  fusion_grid_map.rows = parameters.grid_map_setting_map_size_x / parameters.grid_map_setting_grid_map_resolution + 1;
  fusion_grid_map.cols = parameters.grid_map_setting_map_size_y / parameters.grid_map_setting_grid_map_resolution + 1;
  fusion_grid_map.origin_row = parameters.grid_map_setting_origin_in_map_x / parameters.grid_map_setting_grid_map_resolution + 1;
  fusion_grid_map.origin_col = parameters.grid_map_setting_origin_in_map_y / parameters.grid_map_setting_grid_map_resolution + 1;
  fusion_grid_map.cell_num = fusion_grid_map.rows * fusion_grid_map.cols;
  fusion_grid_map.cells = new GridMapCell[fusion_grid_map.cell_num];
  for (int i = 0; i < fusion_grid_map.cell_num; i++)
  {
    (fusion_grid_map.cells + i)->max_z = -FLT_MAX;
    (fusion_grid_map.cells + i)->min_z = FLT_MAX;
    (fusion_grid_map.cells + i)->obstacle = 0x00;
    (fusion_grid_map.cells + i)->point_num = 0;
    (fusion_grid_map.cells + i)->is_noise = false;
  }
}

DrivableAreaDetector::DrivableAreaDetector()
{
  GridMapInitialize();
  std::cout << "Grid Map Successful initialization\ngrid map：" << grid_map.rows << "*" << grid_map.cols << "=" << grid_map.cell_num << std::endl;
  processed = false;
}

GridMapCell *DrivableAreaDetector::GetCell(GridMap *map, float x, float y)
{
  int r, c;
  if (map == NULL)
    return NULL;
  c = -1 * (int)floor((y + map->resolution / 2) / map->resolution) + map->origin_col;
  r = (int)floor((x + map->resolution / 2) / map->resolution) + map->origin_row;
  if (r < 0 || c < 0 || r >= map->rows || c >= map->cols)
    return NULL;
  else
  {
    return map->cells + (r * map->cols + c);
  }
}

void DrivableAreaDetector::LabelPoint()
{
  //calculate min z for every cell
  for (auto grid_point : frame_data)
  {
    GridMapCell *g_cell =
        GetCell(&grid_map_z, grid_point.point.x, grid_point.point.y);
    if (g_cell != NULL)
      g_cell->min_z = std::min(g_cell->min_z, grid_point.point.z);
  }
  std::vector<GridMapPoint>::iterator it;
  for (it = frame_data.begin(); it < frame_data.end(); it++)
  {
    GridMapCell *g_cell =
        GetCell(&grid_map_z, (*it).point.x, (*it).point.y);
    if (g_cell != NULL)
    {
      if (((*it).point.z - g_cell->min_z) > parameters.grid_map_setting_grid_map_obstacle_threshold)
        (*it).obstacle = true;
    }
  }
}

void DrivableAreaDetector::LabelCell()
{
  for (auto grid_point : frame_data)
  {
    if (grid_point.point.z > parameters.grid_map_setting_cat_bottom_to_pandar &&
        grid_point.point.z < parameters.grid_map_setting_car_top_to_pandar)
    {
      GridMapCell *g_cell =
          GetCell(&grid_map, grid_point.point.x, grid_point.point.y);
      if (g_cell != NULL)
      {
        g_cell->min_z = std::min(g_cell->min_z, grid_point.point.z);
        g_cell->max_z = std::max(g_cell->max_z, grid_point.point.z);
        g_cell->point_num = g_cell->point_num + 1;
        if ((g_cell->max_z - g_cell->min_z) > parameters.grid_map_setting_grid_map_obstacle_threshold)
          g_cell->obstacle = 0x01;
        if (sqrt(pow(grid_point.point.x, 2) + pow(grid_point.point.y, 2)) >
            parameters.grid_map_setting_grid_map_point_to_origin_min)
        {
          if (grid_point.obstacle)
          {
            g_cell->obstacle = 0x01;
          }
        }
      }
    }
  }
}

void DrivableAreaDetector::CleanCarAround()
{

  // clear noisy
  // this variable decide the size of search window.for example,if expend is 1,
  // the search window is 3x3
  int expend = parameters.grid_map_setting_noise_search_window_expend;
  for (int i = 0; i < grid_map.cell_num; i++)
  {
    int num = 0;
    if (i / grid_map.cols < parameters.grid_map_setting_noise_search_window_expend
          || i / grid_map.cols >= grid_map.rows - parameters.grid_map_setting_noise_search_window_expend
          || i % grid_map.cols < parameters.grid_map_setting_noise_search_window_expend
          || i % grid_map.cols >= grid_map.cols - parameters.grid_map_setting_noise_search_window_expend)
      continue;
    for (int j = -1 * expend; j <= expend; j++)
    {
      for (int k = -1 * expend; k <= expend; k++)
      {
        int index = i + j * grid_map.cols + k;
        if (index >= 0 && index < grid_map.cell_num &&
            (grid_map.cells + index)->obstacle == 0x01)
          num++;
      }
    }
    if (num <= parameters.grid_map_setting_noise_threshold)
    {
      (grid_map.cells + i)->is_noise = true;
      (grid_map.cells + i)->obstacle = 0x00;
    }
  }
  // clear noisy around car
  for (int i = -23; i < 8; i++)
  {
    for (int j = -7; j < 8; j++)
    {
      int index =
          (grid_map.origin_row + i) * grid_map.cols + (grid_map.origin_col + j);
      if ((grid_map.cells + index)->obstacle == 0x01)
        (grid_map.cells + index)->obstacle = 0x00;
    }
    // if (i < -5)
    // {
    //   for (int j = -5; j < 5; j++)
    //   {
    //     int index =
    //         (parameters.grid_map_setting_origin_in_map_y + i) * grid_map.cols + (parameters.grid_map_setting_origin_in_map_x + j);
    //     if ((grid_map.cells + index)->obstacle == 0x01)
    //       (grid_map.cells + index)->obstacle = 0x00;
    //   }
    // }
    // else if (i < 8)
    // {
    //   for (int j = -7; j < 8; j++)
    //   {
    //     int index =
    //         (parameters.grid_map_setting_origin_in_map_y + i) * grid_map.cols + (parameters.grid_map_setting_origin_in_map_x + j);
    //     if ((grid_map.cells + index)->obstacle == 0x01)
    //       (grid_map.cells + index)->obstacle = 0x00;
    //   }
    // }
    // else if (i < 11)
    // {
    //   for (int j = -5; j < 5; j++)
    //   {
    //     int index =
    //         (parameters.grid_map_setting_origin_in_map_y + i) * grid_map.cols + (parameters.grid_map_setting_origin_in_map_x + j);
    //     if ((grid_map.cells + index)->obstacle == 0x01)
    //       (grid_map.cells + index)->obstacle = 0x00;
    //   }
    // }
    // else
    // {
    //   for (int j = -2; j < 2; j++)
    //   {
    //     int index =
    //         (parameters.grid_map_setting_origin_in_map_y + i) * grid_map.cols + (parameters.grid_map_setting_origin_in_map_x + j);
    //     if ((grid_map.cells + index)->obstacle == 0x01)
    //       (grid_map.cells + index)->obstacle = 0x00;
    //   }
    // }
  }
  processed = true;
}

// no other dependencies
GridMap DrivableAreaDetector::GetGridMap(OneFrame *frame)
{
  if (frame->frame_data.size() != 0)
  {
    // clean grid map info
    CleanGridInfo();
    // clean frame date
    frame_data.clear();
    // clean processed tag
    processed = false;
    // save new frame date to frame_date
    for (auto point : frame->frame_data)
    {
      if (point.x == 0 && point.y == 0 && point.z == 0)
        continue;
      if (((int)floor(-1 * point.y / grid_map.resolution) + grid_map.origin_col) >
              grid_map.cols ||
          ((int)floor(point.x / grid_map.resolution) + grid_map.origin_row) >
              grid_map.rows)
        continue;
      if(point.z > parameters.grid_map_setting_cat_bottom_to_pandar && point.z <parameters.grid_map_setting_car_top_to_pandar){
        GridMapPoint p(point, false);
        frame_data.push_back(p);
      }
      
    }
    LabelPoint(); // no other dependencies
    LabelCell(); // no other dependencies
    CleanCarAround(); // no other dependencies
    if (processed)
      return grid_map;
  }
  else
  {
    GridMap empty_map;
    return empty_map;
  }
}

void DrivableAreaDetector::CleanGridInfo()
{
  for (int i = 0; i < grid_map_z.cell_num; i++)
  {
    (grid_map_z.cells + i)->max_z = -FLT_MAX;
    (grid_map_z.cells + i)->min_z = FLT_MAX;
    (grid_map_z.cells + i)->obstacle = 0x00;
    (grid_map_z.cells + i)->point_num = 0;
    (grid_map_z.cells + i)->is_noise = false;
  }
  for (int i = 0; i < grid_map.cell_num; i++)
  {
    (grid_map.cells + i)->max_z = -FLT_MAX;
    (grid_map.cells + i)->min_z = FLT_MAX;
    (grid_map.cells + i)->obstacle = 0x00;
    (grid_map.cells + i)->point_num = 0;
    (grid_map.cells + i)->is_noise = false;
  }
}

GridMap DrivableAreaDetector::GetGridMapFromMultiFusionFrame(std::vector<OneFrame> fusion_frames)
{
  //clean date
  for (int i = 0; i < fusion_grid_map.cell_num; i++)
  {
    (fusion_grid_map.cells + i)->max_z = -FLT_MAX;
    (fusion_grid_map.cells + i)->min_z = FLT_MAX;
    (fusion_grid_map.cells + i)->obstacle = 0x00;
    (fusion_grid_map.cells + i)->point_num = 0;
    (fusion_grid_map.cells + i)->is_noise = false;
  }

  GetGridMap(&fusion_frames[0]);
  for (int i = 0; i < fusion_grid_map.cell_num; i++)
  {
    if((grid_map.cells + i)->obstacle == 0x01 ){
      (fusion_grid_map.cells + i)->max_z = (grid_map.cells + i)->max_z;
      (fusion_grid_map.cells + i)->min_z = (grid_map.cells + i)->min_z;
      (fusion_grid_map.cells + i)->obstacle = (grid_map.cells + i)->obstacle;
      (fusion_grid_map.cells + i)->point_num = (grid_map.cells + i)->point_num;
      (fusion_grid_map.cells + i)->is_noise = (grid_map.cells + i)->is_noise;
    }
  }

  GetGridMap(&fusion_frames[1]);
  for (int i = 0; i < fusion_grid_map.cell_num; i++)
  {
    if((grid_map.cells + i)->obstacle == 0x01 ){
      (fusion_grid_map.cells + i)->max_z = (grid_map.cells + i)->max_z;
      (fusion_grid_map.cells + i)->min_z = (grid_map.cells + i)->min_z;
      (fusion_grid_map.cells + i)->obstacle = (grid_map.cells + i)->obstacle;
      (fusion_grid_map.cells + i)->point_num = (grid_map.cells + i)->point_num;
      (fusion_grid_map.cells + i)->is_noise = (grid_map.cells + i)->is_noise;
    }
  }

  GetGridMap(&fusion_frames[2]);
  for (int i = 0; i < fusion_grid_map.cell_num; i++)
  {
    if((grid_map.cells + i)->obstacle == 0x01 ){
      (fusion_grid_map.cells + i)->max_z = (grid_map.cells + i)->max_z;
      (fusion_grid_map.cells + i)->min_z = (grid_map.cells + i)->min_z;
      (fusion_grid_map.cells + i)->obstacle = (grid_map.cells + i)->obstacle;
      (fusion_grid_map.cells + i)->point_num = (grid_map.cells + i)->point_num;
      (fusion_grid_map.cells + i)->is_noise = (grid_map.cells + i)->is_noise;
    }
  }
  return fusion_grid_map;
}
