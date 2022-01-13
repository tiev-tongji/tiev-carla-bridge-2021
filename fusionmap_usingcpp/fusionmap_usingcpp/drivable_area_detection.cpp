#include <drivable_area_detection.h>
using std::cout;
using std::endl;


DrivableAreaDetector::DrivableAreaDetector()
{
    // initialize parameters
    parameters.grid_map_setting_map_size_x = 100;
    parameters.grid_map_setting_map_size_y = 50;
    parameters.grid_map_setting_origin_in_map_x = 30;
    parameters.grid_map_setting_origin_in_map_y = 25;
    parameters.grid_map_setting_grid_map_z_resolution = 0.3;
    parameters.grid_map_setting_grid_map_resolution = 0.2;
    parameters.grid_map_setting_grid_map_obstacle_threshold = 0.15;
    parameters.grid_map_setting_car_top_to_pandar = 0.3;
    parameters.grid_map_setting_cat_bottom_to_pandar = -0.5;
    parameters.grid_map_setting_grid_map_point_to_origin_min = 1;
    parameters.grid_map_setting_noise_threshold = 3;
    parameters.grid_map_setting_noise_search_window_expend = 1;

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
    processed = false;
}

DrivableAreaDetector::~DrivableAreaDetector()
{
    delete(grid_map.cells);
    delete(grid_map_z.cells);
    cout << "DrivableAreaDetector destroyed ! " << endl;
}

void DrivableAreaDetector::GridMapInitialize()
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
    processed = false;

}




// points_1d should be x pointing right, y pointing backward, z pointing downward as in carla
// grid_map.cells id is rows first. id = r*num_cols + c
// points_1d after transformation belongs to a right-hand coordination system with x pointing right and y pointing forward; 
// The origin of the gridmap is at the lower left corner.
// so the coordinate of point cloud data and dynamic object list should be tranformed
void DrivableAreaDetector::GetGridMap(const py::array_t<float> &points_1d, py::list &fusionmap, int show_info,
    vector<vector<float>> object_list, bool eraseObs=true)
{
    time_t start, end;
    start = clock();
    GridMapInitialize();
    end = clock();
    if(show_info) cout << "initialization: " << end - start << endl;
    if (points_1d.size() != 0)
    {
        
        start = clock();
        // clean grid map info
        CleanGridInfo();
        // clean frame date
        frame_data.clear();
        // clean processed tag
        processed = false;
        float x, y, z;
        py::buffer_info buf1 = points_1d.request();
        float *ptr1 = (float *)buf1.ptr;
        
        // save new frame date to frame_date
        for (int i=0;i<points_1d.size();i+=3)
        {
            x = ptr1[i];
            y = -ptr1[i+1];
            z = -ptr1[i + 2];
            if (x == 0 && y == 0 &&z == 0)
                continue;
            if (((int)floor(y / grid_map.resolution) + grid_map.origin_row) >
                grid_map.rows ||
                ((int)floor(x / grid_map.resolution) + grid_map.origin_col) >
                grid_map.cols ||
                ((int)floor(x / grid_map.resolution) + grid_map.origin_col) < 0 || 
                ((int)floor(y / grid_map.resolution) + grid_map.origin_row) < 0
                )
                continue;
            if ( z < parameters.grid_map_setting_car_top_to_pandar) {
                GridMapPoint p(LidarPoint(x,y,z,1), false);
                frame_data.push_back(p);
            }

        }
        end = clock();
        if (show_info) cout << "collect point: " << end - start << endl;

        start = clock();
        LabelPoint(); // no other dependencies
        end = clock();
        if (show_info) cout << "LabelPoint: " << end - start << endl;

        start = clock();
        LabelCell(); // no other dependencies
        end = clock();
        if (show_info) cout << "LabelCell: " << end - start << endl;

        CleanDynamicObs(object_list, eraseObs, &grid_map);
        //CleanCarAround(); // no other dependencies
        start = clock();
        for (int r = grid_map.rows - 1; r >= 0; --r) {
            py::list tmp;
            for (int c = 0; c < grid_map.cols; ++c) {
                tmp.append((grid_map.cells + r * grid_map.cols + c)->obstacle);
            }
            fusionmap.append(tmp);
        }
        end = clock();
        if (show_info) cout << "PackFusionmap: " << end - start << endl;
    }
    else
    {
        py::list empty_fusionmap;
    }
}


GridMapCell *DrivableAreaDetector::GetCell(GridMap *map, float x, float y)
{
  int r, c;
  if (map == NULL)
    return NULL;
  c = (int)floor((x + map->resolution / 2) / map->resolution) + map->origin_col;
  r = (int)floor((y + map->resolution / 2) / map->resolution) + map->origin_row;
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
      g_cell->min_z = (std::min)(g_cell->min_z, grid_point.point.z);
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
    if (grid_point.point.z < parameters.grid_map_setting_car_top_to_pandar)
    {
      GridMapCell *g_cell =
          GetCell(&grid_map, grid_point.point.x, grid_point.point.y);
      if (g_cell != NULL)
      {
        g_cell->min_z = (std::min)(g_cell->min_z, grid_point.point.z);
        g_cell->max_z = (std::max)(g_cell->max_z, grid_point.point.z);
        g_cell->point_num = g_cell->point_num + 1;
        if ((g_cell->max_z - g_cell->min_z) > parameters.grid_map_setting_grid_map_obstacle_threshold)
          g_cell->obstacle = 0x02;
        if (sqrt(pow(grid_point.point.x, 2) + pow(grid_point.point.y, 2)) >
            parameters.grid_map_setting_grid_map_point_to_origin_min)
        {
          if (grid_point.obstacle)
          {
            g_cell->obstacle = 0x02;
          }
        }
      }
    }
  }
}

// clean dynamic object point cloud.
// (obs_start_x,obs_start_y) as origin, (obs_end_x - obs_start_x, obs_end_y - obs_start_y) as x vector
// inflation_size is used for increasing the cleaning area.
void DrivableAreaDetector::CleanDynamicObs(vector<vector<float>> object_list, bool eraseObs, GridMap *map) {
    if (!eraseObs) return;
    float obs_start_x, obs_start_y, obs_end_x, obs_end_y;
    float length, width, inflation_size, theta, x, y, x_trans, y_trans;
    for (int i = 0; i < object_list.size(); ++i) {
        obs_start_x = object_list[i][0];
        obs_start_y = object_list[i][1];
        obs_end_x = object_list[i][2];
        obs_end_y = object_list[i][3];
        length = object_list[i][4];
        width = object_list[i][5];
        inflation_size = object_list[i][6];
        theta = -atan2(obs_end_y - obs_start_y, obs_end_x - obs_start_x);
        for (x = -inflation_size; x <= length + inflation_size;
            x += parameters.grid_map_setting_grid_map_resolution * 0.5)
            for (y = -inflation_size; y <= width + inflation_size;
                y += parameters.grid_map_setting_grid_map_resolution * 0.5) {
            // rotation
            x_trans = cos(theta) * x + sin(theta) * y + obs_start_x;
            y_trans = -sin(theta) * x + cos(theta) * y + obs_start_y;
            auto cell = GetCell(map, x_trans, y_trans);
            if (cell == NULL) continue;
            cell->obstacle = 0x00;
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
            (grid_map.cells + index)->obstacle == 0x02)
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
      if ((grid_map.cells + index)->obstacle == 0x02)
        (grid_map.cells + index)->obstacle = 0x00;
    }
    // if (i < -5)
    // {
    //   for (int j = -5; j < 5; j++)
    //   {
    //     int index =
    //         (parameters.grid_map_setting_origin_in_map_y + i) * grid_map.cols + (parameters.grid_map_setting_origin_in_map_x + j);
    //     if ((grid_map.cells + index)->obstacle == 0x02)
    //       (grid_map.cells + index)->obstacle = 0x00;
    //   }
    // }
    // else if (i < 8)
    // {
    //   for (int j = -7; j < 8; j++)
    //   {
    //     int index =
    //         (parameters.grid_map_setting_origin_in_map_y + i) * grid_map.cols + (parameters.grid_map_setting_origin_in_map_x + j);
    //     if ((grid_map.cells + index)->obstacle == 0x02)
    //       (grid_map.cells + index)->obstacle = 0x00;
    //   }
    // }
    // else if (i < 11)
    // {
    //   for (int j = -5; j < 5; j++)
    //   {
    //     int index =
    //         (parameters.grid_map_setting_origin_in_map_y + i) * grid_map.cols + (parameters.grid_map_setting_origin_in_map_x + j);
    //     if ((grid_map.cells + index)->obstacle == 0x02)
    //       (grid_map.cells + index)->obstacle = 0x00;
    //   }
    // }
    // else
    // {
    //   for (int j = -2; j < 2; j++)
    //   {
    //     int index =
    //         (parameters.grid_map_setting_origin_in_map_y + i) * grid_map.cols + (parameters.grid_map_setting_origin_in_map_x + j);
    //     if ((grid_map.cells + index)->obstacle == 0x02)
    //       (grid_map.cells + index)->obstacle = 0x00;
    //   }
    // }
  }
  processed = true;
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

//GridMap DrivableAreaDetector::GetGridMapFromMultiFusionFrame(std::vector<OneFrame> fusion_frames)
//{
//  //clean date
//  for (int i = 0; i < fusion_grid_map.cell_num; i++)
//  {
//    (fusion_grid_map.cells + i)->max_z = -FLT_MAX;
//    (fusion_grid_map.cells + i)->min_z = FLT_MAX;
//    (fusion_grid_map.cells + i)->obstacle = 0x00;
//    (fusion_grid_map.cells + i)->point_num = 0;
//    (fusion_grid_map.cells + i)->is_noise = false;
//  }
//
//  GetGridMap(&fusion_frames[0]);
//  for (int i = 0; i < fusion_grid_map.cell_num; i++)
//  {
//    if((grid_map.cells + i)->obstacle == 0x02 ){
//      (fusion_grid_map.cells + i)->max_z = (grid_map.cells + i)->max_z;
//      (fusion_grid_map.cells + i)->min_z = (grid_map.cells + i)->min_z;
//      (fusion_grid_map.cells + i)->obstacle = (grid_map.cells + i)->obstacle;
//      (fusion_grid_map.cells + i)->point_num = (grid_map.cells + i)->point_num;
//      (fusion_grid_map.cells + i)->is_noise = (grid_map.cells + i)->is_noise;
//    }
//  }
//
//  GetGridMap(&fusion_frames[1]);
//  for (int i = 0; i < fusion_grid_map.cell_num; i++)
//  {
//    if((grid_map.cells + i)->obstacle == 0x02 ){
//      (fusion_grid_map.cells + i)->max_z = (grid_map.cells + i)->max_z;
//      (fusion_grid_map.cells + i)->min_z = (grid_map.cells + i)->min_z;
//      (fusion_grid_map.cells + i)->obstacle = (grid_map.cells + i)->obstacle;
//      (fusion_grid_map.cells + i)->point_num = (grid_map.cells + i)->point_num;
//      (fusion_grid_map.cells + i)->is_noise = (grid_map.cells + i)->is_noise;
//    }
//  }
//
//  GetGridMap(&fusion_frames[2]);
//  for (int i = 0; i < fusion_grid_map.cell_num; i++)
//  {
//    if((grid_map.cells + i)->obstacle == 0x02 ){
//      (fusion_grid_map.cells + i)->max_z = (grid_map.cells + i)->max_z;
//      (fusion_grid_map.cells + i)->min_z = (grid_map.cells + i)->min_z;
//      (fusion_grid_map.cells + i)->obstacle = (grid_map.cells + i)->obstacle;
//      (fusion_grid_map.cells + i)->point_num = (grid_map.cells + i)->point_num;
//      (fusion_grid_map.cells + i)->is_noise = (grid_map.cells + i)->is_noise;
//    }
//  }
//  return fusion_grid_map;
//}
