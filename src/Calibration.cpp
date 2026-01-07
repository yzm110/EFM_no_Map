#include "Calibration.h"

int p_production_parameter = 0; //0-L2, 1-NOA
int wide_lane_width_start = 550; // 450cm
int wide_lane_width_end = 350; // 350cm
int wide_lane_width_step = 50; // 50cm
int merge_lane_start_width = 200; // 200cm
int merge_lane_end_width = 100; // 100cm
int split_lane_start_width = 100; // 100cm
int split_lane_end_width = 250; // 250cm
int virtual_merge_end_width = 400; // 400cm
int virtual_split_start_width = 400; // 400cm
int offset_line_width = 175; // 175cm
int lane_offset_step_length = 5000; // 5000cm
int car_offset_split = 8000; // 8000cm
int bev_max_offset = 15000; // 10000cm
int k_EFM_enable_intersection_topology = 0;// enable crossrode topology 0:disable, 1:enable
