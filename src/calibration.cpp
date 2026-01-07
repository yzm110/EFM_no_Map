/*
 * @Author: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @Date: 2023-12-26 20:13:42
 * @LastEditors: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @LastEditTime: 2024-04-22 11:32:56
 * @FilePath: /repo82/code/dx11_noa/application/environmentmodelfunction/src/calibration.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "calibration.h"
#include <cstdint> // Include the necessary header file

int p_efm_use_centerline = 1;
uint32_t p_odd_to2_distance = 1600;
uint32_t p_odd_to0_distance = 100;
double p_bigcurve_oddto2_distance = 1820;
double p_bigcurve_oddto0_distance = 310;
float p_bigcurve_odd_radius = 35.0f;
float p_bigcurve_odd_abnomal_radius = 50.0f;
uint16_t p_lastlink_back_search_distance = 100;
uint32_t p_curv_point_dis_min = 10; 
uint32_t p_curv_point_dis_max = 50;
uint16_t p_lane_width_max = 530;
uint32_t p_narrow_lane_width = 200;
double p_kerb_back_distance = -15.0;
double p_kerb_forward_distance = 80.0;
double p_exit_start_end_gap = 500.0;
uint32_t p_efm_use_pointforsplit = 1;
double p_toll_odd_to0_distance = 100.0; 
double p_service_odd_to0_distance = 1.0; 
uint32_t p_tollback_bigcurv_distance = 350;
uint32_t p_tollahead_bigcurv_distance = 150;

double p_back_center_line_sample_dist = 2.5;   // uint m, 
double p_front_center_line_sample_dist = 2.5;  // uint m, 
double p_back_side_line_sample_dist = 2.5;   // uint m, 
double p_front_side_line_sample_dist = 2.5;  // uint m,

uint32_t p_bigwidth_forward_distance = 20; // m
uint32_t p_bigwidth_back_distance = 10; // m
double p_land_end_dist = 500;

RefLineSmoothConfig refline_smooth_config = {1.0e+10, 1.0, 1.0, 0.2,   0.05, 0.1, 20, 20, 2,    25,   25, 0.032, 0.015,
                                             0.8,     10,  10,  0.032, 0.03, 1,   28, 15, 0.55, 0.25, 3,  8,     8,15,15,15,15,10,50,15,0.0034,25,40,100,1.3, 35};
// struct RefLineSmoothConfig {
//     double weight_fem_pos_deviation = 1.0e+10;
//     double weight_path_length = 1.0;
//     double weight_ref_deviation = 1.0;
//     double curvature_constraint = 0.2;
//     double ego_bounds_val = 0.05;
//     double bounds_val = 0.1;
//     double ego_s_front = 20;
//     double ego_s_back = 20;
//     double no_boundary_bounds_val = 2;
//     double no_boundary_s_front = 25;
//     double no_boundary_s_back = 25;
//     double severe_bend_kappa_up_limt = 0.032;
//     double severe_bend_kappa_down_limt = 0.015;
//     double severe_bend_bounds_val = 0.8;
//     double severe_bend_s_front = 10;
//     double severe_bend_s_back = 10;
//     double s_shape_kappa_up_limt = 0.032;
//     double s_shape_kappa_down_limt = 0.03;
//     double s_shape_bounds_val = 1;
//     double s_shape_s_front = 28;
//     double s_shape_s_back = 15;
//     double z_shape_heading_up_limt = 0.55;
//     double z_shape_heading_down_limt = 0.25;
//     double z_shape_bounds_val = 3;
//     double z_shape_s_front = 8;
//     double z_shape_s_back = 8;
// } refline_smooth_config;

double p_offramp_speed_lmt_dist = 1000; // m
double p_tunnel_speed_lmt_front_dist = 300; // m 隧道前几m激活隧道融合限速
double p_tunnel_speed_lmt_keep_dist = 100; // m 过隧道后隧道限速保持多少m
uint32_t p_tollback_odd_distance = 700;
uint32_t p_y_shape_distance = 150; // m
int p_smooth_line_num = 1; // 0,none; 1-ego;2-ego&&prior;3- three lane
double p_toll_speed_lmt_front_dist = 1200;
int p_tunnel_use_hd_speed = 1;
int p_position_to_split_dist = 4000;
double p_heading_for_merge_first_level = 0.05;
double p_heading_for_merge_sec_level = 0.05;
double p_heading_for_split_first_level = 0.05;
double p_heading_for_split_sec_level = 0.05;
uint8_t p_mean_filter_window_size = 30;
double p_limit_extend_ratio = 2.0;
int p_use_efm_log = 1; 
int p_nearest_point_offset_point_num = 20; 
double p_merge_link_centerline_nearest_distance = 0.3;
double p_exit_start_speed_urb_expway = 65;//unit kph
