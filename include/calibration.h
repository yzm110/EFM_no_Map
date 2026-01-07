/*
 * @Author: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @Date: 2024-04-07 18:03:37
 * @LastEditors: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @LastEditTime: 2024-04-29 10:19:50
 * @FilePath: /repo83/code/dx11_noa/application/environmentmodelfunction/include/calibration.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <cstdint>

#include "CompileConfig.h"
#include "CommonDataType.h"

extern int p_efm_use_centerline;
extern uint32_t p_odd_to2_distance;
extern uint32_t p_odd_to0_distance;
extern double p_bigcurve_oddto2_distance; // m
extern double p_bigcurve_oddto0_distance; // m
extern float p_bigcurve_odd_radius;    // m
extern float p_bigcurve_odd_abnomal_radius;   // m
extern uint16_t p_lastlink_back_search_distance;    // m
extern uint32_t p_curv_point_dis_min;  // m
extern uint32_t p_curv_point_dis_max;  //m
extern uint16_t p_lane_width_max;      // cm
extern uint32_t p_narrow_lane_width;   //cm
extern double p_kerb_back_distance;    //m
extern double p_kerb_forward_distance;   //m
extern double p_exit_start_end_gap; //m
extern uint32_t p_efm_use_pointforsplit;
extern double p_toll_odd_to0_distance; // m
extern double p_back_center_line_sample_dist ;   // uint m, 
extern double p_front_center_line_sample_dist ;  // uint m, 
extern double p_back_side_line_sample_dist;   // uint m, 
extern double p_front_side_line_sample_dist ;  // uint m, 
extern double p_service_odd_to0_distance; // m
extern uint32_t p_tollback_bigcurv_distance; // m
extern uint32_t p_tollahead_bigcurv_distance; // m
extern uint32_t p_bigwidth_forward_distance; // m
extern uint32_t p_bigwidth_back_distance; // m
extern RefLineSmoothConfig refline_smooth_config;
extern double p_offramp_speed_lmt_dist; // m
extern double p_tunnel_speed_lmt_front_dist; // m 隧道前几m激活隧道融合限速
extern double p_tunnel_speed_lmt_keep_dist; // m 过隧道后隧道限速保持多少m
extern double p_toll_speed_lmt_front_dist;
extern uint32_t p_tollback_odd_distance; // m
extern uint32_t p_y_shape_distance; // m
extern int p_smooth_line_num; // 0,none; 1-ego;2-ego&&prior;3- three lane
extern int p_tunnel_use_hd_speed;
extern int p_position_to_split_dist;
extern double p_heading_for_merge_first_level;
extern double p_heading_for_merge_sec_level;
extern double p_heading_for_split_first_level;
extern double p_heading_for_split_sec_level;
extern uint8_t p_mean_filter_window_size;
extern double p_limit_extend_ratio;
extern int p_use_efm_log;
extern int p_nearest_point_offset_point_num;
extern double p_merge_link_centerline_nearest_distance;
extern double p_exit_start_speed_urb_expway;
extern double p_land_end_dist;
