/*
 * @Author: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @Date: 2023-12-29 18:36:59
 * @LastEditors: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @LastEditTime: 2024-04-29 10:21:29
 * @FilePath: /repo83/code/dx11_noa/application/environmentmodelfunction/src/efm_config.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "efm_config.h"
#include "store/yaml.h"
// #include "topic/topic_trait.h"
#include "common/framework.h"

void EfmConfig(const std::string file_name) {
#ifdef __QNX__
    std::string path = "/mnt/appfs/JICA/noa/conf" + file_name;
#else
    std::string path = "/home/yzm/code910/dx11_noa/application/environmentmodelfunction/deps" + file_name;
#endif
    earth::mantle::store::yaml::Node config;
    earth::mantle::store::yaml::parse(config, path.c_str());

    try {
        p_efm_use_centerline = config["p_efm_use_centerline"].As<int>();
        p_odd_to2_distance = config["p_odd_to2_distance"].As<uint32_t>();
        p_odd_to0_distance = config["p_odd_to0_distance"].As<uint32_t>();
        p_bigcurve_oddto2_distance = config["p_bigcurve_oddto2_distance"].As<double>();
        p_bigcurve_oddto0_distance = config["p_bigcurve_oddto0_distance"].As<double>();
        p_bigcurve_odd_radius = config["p_bigcurve_odd_radius"].As<float>();
        p_lastlink_back_search_distance = config["p_lastlink_back_search_distance"].As<uint16_t>();
        p_bigcurve_odd_abnomal_radius = config["p_bigcurve_odd_abnomal_radius"].As<float>();
        p_curv_point_dis_min = config["p_curv_point_dis_min"].As<uint32_t>();
        p_curv_point_dis_max = config["p_curv_point_dis_max"].As<uint32_t>();
        p_lane_width_max = config["p_lane_width_max"].As<uint16_t>();
        p_narrow_lane_width = config["p_narrow_lane_width"].As<uint32_t>();
        p_kerb_back_distance = config["p_kerb_back_distance"].As<double>();
        p_kerb_forward_distance = config["p_kerb_forward_distance"].As<double>();
        p_exit_start_end_gap = config["p_exit_start_end_gap"].As<double>();
        p_efm_use_pointforsplit = config["p_efm_use_pointforsplit"].As<uint32_t>();
        p_service_odd_to0_distance = config["p_service_odd_to0_distance"].As<double>();
        p_toll_odd_to0_distance = config["p_toll_odd_to0_distance"].As<double>();
        p_tollback_bigcurv_distance = config["p_tollback_bigcurv_distance"].As<uint32_t>();
        p_tollahead_bigcurv_distance = config["p_tollahead_bigcurv_distance"].As<uint32_t>();
        p_bigwidth_forward_distance  = config["p_bigwidth_forward_distance"].As<uint32_t>();// m
        p_bigwidth_back_distance  = config["p_bigwidth_back_distance"].As<uint32_t>(); // m
        p_back_center_line_sample_dist = config["p_back_center_line_sample_dist"].As<double>();
        p_front_center_line_sample_dist = config["p_front_center_line_sample_dist"].As<double>();
        p_back_side_line_sample_dist  = config["p_back_side_line_sample_dist"].As<double>();// m
        p_front_side_line_sample_dist  = config["p_front_side_line_sample_dist"].As<double>(); // m
        refline_smooth_config.weight_fem_pos_deviation = config["weight_fem_pos_deviation"].As<double>();
        refline_smooth_config.weight_path_length = config["weight_path_length"].As<double>();
        refline_smooth_config.weight_ref_deviation = config["weight_ref_deviation"].As<double>();
        refline_smooth_config.curvature_constraint = config["curvature_constraint"].As<double>();
        refline_smooth_config.bounds_val = config["bounds_val"].As<double>();
        refline_smooth_config.severe_bend_kappa_up_limt = config["severe_bend_kappa_up_limt"].As<double>();
        refline_smooth_config.severe_bend_kappa_down_limt = config["severe_bend_kappa_down_limt"].As<double>();
        refline_smooth_config.severe_bend_bounds_val = config["severe_bend_bounds_val"].As<double>();
        refline_smooth_config.severe_bend_s_front = config["severe_bend_s_front"].As<double>();
        refline_smooth_config.severe_bend_s_back = config["severe_bend_s_back"].As<double>();
        refline_smooth_config.s_shape_kappa_up_limt = config["s_shape_kappa_up_limt"].As<double>();
        refline_smooth_config.s_shape_kappa_down_limt = config["s_shape_kappa_down_limt"].As<double>();
        refline_smooth_config.s_shape_bounds_val = config["s_shape_bounds_val"].As<double>();
        refline_smooth_config.s_shape_s_front = config["s_shape_s_front"].As<double>();
        refline_smooth_config.s_shape_s_back = config["s_shape_s_back"].As<double>();
        refline_smooth_config.p_short_continuous_split_length_s = config["p_short_continuous_split_length_s"].As<double>();
        refline_smooth_config.p_short_continuous_split_bounds_val_factor = config["p_short_continuous_split_bounds_val_factor"].As<double>();
        refline_smooth_config.p_short_continuous_split_front_s = config["p_short_continuous_split_front_s"].As<double>();
        refline_smooth_config.p_merge_gap_to_lane_marker = config["p_merge_gap_to_lane_marker"].As<double>();
        refline_smooth_config.p_straight_split_bounds_val_factor = config["p_straight_split_bounds_val_factor"].As<double>();
        p_offramp_speed_lmt_dist = config["p_offramp_speed_lmt_dist"].As<double>();
        p_tunnel_speed_lmt_front_dist = config["p_tunnel_speed_lmt_front_dist"].As<double>();
        p_tunnel_speed_lmt_keep_dist = config["p_tunnel_speed_lmt_keep_dist"].As<double>(); 
        p_toll_speed_lmt_front_dist = config["p_toll_speed_lmt_front_dist"].As<double>(); 
    	p_tollback_odd_distance = config["p_tollback_odd_distance"].As<uint32_t>();
	p_y_shape_distance  = config["p_y_shape_distance"].As<uint32_t>(); // m
        p_smooth_line_num = config["p_smooth_line_num"].As<int>();
        refline_smooth_config.p_merge_front_s = config["p_merge_front_s"].As<double>(); 
        refline_smooth_config.p_merge_back_s = config["p_merge_back_s"].As<double>(); 
        refline_smooth_config.p_split_front_s = config["p_split_front_s"].As<double>(); 
        refline_smooth_config.p_split_back_s = config["p_split_back_s"].As<double>(); 
        refline_smooth_config.p_split_merge_bounds_val_factor = config["p_split_merge_bounds_val_factor"].As<double>(); 
        refline_smooth_config.p_continuous_split_length_s = config["p_continuous_split_length_s"].As<double>(); 
        refline_smooth_config.p_continuous_split_bounds_val_factor = config["p_continuous_split_bounds_val_factor"].As<double>(); 
        refline_smooth_config.p_curve_take_line_thred = config["p_curve_take_line_thred"].As<double>(); 
        p_tunnel_use_hd_speed  = config["p_tunnel_use_hd_speed"].As<int>(); // m
        p_position_to_split_dist = config["p_position_to_split_dist"].As<int>(); // cm
        p_heading_for_merge_first_level = config["p_heading_for_merge_first_level"].As<double>(); 
        p_heading_for_merge_sec_level = config["p_heading_for_merge_sec_level"].As<double>();
        p_heading_for_split_first_level = config["p_heading_for_split_first_level"].As<double>(); 
        p_heading_for_split_sec_level = config["p_heading_for_split_sec_level"].As<double>();
        p_mean_filter_window_size = config["p_mean_filter_window_size"].As<int>(); 
        p_limit_extend_ratio = config["p_limit_extend_ratio"].As<double>();  
        p_use_efm_log = config["p_use_efm_log"].As<int>(); 
        p_nearest_point_offset_point_num = config["p_nearest_point_offset_point_num"].As<int>(); 
        p_exit_start_speed_urb_expway = config["p_exit_start_speed_urb_expway"].As<double>();
        p_land_end_dist = config["p_land_end_dist"].As<double>();      
} catch (const std::exception &e) {
        std::cerr << "YAML Exception: " << e.what() << std::endl;
    }
}
