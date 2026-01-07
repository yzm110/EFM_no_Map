/*
 * @Author: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @Date: 2023-12-15 10:35:18
 * @LastEditors: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @LastEditTime: 2024-03-19 01:13:38
 * @FilePath: /repo/code/dx11_noa/application/environmentmodelfunction/include/speed_limit.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "CandidateLanesModel.h"
#include "CommonDataType.h"
#include "CompileConfig.h"
#include "calibration.h"
#include "ExtractRefLine.h"
#include "ScenarioJudgeModel.h"
// #include "topic/topic_trait.h"
#include "common/framework.h"
namespace noa {
namespace environmentmodelfunction {
struct lane_type_speed {
    lane_type_speed() : lane_type_(0), speed_(0), link_dist_(0){};
    ~lane_type_speed() = default;

    uint8_t lane_type_;
    uint8_t speed_;
    uint32_t link_dist_;
};

struct lane_offset_speed {
    lane_offset_speed() : offset_(0), speed_(0){};
    ~lane_offset_speed() = default;

    int32_t offset_;
    uint8_t speed_;
};

struct lane_curv {
    lane_curv() : offset_(0), curv_(0.0){};
    ~lane_curv() = default;

    uint32_t offset_;
    float    curv_;
};

class SpeedLimit {
   public:
    static uint64_t on_ramp_pos_offset_;
    static uint8_t last_cycle_ego_lane_type_;
    static bool on_ramp_lock_start_f_;
    static bool last_cycle_tunnel_f_;
    static bool just_pass_tunnel_f_;
    static uint32_t just_pass_tunnel_offset_;
    static uint8_t last_speed_limit_final_value_;
    static uint8_t last_speed_limit_source_; 
    static bool main_road_use_hd_;
    bool Execute(const message::map_position::s_Position_t& map_position,
                 const earth::shell::framework::TopicTrait::MapMapMsg& map_static_info,
                 const std::unordered_map<uint32_t, int>& link_id_index_lane_info_map,
                 const std::shared_ptr<earth::shell::framework::CandidateLanesModel> candidate_lanes_model,
                 const std::shared_ptr<earth::shell::framework::ScenarioJudgeModel> scenario_judge_model,
                 const std::shared_ptr<earth::shell::framework::ExtractRefLine> extract_ref_line,
                 std::shared_ptr<const earth::shell::framework::TopicTrait::TrafficInfoMsg> traffic_info,
                 std::shared_ptr<const earth::shell::framework::TopicTrait::ComInfo> com_info);

    uint8_t cur_speed() { return cur_speed_; }
    uint8_t next_speed() { return next_speed_; }
    uint16_t pntIdx() { return next_speed_dist_ > (250.0 * 255) ? 255 : static_cast<uint16_t>(next_speed_dist_ / 250.0);}
    std::vector<std::pair<double, double>> exit_scene_speed_limit() { return exit_scene_speed_limit_; }
    void init_global_var();
    uint8_t speed_limit_source(){return speed_limit_source_;}//1-hd;2-sd;3-last value
    
   private:
    bool GetLinkInfos(uint32_t link_id, message::map_map::s_LinkInfo_t& link_infos);
    bool speed_limit(const std::shared_ptr<earth::shell::framework::CandidateLanesModel> candidate_lanes_model,
                     const std::shared_ptr<earth::shell::framework::ExtractRefLine> extract_ref_line);
    void exit_scene(const std::shared_ptr<earth::shell::framework::CandidateLanesModel> candidate_lanes_model,
                    const std::shared_ptr<earth::shell::framework::ScenarioJudgeModel> scenario_judge_model,
                    const std::shared_ptr<earth::shell::framework::ExtractRefLine> extract_ref_line);
    /****get lanetype and speed for map_position_ map_static_info_**/
    bool get_lanetype_and_speed(uint8_t& curlane_spd, uint8_t& cur_lane_type, uint8_t& mainroad_max_spd,
                                uint32_t link_id, uint8_t lane_id, uint32_t& speed_dist, bool is_endoffset, 
                                uint8_t& curlane_next_spd, uint32_t& speed_dist_next);
    bool converte_speed(uint8_t& cur_speed, uint8_t& tar_speed, uint8_t lane_type, uint8_t mainroad_max_spd,
                        uint32_t link_id);
    bool get_toll_link();
    bool get_ref_lane_info(const std::shared_ptr<earth::shell::framework::CandidateLanesModel> candidate_lanes_model,
                           const std::shared_ptr<earth::shell::framework::ExtractRefLine> extract_ref_line,
                           std::vector<lane_type_speed>& lane_type_vec);
    uint8_t converte_lanetype(uint8_t lane_type);
    uint32_t get_before_data_dist(std::vector<lane_type_speed> lane_type_vec, uint32_t next_idx);
    bool get_next_speed(std::vector<lane_type_speed> lane_type_vec, uint8_t& cur_speed, uint8_t& next_speed,
                        double& next_speed_dist);
    bool change_speed_back(std::vector<lane_type_speed>& lane_type_vec, uint8_t next_speed);
    int32_t get_lower_speed_dist(std::vector<lane_type_speed> lane_type_vec, uint32_t begin_idx, uint32_t has_dist);
    bool get_lane_curv(std::vector<lane_curv>& CurvPoints, uint32_t link_id, uint8_t lane_id, bool is_first_link, uint32_t begin_length);
    bool change_speed_for_curv(std::vector<lane_type_speed>& lane_type_vec, std::vector<uint8_t>& speed_change_vec, 
                               std::vector<std::vector<lane_curv>> CurvPoints_list);
    bool insert_lane_type(std::vector<lane_type_speed>& lane_type_vec, lane_type_speed lane_type, 
                          std::vector<lane_curv> CurvPoints, uint8_t max_speed);
    bool converte_curv_speed(float curv, uint8_t& tar_speed, uint8_t max_speed);
    void speed_limmit_sd(const std::shared_ptr<earth::shell::framework::CandidateLanesModel> candidate_lanes_model,
                         const std::shared_ptr<earth::shell::framework::ScenarioJudgeModel> scenario_judge_model,
                         const std::shared_ptr<earth::shell::framework::ExtractRefLine> extract_ref_line);

    void speed_ZTEXT();
    bool smooth_curv_speed(std::vector<uint8_t>& all_speeds, std::vector<int>& all_lengths);
    bool smooth_curv_speed_combine(std::vector<uint8_t>& all_speeds, std::vector<int>& all_lengths, 
                                   int& first_exit, int& second_exit);
    bool smooth_curv_speed_rang(std::vector<uint8_t>& all_speeds, std::vector<int>& all_lengths, 
                                        int& first_exit, int& second_exit, uint8_t speed);
    void GetInsertDist(uint8_t lower_speed, uint32_t& insert_dist,uint8_t& insert_val);
    bool get_speed_change_info(std::vector<lane_type_speed> lane_type_vec, std::vector<uint8_t>& speed_change_vec);
    bool change_speed_in_ramp(std::vector<lane_type_speed>& lane_type_vec);
    uint8_t SpecialScene(const std::shared_ptr<earth::shell::framework::ScenarioJudgeModel> scenario_judge_model, double& start_dist,double& end_dist);
    bool change_speed_for_exit(std::vector<lane_type_speed>& lane_type_vec);
    bool ramp_to_main_road(const uint32_t link_id, const uint8_t ref_lane_id, const uint8_t ego_lane_id, bool& is_ramp_to_main_road);

   private:
    uint8_t cur_speed_;
    uint8_t next_speed_;
    double next_speed_dist_;
    std::set<uint32_t> toll_link_id_set_;
    uint8_t cur_lane_type_;
    uint8_t speed_limit_source_;//1-hd;2-sd;3-last value
    std::vector<lane_type_speed> lane_type_vec_;
    // input
    // std::shared_ptr<const std::vector<uint32_t>> link_id_vec_;
    std::shared_ptr<const message::map_position::s_Position_t> map_position_;
    std::shared_ptr<const earth::shell::framework::TopicTrait::MapMapMsg> map_static_info_;
    std::shared_ptr<const earth::shell::framework::TopicTrait::TrafficInfoMsg> traffic_info_;
    std::shared_ptr<const earth::shell::framework::TopicTrait::ComInfo> com_info_;
    std::shared_ptr<const std::unordered_map<uint32_t, int>>
        link_id_index_lane_info_map_;  // <map_static_info_.LinkInfos.LinkInfos[i].InstanceId, i>, laneInfos search here
    std::vector<std::pair<double, double>> exit_scene_speed_limit_;
    bool is_ramp_to_main_road_;
    std::vector<uint32_t> tar_link_id_vec_;
    uint8_t tar_link_max_speed_;
};

}  // namespace environmentmodelfunction
}  // namespace noa
