/*
 * @Author: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @Date: 2023-12-14 15:31:15
 * @LastEditors: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @LastEditTime: 2024-04-29 10:26:57
 * @FilePath: /repo/code/dx11_noa/application/environmentmodelfunction/include/ScenarioJudgeModel.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include "CandidateLanesModel.h"
#include "CompileConfig.h"
// #include "topic/topic_trait.h"
#include "common/framework.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>
#include <unordered_set>

#include "CommonDataType.h"
#include "calibration.h"
#include "ExtractRefLine.h"
#include "MapCommonTool.h"

namespace earth {
namespace shell {
namespace framework {

class ScenarioJudgeModel {
   public:
    // enum LanePos : uint8_t { EGO_LANE = 0, LEFT_LANE = 1, RIGHT_LANE = 2 };

    enum Scenario : uint8_t { SCEN_DEFAULT = 0, ON_RAMP, OFF_RAMP, IN_RAMP };

    ScenarioJudgeModel() : entry_dist_(0.0), exit_dist_(0.0){};

    bool Execute(const int cur_rp_index, const std::shared_ptr<CandidateLanesModel> candidate_lanes_model,const std::shared_ptr<ExtractRefLine> extract_ref_line,
                 const message::map_position::s_Position_t& map_position,
                 const TopicTrait::MapRouteListMsg& map_route_list, const TopicTrait::MapMapMsg& map_static_info,
                 const std::unordered_map<uint32_t, int>& link_id_index_lane_info_map,
                 const std::unordered_map<uint32_t, std::vector<int>>& link_id_index_lane_connect_map,
                 const std::unordered_map<uint32_t, std::vector<int>>& to_link_id_index_lane_connect_map);

    double entry_dist() { return entry_dist_; }
    double entry_end_dist() { return entry_end_dist_; }
    double entry_start_dist() { return entry_start_dist_; }
    double exit_dist() { return exit_dist_; }

    double exit_start_dist() { return exit_start_dist_; }
    double exit_end_dist() { return exit_end_dist_; }

    uint32_t odd_end_dist() { return odd_end_dist_; }
    uint8_t in_odd_type() { return in_odd_type_; }
    uint8_t out_odd_reason() { return out_odd_reason_; }
    double dDistanceByPassMerge() { return dDistanceByPassMerge_; }

    std::vector<GeoFenceSegment> geofence_segments() { return geofence_segments_; }
    std::vector<GeoFenceSegment> valid_geofence_segments() { return valid_geofence_segments_; }
    int exit_start_link_index() { return exit_start_link_index_; }
    int exit_end_link_index() { return exit_end_link_index_; }
    uint8_t exit_end_speed_limit() { return exit_end_speed_limit_; }
    bool is_has_big_curvature(){return is_has_big_curvature_;}
    uint8_t exit_type(){return exit_type_;}
    void InitGlobalVar();
    bool is_has_virtual_lane() { return is_has_virtual_lane_; }
    bool is_y_shape_link() { return is_y_shape_link_; }
    bool is_ramp_to_main_road() { return is_ramp_to_main_road_; }
    uint8_t left_lane_spd_limit() { return left_lane_spd_limit_; }
    std::vector<uint32_t> last_merge_back_links_id() { return last_merge_back_links_id_; }
    std::vector<uint8_t> last_merge_ego_back_lanes_id() { return last_merge_ego_back_lanes_id_; }
    std::vector<uint8_t> last_merge_side_back_lanes_id() { return last_merge_side_back_lanes_id_; }
    uint8_t exit_start_speed_limit_no_connect(){return exit_start_speed_limit_no_connect_;}
    uint8_t exit_start_lane_type_no_connect(){return exit_start_lane_type_no_connect_;}
    uint8_t exit_start_road_class(){return exit_start_road_class_;}
    std::vector<uint32_t> tar_link_list(){return tar_link_list_;}
    uint8_t tar_link_max_speed(){return tar_link_max_speed_;}
    uint32_t road_split_end_link_id(){return road_split_end_link_id_;}//end
    int road_split_end_link_index(){return road_split_end_link_index_;}//end
    uint8_t road_split_lane_num_continue(){return road_split_lane_num_continue_;}
    uint8_t road_split_lane_num_split(){return road_split_lane_num_split_;}
    double road_split_end_dist(){return road_split_end_dist_;}
    double road_split_start_dist(){return road_split_start_dist_;}//一条主路分成两条主路
    uint8_t road_split_start_lane_num(){return road_split_start_lane_num_;}
    uint32_t road_split_start_link_id(){return road_split_start_link_id_;}
    bool main_split_two_main(){return main_split_two_main_;}

    static uint32_t static_last_link_id_;
    static bool is_has_big_curvature_;
    static uint32_t big_curve_position_offset_;
    static bool is_exit_start_dist_f_;//used to log whether the exit start occured
    static double last_cycle_exit_end_dist_;//used to log whether the exit start occured
    static std::vector<uint32_t> tollgate_back_linkids_;
    static std::map<uint32_t, uint8_t> main_road_nearby_ramp_lanenum_;
    static std::vector<uint32_t> last_merge_back_links_id_;
    static std::vector<uint8_t> last_merge_ego_back_lanes_id_;
    static std::vector<uint8_t> last_merge_side_back_lanes_id_;

   private:
    // input
    int cur_rp_index_;
    // std::shared_ptr<const std::vector<uint32_t>> link_id_vec_;
    // std::shared_ptr<const std::vector<std::vector<uint8_t>>> candidate_lanes_vec_vec_;
    std::shared_ptr<const message::map_position::s_Position_t> map_position_;
    std::shared_ptr<const TopicTrait::MapRouteListMsg> map_route_list_;
    std::shared_ptr<const TopicTrait::MapMapMsg> map_static_info_;
    std::shared_ptr<const std::unordered_map<uint32_t, int>>
        link_id_index_lane_info_map_;  // <map_static_info_.LinkInfos.LinkInfos[i].InstanceId, i>, laneInfos search here
    std::shared_ptr<const std::unordered_map<uint32_t, std::vector<int>>>
        link_id_index_lane_connect_map_;  //<map_static_info_.LaneConnectivitys.PairConnectivity[i].FromLinkId.FromLinkId,
                                          // index_vec> , lane connect info search here
    std::shared_ptr<const std::unordered_map<uint32_t, std::vector<int>>> to_link_id_index_lane_connect_map_;
    //<map_static_info_.LaneConnectivitys.PairConnectivity[i].ToLinkId.ToLinkId,
    // index_vec> , lane connect info search here

    // local
    // bool ExitSceneByLaneSize();
    bool EntrySceneByLaneSize();
    void EntrySceneByCandidateLanes(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model);
    bool ExitSceneByLink(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model, int cur_rp_index);
    // bool ExitSceneBackup();
    bool GetRightestAvailableLaneNum(uint32_t search_link_id, uint8_t& righest_lane_num);
    bool StoreLastLinkId();
    bool ODDSceneJudge();
    bool EnterGeoFence();
    bool ExtractValidGeoFenceSegments();
    bool ByPassMerge(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model);
    bool IsByPassLink(uint32_t link_id, uint32_t& from_link_id1, uint32_t& from_link_id2);
    bool IsSplitLink(uint32_t linkid);
    bool LinkHasSplitLane(uint32_t linkid);
    bool YShapeLink();
    bool GetLinkInfos(uint32_t link_id, message::map_map::s_LinkInfo_t& link_infos);
    bool GetLinkMinSpeedLimit(uint32_t link_id, uint8_t& speed_limit);
    bool MainRoadWideLane(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model, const std::shared_ptr<ExtractRefLine> extract_ref_line);
    bool BigCurvature(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model);
    bool DirectExitScene(double exit_start_dist, double exit_end_dist);
    bool GetLineType(const message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, uint8_t& line_type, bool is_left, uint32_t& line_id);
    bool RampToMainRoad(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model);
    void RampToMainRoadToRamp(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model);
    bool GetBackRoutelistLinks();
    void IsVirtuallyLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t& left_lane_id, const uint8_t& right_lane_id, bool is_left, uint8_t& virtual_line_case);
    bool GetLaneCenterLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                           uint32_t& center_line_index);
    bool GetLaneLeftLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                         uint32_t& line_index);
    bool GetLaneRightLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                          uint32_t& line_index);
    bool GetLineGeometry(uint32_t line_id, std::vector<message::map_map::s_GeometryPoint_t>& geometry_points);
    bool IsVirtualLineMerge(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model);
    static bool CompareDis(const BigWidthLane& a, const BigWidthLane& b);  

    void GetRoadClass(std::shared_ptr<const TopicTrait::MapMapMsg> map_msg, uint32_t link_id,uint8_t& road_class);
    bool MainRoadExitSceneByLink(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model,int cur_rp_index);

    double entry_dist_;      //
    double entry_end_dist_;  //
    double entry_start_dist_;
    double exit_dist_;
    double exit_start_dist_;
    uint8_t exit_start_speed_limit_no_connect_;
    uint8_t exit_start_lane_type_no_connect_;
    double exit_end_dist_;
    uint32_t odd_end_dist_;
    std::vector<uint32_t> tollgate_links_;
    uint8_t in_odd_type_;
    uint8_t out_odd_reason_;
    bool is_has_virtual_lane_;
    std::vector<GeoFenceSegment> geofence_segments_;
    std::vector<GeoFenceSegment> valid_geofence_segments_;
    // static bool history_find_entry_on_route_f_;
    Scenario current_scene_;
    int exit_start_link_index_;
    int exit_end_link_index_;
    double dDistanceByPassMerge_;
    bool is_y_shape_link_;
    uint8_t exit_end_speed_limit_;  // exit_end_link_index_+1 's link's lane in maxval
    uint8_t exit_type_; //1-direct off ramp scene
    bool is_ramp_to_main_road_;
    uint8_t left_lane_spd_limit_;
    std::vector<uint32_t> back_routelist_links_;
    double ego_utm_x_;
    double ego_utm_y_;
    double ego_wgs84_x_;
    double ego_wgs84_y_;
    uint8_t exit_start_road_class_;
    static std::vector<uint32_t> tar_link_list_;
    static uint32_t tar_link_max_endoffset_;
    static uint8_t tar_link_max_speed_;
    uint32_t road_split_end_link_id_;//end
    int road_split_end_link_index_;//end
    uint8_t road_split_lane_num_continue_;
    uint8_t road_split_lane_num_split_;
    double road_split_end_dist_;
    double road_split_start_dist_;//一条主路分成两条主路
    uint8_t road_split_start_lane_num_;
    uint32_t road_split_start_link_id_;
    bool main_split_two_main_;
};

}  // namespace framework
}  // namespace shell
}  // namespace earth
