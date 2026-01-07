/*
 * @Author: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @Date: 2023-12-15 13:59:30
 * @LastEditors: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @LastEditTime: 2023-12-15 14:09:08
 * @FilePath: /repo82/code/dx11_noa/application/environmentmodelfunction/include/ExtractRefLine.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include "CandidateLanesModel.h"
#include "CommonDataType.h"
#include "CompileConfig.h"
#include "calibration.h"
// #include "topic/topic_trait.h"
#include "common/framework.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <thread>
#include <utility>
#include <vector>
#ifdef USE_PROJ4
#include "coordinate_convert_tool.h"
#endif
#include "CoordinateTool.h"
#include "MapCommonTool.h"

namespace earth {
namespace shell {
namespace framework {

class ExtractRefLine {
   public:
    bool Execute(const message::map_position::s_Position_t& map_position, const TopicTrait::MapMapMsg& map_static_info,
                 const std::unordered_map<uint32_t, int>& link_id_index_lane_info_map,
                 const std::unordered_map<uint32_t, std::vector<int>> link_id_index_lane_connect_map,
                 const std::unordered_map<uint32_t, int> linear_obj_id_map,std::unordered_map<uint64_t, int> curve_index_map,const std::unordered_map<uint64_t, int> lane_widths_map,
                 const std::shared_ptr<CandidateLanesModel> candidate_lanes_model);

    EFMRefLine ego_path() { return ego_path_; }
    EFMRefLine left_path() { return left_path_; }
    EFMRefLine left_left_path() { return left_left_path_; }
    EFMRefLine right_path() { return right_path_; }
    EFMRefLine right_right_path() { return right_right_path_; }
    EFMRefLine emergency_right_path() { return emergency_right_path_; }
    EFMRefLine emergency_left_path() { return emergency_left_path_; }
    int prior_path_index() { return prior_path_index_; }
    double rest_dist() { return rest_dist_; }
    LaneChangeType_e prior_path_change_type() { return prior_path_change_type_; }

    // EFMRefLine ego_path_for_lane_maker() { return ego_path_for_lane_maker_; }
    // EFMRefLine left_path_for_lane_maker() { return left_path_for_lane_maker_; }
    // EFMRefLine right_path_for_lane_maker() { return right_path_for_lane_maker_; }

   private:
    // input
    std::shared_ptr<const message::map_position::s_Position_t> map_position_;
    std::shared_ptr<const TopicTrait::MapMapMsg> map_static_info_;
    std::shared_ptr<const std::unordered_map<uint32_t, int>>
        link_id_index_lane_info_map_;  // <map_static_info_.LinkInfos.LinkInfos[i].InstanceId, i>, laneInfos search here
    std::shared_ptr<const std::unordered_map<uint32_t, std::vector<int>>>
        link_id_index_lane_connect_map_;  //<map_static_info_.LaneConnectivitys.PairConnectivity[i].FromLinkId.FromLinkId,
                                          // index_vec> , lane connect info search here
    std::vector<uint32_t> link_id_vec_;                    // link id list mapping to candidate lanes
    std::vector<double> link_length_vec_;                  // first length is ego to endoffset dist, rest is link length
    std::vector<std::vector<uint8_t>> all_lanes_vec_vec_;  // all possible running lanes
    std::shared_ptr<const std::unordered_map<uint32_t, int>> linear_obj_id_map_;//key,IDLinearObject; val在LinearObjects里的index
    std::shared_ptr<const std::unordered_map<uint64_t, int>> curve_index_map_;//key 高32lane_id,低32 link id; val在LinkCurvatures里的index 
    std::shared_ptr<const std::unordered_map<uint64_t, int>> lane_widths_map_;//key 高32lane_id,低32 link id; val在s_LaneWidths_t里的index

    int ego_lane_prior_index_;
    int left_lane_prior_index_;
    int left_left_lane_prior_index_;
    int right_lane_prior_index_;
    int right_right_lane_prior_index_;
    std::vector<uint32_t> back_ego_link_id_vec_;
    std::vector<uint32_t> back_left_link_id_vec_;
    std::vector<uint32_t> back_left_left_link_id_vec_;
    std::vector<uint32_t> back_right_link_id_vec_;
    std::vector<uint32_t> back_right_right_link_id_vec_;
    std::vector<uint8_t> back_ego_lane_num_vec_;
    std::vector<uint8_t> back_left_lane_num_vec_;
    std::vector<uint8_t> back_left_left_lane_num_vec_;
    std::vector<uint8_t> back_right_lane_num_vec_;
    std::vector<uint8_t> back_right_right_lane_num_vec_;
    std::vector<uint32_t> back_emergency_left_link_id_vec_;
    std::vector<uint32_t> back_emergency_right_link_id_vec_;
    std::vector<uint8_t> back_emergency_left_lane_num_vec_;
    std::vector<uint8_t> back_emergency_right_lane_num_vec_;
    std::vector<std::vector<uint8_t>> all_emergency_lanes_vec_vec_;  // all possible running lanes

    bool GetLinkInfos(uint32_t want_link_id, message::map_map::s_LinkInfo_t& link_infos);
    bool GetLine(const uint32_t line_index, std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,
                 uint8_t& mrk_type, uint8_t& mrk_color);
    bool GenerateOnePathLine(const std::vector<uint8_t>& candidate_lane, const std::vector<uint32_t>& link_id_vec,
                             const std::vector<uint32_t>& back_link_id_vec,
                             const std::vector<uint8_t>& back_lane_num_vec, EFMRefLine& path);
    //bool RerangeLaneMarkerSection(EFMRefLine& path);
    bool MakePath(double ego_utm_x, double ego_utm_y, EFMRefLine& path);

    bool GenerateOnePathAttribute(const std::vector<uint8_t>& candidate_lane, const std::vector<uint32_t>& link_id_vec,
                                  EFMRefLine& path);
    bool GeneratePaths(int ego_lane_prior_index, int left_lane_prior_index, int left_left_lane_prior_index,
                       int right_lane_prior_index, int right_right_lane_prior_index,int32_t emergency_left_index,int32_t emergency_right_index,
                       EFMRefLine& ego_path, EFMRefLine& left_path, EFMRefLine& left_left_path,
                       EFMRefLine& right_path, EFMRefLine& right_right_path,EFMRefLine& emergency_left_path, EFMRefLine& emergency_right_path);
    bool GeneratePathsAttribute(EFMRefLine& ego_path, EFMRefLine& left_path, EFMRefLine& left_left_path, EFMRefLine& right_path, 
                                EFMRefLine& right_right_path);
    bool GeneratePathsAttributeOne(std::vector<uint8_t>& lanes, EFMRefLine& path, int lane_prior_index);
    bool FixPathDensity(const EFMRefLinePoints& raw_reference_line, double sample_distance, EFMRefLinePoints& fixed_reference_line);
    bool GetLaneCenterLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                           uint32_t& center_line_index);
    bool GetLaneLeftLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                         uint32_t& line_index);
    bool GetLaneRightLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                          uint32_t& line_index);
    bool GetLaneType(uint32_t link_id, uint8_t lane_num, uint8_t& lane_type);

    void CalPriorIndex(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model, const EFMRefLine& ego_path);
    bool FixMarkrDensity(const EFMRefLineMarkr& raw_reference_line, double sample_distance,EFMRefLineMarkr& fixed_reference_line);

    void CutLineToFixRange(EFMRefLine& ego_path, EFMRefLine& left_path, EFMRefLine& right_path);

    bool GetStartEndIndexInRange(double start_s, double end_s, const EFMRefLineMarkr& lane_markr, int& start_index,
                                 int& end_index);

    bool GetStartEndIndexInRange(double start_s, double end_s, const EFMRefLinePoints& line_points, int& start_index,
                                 int& end_index);

    bool SetPathAvailableOne(int32_t side_lane_idx, EFMRefLine& side_path, LaneElementGroupSets lane_element_group_sets);
    bool SetPathAvailable(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model,
                          EFMRefLine& ego_path, EFMRefLine& left_path, EFMRefLine& left_left_path, 
                          EFMRefLine& right_path, EFMRefLine& right_right_path);

    std::pair<std::array<double, 5>, std::array<double, 5>> rotatedRect(double x, double y, double half_width,
                                                                        double half_height, double angle);
    
    bool SetRefPath(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model);

    bool AlterPointYForSplit(EFMRefLinePoints& line_points, int32_t split_point_number, 
                             int32_t split_lane_index, int32_t next_split_point_number,
                             int32_t next_split_lane_index, std::vector<uint8_t> left_markr_section, 
                             std::vector<uint8_t> right_markr_section,
                             std::vector<std::pair<double, int>> curvature_nums_,
                             std::vector<double> line_offsets,
                             uint8_t split_from,
                             int back_lane_split_point_index,
                             bool back_lane_is_split);

    bool AlterPointYForSplitPart(EFMRefLinePoints& line_points, int32_t s_point_number, int32_t e_point_number);

    bool GetSideLine(std::vector<uint8_t>& split_line, 
                                 std::vector<std::pair<double, int>> curvature_nums_, 
                                 std::vector<uint8_t> left_markr_section,
                                 std::vector<uint8_t> right_markr_section,
                                 int32_t split_lane_index,
                                 int32_t next_split_lane_index,
                                 std::vector<double> line_offsets,
                                 std::vector<double>& split_line_offsets,
                                 uint8_t split_from);

    bool GetVirtuallyLineLength(double& Virtually_line_length, std::vector<uint8_t> markr_section,
                                std::vector<double> line_offsets);
    bool GetEndAnchorPoint(EFMRefLinePoints& line_points, int32_t split_point_number, 
                           double Virtually_line_length, int32_t& anchor_point_number);
    bool GetLaneCurvature(const std::vector<uint8_t>& candidate_lane, const std::vector<uint32_t>& link_id_vec,
                          EFMRefLine& path);
    bool GetLaneTrans(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, uint8_t& trans);
    bool GetSplitFrom(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, int& split_from);
    bool SaperateLaneMarkerIntoTwoPart(const EFMRefLineMarkr& marker_raw, EFMPoint tar_point, 
                                       const std::vector<uint32_t>& mkr_offset,uint32_t ego_pos_offset,EFMRefLineMarkrSection& marker_section);
    bool SaperateLaneCenterLineIntoTwoPart(const EFMRefLinePoints& marker_raw, EFMPoint tar_point, 
                                           const std::vector<uint32_t>& pts_offset,uint32_t ego_pos_offset,EFMRefLinePointsSection& marker_section);

    void GetLaneSplitStartS(uint32_t ego_offset, uint32_t link_id, uint8_t lane_num,
                            const message::map_map::s_LinkInfo_t& link_infos, uint32_t next_link_id,std::vector<std::pair<double,double>>& split_start_s_vec);
    void GetLaneMergeEndS(uint32_t ego_offset, uint32_t link_id, uint8_t lane_num,
                          const message::map_map::s_LinkInfo_t& link_infos, std::vector<std::pair<double,double>>& merge_end_s_vec);
    bool GetLaneSplitSmoothInfo(uint32_t ego_offset, uint32_t link_id, uint8_t lane_num,
                                        std::vector<uint32_t> next_link_id_vec, std::vector<uint8_t> next_lane_id_vec, const std::vector<uint32_t>& last_link_id_vec, const std::vector<uint8_t>& last_lane_id_vec,
                                        SmoothSplitInfo& smooth_split_info);
    void PlotSmoothSplitInfo(std::string path_name, const std::vector<SmoothSplitInfo>& smooth_split_info);
    uint8_t GetRoadClass(std::shared_ptr<const TopicTrait::MapMapMsg> map_msg, uint32_t link_id);
    // output
    EFMRefLinePoints ego_path_points_, left_path_points_, left_left_path_points_, right_path_points_, right_right_path_points_;  // prior path fixed points, 2.5m interval, 300m ahead
    EFMRefLine ego_path_, left_path_, left_left_path_, right_path_, right_right_path_, emergency_right_path_, emergency_left_path_;
    int prior_path_index_;
    double rest_dist_;
    LaneChangeType_e prior_path_change_type_;
    // EFMRefLine ego_path_for_lane_maker_, left_path_for_lane_maker_, right_path_for_lane_maker_;
};

}  // namespace framework
}  // namespace shell
}  // namespace earth
