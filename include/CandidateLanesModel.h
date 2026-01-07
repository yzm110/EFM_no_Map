/*
 * @Description:
 * @Author: LiuXiaoFeng
 * @Date: 2023-11-09 11:55:04
 * @LastEditTime: 2023-11-10 13:17:40
 * @LastEditors: LiuXiaoFeng
 */
#pragma once
#include "AlgoRefLaneNode.h"
#include "CommonDataType.h"
#include "CommonMathMethod.h"
#include "CompileConfig.h"
#include "CoordinateTool.h"
#include "MapCommonTool.h"
// #include "topic/topic_trait.h"
#include "common/framework.h"
#include "efm_log.h"
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>
#include <deque>

namespace earth {
namespace shell {
namespace framework {
class CandidateLanesModel {
   public:
    static std::map<uint64_t,uint8_t> split_lind_id_lane_id_map;//key: linkid低32位 laneid 高32位； 保存的是修改了split属性的属性
    static std::map<uint64_t,uint8_t> merge_lind_id_lane_id_map;//key: linkid低32位 laneid 高32位； 保存的是修改了split属性的属性
    static std::deque<uint64_t> split_deque;//first-保存的所map的key; 
    static std::deque<uint64_t> merge_deque;//first-保存的所map的key;
    static std::map<uint64_t,SplitInfo_S> split_info_map; //key:linkid低32位 laneid 高32位； 
    static std::deque<uint64_t> split_info_deque; //和split_info_map配套，保存的是split_info_map的key； 
   static std::map<uint64_t,uint8_t> drive_lind_id_lane_id_map;//key: linkid低32位 laneid 高32位； 保存的是自车走过的link id和link id; second-是path id
   static std::deque<uint64_t> drive_deque;//保存的是drive_lind_id_lane_id_map的key; 

    enum LaneExtraInfoType_e : uint8_t { TRANSIT = 0 };

    // enum LanePos : uint8_t { EGO_LANE = 0, LEFT_LANE = 1, RIGHT_LANE = 2 };

    bool Execute(const message_common::c2c_message::s_EgoMotion2_t& ego_motion,
                 const message::map_position::s_Position_t& map_position,
                 const TopicTrait::MapRouteListMsg& map_route_list, const TopicTrait::MapMapMsg& map_static_info,
                 const std::unordered_map<uint32_t, int>& link_id_index_lane_info_map,
                 const std::unordered_map<uint32_t, std::vector<int>>& link_id_index_lane_connect_map,
                 const std::unordered_map<uint32_t, std::vector<int>>& to_link_id_index_lane_connect_map,
                 const std::unordered_map<uint64_t, int>& curve_index_map,
                 const std::unordered_map<uint32_t, int>& linear_obj_id_map);

    std::vector<uint32_t> link_id_vec() { return link_id_vec_; }
    std::vector<std::vector<uint8_t>> all_lanes_vec_vec() { return all_lanes_vec_vec_; }
    // std::vector<std::vector<uint8_t>> candidate_lanes_vec_vec() { return candidate_lanes_vec_vec_; }
    int ego_lane_prior_index() { return ego_lane_prior_index_; }
    int left_lane_prior_index() { return left_lane_prior_index_; }
    int left_left_lane_prior_index() { return left_left_lane_prior_index_; }
    int right_lane_prior_index() { return right_lane_prior_index_; }
    int right_right_lane_prior_index() { return right_right_lane_prior_index_; }
    size_t ego_lane_prior_length() { return ego_lane_prior_length_; }
    size_t left_lane_prior_length() { return left_lane_prior_length_; }
    size_t left_left_lane_prior_length() { return left_left_lane_prior_length_; }
    size_t right_lane_prior_length() { return right_lane_prior_length_; }
    size_t right_right_lane_prior_length() { return right_right_lane_prior_length_; }
    double ego_lane_prior_dist() { return ego_lane_prior_dist_; }
    double left_lane_prior_dist() { return left_lane_prior_dist_; }
    double left_left_lane_prior_dist() { return left_left_lane_prior_dist_; }
    double right_lane_prior_dist() { return right_lane_prior_dist_; }
    double right_right_lane_prior_dist() { return right_right_lane_prior_dist_; }
    std::vector<uint8_t> ego_path_lane_num() { return ego_path_lane_num_; }
    std::vector<uint8_t> left_path_lane_num() { return left_path_lane_num_; }
    std::vector<uint8_t> left_left_path_lane_num() { return left_left_path_lane_num_; }
    std::vector<uint8_t> right_path_lane_num() { return right_path_lane_num_; }
    std::vector<uint8_t> right_right_path_lane_num() { return right_right_path_lane_num_; }
    std::vector<uint8_t> ref_route_path_lane_num();
    std::vector<bool> is_contain_split_vec() { return is_contain_split_vec_; }
    std::map<uint32_t, SplitNode> candidate_lanes_split_nodes() { return candidate_lanes_split_nodes_; }
    std::vector<uint32_t> back_ego_link_id_vec() { return back_ego_link_id_vec_; }
    std::vector<uint32_t> back_left_link_id_vec() { return back_left_link_id_vec_; }
    std::vector<uint32_t> back_left_left_link_id_vec() { return back_left_left_link_id_vec_; }
    std::vector<uint32_t> back_right_link_id_vec() { return back_right_link_id_vec_; }
    std::vector<uint32_t> back_right_right_link_id_vec() { return back_right_right_link_id_vec_; }
    std::vector<uint8_t> back_ego_lane_num_vec() { return back_ego_lane_num_vec_; }
    std::vector<uint8_t> back_left_lane_num_vec() { return back_left_lane_num_vec_; }
    std::vector<uint8_t> back_left_left_lane_num_vec() { return back_left_left_lane_num_vec_; }
    std::vector<uint8_t> back_right_lane_num_vec() { return back_right_lane_num_vec_; }
    std::vector<uint8_t> back_right_right_lane_num_vec() { return back_right_right_lane_num_vec_; }
    std::vector<double> link_length_vec() { return link_length_vec_; }
    std::vector<uint16_t> ego_lane_width_min_vec() { return ego_lane_width_min_vec_; }
    std::vector<uint16_t> ego_lane_width_max_vec() { return ego_lane_width_max_vec_; }
    std::vector<uint16_t> left_lane_width_min_vec() { return left_lane_width_min_vec_; }
    std::vector<uint16_t> left_lane_width_max_vec() { return left_lane_width_max_vec_; }
    std::vector<uint16_t> left_left_lane_width_min_vec() { return left_left_lane_width_min_vec_; }
    std::vector<uint16_t> left_left_lane_width_max_vec() { return left_left_lane_width_max_vec_; }
    std::vector<uint16_t> right_lane_width_min_vec() { return right_lane_width_min_vec_; }
    std::vector<uint16_t> right_lane_width_max_vec() { return right_lane_width_max_vec_; }
    std::vector<uint16_t> right_right_lane_width_min_vec() { return right_right_lane_width_min_vec_; }
    std::vector<uint16_t> right_right_lane_width_max_vec() { return right_right_lane_width_max_vec_; }
    uint8_t driveable_lane_size() { return driveable_lane_size_; }
    uint8_t right_not_driveable_lane_size() { return right_not_driveable_lane_size_; }
    uint8_t min_driveable_lane_num() { return min_driveable_lane_num_; }
    uint8_t max_driveable_lane_num() { return max_driveable_lane_num_; }
    LaneElementGroupSets lane_element_group_sets() { return lane_element_group_sets_; }
    // int ego_lane_index_for_lane_mkr() { return ego_lane_index_for_lane_mkr_; }
    // int left_lane_index_for_lane_mkr() { return left_lane_index_for_lane_mkr_; }
    // int right_lane_index_for_lane_mkr() { return right_lane_index_for_lane_mkr_; }
    message::efm::s_NodeInfo_t GetNOdeINfo() { return NodeInfo_; };
    int get_ref_lane_prior_index() { return ref_lane_prior_index_; };
    LaneElementGroupSets get_lane_element_group_sets() { return lane_element_group_sets_; };
    int get_ref_lane_split_position() { return ref_lane_split_position_; };
    int get_ref_lane_next_split_position() { return ref_lane_next_split_position_; };
    uint8_t get_ref_lane_split_from() { return ref_lane_split_from_; };
    uint8_t fixed_lane_id() { return fixed_lane_id_; }
    std::vector<LaneExtraInfo_s> get_left_extra_info() {return left_lane_prior_index_ >=0 ? 
                                                              all_lanes_extra_vec_vec_[left_lane_prior_index_] : 
                                                              std::vector<LaneExtraInfo_s> {};}
    std::vector<LaneExtraInfo_s> get_ego_extra_info() {return ego_lane_prior_index_ >=0 ? 
                                                              all_lanes_extra_vec_vec_[ego_lane_prior_index_] : 
                                                              std::vector<LaneExtraInfo_s> {};}
    std::vector<LaneExtraInfo_s> get_right_extra_info() {return right_lane_prior_index_ >=0 ? 
                                                              all_lanes_extra_vec_vec_[right_lane_prior_index_] : 
                                                              std::vector<LaneExtraInfo_s> {};}
    std::vector<LaneExtraInfo_s> get_left_left_extra_info() {return left_left_lane_prior_index_ >=0 ? 
                                                              all_lanes_extra_vec_vec_[left_left_lane_prior_index_] : 
                                                              std::vector<LaneExtraInfo_s> {};}
    std::vector<LaneExtraInfo_s> get_right_right_extra_info() {return right_right_lane_prior_index_ >=0 ? 
                                                              all_lanes_extra_vec_vec_[right_right_lane_prior_index_] : 
                                                              std::vector<LaneExtraInfo_s> {};}
    std::vector<uint32_t> merge_back_links_id(){return merge_back_links_id_;}
    std::vector<uint8_t> merge_ego_back_lanes_id(){return merge_ego_back_lanes_id_;}
    std::vector<uint8_t> merge_side_back_lanes_id(){return merge_side_back_lanes_id_;}
    void InitGlobalVal();

    std::map<uint64_t,uint8_t> get_split_lind_id_lane_id_map() { return split_lind_id_lane_id_map; }
    bool GetStoredSplitInfo(const std::map<uint64_t,SplitInfo_S>& map_res, const std::deque<uint64_t>& deque_res, uint8_t lane_id, uint32_t link_id, SplitInfo_S& split_info);
    
    std::vector<uint32_t> pre_cutin_merge_offset_vec(){return pre_cutin_merge_offset_vec_;}

    std::vector<uint32_t> back_emergency_left_link_id_vec() { return back_emergency_left_link_id_vec_; }
    std::vector<uint32_t> back_emergency_right_link_id_vec() { return back_emergency_right_link_id_vec_; }
    std::vector<uint8_t> back_emergency_left_lane_num_vec() { return back_emergency_left_lane_num_vec_; }
    std::vector<uint8_t> back_emergency_right_lane_num_vec() { return back_emergency_right_lane_num_vec_; }
    int32_t emergency_left_index() { return emergency_left_index_; }
    int32_t emergency_right_index() { return emergency_right_index_; }
    std::vector<std::vector<uint8_t>> all_emergency_lanes_vec_vec(){return all_emergency_lanes_vec_vec_;}
    double emergency_right_lane_dist(){return emergency_right_lane_dist_;} // unit m
    double emergency_left_lane_dist(){return emergency_left_lane_dist_;}  // unit m
   private:
    // input
    std::shared_ptr<const message_common::c2c_message::s_EgoMotion2_t> ego_motion_;
    std::shared_ptr<const message::map_position::s_Position_t> map_position_;
    std::shared_ptr<const TopicTrait::MapRouteListMsg> map_route_list_;
    std::shared_ptr<const TopicTrait::MapMapMsg> map_static_info_;
    std::shared_ptr<const std::unordered_map<uint32_t, int>>
        link_id_index_lane_info_map_;  // <map_static_info_.LinkInfos.LinkInfos[i].InstanceId, i>, laneInfos search here
    std::shared_ptr<const std::unordered_map<uint32_t, std::vector<int>>>
        link_id_index_lane_connect_map_;  //<map_static_info_.LaneConnectivitys.PairConnectivity[i].FromLinkId.FromLinkId,
                                          // index_vec> , lane connect info search here
    std::shared_ptr<const std::unordered_map<uint32_t, std::vector<int>>>
        to_link_id_index_lane_connect_map_;  //<map_static_info_.LaneConnectivitys.PairConnectivity[i].ToLinkId.ToLinkId,
                                             // index_vec> , lane connect info search here, index_vec means the [i]
    std::shared_ptr<const std::unordered_map<uint64_t, int>> curve_index_map_;
    std::shared_ptr<const std::unordered_map<uint32_t, int>> linear_obj_id_map_;
    std::shared_ptr<CAlgoRefLaneNode> p_CAlgoRefLaneNode_;

    // local
    bool GenerateCandidateLanes();
    bool GenerateCandidateLanesV2();
    bool GetLinkInfos(uint32_t want_link_id, message::map_map::s_LinkInfo_t& link_infos);
    bool GetLine(const uint32_t line_index, std::vector<message::map_map::s_GeometryPoint_t>& geometry_points);
    bool GetLine(const uint32_t line_index,
                                  std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,uint8_t& mrk_type);
    bool GetAvailableLaneNum(uint32_t search_link_id, uint8_t& righest_available_lane_num,
                             uint8_t& leftest_available_lane_num);
    bool GetPriorCandidateLanes(uint8_t target_lane_num, const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec,
                                int& prior_index_out);
    bool GetLaneDist(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec,
                     std::shared_ptr<const message::map_position::s_Position_t> map_position,
                     const std::vector<uint32_t>& link_id_vec, int index, double& length);

    bool GetLaneLength(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec, int index, size_t& length);
    double CalculatePathFluctuationValue(const int& diff_index, const std::vector<uint8_t>& lane_num_vec,
                                         const std::vector<uint32_t>& link_id_vec);
    bool DeleteEmergencyLane(std::vector<std::vector<uint8_t>> candidate_lanes_vec_vec,
                             std::vector<uint32_t> link_id_vec);
    bool GetPriorCandidateLanesV2(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec,
                                  std::shared_ptr<const message::map_position::s_Position_t> map_position,
                                  const std::vector<uint32_t>& link_id_vec,
                                  const std::vector<bool> is_contain_split_vec,
                                  const std::map<uint32_t, SplitNode> candidate_lanes_split_nodes,
                                  int& left_prior_index_out, int& ego_prior_index_out, int& right_prior_index_out);
    void GetBackConnectInfo(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec,
                            const std::vector<uint32_t>& link_id_vec);

    void GetLaneWidthOne(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec, 
                         const std::vector<uint32_t>& link_id_vec,
                         int lane_prior_index, std::vector<uint16_t>& lane_width_min_vec, 
                         std::vector<uint16_t>& lane_width_max_vec);

    void GetLaneWidth(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec,
                      const std::vector<uint32_t>& link_id_vec);

    void GetLinkLength(const std::vector<uint32_t>& link_id_vec, std::vector<double>& link_length_vec);
    void GetDrvLaneSize(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec, uint8_t& driveable_lane_size);
    //0630新修改存放额外信息的数据，包括lanetype、merge、split、curv
    bool SaveLaneExtraInfo(const message::map_map::s_LaneInfo_t& lane_info_raw,
                           const message::map_map::s_LaneInfo_t& lane_info_left,
                           const message::map_map::s_LaneInfo_t& lane_info_right,
                           uint32_t link_id,
                           LaneExtraInfo_s& lane_extra_info);
    //0630新修改，做成merge、split
    bool SaveLaneExtraInfoMergeSplit(const message::map_map::s_LaneInfo_t& lane_info_raw,
                                     const message::map_map::s_LaneInfo_t& lane_info_side,
                                     uint32_t link_id,
                                     LaneExtraInfo_s& lane_extra_info,
                                     bool is_left);
    bool SaveLaneCurvInfo(const message::map_map::s_LaneInfo_t& lane_info_raw, const uint32_t& link_id, LaneExtraInfo_s& lane_extra_info);
    //0630新修改，通过向后找的方式判断是否是分歧车道
    bool IsSplitLane(uint8_t cur_lane_id, uint8_t side_lane_id, uint32_t link_id); 
    //0630新修改，通过向前找的方式判断是否是汇入车道                                                     
    bool IsMergeLane(uint8_t cur_lane_id, uint8_t side_lane_id, uint32_t link_id,uint32_t& next_link_id, uint8_t& next_lane_id); 
    //0630新修改，两个车道边线是否是虚拟边线(marking=8) 的方式进行判断
    bool IsVirtually(const message::map_map::s_LaneInfo_t& lane_info_raw,
                     const message::map_map::s_LaneInfo_t& lane_info_side,
                     bool is_left);
    //0630新修改，获取车道类型marking
    bool GetLineType(const message::map_map::s_LaneInfo_t& lane_info, bool is_left, uint8_t& line_type);
    //1029
    bool GetLine(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id,bool is_left, uint32_t& line_id) ;                
    // SaveAllLaneExtraInfo 保存一段link里的一条lane里的信息
    bool SaveLaneAllExtraInfo(uint32_t link_id, uint8_t lane_num, LaneExtraInfo_s& lane_extra_info);
    //0630新修改，设置merge、split、offset，不需要单独写函数赋值
    bool GetExtraData(const std::vector<LaneExtraInfo_s>& lane_extra_vec, LaneElement& element);
	bool GetTransitIndex(const std::vector<LaneExtraInfo_s>& lane_extra_vec, std::map<int, uint8_t>& transit_type);
    bool ConstructLaneElement(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec,
                              const std::vector<std::vector<LaneExtraInfo_s>> all_lanes_extra_vec_vec,
                              LaneElementGroupSets& lane_element_group_sets);
    // void GetDrvLaneSizeV2(const LaneElementGroupSets& lane_element_group_sets, uint8_t righest_available_lane_num,
    //                       uint8_t leftest_available_lane_num, uint8_t& driveable_lane_size,
    //                       uint8_t& right_not_driveable_lane_size);
    void GetDrvLaneSizeV3(const LaneElementGroupSets& lane_element_group_sets, uint8_t& driveable_lane_size,
                          uint8_t& fixed_lane_id);
    bool isMergeInRange(const LaneElement& lane_element, double dist_thrd);
    bool isLaneEndInRange(const LaneElement& lane_element, double dist_thrd);

    bool GetSingleLaneBackConnectInfo(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec,
                                      const std::vector<uint32_t>& link_id_vec, int candidate_lane_index,int lane_dir,
                                      std::vector<uint32_t>& back_link_id_vec, std::vector<uint8_t>& back_lane_num_vec);
    bool ModifyMergeAttribute(const message::map_map::s_LaneInfo_t& lane_info_raw, const message::map_map::s_LaneInfo_t&lane_info_side, 
                                               uint32_t link_id, uint32_t next_link_id, uint8_t next_lane_id, uint8_t& raw_merge, uint8_t& side_merge);

    bool ModifyMergeAttributeBySideLine(const message::map_map::s_LaneInfo_t& lane_info_raw, const message::map_map::s_LaneInfo_t&lane_info_side, 
                                               uint32_t link_id, uint32_t next_link_id, uint8_t next_lane_id,uint8_t& raw_merge, uint8_t& side_merge);  

    bool ModifySplitAttribute(const message::map_map::s_LaneInfo_t& lane_info_raw, const message::map_map::s_LaneInfo_t&lane_info_side, 
                                               uint32_t link_id, uint8_t& raw_split, uint8_t& side_split);

    bool StoreModifyedLaneAttribute(std::map<uint64_t,uint8_t>& map_res, std::deque<uint64_t>& deque_res, uint8_t attribute_val, uint8_t lane_id, uint32_t link_id);

    bool GetModifyedLaneAttribute(const std::map<uint64_t,uint8_t>& map_res, const std::deque<uint64_t>& deque_res, uint8_t lane_id, uint32_t link_id,uint8_t& attribute_val);
    bool get_toll_link();
    bool GetBackLinkLane(uint32_t link_id, uint8_t lane_id, uint32_t& back_link_id, uint8_t& back_lane_id);
    void GetMergeBackLinkLane(uint32_t& link_id, uint8_t& ego_lane_num, uint8_t& side_lane_num);
    double HeadingTransform(double in);
    bool IsRoadMerge(const uint32_t link_id, const uint8_t lane_num_raw, const uint8_t lane_num_side, uint32_t ego_offset, uint32_t ego_path_id,
                                       int32_t& merge_start_offset, int32_t& merge_end_offset);
    bool IsRoadSplit(const uint32_t link_id, const uint8_t lane_num_raw, const uint8_t lane_num_side, uint32_t ego_path_id,
                                       std::vector<std::pair<uint64_t,SplitInfo_S>>& split_info_vec_raw, std::vector<std::pair<uint64_t,SplitInfo_S>>& split_info_vec_side);
    bool StoreSplitInfo(std::map<uint64_t,SplitInfo_S>& map_res, std::deque<uint64_t>& deque_res, SplitInfo_S split_info, uint8_t lane_id, uint32_t link_id);
    bool ChangeStoredSplitInfo(std::map<uint64_t,SplitInfo_S>& map_res, std::deque<uint64_t>& deque_res, uint8_t lane_id, uint32_t link_id, SplitInfo_S split_info);

    template<typename K,typename V>
    bool StoreStaticMapAndDeque(std::map<K,V>& map_res, std::deque<K>& deque_res, K map_key, V map_val, int max_size);

    template<typename K,typename V>
    bool GetStaticMapAndDeque(const std::map<K,V>& map_res, const std::deque<K>& deque_res, K map_key, V& map_val);

    template<typename K,typename V>
    bool ChangeStaticMapAndDeque(std::map<K,V>& map_res, std::deque<K>& deque_res, K map_key, V change_map_val);

    bool StoreEgoDrivePathLinkLane(std::map<uint64_t,uint8_t>& map_res, std::deque<uint64_t>& deque_res, uint32_t path_id, uint8_t lane_id, uint32_t link_id);
    bool GetEgoDrivePathLinkLane(const std::map<uint64_t,uint8_t>& map_res, const std::deque<uint64_t>& deque_res, uint8_t lane_id, uint32_t link_id,uint8_t& path_id);    
    bool CalculateMergeLinkLineHeading(const message::map_map::s_LaneInfo_t& left_lane_infos, const message::map_map::s_LaneInfo_t& right_lane_infos, 
                                        const efm::MapStaticInfo& map_static_info, double& sum_heading_ri_lane_le, double& sum_heading_le_lane_ri,
                                        double& aver_heading_ri_lane_le, double& aver_heading_le_lane_ri);
    
    bool GetEgoLanePreCutInMerge(const std::vector<uint32_t>& link_id_vec, const std::vector<uint8_t>& lane_num_vec, 
                                                  const std::vector<LaneExtraInfo_s>& lane_extro_vec, std::vector<uint32_t>& end_offset_vec);
    bool GetEndOffsetCloseForArrowLane(std::vector<uint32_t> link_id_vec, std::vector<uint8_t> lane_num_vec,
                                                     int32_t close_idx, bool side_dir_is_left,uint32_t& end_offset);
    bool GetLineGeometry(uint32_t line_id, std::vector<EFMPoint>& geometry_points);
    bool GetTwoLineDistance(uint32_t left_line, uint32_t right_line,
                                          double& start_distance, double& end_distance, 
                                          std::vector<EFMPoint> &left_geometry_points, std::vector<EFMPoint> &right_geometry_points);
    void GetBackConnectInfoEmergency(
                     const std::vector<std::vector<uint8_t>>& all_emergency_lanes_vec_vec, const std::vector<uint32_t>& link_id_vec,uint8_t ego_lane_num);
    // output
    std::vector<uint32_t> link_id_vec_;       // link id list mapping to candidate
                                              // lanes
    std::vector<int> link_id_index_vec_;      // index in link_id_index_lane_info_map_
    std::vector<bool> is_contain_split_vec_;  // with link_id_vec_

    std::map<uint32_t, SplitNode> candidate_lanes_split_nodes_;  //<link_id, >

    std::vector<double> link_length_vec_;  // first length is ego to endoffset dist, rest is link length
    std::vector<uint16_t> ego_lane_width_min_vec_;
    std::vector<uint16_t> ego_lane_width_max_vec_;
    std::vector<uint16_t> left_lane_width_min_vec_;
    std::vector<uint16_t> left_lane_width_max_vec_;
    std::vector<uint16_t> left_left_lane_width_min_vec_;
    std::vector<uint16_t> left_left_lane_width_max_vec_;
    std::vector<uint16_t> right_lane_width_min_vec_;
    std::vector<uint16_t> right_lane_width_max_vec_;
    std::vector<uint16_t> right_right_lane_width_min_vec_;
    std::vector<uint16_t> right_right_lane_width_max_vec_;
    /*all_lanes_vec_vec_实际数据示例
        std::vector<std::vector<uint8_t>> all_lanes_vec_vec_ = {
        {10, 11, 12, 13, 14, 15, 16, 17}, // lane 1
        {20, 21, 22, 23, 24, 25, 26, 27}, // lane 2
        {30, 31, 32, 33, 34, 35, 36, 37}, // lane 3
        {0, 0, 0, 0, 1, 1, 0, 0}, // lane 4
        {0, 0, 0, 0, 0, 1, 1, 0}, // lane 5
        {0, 0, 0, 0, 0, 0, 1, 1}, // lane 6
    };
    */

    std::vector<std::vector<uint8_t>> all_lanes_vec_vec_;  // all possible running lanes
    std::vector<std::vector<uint8_t>> all_emergency_lanes_vec_vec_;  // all emergency lanes, new1223
    // std::vector<std::vector<uint8_t>> candidate_lanes_vec_vec_;  // at least one optimal lane, maximun 3 lanes,
    //                                                              // include ego, left, right
    std::vector<std::vector<LaneExtraInfo_s>> all_lanes_extra_vec_vec_;  // mapping to all_lanes_vec_vec_
    int ego_lane_prior_index_;
    int left_lane_prior_index_;
    int left_left_lane_prior_index_;
    int right_lane_prior_index_;
    int right_right_lane_prior_index_;
    int ref_lane_prior_index_;
    int ref_route_lane_prior_index_;
    int32_t emergency_left_index_;
    int32_t emergency_right_index_;
    size_t ego_lane_prior_length_;
    size_t left_lane_prior_length_;
    size_t left_left_lane_prior_length_;
    size_t right_lane_prior_length_;
    size_t right_right_lane_prior_length_;
    double ego_lane_prior_dist_;    // unit m
    double left_lane_prior_dist_;   // unit m
    double left_left_lane_prior_dist_;   // unit m
    double right_lane_prior_dist_;  // unit m
    double right_right_lane_prior_dist_;  // unit m
    double emergency_right_lane_dist_;  // unit m
    double emergency_left_lane_dist_;  // unit m
    std::vector<uint8_t> ego_path_lane_num_;
    std::vector<uint8_t> left_path_lane_num_;
    std::vector<uint8_t> left_left_path_lane_num_;
    std::vector<uint8_t> right_path_lane_num_;
    std::vector<uint8_t> right_right_path_lane_num_;
    // std::vector<uint8_t> candidate_prior_vec;  // ego, left,right lanes prior,
    //                                            // first element is highest
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
    std::vector<uint32_t> merge_back_links_id_;
    std::vector<uint8_t> merge_ego_back_lanes_id_;
    std::vector<uint8_t> merge_side_back_lanes_id_;
    std::vector<uint32_t> back_emergency_left_link_id_vec_;
    std::vector<uint32_t> back_emergency_right_link_id_vec_;
    std::vector<uint8_t> back_emergency_left_lane_num_vec_;
    std::vector<uint8_t> back_emergency_right_lane_num_vec_;

    uint8_t driveable_lane_size_;  // dr
    uint8_t right_not_driveable_lane_size_;
    uint8_t min_driveable_lane_num_;
    uint8_t max_driveable_lane_num_;
    uint8_t fixed_lane_id_;

    LaneElementGroupSets lane_element_group_sets_;
    // int ego_lane_index_for_lane_mkr_;
    // int left_lane_index_for_lane_mkr_;
    // int right_lane_index_for_lane_mkr_;
    int ref_lane_split_position_;
    int ref_lane_next_split_position_;
    uint8_t ref_lane_split_from_;

    message::efm::s_NodeInfo_t NodeInfo_;
    std::set<uint32_t> toll_link_id_set_;
    std::vector<uint32_t> pre_cutin_merge_offset_vec_;//自车道信息，旁车道merge进来，旁车道的merge点提前到2m宽位置，没减去自车的offset
};

}  // namespace framework
}  // namespace shell
}  // namespace earth
