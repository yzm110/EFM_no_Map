/*
 * @Description:
 * @Author: zj
 */
#pragma once
#include "CommonDataType.h"
#include "CommonMathMethod.h"
#include "CompileConfig.h"
#include "CoordinateTool.h"
// #include "topic/topic_trait.h"
#include "common/framework.h"
#include "calibration.h"
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

namespace earth {
namespace shell {
namespace framework {
class CAlgoRefLaneNode {
   public:
    CAlgoRefLaneNode(){
        ego_lane_index_ = -1;
        left_lane_idx_  = -1;
        right_lane_idx_ = -1;
        ref_lane_idx_   = -1;
        ego_left_same_lane_ = false;
        ego_right_same_lane_ = false;
    };
    ~CAlgoRefLaneNode(){};

    bool InitVariable(const message::map_position::s_Position_t& map_position,
                      const TopicTrait::MapRouteListMsg& map_route_list, const TopicTrait::MapMapMsg& map_static_info,
                      const std::unordered_map<uint32_t, int>& link_id_index_lane_info_map,
                      const std::unordered_map<uint32_t, std::vector<int>>& link_id_index_lane_connect_map,
                      const std::unordered_map<uint32_t, std::vector<int>>& to_link_id_index_lane_connect_map);

    bool GetRefLine(std::vector<uint32_t> link_id_vec,
                    LaneElementGroupSets& lane_element_group_sets, int32_t& ego_lane_index, 
                    int32_t& left_lane_idx, int32_t& left_left_lane_idx, 
                    int32_t& right_lane_idx, int32_t& right_right_lane_idx, 
                    int32_t& ref_lane_idx, std::map<uint64_t,uint8_t>& split_lind_id_lane_id_map);

    bool GetNode(std::vector<uint32_t> link_id_vec, LaneElementGroupSets& lane_element_group_sets,
                 int32_t& ego_lane_index, message::efm::s_NodeInfo_t& NodeInfo, int& ref_route_idx);

   private:
    bool SetLaneElementGroupInfo(std::vector<uint32_t> link_id_vec, LaneElementGroupSets& lane_element_group_sets);
    bool GetRefLanesForLength(LaneElementGroupSets& lane_element_group_sets, std::vector<std::vector<int32_t>>& ref_lanes);
    bool GetRefLanesForRoute(std::vector<uint32_t> link_id_vec,
                             LaneElementGroupSets& lane_element_group_sets, std::vector<std::vector<int32_t>>& ref_lanes);
    bool GetRefLanesForClose(std::vector<uint32_t> link_id_vec,
                             LaneElementGroupSets& lane_element_group_sets, std::vector<std::vector<int32_t>>& ref_lanes);
    bool DeleteSplitLane(std::vector<uint32_t> link_id_vec,
                         LaneElementGroupSets& lane_element_group_sets, std::vector<std::vector<int32_t>>& ref_lanes);
    bool MakeRefLine(std::vector<uint32_t> link_id_vec, LaneElementGroupSets& lane_element_group_sets, 
                     std::vector<std::vector<int32_t>>& ref_lanes,
                     int32_t& ego_lane_index, int32_t& left_lane_idx, 
                     int32_t& left_left_lane_idx, int32_t& right_lane_idx, 
                     int32_t& right_right_lane_idx, int32_t& ref_lane_idx);
    bool GetNodeElement(std::vector<uint32_t> link_id_vec, LaneElement& lane_element, 
                        message::efm::s_NodeInfo_t& NodeInfo, std::vector<LaneElement>& lane_element_group);


    bool GetLinkInfos(uint32_t link_id, message::map_map::s_LinkInfo_t& link_infos);
    bool GetPositonToLinkStart(uint32_t link_id, uint32_t& dist);
    bool GetLaneTrans(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, uint8_t& trans);
    bool IsNoVirtually(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, uint8_t& trans,
                       uint8_t& split_from, uint8_t& close_to);
    bool GetLaneType(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, uint8_t& lane_type);
    bool GetSideLane(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, 
                                   uint32_t next_link_id, uint8_t next_lane_id, uint8_t& side_lane_id);
    bool GetSideLaneForSplit(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, uint8_t& side_lane_id);
    bool GetLineType(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, uint8_t& line_type, bool is_left, uint32_t& line_id);
    bool GetNextLinkIDForRoute(uint32_t link_id, uint32_t& next_link_id);
    bool GetConnectLinkLanes(uint32_t next_link_id, uint32_t link_id, std::vector<uint8_t>& connect_next_link_lanes, std::vector<uint8_t>& last_link_lanes);
    bool GetMinStepToRoute(std::vector<uint8_t> lanes_vec, std::vector<uint8_t> connect_next_link_lanes, uint8_t& temp_min_step);
    bool DeleteSplitLaneForOne(std::vector<uint32_t> link_id_vec, LaneElementGroup& lane_element_group, uint32_t max_lane_number);
    bool GetDestElementGroup(std::vector<uint32_t> link_id_vec, LaneElementGroup& lane_element_group, 
                             std::vector<int32_t> lane_idxs, int32_t& real_lane_idx);
    bool GetEgpGroupIdx(std::vector<uint32_t> link_id_vec, LaneElementGroupSets& lane_element_group_sets, int32_t& egp_group_idx);
    bool GetLaneCandidateIndex(LaneElementGroup& lane_element_group, int32_t& lane_candidate_index);
    bool GetLaneCandidateIndexEgo(LaneElementGroup& lane_element_group, int32_t& ego_candidate_index, int32_t& left_candidate_index, 
                                  int32_t& right_candidate_index, std::vector<uint32_t> link_id_vec);
    int32_t GetContinueLaneCandidateIndex(LaneElementGroup& lane_element_group, int32_t ignore_split_idx);
    bool GetRefLaneCandidateIndex(LaneElementGroupSets& lane_element_group_sets,
                                  int32_t& ego_lane_index, int32_t& left_lane_idx,
                                  int32_t& left_left_lane_idx, int32_t& right_lane_idx,
                                  int32_t& right_right_lane_idx, int32_t& ref_lane_idx,
                                  std::vector<std::vector<int32_t>>& ref_lanes,
                                  std::vector<uint32_t> link_id_vec);
    bool GetCloseDir(std::vector<uint8_t> lane_id_vec, uint32_t close_idx, uint8_t& close_to,
                     LaneElementGroupSets& lane_element_group_sets);
    bool GetSplitDir(std::vector<uint8_t> lane_id_vec, uint32_t split_idx, uint8_t& split_from,
                     LaneElementGroupSets& lane_element_group_sets);
    bool GetLaneChgTypeClose(std::vector<uint32_t> link_id_vec, LaneElement& lane_element, int32_t close_idx, uint8_t& LaneChgType);
    bool GetStartOffsetClose(std::vector<uint32_t> link_id_vec, LaneElement& lane_element, int32_t close_idx,
                             uint8_t& LaneChgType, double& start_offset);
    bool GetEndOffsetClose(std::vector<uint32_t> link_id_vec, LaneElement& lane_element, int32_t close_idx,
                           uint8_t& LaneChgType, double& end_offset);
    bool GetLaneChgTypeSplit(std::vector<uint32_t> link_id_vec, LaneElement& lane_element, int32_t split_idx, uint8_t& LaneChgType);
    bool GetStartOffsetSplit(std::vector<uint32_t> link_id_vec, LaneElement& lane_element, int32_t split_idx,
                             uint8_t& LaneChgType, double& start_offset);
    bool GetEndOffsetSplit(std::vector<uint32_t> link_id_vec, LaneElement& lane_element, int32_t split_idx,
                           uint8_t& LaneChgType, double& end_offset);
    bool IsCLoseLink(uint32_t link_id);
    bool IsSplitLink(uint32_t link_id);
    bool GetBackLinkLane(uint32_t link_id, uint8_t lane_id, uint32_t& back_link_id, uint8_t& back_lane_id);
    bool GetBackLinkLaneForConnect(uint32_t link_id, uint8_t lane_id, uint32_t& back_link_id, uint8_t& back_lane_id);
    bool IsChangeToRouteLane(uint32_t link_id, uint8_t& lane_id, std::vector<uint8_t> link_route_lanes);
    bool GetConnectLinkLane(uint32_t link_id, uint32_t& next_link_id, uint8_t lane_id, uint8_t& next_lane_id);
    bool MakeNodeForLastLinkNotINRoute(LaneElementGroupSets& lane_element_group_sets, LaneElement& lane_element, std::vector<uint32_t> link_id_vec);
    bool GetEndOffsetCloseForArrowLane(std::vector<uint32_t> link_id_vec, LaneElement& lane_element,
                                       int32_t close_idx, uint32_t& end_offset);
    bool GetTwoLineDistance(uint32_t left_line, uint32_t right_line, double& start_distance, double& end_distance,
                            std::vector<EFMPoint> &left_geometry_points, std::vector<EFMPoint> &right_geometry_points);
    bool GetLineGeometry(uint32_t line_id, std::vector<EFMPoint>& geometry_points);
    bool isNotMergeLane(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id);
    bool GetElementRealLength(std::vector<uint32_t> link_id_vec, std::vector<LaneElement> lane_element, uint32_t& close_length, uint32_t& length, int merge_to);
    bool SideLaneDistIsShort(std::vector<uint32_t> link_id_vec, LaneElementGroupSets lane_element_group_sets, 
                             int32_t group_idx,int32_t ego_group_idx, bool merge_to_left);
    bool GetRefLanesForFirstVirtuallyMerge(std::vector<uint32_t> link_id_vec, 
                                           LaneElementGroupSets& lane_element_group_sets,
                                           std::vector<std::vector<int32_t>>& ref_lanes);
    bool IsFirstMergeVirtually(LaneElement& s_element, LaneElement& e_element);
    bool DeleteFormRefForFirstMergeVirtually(std::vector<uint32_t> link_id_vec, LaneElement& s_element, LaneElement& e_element);
	bool IsSplitLinkLane(uint32_t link_id, uint8_t& cur_lane_id, uint8_t& side_lane_id);
    //0630 add
    bool GetNodeElementChangelane(std::vector<uint32_t> link_id_vec, LaneElement& lane_element, message::efm::s_NodeInfo_t& NodeInfo);
    bool RefEgoIsOneGroup(std::vector<LaneElement>& lane_element_group);
    bool GetElementForCandidateIndex(std::vector<LaneElement>& lane_element_group, LaneElement& element, int32_t index);
    bool GetRefForEgoSplit(LaneElement element, LaneElement side_element, int32_t& ref_lane_idx);
	bool IsNextMergeShort(LaneElement& lane_element, int32_t close_idx, int32_t& temp_next_close_idx);
    bool GetRouteLaneForSplit(std::vector<int32_t>& min_close_time_idxs, std::vector<uint32_t> link_id_vec, 
                              LaneElementGroup& lane_element_group, int32_t& real_lane_idx);
    bool GetRouteLaneForSplitOne(std::vector<uint32_t> link_id_vec, LaneElementGroup& lane_element_group, 
                                 std::vector<int32_t>& idxs, std::vector<uint8_t> lane_unique_ids, 
                                 std::vector<EfmSplitType> lane_unique_splits, uint32_t lane_idx, std::vector<int32_t>& new_idxs);
    bool GetRouteLaneForSplitLaneId(std::vector<uint32_t> link_id_vec, std::vector<uint8_t> lane_unique_ids,
                                    std::vector<EfmSplitType> lane_unique_splits, uint32_t lane_idx,
                                    uint8_t& route_lane_id, LaneElement element_one);
    bool ChangeTransit(std::vector<uint32_t> link_id_vec, LaneElement& lane_element,
                               std::vector<LaneElement>& lane_element_group, uint32_t idx, 
                               std::map<uint64_t,uint8_t>& split_lind_id_lane_id_map);
    bool ChangeRampLaneTransitForRefLane(std::vector<uint32_t> link_id_vec, LaneElementGroupSets& lane_element_group_sets,
                               int32_t& ref_lane_idx, std::map<uint64_t,uint8_t>& split_lind_id_lane_id_map, int32_t& ego_lane_index);
    bool GetRefRouteIdx(LaneElementGroupSets& lane_element_group_sets, int& ref_route_idx);
    
    // input
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
    uint32_t narrow_lane_width_; //m

    int32_t ego_lane_index_, left_lane_idx_, right_lane_idx_, ref_lane_idx_, left_left_lane_idx_, right_right_lane_idx_;
    int32_t ego_group_index_, left_group_idx_, right_group_idx_, left_left_group_idx_, right_right_group_idx_;
    bool ego_left_same_lane_, ego_right_same_lane_;
    std::vector<uint32_t> link_id_vec_;

};

}  // namespace framework
}  // namespace shell
}  // namespace earth
