#include "AlgoRefLaneNode.h"

#define DOUBLE_DATA_RANGE 0.001  // mm
#define RAMP_TO_MAIN_LENGTH 700  // m
#define CONTINUOUS_DIVER_LENGTH 10000  // cm
#define POSITION_TO_SPLIT_DIST 4000  // cm
#define POSITION_TO_MERGE_DIST 6000  // cm
#define NEXT_MERGE_DIST 10000  // cm
#define NEXT_SPLIT_DIST 50000  // cm
#define SHORT_ELEMENT_DIST 30000  // cm

namespace earth {
namespace shell {
namespace framework {

bool CAlgoRefLaneNode::InitVariable(
    const message::map_position::s_Position_t& map_position, const TopicTrait::MapRouteListMsg& map_route_list,
    const TopicTrait::MapMapMsg& map_static_info, const std::unordered_map<uint32_t, int>& link_id_index_lane_info_map,
    const std::unordered_map<uint32_t, std::vector<int>>& link_id_index_lane_connect_map,
    const std::unordered_map<uint32_t, std::vector<int>>& to_link_id_index_lane_connect_map) {
    map_position_ = std::make_shared<const message::map_position::s_Position_t>(map_position);
    map_route_list_ = std::make_shared<const TopicTrait::MapRouteListMsg>(map_route_list);
    map_static_info_ = std::make_shared<const TopicTrait::MapMapMsg>(map_static_info);
    link_id_index_lane_info_map_ =
        std::make_shared<const std::unordered_map<uint32_t, int>>(link_id_index_lane_info_map);
    link_id_index_lane_connect_map_ =
        std::make_shared<const std::unordered_map<uint32_t, std::vector<int>>>(link_id_index_lane_connect_map);
    to_link_id_index_lane_connect_map_ =
        std::make_shared<const std::unordered_map<uint32_t, std::vector<int>>>(to_link_id_index_lane_connect_map);
    narrow_lane_width_ = p_narrow_lane_width;

    return true;
}

bool CAlgoRefLaneNode::GetRefLine(std::vector<uint32_t> link_id_vec, LaneElementGroupSets& lane_element_group_sets,
                                  int32_t& ego_lane_index, int32_t& left_lane_idx, int32_t& left_left_lane_idx, 
                                  int32_t& right_lane_idx, int32_t& right_right_lane_idx, int32_t& ref_lane_idx, 
                                  std::map<uint64_t,uint8_t>& split_lind_id_lane_id_map) {
    ego_lane_index = -1;
    left_lane_idx = -1;
    left_left_lane_idx = -1;
    right_lane_idx = -1;
    right_right_lane_idx = -1;
    ref_lane_idx = -1;
    link_id_vec_ = link_id_vec;

    if (0 == lane_element_group_sets.size()) {
        // no data
        // LOGE("lane_element_group_sets.size :{}", lane_element_group_sets.size());
        return false;
    }

    if (false == SetLaneElementGroupInfo(link_id_vec, lane_element_group_sets)) {
        // LOGE("get SetLaneElementGroupInfo error!");
        return false;
    }

    if (1 == lane_element_group_sets.size() && 1 == lane_element_group_sets[0].size()) {
        // one lane
        int32_t ego_group_idx = -1;
        if (false == GetEgpGroupIdx(link_id_vec, lane_element_group_sets, ego_group_idx)){
            return false;
        }
        
        ego_lane_index = lane_element_group_sets[0][0].candidate_index;
        ref_lane_idx = lane_element_group_sets[0][0].candidate_index;
        // return true;
    }

    std::vector<std::vector<int32_t>> ref_lanes;
    if (false == GetRefLanesForLength(lane_element_group_sets, ref_lanes)) {
        // LOGE("get GetRefLanesForLength error!");
        return false;
    }
    // std::cout << __FILE__ << "," << __LINE__ << "," << " ref_lanes.size: " << ref_lanes.size() << std::endl;
    // for (size_t i = 0; i < ref_lanes.size(); i++)
    // {
    //     std::cout << __FILE__ << "," << __LINE__ << "," << " ref_lanes[i].size(): " << ref_lanes[i].size() <<
    //     std::endl; for (size_t j = 0; j < ref_lanes[i].size(); j++)
    //     {
    //         std::cout << __FILE__ << "," << __LINE__ << "," << " ref_lan: " << ref_lanes[i][j] << std::endl;
    //     }
    // }

    if (false == GetRefLanesForRoute(link_id_vec, lane_element_group_sets, ref_lanes)) {
        // LOGE("get GetRefLanesForRoute error!");
        return false;
    }

    if (false == GetRefLanesForClose(link_id_vec, lane_element_group_sets, ref_lanes)) {
        // LOGE("get GetRefLanesForClose error!");
        return false;
    }

    if (false == DeleteSplitLane(link_id_vec, lane_element_group_sets, ref_lanes)) {
        // LOGE("get DeleteSplitLane error!");
        return false;
    } 

    if (false == MakeRefLine(link_id_vec, lane_element_group_sets, ref_lanes, ego_lane_index, left_lane_idx,
                             left_left_lane_idx, right_lane_idx, right_right_lane_idx, ref_lane_idx)) {
        // LOGE("get MakeRefLine error!");
        return false;
    }

    if (false == ChangeRampLaneTransitForRefLane(link_id_vec, lane_element_group_sets, ref_lane_idx, split_lind_id_lane_id_map, ego_lane_index)) {
        // LOGE("get ChangeRampLaneTransitForRefLane error!");
        return false;
    }
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //         << " ego_lane_index: " << ego_lane_index << std::endl;
    return true;
}

bool CAlgoRefLaneNode::ChangeRampLaneTransitForRefLane(std::vector<uint32_t> link_id_vec, LaneElementGroupSets& lane_element_group_sets,
                               int32_t& ref_lane_idx, std::map<uint64_t,uint8_t>& split_lind_id_lane_id_map, int32_t& ego_lane_index) {
    if (ref_lane_idx < 0){
        return false;
    }

    int ref_group_idx = -1;
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            if (ref_lane_idx == lane_element.candidate_index) {
                ref_group_idx = lane_idx;
                ChangeTransit(link_id_vec, lane_element, lane_element_group_sets[lane_idx], lane_dir_idx, split_lind_id_lane_id_map);
                break;
            }
        }
    }

    int ego_group_idx = -1;
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            if (ego_lane_index == lane_element.candidate_index) {
                ego_group_idx = lane_idx;
                break;
            }
        }
    }

    if (ego_group_idx >= 0 && ref_group_idx >= 0 && ego_group_idx != ref_group_idx){
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[ego_group_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[ego_group_idx][lane_dir_idx];
            if (true == lane_element.is_group_dest) {
                ChangeTransit(link_id_vec, lane_element, lane_element_group_sets[ego_group_idx], lane_dir_idx, split_lind_id_lane_id_map);
                break;
            }
        }
    }
    
    
    return true;
}


bool CAlgoRefLaneNode::ChangeTransit(std::vector<uint32_t> link_id_vec, LaneElement& lane_element,
                               std::vector<LaneElement>& lane_element_group, uint32_t idx, 
                               std::map<uint64_t,uint8_t>& split_lind_id_lane_id_map) {
    if (idx >= lane_element_group.size()){
        return false;
    }

    if ((lane_element.lane_num_vec.size() != lane_element.lane_extra_infos.size()) || 
        (lane_element.lane_num_vec.size() > link_id_vec.size())){
        return false;
    }
    

    for (size_t lane_idx = 1; lane_idx < lane_element.lane_num_vec.size(); lane_idx++){
        auto & extra_info = lane_element.lane_extra_infos[lane_idx];
        auto & back_extra_info = lane_element.lane_extra_infos[lane_idx-1];
        if (!((0 == extra_info.lane_type || 3 == extra_info.lane_type || 
            9 == extra_info.lane_type || 17 == extra_info.lane_type) || 
            ( 3 == back_extra_info.lane_type || 
            9 == back_extra_info.lane_type || 17 == back_extra_info.lane_type))){
            uint64_t key_temp = ((static_cast<uint64_t>(lane_element.lane_num_vec[lane_idx])) <<32) | (static_cast<uint64_t>(link_id_vec[lane_idx]));
            if (extra_info.split_value == EFM_SplitType_SPLIT_FROM_LEFT){
                extra_info.split_value = EFM_SplitType_CONTIUE_FROM_LEFT;
                split_lind_id_lane_id_map[key_temp] = 1;
                for (size_t new_idx = 0; new_idx < lane_element_group.size(); new_idx++){
                    if (lane_element_group[new_idx].lane_num_vec.size() > lane_idx && lane_element_group[new_idx].lane_extra_infos.size() > lane_idx){
                        if (lane_element_group[new_idx].lane_num_vec[lane_idx -1] == lane_element.lane_num_vec[lane_idx -1]){
                            if (lane_element_group[new_idx].lane_num_vec[lane_idx] == lane_element.lane_num_vec[lane_idx]){
                                lane_element_group[new_idx].lane_extra_infos[lane_idx].split_value = extra_info.split_value;
                            }else{
                                lane_element_group[new_idx].lane_extra_infos[lane_idx].split_value = EFM_SplitType_SPLIT_FROM_RIGHT;
                                uint64_t new_key_temp = ((static_cast<uint64_t>(lane_element_group[new_idx].lane_num_vec[lane_idx])) <<32) | (static_cast<uint64_t>(link_id_vec[lane_idx]));
                                split_lind_id_lane_id_map[new_key_temp] = 3;
                            }
                        }
                    }
                }
            }
            if (extra_info.split_value == EFM_SplitType_SPLIT_FROM_RIGHT){
                extra_info.split_value = EFM_SplitType_CONTIUE_FROM_RIGHT;
                split_lind_id_lane_id_map[key_temp] = 1;
                for (size_t new_idx = 0; new_idx < lane_element_group.size(); new_idx++){
                    if (lane_element_group[new_idx].lane_num_vec.size() > lane_idx && lane_element_group[new_idx].lane_extra_infos.size() > lane_idx){
                        if (lane_element_group[new_idx].lane_num_vec[lane_idx -1] == lane_element.lane_num_vec[lane_idx -1]){
                            if (lane_element_group[new_idx].lane_num_vec[lane_idx] == lane_element.lane_num_vec[lane_idx]){
                                lane_element_group[new_idx].lane_extra_infos[lane_idx].split_value = extra_info.split_value;
                            }else{
                                lane_element_group[new_idx].lane_extra_infos[lane_idx].split_value = EFM_SplitType_SPLIT_FROM_LEFT;
                                uint64_t new_key_temp = ((static_cast<uint64_t>(lane_element_group[new_idx].lane_num_vec[lane_idx])) <<32) | (static_cast<uint64_t>(link_id_vec[lane_idx]));
                                split_lind_id_lane_id_map[new_key_temp] = 3;
                            }
                        }
                    }
                }
            }           
        }
        
    }
    
    return true;
}

bool CAlgoRefLaneNode::GetRefRouteIdx(LaneElementGroupSets& lane_element_group_sets, int& ref_route_idx) {
    ref_route_idx = ref_lane_idx_;

    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            if (lane_element.is_dest) {
                ref_route_idx = lane_element.candidate_index;
                if (lane_element.is_group_dest){
                    break;
                }
            }
        }
    }

    return true;
}

bool CAlgoRefLaneNode::GetNode(std::vector<uint32_t> link_id_vec, LaneElementGroupSets& lane_element_group_sets,
                               int32_t& ego_lane_index, message::efm::s_NodeInfo_t& NodeInfo, int& ref_route_idx) {
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //     << " ego_lane_index: " << ego_lane_index << std::endl;
    GetRefRouteIdx(lane_element_group_sets, ref_route_idx);
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            if (ego_lane_index == lane_element.candidate_index) {
                //std::cout << __FILE__ << "," << __LINE__ << "," << " ego_left_same_lane_: " << std::endl;
                GetNodeElement(link_id_vec, lane_element, NodeInfo, lane_element_group_sets[lane_idx]);
                break;
            }
        }
    }

    return true;
}

bool CAlgoRefLaneNode::SetLaneElementGroupInfo(std::vector<uint32_t> link_id_vec,
                                               LaneElementGroupSets& lane_element_group_sets) {
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            if (lane_element.lane_num_vec.size() > link_id_vec.size()) {
                continue;
            }

            // merge/split
            bool split_exits_flag = false;
            bool close_exits_flag = false;
            bool next_split_exits_flag = false;
            lane_element.close_times = 0;
            lane_element.split_from = 0;
            lane_element.close_to = 0;
            //std::cout << __FILE__ << "," << __LINE__ << "," << " candidate_index: " << lane_element.candidate_index << std::endl;
            for (size_t num_idx = 0; num_idx < lane_element.lane_num_vec.size(); num_idx++) {
                auto& temp_merge_value = lane_element.lane_extra_infos[num_idx].merge_value;
                auto& temp_split_value = lane_element.lane_extra_infos[num_idx].split_value;
                //std::cout << __FILE__ << "," << __LINE__ << "," << " num_idx: " << num_idx << std::endl;
                //std::cout << __FILE__ << "," << __LINE__ << "," << " linkid: " << link_id_vec[num_idx] << std::endl;
                //std::cout << __FILE__ << "," << __LINE__ << "," << " lane: " << int(lane_element.lane_num_vec[num_idx]) << std::endl;
                //std::cout << __FILE__ << "," << __LINE__ << "," << " temp_split_value: " << int(temp_split_value) << std::endl;
                //std::cout << __FILE__ << "," << __LINE__ << "," << " temp_merge_value: " << int(temp_merge_value) << std::endl;
                if (EFM_MergeType_NONE != temp_merge_value && EFM_MergeType_FROM_LEFT != temp_merge_value &&
                    EFM_MergeType_FROM_RIGHT != temp_merge_value){
                    // Merging_
                    if (false == close_exits_flag) {
                        lane_element.close_position = num_idx;
                        if (EFM_MergeType_TO_LEFT == temp_merge_value || EFM_MergeType_LEFT_TO_MIDDLE == temp_merge_value){
                            lane_element.close_to = 1;
                        }else{
                            lane_element.close_to = 2;
                        }

                        if (EFM_MergeType_RIGHT_TO_MIDDLE == temp_merge_value || EFM_MergeType_LEFT_TO_MIDDLE == temp_merge_value){
                            lane_element.is_virtually_close = true;
                        }else{
                            lane_element.is_virtually_close = false;
                            lane_element.close_times++;
                        }
                    }else{
                        if (EFM_MergeType_TO_LEFT == temp_merge_value || EFM_MergeType_TO_RIGHT == temp_merge_value){
                            lane_element.close_times++;
                        }
                    }
                    close_exits_flag = true;
                }

                if (EFM_SplitType_NONE != temp_split_value && EFM_SplitType_TO_LEFT != temp_split_value && 
                    EFM_SplitType_TO_RIGHT != temp_split_value && false == split_exits_flag && num_idx > 0){
                    
                    lane_element.split_position = num_idx;
                    lane_element.split_offset = lane_element.lane_extra_infos[num_idx].s_offset;
                    if (EFM_SplitType_FROM_LEFT == temp_split_value || EFM_SplitType_CONTIUE_FROM_LEFT == temp_split_value ||
                        EFM_SplitType_SPLIT_FROM_LEFT == temp_split_value){
                        lane_element.split_from = 2;
                    }else{
                        lane_element.split_from = 1;
                    }
                    

                    if (EFM_SplitType_FROM_LEFT == temp_split_value || EFM_SplitType_FROM_RIGHT == temp_split_value){
                        lane_element.is_virtually_split = false;
                    }else{
                        lane_element.is_virtually_split = true;
                    }
                    split_exits_flag = true;
                    continue;
                }

                if (EFM_SplitType_NONE != temp_split_value && EFM_SplitType_TO_LEFT != temp_split_value && 
                    EFM_SplitType_TO_RIGHT != temp_split_value && true == split_exits_flag && num_idx > 0 &&
                    false == next_split_exits_flag){
                    lane_element.next_split_position = num_idx;
                    next_split_exits_flag = true;
                }             

                // message::map_map::s_LinkInfo_t link_infos;
                // if (false == GetLinkInfos(link_id_vec[num_idx], link_infos)) {
                //     // static data none
                //     continue;
                // }
                // uint8_t trans = 0;
                // GetLaneTrans(link_infos, lane_element.lane_num_vec[num_idx], trans);
                // uint8_t temp_split_from = 0;
                // uint8_t temp_close_to = 0;

                // if (2 == trans) {
                //     if (isNotMergeLane(link_infos, lane_element.lane_num_vec[num_idx])){
                //         continue;
                //     }
                    
                //     // Merging_
                //     if (false == close_exits_flag) {
                //         lane_element.close_position = num_idx;
                //         lane_element.is_virtually_close = true;
                //     }

                //     if (IsNoVirtually(link_infos, lane_element.lane_num_vec[num_idx], trans, temp_split_from,
                //                       temp_close_to)) {
                //         lane_element.close_times++;
                //         if (false == close_exits_flag) {
                //             lane_element.is_virtually_close = false;
                //             lane_element.close_to = temp_close_to;
                //         }
                //     }

                //     if (false == close_exits_flag) {
                //         if (0 == temp_close_to) {
                //             GetCloseDir(lane_element.lane_num_vec, num_idx, lane_element.close_to,
                //                         lane_element_group_sets);
                //         } else {
                //             lane_element.close_to = temp_close_to;
                //         }
                //     }

                //     close_exits_flag = true;

                // } else if (3 == trans && false == split_exits_flag && num_idx > 0) {
                //     // Splitting
                //     split_exits_flag = true;
                //     lane_element.split_position = num_idx;
                //     lane_element.split_offset = link_infos.PathOffset.PathOffset;
                //     lane_element.is_virtually_split = true;
                //     if (IsNoVirtually(link_infos, lane_element.lane_num_vec[num_idx], trans, temp_split_from,
                //                       temp_close_to)) {
                //         lane_element.is_virtually_split = false;
                //         lane_element.split_from = temp_split_from;
                //     }
                //     if (0 == temp_split_from) {
                //         GetSplitDir(lane_element.lane_num_vec, num_idx, lane_element.split_from,
                //                     lane_element_group_sets);
                //     } else {
                //         lane_element.split_from = temp_split_from;
                //     }
                // }else if (3 == trans && true == split_exits_flag && num_idx > 0) {
                //     // Splitting
                //     if (link_infos.PathOffset.PathOffset > lane_element.split_offset + CONTINUOUS_DIVER_LENGTH ||
                //         0 == lane_element.split_offset){
                //         continue;
                //     }
                //     lane_element.next_split_position = num_idx;
                // }
            }

            // last link is split
            lane_element.org_rest_length = lane_element.rest_length;
            std::vector<uint8_t> last_link_route_lanes;
            uint32_t last_link_id = link_id_vec[lane_element.lane_num_vec.size() - 1];
            uint8_t last_lane_id = lane_element.lane_num_vec[lane_element.lane_num_vec.size() - 1];

            std::vector<uint8_t> next_link_route_lanes;
            uint32_t next_link_id = 0;
            if (false == GetNextLinkIDForRoute(last_link_id, next_link_id)) {
                continue;
            }

            double temp_rest_length = 0.0;
            uint8_t loop_times = 0;

            while (loop_times < 100) {
                GetConnectLinkLanes(next_link_id, last_link_id, last_link_route_lanes, next_link_route_lanes);
                if (IsChangeToRouteLane(last_link_id, last_lane_id, last_link_route_lanes)) {
                    break;
                }

                next_link_route_lanes.clear();
                next_link_route_lanes.assign(last_link_route_lanes.begin(), last_link_route_lanes.end());
                last_link_route_lanes.clear();
                next_link_id = last_link_id;
                uint8_t next_lane_id = last_lane_id;
                if (false == GetBackLinkLane(next_link_id, next_lane_id, last_link_id, last_lane_id)) {
                    if (next_link_id == link_id_vec[0]){
                        lane_element.first_lane_is_no_route = true;
                    }
                    
                    break;
                }

                message::map_map::s_LinkInfo_t link_infos;
                if (false == GetLinkInfos(next_link_id, link_infos)) {
                    // static data none
                    return true;
                }
                double link_length = (link_infos.EndOffset.EndOffset - link_infos.PathOffset.PathOffset) / 100.0;
                temp_rest_length += link_length;
                loop_times++;
            }
            lane_element.rest_length =
                lane_element.rest_length > temp_rest_length ? lane_element.rest_length - temp_rest_length : 0;
        }
    }

    return true;
}

bool CAlgoRefLaneNode::GetRefLanesForLength(LaneElementGroupSets& lane_element_group_sets,
                                            std::vector<std::vector<int32_t>>& ref_lanes) {
    double max_length = -1.0;
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            double lane_length = lane_element_group_sets[lane_idx][lane_dir_idx].rest_length;
            max_length = max_length > lane_length ? max_length : lane_length;
        }
    }

    if (max_length < 0) {
        return false;
    }

    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            double lane_length = lane_element_group_sets[lane_idx][lane_dir_idx].rest_length;
            if (fabs(max_length - lane_length) < DOUBLE_DATA_RANGE) {
                lane_element_group_sets[lane_idx][lane_dir_idx].is_dest = true;
                std::vector<int32_t> temp_lane_idx;
                temp_lane_idx.push_back(lane_idx);
                temp_lane_idx.push_back(lane_dir_idx);
                ref_lanes.push_back(temp_lane_idx);
            }
        }
    }

    return true;
}

bool CAlgoRefLaneNode::GetRefLanesForRoute(std::vector<uint32_t> link_id_vec,
                                           LaneElementGroupSets& lane_element_group_sets,
                                           std::vector<std::vector<int32_t>>& ref_lanes) {
    if (ref_lanes.size() < 1) {  // no ref lane
        return false;
    }

    auto lanes_vec = lane_element_group_sets[ref_lanes[0][0]][ref_lanes[0][1]].lane_num_vec;
    if (link_id_vec.size() < lanes_vec.size()) {  // no data
        return false;
    }

    uint8_t last_lane_id = lanes_vec[lanes_vec.size() - 1];
    uint32_t last_link_id = link_id_vec[lanes_vec.size() - 1];

    uint32_t next_link_id = 0;
    if (false == GetNextLinkIDForRoute(last_link_id, next_link_id)) {
        // last link
        return true;
    }

    message::map_map::s_LinkInfo_t next_link_infos;
    if (false == GetLinkInfos(next_link_id, next_link_infos)) {
        // static data none
        return true;
    }

    std::vector<uint8_t> connect_next_link_lanes;
    std::vector<uint8_t> last_link_lanes;
    GetConnectLinkLanes(next_link_id, last_link_id, connect_next_link_lanes, last_link_lanes);
    if (0 == connect_next_link_lanes.size()) {
        // no connect
        return true;
    }

    uint8_t min_step = 100;
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            if (false == lane_element.is_dest) {
                continue;
            }

            uint8_t temp_min_step = 100;
            GetMinStepToRoute(lane_element.lane_num_vec, connect_next_link_lanes, temp_min_step);
            min_step = min_step > temp_min_step ? temp_min_step : min_step;
            lane_element.min_step = temp_min_step;
        }
    }
    if (100 == min_step) {
        // no connect
        return true;
    }

    //
    ref_lanes.clear();
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            if (false == lane_element.is_dest) {
                continue;
            }

            if (lane_element.min_step > min_step) {
                lane_element.is_dest = false;
                lane_element.min_step = 0;
                continue;
            }

            std::vector<int32_t> temp_lane_idx;
            temp_lane_idx.push_back(lane_idx);
            temp_lane_idx.push_back(lane_dir_idx);
            ref_lanes.push_back(temp_lane_idx);
        }
    }

    return true;
}

/**
 * @brief get a link's link infos
 *
 * @return true
 * @return false
 */
bool CAlgoRefLaneNode::GetLinkInfos(uint32_t link_id, message::map_map::s_LinkInfo_t& link_infos) {
    if (link_id_index_lane_info_map_->find(link_id) != link_id_index_lane_info_map_->end()) {
        int link_index = link_id_index_lane_info_map_->at(link_id);
        link_infos = map_static_info_->LinkInfos.LinkInfos[link_index];
        return true;
    } else {
        return false;
    }
}

bool CAlgoRefLaneNode::IsCLoseLink(uint32_t link_id) {
    if (to_link_id_index_lane_connect_map_->find(link_id) != to_link_id_index_lane_connect_map_->end()) {
        uint32_t temp_link_id = 0;
        for (auto idx : to_link_id_index_lane_connect_map_->at(link_id)) {
            if (0 == temp_link_id) {
                temp_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId;
            } else {
                if (temp_link_id != map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId) {
                    return true;
                }
            }
        }
    }

    return false;
}

bool CAlgoRefLaneNode::IsSplitLink(uint32_t link_id) {
    if (link_id_index_lane_connect_map_->find(link_id) != link_id_index_lane_connect_map_->end()) {
        uint32_t temp_link_id = 0;
        for (auto idx : link_id_index_lane_connect_map_->at(link_id)) {
            if (0 == temp_link_id) {
                temp_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].ToLinkId.ToLink;
            } else {
                if (temp_link_id != map_static_info_->LaneConnectivitys.PairConnectivity[idx].ToLinkId.ToLink) {
                    return true;
                }
            }
        }
    }

    return false;
}

bool CAlgoRefLaneNode::IsSplitLinkLane(uint32_t link_id, uint8_t& cur_lane_id, uint8_t& side_lane_id) {
    uint32_t cur_next_link_id = 0;
    uint32_t side_next_link_id = 0;
    uint8_t cur_next_lane_id = 0; 
    uint8_t side_next_lane_id = 0;
    if (link_id_index_lane_connect_map_->find(link_id) != link_id_index_lane_connect_map_->end()) {
        for (auto idx : link_id_index_lane_connect_map_->at(link_id)) {
            if (map_static_info_->LaneConnectivitys.PairConnectivity[idx].InitLaneNum.InitLaneNum == cur_lane_id){
                cur_next_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].ToLinkId.ToLink;
                cur_next_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum;
            }
            
            if (map_static_info_->LaneConnectivitys.PairConnectivity[idx].InitLaneNum.InitLaneNum == side_lane_id){
                side_next_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].ToLinkId.ToLink;
                side_next_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum;
            }
        }
    }

    if (cur_next_link_id != side_next_link_id && cur_next_link_id > 0 && side_next_link_id > 0){
        return true;
    }

    if (cur_next_link_id == side_next_link_id && cur_next_link_id > 0 && side_next_link_id > 0){
        cur_lane_id = cur_next_lane_id;
        side_lane_id = side_next_lane_id;
    }else{
        cur_lane_id = 0;
        side_lane_id = 0;
    }
    

    return false;
}

bool CAlgoRefLaneNode::GetLaneTrans(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, uint8_t& trans) {
    trans = 0;
    for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
        if (lane_id == laneinfo.LaneNum.LaneNum) {

            if (0 == laneinfo.Transit.data || 99 == laneinfo.Transit.data){
                trans = 2;
            }else{
                trans = laneinfo.Transit.data;
            }
            
        }
    }

    return true;
}

bool CAlgoRefLaneNode::GetLaneType(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, uint8_t& lane_type) {
    lane_type = 0;
    for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
        if (lane_id == laneinfo.LaneNum.LaneNum) {
            lane_type = laneinfo.LaneType.data;
        }
    }

    return true;
}

bool CAlgoRefLaneNode::GetSideLane(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, 
                                   uint32_t next_link_id, uint8_t next_lane_id,
                                   uint8_t& side_lane_id) {
    side_lane_id = 0;
    for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
        uint32_t temp_next_link_id = 0;
        uint8_t  temp_next_lane_id = 0;
        GetConnectLinkLane(link_infos.InstanceId.InstanceId, temp_next_link_id, 
                            laneinfo.LaneNum.LaneNum, temp_next_lane_id);
        if (temp_next_link_id == next_link_id && temp_next_lane_id == next_lane_id &&
            ((lane_id + 1) == laneinfo.LaneNum.LaneNum || lane_id == (laneinfo.LaneNum.LaneNum + 1))){
            side_lane_id = laneinfo.LaneNum.LaneNum;
            return true;
        }
    }
    if (0 == side_lane_id){
        return false;
    }
    
    return true;
}

bool CAlgoRefLaneNode::GetSideLaneForSplit(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, 
                                   uint8_t& side_lane_id) {
    side_lane_id = 0;
    uint32_t back_link_id = 0;
    uint8_t  back_lane_id = 0;
    if (false == GetBackLinkLaneForConnect(link_infos.InstanceId.InstanceId, lane_id, back_link_id, back_lane_id)){
        return false;
    }
    
    for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
        if (lane_id == laneinfo.LaneNum.LaneNum){
            continue;
        }
        
        uint32_t temp_back_link_id = 0;
        uint8_t  temp_back_lane_id = 0;
        GetBackLinkLaneForConnect(link_infos.InstanceId.InstanceId, laneinfo.LaneNum.LaneNum, temp_back_link_id, temp_back_lane_id);
        if (temp_back_link_id == back_link_id && temp_back_lane_id == back_lane_id &&
            ((lane_id + 1) == laneinfo.LaneNum.LaneNum || lane_id == (laneinfo.LaneNum.LaneNum + 1))){
            side_lane_id = laneinfo.LaneNum.LaneNum;
            return true;
        }
    }
    if (0 == side_lane_id){
        return false;
    }
    
    return true;
}

bool CAlgoRefLaneNode::isNotMergeLane(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id) {
    for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
        uint8_t temp_trans;
        GetLaneTrans(link_infos, laneinfo.LaneNum.LaneNum, temp_trans);
        if ((lane_id + 1) == laneinfo.LaneNum.LaneNum && 2 == temp_trans) {
            uint8_t left_line_type = 0;
            uint8_t right_line_type = 0;
            uint32_t temp_line_id = 0;
            GetLineType(link_infos, lane_id, left_line_type, true, temp_line_id);
            GetLineType(link_infos, lane_id + 1, right_line_type, false, temp_line_id);

            if (8 != left_line_type && 8 == right_line_type) {
                return true;
            }

        } else if (lane_id == (laneinfo.LaneNum.LaneNum + 1) && 2 == temp_trans) {
            uint8_t left_line_type = 0;
            uint8_t right_line_type = 0;
            uint32_t temp_line_id = 0;
            GetLineType(link_infos, lane_id - 1, left_line_type, true, temp_line_id);
            GetLineType(link_infos, lane_id, right_line_type, false, temp_line_id);

            if (8 == left_line_type && 8 != right_line_type) {
                return true;
            }
        }
    }

    return false;
}

bool CAlgoRefLaneNode::IsNoVirtually(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, uint8_t& trans,
                                     uint8_t& split_from, uint8_t& close_to) {
    split_from = 0;
    close_to = 0;
    for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
        uint8_t temp_trans;
        GetLaneTrans(link_infos, laneinfo.LaneNum.LaneNum, temp_trans);
        if ((lane_id + 1) == laneinfo.LaneNum.LaneNum && trans == temp_trans) {
            uint8_t left_line_type = 0;
            uint8_t right_line_type = 0;
            uint32_t temp_line_id = 0;
            GetLineType(link_infos, lane_id, left_line_type, true, temp_line_id);
            GetLineType(link_infos, lane_id + 1, right_line_type, false, temp_line_id);

            if (2 == trans) {
                close_to = 1;
            } else {
                split_from = 2;
            }

            if (8 == left_line_type && 8 == right_line_type) {
                return false;
            }

        } else if (lane_id == (laneinfo.LaneNum.LaneNum + 1) && trans == temp_trans) {
            uint8_t left_line_type = 0;
            uint8_t right_line_type = 0;
            uint32_t temp_line_id = 0;
            GetLineType(link_infos, lane_id - 1, left_line_type, true, temp_line_id);
            GetLineType(link_infos, lane_id, right_line_type, false, temp_line_id);

            if (2 == trans) {
                close_to = 2;
            } else {
                split_from = 1;
            }

            if (8 == left_line_type && 8 == right_line_type) {
                return false;
            }
        }
    }

    uint32_t next_link_id = 0;
    uint8_t next_lane_id = 0;
    GetConnectLinkLane(link_infos.InstanceId.InstanceId, next_link_id, lane_id, next_lane_id);
    if (0 == next_link_id || 0 == next_lane_id) {
        return true;
    }

    for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
        if ((lane_id + 1) == laneinfo.LaneNum.LaneNum) {
            uint32_t temp_next_link_id = 0;
            uint8_t temp_next_lane_id = 0;
            GetConnectLinkLane(link_infos.InstanceId.InstanceId, temp_next_link_id, laneinfo.LaneNum.LaneNum,
                               temp_next_lane_id);
            if (temp_next_link_id == next_link_id && temp_next_lane_id == next_lane_id) {
                if (2 == trans) {
                    close_to = 1;
                } else {
                    split_from = 2;
                }
                break;
            }
        }

        if (lane_id == (laneinfo.LaneNum.LaneNum + 1)) {
            uint32_t temp_next_link_id = 0;
            uint8_t temp_next_lane_id = 0;
            GetConnectLinkLane(link_infos.InstanceId.InstanceId, temp_next_link_id, laneinfo.LaneNum.LaneNum,
                               temp_next_lane_id);
            if (temp_next_link_id == next_link_id && temp_next_lane_id == next_lane_id) {
                if (2 == trans) {
                    close_to = 2;
                } else {
                    split_from = 1;
                }
                break;
            }
        }
    }

    return true;
}

bool CAlgoRefLaneNode::GetLineType(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, uint8_t& line_type,
                                   bool is_left, uint32_t& line_id) {
    line_id = 0;
    for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
        if (lane_id == laneinfo.LaneNum.LaneNum) {
            if (is_left) {
                line_id = laneinfo.LBound.LBound;
            } else {
                line_id = laneinfo.RBound.RBound;
            }
            break;
        }
    }

    line_type = 0;
    for (auto& lineinfo : map_static_info_->LinearObjects.LinearObjects) {
        if (line_id == lineinfo.IDLinearObject.IDLinearObject) {
            line_type = lineinfo.LinearObjectMarking.data;
            break;
        }
    }

    return true;
}

bool CAlgoRefLaneNode::GetNextLinkIDForRoute(uint32_t link_id, uint32_t& next_link_id) {
    next_link_id = 0;
    int link_idx = -1;
    for (int i = 0; i < map_route_list_->LinkIds.LinkIds.size(); i++) {
        if (map_route_list_->LinkIds.LinkIds[i].LinkId == link_id) {
            link_idx = i;
            break;
        }
    }
    if (link_idx >= 0 && link_idx < (map_route_list_->LinkIds.LinkIds.size() - 1)) {
        next_link_id = map_route_list_->LinkIds.LinkIds[link_idx + 1].LinkId;
    }

    if (0 == next_link_id) {
        // link_id is last link
        return false;
    }

    return true;
}

bool CAlgoRefLaneNode::GetBackLinkLane(uint32_t link_id, uint8_t lane_id, uint32_t& back_link_id,
                                       uint8_t& back_lane_id) {
    back_link_id = 0;
    back_lane_id = 0;
    int link_idx = -1;
    for (int i = 0; i < link_id_vec_.size(); i++) {
        if (link_id_vec_[i] == link_id) {
            link_idx = i;
            break;
        }
    }
    if (link_idx > 0) {
        back_link_id = link_id_vec_[link_idx - 1];
        if (to_link_id_index_lane_connect_map_->find(link_id) != to_link_id_index_lane_connect_map_->end()) {
            uint32_t temp_link_id = 0;
            for (auto idx : to_link_id_index_lane_connect_map_->at(link_id)) {
                uint32_t temp_back_link_id =
                    map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId;
                uint8_t temp_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum;
                if (temp_back_link_id == back_link_id && lane_id == temp_lane_id) {
                    back_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].InitLaneNum.InitLaneNum;
                    break;
                }
            }
        }
    }

    if (0 == back_link_id || 0 == back_lane_id) {
        // link_id is last link
        return false;
    }

    return true;
}

bool CAlgoRefLaneNode::GetBackLinkLaneForConnect(uint32_t link_id, uint8_t lane_id, uint32_t& back_link_id,
                                       uint8_t& back_lane_id) {
    back_link_id = 0;
    back_lane_id = 0;

    if (to_link_id_index_lane_connect_map_->find(link_id) != to_link_id_index_lane_connect_map_->end()) {
        uint32_t temp_link_id = 0;
        for (auto idx : to_link_id_index_lane_connect_map_->at(link_id)) {
            uint32_t temp_link_id =
                map_static_info_->LaneConnectivitys.PairConnectivity[idx].ToLinkId.ToLink;
            uint8_t temp_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum;
            if (temp_link_id == link_id && lane_id == temp_lane_id) {
                back_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId;
                back_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].InitLaneNum.InitLaneNum;
                break;
            }
        }
    }

    if (0 == back_link_id || 0 == back_lane_id) {
        // link_id is last link
        return false;
    }

    return true;
}

bool CAlgoRefLaneNode::GetConnectLinkLanes(uint32_t next_link_id, uint32_t link_id,
                                           std::vector<uint8_t>& connect_next_link_lanes,
                                           std::vector<uint8_t>& last_link_lanes) {
    message::map_map::s_LinkInfo_t link_infos;
    if (false == GetLinkInfos(link_id, link_infos)) {
        // static data none
        return false;
    }

    std::vector<int> link_staticmap_indices{};
    if (to_link_id_index_lane_connect_map_->find(next_link_id) != to_link_id_index_lane_connect_map_->end()) {
        link_staticmap_indices = to_link_id_index_lane_connect_map_->at(next_link_id);
        for (auto index : link_staticmap_indices) {
            if (link_id == map_static_info_->LaneConnectivitys.PairConnectivity[index].FromLinkId.FromLinkId &&
                next_link_id == map_static_info_->LaneConnectivitys.PairConnectivity[index].ToLinkId.ToLink) {
                uint8_t lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum;
                uint8_t new_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum;
                uint8_t temp_lane_type = 0;
                GetLaneType(link_infos, lane_id, temp_lane_type);
                if (3 == temp_lane_type || 9 == temp_lane_type || 17 == temp_lane_type) {
                    continue;
                }

                if (0 == last_link_lanes.size()) {
                    connect_next_link_lanes.push_back(lane_id);
                } else {
                    if (std::find(last_link_lanes.begin(), last_link_lanes.end(), new_lane_id) != last_link_lanes.end()) {
                        connect_next_link_lanes.push_back(lane_id);
                    }
                }
            }
        }
    }

    return true;
}

bool CAlgoRefLaneNode::GetConnectLinkLane(uint32_t link_id, uint32_t& next_link_id, uint8_t lane_id,
                                          uint8_t& next_lane_id) {
    next_link_id = 0;
    next_lane_id = 0;
    if (link_id_index_lane_connect_map_->find(link_id) != link_id_index_lane_connect_map_->end()) {
        uint32_t temp_link_id = 0;
        for (auto idx : link_id_index_lane_connect_map_->at(link_id)) {
            if (lane_id == map_static_info_->LaneConnectivitys.PairConnectivity[idx].InitLaneNum.InitLaneNum &&
                link_id == map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId) {
                next_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].ToLinkId.ToLink;
                next_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum;
                break;
            }
        }
    }

    return true;
}

bool CAlgoRefLaneNode::GetMinStepToRoute(std::vector<uint8_t> lanes_vec, std::vector<uint8_t> connect_next_link_lanes,
                                         uint8_t& temp_min_step) {
    auto lane = lanes_vec[lanes_vec.size() - 1];
    temp_min_step = 100;
    for (size_t i = 0; i < connect_next_link_lanes.size(); i++)
    {
        int32_t step = abs(connect_next_link_lanes[i] - lane);
        temp_min_step = temp_min_step > step ? step : temp_min_step;
    }
    if (100 == temp_min_step){
        return false;
    }

    return true;
}

bool CAlgoRefLaneNode::GetRefLanesForClose(std::vector<uint32_t> link_id_vec,
                                           LaneElementGroupSets& lane_element_group_sets,
                                           std::vector<std::vector<int32_t>>& ref_lanes) {
    if (ref_lanes.size() <= 1) {
        return true;
    } 

    uint32_t min_close = 100;
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            if (false == lane_element.is_dest) {
                continue;
            }
            min_close = min_close > lane_element.close_times ? lane_element.close_times : min_close;
        }
    }

    ref_lanes.clear();
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            if (false == lane_element.is_dest) {
                continue;
            }

            if (lane_element.close_times > min_close) {
                lane_element.is_dest = false;
                lane_element.is_close_dest = true;
            } else {
                std::vector<int32_t> temp_lane_idx;
                temp_lane_idx.push_back(lane_idx);
                temp_lane_idx.push_back(lane_dir_idx);
                ref_lanes.push_back(temp_lane_idx);
            }
        }
    }

    if (false == GetRefLanesForFirstVirtuallyMerge(link_id_vec, lane_element_group_sets, ref_lanes)) {
        // LOGE("get DeleteSplitLane error!");
        return true;
    } 

    return true;
}

bool CAlgoRefLaneNode::DeleteSplitLane(std::vector<uint32_t> link_id_vec, LaneElementGroupSets& lane_element_group_sets,
                                       std::vector<std::vector<int32_t>>& ref_lanes) {
    if (0 == ref_lanes.size()) {
        return false;
    }
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //         << " ref_lanes.size: " << ref_lanes.size() << std::endl;
    uint32_t max_lane_number = lane_element_group_sets[ref_lanes[0][0]][ref_lanes[0][1]].lane_num_vec.size();
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //         << " max_lane_number: " << max_lane_number << std::endl;
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        if (lane_element_group_sets[lane_idx].size() <= 1) {
            continue;
        }

        DeleteSplitLaneForOne(link_id_vec, lane_element_group_sets[lane_idx], max_lane_number);
    }

    return true;
}

bool CAlgoRefLaneNode::GetRefLanesForFirstVirtuallyMerge(std::vector<uint32_t> link_id_vec, 
                                                         LaneElementGroupSets& lane_element_group_sets,
                                                         std::vector<std::vector<int32_t>>& ref_lanes) {
    if (0 == ref_lanes.size()) {
        return false;
    }

    if (1 == ref_lanes.size()) {
        return true;
    }

    for (size_t s_idx = 0; s_idx < (ref_lanes.size() - 1); s_idx++){
        auto& s_element = lane_element_group_sets[ref_lanes[s_idx][0]][ref_lanes[s_idx][1]];
        if (false == s_element.is_dest){
            continue;
        }

        for (size_t e_idx = s_idx + 1; e_idx < ref_lanes.size(); e_idx++){
            auto& e_element = lane_element_group_sets[ref_lanes[e_idx][0]][ref_lanes[e_idx][1]];
            if (false == e_element.is_dest){
                continue;
            }
            
            if (IsFirstMergeVirtually(s_element, e_element)){
                DeleteFormRefForFirstMergeVirtually(link_id_vec, s_element, e_element);
            }
        }
    }

    ref_lanes.clear();
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            if (false == lane_element.is_dest) {
                continue;
            }
            std::vector<int32_t> temp_lane_idx;
            temp_lane_idx.push_back(lane_idx);
            temp_lane_idx.push_back(lane_dir_idx);
            ref_lanes.push_back(temp_lane_idx);
        }
    }
    
    return true;
}

bool CAlgoRefLaneNode::IsFirstMergeVirtually(LaneElement& s_element, LaneElement& e_element) {

    if (s_element.lane_num_vec.size() != e_element.lane_num_vec.size()){
        return false;
    }

    uint32_t min_lane_num = s_element.lane_num_vec.size();
    if (s_element.close_position == e_element.close_position && s_element.close_position < (min_lane_num - 1) &&
        true == s_element.is_virtually_close && true == e_element.is_virtually_close){
        if ((s_element.lane_num_vec[s_element.close_position + 1] == e_element.lane_num_vec[e_element.close_position + 1]) &&
            (s_element.lane_num_vec[s_element.close_position] != e_element.lane_num_vec[e_element.close_position]) &&
            (s_element.lane_extra_infos[s_element.close_position].e_offset >= POSITION_TO_MERGE_DIST)){
            return true;
        }
    }

    return false;
}

bool CAlgoRefLaneNode::DeleteFormRefForFirstMergeVirtually(std::vector<uint32_t> link_id_vec,
                                                           LaneElement& s_element, 
                                                           LaneElement& e_element) {
    //close                                                        
    uint16_t next_close_to = 0;
    for (size_t i = s_element.close_position + 1; i < s_element.lane_num_vec.size(); i++){
        if ((EFM_MergeType_NONE == s_element.lane_extra_infos[i].merge_value) || 
            (EFM_MergeType_FROM_LEFT == s_element.lane_extra_infos[i].merge_value) || 
            (EFM_MergeType_FROM_RIGHT == s_element.lane_extra_infos[i].merge_value)){
            continue;
        }

        if ((EFM_MergeType_TO_LEFT == s_element.lane_extra_infos[i].merge_value) || 
            (EFM_MergeType_LEFT_TO_MIDDLE == s_element.lane_extra_infos[i].merge_value)){
            next_close_to = 1;
        }else{
            next_close_to = 2;
        }
        break;
    }

    //route
    if (0 == next_close_to){
        uint32_t cur_link_id = link_id_vec[s_element.lane_num_vec.size() -1];
        uint32_t next_link_id = 0;
        if (true == GetNextLinkIDForRoute(cur_link_id, next_link_id)) {
            message::map_map::s_LinkInfo_t next_link_infos;
            if (true == GetLinkInfos(next_link_id, next_link_infos)) {
                std::vector<uint8_t> connect_next_link_lanes;
                std::vector<uint8_t> last_link_lanes;
                GetConnectLinkLanes(next_link_id, cur_link_id, connect_next_link_lanes, last_link_lanes);
                if (connect_next_link_lanes.size() > 0 && last_link_lanes.size() > 0) {
                    if (last_link_lanes[0] > s_element.lane_num_vec[s_element.lane_num_vec.size() -1]){
                        next_close_to = 1;
                    }else{
                        next_close_to = 2;
                    }
                }
            }
        }
    }

    //side
    message::map_map::s_LinkInfo_t link_infos;
    if (false == GetLinkInfos(link_id_vec[s_element.close_position], link_infos)){
        return false;
    }
    uint8_t s_lane_id = s_element.lane_num_vec[s_element.close_position];
    uint8_t e_lane_id = e_element.lane_num_vec[s_element.close_position];
    double  temp_d_length = 0.0;
    if (s_element.lane_extra_infos.size() > s_element.close_position){
        temp_d_length = (static_cast<double>(s_element.lane_extra_infos[s_element.close_position].e_offset))/100.0;
    }

    {
        if (0 == next_close_to){
            uint8_t max_lane_id = std::max(s_lane_id, e_lane_id);
            uint8_t min_lane_id = std::min(s_lane_id, e_lane_id);
            bool max_flag = false;
            bool min_flag = false;
            for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
                if (max_lane_id < laneinfo.LaneNum.LaneNum && 
                    (3 != laneinfo.LaneType.data && 9 != laneinfo.LaneType.data && 17 != laneinfo.LaneType.data)) {
                    max_flag = true;
                    continue;
                }

                if (min_lane_id > laneinfo.LaneNum.LaneNum && 
                    (3 != laneinfo.LaneType.data && 9 != laneinfo.LaneType.data && 17 != laneinfo.LaneType.data)) {
                    min_flag = true;
                    continue;
                }
            }

            if (true == max_flag && false == min_flag){
                next_close_to = 1;
            }else if (false == max_flag && true == min_flag){
                next_close_to = 2;
            }
        }
        
    }


    //jude
    if (0 == next_close_to){
        return true;
    }

    uint8_t lane_id = 0;

    uint32_t left_line = 0;
    uint8_t  left_line_type = 0;
    uint32_t right_line = 0;
    uint8_t  right_line_type = 0;

    if (1 == next_close_to){
        lane_id = std::max(s_lane_id, e_lane_id);
        GetLineType(link_infos, lane_id, left_line_type, true, left_line);
        GetLineType(link_infos, lane_id + 1, right_line_type, false, right_line);

        if ((3 == left_line_type || 5 == left_line_type || 6 == left_line_type || 8 == left_line_type) && 
            (3 == right_line_type || 5 == right_line_type || 6 == right_line_type || 8 == right_line_type)){
            if (s_lane_id > e_lane_id){
                e_element.is_dest = false;
                e_element.rest_length = temp_d_length;
            }else{
                s_element.is_dest = false;
                s_element.rest_length = temp_d_length;
            }
        }
        
    }else{
        lane_id = std::min(s_lane_id, e_lane_id);
        if (lane_id <= 1){
            return true;
        }
        
        GetLineType(link_infos, lane_id, right_line_type, false, right_line);
        GetLineType(link_infos, lane_id - 1, left_line_type, true, left_line);

        if ((3 == left_line_type || 5 == left_line_type || 7 == left_line_type || 8 == left_line_type) && 
            (3 == right_line_type || 5 == right_line_type || 7 == right_line_type || 8 == right_line_type)){
            if (s_lane_id > e_lane_id){
                s_element.is_dest = false;
                s_element.rest_length = temp_d_length;
            }else{
                e_element.is_dest = false;
                e_element.rest_length = temp_d_length;
            }
        } 
    }

    return true;
}

bool CAlgoRefLaneNode::MakeRefLine(std::vector<uint32_t> link_id_vec, LaneElementGroupSets& lane_element_group_sets,
                                   std::vector<std::vector<int32_t>>& ref_lanes, int32_t& ego_lane_index,
                                   int32_t& left_lane_idx, int32_t& left_left_lane_idx, int32_t& right_lane_idx, 
                                   int32_t& right_right_lane_idx, int32_t& ref_lane_idx) {
    ego_group_index_ = -1;
    left_group_idx_  = -1;
    right_group_idx_ = -1;
    left_left_group_idx_ = -1;
    right_right_group_idx_ = -1;
    //std::cout << __FILE__ << "," << __LINE__ << "," << " left_left_group_idx_: " << left_left_group_idx_ << std::endl;
    if (false == GetEgpGroupIdx(link_id_vec, lane_element_group_sets, ego_group_index_)) {
        ego_lane_index = -1;
        return false;
    }

    ego_lane_index = 1;
    left_lane_idx = -1;
    left_left_lane_idx = -1;
    right_lane_idx = -1;
    right_right_lane_idx = -1;
    GetLaneCandidateIndexEgo(lane_element_group_sets[ego_group_index_], ego_lane_index, left_lane_idx, right_lane_idx, link_id_vec);
    if (-1 == ego_lane_index){
        return false;
    }

    //jude
    ego_left_same_lane_ = false;
    ego_right_same_lane_ = false;
    if (left_lane_idx >= 0){
        left_group_idx_ = ego_group_index_;
        ego_left_same_lane_ = true;
    }else if (right_lane_idx >= 0){
        right_group_idx_ = ego_group_index_;
        ego_right_same_lane_ = true;
    }   
    
    //get
    if (ego_left_same_lane_){
        if ((ego_group_index_ + 1) < lane_element_group_sets.size()){
            left_left_group_idx_ = ego_group_index_ + 1;
            GetLaneCandidateIndex(lane_element_group_sets[left_left_group_idx_], left_left_lane_idx);
        }
    }else{
        if ((ego_group_index_ + 1) < lane_element_group_sets.size()){
            left_group_idx_ = ego_group_index_ + 1;
            GetLaneCandidateIndex(lane_element_group_sets[left_group_idx_], left_lane_idx);
        } 
        if ((ego_group_index_ + 2) < lane_element_group_sets.size()){
            left_left_group_idx_ = ego_group_index_ + 2;
            GetLaneCandidateIndex(lane_element_group_sets[left_left_group_idx_], left_left_lane_idx);
        } 
    }

    if (ego_right_same_lane_){
        if (ego_group_index_ > 0){
            right_right_group_idx_ = ego_group_index_ - 1;
            GetLaneCandidateIndex(lane_element_group_sets[right_right_group_idx_], right_right_lane_idx);
        }
    }else{
        if (ego_group_index_ > 0){
            right_group_idx_ = ego_group_index_ - 1;
            GetLaneCandidateIndex(lane_element_group_sets[right_group_idx_], right_lane_idx);
        } 
        if (ego_group_index_ > 1){
            right_right_group_idx_ = ego_group_index_ - 2;
            GetLaneCandidateIndex(lane_element_group_sets[right_right_group_idx_], right_right_lane_idx);
        } 
    }
    
    // if (-1 == left_lane_idx && ego_group_idx < (lane_element_group_sets.size() - 1)){
    //     //left_lane_idx = lane_element_group_sets[ego_group_idx + 1][0].candidate_index;
    //     GetLaneCandidateIndex(lane_element_group_sets[ego_group_idx + 1], left_lane_idx);
    // }

    // if (-1 == right_lane_idx && ego_group_idx > 0){
    //     // int32_t right_lane_num = lane_element_group_sets[ego_group_idx - 1].size();
    //     // right_lane_idx = lane_element_group_sets[ego_group_idx - 1][right_lane_num - 1].candidate_index;
    //     GetLaneCandidateIndex(lane_element_group_sets[ego_group_idx - 1], right_lane_idx);
    // }

    GetRefLaneCandidateIndex(lane_element_group_sets, ego_lane_index, left_lane_idx, left_left_lane_idx, 
                             right_lane_idx, right_right_lane_idx, ref_lane_idx, ref_lanes, link_id_vec);
    //std::cout << __FILE__ << "," << __LINE__ << "," << " left_left_lane_idx: " << left_left_lane_idx << std::endl;

    return true;
}

bool CAlgoRefLaneNode::DeleteSplitLaneForOne(std::vector<uint32_t> link_id_vec, LaneElementGroup& lane_element_group,
                                             uint32_t max_lane_number) {
    if (lane_element_group.size() == 0){
        return false;
    }

    if (lane_element_group.size() == 1){
        lane_element_group[0].is_group_dest = true;
        return true;
    }
    
    std::vector<int32_t> dest_lane_idxs;
    std::vector<int32_t> close_dest_lane_idxs;
    std::vector<int32_t> max_lane_number_idxs;
    std::vector<int32_t> all_lane_idxs;
    for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group.size(); lane_dir_idx++) {
        lane_element_group[lane_dir_idx].is_group_dest = false;
        if (lane_element_group[lane_dir_idx].is_dest) {
            dest_lane_idxs.push_back(lane_dir_idx);
        }

        if (lane_element_group[lane_dir_idx].is_close_dest) {
            close_dest_lane_idxs.push_back(lane_dir_idx);
        }

        if (lane_element_group[lane_dir_idx].lane_num_vec.size() == max_lane_number) {
            max_lane_number_idxs.push_back(lane_dir_idx);
        }

        all_lane_idxs.push_back(lane_dir_idx);
    }

    int32_t real_lane_idx = -1;
    if (0 == dest_lane_idxs.size() && 0 == close_dest_lane_idxs.size() && 0 == max_lane_number_idxs.size()) {
        // std::cout << __FILE__ << "," << __LINE__ << "," << " lane_element_group.size: " << lane_element_group.size()
        // << std::endl;
        GetDestElementGroup(link_id_vec, lane_element_group, all_lane_idxs, real_lane_idx);
    } else if (dest_lane_idxs.size() > 0) {
        // std::cout << __FILE__ << "," << __LINE__ << "," << " lane_element_group.size: " << lane_element_group.size()
        // << std::endl;
        GetDestElementGroup(link_id_vec, lane_element_group, dest_lane_idxs, real_lane_idx);
    } else if (close_dest_lane_idxs.size() > 0) {
        // std::cout << __FILE__ << "," << __LINE__ << "," << " lane_element_group.size: " << lane_element_group.size()
        // << std::endl;
        GetDestElementGroup(link_id_vec, lane_element_group, close_dest_lane_idxs, real_lane_idx);
    } else {
        // std::cout << __FILE__ << "," << __LINE__ << "," << " lane_element_group.size: " << lane_element_group.size()
        // << std::endl;
        GetDestElementGroup(link_id_vec, lane_element_group, max_lane_number_idxs, real_lane_idx);
    }
    // std::cout << __FILE__ << "," << __LINE__ << "," << " lane_element_group.size: " << lane_element_group.size() <<
    // std::endl;
    lane_element_group[real_lane_idx].is_group_dest = true;

    return true;
}

bool CAlgoRefLaneNode::GetDestElementGroup(std::vector<uint32_t> link_id_vec, LaneElementGroup& lane_element_group,
                                           std::vector<int32_t> lane_idxs, int32_t& real_lane_idx) {
    real_lane_idx = 0;
    if (lane_idxs.empty()){
        return false;
    }

    real_lane_idx = lane_idxs[0];
    if (1 == lane_idxs.size()) { 
        return true;
    }

    // for len
    int32_t max_leng = -1;
    for (auto& idx : lane_idxs) {
        int32_t temp_leng = lane_element_group[idx].lane_num_vec.size();
        max_leng = temp_leng > max_leng ? temp_leng : max_leng;
    }

    std::vector<int32_t> max_lane_lane_idxs;
    for (auto& idx : lane_idxs) {
        if (lane_element_group[idx].lane_num_vec.size() == max_leng) {
            max_lane_lane_idxs.push_back(idx);
        }
    }

    if (1 == max_lane_lane_idxs.size()) {
        real_lane_idx = max_lane_lane_idxs[0];
        return true;
    }else if(max_lane_lane_idxs.empty()){
        return false;
    }

    // for last link
    std::vector<int32_t> route_lane_lane_idxs;
    route_lane_lane_idxs.assign(max_lane_lane_idxs.begin(), max_lane_lane_idxs.end());
    auto lanes_vec = lane_element_group[max_lane_lane_idxs[0]].lane_num_vec;
    if (link_id_vec.size() < lanes_vec.size()||lanes_vec.empty()) {  // no data
        return false;
    }

    uint8_t last_lane_id = lanes_vec[lanes_vec.size() - 1];
    uint32_t last_link_id = link_id_vec[lanes_vec.size() - 1];
    uint32_t next_link_id = 0;
    if (GetNextLinkIDForRoute(last_link_id, next_link_id)) {
        message::map_map::s_LinkInfo_t next_link_infos;
        if (GetLinkInfos(next_link_id, next_link_infos)) {
            std::vector<uint8_t> connect_next_link_lanes;
            std::vector<uint8_t> last_link_lanes;
            GetConnectLinkLanes(next_link_id, last_link_id, connect_next_link_lanes, last_link_lanes);
            // std::cout << __FILE__ << "," << __LINE__ << "," << " connect_next_link_lanes.size: " <<
            // connect_next_link_lanes.size() << std::endl;
            if (connect_next_link_lanes.size() > 0) {
                uint8_t min_step = 100;
                for (auto& idx : max_lane_lane_idxs) {
                    // std::cout << __FILE__ << "," << __LINE__ << "," << " idx: " << idx << std::endl;
                    uint8_t temp_min_step = 100;
                    GetMinStepToRoute(lane_element_group[idx].lane_num_vec, connect_next_link_lanes, temp_min_step);
                    min_step = min_step > temp_min_step ? temp_min_step : min_step;
                }
                // std::cout << __FILE__ << "," << __LINE__ << "," << " min_step: " << min_step << std::endl;

                route_lane_lane_idxs.clear();
                for (auto& idx : max_lane_lane_idxs) {
                    uint8_t temp_min_step = 100;
                    GetMinStepToRoute(lane_element_group[idx].lane_num_vec, connect_next_link_lanes, temp_min_step);
                    if (temp_min_step == min_step) {
                        route_lane_lane_idxs.push_back(idx);
                    }
                }
            }
        }
    }
    // std::cout << __FILE__ << "," << __LINE__ << "," << " route_lane_lane_idxs.size: " << route_lane_lane_idxs.size()
    // << std::endl;
    if (1 == route_lane_lane_idxs.size()) {
        real_lane_idx = route_lane_lane_idxs[0];
        // std::cout << __FILE__ << "," << __LINE__ << "," << " real_lane_idx: " << real_lane_idx << std::endl;
        return true;
    }

    // for close
    uint32_t min_close_time = 100;
    for (auto& idx : route_lane_lane_idxs) {
        min_close_time =
            lane_element_group[idx].close_times < min_close_time ? lane_element_group[idx].close_times : min_close_time;
    }

    std::vector<int32_t> min_close_time_idxs;
    for (auto& idx : route_lane_lane_idxs) {
        if (lane_element_group[idx].close_times == min_close_time) {
            min_close_time_idxs.push_back(idx);
        }
    }

    if (1 == min_close_time_idxs.size()) {
        real_lane_idx = min_close_time_idxs[0];
        return true;
    }else if(min_close_time_idxs.empty()){
        return false;
    }

    GetRouteLaneForSplit(min_close_time_idxs, link_id_vec, lane_element_group, real_lane_idx);

    return true;
}

bool CAlgoRefLaneNode::GetRouteLaneForSplit(std::vector<int32_t>& min_close_time_idxs, std::vector<uint32_t> link_id_vec, 
                                            LaneElementGroup& lane_element_group, int32_t& real_lane_idx)
{
    real_lane_idx = 0;
    if (0 == min_close_time_idxs.size()){
        return false;
    }

    if (1 == min_close_time_idxs.size()){
        real_lane_idx = min_close_time_idxs[0];
        return true;
    }

    uint32_t max_number = lane_element_group[min_close_time_idxs[0]].lane_num_vec.size();
    for (size_t lane_idx = 1; lane_idx < max_number; lane_idx++){
        std::vector<uint8_t> lane_unique_ids;
        std::vector<EfmSplitType> lane_unique_splits;
        for (auto idx : min_close_time_idxs){
            uint8_t temp_lane_id = lane_element_group[idx].lane_num_vec[lane_idx];
            if (0 == lane_unique_ids.size()){
                lane_unique_ids.push_back(temp_lane_id);
                lane_unique_splits.push_back(lane_element_group[idx].lane_extra_infos[lane_idx].split_value);
                continue;
            }

            if (std::find(lane_unique_ids.begin(), lane_unique_ids.end(), temp_lane_id) == lane_unique_ids.end()){
                lane_unique_ids.push_back(temp_lane_id);
                lane_unique_splits.push_back(lane_element_group[idx].lane_extra_infos[lane_idx].split_value);
            }
        }

        if (1 == lane_unique_ids.size()){
            continue;
        }

        std::vector<int32_t> new_min_close_time_idxs;
        GetRouteLaneForSplitOne(link_id_vec, lane_element_group, min_close_time_idxs, lane_unique_ids, lane_unique_splits, lane_idx, new_min_close_time_idxs);
        if (0 == new_min_close_time_idxs.size()){
            real_lane_idx = 0;
            return false;
        }

        if (1 == new_min_close_time_idxs.size()){
            real_lane_idx = new_min_close_time_idxs[0];
            return true;
        }

        min_close_time_idxs = new_min_close_time_idxs;
    }

    if (min_close_time_idxs.size() >= 1){
        real_lane_idx = min_close_time_idxs[0];
    }else{
        real_lane_idx = 0;
    }
    
    return true; 
}

bool CAlgoRefLaneNode::GetRouteLaneForSplitOne(std::vector<uint32_t> link_id_vec, LaneElementGroup& lane_element_group, 
                                               std::vector<int32_t>& idxs, std::vector<uint8_t> lane_unique_ids, 
                                               std::vector<EfmSplitType> lane_unique_splits, uint32_t lane_idx, std::vector<int32_t>& new_idxs){
    new_idxs.clear();
    uint8_t route_lane_id = 0;
    LaneElement element_one = LaneElement{};
    if (idxs.size() > 0){
        if (lane_element_group.size() > idxs[0]){
            element_one = lane_element_group[idxs[0]];
        }
    }
    GetRouteLaneForSplitLaneId(link_id_vec, lane_unique_ids, lane_unique_splits, lane_idx, route_lane_id, lane_element_group[idxs[0]]);
    for (auto idx : idxs){
        if (lane_element_group[idx].lane_num_vec[lane_idx] == route_lane_id){
            new_idxs.push_back(idx);
        }
    }

    return true; 
}

bool CAlgoRefLaneNode::GetRouteLaneForSplitLaneId(std::vector<uint32_t> link_id_vec, std::vector<uint8_t> lane_unique_ids,
                                                  std::vector<EfmSplitType> lane_unique_splits, uint32_t lane_idx,
                                                  uint8_t& route_lane_id,
                                                  LaneElement element_one){
    route_lane_id = 0;
    if (lane_unique_ids.size() == 0){
        return false;
    }

    if (lane_unique_ids.size() == 1 || lane_unique_ids.size() > 2 || lane_idx == 0){
        route_lane_id = lane_unique_ids[0];
        return true;
    }



    if (lane_unique_splits[0] == EFM_SplitType_NONE || lane_unique_splits[0] == EFM_SplitType_TO_LEFT ||
        lane_unique_splits[0] == EFM_SplitType_TO_RIGHT){
        route_lane_id = lane_unique_ids[0];
        return true;
    }else if (lane_unique_splits[0] == EFM_SplitType_FROM_LEFT || lane_unique_splits[0] == EFM_SplitType_FROM_RIGHT){
        route_lane_id = lane_unique_ids[1];
        return true;
    }else{
        // if (lane_unique_splits[0] == EFM_SplitType_CONTIUE_FROM_LEFT || lane_unique_splits[0] == EFM_SplitType_CONTIUE_FROM_RIGHT){
        //     route_lane_id = lane_unique_ids[0];
        // }else{
        //     route_lane_id = lane_unique_ids[1];
        // }
        // return true;
        message::map_map::s_LinkInfo_t link_infos;
        if (false == GetLinkInfos(link_id_vec[lane_idx], link_infos)) {
            if (lane_unique_splits[0] == EFM_SplitType_CONTIUE_FROM_LEFT || lane_unique_splits[0] == EFM_SplitType_CONTIUE_FROM_RIGHT){
                route_lane_id = lane_unique_ids[0];
            }else{
                route_lane_id = lane_unique_ids[1];
            }
            return true;
        }
        uint32_t temp_length = 0;
        uint32_t begin_link_id = link_id_vec[lane_idx];
        int32_t temp_lane_idx = lane_idx - 1;
        while (temp_length < NEXT_SPLIT_DIST && temp_lane_idx >= 0){
            auto& back_lane_split_value = element_one.lane_extra_infos[temp_lane_idx].split_value;
            if (EFM_SplitType_NONE == back_lane_split_value){
                temp_length += element_one.lane_extra_infos[temp_lane_idx].e_offset - element_one.lane_extra_infos[temp_lane_idx].s_offset;
                temp_lane_idx --;
            }else{
                if (EFM_SplitType_TO_LEFT == back_lane_split_value || EFM_SplitType_FROM_LEFT == back_lane_split_value || 
                    EFM_SplitType_CONTIUE_FROM_LEFT == back_lane_split_value || EFM_SplitType_SPLIT_FROM_LEFT == back_lane_split_value){
                    if (lane_unique_ids[0] > lane_unique_ids[1]){
                        route_lane_id = lane_unique_ids[0];
                    }else{
                        route_lane_id = lane_unique_ids[1];
                    }
                }else{
                    if (lane_unique_ids[0] < lane_unique_ids[1]){
                        route_lane_id = lane_unique_ids[0];
                    }else{
                        route_lane_id = lane_unique_ids[1];
                    }
                }
                return true;
            }
                       

            // if (IsSplitLink(begin_link_id)){
            //     uint8_t need_lane_id = 0;
            //     for (auto& laneinfo : link_infos.LaneInfos.LaneInfos) {
            //         if (3 != laneinfo.LaneType.data && 9 != laneinfo.LaneType.data && 17 != laneinfo.LaneType.data){
            //             if (laneinfo.LaneNum.LaneNum > std::max(lane_unique_ids[0], lane_unique_ids[1]) || 
            //                 laneinfo.LaneNum.LaneNum < std::min(lane_unique_ids[0], lane_unique_ids[1])){
            //                 need_lane_id = laneinfo.LaneNum.LaneNum;
            //                 break;
            //             }
            //         }
            //     }
            //     if (0 == need_lane_id){
            //         if (lane_unique_splits[0] == EFM_SplitType_CONTIUE_FROM_LEFT || lane_unique_splits[0] == EFM_SplitType_CONTIUE_FROM_RIGHT){
            //             route_lane_id = lane_unique_ids[0];
            //         }else{
            //             route_lane_id = lane_unique_ids[1];
            //         }
            //     }else if (need_lane_id > lane_unique_ids[0]){
            //         if (lane_unique_ids[0] > lane_unique_ids[1]){
            //             route_lane_id = lane_unique_ids[0];
            //         }else{
            //             route_lane_id = lane_unique_ids[1];
            //         }
            //     }else{
            //         if (lane_unique_ids[0] < lane_unique_ids[1]){
            //             route_lane_id = lane_unique_ids[0];
            //         }else{
            //             route_lane_id = lane_unique_ids[1];
            //         }
            //     }
            //     return true;
            // }

            // temp_length += link_infos.EndOffset.EndOffset - link_infos.PathOffset.PathOffset;
            // uint32_t next_link_id = 0;
            // GetNextLinkIDForRoute(begin_link_id, next_link_id);
            // begin_link_id = next_link_id;
            // if (false == GetLinkInfos(next_link_id, link_infos)){
            //     if (lane_unique_splits[0] == EFM_SplitType_CONTIUE_FROM_LEFT || lane_unique_splits[0] == EFM_SplitType_CONTIUE_FROM_RIGHT){
            //         route_lane_id = lane_unique_ids[0];
            //     }else{
            //         route_lane_id = lane_unique_ids[1];
            //     }
            //     return true;
            // }  
        }
        
        if (lane_unique_splits[0] == EFM_SplitType_CONTIUE_FROM_LEFT || lane_unique_splits[0] == EFM_SplitType_CONTIUE_FROM_RIGHT){
            route_lane_id = lane_unique_ids[0];
        }else{
            route_lane_id = lane_unique_ids[1];
        }

    }

    return true;
}

bool CAlgoRefLaneNode::GetEgpGroupIdx(std::vector<uint32_t> link_id_vec, LaneElementGroupSets& lane_element_group_sets,
                                      int32_t& egp_group_idx) {
    egp_group_idx = -1;
    if (map_position_->LinkId != link_id_vec[0]) {
        // link error
        return false;
    }

    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        auto& lane_element = lane_element_group_sets[lane_idx][0];
        if (lane_element.lane_num_vec[0] == map_position_->LaneId) {
            egp_group_idx = lane_idx;
            break;
        }
    }

    if (-1 == egp_group_idx) {
        return false;
    }

    return true;
}

bool CAlgoRefLaneNode::GetLaneCandidateIndex(LaneElementGroup& lane_element_group, int32_t& lane_candidate_index) {
    lane_candidate_index = -1;
    for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group.size(); lane_dir_idx++) {
        // std::cout << __FILE__ << "," << __LINE__ << "," << " lane_dir_idx: " << lane_dir_idx << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "," << " lane_element_group[lane_dir_idx].is_group_dest: " <<
        // int(lane_element_group[lane_dir_idx].is_group_dest) << std::endl;
        if (lane_element_group[lane_dir_idx].is_group_dest) {
            lane_candidate_index = lane_element_group[lane_dir_idx].candidate_index;
            break;
        }
    }

    if (-1 == lane_candidate_index) {
        return false;
    }

    return true;
}

bool CAlgoRefLaneNode::GetLaneCandidateIndexEgo(LaneElementGroup& lane_element_group, 
                                                int32_t& ego_candidate_index, 
                                                int32_t& left_candidate_index, 
                                                int32_t& right_candidate_index,
                                                std::vector<uint32_t> link_id_vec)
{
    ego_candidate_index = -1;
    left_candidate_index = -1;
    right_candidate_index = -1;
    int32_t max_len_size = link_id_vec.size();

    if (0 == lane_element_group.size()){
        return false;
    }else if (1 == lane_element_group.size()){
        ego_candidate_index = lane_element_group[0].candidate_index;
    }else{
        int32_t first_split_idx = -1;
        int32_t min_split_value = max_len_size;
        for (size_t i = 0; i < lane_element_group.size(); i++){
            for (size_t lane_extra_idx = 1; lane_extra_idx < lane_element_group[i].lane_extra_infos.size(); lane_extra_idx++){
                auto& temp_extra = lane_element_group[i].lane_extra_infos[lane_extra_idx].split_value;
                if (EFM_SplitType_NONE == temp_extra || EFM_SplitType_TO_LEFT == temp_extra || EFM_SplitType_TO_RIGHT == temp_extra
                    || EFM_SplitType_CONTIUE_FROM_LEFT == temp_extra || EFM_SplitType_CONTIUE_FROM_RIGHT == temp_extra){
                    continue;
                }else{
                    if (min_split_value > lane_extra_idx){
                        first_split_idx = i;
                        min_split_value = lane_extra_idx;
                    }
                    
                    break;
                } 
            }
        }

        if (-1 == first_split_idx || min_split_value == max_len_size){
            ego_candidate_index = lane_element_group[0].candidate_index;
            return true;
        }
        uint32_t position_to_link_start = 0;
        bool re_value =  GetPositonToLinkStart(link_id_vec[min_split_value], position_to_link_start);
        if (position_to_link_start > p_position_to_split_dist || false == re_value){
            //position + POSITION_TO_SPLIT_DIST < split_point
            ego_candidate_index = GetContinueLaneCandidateIndex(lane_element_group, -1);
            return true;
        }
        //position + POSITION_TO_SPLIT_DIST >= split_point
        uint8_t split_lane_id = lane_element_group[first_split_idx].lane_num_vec[min_split_value];
        uint8_t continue_lane_id = 0;
        LaneElementGroup ego_element_group;
        LaneElementGroup split_element_group;
        for (auto& lane_element : lane_element_group){
            if (lane_element.lane_num_vec.size() <= min_split_value){
                continue;
            }

            if (lane_element.lane_num_vec[min_split_value] == split_lane_id){
                LaneElement temp_ele = lane_element;
                temp_ele.is_group_dest =false;
                split_element_group.push_back(temp_ele);
            }else{
                continue_lane_id = lane_element.lane_num_vec[min_split_value];
                ego_element_group.push_back(lane_element);
            }
        }
        //ego_candidate_index = GetContinueLaneCandidateIndex(ego_element_group, -1);
        //ego
        if (ego_element_group.size() < 1 || split_element_group.size() < 1){
            return false;
        }
        
        uint32_t ego_max_lane_number = 0;
        for (size_t ego_temp_id = 0; ego_temp_id < ego_element_group.size(); ego_temp_id ++){
            if (ego_max_lane_number < ego_element_group[ego_temp_id].lane_num_vec.size()){
                ego_max_lane_number = ego_element_group[ego_temp_id].lane_num_vec.size();
            }
        }
        DeleteSplitLaneForOne(link_id_vec, ego_element_group, ego_max_lane_number); 
        for (size_t ego_temp_id = 0; ego_temp_id < ego_element_group.size(); ego_temp_id ++){
            if (ego_element_group[ego_temp_id].is_group_dest){
                ego_candidate_index = ego_element_group[ego_temp_id].candidate_index;
                break;
            }
        }
               

        //split
        uint32_t split_max_lane_number = 0;
        for (size_t temp_id = 0; temp_id < split_element_group.size(); temp_id ++){
            if (split_max_lane_number < split_element_group[temp_id].lane_num_vec.size()){
                split_max_lane_number = split_element_group[temp_id].lane_num_vec.size();
            }
        }

        DeleteSplitLaneForOne(link_id_vec, split_element_group, split_max_lane_number);
        
        //jude
        if (continue_lane_id > split_lane_id){
            //right_candidate_index = GetContinueLaneCandidateIndex(split_element_group, min_split_value);
            for (size_t temp_id = 0; temp_id < split_element_group.size(); temp_id ++){
                if (split_element_group[temp_id].is_group_dest){
                    right_candidate_index = split_element_group[temp_id].candidate_index;
                    break;
                }
            }
        }else{
            //left_candidate_index = GetContinueLaneCandidateIndex(split_element_group, min_split_value);
            for (size_t temp_id = 0; temp_id < split_element_group.size(); temp_id ++){
                if (split_element_group[temp_id].is_group_dest){
                    left_candidate_index = split_element_group[temp_id].candidate_index;
                    break;
                }
            }
        }
    }
    
    return true;
}

bool CAlgoRefLaneNode::GetPositonToLinkStart(uint32_t link_id, uint32_t& dist){

    message::map_map::s_LinkInfo_t link_infos;
    if (false == GetLinkInfos(link_id, link_infos)) {
        // static data none
        return false;
    }

    if (link_infos.PathOffset.PathOffset >= map_position_->PathOffset) {
        dist = link_infos.PathOffset.PathOffset - map_position_->PathOffset;
    } else {
        dist = 0;
        return false;
    }

    return true;
}

int32_t CAlgoRefLaneNode::GetContinueLaneCandidateIndex(LaneElementGroup& lane_element_group, int32_t ignore_split_idx){

    int32_t continue_idx = 0;
    int32_t times = 0;
    while (times < 1000){
        auto& element =  lane_element_group[continue_idx];
        int32_t continue_split = -1;
        for (size_t lane_extra_idx = 0; lane_extra_idx < element.lane_extra_infos.size(); lane_extra_idx++){
            if (ignore_split_idx == lane_extra_idx){
                continue;
            }
            
            auto& temp_extra = element.lane_extra_infos[lane_extra_idx].split_value;
            if (EFM_SplitType_NONE == temp_extra || EFM_SplitType_TO_LEFT == temp_extra || EFM_SplitType_TO_RIGHT == temp_extra
                || EFM_SplitType_CONTIUE_FROM_LEFT == temp_extra || EFM_SplitType_CONTIUE_FROM_RIGHT == temp_extra){
                continue;
            }else{
                continue_split = lane_extra_idx;
                break;
            } 
        }


        if (-1 == continue_split){
            break;
        }
        //std::cout << __FILE__ << "," << __LINE__ << "," << " continue_idx: " << continue_idx << std::endl;
        //std::cout << __FILE__ << "," << __LINE__ << "," << " continue_split: " << continue_split << std::endl;

        int32_t new_continue_idx = continue_idx;
        for (size_t i = 0; i < lane_element_group.size(); i++){
            if (i == continue_idx){
                continue;
            }

            int32_t temp_continue_split = -1;
            for (size_t lane_extra_idx = 0; lane_extra_idx < lane_element_group[i].lane_extra_infos.size(); lane_extra_idx++){
                if (ignore_split_idx == lane_extra_idx){
                    continue;
                }
                
                auto& temp_extra = lane_element_group[i].lane_extra_infos[lane_extra_idx].split_value;
                if (EFM_SplitType_NONE == temp_extra || EFM_SplitType_TO_LEFT == temp_extra || EFM_SplitType_TO_RIGHT == temp_extra
                    || EFM_SplitType_CONTIUE_FROM_LEFT == temp_extra || EFM_SplitType_CONTIUE_FROM_RIGHT == temp_extra){
                    continue;
                }else{
                    temp_continue_split = lane_extra_idx;
                    break;
                } 
            }

            if ( -1 == temp_continue_split){
                continue_idx = i;
                new_continue_idx = continue_idx;
                break;
            }

            if (temp_continue_split > continue_split){
                new_continue_idx = i;
                break;
            }
        }

        if (new_continue_idx == continue_idx){
            break;
        }else{
            continue_idx = new_continue_idx;
        }
        
        times ++;
    }
    
    

    return lane_element_group[continue_idx].candidate_index;
}

bool CAlgoRefLaneNode::GetElementForCandidateIndex(std::vector<LaneElement>& lane_element_group, 
                                                   LaneElement& element, int32_t index){
    element = LaneElement{};
    for (auto & temp_ele : lane_element_group){
        if (index == temp_ele.candidate_index){
            element = temp_ele;
            break;
        }
    }
                                                    
    return true;                                       
}

bool CAlgoRefLaneNode::GetRefForEgoSplit(LaneElement element, LaneElement side_element, int32_t& ref_lane_idx){
    if (element.is_dest || element.is_group_dest){
        ref_lane_idx = element.candidate_index;
    }else if (side_element.is_dest || side_element.is_group_dest){
        ref_lane_idx = side_element.candidate_index;
    }else{
        if (element.rest_length > side_element.rest_length){
            ref_lane_idx = element.candidate_index;
        }else if (element.rest_length < side_element.rest_length){
            ref_lane_idx = side_element.candidate_index;
        }else{
            if (element.close_times <= side_element.close_times){
                ref_lane_idx = element.candidate_index;
            }else{
                ref_lane_idx = side_element.candidate_index;
            }
        }
    }                                          
    return true;                                       
}

bool CAlgoRefLaneNode::GetRefLaneCandidateIndex(LaneElementGroupSets& lane_element_group_sets,
                                                int32_t& ego_lane_index, int32_t& left_lane_idx,
                                                int32_t& left_left_lane_idx, int32_t& right_lane_idx,
                                                int32_t& right_right_lane_idx, int32_t& ref_lane_idx,
                                                std::vector<std::vector<int32_t>>& ref_lanes,
                                                std::vector<uint32_t> link_id_vec) {
    ref_lane_idx = ego_lane_index;
    std::set<int32_t> ref_lanes_group_idxs;
    for (auto ref_lane : ref_lanes) {
        if (ref_lanes_group_idxs.find(ref_lane[0]) == ref_lanes_group_idxs.end()) {
            ref_lanes_group_idxs.insert(ref_lane[0]);
        }
    }

    uint32_t change_times = 0;
    DirectionType dir_type = DIRECTIONTYPE_UNKNOW;
    if (ref_lanes_group_idxs.find(ego_group_index_) != ref_lanes_group_idxs.end()) {
        //自车道所在是目标车道
        LaneElement temp_ego_ele;
        GetElementForCandidateIndex(lane_element_group_sets[ego_group_index_], temp_ego_ele, ego_lane_index);
        if (false == ego_left_same_lane_ && false == ego_right_same_lane_){
            ref_lane_idx = ego_lane_index;
        }else if (ego_left_same_lane_){
            LaneElement temp_left_ele;
            GetElementForCandidateIndex(lane_element_group_sets[ego_group_index_], temp_left_ele, left_lane_idx);
            GetRefForEgoSplit(temp_ego_ele, temp_left_ele, ref_lane_idx);       
        }else{
            LaneElement temp_right_ele;
            GetElementForCandidateIndex(lane_element_group_sets[ego_group_index_], temp_right_ele, right_lane_idx);
            GetRefForEgoSplit(temp_ego_ele, temp_right_ele, ref_lane_idx);
        }
    } else if (ref_lanes_group_idxs.find(left_group_idx_) != ref_lanes_group_idxs.end()) {
        //左车道所在是目标车道
        ref_lane_idx = left_lane_idx;
        change_times = 1;
        dir_type = DIRECTIONTYPE_LEFT;
    } else if (ref_lanes_group_idxs.find(right_group_idx_) != ref_lanes_group_idxs.end()) {
        //右车道所在是目标车道
        ref_lane_idx = right_lane_idx;
        change_times = 1;
        dir_type = DIRECTIONTYPE_RIGHT;
    } else if (ref_lanes_group_idxs.find(left_left_group_idx_) != ref_lanes_group_idxs.end()) {
        //左左车道所在是目标车道
        if (left_lane_idx >= 0 && false == SideLaneDistIsShort(link_id_vec, lane_element_group_sets, left_group_idx_, ego_group_index_, false)){
            //有左车道，且大于200米
            ref_lane_idx = left_left_lane_idx;
            change_times = left_left_group_idx_ > ego_group_index_ ? left_left_group_idx_ - ego_group_index_ : 0;
            dir_type = DIRECTIONTYPE_LEFT;
        }else{
            //自车道所在是目标车道
            LaneElement temp_ego_ele;
            GetElementForCandidateIndex(lane_element_group_sets[ego_group_index_], temp_ego_ele, ego_lane_index);
            if (false == ego_left_same_lane_ && false == ego_right_same_lane_){
                ref_lane_idx = ego_lane_index;
            }else if (ego_left_same_lane_){
                LaneElement temp_left_ele;
                GetElementForCandidateIndex(lane_element_group_sets[ego_group_index_], temp_left_ele, left_lane_idx);
                GetRefForEgoSplit(temp_ego_ele, temp_left_ele, ref_lane_idx);       
            }else{
                LaneElement temp_right_ele;
                GetElementForCandidateIndex(lane_element_group_sets[ego_group_index_], temp_right_ele, right_lane_idx);
                GetRefForEgoSplit(temp_ego_ele, temp_right_ele, ref_lane_idx);
            }
        }
    } else if (ref_lanes_group_idxs.find(right_right_group_idx_) != ref_lanes_group_idxs.end()) {
        //右右车道所在是目标车道
        if (right_lane_idx >= 0 && false == SideLaneDistIsShort(link_id_vec, lane_element_group_sets, right_group_idx_, ego_group_index_, true)){
            //有右车道，且大于200米
            ref_lane_idx = right_right_lane_idx;
            change_times = right_right_group_idx_ < ego_group_index_ ? ego_group_index_ - right_right_group_idx_ : 0;
            dir_type = DIRECTIONTYPE_RIGHT;
        }else{
            //自车道所在是目标车道
            LaneElement temp_ego_ele;
            GetElementForCandidateIndex(lane_element_group_sets[ego_group_index_], temp_ego_ele, ego_lane_index);
            if (false == ego_left_same_lane_ && false == ego_right_same_lane_){
                ref_lane_idx = ego_lane_index;
            }else if (ego_left_same_lane_){
                LaneElement temp_left_ele;
                GetElementForCandidateIndex(lane_element_group_sets[ego_group_index_], temp_left_ele, left_lane_idx);
                GetRefForEgoSplit(temp_ego_ele, temp_left_ele, ref_lane_idx);       
            }else{
                LaneElement temp_right_ele;
                GetElementForCandidateIndex(lane_element_group_sets[ego_group_index_], temp_right_ele, right_lane_idx);
                GetRefForEgoSplit(temp_ego_ele, temp_right_ele, ref_lane_idx);
            }
        }
    } else {
        //目标车道不在5车道范围
        int32_t min_step = 100;
        int32_t min_idx = -1;
        for (auto idx : ref_lanes_group_idxs) {
            if (min_step > abs(idx - ego_group_index_)) {
                min_step = abs(idx - ego_group_index_);
                min_idx = idx;
            }
        }

        if (-1 == min_idx) {
            //未找到目标车道
            ref_lane_idx = ego_lane_index;
        } else if (min_idx > ego_group_index_) {
            //目标车道在左边
            bool left_left_short_flag = true;
            if (-1 != left_left_lane_idx && 
                false == SideLaneDistIsShort(link_id_vec, lane_element_group_sets, left_left_group_idx_, ego_group_index_, false)){
                //左左车道有效且可行使大于200米
                left_left_short_flag = false;
            }

            bool left_short_flag = true;
            if (-1 != left_lane_idx && 
                false == SideLaneDistIsShort(link_id_vec, lane_element_group_sets, left_group_idx_, ego_group_index_, false)){
                //左车道有效且可行使大于200米
                left_short_flag = false;
            }

            if (ego_left_same_lane_){
                //自车道和左车道属于同一个车道分歧
                if (left_left_short_flag){
                    ref_lane_idx = left_lane_idx;
                    change_times = 0;
                    dir_type = DIRECTIONTYPE_LEFT;
                }else{
                    ref_lane_idx = left_left_lane_idx;
                    change_times = min_idx - ego_group_index_;
                    dir_type = DIRECTIONTYPE_LEFT;
                }
            }else{
                if (left_short_flag){
                    ref_lane_idx = ego_lane_index;
                }else if (left_left_short_flag){
                    ref_lane_idx = left_lane_idx;
                    change_times = left_group_idx_ - ego_group_index_;
                    dir_type = DIRECTIONTYPE_LEFT;
                }else{
                    ref_lane_idx = left_left_lane_idx;
                    change_times = min_idx - ego_group_index_;
                    dir_type = DIRECTIONTYPE_LEFT;
                }
            }
        } else {
            //目标车道在右边
            bool right_right_short_flag = true;
            if (-1 != right_right_lane_idx && 
                false == SideLaneDistIsShort(link_id_vec, lane_element_group_sets, right_right_group_idx_, ego_group_index_, true)){
                //右右车道有效且可行使大于200米
                right_right_short_flag = false;
            }

            bool right_short_flag = true;
            if (-1 != right_lane_idx && 
                false == SideLaneDistIsShort(link_id_vec, lane_element_group_sets, right_group_idx_, ego_group_index_, true)){
                //右车道有效且可行使大于200米
                right_short_flag = false;
            }

            if (ego_right_same_lane_){
                //自车道和右车道属于同一个车道分歧
                if (right_right_short_flag){
                    ref_lane_idx = right_lane_idx;
                    change_times = 0;
                    dir_type = DIRECTIONTYPE_RIGHT;
                }else{
                    ref_lane_idx = right_right_lane_idx;
                    change_times = ego_group_index_ - min_idx;
                    dir_type = DIRECTIONTYPE_RIGHT;
                }
            }else{
                if (right_short_flag){
                    ref_lane_idx = ego_lane_index;
                }else if (right_right_short_flag){
                    ref_lane_idx = right_lane_idx;
                    change_times = ego_group_index_ - right_group_idx_;
                    dir_type = DIRECTIONTYPE_RIGHT;
                }else{
                    ref_lane_idx = right_right_lane_idx;
                    change_times = ego_group_index_ - min_idx;
                    dir_type = DIRECTIONTYPE_RIGHT;
                }
            }
        }
    }

    // set change
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            if (lane_element.candidate_index == ego_lane_index) {
                lane_element.change_times = change_times;
                lane_element.dir_type = dir_type;
                if ((ref_lanes_group_idxs.find(ego_group_index_) != ref_lanes_group_idxs.end()) && DIRECTIONTYPE_UNKNOW == dir_type){
                    MakeNodeForLastLinkNotINRoute(lane_element_group_sets, lane_element, link_id_vec);
                }
            }
        }
    }

    ego_lane_index_ = ego_lane_index;
    left_lane_idx_  = left_lane_idx;
    left_left_lane_idx_  = left_left_lane_idx;
    right_lane_idx_ = right_lane_idx;
    right_right_lane_idx_ = right_right_lane_idx;
    ref_lane_idx_   = ref_lane_idx;

    return true;
}

bool CAlgoRefLaneNode::SideLaneDistIsShort(std::vector<uint32_t> link_id_vec, 
                                          LaneElementGroupSets lane_element_group_sets, 
                                          int32_t group_idx,
                                          int32_t ego_group_idx,
                                          bool merge_to_left){
    if (group_idx < 0){
        return true;
    }
    
    uint32_t group_idx_close_length = 0;
    uint32_t group_idx_length = 0;
    GetElementRealLength(link_id_vec, lane_element_group_sets[group_idx], group_idx_close_length, group_idx_length, 
                         merge_to_left ? 1 : 2); 

    uint32_t ego_group_idx_close_length = 0;
    uint32_t ego_group_idx_length = 0;
    GetElementRealLength(link_id_vec, lane_element_group_sets[ego_group_idx], ego_group_idx_close_length, ego_group_idx_length, 0); 

    if (0 == group_idx_close_length || 0== group_idx_length || 0 == ego_group_idx_close_length || 0 == ego_group_idx_length){
        return false;
    }
    
    if (group_idx_close_length < SHORT_ELEMENT_DIST){
        if (ego_group_idx_close_length > group_idx_close_length){
            return true;
        // }else if (ego_group_idx_close_length == group_idx_close_length){
        //     if (group_idx_length > group_idx_close_length && ego_group_idx_length > ego_group_idx_close_length){
        //         return true;
        //     }
        }
    }
    
    return false;
}

bool CAlgoRefLaneNode::GetElementRealLength(std::vector<uint32_t> link_id_vec, 
                                            std::vector<LaneElement> lane_elements, 
                                            uint32_t& close_length,
                                            uint32_t& length,
                                            int merge_to){//merge_to, 1:left;2:right;0:all
    close_length = 0;
    length = 0;                            
    for (size_t lane_dir_idx = 0; lane_dir_idx < lane_elements.size(); lane_dir_idx++) {
        auto& lane_element = lane_elements[lane_dir_idx];
        if (link_id_vec.size() <  lane_element.lane_num_vec.size()){
            return false;
        }
        if (false == lane_element.is_group_dest){
            continue;
        }

        length = static_cast<uint32_t>(lane_element.rest_length * 100.0);
        close_length = length;
        if (lane_element.close_position + 1 < lane_element.lane_num_vec.size()){
            auto& extra_data = lane_element.lane_extra_infos[lane_element.close_position];
            if (0 == merge_to){
                close_length = extra_data.e_offset >= 0 ? extra_data.e_offset : length;
            }else if (lane_element.close_to == merge_to){
                close_length = extra_data.e_offset >= 0 ? extra_data.e_offset : length;
            }else{
                close_length = length;
            }
            
            

            // message::map_map::s_LinkInfo_t link_infos;
            // if (false == GetLinkInfos(link_id_vec[lane_element.close_position], link_infos)) {
            //     // static data none
            //     return false;
            // }

            // if (link_infos.EndOffset.EndOffset > map_position_->PathOffset){
            //     close_length = link_infos.EndOffset.EndOffset - map_position_->PathOffset;
            //     close_length = close_length > length ? length : close_length;
            // }else{
            //     close_length = length;
            // }
        }
        
    }
    
    return true;
}

bool CAlgoRefLaneNode::GetCloseDir(std::vector<uint8_t> lane_id_vec, uint32_t close_idx, uint8_t& close_to,
                                   LaneElementGroupSets& lane_element_group_sets) {
    if ((close_idx + 1) >= lane_id_vec.size()) {
        return true;
    }

    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            if (lane_element.lane_num_vec.size() > (close_idx + 1)) {
                if ((lane_id_vec[close_idx] != lane_element.lane_num_vec[close_idx]) &&
                    (lane_id_vec[close_idx + 1] == lane_element.lane_num_vec[close_idx + 1])) {
                    if (lane_id_vec[close_idx] > lane_element.lane_num_vec[close_idx]) {
                        close_to = 2;
                    } else {
                        close_to = 1;
                    }
                    return true;
                }
            }
        }
    }

    return true;
}

bool CAlgoRefLaneNode::GetSplitDir(std::vector<uint8_t> lane_id_vec, uint32_t split_idx, uint8_t& split_from,
                                   LaneElementGroupSets& lane_element_group_sets) {
    if (0 == split_idx) {
        return true;
    }

    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            if (lane_element.lane_num_vec.size() > split_idx) {
                if ((lane_id_vec[split_idx] != lane_element.lane_num_vec[split_idx]) &&
                    (lane_id_vec[split_idx - 1] == lane_element.lane_num_vec[split_idx - 1])) {
                    if (lane_id_vec[split_idx] > lane_element.lane_num_vec[split_idx]) {
                        split_from = 1;
                    } else {
                        split_from = 2;
                    }
                    return true;
                }
            }
        }
    }
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << " split_idx: " << split_idx << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << " split_from: " << split_from << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << " lane_id_vec[0]: " << lane_id_vec[0] << std::endl;
    return true;
}

bool CAlgoRefLaneNode::GetNodeElementChangelane(std::vector<uint32_t> link_id_vec, LaneElement& lane_element,
                                      message::efm::s_NodeInfo_t& NodeInfo) {

        NodeInfo.StartPointOffset = lane_element.lane_change_position;
        if (lane_element.first_lane_is_no_route){
            NodeInfo.EndPointOffset = lane_element.org_rest_length > 200.0 ? lane_element.org_rest_length - 200.0 : 0.0;
        }else{
            NodeInfo.EndPointOffset = lane_element.rest_length;
        }
        NodeInfo.DirectionType.data_ = lane_element.dir_type;
        NodeInfo.LaneChgType = 1 | (1 << 1);
        NodeInfo.LaneChgTimes = lane_element.change_times;

        if (lane_element.rest_length < lane_element.lane_change_position + RAMP_TO_MAIN_LENGTH){

            message::map_map::s_LinkInfo_t last_link_infos;
            if (false == GetLinkInfos(link_id_vec[lane_element.lane_num_vec.size() - 1], last_link_infos)) {
                // static data none
                return false;
            }
            uint8_t last_lane_type = 0;
            GetLaneType(last_link_infos, lane_element.lane_num_vec[lane_element.lane_num_vec.size() - 1], last_lane_type);  


            message::map_map::s_LinkInfo_t first_link_infos;
            if (false == GetLinkInfos(link_id_vec[0], first_link_infos)) {
                // static data none
                return false;
            }
            uint8_t first_lane_type = 0;
            GetLaneType(first_link_infos, lane_element.lane_num_vec[0], first_lane_type);

            if ((0 == last_lane_type || 1 == last_lane_type || 7 == last_lane_type || 2 == last_lane_type || 8 == last_lane_type) && 
                (4 == first_lane_type || 5 == first_lane_type || 6 == first_lane_type)){
                NodeInfo.LaneChgType = NodeInfo.LaneChgType | (1 << 3);
            }
        } 

    return true;
}

bool CAlgoRefLaneNode::RefEgoIsOneGroup(std::vector<LaneElement>& lane_element_group){
    if (!(ego_lane_index_ >= 0 && ref_lane_idx_ >=0)){
        return false;
    }

    uint8_t ego_first_lane_id = 0;
    uint8_t ref_first_lane_id = 0;

    for (auto& element : lane_element_group){
        if (element.candidate_index == ego_lane_index_){
            ego_first_lane_id = element.lane_num_vec[0];
        }

        if (element.candidate_index == ref_lane_idx_){
            ref_first_lane_id = element.lane_num_vec[0];
        }        
    }

    if (ego_first_lane_id > 0 && ref_first_lane_id > 0 && ego_first_lane_id == ref_first_lane_id){
        return true;
    }
    
    return false;
}

bool CAlgoRefLaneNode::GetNodeElement(std::vector<uint32_t> link_id_vec, LaneElement& lane_element,
                                      message::efm::s_NodeInfo_t& NodeInfo, 
                                      std::vector<LaneElement>& lane_element_group) {
    if (0 == lane_element.lane_num_vec.size() || 0 == link_id_vec.size() || 
        link_id_vec.size() < lane_element.lane_num_vec.size()){
        return false;
    }
    
    // change lane
    message::efm::s_NodeInfo_t change_lane_NodeInfo;
    LaneElement split_lane_element = lane_element;
    bool split_dest_flag = false;
    if (DIRECTIONTYPE_UNKNOW != lane_element.dir_type) {
        GetNodeElementChangelane(link_id_vec, lane_element, change_lane_NodeInfo);

        //std::cout << __FILE__ << "," << __LINE__ << "," << " ego_lane_index_: " << ego_lane_index_ << std::endl;
        //std::cout << __FILE__ << "," << __LINE__ << "," << " ref_lane_idx_: " << ref_lane_idx_ << std::endl;
        //std::cout << __FILE__ << "," << __LINE__ << "," << " lane_element.is_group_dest: " << int(lane_element.is_group_dest) << std::endl;
        //std::cout << __FILE__ << "," << __LINE__ << "," << " candidate_index: " << lane_element.candidate_index << std::endl;
        if (false == lane_element.is_group_dest && RefEgoIsOneGroup(lane_element_group)){
            for (auto& temp_split_element: lane_element_group){
                if (true == temp_split_element.is_group_dest){
                    //std::cout << __FILE__ << "," << __LINE__ << "," << "candidate_index: " << temp_split_element.candidate_index << std::endl;
                    split_lane_element = temp_split_element;
                    split_dest_flag = true;
                    break;
                }
            }
        }
    }

    // split  or merge
    message::efm::s_NodeInfo_t split_merge_NodeInfo = {};
    if (lane_element.close_position < split_lane_element.split_position && lane_element.close_position < lane_element.lane_num_vec.size()) {
        //std::cout << __FILE__ << "," << __LINE__ << "," << " lane_element.close_to: " << int(lane_element.close_to) << std::endl;
        int32_t temp_close_idx = lane_element.close_position;
        //IsNextMergeShort(lane_element, lane_element.close_position, temp_close_idx);
        GetLaneChgTypeClose(link_id_vec, lane_element, temp_close_idx, split_merge_NodeInfo.LaneChgType);
        GetStartOffsetClose(link_id_vec, lane_element, temp_close_idx, split_merge_NodeInfo.LaneChgType,
                            split_merge_NodeInfo.StartPointOffset);
        GetEndOffsetClose(link_id_vec, lane_element, temp_close_idx, split_merge_NodeInfo.LaneChgType,
                          split_merge_NodeInfo.EndPointOffset);
        if (EFM_MergeType_TO_LEFT == lane_element.lane_extra_infos[temp_close_idx].merge_value || 
            EFM_MergeType_LEFT_TO_MIDDLE == lane_element.lane_extra_infos[temp_close_idx].merge_value){
            split_merge_NodeInfo.DirectionType.data_ = 1;
        }else{
            split_merge_NodeInfo.DirectionType.data_ = 2;
        }
        split_merge_NodeInfo.LaneChgTimes = split_merge_NodeInfo.LaneChgType > 1 ? 1 : 0;
    } else if (split_lane_element.split_position < lane_element.close_position && split_lane_element.split_position < split_lane_element.lane_num_vec.size()) {
        //std::cout << __FILE__ << "," << __LINE__ << "," << " split_lane_element.split_from: " << int(split_lane_element.split_from) << std::endl;
        GetLaneChgTypeSplit(link_id_vec, split_lane_element, split_lane_element.split_position, split_merge_NodeInfo.LaneChgType);
        GetStartOffsetSplit(link_id_vec, split_lane_element, split_lane_element.split_position, split_merge_NodeInfo.LaneChgType,
                            split_merge_NodeInfo.StartPointOffset);
        GetEndOffsetSplit(link_id_vec, split_lane_element, split_lane_element.split_position, split_merge_NodeInfo.LaneChgType,
                          split_merge_NodeInfo.EndPointOffset);
        split_merge_NodeInfo.DirectionType.data_ = split_lane_element.split_from;
        split_merge_NodeInfo.LaneChgTimes = split_merge_NodeInfo.LaneChgType > 1 ? 1 : 0;
    }

    if (split_dest_flag && true == split_lane_element.is_dest && split_lane_element.split_position < lane_element.close_position){
        split_merge_NodeInfo.EndPointOffset = lane_element.rest_length;
        NodeInfo = split_merge_NodeInfo;
        return true;
    }
    

    if (split_merge_NodeInfo.EndPointOffset < 0 && split_merge_NodeInfo.LaneChgType > 0){
        NodeInfo = split_merge_NodeInfo;
        return false;
    }
    
    // TODO merge node 和目标方向不一致时，endoffset取merge node的endoffset
    if (DIRECTIONTYPE_UNKNOW != lane_element.dir_type) {
        NodeInfo = change_lane_NodeInfo;
        if (change_lane_NodeInfo.StartPointOffset >= split_merge_NodeInfo.EndPointOffset && 
            change_lane_NodeInfo.StartPointOffset > 0 && split_merge_NodeInfo.EndPointOffset > 0){
            NodeInfo = split_merge_NodeInfo;
            return true;
        }

        if (change_lane_NodeInfo.StartPointOffset >= split_merge_NodeInfo.StartPointOffset && 
            change_lane_NodeInfo.StartPointOffset > 0 && split_merge_NodeInfo.StartPointOffset > 0){
            NodeInfo = split_merge_NodeInfo;
            return true;
        }
        
        if ( change_lane_NodeInfo.EndPointOffset > split_merge_NodeInfo.EndPointOffset && 
            0 == split_merge_NodeInfo.StartPointOffset && 0 != change_lane_NodeInfo.DirectionType.data_ &&
            split_merge_NodeInfo.EndPointOffset > 0) {
                if (change_lane_NodeInfo.DirectionType.data_ == split_merge_NodeInfo.DirectionType.data_ && split_merge_NodeInfo.EndPointOffset <= 50){
                    NodeInfo = split_merge_NodeInfo;
                }else if (ego_lane_index_ == ref_lane_idx_){
                    NodeInfo = split_merge_NodeInfo;
                }else if (split_merge_NodeInfo.LaneChgType > 1 || change_lane_NodeInfo.DirectionType.data_ == split_merge_NodeInfo.DirectionType.data_) {
                    NodeInfo.EndPointOffset = split_merge_NodeInfo.EndPointOffset;
                }
        } else {
            NodeInfo = change_lane_NodeInfo;
        }
    } else {
        NodeInfo = split_merge_NodeInfo;
    }

    if (0 == NodeInfo.StartPointOffset && NodeInfo.LaneChgType > 1 && NodeInfo.LaneChgTimes > 0 && ego_lane_index_  != ref_lane_idx_){
        for (auto& ref_element : lane_element_group){
            if (ref_element.candidate_index == ref_lane_idx_){
                for (int32_t idx = 0; idx < ref_element.lane_num_vec.size() && idx < lane_element.lane_num_vec.size(); idx++){
                    if (ref_element.lane_num_vec[idx] != lane_element.lane_num_vec[idx]){
                        NodeInfo.StartPointOffset = lane_element.lane_extra_infos[idx].s_offset/100.0;
                        break;
                    }
                    
                }
                break;
                
            }
            
        }
        
    }

    if (NodeInfo.EndPointOffset < NodeInfo.StartPointOffset){
        NodeInfo.StartPointOffset = 0;
    }
    

    return true;
}

bool CAlgoRefLaneNode::IsNextMergeShort(LaneElement& lane_element, int32_t close_idx, int32_t& temp_next_close_idx) {
    temp_next_close_idx = close_idx;
    auto temp_merge_value = lane_element.lane_extra_infos[close_idx].merge_value;
    if (!(EFM_MergeType_LEFT_TO_MIDDLE == temp_merge_value || EFM_MergeType_RIGHT_TO_MIDDLE == temp_merge_value)){
        return true;
    }
    
    int32_t first_close_offset = lane_element.lane_extra_infos[close_idx].e_offset;
    for (size_t i = close_idx + 1; i < lane_element.lane_num_vec.size(); i++){
        LaneExtraInfo_s temp_extra = lane_element.lane_extra_infos[i];
        int32_t next_close_offset = temp_extra.e_offset;
        if (next_close_offset <= (first_close_offset + NEXT_MERGE_DIST)){
            if (EFM_SplitType_NONE != temp_extra.split_value){
                break;
            }

            if (EFM_MergeType_NONE == temp_extra.merge_value || EFM_MergeType_FROM_LEFT == temp_extra.merge_value ||
                EFM_MergeType_FROM_RIGHT == temp_extra.merge_value) {
                continue;
            }else{
                temp_next_close_idx = i;
                if ((EFM_MergeType_TO_LEFT == temp_extra.merge_value) || (EFM_MergeType_LEFT_TO_MIDDLE == temp_extra.merge_value)){
                    lane_element.close_to = 1;
                }else{
                    lane_element.close_to = 2;
                }
                break;
            }
            
        }else{
            break;
        }
        
    }

    return true;
}

bool CAlgoRefLaneNode::GetLaneChgTypeClose(std::vector<uint32_t> link_id_vec, LaneElement& lane_element,
                                           int32_t close_idx, uint8_t& LaneChgType) {
    LaneChgType = 1;
    message::map_map::s_LinkInfo_t link_infos;
    if (false == GetLinkInfos(link_id_vec[close_idx], link_infos)) {
        // static data none
        return false;
    }
    uint8_t lane_type = 0;
    GetLaneType(link_infos, lane_element.lane_num_vec[close_idx], lane_type);

    if (lane_element.lane_num_vec.size() < close_idx + 2){
        return false;
    }
    
    uint8_t side_lane_id = 0;
    GetSideLane(link_infos, lane_element.lane_num_vec[close_idx], link_id_vec[close_idx + 1], 
                lane_element.lane_num_vec[close_idx + 1], side_lane_id);
    uint8_t side_lane_type = 0;
    GetLaneType(link_infos, side_lane_id, side_lane_type);

    if ((1 == lane_type || 4 == lane_type || 5 == lane_type || 6 == lane_type || 7 == lane_type) &&
        (!(1 == side_lane_type || 4 == side_lane_type || 5 == side_lane_type || 6 == side_lane_type || 7 == side_lane_type))) {
        if (lane_element.lane_num_vec.size() > (close_idx + 1)) {
            message::map_map::s_LinkInfo_t next_link_infos;
            if (false == GetLinkInfos(link_id_vec[close_idx + 1], next_link_infos)) {
                // static data none
                return false;
            }
            uint8_t next_lane_type = 0;
            GetLaneType(next_link_infos, lane_element.lane_num_vec[close_idx + 1], next_lane_type);

            if (!(1 == next_lane_type || 4 == next_lane_type || 5 == next_lane_type || 6 == next_lane_type ||
                  7 == next_lane_type)) {
                LaneChgType = LaneChgType | 1 << 3;
                LaneChgType = LaneChgType | 1 << 1;
            }
        }
    }

    if (false == lane_element.is_virtually_close) {
        LaneChgType = LaneChgType | 1 << 1;
    }

    if (3 == LaneChgType){
        //main merge or ramp merge
        LaneChgType = LaneChgType | 1 << 4;
    }
    

    return true;
}

bool CAlgoRefLaneNode::GetStartOffsetClose(std::vector<uint32_t> link_id_vec, LaneElement& lane_element,
                                           int32_t close_idx, uint8_t& LaneChgType, double& start_offset) {
    start_offset = 0;
    if ((1 << 3) == (LaneChgType & (1 << 3))) {
        for (int8_t temp_close_idx = close_idx; temp_close_idx >= 0; temp_close_idx--) {
            message::map_map::s_LinkInfo_t link_infos;
            if (false == GetLinkInfos(link_id_vec[temp_close_idx], link_infos)) {
                // static data none
                return false;
            }
            if (IsCLoseLink(link_id_vec[temp_close_idx])) {
                if (link_infos.PathOffset.PathOffset > map_position_->PathOffset) {
                    start_offset = (link_infos.PathOffset.PathOffset - map_position_->PathOffset) / 100.0;
                } else {
                    start_offset = 0;
                }
                break;
            }
        }
    } else {
        start_offset = 0;
    }

    return true;
}

bool CAlgoRefLaneNode::GetEndOffsetClose(std::vector<uint32_t> link_id_vec, LaneElement& lane_element,
                                         int32_t close_idx, uint8_t& LaneChgType, double& end_offset) {
    end_offset = lane_element.rest_length;
    if ((close_idx + 1) > link_id_vec.size() || (close_idx + 1) > lane_element.lane_num_vec.size()){
        return false;
    }
    
    message::map_map::s_LinkInfo_t link_infos;
    if (false == GetLinkInfos(link_id_vec[close_idx], link_infos)) {
        // static data none
        return false;
    }

    if ((LaneChgType & (1 << 1)) == (1 << 1)){
        uint32_t temp_endoffset;
        if (false == GetEndOffsetCloseForArrowLane(link_id_vec, lane_element, close_idx, temp_endoffset)){
            temp_endoffset = link_infos.EndOffset.EndOffset;
        }
        
        if (temp_endoffset <= map_position_->PathOffset){
            end_offset = -1.0;
        }else{
            end_offset = (temp_endoffset - map_position_->PathOffset) / 100.0;
        }
        
    }else if (link_infos.EndOffset.EndOffset > map_position_->PathOffset) {
        end_offset = (link_infos.EndOffset.EndOffset - map_position_->PathOffset) / 100.0;
    }
    
    return true;
}

bool CAlgoRefLaneNode::GetEndOffsetCloseForArrowLane(std::vector<uint32_t> link_id_vec, LaneElement& lane_element,
                                                     int32_t close_idx, uint32_t& end_offset) {
    end_offset = 0;
    if ((close_idx + 1) > link_id_vec.size() || (close_idx + 1) > lane_element.lane_num_vec.size()){
        return false;
    }
    
    std::vector<EFMPoint> combine_left_geometry_points{};
    std::vector<EFMPoint> combine_right_geometry_points{};
    for (int idx = close_idx; idx >= 0; idx--){
        message::map_map::s_LinkInfo_t link_infos;
        if (false == GetLinkInfos(link_id_vec[idx], link_infos)) {
            // static data none
            return false;
        }
        // std::cout << __FILE__ << "," << __LINE__ << "," << " link_id: " << link_id_vec[idx] << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "," << " lane_id: " << (int)lane_element.lane_num_vec[idx] << std::endl;

        uint32_t left_line = 0;
        uint8_t  left_line_type = 0;
        uint32_t right_line = 0;
        uint8_t  right_line_type = 0;
        if (1 == lane_element.close_to){
            GetLineType(link_infos, lane_element.lane_num_vec[idx], left_line_type, true, left_line);
            if (8 == left_line_type){
                //Virtually
                GetLineType(link_infos, lane_element.lane_num_vec[idx] + 1, left_line_type, false, left_line);
            }
            GetLineType(link_infos, lane_element.lane_num_vec[idx], right_line_type, false, right_line);
        }else if (2 == lane_element.close_to){
            GetLineType(link_infos, lane_element.lane_num_vec[idx], right_line_type, false, right_line);
            if (8 == right_line_type){
                //Virtually
                GetLineType(link_infos, lane_element.lane_num_vec[idx] - 1, right_line_type, true, right_line);
            }
            GetLineType(link_infos, lane_element.lane_num_vec[idx], left_line_type, true, left_line);
        }else{
            return false;
        }
        // std::cout << __FILE__ << "," << __LINE__ << "," << " left_line: " << left_line << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "," << " right_line: " << right_line << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "," << " left_line_type: " << (int)left_line_type << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "," << " right_line: " << (int)right_line_type << std::endl;
        //if Virtually
        if (8 == left_line_type && 8 == right_line_type){
            end_offset = link_infos.EndOffset.EndOffset;
            return true;
        }

        double start_distance = 0;
        double end_distance = 0;
        std::vector<EFMPoint> left_geometry_points{};
        std::vector<EFMPoint> right_geometry_points{};
        if (false == GetTwoLineDistance(left_line, right_line, start_distance, end_distance,left_geometry_points,right_geometry_points)){
            return false;
        }
        combine_left_geometry_points.insert(combine_left_geometry_points.begin(),left_geometry_points.begin(),left_geometry_points.end());
        combine_right_geometry_points.insert(combine_right_geometry_points.begin(),right_geometry_points.begin(),right_geometry_points.end());
        // std::cout << __FILE__ << "," << __LINE__ << "," << " start_distance: " << start_distance << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "," << " end_distance: " << end_distance << std::endl;

        if (start_distance <= narrow_lane_width_){
            //back
            end_offset = link_infos.PathOffset.PathOffset;
            continue;
        }

        if (end_distance >= narrow_lane_width_){
            end_offset = link_infos.EndOffset.EndOffset;
            return true;
        }else{
            //计算距离2m的位置
            uint32_t line_start_offset = link_infos.PathOffset.PathOffset;
            std::vector<EFMPoint> left_geometry_points_body{};
            std::vector<EFMPoint> right_geometry_points_body{};
            CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBody(combine_left_geometry_points,left_geometry_points_body,
                                                                          map_position_, static_cast<double>(map_position_->Heading.Heading));
            CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBody(combine_right_geometry_points,right_geometry_points_body,
                                                                          map_position_, static_cast<double>(map_position_->Heading.Heading));
            double gap_first = 0;
            double gap_second = 0;
            uint32_t offset_first = 0;
            uint32_t offset_second = 0;
            double point_dist = 0;
            uint32_t point_offset = 0;
            if(1 == lane_element.close_to){//close to left, left line is base
               for(int i =0;i<right_geometry_points_body.size();i++){
                   SLPoint sl;
                   CommonMathMethod::DiscretePointsMath::CalPointSLBodyCoordinateV2(left_geometry_points_body, right_geometry_points_body[i], sl);
                   if(i>0){
                       point_dist = pow(right_geometry_points_body[i].x - right_geometry_points_body[i-1].x, 2)
                                       + pow(right_geometry_points_body[i].y - right_geometry_points_body[i-1].y, 2);
                       point_dist = sqrt(point_dist);
                       point_offset += static_cast<uint32_t>(point_dist*100);
                   }else{
                       sl.l = start_distance;
                       point_offset = line_start_offset;
                   }
                   
                   if(static_cast<uint32_t>(sl.l*100)>narrow_lane_width_){
                       gap_first = sl.l;
                       offset_first = point_offset;
                   }else{
                       gap_second = sl.l;
                       offset_second = point_offset;
                       break;
                   }
               }
            }else{
               for(int i =0;i<left_geometry_points_body.size();i++){
                   SLPoint sl;
                   CommonMathMethod::DiscretePointsMath::CalPointSLBodyCoordinateV2(right_geometry_points_body, left_geometry_points_body[i], sl);
                   if(i>0){
                       point_dist = pow(left_geometry_points_body[i].x - left_geometry_points_body[i-1].x, 2)
                                       + pow(left_geometry_points_body[i].y - left_geometry_points_body[i-1].y, 2);
                       point_dist = sqrt(point_dist);
                       point_offset += static_cast<uint32_t>(point_dist*100);
                   }else{
                       sl.l = start_distance;
                       point_offset = line_start_offset;
                   }
                   
                   if(static_cast<uint32_t>(sl.l*100)>narrow_lane_width_){
                       gap_first = sl.l;
                       offset_first = point_offset;
                   }else{
                       gap_second = sl.l;
                       offset_second = point_offset;
                       break;
                   }
               }                
            }
            // std::cout << __FILE__ << "," << __LINE__ << "," << " gap_first: " << gap_first << std::endl;
            // std::cout << __FILE__ << "," << __LINE__ << "," << " gap_second: " << gap_second << std::endl;
            // std::cout << __FILE__ << "," << __LINE__ << "," << " offset_first: " << offset_first << std::endl;
            // std::cout << __FILE__ << "," << __LINE__ << "," << " offset_second: " << offset_second << std::endl;
            //根据gao 和offset， 按比例计算2m点的offset
            double ratio = (offset_second - offset_first)/(gap_first - gap_second);
            end_offset = offset_first;
            double temp_length = gap_first - static_cast<double>(narrow_lane_width_)/100.0;
            if (temp_length > 0){
                end_offset += (int)(temp_length * ratio);
            }
            return true;
        }    
        
    }
    
    return true;
}

bool CAlgoRefLaneNode::GetTwoLineDistance(uint32_t left_line, uint32_t right_line,
                                          double& start_distance, double& end_distance, 
                                          std::vector<EFMPoint> &left_geometry_points, std::vector<EFMPoint> &right_geometry_points) {
    if (0 == left_line || 0 == right_line){
        return false;
    }
    
    left_geometry_points.clear();
    if (false == GetLineGeometry(left_line, left_geometry_points)){
        return false;
    }
    
    right_geometry_points.clear();
    if (false == GetLineGeometry(right_line, right_geometry_points)){
        return false;
    }

    if (left_geometry_points.size() < 2 || right_geometry_points.size() < 2){
        return false;
    }

    CommonTool::CoordinateTool::GetInstance()->calculateDistanceGC02(left_geometry_points[0].x,
                                                                 left_geometry_points[0].y,
                                                                 0.0,
                                                                 right_geometry_points[0].x,
                                                                 right_geometry_points[0].y,
                                                                 0.0,
                                                                 start_distance);
    start_distance = start_distance *100.0;

    CommonTool::CoordinateTool::GetInstance()->calculateDistanceGC02(left_geometry_points[left_geometry_points.size() - 1].x,
                                                                 left_geometry_points[left_geometry_points.size() - 1].y,
                                                                 0.0,
                                                                 right_geometry_points[right_geometry_points.size() - 1].x,
                                                                 right_geometry_points[right_geometry_points.size() - 1].y,
                                                                 0.0,
                                                                 end_distance);
    end_distance = end_distance *100.0;

    return true;
}

bool CAlgoRefLaneNode::GetLineGeometry(uint32_t line_id, std::vector<EFMPoint>& geometry_points) {

    geometry_points.clear();
    if (0 == line_id){
        return false;
    }
    
    for (auto& lineinfo : map_static_info_->LinearObjects.LinearObjects) {
        if (line_id == lineinfo.IDLinearObject.IDLinearObject) {
            int point_max_size = (lineinfo.PointCount.PointCount < lineinfo.GeometryPoints.GeometryPoints.size())
                                        ? lineinfo.PointCount.PointCount
                                        : lineinfo.GeometryPoints.GeometryPoints.size();
            for (int point_idx = 0; point_idx < point_max_size; point_idx++) {
                EFMPoint temp_point;
                temp_point.x =
                    static_cast<double>(lineinfo.GeometryPoints.GeometryPoints[point_idx].Longitude.Longitude) *
                    360.0 / (pow(2, 32));
                temp_point.y =
                    static_cast<double>(lineinfo.GeometryPoints.GeometryPoints[point_idx].Latitude.Latitude) *
                    360.0 / (pow(2, 32));
                geometry_points.push_back(temp_point);
            }
            break;
        }
    }

    if (geometry_points.empty()){
        return false;
    }
    
    return true;
}

bool CAlgoRefLaneNode::GetLaneChgTypeSplit(std::vector<uint32_t> link_id_vec, LaneElement& lane_element,
                                           int32_t split_idx, uint8_t& LaneChgType) {
    LaneChgType = 1;
    message::map_map::s_LinkInfo_t link_infos;
    if (false == GetLinkInfos(link_id_vec[split_idx], link_infos)) {
        // static data none
        return false;
    }

    if (false == lane_element.is_virtually_split){
        LaneChgType = LaneChgType | 1 << 1;
    }
    
    uint8_t lane_type = 0;
    GetLaneType(link_infos, lane_element.lane_num_vec[split_idx], lane_type);

    uint8_t side_split_lane_id = 0;
    GetSideLaneForSplit(link_infos, lane_element.lane_num_vec[split_idx], side_split_lane_id);
    uint8_t side_lane_type = 0;
    GetLaneType(link_infos, side_split_lane_id, side_lane_type);

    if ((2 == lane_type || 4 == lane_type || 5 == lane_type || 6 == lane_type || 8 == lane_type) && 
        !(2 == side_lane_type || 4 == side_lane_type || 5 == side_lane_type || 6 == side_lane_type || 8 == side_lane_type)) {
        if (split_idx > 0) {
            message::map_map::s_LinkInfo_t back_link_infos;
            if (false == GetLinkInfos(link_id_vec[split_idx - 1], back_link_infos)) {
                // static data none
                return false;
            }
            uint8_t back_lane_type = 0;
            GetLaneType(back_link_infos, lane_element.lane_num_vec[split_idx - 1], back_lane_type);

            if (!(2 == back_lane_type || 4 == back_lane_type || 5 == back_lane_type || 6 == back_lane_type ||
                  8 == back_lane_type)) {
                LaneChgType = LaneChgType | 1 << 2;
                LaneChgType = LaneChgType | 1 << 1;
            }
        }
    }

    // virtually
    if (lane_element.is_virtually_split && (LaneChgType & (1 << 1)) != (1 << 1)) {
        uint32_t all_length = 0;
        int32_t temp_split_idx = split_idx;
        uint8_t cur_lane_id = lane_element.lane_num_vec[temp_split_idx];
        uint8_t side_lane_id = 0;
        GetSideLaneForSplit(link_infos, cur_lane_id, side_lane_id);
        if (0 == side_lane_id){
            return true;
        }
        
        while (all_length < 15000 && temp_split_idx < lane_element.lane_num_vec.size()) {
            if (IsSplitLinkLane(link_id_vec[temp_split_idx], cur_lane_id, side_lane_id)) {
                LaneChgType = LaneChgType | 1 << 1;
                break;
            }

            if (0 == cur_lane_id || 0 == side_lane_id){
                break;
            }
            

            message::map_map::s_LinkInfo_t link_infos;
            if (false == GetLinkInfos(link_id_vec[temp_split_idx], link_infos)) {
                // static data none
                return false;
            }
            uint32_t temp_length = link_infos.EndOffset.EndOffset - link_infos.PathOffset.PathOffset;
            temp_split_idx++;
            all_length += temp_length;
        }
    }

    return true;
}

bool CAlgoRefLaneNode::GetStartOffsetSplit(std::vector<uint32_t> link_id_vec, LaneElement& lane_element,
                                           int32_t split_idx, uint8_t& LaneChgType, double& start_offset) {
    start_offset = lane_element.rest_length;
    message::map_map::s_LinkInfo_t link_infos;
    if (false == GetLinkInfos(link_id_vec[split_idx], link_infos)) {
        // static data none
        return false;
    }
    if (link_infos.PathOffset.PathOffset > map_position_->PathOffset) {
        start_offset = (link_infos.PathOffset.PathOffset - map_position_->PathOffset) / 100.0;
    } else {
        start_offset = 0;
    }

    return true;
}

bool CAlgoRefLaneNode::GetEndOffsetSplit(std::vector<uint32_t> link_id_vec, LaneElement& lane_element,
                                         int32_t split_idx, uint8_t& LaneChgType, double& end_offset) {
    end_offset = lane_element.rest_length;
    if ((1 << 2) == (LaneChgType & (1 << 2))) {
        for (uint8_t temp_split_idx = split_idx; temp_split_idx < lane_element.lane_num_vec.size(); temp_split_idx++) {
            message::map_map::s_LinkInfo_t link_infos;
            if (false == GetLinkInfos(link_id_vec[temp_split_idx], link_infos)) {
                // static data none
                return false;
            }
            if (IsSplitLink(link_id_vec[temp_split_idx])) {
                if (link_infos.EndOffset.EndOffset > map_position_->PathOffset) {
                    end_offset = (link_infos.EndOffset.EndOffset - map_position_->PathOffset) / 100.0;
                } else {
                    end_offset = 0;
                }
                break;
            }
        }
    } else {
        end_offset = lane_element.rest_length;
    }

    return true;
}

bool CAlgoRefLaneNode::IsChangeToRouteLane(uint32_t link_id, uint8_t& lane_id, std::vector<uint8_t> link_route_lanes) {
    if (std::find(link_route_lanes.begin(), link_route_lanes.end(), lane_id) != link_route_lanes.end()) {
        return true;
    }

    message::map_map::s_LinkInfo_t link_infos;
    if (false == GetLinkInfos(link_id, link_infos)) {
        // static data none
        return true;
    }

    int32_t min_step = 100;
    uint8_t near_route_lane_id = 0;
    for (size_t i = 0; i < link_route_lanes.size(); i++) {
        int32_t temp_step = abs(link_route_lanes[i] - lane_id);
        if (temp_step < min_step) {
            min_step = temp_step;
            near_route_lane_id = link_route_lanes[i];
        }
    }

    bool break_flag = false;
    uint8_t temp_lane_id = lane_id;
    if (lane_id > near_route_lane_id) {
        for (uint8_t step_lane_id = temp_lane_id; step_lane_id > near_route_lane_id; step_lane_id--) {
            uint8_t line_type = 0;
            uint32_t temp_line_id = 0;
            GetLineType(link_infos, step_lane_id, line_type, false, temp_line_id);
            if (0 == line_type || 2 == line_type || 4 == line_type || 6 == line_type) {
                lane_id = step_lane_id;
                break_flag = true;
                break;
            }
        }
    } else {
        for (uint8_t step_lane_id = temp_lane_id; step_lane_id < near_route_lane_id; step_lane_id++) {
            uint8_t line_type = 0;
            uint32_t temp_line_id = 0;
            GetLineType(link_infos, step_lane_id, line_type, true, temp_line_id);
            if (0 == line_type || 2 == line_type || 4 == line_type || 7 == line_type) {
                lane_id = step_lane_id;
                break_flag = true;
                break;
            }
        }
    }

    return (false == break_flag);
}

bool CAlgoRefLaneNode::MakeNodeForLastLinkNotINRoute(LaneElementGroupSets& lane_element_group_sets,
                                                     LaneElement& lane_element,
                                                     std::vector<uint32_t> link_id_vec){
    if (0 == lane_element.lane_num_vec.size() || 0 == link_id_vec.size() ||
       lane_element.lane_num_vec.size() > link_id_vec.size() || 
       lane_element.lane_extra_infos.size() > link_id_vec.size()){
        return false;
    }

    uint32_t last_link_id = link_id_vec[lane_element.lane_num_vec.size() - 1];
    uint32_t next_link_id = 0;
    if (false == GetNextLinkIDForRoute(last_link_id, next_link_id)) {
        // last link
        return true;
    }

    if (1 == lane_element.lane_num_vec.size()){
        return true;
    }
    
    DirectionType dir_type = DIRECTIONTYPE_UNKNOW;
    std::vector<uint8_t> connect_next_link_lanes;
    std::vector<uint8_t> next_link_lanes;
    int32_t merge_link_idx = -1;
    for (int32_t link_idx = lane_element.lane_num_vec.size() - 1 ; link_idx >= 0; link_idx--){
        // std::cout << __FILE__ << "," << __LINE__ << "," << " link_idx: " << link_idx << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "," << " next_link_lanes: " << next_link_id << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "," << " last_link_id: " << last_link_id << std::endl;

        message::map_map::s_LinkInfo_t next_link_infos;
        if (false == GetLinkInfos(next_link_id, next_link_infos)) {
            // static data none
            break;
        }
        GetConnectLinkLanes(next_link_id, last_link_id, connect_next_link_lanes, next_link_lanes);
        if (0 == connect_next_link_lanes.size()) {
            // no connect
            merge_link_idx = link_idx;
            break;
        }

        if (link_idx == lane_element.lane_num_vec.size() - 1){
            if (lane_element.lane_num_vec[link_idx] > connect_next_link_lanes[0]){
                lane_element.dir_type = DIRECTIONTYPE_RIGHT;
            }
            else{
                lane_element.dir_type = DIRECTIONTYPE_LEFT;
            }   
        }
        lane_element.change_times = 1;
        
        next_link_id = last_link_id;
        if (link_idx > 0){
            last_link_id = link_id_vec[link_idx - 1];
        }
        next_link_lanes.clear();
        next_link_lanes.assign(connect_next_link_lanes.begin(), connect_next_link_lanes.end());
        connect_next_link_lanes.clear();
    }

    // std::cout << __FILE__ << "," << __LINE__ << "," << " merge_link_idx: " << merge_link_idx << std::endl;
    if (-1 == merge_link_idx){
        lane_element.lane_change_position = 0;
    }
    else{
        message::map_map::s_LinkInfo_t link_infos;
        if (false == GetLinkInfos(link_id_vec[merge_link_idx], link_infos)) {
            // static data none
            return true;
        }
        //lane_element.lane_change_position = s_offset > 0.0 ? s_offset : 0.0;  
        int temp_merge_link_idx = merge_link_idx;
        for (size_t i = merge_link_idx + 1; i < lane_element.lane_extra_infos.size(); i++){

            message::map_map::s_LinkInfo_t temp_link_infos;
            if (false == GetLinkInfos(link_id_vec[i], temp_link_infos)) {
                // static data none
                return true;
            }
            if (temp_link_infos.EndOffset.EndOffset > link_infos.EndOffset.EndOffset + 20000){
                break;
            }
            

            if (lane_element.lane_extra_infos[i].merge_value == EFM_MergeType_FROM_RIGHT && lane_element.dir_type == DIRECTIONTYPE_RIGHT){
                temp_merge_link_idx = i;
                break;
            }

            if (lane_element.lane_extra_infos[i].merge_value == EFM_MergeType_FROM_LEFT && lane_element.dir_type == DIRECTIONTYPE_LEFT){
                temp_merge_link_idx = i;
                break;
            }            
        }
        
        if (temp_merge_link_idx > merge_link_idx){
            if (false == GetLinkInfos(link_id_vec[temp_merge_link_idx], link_infos)) {
                // static data none
                return true;
            }
            lane_element.dir_type = DIRECTIONTYPE_UNKNOW;
        }

        double s_offset = (link_infos.EndOffset.EndOffset - map_position_->PathOffset) / 100.0;
        lane_element.lane_change_position = s_offset > 0.0 ? s_offset : 0.0;  
    }
    
    return true;

}

}  // namespace framework
}  // namespace shell
}  // namespace earth
