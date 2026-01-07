#include "CandidateLanesModel.h"

namespace earth {
namespace shell {
namespace framework {
   std::map<uint64_t,uint8_t> CandidateLanesModel::split_lind_id_lane_id_map={};//key: linkid低32位 laneid 高32位； value：保存的是修改了split属性的属性
   std::map<uint64_t,uint8_t> CandidateLanesModel::merge_lind_id_lane_id_map={};//key: linkid低32位 laneid 高32位； value：保存的是修改了split属性的属性
   std::deque<uint64_t> CandidateLanesModel::split_deque={};//保存的是修改了split属性的属性
   std::deque<uint64_t> CandidateLanesModel::merge_deque={};//保存的是修改了merge属性的属性
   std::map<uint64_t,SplitInfo_S> CandidateLanesModel::split_info_map = {}; //key:linkid低32位 laneid 高32位； 
   std::deque<uint64_t> CandidateLanesModel::split_info_deque = {}; //和split_info_map配套，保存的是split_info_map的key； 
   std::map<uint64_t,uint8_t> CandidateLanesModel::drive_lind_id_lane_id_map={};//key: linkid低32位 laneid 高32位； 保存的是自车走过的link id和link id; second-是path id
   std::deque<uint64_t> CandidateLanesModel::drive_deque={};//保存的是drive_lind_id_lane_id_map的key; 

bool CandidateLanesModel::Execute(
    const message_common::c2c_message::s_EgoMotion2_t& ego_motion,
    const message::map_position::s_Position_t& map_position, const TopicTrait::MapRouteListMsg& map_route_list,
    const TopicTrait::MapMapMsg& map_static_info, const std::unordered_map<uint32_t, int>& link_id_index_lane_info_map,
    const std::unordered_map<uint32_t, std::vector<int>>& link_id_index_lane_connect_map,
    const std::unordered_map<uint32_t, std::vector<int>>& to_link_id_index_lane_connect_map,
    const std::unordered_map<uint64_t, int>& curve_index_map, const std::unordered_map<uint32_t, int>& linear_obj_id_map) {
    // get input
    ego_motion_ = std::make_shared<const message_common::c2c_message::s_EgoMotion2_t>(ego_motion);
    map_position_ = std::make_shared<const message::map_position::s_Position_t>(map_position);
    map_route_list_ = std::make_shared<const TopicTrait::MapRouteListMsg>(map_route_list);
    map_static_info_ = std::make_shared<const TopicTrait::MapMapMsg>(map_static_info);
    link_id_index_lane_info_map_ =
        std::make_shared<const std::unordered_map<uint32_t, int>>(link_id_index_lane_info_map);
    link_id_index_lane_connect_map_ =
        std::make_shared<const std::unordered_map<uint32_t, std::vector<int>>>(link_id_index_lane_connect_map);
    to_link_id_index_lane_connect_map_ =
        std::make_shared<const std::unordered_map<uint32_t, std::vector<int>>>(to_link_id_index_lane_connect_map);
    curve_index_map_ = std::make_shared<const std::unordered_map<uint64_t, int>>(curve_index_map);
    linear_obj_id_map_ = std::make_shared<const std::unordered_map<uint32_t, int>>(linear_obj_id_map);

    p_CAlgoRefLaneNode_ = std::make_shared<CAlgoRefLaneNode>();
    p_CAlgoRefLaneNode_->InitVariable(map_position, map_route_list, map_static_info, link_id_index_lane_info_map,
                                      link_id_index_lane_connect_map, to_link_id_index_lane_connect_map);
    get_toll_link();
    StoreEgoDrivePathLinkLane(drive_lind_id_lane_id_map, drive_deque, map_position.PathId, map_position.LaneId, map_position.LinkId);
    // get all candidate lanes
    GenerateCandidateLanesV2();

    GetLinkLength(link_id_vec_, link_length_vec_);

// #undef CLM_COUT
#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << " raw all_lanes_vec_vec_.size(): " << all_lanes_vec_vec_.size()
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " raw all_lanes_vec_vec_:: " << std::endl;
    for (int i = 0; i < all_lanes_vec_vec_.size(); i++) {
        std::cout << " all_lanes_vec_vec_[ " << i << " ]::  [";
        for (auto lane_num : all_lanes_vec_vec_[i]) {
            std::cout << " " << static_cast<int>(lane_num) << ",";
        }
        std::cout << "]" << std::endl;
    }

    std::cout << __FILE__ << "," << __LINE__ << ","
              << " raw all_lanes_extra_vec_vec_.size(): " << all_lanes_extra_vec_vec_.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " raw all_lanes_extra_vec_vec_::transit:: " << std::endl;
    for (int i = 0; i < all_lanes_extra_vec_vec_.size(); i++) {
        std::cout << " all_lanes_extra_vec_vec_transit[ " << i << " ]::  [";
        for (auto extra_info : all_lanes_extra_vec_vec_[i]) {
            std::cout << " " << static_cast<int>(extra_info.transit) << ",";
        }
        std::cout << "]" << std::endl;
    }
#endif
    // TODO潜在优化点
    // DeleteEmergencyLane(all_lanes_vec_vec_, link_id_vec_);

    // GetDrvLaneSize(all_lanes_vec_vec_, driveable_lane_size_);

    // TODO 该函数没有被用到
    GetAvailableLaneNum(map_position_->LinkId, min_driveable_lane_num_, max_driveable_lane_num_);
// PLOT
// #undef CLM_COUT
#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << " final all_lanes_vec_vec_.size(): " << all_lanes_vec_vec_.size()
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " final all_lanes_vec_vec_:: " << std::endl;
    for (int i = 0; i < all_lanes_vec_vec_.size(); i++) {
        std::cout << " all_lanes_vec_vec_[ " << i << " ]::  [";
        for (auto lane_num : all_lanes_vec_vec_[i]) {
            std::cout << " " << static_cast<int>(lane_num) << ",";
        }
        std::cout << "]" << std::endl;
    }

    std::cout << __FILE__ << "," << __LINE__ << ","
              << " final all_lanes_extra_vec_vec_.size(): " << all_lanes_extra_vec_vec_.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " final all_lanes_extra_vec_vec_:transit: " << std::endl;
    for (int i = 0; i < all_lanes_extra_vec_vec_.size(); i++) {
        std::cout << "final all_lanes_extra_vec_vec_ transit[ " << i << " ]::  [";
        for (auto extra_info : all_lanes_extra_vec_vec_[i]) {
            std::cout << " " << static_cast<int>(extra_info.transit) << ",";
        }
        std::cout << "]" << std::endl;
    }

    std::cout << " link_id_vec_.size(): " << link_id_vec_.size() << std::endl;
    std::cout << " link_id_vec_:: [ ";
    for (int i = 0; i < link_id_vec_.size(); i++) {
        std::cout << " " << static_cast<int>(link_id_vec_[i]) << ",";
    }
    std::cout << " " << std::endl;
    std::stringstream ss;
    for (auto link_id : map_route_list.LinkIds.LinkIds) {
        ss << " " << link_id.LinkId << ",";
    }
    std::cout << "map_route_list-" << ss.str() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "link length: ";
    for (auto length : link_length_vec_) {
        std::cout << length << " , ";
    }
    std::cout << " " << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << " final all_emergency_lanes_vec_vec_.size(): " << all_emergency_lanes_vec_vec_.size()
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " final all_emergency_lanes_vec_vec_:: " << std::endl;
    for (int i = 0; i < all_emergency_lanes_vec_vec_.size(); i++) {
        std::cout << " all_emergency_lanes_vec_vec_[ " << i << " ]::  [";
        for (auto lane_num : all_emergency_lanes_vec_vec_[i]) {
            std::cout << " " << static_cast<int>(lane_num) << ",";
        }
        std::cout << "]" << std::endl;
    }
#endif

    // // only used in lane maker generate in Extract Refline
    // ego_lane_index_for_lane_mkr_ = -1;
    // left_lane_index_for_lane_mkr_ = -1;
    // right_lane_index_for_lane_mkr_ = -1;
    // GetPriorCandidateLanesV2(all_lanes_vec_vec_, map_position_, link_id_vec_, is_contain_split_vec_,
    //                          candidate_lanes_split_nodes_, left_lane_index_for_lane_mkr_, ego_lane_index_for_lane_mkr_,
    //                          right_lane_index_for_lane_mkr_);
    // // std::cout << __FILE__ << "," << __LINE__ << ","
    // //           << " left_lane_index_for_lane_mkr_: " << left_lane_index_for_lane_mkr_ << std::endl;
    // // std::cout << __FILE__ << "," << __LINE__ << ","
    // //           << " ego_lane_index_for_lane_mkr_: " << ego_lane_index_for_lane_mkr_ << std::endl;
    // // std::cout << __FILE__ << "," << __LINE__ << ","
    // //           << " right_lane_index_for_lane_mkr_: " << right_lane_index_for_lane_mkr_ << std::endl;
    ////
    // get ego ,left ,right lane
    ego_lane_prior_index_ = -1;
    left_lane_prior_index_ = -1;
    left_left_lane_prior_index_ = -1;
    right_lane_prior_index_ = -1;
    right_right_lane_prior_index_ = -1;
    ref_lane_prior_index_ = -1;
    ref_lane_split_position_ = -1;
    ref_lane_next_split_position_ = 1;
    ref_lane_split_from_ = 0;
    ref_route_lane_prior_index_ = -1;
    std::map<uint64_t,uint8_t> temp_split_lind_id_lane_id_map;
    ConstructLaneElement(all_lanes_vec_vec_, all_lanes_extra_vec_vec_, lane_element_group_sets_);
    p_CAlgoRefLaneNode_->GetRefLine(link_id_vec_, lane_element_group_sets_, ego_lane_prior_index_,
                                    left_lane_prior_index_, left_left_lane_prior_index_, 
                                    right_lane_prior_index_, right_right_lane_prior_index_,
                                    ref_lane_prior_index_, temp_split_lind_id_lane_id_map);
    if (temp_split_lind_id_lane_id_map.size() > 0){
        for (auto &iter : temp_split_lind_id_lane_id_map){
            split_lind_id_lane_id_map[iter.first] = iter.second;
        }
    }
    
    if (-1 == ego_lane_prior_index_) {
        return false;
    }

    // get ref split position
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets_.size(); lane_idx++) {
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets_[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets_[lane_idx][lane_dir_idx];
            if (lane_element.candidate_index == ref_lane_prior_index_ &&
                lane_element.split_position < lane_element.lane_num_vec.size()) {
                ref_lane_split_position_ = lane_element.split_position;
                ref_lane_split_from_ = lane_element.split_from;

                if (lane_element.next_split_position < lane_element.lane_num_vec.size()) {
                    ref_lane_next_split_position_ = lane_element.next_split_position;
                }

                break;
            }
        }
    }

    // std::cout << __FILE__ << "," << __LINE__ << "," << " ego_lane_prior_index_: " << ego_lane_prior_index_ <<
    // std::endl; std::cout << __FILE__ << "," << __LINE__ << "," << " left_lane_prior_index_: " <<
    // left_lane_prior_index_ << std::endl; std::cout << __FILE__ << "," << __LINE__ << "," << "
    // right_lane_prior_index_: " << right_lane_prior_index_ << std::endl; std::cout << __FILE__ << "," << __LINE__ <<
    // "," << " ref_lane_prior_index_: " << ref_lane_prior_index_ << std::endl;
    NodeInfo_ = {};
    p_CAlgoRefLaneNode_->GetNode(link_id_vec_, lane_element_group_sets_, ego_lane_prior_index_, NodeInfo_, ref_route_lane_prior_index_);
    // std::cout << __FILE__ << "," << __LINE__ << "," << " NodeInfo_: " << NodeInfo_.StartPointOffset << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << " NodeInfo_: " << NodeInfo_.EndPointOffset << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << " NodeInfo_: " << int(NodeInfo_.DirectionType.data_) <<
    // std::endl; std::cout << __FILE__ << "," << __LINE__ << "," << " NodeInfo_: " << int(NodeInfo_.LaneChgType) <<
    // std::endl; std::cout << __FILE__ << "," << __LINE__ << "," << " NodeInfo_: " << int(NodeInfo_.LaneChgTimes) <<
    // std::endl;
#ifdef EM_COUT
    // ZTEXT("EFM_INFO", "ego_lane_prior_index: ", 120, -10, "ego_lane_prior_index: {}", ego_lane_prior_index_);
    if (ego_lane_prior_index_ >= 0) {
        ZTEXT("EFM_INFO", "ego_lane_: ", 120, -13, "ego_lane_: {}", all_lanes_vec_vec_[ego_lane_prior_index_][0]);
    }
    // ZTEXT("EFM_INFO", "left_lane_prior_index_: ", 120, -16, "left_lane_prior_index_: {}", left_lane_prior_index_);
    if (left_lane_prior_index_ >= 0) {
        ZTEXT("EFM_INFO", "left_lane: ", 120, -19, "left_lane: {}", all_lanes_vec_vec_[left_lane_prior_index_][0]);
    }
    // ZTEXT("EFM_INFO", "right_lane_prior_index_: ", 120, -22, "right_lane_prior_index_: {}", right_lane_prior_index_);
    if (right_lane_prior_index_ >= 0) {
        ZTEXT("EFM_INFO", "right_lane: ", 120, -25, "right_lane: {}", all_lanes_vec_vec_[right_lane_prior_index_][0]);
    }
    // ZTEXT("EFM_INFO", "ref_lane_prior_index_: ", 120, -28, "ref_lane_prior_index_: {}", ref_lane_prior_index_);
    if (ref_lane_prior_index_ >= 0) {
        ZTEXT("EFM_INFO", "ref_lane: ", 120, -31, "ref_lane: {}", all_lanes_vec_vec_[ref_lane_prior_index_][0]);
    }
    // ZTEXT("EFM_INFO", "node.StartPointOffset: ", 120, -34, "StartPointOffset: {}", NodeInfo_.StartPointOffset);
    // ZTEXT("EFM_INFO", "node.EndPointOffset: ", 120, -37, "EndPointOffset: {}", NodeInfo_.EndPointOffset);
    // ZTEXT("EFM_INFO", "node.DirectionType: ", 120, -40, "DirectionType: {}", int(NodeInfo_.DirectionType.data_));
    // ZTEXT("EFM_INFO", "node.LaneChgType: ", 120, -43, "LaneChgType: {}", int(NodeInfo_.LaneChgType));
    // ZTEXT("EFM_INFO", "node.LaneChgTimes: ", 120, -46, "LaneChgTimes: {}", int(NodeInfo_.LaneChgTimes));
#endif
    // GetPriorCandidateLanes(map_position_->LaneId, all_lanes_vec_vec_, ego_lane_prior_index_);
    // GetPriorCandidateLanes(map_position_->LaneId + 1, all_lanes_vec_vec_, left_lane_prior_index_);
    // GetPriorCandidateLanes(map_position_->LaneId - 1, all_lanes_vec_vec_, right_lane_prior_index_);
    // GetDrvLaneSizeV2(lane_element_group_sets_, min_driveable_lane_num_, max_driveable_lane_num_,
    // driveable_lane_size_,
    //                  right_not_driveable_lane_size_);
    GetDrvLaneSizeV3(lane_element_group_sets_, driveable_lane_size_, fixed_lane_id_);
    GetLaneWidth(all_lanes_vec_vec_, link_id_vec_);

#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " ego_lane_width_min_vec_.size(): " << ego_lane_width_min_vec_.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " ego_lane_width_max_vec_.size(): " << ego_lane_width_max_vec_.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " left_lane_width_min_vec_.size(): " << left_lane_width_min_vec_.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " left_lane_width_max_vec_.size(): " << left_lane_width_max_vec_.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " right_lane_width_min_vec_.size(): " << right_lane_width_min_vec_.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " right_lane_width_max_vec_.size(): " << right_lane_width_max_vec_.size() << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << "ego_lane_width_min_vec_: ";
    for (auto width : ego_lane_width_min_vec_) {
        std::cout << width << " , ";
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << "ego_lane_width_max_vec_: ";
    for (auto width : ego_lane_width_max_vec_) {
        std::cout << width << " , ";
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << "left_lane_width_min_vec_: ";
    for (auto width : left_lane_width_min_vec_) {
        std::cout << width << " , ";
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << "left_lane_width_max_vec_: ";
    for (auto width : left_lane_width_max_vec_) {
        std::cout << width << " , ";
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << "right_lane_width_min_vec_: ";
    for (auto width : right_lane_width_min_vec_) {
        std::cout << width << " , ";
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << "right_lane_width_max_vec_: ";
    for (auto width : right_lane_width_max_vec_) {
        std::cout << width << " , ";
    }
    std::cout << std::endl;

#endif

    ego_lane_prior_length_ = 0;
    left_lane_prior_length_ = 0;
    left_left_lane_prior_length_ = 0;
    right_lane_prior_length_ = 0;
    right_right_lane_prior_length_ = 0;
    ego_lane_prior_dist_ = 0;    // unit m
    left_lane_prior_dist_ = 0;   // unit m
    left_left_lane_prior_dist_ = 0;   // unit m
    right_lane_prior_dist_ = 0;  // unit m
    right_right_lane_prior_dist_ = 0;  // unit m
    emergency_right_lane_dist_ = 0;
    emergency_left_lane_dist_ = 0;
#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << "left_lane_prior_index_  " << left_lane_prior_index_
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "ego_lane_prior_index_  " << ego_lane_prior_index_ << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "right_lane_prior_index_  " << right_lane_prior_index_
              << std::endl;
#endif
    GetLaneDist(all_lanes_vec_vec_, map_position_, link_id_vec_, ego_lane_prior_index_, ego_lane_prior_dist_);
    GetLaneDist(all_lanes_vec_vec_, map_position_, link_id_vec_, left_lane_prior_index_, left_lane_prior_dist_);
    GetLaneDist(all_lanes_vec_vec_, map_position_, link_id_vec_, left_left_lane_prior_index_, left_left_lane_prior_dist_);
    GetLaneDist(all_lanes_vec_vec_, map_position_, link_id_vec_, right_lane_prior_index_, right_lane_prior_dist_);
    GetLaneDist(all_lanes_vec_vec_, map_position_, link_id_vec_, right_right_lane_prior_index_, right_right_lane_prior_dist_);
    GetLaneDist(all_emergency_lanes_vec_vec_, map_position_, link_id_vec_, emergency_left_index_, emergency_left_lane_dist_);
    GetLaneDist(all_emergency_lanes_vec_vec_, map_position_, link_id_vec_, emergency_right_index_, emergency_right_lane_dist_);
    // std::cout << __FILE__ << "," << __LINE__ << "," << "emergency_left_lane_dist_  " << emergency_left_lane_dist_ << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "," << "emergency_right_lane_dist_  " << emergency_right_lane_dist_ << std::endl;
    GetLaneLength(all_lanes_vec_vec_, ego_lane_prior_index_, ego_lane_prior_length_);
    GetLaneLength(all_lanes_vec_vec_, left_lane_prior_index_, left_lane_prior_length_);
    GetLaneLength(all_lanes_vec_vec_, left_left_lane_prior_index_, left_left_lane_prior_length_);
    GetLaneLength(all_lanes_vec_vec_, right_lane_prior_index_, right_lane_prior_length_);
    GetLaneLength(all_lanes_vec_vec_, right_right_lane_prior_index_, right_right_lane_prior_length_);

    GetBackConnectInfo(all_lanes_vec_vec_, link_id_vec_);
    GetBackConnectInfoEmergency(all_emergency_lanes_vec_vec_,link_id_vec_,map_position_->LaneId);

    ego_path_lane_num_.clear();
    left_path_lane_num_.clear();
    left_left_path_lane_num_.clear();
    right_path_lane_num_.clear();
    right_right_path_lane_num_.clear();
    if (ego_lane_prior_index_ >= 0 && ego_lane_prior_index_ < all_lanes_vec_vec_.size()) {
        ego_path_lane_num_ = all_lanes_vec_vec_[ego_lane_prior_index_];
    }
    if (left_lane_prior_index_ >= 0 && left_lane_prior_index_ < all_lanes_vec_vec_.size()) {
        left_path_lane_num_ = all_lanes_vec_vec_[left_lane_prior_index_];
    }
    if (left_left_lane_prior_index_ >= 0 && left_left_lane_prior_index_ < all_lanes_vec_vec_.size()) {
        left_left_path_lane_num_ = all_lanes_vec_vec_[left_left_lane_prior_index_];
    }
    if (right_lane_prior_index_ >= 0 && right_lane_prior_index_ < all_lanes_vec_vec_.size()) {
        right_path_lane_num_ = all_lanes_vec_vec_[right_lane_prior_index_];
    }
    if (right_right_lane_prior_index_ >= 0 && right_right_lane_prior_index_ < all_lanes_vec_vec_.size()) {
        right_right_path_lane_num_ = all_lanes_vec_vec_[right_right_lane_prior_index_];
    }
    
    //1029 new add
    if(ego_lane_prior_index_<all_lanes_extra_vec_vec_.size() && ego_path_lane_num_.size()>0){
        pre_cutin_merge_offset_vec_.clear();
        GetEgoLanePreCutInMerge(link_id_vec_, ego_path_lane_num_, 
                                                  all_lanes_extra_vec_vec_[ego_lane_prior_index_], pre_cutin_merge_offset_vec_);
    }

//  get path line
// #undef CLM_COUT
#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << "pre_cutin_merge_offset_vec_.size(): " << pre_cutin_merge_offset_vec_.size()<< std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "pre_cutin_merge_offset_vec_: " << std::endl;
    for(auto offset:pre_cutin_merge_offset_vec_){
        std::cout<<" ,"<<offset;
    }    
    std::cout<<std::endl;
    
    std::cout << __FILE__ << "," << __LINE__ << "," << "ego_lane_prior_length_: " << ego_lane_prior_length_
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "left_lane_prior_length_: " << left_lane_prior_length_
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "right_lane_prior_length_: " << right_lane_prior_length_
              << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << "is_contain_split_vec_.size(): " << is_contain_split_vec_.size()
              << std::endl;
    std::stringstream ss2;
    for (auto is_split : is_contain_split_vec_) {
        ss2 << " " << static_cast<int>(is_split) << ",";
    }
    std::cout << "is_contain_split_vec_-" << ss2.str() << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << ","
              << "candidate_lanes_split_nodes_.size(): " << candidate_lanes_split_nodes_.size() << std::endl;
    std::stringstream ss3, ss4;
    for (auto node : candidate_lanes_split_nodes_) {
        ss3 << " " << node.second.origin_node.link_id << ",";
        ss4 << "origin_link: " << node.second.origin_node.link_id;
        for (auto split : node.second.split_nodes) {
            ss4 << " link_: " << split.link_id << " lane_num: " << static_cast<int>(split.lane_num);
        }
        ss4 << std::endl;
    }
    std::cout << "origin_node.link_id-" << ss3.str() << std::endl;
    std::cout << "split info: " << std::endl;
    std::cout << ss4.str() << std::endl;

    // lane_element_group_sets_
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "lane_element_group_sets_.size(): " << lane_element_group_sets_.size() << std::endl;
    for (int i = 0; i < lane_element_group_sets_.size(); i++) {
        LaneElementGroup lane_element_group = lane_element_group_sets_[i];
        for (int j = 0; j < lane_element_group.size(); j++) {
            LaneElement lane_element = lane_element_group[j];
            if (j == 0) {
                std::cout << __FILE__ << "," << __LINE__ << "," << " group_sets_: " << i << " :" << std::endl;
            }
            std::cout << "lane_num_vec [" << j << "]: ";
            for (int k = 0; k < lane_element.lane_num_vec.size(); k++) {
                std::cout << "," << (int)lane_element.lane_num_vec[k] << " ";
            }
            std::cout << "rest_dist: " << lane_element.rest_length << " ,index: " << lane_element.candidate_index
                      << std::endl;
            std::cout << "transit:: ";
            for (auto trans : lane_element.transit_type) {
                std::cout << "<index: " << trans.first << " ,trans_type: " << (int)trans.second << ">";
            }
            std::cout << std::endl;
        }
    } 
    std::cout << "split_lind_id_lane_id_map.size(): "<<split_lind_id_lane_id_map.size()<<std::endl;
std::cout << "split_lind_id_lane_id_map: ";
    for(auto iter:split_lind_id_lane_id_map){
       std::cout<<" ,<"<<(iter.first & (uint64_t)4294967295)<<", "<<((iter.first >> 32) & (((uint64_t)4294967295)))<<", "<<(int)iter.second<<">";
    }
    std::cout <<std::endl;

std::cout << "split_deque.size(): "<<split_deque.size()<<std::endl;
std::cout << "split_deque: ";
    for(auto iter:split_deque){
       std::cout<<" ,"<<(iter & (uint64_t)4294967295)<<", "<<(iter >> 32 & (((uint64_t)4294967295)));
    }
    std::cout <<std::endl;


std::cout << "merge_lind_id_lane_id_map.size(): "<<merge_lind_id_lane_id_map.size()<<std::endl;
std::cout << "merge_lind_id_lane_id_map: ";
    for(auto iter:merge_lind_id_lane_id_map){
       std::cout<<" ,<"<<(iter.first & (uint64_t)4294967295)<<", "<<((iter.first >> 32) & (((uint64_t)4294967295)))<<", "<<(int)iter.second<<">";
    }
    std::cout <<std::endl;
std::cout << "merge_deque.size(): "<<merge_deque.size()<<std::endl;
std::cout << "merge_deque: ";
    for(auto iter:merge_deque){
       std::cout<<" ,"<<(iter & (uint64_t)4294967295)<<", "<<(iter >> 32 & (((uint64_t)4294967295)));
    }
    std::cout <<std::endl;  
#endif
#ifdef SPLIT_0930
    std::cout << "split_info_map.size(): "<<split_info_map.size()<<std::endl;
std::cout << "split_info_map: ";
    for(auto iter:split_info_map){
       std::cout<<" ,<"<<(iter.first & (uint64_t)4294967295)<<", "<<((iter.first >> 32) & (((uint64_t)4294967295)))<<",|| "<<iter.second.is_road_split<<", "<<iter.second.s_offset<<", "<<iter.second.e_offset<<", "<<(int)iter.second.split_dir<<">";
    }
    std::cout <<std::endl;

std::cout << "split_info_deque.size(): "<<split_info_deque.size()<<std::endl;
std::cout << "split_info_deque: ";
    for(auto iter:split_info_deque){
       std::cout<<" ,"<<(iter & (uint64_t)4294967295)<<", "<<(iter >> 32 & (((uint64_t)4294967295)));
    }
    std::cout <<std::endl;
    std::cout <<std::endl; 
std::cout << "drive_lind_id_lane_id_map.size(): "<<drive_lind_id_lane_id_map.size()<<std::endl;
std::cout << "drive_lind_id_lane_id_map: ";
    for(auto iter:drive_lind_id_lane_id_map){
       std::cout<<" ,<"<<(iter.first & (uint64_t)4294967295)<<", "<<((iter.first >> 32) & (((uint64_t)4294967295))) <<", "<<(int)iter.second<<">";
    }
    std::cout <<std::endl;
std::cout << "drive_deque.size(): "<<drive_deque.size()<<std::endl;
std::cout << "drive_deque: ";
    for(auto iter:drive_deque){
       std::cout<<", "<<(iter & (uint64_t)4294967295)<<", "<<(iter >> 32 & (((uint64_t)4294967295)));
    }
    std::cout <<std::endl;   
#endif
    return true;
}

/**
 * @brief 生成候选lane groups
 *
 * @return true
 * @return false
 */
bool CandidateLanesModel::GenerateCandidateLanes() { return true; }

/**
 * @brief 生成候选lane groups
 *
 * @return true
 * @return false
 */
bool CandidateLanesModel::GenerateCandidateLanesV2() {
    /*数据示例
    std::vector<std::vector<int>> all_lanes_vec_vec_ = {{0, 1}, {2, 3}, {4}};
    std::vector<int> link_id_vec_ = {0, 1, 2};
    */
    // std::cout << "GenerateCandidateLanesV2 start " << std::endl;
    link_id_vec_.clear();
    all_lanes_vec_vec_.clear();
    is_contain_split_vec_.clear();
    candidate_lanes_split_nodes_.clear();
    all_lanes_extra_vec_vec_.clear();
    all_emergency_lanes_vec_vec_.clear();
    uint8_t ego_lane_num = map_position_->LaneId;
    uint32_t ego_link_id = map_position_->LinkId;
    // int cur_link_index = link_id_index_lane_info_map_->at(cur_link_id);
    message::map_map::s_LinkInfo_t ego_link_infos;
    if (!efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_,*link_id_index_lane_info_map_,ego_link_id, ego_link_infos)) {
        // std::cout << "GenerateCandidateLanesV2-"
        //           << "ego link not found" << std::endl;
        return false;
    }
    uint8_t ego_link_lane_size = ego_link_infos.LaneInfos.LaneInfos.size();
    int ego_rp_index = -1;
    for (int i = 0; i < map_route_list_->LinkIds.LinkIds.size(); i++) {
        if (map_route_list_->LinkIds.LinkIds[i].LinkId == ego_link_id) {
            ego_rp_index = i;
            break;
        }
    }
    if (ego_rp_index < 0) {
        // std::cout << "not found ego link in route  " << std::endl;
        return false;
    }

    link_id_vec_.push_back(ego_link_id);
    size_t routingpath_link_size = map_route_list_->LinkIds.LinkIds.size();

    // 从自车link开始, 找到3，9， 17， 不再往后拼
    // std::cout << "从自车link开始 start " << std::endl;
    {
        for (int i = 0; i < ego_link_lane_size; i++) {
            uint8_t lane_num = ego_link_infos.LaneInfos.LaneInfos[i].LaneNum.LaneNum;
            message::map_map::s_LaneInfo_t lane_info{};
            if(!efm::MapCommonTool::GetInstance()->GetLaneInfo(ego_link_infos, lane_num, lane_info)){
                return false;
            }
            if(lane_info.LaneType.data == 3 || lane_info.LaneType.data == 9 || lane_info.LaneType.data == 17){
                //save to all_emergency_lanes_vec_vec_
                if(lane_info.LaneType.data == 3){
                    all_emergency_lanes_vec_vec_.push_back(std::vector<uint8_t>{lane_num});         
                }
            }else{
                all_lanes_vec_vec_.push_back(std::vector<uint8_t>{lane_num});
                std::vector<LaneExtraInfo_s> lane_extra_info_temp_vec{};
                LaneExtraInfo_s extra_info{};
                SaveLaneAllExtraInfo(ego_link_id, lane_num, extra_info);
                lane_extra_info_temp_vec.push_back(extra_info);
                all_lanes_extra_vec_vec_.push_back(lane_extra_info_temp_vec);                
            }
        }

        if (all_lanes_extra_vec_vec_.size() != all_lanes_vec_vec_.size()) {
            return false;
        }
        // bubble sort, min->max
        for (int i = 0; i < all_lanes_vec_vec_.size(); i++) {
            for (int j = 0; j < all_lanes_vec_vec_.size() - 1 - i; j++) {
                if (all_lanes_vec_vec_[j].back() > all_lanes_vec_vec_[j + 1].back()) {
                    std::vector<uint8_t> tmp{};
                    tmp = all_lanes_vec_vec_[j];
                    all_lanes_vec_vec_[j] = all_lanes_vec_vec_[j + 1];
                    all_lanes_vec_vec_[j + 1] = tmp;

                    std::vector<LaneExtraInfo_s> lane_extra_info_temp_vec{};
                    lane_extra_info_temp_vec = all_lanes_extra_vec_vec_[j];
                    all_lanes_extra_vec_vec_[j] = all_lanes_extra_vec_vec_[j + 1];
                    all_lanes_extra_vec_vec_[j + 1] = lane_extra_info_temp_vec;
                }
            }
        }
    }
//}
// #undef CLM_COUT
#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << "ego_link_infos.EndOffset.EndOffset "
              << ego_link_infos.EndOffset.EndOffset << std::endl;
    std::cout << "just ego , all_lanes_vec_vec_.size(): " << all_lanes_vec_vec_.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " just ego all_lanes_vec_vec_:: " << std::endl;
    for (int i = 0; i < all_lanes_vec_vec_.size(); i++) {
        std::cout << " all_lanes_vec_vec_[ " << i << " ]::  [";
        for (auto lane_num : all_lanes_vec_vec_[i]) {
            std::cout << " " << static_cast<int>(lane_num) << ",";
        }
        std::cout << "]" << std::endl;
    }
    std::cout << "routingpath_link_size: " << routingpath_link_size << std::endl;
    std::cout << "ego_rp_index: " << ego_rp_index << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "ego_link_id " << ego_link_id << std::endl;
#endif
    int valid_link_count = 0;
    int emergency_valid_link_count = 0;
    double forward_distance =
        static_cast<double>(ego_link_infos.EndOffset.EndOffset) - static_cast<double>(map_position_->PathOffset);
    // std::cout << "just ego , forward_distance: " << forward_distance << std::endl;
    double kForwardExploreDistanceThreshold = 200000;  // unit cm
    message::map_map::s_LinkInfo_t link_info_temp{};
    for (size_t i = ego_rp_index; i < routingpath_link_size - 1; i++) {
        valid_link_count = i - ego_rp_index;
        emergency_valid_link_count = i - ego_rp_index;
        // std::cout << "i: " << i << " ,valid_link_count: " << valid_link_count << std::endl;
        uint32_t rp_link_id = map_route_list_->LinkIds.LinkIds[i].LinkId;
        // 当前link在静态地图中的索引
        // int link_index = link_id_index_lane_info_map_->at(rp_link_id);

        if (!efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_,*link_id_index_lane_info_map_,rp_link_id, link_info_temp)) {
            // std::cout << "GenerateCandidateLanesV2-"
            //           << " link not found: " << rp_link_id << std::endl;
            break;
        }

        // get link connect info
        std::vector<int> cur_link_staticmap_indices{};
        if (link_id_index_lane_connect_map_->find(rp_link_id) != link_id_index_lane_connect_map_->end()) {
            cur_link_staticmap_indices = link_id_index_lane_connect_map_->at(rp_link_id);
        } else {
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << " can't find connect : " << rp_link_id << std::endl;
            break;
        }
        uint32_t next_rp_link_id = map_route_list_->LinkIds.LinkIds[i + 1].LinkId;
        if (link_id_index_lane_info_map_->find(next_rp_link_id) == link_id_index_lane_info_map_->end()) {
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << " can't find next_rp_link_id : " << next_rp_link_id << std::endl;
            break;
        }
        std::map<int, int> lane_end;
        for (int j = 0; j < all_lanes_vec_vec_.size(); j++) {
            if (all_lanes_vec_vec_[j].size() == valid_link_count + 1) {
                lane_end.emplace(j, all_lanes_vec_vec_[j].back());
            } else {
                lane_end.emplace(j, 255);
            }
        }
        std::map<int, int> emergency_lane_end;
        for (int j = 0; j < all_emergency_lanes_vec_vec_.size(); j++) {
            if (all_emergency_lanes_vec_vec_[j].size() == emergency_valid_link_count + 1) {
                emergency_lane_end.emplace(j, all_emergency_lanes_vec_vec_[j].back());
            } else {
                emergency_lane_end.emplace(j, 255);
            }
        }
        // std::cout << "i: " << i << " ,lane_end size: " << lane_end.size() << std::endl;
        if(lane_end.size()>1000 || emergency_lane_end.size()>1000){
            //连接关系异常
            return false;
        }

        uint32_t cur_link_connectivity_link_id = 0;
        bool find_connect_lane = false;
        bool find_split_f = false;
        int all_lanes_vec_end_index = 0;  // 因为填充all_lanes_vec_vec_时，有insert操作，所以对于lane_end的[k]，
                                          // insert后，lane_end[k]对应all_lanes_vec_vec_的[K+1]
        for (int k = 0; k < lane_end.size(); k++) {
            int split_lane_number = 1;
            std::vector<LanesNode> lane_node_temp{};
            uint8_t search_link_lanes_num = lane_end[k];
            // std::cout << __FILE__ << "," << __LINE__ << "," << "k: " << k
            //           << " , lane_end: " << static_cast<int>(search_link_lanes_num) << std::endl;
            // std::cout << __FILE__ << "," << __LINE__ << "," << "all_lanes_vec_end_index: " << all_lanes_vec_end_index
            //           << std::endl;
            // for (int i = 0; i < all_lanes_vec_vec_.size(); i++) {
            //     std::cout << "lane end begin internal all_lanes_vec_vec_[ " << i << " ]::  [";
            //     for (auto lane_num : all_lanes_vec_vec_[i]) {
            //         std::cout << " " << static_cast<int>(lane_num) << ",";
            //     }
            //     std::cout << "]" << std::endl;
            // }
            // 对于当前link所有的链接关系，找到对应的lane num的
            for (auto index : cur_link_staticmap_indices) {
                uint8_t init_lane_num =
                    map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum;

                cur_link_connectivity_link_id =
                    map_static_info_->LaneConnectivitys.PairConnectivity[index].ToLinkId.ToLink;
                message::map_map::s_LaneInfo_t lane_tmp{}; 
                if(!efm::MapCommonTool::GetInstance()->GetLaneInfo(link_info_temp, init_lane_num,lane_tmp)){
                    return false;
                }
                uint8_t this_lane_type = lane_tmp.LaneType.data;//如果
                // std::cout << "init_lane_num: " << static_cast<int>(init_lane_num) << std::endl;
                // std::cout << "search_link_lanes_num: " << static_cast<int>(search_link_lanes_num) << std::endl;
                // std::cout << "cur_link_connectivity_link_id: " << static_cast<int>(cur_link_connectivity_link_id)
                //           << std::endl;
                // std::cout << "map_route_list_->LinkIds.LinkIds[i + 1].LinkId: "
                //           << map_route_list_->LinkIds.LinkIds[i + 1].LinkId << std::endl;
                if (init_lane_num == search_link_lanes_num && (this_lane_type != 3 && this_lane_type != 9 && this_lane_type != 17)) {
                    if (i + 1 < routingpath_link_size &&
                        cur_link_connectivity_link_id == map_route_list_->LinkIds.LinkIds[i + 1].LinkId) {
                        // connect link id is on route
                        find_connect_lane = true;
                        uint8_t new_lane_num =
                            map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum;
                        // std::cout << "new_lane_num: " << static_cast<int>(new_lane_num) << std::endl;
                        if (all_lanes_vec_vec_[all_lanes_vec_end_index].size() <= valid_link_count + 1) {
                            all_lanes_vec_vec_[all_lanes_vec_end_index].push_back(new_lane_num);
                            // split node
                            LanesNode node_temp{};
                            node_temp.link_id = cur_link_connectivity_link_id;
                            node_temp.lane_num = new_lane_num;
                            lane_node_temp.push_back(node_temp);
                            // lane extra info
                            LaneExtraInfo_s lane_extra_info_temp{};
                            SaveLaneAllExtraInfo(cur_link_connectivity_link_id, new_lane_num, lane_extra_info_temp);
                            all_lanes_extra_vec_vec_[all_lanes_vec_end_index].push_back(lane_extra_info_temp);
                        } else {
                            // store a new lane // 保证 all_lanes_vec_vec_[0] 是1，1，1，1， all_lanes_vec_vec_有序排列
                            split_lane_number++;
                            std::vector<uint8_t> line_temp(all_lanes_vec_vec_[all_lanes_vec_end_index].begin(),
                                                           all_lanes_vec_vec_[all_lanes_vec_end_index].end());
                            // lane extra info
                            std::vector<LaneExtraInfo_s> extra_info_temp(
                                all_lanes_extra_vec_vec_[all_lanes_vec_end_index].begin(),
                                all_lanes_extra_vec_vec_[all_lanes_vec_end_index].end());
                            uint8_t old_lane_num = line_temp.back();
                            line_temp.back() = new_lane_num;
                            bool insert_f_temp = false;
                            // std::cout << __FILE__ << "," << __LINE__ << "," << "new_lane_num:" << (int)new_lane_num
                            //           << " ,old_lane_num " << (int)old_lane_num << std::endl;
                            if (new_lane_num < old_lane_num) {
                                all_lanes_vec_vec_.insert(all_lanes_vec_vec_.begin() + all_lanes_vec_end_index,
                                                          line_temp);
                                insert_f_temp = true;
                                // all_lanes_vec_end_index++;
                            } else {
                                all_lanes_vec_vec_.insert(all_lanes_vec_vec_.begin() + all_lanes_vec_end_index + 1,
                                                          line_temp);
                            }
                            // split node
                            LanesNode node_temp{};
                            node_temp.link_id = cur_link_connectivity_link_id;
                            node_temp.lane_num = new_lane_num;
                            lane_node_temp.push_back(node_temp);
                            SaveLaneAllExtraInfo(cur_link_connectivity_link_id, new_lane_num, extra_info_temp.back());
                            if (insert_f_temp == true) {
                                all_lanes_extra_vec_vec_.insert(
                                    all_lanes_extra_vec_vec_.begin() + all_lanes_vec_end_index, extra_info_temp);
                                    all_lanes_vec_end_index++;//都用好了，最后加加
                            } else {
                                all_lanes_extra_vec_vec_.insert(
                                    all_lanes_extra_vec_vec_.begin() + all_lanes_vec_end_index + 1, extra_info_temp);
                                    all_lanes_vec_end_index++;//都用好了，最后加加
                            }
                        }
                        // std::cout << __FILE__ << "," << __LINE__ << ","
                        //           << "sub: all_lanes_vec_end_index: " << all_lanes_vec_end_index << std::endl;
                    }
                    // cur_lane_connectivity_size += 1;
                }
            }
            all_lanes_vec_end_index++;
            if (split_lane_number > 1) {
                // std::cout << __FILE__ << "," << __LINE__ << ","
                //           << "split_lane_number > 1 " << std::endl;
                SplitNode split_node_temp{};
                split_node_temp.origin_node.link_id = rp_link_id;
                split_node_temp.origin_node.lane_num = search_link_lanes_num;
                split_node_temp.split_nodes = lane_node_temp;
                candidate_lanes_split_nodes_.emplace(rp_link_id, split_node_temp);
                find_split_f = true;
            }
        }
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "find_split_f: " << find_split_f << std::endl;
        if (find_split_f) {
            is_contain_split_vec_.push_back(true);
        } else {
            is_contain_split_vec_.push_back(false);
        }
        if (find_connect_lane) {
            link_id_vec_.push_back(map_route_list_->LinkIds.LinkIds[i + 1].LinkId);
        } else {
            // std::cout << "generate done!!!!!!!!!!!!!" << std::endl;
            break;
        }
        
        //new 1223 emergency lane
        uint32_t emergency_cur_link_connectivity_link_id = 0;
        bool emergency_find_connect_lane = false;
        bool emergency_find_split_f = false;
        int emergency_all_lanes_vec_end_index = 0;
        for(int k = 0; k < emergency_lane_end.size(); k++){
            uint8_t search_link_lanes_num = emergency_lane_end[k];
            // std::cout << __FILE__ << "," << __LINE__ << "," << "k: " << k
            //           << " , lane_end: " << static_cast<int>(search_link_lanes_num) << std::endl;
            // std::cout << __FILE__ << "," << __LINE__ << "," << "all_lanes_vec_end_index: " << emergency_all_lanes_vec_end_index
            //           << std::endl;
            // for (int i = 0; i < all_emergency_lanes_vec_vec_.size(); i++) {
            //     std::cout << "lane end begin internal all_lanes_vec_vec_[ " << i << " ]::  [";
            //     for (auto lane_num : all_emergency_lanes_vec_vec_[i]) {
            //         std::cout << " " << static_cast<int>(lane_num) << ",";
            //     }
            //     std::cout << "]" << std::endl;
            // }
            for (auto index : cur_link_staticmap_indices) {
                uint8_t init_lane_num =
                    map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum;

                emergency_cur_link_connectivity_link_id =
                    map_static_info_->LaneConnectivitys.PairConnectivity[index].ToLinkId.ToLink;
                message::map_map::s_LaneInfo_t lane_tmp{}; 
                if(!efm::MapCommonTool::GetInstance()->GetLaneInfo(link_info_temp, init_lane_num,lane_tmp)){
                    break;
                }
                uint8_t this_lane_type = lane_tmp.LaneType.data;//如果
                // std::cout << "init_lane_num: " << static_cast<int>(init_lane_num) << std::endl;
                // std::cout << "search_link_lanes_num: " << static_cast<int>(search_link_lanes_num) << std::endl;
                // std::cout << "emergency_cur_link_connectivity_link_id: " << static_cast<int>(emergency_cur_link_connectivity_link_id)
                //           << std::endl;
                // std::cout << "map_route_list_->LinkIds.LinkIds[i + 1].LinkId: "
                //           << map_route_list_->LinkIds.LinkIds[i + 1].LinkId << std::endl;
                if (init_lane_num == search_link_lanes_num && this_lane_type == 3) {
                    if (i + 1 < routingpath_link_size &&
                        emergency_cur_link_connectivity_link_id == map_route_list_->LinkIds.LinkIds[i + 1].LinkId) {
                        // connect link id is on route
                        emergency_find_connect_lane = true;
                        uint8_t new_lane_num =
                            map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum;
                        // std::cout << "new_lane_num: " << static_cast<int>(new_lane_num) << std::endl;
                        if (all_emergency_lanes_vec_vec_[emergency_all_lanes_vec_end_index].size() <= emergency_valid_link_count + 1) {
                            all_emergency_lanes_vec_vec_[emergency_all_lanes_vec_end_index].push_back(new_lane_num);
                            // lane extra info
                            // LaneExtraInfo_s lane_extra_info_temp{};
                            // SaveLaneAllExtraInfo(cur_link_connectivity_link_id, new_lane_num, lane_extra_info_temp);
                            // all_lanes_extra_vec_vec_[all_lanes_vec_end_index].push_back(lane_extra_info_temp);
                        } else {
                            // store a new lane // 保证 all_lanes_vec_vec_[0] 是1，1，1，1， all_lanes_vec_vec_有序排列
                            std::vector<uint8_t> line_temp(all_emergency_lanes_vec_vec_[emergency_all_lanes_vec_end_index].begin(),
                                                           all_emergency_lanes_vec_vec_[emergency_all_lanes_vec_end_index].end());
                            // lane extra info
                            // std::vector<LaneExtraInfo_s> extra_info_temp(
                            //     all_lanes_extra_vec_vec_[all_lanes_vec_end_index].begin(),
                            //     all_lanes_extra_vec_vec_[all_lanes_vec_end_index].end());
                            uint8_t old_lane_num = line_temp.back();
                            line_temp.back() = new_lane_num;
                            bool insert_f_temp = false;
                            // std::cout << __FILE__ << "," << __LINE__ << "," << "new_lane_num:" << (int)new_lane_num
                            //           << " ,old_lane_num " << (int)old_lane_num << std::endl;
                            if (new_lane_num < old_lane_num) {
                                all_emergency_lanes_vec_vec_.insert(all_emergency_lanes_vec_vec_.begin() + emergency_all_lanes_vec_end_index,
                                                          line_temp);
                                insert_f_temp = true;
                                // all_lanes_vec_end_index++;
                            } else {
                                all_emergency_lanes_vec_vec_.insert(all_emergency_lanes_vec_vec_.begin() + emergency_all_lanes_vec_end_index + 1,
                                                          line_temp);
                            }
                            // SaveLaneAllExtraInfo(cur_link_connectivity_link_id, new_lane_num, extra_info_temp.back());
                            // if (insert_f_temp == true) {
                            //     all_lanes_extra_vec_vec_.insert(
                            //         all_lanes_extra_vec_vec_.begin() + all_lanes_vec_end_index, extra_info_temp);
                            //         all_lanes_vec_end_index++;//都用好了，最后加加
                            // } else {
                            //     all_lanes_extra_vec_vec_.insert(
                            //         all_lanes_extra_vec_vec_.begin() + all_lanes_vec_end_index + 1, extra_info_temp);
                            //         all_lanes_vec_end_index++;//都用好了，最后加加
                            // }
                        }
                        // std::cout << __FILE__ << "," << __LINE__ << ","
                        //           << "sub: emergency_all_lanes_vec_end_index: " << emergency_all_lanes_vec_end_index << std::endl;
                    }
                    // cur_lane_connectivity_size += 1;
                }
            }
        }
        forward_distance = link_info_temp.EndOffset.EndOffset - map_position_->PathOffset;
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "loop end link_id_vec_.size(): " << link_id_vec_.size() << std::endl;
        // std::cout << " loop end link_id_vec_:: [ ";
        // for (int k = 0; k < link_id_vec_.size(); k++) {
        //     std::cout << " " << static_cast<int>(link_id_vec_[k]) << ",";
        // }
        // std::cout << " " << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "all_lanes_vec_vec_.back().size(): " << static_cast<int>(all_lanes_vec_vec_.back().size())
        //           << std::endl;
        // for (int i = 0; i < all_lanes_vec_vec_.size(); i++) {
        //     std::cout << "fffffff internal all_lanes_vec_vec_[ " << i << " ]::  [";
        //     for (auto lane_num : all_lanes_vec_vec_[i]) {
        //         std::cout << " " << static_cast<int>(lane_num) << ",";
        //     }
        //     std::cout << "]" << std::endl;
        // }
    }
    is_contain_split_vec_.push_back(false);  // last can't judge split

    // if (forward_distance < kForwardExploreDistanceThreshold) {
    //     uint64_t last_link_id = map_route_list_->LinkIds.LinkIds[map_route_list_->LinkIds.LinkIds.size() - 1];
    //     int last_link_index = link_id_index_lane_info_map_[last_link_id];
    // }
    return true;
}

bool CandidateLanesModel::GetAvailableLaneNum(uint32_t search_link_id, uint8_t& righest_available_lane_num,
                                              uint8_t& leftest_available_lane_num) {
    // std::cout << "GetRightestLaneNum start" << std::endl;
    // int cur_link_index = link_id_index_lane_info_map_->at(search_link_id);
    message::map_map::s_LinkInfo_t link_infos;
    if (!GetLinkInfos(search_link_id, link_infos)) {
        return false;
    }
    size_t cur_lane_size = link_infos.LaneInfos.LaneInfos.size();
    bool get_rightest_lane_f = false;
    righest_available_lane_num = 1;
    leftest_available_lane_num = 1;
    std::vector<uint8_t> lane_num_vec{};

    for (int j = 0; j < cur_lane_size; j++) {
        // get all avalible lane num
        if (link_infos.LaneInfos.LaneInfos[j].LaneType.data == 0 ||
            link_infos.LaneInfos.LaneInfos[j].LaneType.data == 1 ||
            link_infos.LaneInfos.LaneInfos[j].LaneType.data == 2 ||
            link_infos.LaneInfos.LaneInfos[j].LaneType.data == 4 ||
            link_infos.LaneInfos.LaneInfos[j].LaneType.data == 5 ||
            link_infos.LaneInfos.LaneInfos[j].LaneType.data == 6 ||
            link_infos.LaneInfos.LaneInfos[j].LaneType.data == 7 ||
            link_infos.LaneInfos.LaneInfos[j].LaneType.data == 8) {
            uint8_t lane_num = link_infos.LaneInfos.LaneInfos[j].LaneNum.LaneNum;
            lane_num_vec.push_back(lane_num);
        }
    }

    if (lane_num_vec.empty()) {
        return false;
    } else {
        auto min = std::min_element(lane_num_vec.begin(), lane_num_vec.end());
        righest_available_lane_num = *min;
        auto max = std::max_element(lane_num_vec.begin(), lane_num_vec.end());
        leftest_available_lane_num = *max;
    }

    // std::cout << "while_counter: " << while_counter << std::endl;
    return true;
}

std::vector<uint8_t> CandidateLanesModel::ref_route_path_lane_num() {
    if (ref_route_lane_prior_index_ >= 0 && ref_route_lane_prior_index_ < all_lanes_vec_vec_.size()){
        return all_lanes_vec_vec_[ref_route_lane_prior_index_];
    }

    if (ref_lane_prior_index_ >= 0 && ref_lane_prior_index_ < all_lanes_vec_vec_.size()){
        return all_lanes_vec_vec_[ref_lane_prior_index_];
    }

    if (ego_lane_prior_index_ >= 0 && ego_lane_prior_index_ < all_lanes_vec_vec_.size()){
        return all_lanes_vec_vec_[ego_lane_prior_index_];
    }
    
    return std::vector<uint8_t> {};
}

/**
 * @brief get a link's link infos
 *
 * @return true
 * @return false
 */
bool CandidateLanesModel::GetLinkInfos(uint32_t want_link_id, message::map_map::s_LinkInfo_t& link_infos) {
    if (link_id_index_lane_info_map_->find(want_link_id) != link_id_index_lane_info_map_->end()) {
        int link_index = link_id_index_lane_info_map_->at(want_link_id);
        link_infos = map_static_info_->LinkInfos.LinkInfos[link_index];
        return true;
    } else {
        return false;
    }
}

/**
 * @brief get ego left, right lanes
 *
 * @return true
 * @return false
 */
bool CandidateLanesModel::GetPriorCandidateLanes(uint8_t target_lane_num,
                                                 const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec,
                                                 int& prior_index_out) {
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "target_lane_num: " << static_cast<int>(target_lane_num) << std::endl;
    // step 1, get longest lane
    std::vector<uint32_t> link_id_vec = link_id_vec_;
    int longest_index = -1;
    int longest_lane_langth = 0;
    std::vector<std::vector<uint8_t>> lanes_temp{};
    std::vector<int> lane_index_vec{};  // for debug
    for (int i = 0; i < all_lanes_vec_vec.size(); i++) {
        if (all_lanes_vec_vec[i].front() == target_lane_num) {
            if (all_lanes_vec_vec[i].size() > longest_lane_langth) {
                longest_lane_langth = all_lanes_vec_vec[i].size();
                lanes_temp.clear();
                lane_index_vec.clear();
                lanes_temp.push_back(all_lanes_vec_vec[i]);
                lane_index_vec.push_back(i);
            } else if (all_lanes_vec_vec[i].size() == longest_lane_langth) {
                lanes_temp.push_back(all_lanes_vec_vec[i]);
                lane_index_vec.push_back(i);
            } else {
                // do nothing
            }
        }
    }
    if (lanes_temp.empty()) {
        return true;
    }
    // 比较路线平直性，采取s方向分段比较平直性。
    // 先找出两组不一致的地方，计算不一致的两段的heading变化值，取变化小者
    int prior_index = 0;
    // int prior_index_in_all_lanes = 0;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "lanes_temp.size(): " << lanes_temp.size() << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "lane_index_vec(): ";
    // for (auto index : lane_index_vec) {
    //     std::cout << index << ", ";
    // }
    // std::cout << " " << std::endl;

    if (lanes_temp.size() > 1) {
        for (int i = 1; i < lanes_temp.size(); i++) {
            auto diff_pair = std::mismatch(lanes_temp[i].begin(), lanes_temp[i].end(), lanes_temp[prior_index].begin());
            int idx = std::distance(lanes_temp[i].begin(), diff_pair.first);
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << "diff_index : " << idx << std::endl;
            // 计算两组数据yaw变化值
            double yaw_bias_1 = CalculatePathFluctuationValue(idx, lanes_temp[i], link_id_vec);
            double yaw_bias_2 = CalculatePathFluctuationValue(idx, lanes_temp[prior_index], link_id_vec);
            if (yaw_bias_1 < yaw_bias_2) {
                prior_index = i;
            }
        }
    }
    // std::cout << "prior_index_out: " << std::endl;
    prior_index_out = lane_index_vec[prior_index];
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "prior_index_out: " << prior_index_out << std::endl;

    return true;
}

double CandidateLanesModel::CalculatePathFluctuationValue(const int& diff_index,
                                                          const std::vector<uint8_t>& lane_num_vec,
                                                          const std::vector<uint32_t>& link_id_vec) {
    // 仅有一个link，不比较，默认0.0
    if (lane_num_vec.size() == 1) return 0.0;
    // 考察headings变化，曲率变化
    std::vector<double> average_curvature_vec;
    // lane id组合--->
    // 对应平均曲率headings组合，再根据曲率/heading的变化量，取均值
    for (int i = 0; i < 2; i++) {
        message::map_map::s_LinkInfo_t link_infos;
        std::vector<std::pair<double, double>> xy_points;
        std::vector<double> headings, accumulated_s, kappas, dkappas;
        //
        int index = diff_index - i;
        // int rp_index = id + cur_rp_index;
        int lane_num = lane_num_vec[index];
        uint32_t link_id = link_id_vec[index];
        // get link
        if (GetLinkInfos(link_id, link_infos)) {
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << "GetLinkInfos farlure!!!: " << link_id << std::endl;
            return DBL_MAX;
        }
        // 车道线索引号序列
        uint32_t line_index = 0;
        for (auto lane : link_infos.LaneInfos.LaneInfos) {
            if (lane.LaneNum.LaneNum == lane_num) {
                line_index = lane.Centeline.Centeline;
                break;
            }
        }
        if (line_index == 0) {
            // std::cout << "line not found" << std::endl;
            return DBL_MAX;
        }
        std::vector<message::map_map::s_GeometryPoint_t> geometry_points;
        if (GetLine(line_index, geometry_points)) {
            // std::cout << "line not found 222" << std::endl;
            return DBL_MAX;
        }
        EFMRefLinePoints ref_line_points{};
        CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(geometry_points, ref_line_points, map_position_);

        if (!CommonMathMethod::DiscretePointsMath::ComputePathProfile(ref_line_points, &headings, &accumulated_s,
                                                                      &kappas, &dkappas) ||
            headings.size() < 1) {
            // std::cout << "ComputePathProfile failed" << std::endl;
            return DBL_MAX;
        }

        // Keep for debugging
        // LOG(INFO) << "headings size = " << headings.size();
        // for (auto& heading : headings) {
        //   LOG(INFO) << " " << heading;
        // }

        // only select some points nearby, avoiding too long
        int size_max = 8;
        int cnt = (headings.size() < size_max) ? headings.size() : size_max;
        double average_h;
        if (i == 0) {
            average_h = std::accumulate(headings.begin(), headings.begin() + cnt, 0.0) / cnt;
        } else {
            average_h = std::accumulate(headings.rbegin(), headings.rbegin() + cnt, 0.0) / cnt;
        }

        // LOG(INFO) << "average: " << average_h;
        average_curvature_vec.emplace_back(average_h);
    }

    double cost = 0.0;
    for (int i = 1; i < average_curvature_vec.size(); i++) {
        cost += std::abs(average_curvature_vec[i] - average_curvature_vec[i - 1]);
    }

    return cost;
}

bool CandidateLanesModel::GetLine(const uint32_t line_index,
                                  std::vector<message::map_map::s_GeometryPoint_t>& geometry_points) {
    for (int i = 0; i < map_static_info_->LinearObjects.LinearObjects.size(); i++) {
        if (map_static_info_->LinearObjects.LinearObjects[i].IDLinearObject.IDLinearObject == line_index) {
            geometry_points = map_static_info_->LinearObjects.LinearObjects[i].GeometryPoints.GeometryPoints;
            return true;
        }
    }
    return false;
}

bool CandidateLanesModel::GetLine(const uint32_t line_index,
                                  std::vector<message::map_map::s_GeometryPoint_t>& geometry_points,uint8_t& mrk_type) {
    for (int i = 0; i < map_static_info_->LinearObjects.LinearObjects.size(); i++) {
        if (map_static_info_->LinearObjects.LinearObjects[i].IDLinearObject.IDLinearObject == line_index) {
            geometry_points = map_static_info_->LinearObjects.LinearObjects[i].GeometryPoints.GeometryPoints;
            mrk_type = map_static_info_->LinearObjects.LinearObjects[i].LinearObjectMarking.data;
            return true;
        }
    }
    return false;
}

bool CandidateLanesModel::GetLaneDist(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec,
                                      std::shared_ptr<const message::map_position::s_Position_t> map_position,
                                      const std::vector<uint32_t>& link_id_vec, int index, double& length) {
    uint32_t ego_offset = map_position->PathOffset;
    if (!(index >= 0 && index < all_lanes_vec_vec.size())) {
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "GetLaneDist error!!!!!!" << std::endl;
        return false;
    }
    size_t lane_vec_size = all_lanes_vec_vec[index].size();
    if (!(lane_vec_size > 0 && lane_vec_size <= link_id_vec.size())) {
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "lane_vec_size error!!!!!!" << std::endl;
        return false;
    }
    int end_index = lane_vec_size - 1;
    uint32_t end_link = link_id_vec[end_index];
    message::map_map::s_LinkInfo_t link_infos;
    if (!GetLinkInfos(end_link, link_infos)) {
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "GetLinkInfos error!!!!!!  " << end_link << std::endl;
        return false;
    }
    length = (static_cast<int>(link_infos.EndOffset.EndOffset) - static_cast<int>(ego_offset)) / 100.0;

    return true;
}

bool CandidateLanesModel::GetLaneLength(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec, int index,
                                        size_t& length) {
    if (!(index >= 0 && index < all_lanes_vec_vec.size())) {
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "GetLaneLength error222222!!!!!!" << std::endl;
        return false;
    }
    length = all_lanes_vec_vec[index].size();
    return true;
}

bool CandidateLanesModel::DeleteEmergencyLane(std::vector<std::vector<uint8_t>> all_lanes_vec_vec,
                                              std::vector<uint32_t> link_id_vec) {
    // std::cout << __FILE__ << "," << __LINE__ << ",  "
    //           << "DeleteEmergencyLane start" << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ",  "
    //           << "all_lanes_vec_vec.size()" << all_lanes_vec_vec.size() << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ",  "
    //           << "link_id_vec.size()" << link_id_vec.size() << std::endl;
    std::vector<int> indexes{};
    for (int i = 0; i < all_lanes_vec_vec.size(); i++) {
        std::vector<uint8_t> lane_vec = all_lanes_vec_vec[i];
        // std::cout << __FILE__ << "," << __LINE__ << ",  "
        //           << "i: " << i << std::endl;
        if (link_id_vec.size() < lane_vec.size()) {
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << "lane_vec_size error!!!!!!  link_id_vec.size(): " << link_id_vec.size() << "lane_vec.size()
            //           "
            //           << lane_vec.size() << std::endl;
            return true;
        }
        for (int k = 0; k < lane_vec.size(); k++) {
            uint32_t link_id = link_id_vec[k];
            uint8_t lane_num = lane_vec[k];
            bool find_emerg_f = false;
            message::map_map::s_LinkInfo_t link_infos;
            if (!GetLinkInfos(link_id, link_infos)) {
                //  std::cout << __FILE__ << "," << __LINE__ << ","
                //            << "GetLinkInfos error!!!!!!  " << link_id << std::endl;
                break;
            }
            uint8_t lane_type = 0;
            for (auto lane : link_infos.LaneInfos.LaneInfos) {
                if (lane.LaneNum.LaneNum == lane_num) {
                    lane_type = lane.LaneType.data;
                    // std::cout << __FILE__ << "," << __LINE__ << ","
                    //           << "lane_type!!!!!!  " << static_cast<int>(lane_type) << std::endl;
                    if (lane_type == 3 || lane_type == 17 || lane_type == 9) {
                        indexes.push_back(i);
                        find_emerg_f = true;
                        //  std::cout << __FILE__ << "," << __LINE__ << ","
                        //            << "i need to be delete!!!!!!  " << i << std::endl;
                        break;
                    }
                }
            }
            if (find_emerg_f) {
                break;
            }
        }
    }
    // std::cout << __FILE__ << "," << __LINE__ << ",  "
    //           << "indexes.size() " << indexes.size() << std::endl;

    std::vector<std::vector<uint8_t>> lanes_temp = all_lanes_vec_vec_;
    std::vector<std::vector<LaneExtraInfo_s>> lanes_extra_temp = all_lanes_extra_vec_vec_;
    all_lanes_vec_vec_.clear();
    all_lanes_extra_vec_vec_.clear();
    if (lanes_temp.size() == lanes_extra_temp.size()) {
        for (int i = 0; i < lanes_temp.size(); i++) {
            auto it = std::find(indexes.begin(), indexes.end(), i);
            if (it == indexes.end()) {
                // not in indexes means , is not emergency
                all_lanes_vec_vec_.push_back(lanes_temp[i]);
                all_lanes_extra_vec_vec_.push_back(lanes_extra_temp[i]);
            }
        }
    }

    return true;
}

bool CandidateLanesModel::GetPriorCandidateLanesV2(
    const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec,
    std::shared_ptr<const message::map_position::s_Position_t> map_position, const std::vector<uint32_t>& link_id_vec,
    const std::vector<bool> is_contain_split_vec, const std::map<uint32_t, SplitNode> candidate_lanes_split_nodes,
    int& left_prior_index_out, int& ego_prior_index_out, int& right_prior_index_out) {
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "GetPriorCandidateLanesV2 start " << std::endl;
    double p_SplitRangeThrd = 100;
    // cal value to decide whether to take two split ego lane as one ego and one left, or one ego one right
    // 1. judge split infront ego lane, then find ego and neighbor lane
    uint8_t ego_lane_num = std::max(map_position->LaneId, static_cast<uint8_t>(0));
    for (int i = 0; i < is_contain_split_vec.size() - 1; i++) {
        if (is_contain_split_vec[i]) {
            uint32_t link_id = link_id_vec[i];
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << "is_contain_split_vec:  true: " << i << ", link_id: " << link_id << std::endl;
            if (candidate_lanes_split_nodes.find(link_id) != candidate_lanes_split_nodes.end()) {
                uint8_t origin_lane_num = candidate_lanes_split_nodes.at(link_id).origin_node.lane_num;
                std::vector<std::vector<std::vector<uint8_t>>::const_iterator>
                    split_lane_iter_vec{};  // store same first lane num lanes
                // find out all split lanes
                for (auto split_node : candidate_lanes_split_nodes.at(link_id).split_nodes) {
                    // [i+1] is split link, [i] is origin link
                    auto iter = std::find_if(all_lanes_vec_vec.begin(), all_lanes_vec_vec.end(),
                                             [&](const std::vector<uint8_t>& it) {
                                                 if (it.size() > (i + 1)) {
                                                     if (it[i] == origin_lane_num && it[i + 1] == split_node.lane_num) {
                                                         return true;
                                                     }
                                                 }
                                                 return false;
                                             });
                    if (iter != all_lanes_vec_vec.end() && (*iter).size() != 0) {
                        // lane's ego position 's lane num == ego_lane_num, means this lane is ego lane
                        if ((*iter).front() == ego_lane_num) {
                            split_lane_iter_vec.push_back(iter);
                        }
                    }
                }
                // std::cout << __FILE__ << "," << __LINE__ << ","
                //           << "split_lane_iter_vec.size() " << split_lane_iter_vec.size() << std::endl;
                // std::cout << __FILE__ << "," << __LINE__ << ","
                //           << "ego split lane index: ";
                // for (auto split : split_lane_iter_vec) {
                //     std::cout << " ," << std::distance(all_lanes_vec_vec.begin(), split);
                // }
                // std::cout << std::endl;
                if (!split_lane_iter_vec.empty()) {
                    // split is in 30m range, split
                    message::map_map::s_LinkInfo_t link_infos;  // split origin link
                    if (!GetLinkInfos(link_id, link_infos)) {
                        // std::cout << __FILE__ << "," << __LINE__ << ","
                        //           << "can't find link id: " << link_id << std::endl;
                        break;
                    }
                    double dist = (static_cast<double>(link_infos.EndOffset.EndOffset) -
                                   static_cast<double>(map_position->PathOffset)) /
                                  100.0;
                    // std::cout << __FILE__ << "," << __LINE__ << ","
                    //           << "split_dist " << dist << std::endl;
                    uint8_t ego_lane_num_in_split = 0;
                    std::vector<uint8_t> neighbor_lane_num_in_split{};
                    std::vector<int> neighbor_lane_index_vec{};
                    if (dist >= 0 && dist <= p_SplitRangeThrd) {
                        uint32_t split_link_id = link_id_vec[i + 1];
                        message::map_map::s_LinkInfo_t link_infos_split;  // split link
                        if (!GetLinkInfos(split_link_id, link_infos_split)) {
                            // std::cout << __FILE__ << "," << __LINE__ << ","
                            //           << "can't find link id: " << link_id << std::endl;
                            break;
                        }
                        for (auto lane_iter : split_lane_iter_vec) {
                            uint8_t lane_num = (*lane_iter)[i + 1];
                            // std::cout << __FILE__ << "," << __LINE__ << ","
                            //           << "split lane_num " << static_cast<int>(lane_num) << std::endl;
                            auto iter = std::find_if(link_infos_split.LaneInfos.LaneInfos.begin(),
                                                     link_infos_split.LaneInfos.LaneInfos.end(),
                                                     [&](const message::map_map::s_LaneInfo_t& it) {
                                                         return it.LaneNum.LaneNum == lane_num;
                                                     });
                            if (iter != link_infos_split.LaneInfos.LaneInfos.end()) {
                                if ((*iter).Transit.data == 1) {
                                    // transit continue
                                    ego_lane_num_in_split = lane_num;
                                    ego_prior_index_out = std::distance(all_lanes_vec_vec.begin(), lane_iter);
                                } else if ((*iter).Transit.data == 3) {
                                    neighbor_lane_num_in_split.push_back(lane_num);
                                    neighbor_lane_index_vec.push_back(
                                        std::distance(all_lanes_vec_vec.begin(), lane_iter));
                                }
                            }
                        }
                        // std::cout << __FILE__ << "," << __LINE__ << ","
                        //           << "ego_lane_num_in_split " << static_cast<int>(ego_lane_num_in_split) <<
                        //           std::endl;
                        // std::cout << __FILE__ << "," << __LINE__ << ","
                        //           << "ego_prior_index_out " << ego_prior_index_out << std::endl;
                        // std::cout << __FILE__ << "," << __LINE__ << ","
                        //           << "neighbor_lane_index_vec.size() " << neighbor_lane_index_vec.size() <<
                        //           std::endl;
                        // get all split lane , one ego lane, and at least one split neighbor lane
                        if (ego_lane_num_in_split > 0 && !neighbor_lane_index_vec.empty()) {
                            for (int k = 0; k < neighbor_lane_index_vec.size() && k < neighbor_lane_num_in_split.size();
                                 k++) {
                                if (neighbor_lane_num_in_split[k] == ego_lane_num_in_split + 1) {
                                    left_prior_index_out = neighbor_lane_index_vec[k];
                                } else if (neighbor_lane_num_in_split[k] == ego_lane_num_in_split - 1) {
                                    right_prior_index_out = neighbor_lane_index_vec[k];
                                }
                            }
                        }
                    }
                    break;
                }
            }
        }
    }
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "left_prior_index_out " << left_prior_index_out << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "right_prior_index_out " << right_prior_index_out << std::endl;
    // if  there's not found
    if (ego_prior_index_out < 0) {
        GetPriorCandidateLanes(ego_lane_num, all_lanes_vec_vec, ego_prior_index_out);
    }
    if (left_prior_index_out < 0) {
        uint8_t left_lane_num = std::max(ego_lane_num + 1, 0);
        GetPriorCandidateLanes(left_lane_num, all_lanes_vec_vec, left_prior_index_out);
    }
    if (right_prior_index_out < 0) {
        uint8_t right_lane_num = std::max(ego_lane_num - 1, 0);
        GetPriorCandidateLanes(right_lane_num, all_lanes_vec_vec, right_prior_index_out);
    }

    return true;
}

void CandidateLanesModel::GetBackConnectInfo(
    const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec, const std::vector<uint32_t>& link_id_vec) {
    // back_ego_link_id_vec.clear();
    // back_left_link_id_vec.clear();
    // back_right_link_id_vec.clear();
    // back_ego_lane_num_vec.clear();
    // back_left_lane_num_vec.clear();
    // back_right_lane_num_vec.clear();
#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << "ego_get_back####################  " << std::endl;
#endif
    GetSingleLaneBackConnectInfo(all_lanes_vec_vec, link_id_vec, ego_lane_prior_index_, 0,back_ego_link_id_vec_,
                                 back_ego_lane_num_vec_);
#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << "left_get_back####################  " << std::endl;
#endif
    GetSingleLaneBackConnectInfo(all_lanes_vec_vec, link_id_vec, left_lane_prior_index_, 1,back_left_link_id_vec_,
                                 back_left_lane_num_vec_);
#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << "left_get_back####################  " << std::endl;
#endif
    GetSingleLaneBackConnectInfo(all_lanes_vec_vec, link_id_vec, left_left_lane_prior_index_, 1,back_left_left_link_id_vec_,
                                 back_left_left_lane_num_vec_);
#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << "right_get_back####################  " << std::endl;
#endif
    GetSingleLaneBackConnectInfo(all_lanes_vec_vec, link_id_vec, right_lane_prior_index_, 2,back_right_link_id_vec_,
                                 back_right_lane_num_vec_);
#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << "left_get_back####################  " << std::endl;
#endif
    GetSingleLaneBackConnectInfo(all_lanes_vec_vec, link_id_vec, right_right_lane_prior_index_, 2,back_right_right_link_id_vec_,
                                 back_right_right_lane_num_vec_);

#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << "back_ego_link_id_vec.size()  " << back_ego_link_id_vec_.size()
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "back_left_link_id_vec.size()  " << back_left_link_id_vec_.size()
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "back_right_link_id_vec.size()  "
              << back_right_link_id_vec_.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "back_ego_lane_num_vec.size()  " << back_ego_lane_num_vec_.size()
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "back_left_lane_num_vec.size()  "
              << back_left_lane_num_vec_.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "back_right_lane_num_vec.size()  "
              << back_right_lane_num_vec_.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " back_ego_link_id_vec: ";
    for (auto link : back_ego_link_id_vec_) {
        std::cout << " , " << link;
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << " back_left_link_id_vec: ";
    for (auto link : back_left_link_id_vec_) {
        std::cout << " , " << link;
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << " back_right_link_id_vec: ";
    for (auto link : back_right_link_id_vec_) {
        std::cout << " , " << link;
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << " back_ego_lane_num_vec: ";
    for (auto lane_num : back_ego_lane_num_vec_) {
        std::cout << " , " << static_cast<int>(lane_num);
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << " back_left_lane_num_vec: ";
    for (auto lane_num : back_left_lane_num_vec_) {
        std::cout << " , " << static_cast<int>(lane_num);
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << " back_right_lane_num_vec: ";
    for (auto lane_num : back_right_lane_num_vec_) {
        std::cout << " , " << static_cast<int>(lane_num);
    }
    std::cout << std::endl;
#endif
}

void CandidateLanesModel::GetBackConnectInfoEmergency(
    const std::vector<std::vector<uint8_t>>& all_emergency_lanes_vec_vec, const std::vector<uint32_t>& link_id_vec, uint8_t ego_lane_num) {
    if(ego_lane_num ==0 || all_emergency_lanes_vec_vec.size()==0){
        return;
    }

    emergency_left_index_ = -1;
    emergency_right_index_ = -1;
    for(size_t i=0; i<all_emergency_lanes_vec_vec.size();i++){
        if(all_emergency_lanes_vec_vec[i].size()>0){
            if(ego_lane_num == (all_emergency_lanes_vec_vec[i].front() +1)){
                emergency_right_index_ = i;
            }
            if(ego_lane_num == (all_emergency_lanes_vec_vec[i].front() -1)){
                emergency_left_index_ = i;
            }
        }
    }
    
    if(emergency_left_index_ <0 && emergency_right_index_<0){
        return;
    }
    GetSingleLaneBackConnectInfo(all_emergency_lanes_vec_vec, link_id_vec, emergency_left_index_, 1,back_emergency_left_link_id_vec_,
                                 back_emergency_left_lane_num_vec_);   
    GetSingleLaneBackConnectInfo(all_emergency_lanes_vec_vec, link_id_vec, emergency_right_index_, 2,back_emergency_right_link_id_vec_,
                                 back_emergency_right_lane_num_vec_);  
#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << "back_emergency_left_link_id_vec_.size()  " << back_emergency_left_link_id_vec_.size()
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "back_emergency_right_link_id_vec_.size()  " << back_emergency_right_link_id_vec_.size()
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " back_emergency_left_link_id_vec_: ";
    for (auto link : back_emergency_left_link_id_vec_) {
        std::cout << " , " << link;
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << " back_emergency_right_link_id_vec_: ";
    for (auto link : back_emergency_right_link_id_vec_) {
        std::cout << " , " << link;
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << "back_emergency_left_lane_num_vec_.size()  " << back_emergency_left_lane_num_vec_.size()
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "back_emergency_right_lane_num_vec_.size()  " << back_emergency_right_lane_num_vec_.size()
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << " back_emergency_left_lane_num_vec_: ";
    for (auto link : back_emergency_left_lane_num_vec_) {
        std::cout << " , " << (int)link;
    }
    std::cout << std::endl;

    std::cout << __FILE__ << "," << __LINE__ << "," << " back_emergency_right_lane_num_vec_: ";
    for (auto link : back_emergency_right_lane_num_vec_) {
        std::cout << " , " << (int)link;
    }
    std::cout << std::endl;
#endif
}

void CandidateLanesModel::GetLaneWidthOne(
    const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec, const std::vector<uint32_t>& link_id_vec,
    int lane_prior_index, std::vector<uint16_t>& lane_width_min_vec, std::vector<uint16_t>& lane_width_max_vec) {

    lane_width_min_vec.clear();
    lane_width_max_vec.clear();
    if (lane_prior_index >= 0 && lane_prior_index < all_lanes_vec_vec.size()) {
        std::vector<uint8_t> path_lane_num = all_lanes_vec_vec[lane_prior_index];
        if (path_lane_num.size() <= link_id_vec.size()) {
            for (int i = 0; i < path_lane_num.size(); i++) {
                uint8_t lane_num = path_lane_num[i];
                uint32_t link_id = link_id_vec[i];
                auto iter = std::find_if(
                    map_static_info_->LaneWidths.LaneWidths.begin(), map_static_info_->LaneWidths.LaneWidths.end(),
                    [&](const message::map_map::s_LaneWidth_t& it) {
                        return (it.LaneNum.LaneNum == lane_num && it.InstanceId.InstanceId == link_id);
                    });
                if (iter != map_static_info_->LaneWidths.LaneWidths.end()) {
                    lane_width_min_vec.push_back((*iter).MinWidth.MinWidth);
                    lane_width_max_vec.push_back((*iter).MaxWidth.MaxWidth);
                } else {
                    break;
                }
            }
        } else {
            // error
        }
    }
}

void CandidateLanesModel::GetLaneWidth(
    const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec, const std::vector<uint32_t>& link_id_vec) {
    //ego
    GetLaneWidthOne(all_lanes_vec_vec, link_id_vec, ego_lane_prior_index_, ego_lane_width_min_vec_, ego_lane_width_max_vec_);
    //left
    GetLaneWidthOne(all_lanes_vec_vec, link_id_vec, left_lane_prior_index_, left_lane_width_min_vec_, left_lane_width_max_vec_);
    //right
    GetLaneWidthOne(all_lanes_vec_vec, link_id_vec, right_lane_prior_index_, right_lane_width_min_vec_, right_lane_width_max_vec_);
    //left_left
    GetLaneWidthOne(all_lanes_vec_vec, link_id_vec, left_left_lane_prior_index_, left_left_lane_width_min_vec_, left_left_lane_width_max_vec_);
    //right_right
    GetLaneWidthOne(all_lanes_vec_vec, link_id_vec, right_right_lane_prior_index_, right_right_lane_width_min_vec_, right_right_lane_width_max_vec_);

}

void CandidateLanesModel::GetLinkLength(const std::vector<uint32_t>& link_id_vec,
                                        std::vector<double>& link_length_vec) {
    link_length_vec.clear();
    int jj = 0;
    for (auto link : link_id_vec) {
        message::map_map::s_LinkInfo_t link_infos;
        if (!GetLinkInfos(link, link_infos)) {
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << "can't find link " << link;
        }
        if (jj == 0) {
            link_length_vec.push_back(
                (static_cast<double>(link_infos.EndOffset.EndOffset) - static_cast<double>(map_position_->PathOffset)) /
                100.0);
        } else {
            link_length_vec.push_back((static_cast<double>(link_infos.EndOffset.EndOffset) -
                                       static_cast<double>(link_infos.PathOffset.PathOffset)) /
                                      100.0);
        }
        jj++;
    }
}

// TODO all_lanes_vec_vec 入参可以删掉，后面没用到
void CandidateLanesModel::GetDrvLaneSize(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec,
                                         uint8_t& driveable_lane_size) {
    driveable_lane_size = 0;
    uint32_t ego_link_id = map_position_->LinkId;
    message::map_map::s_LinkInfo_t link_infos;
    if (!GetLinkInfos(ego_link_id, link_infos)) {
        //  std::cout << __FILE__ << "," << __LINE__ << ","
        //            << "GetLinkInfos error!!!!!!  " << link_id << std::endl;
        return;
    }
    for (auto lane : link_infos.LaneInfos.LaneInfos) {
        if (lane.LaneType.data != 3 && lane.LaneType.data != 17) {
            driveable_lane_size++;
        }
    }
}

bool CandidateLanesModel::GetLineType(const message::map_map::s_LaneInfo_t& lane_info,
                                   bool is_left, uint8_t& line_type) {
    uint32_t line_id = 0;
    if (is_left) {
        line_id = lane_info.LBound.LBound;
    } else {
        line_id = lane_info.RBound.RBound;
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

bool CandidateLanesModel::IsVirtually(const message::map_map::s_LaneInfo_t& lane_info_raw,
                                      const message::map_map::s_LaneInfo_t& lane_info_side,
                                      bool is_left){
    uint8_t cur_line_type = 0;
    uint8_t side_line_type = 0;
    if (is_left){
        GetLineType(lane_info_raw, true, cur_line_type);
        GetLineType(lane_info_side, false, side_line_type);
    }else{
        GetLineType(lane_info_raw, false, cur_line_type);
        GetLineType(lane_info_side, true, side_line_type);        
    }

    if (8 == cur_line_type && 8 == side_line_type){
        return true;
    }
    
    
    return false;
}

bool CandidateLanesModel::IsMergeLane(uint8_t cur_lane_id, uint8_t side_lane_id, uint32_t link_id,
                                      uint32_t& next_link_id, uint8_t& next_lane_id){
    next_link_id = 0;
    next_lane_id = 0;
    if (link_id_index_lane_connect_map_->find(link_id) != link_id_index_lane_connect_map_->end()) {
        uint32_t cur_next_link_id = 0;
        uint8_t cur_next_lane_id = 0;
        uint32_t side_next_link_id = 0;
        uint8_t side_next_lane_id = 0;
        for (auto idx : link_id_index_lane_connect_map_->at(link_id)) {
            if (cur_lane_id == map_static_info_->LaneConnectivitys.PairConnectivity[idx].InitLaneNum.InitLaneNum &&
                link_id == map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId) {
                cur_next_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].ToLinkId.ToLink;
                cur_next_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum;
            }else if (side_lane_id == map_static_info_->LaneConnectivitys.PairConnectivity[idx].InitLaneNum.InitLaneNum &&
                      link_id == map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId) {
                side_next_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].ToLinkId.ToLink;
                side_next_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum;
            }
        }

        if (cur_next_link_id == side_next_link_id && cur_next_lane_id == side_next_lane_id){
            next_link_id = cur_next_link_id;
            next_lane_id = cur_next_lane_id;
            return true;
        }
        
    }

    return false;
}

bool CandidateLanesModel::IsSplitLane(uint8_t cur_lane_id, uint8_t side_lane_id, uint32_t link_id){
    
    if (to_link_id_index_lane_connect_map_->find(link_id) != to_link_id_index_lane_connect_map_->end()) {
        uint32_t cur_back_link_id = 0;
        uint8_t cur_back_lane_id = 0;
        uint32_t side_back_link_id = 0;
        uint8_t side_back_lane_id = 0;
        for (auto idx : to_link_id_index_lane_connect_map_->at(link_id)) {
            if (cur_lane_id == map_static_info_->LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum &&
                link_id == map_static_info_->LaneConnectivitys.PairConnectivity[idx].ToLinkId.ToLink) {
                cur_back_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId;
                cur_back_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].InitLaneNum.InitLaneNum;
            }else if (side_lane_id == map_static_info_->LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum &&
                      link_id == map_static_info_->LaneConnectivitys.PairConnectivity[idx].ToLinkId.ToLink) {
                side_back_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId;
                side_back_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].InitLaneNum.InitLaneNum;
            }
        }

        if (cur_back_link_id == side_back_link_id && cur_back_lane_id == side_back_lane_id){
            return true;
        }
        
    }

    return false;
}

bool CandidateLanesModel::SaveLaneExtraInfoMergeSplit(const message::map_map::s_LaneInfo_t& lane_info_raw,
                                                      const message::map_map::s_LaneInfo_t& lane_info_side,
                                                      uint32_t link_id,
                                                      LaneExtraInfo_s& lane_extra_info,
                                                      bool is_left) {
    if (1 == lane_info_raw.Transit.data && 1 == lane_info_side.Transit.data){
        return true;
    }

    uint32_t next_link_id = 0;
    uint8_t next_lane_id = 0;
    if (IsMergeLane(lane_info_raw.LaneNum.LaneNum, lane_info_side.LaneNum.LaneNum, link_id, next_link_id, next_lane_id)){
        //std::cout << __FILE__ << "," << __LINE__ << "," << " link_id: " << link_id << std::endl;
        int32_t merge_start_offset = 0;
        int32_t merge_end_offset = 0;
        lane_extra_info.is_road_merge= false;
        if(IsRoadMerge(link_id,lane_info_raw.LaneNum.LaneNum, lane_info_side.LaneNum.LaneNum,
                                         map_position_->PathOffset,map_position_->PathId,merge_start_offset,merge_end_offset)){
            lane_extra_info.is_road_merge = true;
            lane_extra_info.road_merge_s_e_offset = std::make_pair(merge_start_offset,merge_end_offset);
        }

        if(IsVirtually(lane_info_raw, lane_info_side, is_left)){   
            uint8_t ego_lane_num = lane_info_raw.LaneNum.LaneNum;
            uint8_t side_lane_num = lane_info_side.LaneNum.LaneNum;
            //std::cout << __FILE__ << "," << __LINE__ << "," << " link_id: " << link_id << std::endl;   
            // if(std::find(merge_back_links_id_.begin(), merge_back_links_id_.end(), link_id) != merge_back_links_id_.end()){
            GetMergeBackLinkLane(link_id, ego_lane_num, side_lane_num);//改merge绑路用
            // }
            uint8_t raw_merge = 3;//continue:1;merge:2;middle:3
            uint8_t side_merge = 3;//continue:1;merge:2;middle:3
            //(lane_info_raw, lane_info_side, link_id, next_link_id, next_lane_id, raw_merge, side_merge)
            //ModifyMergeAttribute(lane_info_raw, lane_info_side,link_id,  next_link_id, next_lane_id, raw_merge, side_merge);
            bool link_is_in_toll_f = false; 
            for(auto id: toll_link_id_set_){
                if(id == link_id){
                    link_is_in_toll_f = true;
                    break;
                }    
            }
            if((GetModifyedLaneAttribute(merge_lind_id_lane_id_map, merge_deque, lane_info_raw.LaneNum.LaneNum,link_id,raw_merge) 
               && GetModifyedLaneAttribute(merge_lind_id_lane_id_map, merge_deque, lane_info_side.LaneNum.LaneNum,link_id,side_merge)) 
               || link_is_in_toll_f){
                 //use stored result
            }else{
                if(p_use_efm_log){
                    for(auto id: merge_attrbute_link_log){
                        if(id == link_id){
                            raw_merge = lane_info_raw.Transit.data;
                            side_merge = lane_info_side.Transit.data;
                        }else{
                            ModifyMergeAttributeBySideLine(lane_info_raw, lane_info_side,link_id, next_link_id, next_lane_id,raw_merge, side_merge);
                            StoreModifyedLaneAttribute(merge_lind_id_lane_id_map, merge_deque, raw_merge, lane_info_raw.LaneNum.LaneNum, link_id);
                            StoreModifyedLaneAttribute(merge_lind_id_lane_id_map, merge_deque, side_merge, lane_info_side.LaneNum.LaneNum , link_id);
                        }
                    }                    
                }else{
                    ModifyMergeAttributeBySideLine(lane_info_raw, lane_info_side,link_id, next_link_id, next_lane_id,raw_merge, side_merge);
                    StoreModifyedLaneAttribute(merge_lind_id_lane_id_map, merge_deque, raw_merge, lane_info_raw.LaneNum.LaneNum, link_id);
                    StoreModifyedLaneAttribute(merge_lind_id_lane_id_map, merge_deque, side_merge, lane_info_side.LaneNum.LaneNum , link_id);                    
                }

            } 
#ifdef MODIFY_MERGE
            std::cout << __FILE__ << "," << __LINE__ << "," << " link_id: " << link_id << std::endl;
            std::cout << __FILE__ << "," << __LINE__ << "," << " lane_info_raw.LaneNum.LaneNum: " << (int)lane_info_raw.LaneNum.LaneNum << std::endl;
            std::cout<<"raw_merge: "<<(int)raw_merge<<std::endl; 
            std::cout << __FILE__ << "," << __LINE__ << "," << " lane_info_side.LaneNum.LaneNum: " << (int)lane_info_side.LaneNum.LaneNum << std::endl;
            std::cout<<"side_merge: "<<(int)side_merge<<std::endl; 
#endif
            if (1 == raw_merge && 2 == side_merge){
                if (is_left){
                    lane_extra_info.merge_value = EFM_MergeType_FROM_LEFT;
                }else{
                    lane_extra_info.merge_value = EFM_MergeType_FROM_RIGHT;              
                }
            }else if (2 == raw_merge && 1 == side_merge){
                if (is_left){
                    lane_extra_info.merge_value = EFM_MergeType_TO_LEFT;
                }else{
                    lane_extra_info.merge_value = EFM_MergeType_TO_RIGHT;              
                }
            }else{
                if (is_left){
                    lane_extra_info.merge_value = EFM_MergeType_LEFT_TO_MIDDLE;
                }else{
                    lane_extra_info.merge_value = EFM_MergeType_RIGHT_TO_MIDDLE;
                }
            }
        }else{
            //std::cout << __FILE__ << "," << __LINE__ << "," << " link_id: " << link_id << std::endl;
            if (is_left){
                if (1 == lane_info_raw.Transit.data || 3 == lane_info_raw.Transit.data){
                    lane_extra_info.merge_value = EFM_MergeType_FROM_LEFT;
                }else{
                    lane_extra_info.merge_value = EFM_MergeType_TO_LEFT;
                }
            }else{
                if (1 == lane_info_raw.Transit.data || 3 == lane_info_raw.Transit.data){
                    lane_extra_info.merge_value = EFM_MergeType_FROM_RIGHT;
                }else{
                    lane_extra_info.merge_value = EFM_MergeType_TO_RIGHT;
                }                
            }
        }
    }else if (IsSplitLane(lane_info_raw.LaneNum.LaneNum, lane_info_side.LaneNum.LaneNum, link_id)){
        if (IsVirtually(lane_info_raw, lane_info_side, is_left)){
            uint8_t raw_split = lane_info_raw.Transit.data;//continue:1;split:3
            uint8_t side_split = lane_info_side.Transit.data;//continue:1;split:3
            if(GetModifyedLaneAttribute(split_lind_id_lane_id_map, split_deque, lane_info_raw.LaneNum.LaneNum,link_id,raw_split) 
               && GetModifyedLaneAttribute(split_lind_id_lane_id_map, split_deque, lane_info_side.LaneNum.LaneNum,link_id,side_split)){
                 //use stored result
                //  std::cout << __FILE__ << "," << __LINE__ << "," << " use stored result split " << std::endl;
            }else{
                ModifySplitAttribute(lane_info_raw, lane_info_side,link_id,raw_split, side_split);
                StoreModifyedLaneAttribute(split_lind_id_lane_id_map, split_deque, raw_split, lane_info_raw.LaneNum.LaneNum, link_id);
                StoreModifyedLaneAttribute(split_lind_id_lane_id_map, split_deque, side_split, lane_info_side.LaneNum.LaneNum , link_id);
            }
            
            // std::cout << __FILE__ << "," << __LINE__ << "," << " link_id: " << link_id << std::endl;
            // std::cout << __FILE__ << "," << __LINE__ << "," << " lane_info_raw.LaneNum.LaneNum: " << (int)lane_info_raw.LaneNum.LaneNum << std::endl;
            // std::cout<<"raw_split: "<<(int)raw_split<<std::endl; 
            // std::cout << __FILE__ << "," << __LINE__ << "," << " lane_info_side.LaneNum.LaneNum: " << (int)lane_info_side.LaneNum.LaneNum << std::endl;
            // std::cout<<"side_split: "<<(int)side_split<<std::endl;             
            if (is_left){
                if (1 == raw_split || 2 == raw_split || 99 == raw_split){
                    lane_extra_info.split_value = EFM_SplitType_CONTIUE_FROM_LEFT;
                }else{
                    lane_extra_info.split_value = EFM_SplitType_SPLIT_FROM_LEFT;
                }
            }else{
                if (1 == raw_split || 2 == raw_split || 99 == raw_split){
                    lane_extra_info.split_value = EFM_SplitType_CONTIUE_FROM_RIGHT;
                }else{
                    lane_extra_info.split_value = EFM_SplitType_SPLIT_FROM_RIGHT;
                }
            }
        }else{
            if (is_left){
                if (1 == lane_info_raw.Transit.data || 2 == lane_info_raw.Transit.data || 99 == lane_info_raw.Transit.data){
                    lane_extra_info.split_value = EFM_SplitType_TO_LEFT;
                }else{
                    lane_extra_info.split_value = EFM_SplitType_FROM_LEFT;
                }
            }else{
                if (1 == lane_info_raw.Transit.data || 2 == lane_info_raw.Transit.data || 99 == lane_info_raw.Transit.data){
                    lane_extra_info.split_value = EFM_SplitType_TO_RIGHT;
                }else{
                    lane_extra_info.split_value = EFM_SplitType_FROM_RIGHT;
                }                
            }
        }
        
        //存储split的信息，用于0930的道路split新需求
#ifdef SPLIT_0930
        std::cout << __FILE__ << "," << __LINE__ << ","<<"lane_info_raw.LaneNum.LaneNum: "<<(int)lane_info_raw.LaneNum.LaneNum<<" ,link_id: "<<link_id<<std::endl;
#endif
        SplitInfo_S split_info_tmp;
        bool is_stored_f = GetStoredSplitInfo(split_info_map,split_info_deque, lane_info_raw.LaneNum.LaneNum, link_id,split_info_tmp);
        //offset 变了需要更新已经保存的
        bool is_need_change = false;
        if(is_stored_f){
            message::map_map::s_LinkInfo_t link_infos{};
            if(efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_, *link_id_index_lane_info_map_, link_id, link_infos)){
                if(link_infos.PathOffset.PathOffset != split_info_tmp.s_offset){
                    is_need_change = true;
                }
            }            
        }

        if(is_need_change == true){}
        if(is_stored_f == false || is_need_change == true){
            std::vector<std::pair<uint64_t,SplitInfo_S>> split_info_vec_raw{};
            std::vector<std::pair<uint64_t,SplitInfo_S>> split_info_vec_side{};
            IsRoadSplit(link_id,lane_info_raw.LaneNum.LaneNum, lane_info_side.LaneNum.LaneNum,
                              map_position_->PathId,split_info_vec_raw,split_info_vec_side); 
            for(auto info:split_info_vec_raw){
                uint32_t link_id_tmp = static_cast<uint32_t>(info.first & (uint64_t)4294967295);
                uint8_t lane_num_tmp = static_cast<uint8_t>((info.first >> 32) & ((uint64_t)4294967295));
                SplitInfo_S split_info = info.second;
                split_info.split_dir = lane_extra_info.split_value;
                //没有保存成功，那么map里已经有了，就change
                if(!StoreSplitInfo(split_info_map, split_info_deque, split_info, lane_num_tmp, link_id_tmp)){
                     ChangeStoredSplitInfo(split_info_map, split_info_deque, lane_num_tmp, link_id_tmp, split_info);
                }  
            }                      
        }else{
            // do nothing
        }
        // end
    }
    
    return true;
}

bool CandidateLanesModel::SaveLaneCurvInfo(const message::map_map::s_LaneInfo_t& lane_info_raw, const uint32_t& link_id,
                                            LaneExtraInfo_s& lane_extra_info){
    // std::vector<CurvPoint> curvpoints{};
    // std::cout << __FILE__ << __LINE__ << "lane_info_raw.LaneNum.LaneNum: " << int(lane_info_raw.LaneNum.LaneNum) << std::endl;
    // std::cout << __FILE__ << __LINE__ << "link_id: " << link_id << std::endl;
    CurvPoint curvpoint{};
    for(auto& link_curv_info: map_static_info_->LinkCurvatures.LinkCurvatures){
        if (link_curv_info.InstanceId.InstanceId == link_id && link_curv_info.LaneNum.LaneNum == lane_info_raw.LaneNum.LaneNum){
            for(auto& curvpoint_info: link_curv_info.CurvPoints.CurvPoints){
                // std::cout << __FILE__ << __LINE__ << "curvpoint_info.CurvPointValue.CurvPointValue: " << curvpoint_info.CurvPointValue.CurvPointValue << std::endl;
                // std::cout << __FILE__ << __LINE__ << "curvpoint_info.PathOffset.PathOffset: " << int(curvpoint_info.PathOffset.PathOffset + link_curv_info.PathOffset.PathOffset) << std::endl;
                curvpoint.CurvPointValue = curvpoint_info.CurvPointValue.CurvPointValue / 100000.0f;
                curvpoint.CurvPointPathOffset = curvpoint_info.PathOffset.PathOffset + link_curv_info.PathOffset.PathOffset;
                curvpoint.linkid = link_id;
                lane_extra_info.curvpoints.push_back(curvpoint);
            }
        }
    }
    return true;
}

bool CandidateLanesModel::SaveLaneExtraInfo(const message::map_map::s_LaneInfo_t& lane_info_raw,
                                            const message::map_map::s_LaneInfo_t& lane_info_left,
                                            const message::map_map::s_LaneInfo_t& lane_info_right,
                                            uint32_t link_id,
                                            LaneExtraInfo_s& lane_extra_info) {
    //lane type
    lane_extra_info.lane_type = lane_info_raw.LaneType.data;

    //curvature
    if (lane_info_raw.LaneNum.LaneNum > 0){
        // SaveLaneCurvInfo(lane_info_raw, link_id, lane_extra_info);
        std::vector<CurvPoint> curvpoints{};
        efm::MapCommonTool::GetInstance()->SaveLaneCurvInfo(lane_info_raw.LaneNum.LaneNum, link_id, *curve_index_map_, *map_static_info_, curvpoints);
        lane_extra_info.curvpoints = curvpoints;
        // std::cout << __FILE__ << "," << __LINE__ << "," << "############# link_id: " << link_id <<" ,lane_info_raw.LaneNum.LaneNum: "<<(int)lane_info_raw.LaneNum.LaneNum<< std::endl;
        // for(auto cuv:lane_extra_info.curvpoints){
        //     std::cout<<" ,<CurvPointValue: "<<cuv.CurvPointValue<<" ,CurvPointPathOffset: "<<cuv.CurvPointPathOffset<<" ,link: "<< cuv.linkid<<">";
        // }
        // std::cout<<std::endl;
    }

    //merge/split
    if (lane_info_left.LaneNum.LaneNum > 0){
        SaveLaneExtraInfoMergeSplit(lane_info_raw, lane_info_left, link_id, lane_extra_info, true);
    }

    if (lane_info_right.LaneNum.LaneNum > 0){
        SaveLaneExtraInfoMergeSplit(lane_info_raw, lane_info_right, link_id, lane_extra_info, false);
    }
    
    return true;
}

bool CandidateLanesModel::SaveLaneAllExtraInfo(uint32_t link_id, uint8_t lane_num, LaneExtraInfo_s& lane_extra_info) {
    message::map_map::s_LinkInfo_t link_infos{};
    if (!GetLinkInfos(link_id, link_infos)) {
        //  std::cout << __FILE__ << "," << __LINE__ << ","
        //            << "GetLinkInfos error!!!!!!  " << link_id << std::endl;
        return false;
    }
    message::map_map::s_LaneInfo_t lane_info_raw{};
    message::map_map::s_LaneInfo_t lane_info_left{};
    message::map_map::s_LaneInfo_t lane_info_right{};
    lane_info_left.LaneNum.LaneNum = 0;
    lane_info_right.LaneNum.LaneNum = 0;
    bool find_lane = false;
    for (auto& lane : link_infos.LaneInfos.LaneInfos) {
        if (lane.LaneNum.LaneNum == lane_num) {
            lane_info_raw = lane;
            find_lane = true;
        }else if (lane.LaneNum.LaneNum + 1 == lane_num) {
            lane_info_right = lane;
        }else if (lane.LaneNum.LaneNum == lane_num + 1) {
            lane_info_left = lane;
        }
    }
    if (find_lane == false) {
        return false;
    }

    lane_extra_info.s_offset = static_cast<int32_t>(link_infos.PathOffset.PathOffset) - static_cast<int32_t>(map_position_->PathOffset) ;
    lane_extra_info.e_offset = static_cast<int32_t>(link_infos.EndOffset.EndOffset) - static_cast<int32_t>(map_position_->PathOffset) ;
    lane_extra_info.s_offset_raw = link_infos.PathOffset.PathOffset;
    lane_extra_info.e_offset_raw = link_infos.EndOffset.EndOffset;
    lane_extra_info.transit  = lane_info_raw.Transit.data;

    SaveLaneExtraInfo(lane_info_raw, lane_info_left, lane_info_right, link_id, lane_extra_info);
    // SaveLaneExtraInfo(link_id, lane_num, LaneExtraInfoType_e::NEW_CASE, lane_extra_info);
    return true;
}

bool CandidateLanesModel::GetTransitIndex(const std::vector<LaneExtraInfo_s>& lane_extra_vec,
                                          std::map<int, uint8_t>& transit_type) {
    // static constexpr uint8_t None_ = 0;
    // static constexpr uint8_t Continue_ = 1;
    // static constexpr uint8_t Merging_ = 2;
    // static constexpr uint8_t Splitting_ = 3;
    // static constexpr uint8_t Other_ = 99;
    transit_type.clear();
    for (int i = 0; i < lane_extra_vec.size(); i++) {
        if (lane_extra_vec[i].transit == 2 || lane_extra_vec[i].transit == 3 || lane_extra_vec[i].transit == 0 ||
            lane_extra_vec[i].transit == 99) {
            transit_type.emplace(i, lane_extra_vec[i].transit);
        }
    }
    return true;
}

bool CandidateLanesModel::ConstructLaneElement(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec,
                                               const std::vector<std::vector<LaneExtraInfo_s>> all_lanes_extra_vec_vec,
                                               LaneElementGroupSets& lane_element_group_sets) {
    lane_element_group_sets.clear();
    if (all_lanes_vec_vec.size() != all_lanes_extra_vec_vec.size()) {
        //  std::cout << __FILE__ << "," << __LINE__ << ","
        //            << "all_lanes_vec_vec,all_lanes_extra_vec_vec,size() error!!!!!!  "  << std::endl;
        return false;
    }
    for (int i = 0; i < all_lanes_vec_vec.size(); i++) {
        if (all_lanes_vec_vec[i].size() != all_lanes_extra_vec_vec[i].size() || all_lanes_vec_vec[i].empty() ||
            all_lanes_extra_vec_vec[i].empty()) {
            //  std::cout << __FILE__ << "," << __LINE__ << ","
            //            << "all_lanes_vec_vec[i] error!!!!!!  "<<i  << std::endl;
            continue;
        }
        uint8_t start_lane_num = all_lanes_vec_vec[i].front();
        bool a_new_group = true;
        int old_group_index = -1;
        for (int k = 0; k < lane_element_group_sets.size(); k++) {
            if (start_lane_num == lane_element_group_sets[k].front().lane_num_vec.front()) {
                a_new_group = false;
                old_group_index = k;
                break;
            }
        }
        if (a_new_group) {
            LaneElementGroup lane_elemnet_group{};
            LaneElement lane_element{};
            lane_element.lane_num_vec = all_lanes_vec_vec[i];
            lane_element.candidate_index = i;
            GetTransitIndex(all_lanes_extra_vec_vec[i], lane_element.transit_type);
            GetExtraData(all_lanes_extra_vec_vec[i], lane_element);
            GetLaneDist(all_lanes_vec_vec_, map_position_, link_id_vec_, i, lane_element.rest_length);
            lane_elemnet_group.push_back(lane_element);
            lane_element_group_sets.push_back(lane_elemnet_group);
        } else {
            if (old_group_index >= 0 && old_group_index < lane_element_group_sets.size()) {
                LaneElement lane_element{};
                lane_element.lane_num_vec = all_lanes_vec_vec[i];
                lane_element.candidate_index = i;
                GetTransitIndex(all_lanes_extra_vec_vec[i], lane_element.transit_type);
                GetExtraData(all_lanes_extra_vec_vec[i], lane_element);
                GetLaneDist(all_lanes_vec_vec_, map_position_, link_id_vec_, i, lane_element.rest_length);
                lane_element_group_sets[old_group_index].push_back(lane_element);
            }
        }
    }

    // bubble sort
    for (int i = 0; i < lane_element_group_sets.size(); i++) {
        for (int j = 0; j < lane_element_group_sets.size() - 1 - i; j++) {
            if (lane_element_group_sets[j].empty() || lane_element_group_sets[j + 1].empty() ||
                lane_element_group_sets[j].front().lane_num_vec.empty() ||
                lane_element_group_sets[j + 1].front().lane_num_vec.empty()) {
                return false;
            }
            if (lane_element_group_sets[j].front().lane_num_vec[0] >
                lane_element_group_sets[j + 1].front().lane_num_vec[0]) {
                LaneElementGroup tmp{};
                tmp = lane_element_group_sets[j];
                lane_element_group_sets[j] = lane_element_group_sets[j + 1];
                lane_element_group_sets[j + 1] = tmp;
            }
        }
    }

    return true;
}

bool CandidateLanesModel::GetExtraData(const std::vector<LaneExtraInfo_s>& lane_extra_vec, LaneElement& element) {
    element.lane_extra_infos.clear();

    if (lane_extra_vec.size() != element.lane_num_vec.size()){
        element.lane_extra_infos.assign(element.lane_num_vec.size(), LaneExtraInfo_s{});
        return false;
    }

    if (lane_extra_vec.empty()){
        return false;
    }
    element.lane_extra_infos.assign(lane_extra_vec.begin(), lane_extra_vec.end());
    return true;
}

// void CandidateLanesModel::GetDrvLaneSizeV2(const LaneElementGroupSets& lane_element_group_sets,
//                                            uint8_t righest_available_lane_num, uint8_t leftest_available_lane_num,
//                                            uint8_t& driveable_lane_size, uint8_t& right_not_driveable_lane_size) {
//     driveable_lane_size = 0;
//     right_not_driveable_lane_size = 0;
//     uint8_t driveable_lane_size_temp = 0;
//     uint32_t ego_link_id = map_position_->LinkId;
//     message::map_map::s_LinkInfo_t link_infos;
//     if (!GetLinkInfos(ego_link_id, link_infos)) {
//         //  std::cout << __FILE__ << "," << __LINE__ << ","
//         //            << "GetLinkInfos error!!!!!!  " << link_id << std::endl;
//         return;
//     }
//     for (auto lane : link_infos.LaneInfos.LaneInfos) {
//         if (lane.LaneType.data != 3 && lane.LaneType.data != 17) {
//             driveable_lane_size_temp++;
//         }
//     }

//     driveable_lane_size = driveable_lane_size_temp;
//     // if ref line is right , return, no need to fix lanenumber
//     if (ref_lane_prior_index_ == right_lane_prior_index_) {
//         return;
//     }

//     if (lane_element_group_sets.empty()) {
//         return;
//     }

//     double p_NotDriveDist = 1000;
//     uint8_t ego_lane_num = map_position_->LaneId;
//     // judge right merge
//     right_not_driveable_lane_size = 0;
//     for (int i = 0; i < lane_element_group_sets.size(); i++) {
//         if (lane_element_group_sets[i].empty() == false) {
//             if ((lane_element_group_sets[i].front().lane_num_vec.empty() == false) &&
//                 (ego_lane_num > lane_element_group_sets[i].front().lane_num_vec.front())) {
//                 bool is_merge_f = false;
//                 for (auto lane_elem : lane_element_group_sets[i]) {
//                     if (isMergeInRange(lane_elem, p_NotDriveDist)) {
//                         right_not_driveable_lane_size++;
//                         is_merge_f = true;
//                         break;
//                     }
//                 }
//                 if (is_merge_f == false) {
//                     bool is_lane_end = true;
//                     for (auto lane_elem : lane_element_group_sets[i]) {
//                         if (isLaneEndInRange(lane_elem, p_NotDriveDist) == false) {
//                             is_lane_end = false;
//                             break;
//                         }
//                     }
//                     if (is_lane_end) {
//                         right_not_driveable_lane_size++;
//                     }
//                 }
//             }
//         }
//     }
// #ifdef CLM_COUT
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << " driveable_lane_size: " << (int)driveable_lane_size << std::endl;
//     std::cout << __FILE__ << "," << __LINE__ << ","
//               << " right_not_driveable_lane_size: " << (int)right_not_driveable_lane_size << std::endl;
// #endif
//     driveable_lane_size = driveable_lane_size - right_not_driveable_lane_size;
//     driveable_lane_size = std::max(static_cast<uint8_t>(1), driveable_lane_size);

//     return;
// }

void CandidateLanesModel::GetDrvLaneSizeV3(const LaneElementGroupSets& lane_element_group_sets,
                                           uint8_t& driveable_lane_size, uint8_t& fixed_lane_id) {
    fixed_lane_id =  map_position_->LinkId;   
    driveable_lane_size =  map_position_->LinkId;   
    right_not_driveable_lane_size_ = 0;                                
    if (lane_element_group_sets.size() <= 0) {
        return;
    }

    for (auto lane_element_group : lane_element_group_sets) {
        if (lane_element_group.size() <= 0) {
            return;
        } else {
            for (auto lane_element : lane_element_group) {
                if (lane_element.lane_num_vec.size() <= 0) {
                    return;
                }
            }
        }
    }
    int pos_lane_id = static_cast<int>(map_position_->LaneId);
    int ego_lane_size = 0;
    for (auto lane_element_group : lane_element_group_sets) {
        if (lane_element_group.front().lane_num_vec.front() == pos_lane_id) {
            // get lane_element group
            for (auto lane_element : lane_element_group) {
                if (lane_element.is_group_dest == true) {
                    ego_lane_size = lane_element.lane_num_vec.size();
                }
            }
        }
    }
    if (ego_lane_size == 0) {
        return;
    }

    uint8_t driveable_lane_size_temp = 0;
    uint32_t ego_link_id = map_position_->LinkId;
    message::map_map::s_LinkInfo_t link_infos;
    if (!GetLinkInfos(ego_link_id, link_infos)) {
        //  std::cout << __FILE__ << "," << __LINE__ << ","
        //            << "GetLinkInfos error!!!!!!  " << link_id << std::endl;
        return;
    }
    fixed_lane_id = pos_lane_id;
    for (auto lane : link_infos.LaneInfos.LaneInfos) {
        if (lane.LaneType.data != 3 && lane.LaneType.data != 17 && lane.LaneType.data != 9) {
            driveable_lane_size_temp++;
        } else {
            fixed_lane_id--;
            fixed_lane_id = std::max(static_cast<uint8_t>(1), fixed_lane_id);
        }
    }
    driveable_lane_size = driveable_lane_size_temp;
#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << "raw driveable_lane_size: " << (int)driveable_lane_size
              << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "raw fixed_lane_id: " << (int)fixed_lane_id << std::endl;
#endif
    uint8_t right_not_driveable_lane_size = 0;
    for (int lane_num = pos_lane_id - 1; lane_num > 0; lane_num--) {
        for (auto lane_element_group : lane_element_group_sets) {
            if (lane_element_group.front().lane_num_vec.front() == lane_num) {
                // get lane_element group
                bool is_merge_f = false;
                bool is_lane_end = true;
                for (auto lane_element : lane_element_group) {
                    if (lane_element.is_group_dest == true) {
                        if (lane_element.lane_num_vec.size() < ego_lane_size && lane_element.rest_length<p_land_end_dist) {
                            // lane end
                            right_not_driveable_lane_size++;
                            break;
                        } else {
                            //如果右边这条group最长,那就不判断merge了
                            // if(lane_element.lane_num_vec.size()>ego_lane_size){
                            //     break;
                            // }                            
                            // merge
                            double merge_dist = 0;
                            bool fisrt_link_length_zero = false;
                            bool link_is_in_toll_f = false; 
                            if (lane_element.close_position >= 0 && lane_element.close_position != 100 &&
                                link_length_vec_.size() > lane_element.close_position && link_length_vec_.size() > 0 && 
                                link_id_vec_.size()>lane_element.close_position && link_id_vec_.size()>0) {
                                uint32_t link_id_tmp = link_id_vec_[lane_element.close_position];                              
                                for(auto id: toll_link_id_set_){
                                    if(id == link_id_tmp){
                                        link_is_in_toll_f = true;
                                        break;
                                    }    
                                }
                                for (int k = 0; k <= lane_element.close_position; k++) {
                                    merge_dist += link_length_vec_[k];
                                    if (merge_dist == 0) {
                                        merge_dist = 0;
                                        fisrt_link_length_zero = true;
                                    }
                                }
                            }
                            // std::cout << __FILE__ << "," << __LINE__ << ","
                            //           << "link_length_vec_[0]: " << link_length_vec_[0] << std::endl;
                            // std::cout << __FILE__ << "," << __LINE__ << ","
                            //           << "lane_element.close_position: " << lane_element.close_position << std::endl;
                            // std::cout << __FILE__ << "," << __LINE__ << ","
                            //           << "merge_dist: " << merge_dist << std::endl;
                            if(!link_is_in_toll_f){
                                if ((merge_dist > 0 || (fisrt_link_length_zero == true && merge_dist < 0.0001)) &&
                                    merge_dist < 1000 && ego_lane_size >= lane_element.close_position) {
                                    right_not_driveable_lane_size++;
                                    break;
                                }
                            }

                        }
                    }
                }
            }
        }
    }

    if (driveable_lane_size > right_not_driveable_lane_size) {
        driveable_lane_size = driveable_lane_size - right_not_driveable_lane_size;
        right_not_driveable_lane_size_ = right_not_driveable_lane_size;
    }
    if (fixed_lane_id > right_not_driveable_lane_size) {
        fixed_lane_id = fixed_lane_id - right_not_driveable_lane_size;
        right_not_driveable_lane_size_ = right_not_driveable_lane_size;
    }

#ifdef CLM_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "right_not_driveable_lane_size: " << (int)right_not_driveable_lane_size << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "driveable_lane_size: " << (int)driveable_lane_size << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << "," << "fixed_lane_id: " << (int)fixed_lane_id << std::endl;
#endif

    return;
}

bool CandidateLanesModel::isMergeInRange(const LaneElement& lane_element, double dist_thrd) {
    // judge is there merge in range of dist_thrd
    const std::vector<double>& link_length_vec = link_length_vec_;
    if (link_length_vec.size() < lane_element.lane_num_vec.size()) {
        return false;
    }

    if (lane_element.transit_type.empty()) {
        return false;
    } else {
        double min_dist = 99999;
        for (auto trans : lane_element.transit_type) {
            if (trans.second == 99 || trans.second == 0) {
                trans.second = 2;
            }
            if (trans.second == 2 && trans.first < link_length_vec.size()) {
                double dist = 0;
                for (int i = 0; i <= trans.first; i++) {
                    dist += link_length_vec[i];
                }
                if (min_dist > dist) {
                    min_dist = dist;
                }
            }
        }
        if (min_dist < dist_thrd && min_dist > 0.0001) {
            return true;
        }
    }
    return false;
}

bool CandidateLanesModel::isLaneEndInRange(const LaneElement& lane_element, double dist_thrd) {
    // judge is there all lane shorter than dist_thrd
    const std::vector<double>& link_length_vec = link_length_vec_;
    if (link_length_vec.size() < lane_element.lane_num_vec.size()) {
        return false;
    }
    double dist = 0;
    for (int i = 0; i < lane_element.lane_num_vec.size(); i++) {
        dist += link_length_vec[i];
    }

    if (dist > 0.0001 && dist < dist_thrd) {
        return true;
    }

    return false;
}

bool CandidateLanesModel::GetSingleLaneBackConnectInfo(const std::vector<std::vector<uint8_t>>& all_lanes_vec_vec,
                                                       const std::vector<uint32_t>& link_id_vec,
                                                       int candidate_lane_index, int lane_dir,
                                                       std::vector<uint32_t>& back_link_id_vec,
                                                       std::vector<uint8_t>& back_lane_num_vec) {
    back_link_id_vec.clear();
    back_lane_num_vec.clear();
    if (candidate_lane_index >= 0 && candidate_lane_index < all_lanes_vec_vec.size()) {
        std::vector<uint8_t> lane_vec = all_lanes_vec_vec[candidate_lane_index];
        if (!lane_vec.empty() && !link_id_vec.empty()) {
            uint8_t lane_num = lane_vec.front();
            uint32_t link_id = link_id_vec.front();
            bool get_back_lane_f = false;
            bool get_continue_f = false;
            message::map_map::s_LinkInfo_t link_infos;
            if (!GetLinkInfos(link_id, link_infos)) {
                // std::cout << __FILE__ << "," << __LINE__ << ","
                //           << "can't find link " << link_id;
            } else {
                double back_distance = (static_cast<double>(map_position_->PathOffset) -
                                        static_cast<double>(link_infos.PathOffset.PathOffset)) /
                                       100.0;
                int count = 0;
                while (back_distance >= 0 && back_distance < 100) {
                    // std::cout << __FILE__ << "," << __LINE__ << ","
                    //           << "link_id  " << link_id << " ,lane_num: " << (int)lane_num << " ,count:" << count
                    //           << std::endl;

                    count++;
                    std::vector<int> link_staticmap_indices{};
                    if (to_link_id_index_lane_connect_map_->find(link_id) !=
                        to_link_id_index_lane_connect_map_->end()) {
                        link_staticmap_indices = to_link_id_index_lane_connect_map_->at(link_id);
                        // ############plot
                        //  std::cout << __FILE__ << "," << __LINE__ << ","
                        //            << " ### to link id : " << link_id << std::endl;
                        //  for (auto index : link_staticmap_indices) {
                        //      std::cout
                        //          << " , from link id: "
                        //          << map_static_info_->LaneConnectivitys.PairConnectivity[index].FromLinkId.FromLinkId
                        //          << " ,new lane num: "
                        //          << (int)map_static_info_->LaneConnectivitys.PairConnectivity[index]
                        //                 .NewLaneNum.NewLaneNum
                        //          << " ,init lane num: "
                        //          << (int)map_static_info_->LaneConnectivitys.PairConnectivity[index]
                        //                 .InitLaneNum.InitLaneNum
                        //          << std::endl;
                        //  }
                        //  std::cout << "###" << std::endl;

                        // step 1, try to get an continue lane;
                        get_continue_f = false;
                        get_back_lane_f = false;
                        //1. 仅对于自车道，先在存储的历史走过的信息中找是不是有；path id, link id, lane id 都要检查
                        if(lane_dir == 0){
                            int max_drive_deque_index = -1;
                            for (auto index : link_staticmap_indices) {
                                if (lane_num ==
                                    map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum) {
                                    uint32_t temp_link_id =
                                        map_static_info_->LaneConnectivitys.PairConnectivity[index].FromLinkId.FromLinkId;
                                    uint8_t temp_lane_num =
                                        map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum;  

                                    uint64_t key_temp = ((static_cast<uint64_t>(temp_lane_num)) <<32) | (static_cast<uint64_t>(temp_link_id));
                                    // 有可能merge处的lane id 变了，导致相同的link id 保存了两个 lane id; 需要用deque里最新的lane id 和link id
                                    if(drive_lind_id_lane_id_map.find(key_temp)!=drive_lind_id_lane_id_map.end()){
                                        if(drive_lind_id_lane_id_map[key_temp] == map_position_-> PathId){
                                            //找到在deque里的最新的, index越大越新
                                            for(int i = drive_deque.size() - 1; i >=0 && i< drive_deque.size();i--){
                                                if(key_temp == drive_deque[i]){
                                                    if(max_drive_deque_index < i){
                                                        max_drive_deque_index = i;
                                                    }
                                                    break;
                                                }
                                            }                                      
                                        }
                                    }
                                }
                            }
                            if(max_drive_deque_index >= 0 && max_drive_deque_index< drive_deque.size()){
                            //找到了，用上
                                uint32_t get_link_id = static_cast<uint32_t>(drive_deque[max_drive_deque_index] & static_cast<uint64_t>(4294967295));
                                uint8_t get_lane_id = static_cast<uint8_t>((drive_deque[max_drive_deque_index]>>32) & static_cast<uint64_t>(4294967295));
                                get_continue_f = true;
                                get_back_lane_f = true;
                                back_link_id_vec.push_back(get_link_id);
                                back_lane_num_vec.push_back(get_lane_id);

                                // store for next while
                                lane_num = get_lane_id;
                                link_id = get_link_id;                                 
                            }                             
                        }
                       
                        //2. 再在存储的地方找是不是有修改的merge类型
                        if (get_continue_f == false && get_back_lane_f == false) {  
                            for (auto index : link_staticmap_indices) {
                                if (lane_num ==
                                    map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum) {
                                    uint32_t temp_link_id =
                                        map_static_info_->LaneConnectivitys.PairConnectivity[index].FromLinkId.FromLinkId;
                                    uint8_t temp_lane_num =
                                        map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum;  

                                    uint64_t key_temp = ((static_cast<uint64_t>(temp_lane_num)) <<32) | (static_cast<uint64_t>(temp_link_id));
                                    if(merge_lind_id_lane_id_map.find(key_temp)!=merge_lind_id_lane_id_map.end()){
                                        if(merge_lind_id_lane_id_map.at(key_temp) == 1){
                                            get_continue_f = true;
                                            get_back_lane_f = true;
                                            back_link_id_vec.push_back(temp_link_id);
                                            back_lane_num_vec.push_back(temp_lane_num);

                                            // store for next while
                                            lane_num = temp_lane_num;
                                            link_id = temp_link_id;
                                            break;                                        
                                        }
                                    }
                                }
                            }
                        }
                        
                        //3. 还没找到，找原始的continue
                        if (get_continue_f == false && get_back_lane_f == false) {
                            for (auto index : link_staticmap_indices) {
                                if (lane_num ==
                                    map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum) {
                                    uint32_t temp_link_id =
                                        map_static_info_->LaneConnectivitys.PairConnectivity[index].FromLinkId.FromLinkId;
                                    uint8_t temp_lane_num =
                                        map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum;                                                                   
                                    if (GetLinkInfos(temp_link_id, link_infos)) {                                   
                                        for (auto lane : link_infos.LaneInfos.LaneInfos) {
                                            if (lane.LaneNum.LaneNum == temp_lane_num &&
                                                lane.Transit.data == 1) {  // continue
                                                get_continue_f = true;
                                                get_back_lane_f = true;
                                                back_link_id_vec.push_back(temp_link_id);
                                                back_lane_num_vec.push_back(temp_lane_num);

                                                // store for next while
                                                lane_num = temp_lane_num;
                                                link_id = temp_link_id;
                                                // std::cout << __FILE__ << "," << __LINE__ << ","
                                                //           << "get_continue: link_id  " << temp_link_id
                                                //           << " ,lane_num: " << (int)temp_lane_num
                                                //           << " ,count:" << std::endl;
                                                break;
                                            }
                                        }
                                    } else {
                                        // std::cout << __FILE__ << "," << __LINE__ << ","
                                        //           << "can't find link_id  " << temp_link_id << std::endl;
                                    }
                                }
                                if (get_continue_f == true && get_back_lane_f == true) {
                                    break;
                                }
                            }
                        }

                        // step 4, if continue not found, use first found lane
                        if (get_continue_f == false && get_back_lane_f == false) {
                            // for (auto index : link_staticmap_indices) {
                            for (int k = link_staticmap_indices.size() - 1; k >= 0 && k < link_staticmap_indices.size();
                                 k--) {
                                int index = link_staticmap_indices[k];
                                if (lane_num ==
                                    map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum) {
                                    link_id = map_static_info_->LaneConnectivitys.PairConnectivity[index]
                                                  .FromLinkId.FromLinkId;
                                    back_link_id_vec.push_back(link_id);
                                    back_lane_num_vec.push_back(
                                        map_static_info_->LaneConnectivitys.PairConnectivity[index]
                                            .InitLaneNum.InitLaneNum);
                                    lane_num = map_static_info_->LaneConnectivitys.PairConnectivity[index]
                                                   .InitLaneNum.InitLaneNum;
                                    // std::cout << __FILE__ << "," << __LINE__ << ","
                                    //           << "can't get_continue: link_id  " << link_id
                                    //           << " ,lane_num: " << (int)lane_num << " ,count:" << std::endl;
                                    break;
                                }
                            }
                        }

                        if (!GetLinkInfos(link_id, link_infos)) {
                            // std::cout << __FILE__ << "," << __LINE__ << ","
                            //           << "can't find link " << link_id;
                            break;
                        } else {
                            back_distance = (static_cast<double>(map_position_->PathOffset) -
                                             static_cast<double>(link_infos.PathOffset.PathOffset)) /
                                            100.0;
                            // std::cout << __FILE__ << "," << __LINE__ << ","
                            //           << "map_position_->PathOffset  " << map_position_->PathOffset << std::endl;
                            // std::cout << __FILE__ << "," << __LINE__ << ","
                            //           << "ink_infos.PathOffset.PathOffset  " << link_infos.PathOffset.PathOffset
                            //           << std::endl;
                            // std::cout << __FILE__ << "," << __LINE__ << ","
                            //           << "link_id  " << link_id << std::endl;
                            // std::cout << __FILE__ << "," << __LINE__ << ","
                            //           << "back_distance  " << back_distance << std::endl;
                        }
                    } else {
                        // std::cout << __FILE__ << "," << __LINE__ << ","
                        //           << "to_link_id_index_lane_connect_map_ not find to link  " << link_id << std::endl;
                        break;
                    }
                    if (count == 10) {
                        break;
                    }
                }
                // std::cout << __FILE__ << "," << __LINE__ << ","
                //           << "count  " << count << std::endl;
            }
        }
    }
    return true;
}

bool CandidateLanesModel::ModifyMergeAttribute(const message::map_map::s_LaneInfo_t& lane_info_raw, const message::map_map::s_LaneInfo_t&lane_info_side, 
                                               uint32_t link_id, uint32_t next_link_id, uint8_t next_lane_id, uint8_t& raw_merge, uint8_t& side_merge){
    // std::cout<<"ModifyMergeAttribute:"<<std::endl;
    {  // 判断下一段是不是merge
        message::map_map::s_LinkInfo_t first_link_infos;
        if (!GetLinkInfos(link_id, first_link_infos)) {
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << "GetLinkInfos farlure!!!: " << link_id << std::endl;
            return false;
        }
        uint32_t first_end_offset = first_link_infos.EndOffset.EndOffset;
        uint32_t offset_gap = 0;
        uint32_t cur_link_id = next_link_id;
        uint32_t cur_lane_id = next_link_id;
        uint32_t n_link_id = 0;
        uint8_t n_lane_id = 0;
        int count = 0;
        while (offset_gap < 10000) {
                    //     std::cout << __FILE__ << "," << __LINE__ << ","
                    //   << "count : " << count <<" , cur_link_id: "<< cur_link_id<<" ,n_link_id: "<<n_link_id<<std::endl;
            message::map_map::s_LaneInfo_t next_lane_info_raw{};
            message::map_map::s_LaneInfo_t next_lane_info_left{};
            message::map_map::s_LaneInfo_t next_lane_info_right{};
            next_lane_info_left.LaneNum.LaneNum = 0;
            next_lane_info_right.LaneNum.LaneNum = 0;
            bool find_lane = false;
            message::map_map::s_LinkInfo_t next_link_infos;
            if (!GetLinkInfos(cur_link_id, next_link_infos)) {
                // std::cout << __FILE__ << "," << __LINE__ << ","
                //           << "GetLinkInfos farlure!!!: " << link_id << std::endl;
                return false;
            }
            for (auto& lane : next_link_infos.LaneInfos.LaneInfos) {
                if (lane.LaneNum.LaneNum == cur_lane_id) {
                    next_lane_info_raw = lane;
                    find_lane = true;
                } else if (lane.LaneNum.LaneNum + 1 == cur_lane_id) {
                    next_lane_info_right = lane;
                } else if (lane.LaneNum.LaneNum == cur_lane_id + 1) {
                    next_lane_info_left = lane;
                }
            }
            if (find_lane == false) {
                return false;
            }
            bool next_lane_is_merge = false;
            if (next_lane_info_left.LaneNum.LaneNum > 0) {
                next_lane_is_merge =
                    IsMergeLane(next_lane_info_raw.LaneNum.LaneNum, next_lane_info_left.LaneNum.LaneNum, cur_link_id,
                                n_link_id, n_lane_id);
            }
            if (next_lane_info_right.LaneNum.LaneNum > 0) {
                next_lane_is_merge =
                    IsMergeLane(next_lane_info_raw.LaneNum.LaneNum, next_lane_info_right.LaneNum.LaneNum, cur_link_id,
                                n_link_id, n_lane_id);
            }
                        //     std::cout << __FILE__ << "," << __LINE__ << ","
                        //   << "next_lane_is_merge!!!: " << next_lane_is_merge << std::endl;
            if (next_lane_is_merge) {
                raw_merge = 3;
                side_merge = 3;
                return true;
            }
            offset_gap = next_link_infos.EndOffset.EndOffset - first_end_offset;
            count++;
            if (count >= 3) {
                break;
            }
            cur_link_id = n_link_id;
            cur_lane_id = n_lane_id;
                    //     std::cout << __FILE__ << "," << __LINE__ << ","
                    //   << "offset_gap : " << offset_gap <<std::endl;            
        }
    }

    EFMRefLinePoints line_points_raw{};
    EFMRefLinePoints line_points_side{};
    double acumulate_heading_raw = 0;
    double acumulate_heading_side = 0;
    double average_heading_raw = 0;
    double average_heading_side = 0;   
    double acumulate_heading_next = 0;
    double average_heading_next = 0;      
    {//get cur line
        uint32_t line_index = lane_info_raw.Centeline.Centeline;
        // std::cout<< "line_index: "<<line_index<<std::endl;
        std::vector<message::map_map::s_GeometryPoint_t> geometry_points;
        if (!GetLine(line_index, geometry_points)) {
            // std::cout << "line not found 111" << std::endl;
            return false;
        }
        EFMRefLinePoints ref_line_points{};
        CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(geometry_points, ref_line_points, map_position_);  
        // 取几个点
        
        EFMRefLinePoints::const_iterator end_iter =ref_line_points.end();
        int size = ref_line_points.size();
        int start_index = std::min(size,4);
        EFMRefLinePoints::const_iterator start_iter =ref_line_points.end() - start_index;
        EFMRefLinePoints ref_line_points_temp{};
        ref_line_points_temp.assign(start_iter,end_iter);        

        std::vector<double> raw_accumulated_s, raw_kappas, raw_dkappas, raw_headings;
        CommonMathMethod::DiscretePointsMath::ComputePathProfile(ref_line_points_temp, &raw_headings, &raw_accumulated_s,
                                                            &raw_kappas, &raw_dkappas);   
        if(raw_headings.size()>0){
             for(int i =0;i<raw_headings.size();i++){
                acumulate_heading_raw += raw_headings[i];
             }
              average_heading_raw = acumulate_heading_raw/(raw_headings.size());
        }               
    }
    {//get side line
        uint32_t line_index = lane_info_side.Centeline.Centeline;
        std::vector<message::map_map::s_GeometryPoint_t> geometry_points;
        if (!GetLine(line_index, geometry_points)) {
            // std::cout << "line not found 222" << std::endl;
            return false;
        }
        EFMRefLinePoints ref_line_points{};
        CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(geometry_points, ref_line_points, map_position_); 
        //取几个点
        EFMRefLinePoints::const_iterator end_iter =ref_line_points.end();
        int size = ref_line_points.size();
        int start_index = std::min(size,4);
        EFMRefLinePoints::const_iterator start_iter =ref_line_points.end() - start_index;
        EFMRefLinePoints ref_line_points_temp{};
        ref_line_points_temp.assign(start_iter,end_iter);  

        std::vector<double> raw_accumulated_s, raw_kappas, raw_dkappas, raw_headings;
        CommonMathMethod::DiscretePointsMath::ComputePathProfile(ref_line_points_temp, &raw_headings, &raw_accumulated_s,
                                                            &raw_kappas, &raw_dkappas);   
        if(raw_headings.size()>0){
             for(int i =0;i<raw_headings.size();i++){
                acumulate_heading_side += raw_headings[i];
             }
              average_heading_side = acumulate_heading_side/(raw_headings.size());
        }   
    }    
    //get next lane's center line
    {
        message::map_map::s_LinkInfo_t next_link_infos;
        std::vector<std::pair<double, double>> xy_points;
        std::vector<double> headings, accumulated_s, kappas, dkappas;
        if (!GetLinkInfos(next_link_id, next_link_infos)) {
            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << "GetLinkInfos farlure!!!: " << link_id << std::endl;
            return false;
        }
        uint32_t next_line_index = 0;
        for (auto lane : next_link_infos.LaneInfos.LaneInfos) {
            if (lane.LaneNum.LaneNum == next_lane_id) {
                next_line_index = lane.Centeline.Centeline;
                break;
            }
        }
        if (next_line_index == 0) {
            // std::cout << "line not found" << std::endl;
            return false;
        }
        std::vector<message::map_map::s_GeometryPoint_t> geometry_points;
        if (!GetLine(next_line_index, geometry_points)) {
            // std::cout << "line not found 333" << std::endl;
            return false;
        }
        EFMRefLinePoints ref_line_points{};
        CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(geometry_points, ref_line_points, map_position_);
        //取前几个点
        EFMRefLinePoints::const_iterator start_iter =ref_line_points.begin();
        int size = ref_line_points.size();
        int end_index = std::min(size,4);
        EFMRefLinePoints::const_iterator end_iter =ref_line_points.begin() + end_index;
        EFMRefLinePoints ref_line_points_temp{};
        ref_line_points_temp.assign(start_iter,end_iter);          

        std::vector<double> raw_accumulated_s, raw_kappas, raw_dkappas, raw_headings;

        CommonMathMethod::DiscretePointsMath::ComputePathProfile(ref_line_points_temp, &raw_headings, &raw_accumulated_s,
                                                            &raw_kappas, &raw_dkappas);   
        if(raw_headings.size()>0){
             for(int i =0;i<raw_headings.size();i++){
                acumulate_heading_next += raw_headings[i];
             }
              average_heading_next = acumulate_heading_next/(raw_headings.size());
        } 
    }

    // std::cout<<"acumulate_heading_raw: "<< acumulate_heading_raw<<std::endl;
    // std::cout<<"acumulate_heading_side: "<< acumulate_heading_side<<std::endl;
    // std::cout<<"average_heading_raw: "<< average_heading_raw<<std::endl;
    // std::cout<<"average_heading_side: "<< average_heading_side<<std::endl;
    // std::cout<<"acumulate_heading_next: "<< acumulate_heading_next<<std::endl;
    // std::cout<<"average_heading_next: "<< average_heading_next<<std::endl; 
    //计算平均heading，判断哪个直
    double raw_heading_cost = abs(average_heading_next - average_heading_raw);
    double side_heading_cost = abs(average_heading_next - average_heading_side);
    // std::cout<<"raw_heading_cost: "<< raw_heading_cost<<std::endl;
    // std::cout<<"side_heading_cost: "<< side_heading_cost<<std::endl; 
    if(raw_heading_cost>p_heading_for_merge_first_level && side_heading_cost> p_heading_for_merge_first_level){
             raw_merge = 3;
            side_merge = 3;         
    }else{
        if(raw_heading_cost - side_heading_cost > p_heading_for_merge_sec_level){
            raw_merge = 2;
            side_merge = 1;
        }else if(side_heading_cost - raw_heading_cost > p_heading_for_merge_sec_level){
            raw_merge = 1;
            side_merge = 2;
        }
    }

    return true;
}

bool CandidateLanesModel::ModifyMergeAttributeBySideLine(const message::map_map::s_LaneInfo_t& lane_info_raw, const message::map_map::s_LaneInfo_t&lane_info_side, 
                                               uint32_t link_id,uint32_t next_link_id, uint8_t next_lane_id, uint8_t& raw_merge, uint8_t& side_merge){
    message::map_map::s_LaneInfo_t left_lane{};
    message::map_map::s_LaneInfo_t right_lane{};
    uint8_t ri_merge = 3;
    uint8_t le_merge = 3;
    int dir = 0;
    if(lane_info_raw.LaneNum.LaneNum < lane_info_side.LaneNum.LaneNum){
        right_lane = lane_info_raw;
        left_lane = lane_info_side;
        dir = 1;
    }else if(lane_info_raw.LaneNum.LaneNum > lane_info_side.LaneNum.LaneNum){
        left_lane = lane_info_raw;
        right_lane = lane_info_side;
        dir =2;
    }else{
        return false;
    }

    double acumulate_heading_ri_lane_le = 0;
    double acumulate_heading_le_lane_ri = 0;
    double aver_heading_ri_lane_le = 0;
    double aver_heading_le_lane_ri = 0;
#ifdef MODIFY_MERGE        
    std::cout<<"link_id: "<< link_id<<std::endl;
#endif

    CalculateMergeLinkLineHeading(left_lane, right_lane, *map_static_info_, acumulate_heading_ri_lane_le, acumulate_heading_le_lane_ri, 
                                    aver_heading_ri_lane_le, aver_heading_le_lane_ri);

    bool next_lane_is_avl = false;
    uint8_t next_lane_type = 0;
    uint32_t next_lane_line_index = 0;
    double acumulate_heading_next_lane = 0;
    double aver_heading_next_lane = 0;
    {//get next lane type, and center line    
        message::map_map::s_LinkInfo_t next_link_infos;
        if(GetLinkInfos(next_link_id, next_link_infos)){
            for(auto lane:next_link_infos.LaneInfos.LaneInfos){
                if(next_lane_id == lane.LaneNum.LaneNum){
                    next_lane_type = lane.LaneType.data;
                    next_lane_line_index = lane.Centeline.Centeline;
                    std::vector<message::map_map::s_GeometryPoint_t> geometry_points{};
                    if (GetLine(next_lane_line_index, geometry_points)) {
                           // std::cout << "line not found 222" << std::endl;
                        next_lane_is_avl = true;  
                        EFMRefLinePoints line_points{};
                        CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(geometry_points, line_points, map_position_);
                        EFMRefLinePoints line_points_tmp{};
                        for(auto point: line_points){
                            line_points_tmp.push_back(point);
                            if(line_points_tmp.size()>=4){
                                break;
                            }
                        }
                        std::vector<double> raw_accumulated_s, raw_kappas, raw_dkappas, raw_headings;
                        CommonMathMethod::DiscretePointsMath::ComputePathProfile(line_points_tmp, &raw_headings, &raw_accumulated_s,
                                                                         &raw_kappas, &raw_dkappas);
                        if (raw_headings.size() > 0) {
                            for (int i = 0; i < raw_headings.size(); i++) {
                                double heading_tmp = raw_headings[i];
                                heading_tmp = HeadingTransform(raw_headings[i]);
                                acumulate_heading_next_lane += heading_tmp;
                            }
                            aver_heading_next_lane = acumulate_heading_next_lane / (raw_headings.size());
                        }                       
                        break;
                    } 
                }
            }
        }else{
            next_lane_is_avl = false;
        } 
#ifdef MODIFY_MERGE
        std::cout<<"next_link_id: "<< (int)next_link_id<<std::endl;  
#endif
    } 

//1.如果连接的车道lane type不同，分为主路和非主路,那么和merge后的车道的lane type一致的认为是continue；
    uint8_t le_lane_type = left_lane.LaneType.data;
    uint8_t ri_lane_type = right_lane.LaneType.data;
#ifdef MODIFY_MERGE 
            std::cout<<"le_lane_type: "<< (int)le_lane_type<<std::endl;
            std::cout<<"ri_lane_type: "<< (int)ri_lane_type<<std::endl;            
            std::cout<<"next_lane_is_avl: "<< (int)next_lane_is_avl<<std::endl;
#endif
    if(next_lane_is_avl == true && ((le_lane_type == 0 && ri_lane_type != 0) || (ri_lane_type == 0 && le_lane_type != 0))){
        if(next_lane_type == 0 || next_lane_type == 7){
            if(le_lane_type == 0){
                le_merge = 1;
                ri_merge = 2;
            }else{
                le_merge = 2;                       
                ri_merge = 1;                        
            }
        }else{
            // next lane not 0
            if(le_lane_type == 0){
               le_merge = 2;
                ri_merge = 1;
            }else{
                le_merge = 1;
                ri_merge = 2;                        
            }                    
        }
        if(dir == 1){
            raw_merge = ri_merge;
            side_merge = le_merge;
        }else if(dir ==2){
            raw_merge = le_merge;
            side_merge = ri_merge;
        }
        // std::cout << "lane type return" << std::endl;
        return true;  
    }
    double acumulate_heading_last_lane_common = 0;
    double aver_heading_last_lane_common = 0;
    std::vector<double> acumulate_heading_le_last_lane_ri_vec{};
    std::vector<double> aver_heading_le_last_lane_ri_vec{};
    std::vector<double> acumulate_heading_ri_last_lane_le_vec{};
    std::vector<double> aver_heading_ri_last_lane_le_vec{};
    int conuter = 0;
    bool find_common_line_f = false;
    bool more_than_one_lane = false;
    uint32_t while_link_id = link_id;
    
    //如果merge的左边还有车道，而且左边的车道是主路而且，左边车道的右线是虚线，那么merge middle
    //如果merge的左车道是最左车道，如果merge的右边还有车道，而且右边车道的左线是虚线，那么merge middle
    {
        message::map_map::s_LinkInfo_t link_infos;
        uint8_t left_lane_num = 0;
        uint8_t right_lane_num = 0;
        if(lane_info_raw.LaneNum.LaneNum> lane_info_side.LaneNum.LaneNum){
            left_lane_num = lane_info_raw.LaneNum.LaneNum;
            right_lane_num = lane_info_side.LaneNum.LaneNum;
        }else{
            left_lane_num = lane_info_side.LaneNum.LaneNum;
            right_lane_num = lane_info_raw.LaneNum.LaneNum;
        }
// std::cout << __FILE__ << "," << __LINE__ << "," << " left_lane_num: " << (int)left_lane_num<< std::endl;
// std::cout << __FILE__ << "," << __LINE__ << "," << " right_lane_num: " << (int)right_lane_num<< std::endl;
        if(left_lane_num != 0 && right_lane_num!=0){
            if(GetLinkInfos(link_id, link_infos)){
                uint8_t max_lane_num = 0;
                for(auto lane:link_infos.LaneInfos.LaneInfos){
                    if(max_lane_num< lane.LaneNum.LaneNum){
                        max_lane_num = lane.LaneNum.LaneNum;
                    }
                }
                // std::cout << __FILE__ << "," << __LINE__ << "," << " max_lane_num: " << (int)max_lane_num<< std::endl;
                if(left_lane_num == max_lane_num){//最左侧车道
                    for(auto lane:link_infos.LaneInfos.LaneInfos){
                        if(lane.LaneNum.LaneNum == right_lane_num -1 && lane.LaneNum.LaneNum != 0){
                            // std::cout << __FILE__ << "," << __LINE__ << "," << " lane.LaneNum.LaneNum : " << (int)lane.LaneNum.LaneNum << std::endl;
                            uint32_t line_index = lane.LBound.LBound;
                            uint8_t mrk_type = 0;
                            efm::MapCommonTool::GetInstance()->GetLineType(line_index, *map_static_info_, *linear_obj_id_map_,mrk_type); 
                            // std::cout << __FILE__ << "," << __LINE__ << "," << " line_index: " << (int)line_index<< std::endl;
                            // std::cout << __FILE__ << "," << __LINE__ << "," << " mrk_type: " << (int)mrk_type<< std::endl;
                            if(mrk_type == 3 ||mrk_type == 7){//虚线或左虚右实，可跨越，那么merge middle
                                raw_merge = 3;
                                side_merge = 3;
                                return true;
                            }                            
                        }
                    }
                }else{
                    for(auto lane:link_infos.LaneInfos.LaneInfos){
                        if(lane.LaneNum.LaneNum == left_lane_num+1){
                            uint32_t line_index = lane.RBound.RBound;
                            uint8_t mrk_type = 0;
                            efm::MapCommonTool::GetInstance()->GetLineType(line_index, *map_static_info_, *linear_obj_id_map_,mrk_type); 
                            if(mrk_type == 3 ||mrk_type == 6){//虚线或左实右虚，可跨越，那么merge middle
                                raw_merge = 3;
                                side_merge = 3;
                                return true;
                            }
                        }
                    }                    
                }

            }                  
        }else{
            return false;
        }
      
    }

    //往回找多段link，
    // 2. 如果往回找到一条车道是虚拟，一条车道是非虚拟的，那么非虚拟的车道线的路是continue
    // 3. 如果回找 找不到共同边界线的情况，那么久直接return， 双侧都是3；
    // 4. 如果回找， 找到共同线，但是车道又存在2合1的，那么就用双侧虚拟的边界线和merge后的中心线求 heading差值
    // 5. 找到共同边界线的，那么就用共同边界线和 到merge处的边界线的heading累加值求heading差值
    while(conuter<=10){//get last lane common line
#ifdef MODIFY_MERGE
        std::cout<<"################# while : "<< conuter<< " ,link_id: "<< while_link_id<<std::endl;
#endif
        std::vector<std::pair<uint8_t,uint32_t>> ri_lane_last_lane_id_vec{};
        std::vector<std::pair<uint8_t,uint32_t>> le_lane_last_lane_id_vec{};
        std::vector<int> link_staticmap_indices{};
        if (to_link_id_index_lane_connect_map_->find(while_link_id) != to_link_id_index_lane_connect_map_->end()) {
            link_staticmap_indices = to_link_id_index_lane_connect_map_->at(while_link_id);
        }
        if(link_staticmap_indices.empty()) {
            return false;
        } 
        uint32_t last_link_id = 0;
        for(auto index : link_staticmap_indices){
            if(map_static_info_->LaneConnectivitys.PairConnectivity[index].ToLinkId.ToLink == while_link_id && map_static_info_->LaneConnectivitys.PairConnectivity[index].PathId.PathId == map_position_->PathId){
                last_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[index].FromLinkId.FromLinkId;
                if(map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum == right_lane.LaneNum.LaneNum){
                    ri_lane_last_lane_id_vec.push_back(std::make_pair(map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum,last_link_id));
                }else if(map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum == left_lane.LaneNum.LaneNum){
                    le_lane_last_lane_id_vec.push_back(std::make_pair(map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum,last_link_id));
                }
            }
        }

        message::map_map::s_LaneInfo_t left_lane_last_lane{};
        left_lane_last_lane.LaneNum.LaneNum = 0;
        message::map_map::s_LaneInfo_t right_lane_last_lane{}; 
        right_lane_last_lane.LaneNum.LaneNum = 0;
        bool found_lanes = false;       
        for(auto le_id:le_lane_last_lane_id_vec){
            message::map_map::s_LinkInfo_t last_link_infos;
            if(!GetLinkInfos(le_id.second, last_link_infos)){
                return false;
            }
            for(auto ri_id:ri_lane_last_lane_id_vec){
                if(le_id.first == (ri_id.first+1)&& le_id.second == ri_id.second){
                    for(auto lane:last_link_infos.LaneInfos.LaneInfos){
                        if(lane.LaneNum.LaneNum == le_id.first){
                            left_lane_last_lane = lane;
                        }
                        if(lane.LaneNum.LaneNum == ri_id.first){
                            right_lane_last_lane = lane;
                        }
                        if(right_lane_last_lane.LaneNum.LaneNum !=0 && left_lane_last_lane.LaneNum.LaneNum !=0){
                            found_lanes =true;
                            while_link_id = ri_id.second; // update while link
                            break;
                        }
                    }
                }
                if(found_lanes == true){
                    break;
                }
            }
            if(found_lanes == true){
                break;
            }    
        }

        if(found_lanes == true && (ri_lane_last_lane_id_vec.size()>1 || le_lane_last_lane_id_vec.size()>1)){
            more_than_one_lane =true;
        }
#ifdef MODIFY_MERGE         
        std::cout<<"found_lanes: "<< found_lanes<<std::endl;
        std::cout<<"le_lane_last_lane_id_vec: ";
        for(auto le_id:le_lane_last_lane_id_vec){
            std::cout<<" ,< lane_num: "<<(int)le_id.first<<" ,link: "<<le_id.second<<">";
        }
        std::cout<<std::endl;
        std::cout<<"ri_lane_last_lane_id_vec: ";
        for(auto le_id:ri_lane_last_lane_id_vec){
            std::cout<<" ,< lane_num: "<<(int)le_id.first<<" ,link: "<<le_id.second<<">";
        }
        std::cout<<std::endl;   
        std::cout<<"left_lane_last_lane.LaneNum.LaneNum: "<< (int)left_lane_last_lane.LaneNum.LaneNum<<std::endl;
        std::cout<<"right_lane_last_lane.LaneNum.LaneNum: "<< (int)right_lane_last_lane.LaneNum.LaneNum<<std::endl;
#endif
              
        if(found_lanes == true){
            // get  line,如果有一条所实际的线，那么实际的线的路就是continue
            uint32_t le_lane_ri_index = left_lane_last_lane.RBound.RBound;
            uint8_t le_lane_ri_type =0;
            
            // std::cout<<"left_lane_last_lane.RBound.RBound: "<< left_lane_last_lane.RBound.RBound<<std::endl;
            std::vector<message::map_map::s_GeometryPoint_t> geometry_points_le_lane_ri{};
            if (!GetLine(le_lane_ri_index, geometry_points_le_lane_ri,le_lane_ri_type)) {
                // std::cout << "line not found 222" << std::endl;
                return false;
            }
            uint32_t ri_lane_le_index = right_lane_last_lane.LBound.LBound;
            uint8_t ri_lane_le_type =0;
            
            // std::cout<<"right_lane_last_lane.LBound.LBound: "<< right_lane_last_lane.LBound.LBound<<std::endl;
            std::vector<message::map_map::s_GeometryPoint_t> geometry_points_ri_lane_le{};
            if (!GetLine(ri_lane_le_index, geometry_points_ri_lane_le,ri_lane_le_type)) {
                // std::cout << "line not found 222" << std::endl;
                return false;
            } 
            //一侧实际线的为continue
#ifdef MODIFY_MERGE 
            std::cout<<"ri_lane_le_index: "<< (int)ri_lane_le_index<<std::endl;
            std::cout<<"le_lane_ri_index: "<< (int)le_lane_ri_index<<std::endl;            
            std::cout<<"ri_lane_le_type: "<< (int)ri_lane_le_type<<std::endl;
            std::cout<<"le_lane_ri_type: "<< (int)le_lane_ri_type<<std::endl;
#endif
            if(le_lane_ri_type ==8 && (ri_lane_le_type==2 ||ri_lane_le_type==3 ||ri_lane_le_type==4 ||ri_lane_le_type==5 ||ri_lane_le_type==6 ||ri_lane_le_type==7)){
                //如果有一条所实际的线，那么实际的线的路就是continue
                ri_merge = 1;
                le_merge = 2;
                if(dir == 1){
                    raw_merge = ri_merge;
                    side_merge = le_merge;
                }else if(dir ==2){
                    raw_merge = le_merge;
                    side_merge = ri_merge;
                }
                // std::cout<<"ri lane is continue "<<std::endl;
                return true;
            }else if(ri_lane_le_type ==8 && (le_lane_ri_type==2 ||le_lane_ri_type==3 ||le_lane_ri_type==4 ||le_lane_ri_type==5 ||le_lane_ri_type==6 ||le_lane_ri_type==7)){
                //如果有一条所实际的线，那么实际的线的路就是continue
                ri_merge = 2;
                le_merge = 1;
                if(dir == 1){
                    raw_merge = ri_merge;
                    side_merge = le_merge;
                }else if(dir ==2){
                    raw_merge = le_merge;
                    side_merge = ri_merge;
                }
                // std::cout<<"le lane is continue "<<std::endl;
                return true;
            }else{
                EFMRefLinePoints ri_lane_le_line_points{};
                CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(geometry_points_ri_lane_le, ri_lane_le_line_points, map_position_);
                if((le_lane_ri_index == ri_lane_le_index)){//相同边界线只取4个点
                    EFMRefLinePoints pnts_tmp = ri_lane_le_line_points;
                    ri_lane_le_line_points.clear();
                    for(int i = pnts_tmp.size()-1;i>=0 && i< pnts_tmp.size();i--){
                        ri_lane_le_line_points.insert(ri_lane_le_line_points.begin(),pnts_tmp[i]);
                        if(ri_lane_le_line_points.size()>=4){
                            break;
                        }
                    }
                }
                std::vector<double> raw_accumulated_s, raw_kappas, raw_dkappas, raw_headings;
                CommonMathMethod::DiscretePointsMath::ComputePathProfile(ri_lane_le_line_points, &raw_headings, &raw_accumulated_s,
                                                                         &raw_kappas, &raw_dkappas);
                double acumulate_heading_ri_last_lane = 0;
                double aver_heading_ri_last_lane = 0;
                if (raw_headings.size() > 0) {
                    for (int i = 0; i < raw_headings.size(); i++) {
                        double heading_tmp = raw_headings[i];
                        heading_tmp = HeadingTransform(raw_headings[i]);
                        acumulate_heading_ri_last_lane += heading_tmp;
                    }
                    aver_heading_ri_last_lane = acumulate_heading_ri_last_lane / (raw_headings.size());
                }

                EFMRefLinePoints le_lane_ri_line_points{};
                CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(geometry_points_le_lane_ri, le_lane_ri_line_points, map_position_);
                if((le_lane_ri_index == ri_lane_le_index)){//相同边界线只取4个点
                    EFMRefLinePoints pnts_tmp = le_lane_ri_line_points;
                    le_lane_ri_line_points.clear();
                    for(int i = pnts_tmp.size()-1;i>=0 && i< pnts_tmp.size();i--){
                        le_lane_ri_line_points.insert(le_lane_ri_line_points.begin(),pnts_tmp[i]);
                        if(le_lane_ri_line_points.size()>=4){
                            break;
                        }
                    }
                }
                std::vector<double> raw_accumulated_s_tmp, raw_kappas_tmp, raw_dkappas_tmp, raw_headings_tmp;
                CommonMathMethod::DiscretePointsMath::ComputePathProfile(le_lane_ri_line_points, &raw_headings_tmp, &raw_accumulated_s_tmp,
                                                                         &raw_kappas_tmp, &raw_dkappas_tmp);
                double acumulate_heading_le_last_lane = 0;
                double aver_heading_le_last_lane = 0;
                if (raw_headings_tmp.size() > 0) {
                    for (int i = 0; i < raw_headings_tmp.size(); i++) {
                        double heading_tmp = raw_headings_tmp[i];
                        heading_tmp = HeadingTransform(raw_headings_tmp[i]);
                        acumulate_heading_le_last_lane += heading_tmp;
                    }
                    aver_heading_le_last_lane = acumulate_heading_le_last_lane / (raw_headings_tmp.size());
                }

                if(le_lane_ri_index == ri_lane_le_index){
                    acumulate_heading_last_lane_common = acumulate_heading_le_last_lane;
                    aver_heading_last_lane_common = aver_heading_le_last_lane;
                    find_common_line_f = true;
#ifdef MODIFY_MERGE
                    std::cout<<"common line id: "<< le_lane_ri_index<<std::endl;
#endif
                    break;
                }else{
                    acumulate_heading_le_last_lane_ri_vec.push_back(acumulate_heading_le_last_lane);
                    aver_heading_le_last_lane_ri_vec.push_back(aver_heading_le_last_lane);
                    acumulate_heading_ri_last_lane_le_vec.push_back(acumulate_heading_ri_last_lane);
                    aver_heading_ri_last_lane_le_vec.push_back(aver_heading_ri_last_lane);                    
                }                               
            }
        }else{
            return false;
        }
        conuter++;
    }
#ifdef MODIFY_MERGE
    std::cout<<"acumulate_heading_next_lane: "<< acumulate_heading_next_lane<<std::endl;
    std::cout<<"aver_heading_next_lane: "<< aver_heading_next_lane<<std::endl;
    std::cout<<"acumulate_heading_ri_lane_le: "<< acumulate_heading_ri_lane_le<<std::endl;
    std::cout<<"aver_heading_ri_lane_le: "<< aver_heading_ri_lane_le<<std::endl;
    std::cout<<"acumulate_heading_le_lane_ri: "<< acumulate_heading_le_lane_ri<<std::endl;
    std::cout<<"aver_heading_le_lane_ri: "<< aver_heading_le_lane_ri<<std::endl;
    std::cout<<"aver_heading_ri_last_lane_le_vec: ";
    for(auto ri_h:aver_heading_ri_last_lane_le_vec){
        std::cout<<" ,"<<ri_h;
    }
    std::cout<<std::endl;
    std::cout<<"aver_heading_le_last_lane_ri_vec: ";
    for(auto ri_h:aver_heading_le_last_lane_ri_vec){
        std::cout<<" ,"<<ri_h;
    }
    std::cout<<std::endl;
    std::cout<<"aver_heading_last_lane_common: "<< aver_heading_last_lane_common<<std::endl;
    std::cout<<"find_common_line_f: "<< (int)find_common_line_f<<std::endl;
    std::cout<<"more_than_one_lane: "<< (int)more_than_one_lane<<std::endl;
    std::cout<<"next_lane_is_avl: "<< (int)next_lane_is_avl<<std::endl;
#endif
    //如果回找 找不到共同边界线的情况，那么久直接return， 双侧都是3；
    if(aver_heading_le_last_lane_ri_vec.size() != aver_heading_ri_last_lane_le_vec.size() || find_common_line_f == false){
        return false;
    }

    double common_heading = 0;
    if(find_common_line_f == true && more_than_one_lane == true && next_lane_is_avl == true){
        //如果回找， 找到共同线，但是车道又存在2合1的，那么就用双侧虚拟的边界线和merge后的中心线求 heading差值
        common_heading = aver_heading_next_lane;
    }else{
        common_heading = aver_heading_last_lane_common;
    }
    //计算平均heading，判断哪个直
    double ri_heading_cost = 0;
    double le_heading_cost = 0;
    //heading 用atan2算的，需要判断正负
    if(abs(aver_heading_ri_lane_le - common_heading)>3.14){
        ri_heading_cost = abs(6.28 - abs(aver_heading_ri_lane_le - common_heading));
    }else{
        ri_heading_cost = abs(aver_heading_ri_lane_le - common_heading);
    }
    for(auto ri_h:aver_heading_ri_last_lane_le_vec){
        if(abs(ri_h - common_heading)>3.14){
            ri_heading_cost += abs(6.28 - abs(ri_h - common_heading));
        }else{
            ri_heading_cost += abs(ri_h - common_heading);
        }
    }
    if(abs(aver_heading_le_lane_ri - common_heading)>3.14){
        le_heading_cost = abs(6.28 - abs(aver_heading_le_lane_ri - common_heading));
    }else{
        le_heading_cost = abs(aver_heading_le_lane_ri - common_heading);
    }
    for(auto le_h:aver_heading_le_last_lane_ri_vec){
        if(abs(le_h - common_heading)>3.14){
            le_heading_cost += abs(6.28 - abs(le_h - common_heading));
        }else{
            le_heading_cost += abs(le_h - common_heading);
        }
    }
#ifdef MODIFY_MERGE
    std::cout<<"ri_heading_cost: "<< ri_heading_cost<<std::endl;
    std::cout<<"le_heading_cost: "<< le_heading_cost<<std::endl; 
    std::cout<<"p_heading_for_merge_first_level: "<< p_heading_for_merge_first_level<<std::endl; 
    std::cout<<"p_heading_for_merge_sec_level: "<< p_heading_for_merge_sec_level<<std::endl;
#endif    
    
    double heading_gap = abs(ri_heading_cost - le_heading_cost);
    if(le_heading_cost<p_heading_for_merge_first_level && ri_heading_cost<p_heading_for_merge_first_level){
            ri_merge = 3;
            le_merge = 3;
    }else if(heading_gap > p_heading_for_merge_sec_level){
        if(ri_heading_cost>le_heading_cost){
            ri_merge = 2;
            le_merge = 1;            
        }else{
            ri_merge = 1;
            le_merge = 2;             
        }
    }else{
            ri_merge = 3;
            le_merge = 3; 
    }
    
    // std::cout<<"dir: "<< dir<<std::endl;
    if(dir == 1){
        raw_merge = ri_merge;
        side_merge = le_merge;
    }else if(dir ==2){
        raw_merge = le_merge;
        side_merge = ri_merge;
    }
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"raw_merge: "<< (int)raw_merge<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"side_merge: "<< (int)side_merge<<std::endl;
    return true;
}


bool CandidateLanesModel::ModifySplitAttribute(const message::map_map::s_LaneInfo_t& lane_info_raw, const message::map_map::s_LaneInfo_t&lane_info_side, 
                                               uint32_t link_id, uint8_t& raw_split, uint8_t& side_split){
    // std::cout <<"ModifySplitAttribute: "<<std::endl;
    message::map_map::s_LaneInfo_t left_lane{};
    message::map_map::s_LaneInfo_t right_lane{};
    int dir = 0;
    if(lane_info_raw.LaneNum.LaneNum < lane_info_side.LaneNum.LaneNum){
        right_lane = lane_info_raw;
        left_lane = lane_info_side;
        dir = 1;
    }else if(lane_info_raw.LaneNum.LaneNum > lane_info_side.LaneNum.LaneNum){
        left_lane = lane_info_raw;
        right_lane = lane_info_side;
        dir =2;
    }else{
        return false;
    }
#ifdef MODIFY_ATTRIBUTE    
    std::cout<<"lane_info_raw.trans: "<< (int)lane_info_raw.Transit.data<<" ,lane_info_raw.lane_num:"<<(int)lane_info_raw.LaneNum.LaneNum<<std::endl;
    std::cout<<"lane_info_side.trans: "<< (int)lane_info_side.Transit.data<<" ,lane_info_side.lane_num:"<<(int)lane_info_side.LaneNum.LaneNum<<std::endl;
#endif
    double acumulate_heading_ri_lane_center = 0;
    double acumulate_heading_le_lane_center = 0;
    double aver_heading_ri_lane_center = 0;
    double aver_heading_le_lane_center = 0;  
#ifdef MODIFY_ATTRIBUTE     
    std::cout<<"link_id: "<< link_id<<std::endl;
#endif  
    {//get right lane center line
        uint32_t line_index = right_lane.Centeline.Centeline;
#ifdef MODIFY_ATTRIBUTE           
        std::cout<<"right_lane.LaneNum.LaneNum: "<< (int)right_lane.LaneNum.LaneNum<<std::endl;
        std::cout<<"right_lane.Centeline.Centeline: "<< right_lane.Centeline.Centeline<<std::endl;
#endif           
        std::vector<message::map_map::s_GeometryPoint_t> geometry_points;
        if (!GetLine(line_index, geometry_points)) {
            // std::cout << "line not found 222" << std::endl;
            return false;
        }
        EFMRefLinePoints ref_line_points{};
        CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(geometry_points, ref_line_points, map_position_); 

        std::vector<double> raw_accumulated_s, raw_kappas, raw_dkappas, raw_headings;
        CommonMathMethod::DiscretePointsMath::ComputePathProfile(ref_line_points, &raw_headings, &raw_accumulated_s,
                                                            &raw_kappas, &raw_dkappas);   
        // std::cout<<" right lane left line ,raw_headings.size(): "<<raw_headings.size()<<std::endl;                                                    
        if(raw_headings.size()>0){
             for(int i =0;i<raw_headings.size();i++){
                acumulate_heading_ri_lane_center += raw_headings[i];
             }
              aver_heading_ri_lane_center = acumulate_heading_ri_lane_center/(raw_headings.size());
        }   
    }  
    {//get left lane center line
        uint32_t line_index = left_lane.Centeline.Centeline;
#ifdef MODIFY_ATTRIBUTE           
        std::cout<<"left_lane.LaneNum.LaneNum: "<< (int)left_lane.LaneNum.LaneNum<<std::endl;
        std::cout<<"left_lane.Centeline.Centeline: "<< left_lane.Centeline.Centeline<<std::endl;
#endif        
        std::vector<message::map_map::s_GeometryPoint_t> geometry_points;
        if (!GetLine(line_index, geometry_points)) {
            // std::cout << "line not found 222" << std::endl;
            return false;
        }
        EFMRefLinePoints ref_line_points{};
        CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(geometry_points, ref_line_points, map_position_); 

        std::vector<double> raw_accumulated_s, raw_kappas, raw_dkappas, raw_headings;
        CommonMathMethod::DiscretePointsMath::ComputePathProfile(ref_line_points, &raw_headings, &raw_accumulated_s,
                                                            &raw_kappas, &raw_dkappas);   
        // std::cout<<"  left lane right line ,raw_headings.size(): "<<raw_headings.size()<<std::endl;  
        if(raw_headings.size()>0){
             for(int i =0;i<raw_headings.size();i++){
                acumulate_heading_le_lane_center += raw_headings[i];
             }
              aver_heading_le_lane_center = acumulate_heading_le_lane_center/(raw_headings.size());
        }   
    } 

    double acumulate_heading_next_lane = 0;
    double aver_heading_next_lane = 0;  
    double acumulate_heading_last_lane = 0;
    double aver_heading_last_lane = 0;    
    bool found_lanes = false; //next 是不是同一个link，不是false        
    {//get next lane common line
        std::vector<std::pair<uint8_t,uint32_t>> ri_lane_next_lane_id_vec{};
        std::vector<std::pair<uint8_t,uint32_t>> le_lane_next_lane_id_vec{};
        std::vector<int> link_staticmap_indices{};
        if (link_id_index_lane_connect_map_->find(link_id) != link_id_index_lane_connect_map_->end()) {
            link_staticmap_indices = link_id_index_lane_connect_map_->at(link_id);
        }
        if(link_staticmap_indices.empty()) {
            return false;
        } 
        uint32_t next_link_id = 0;
        for(auto index : link_staticmap_indices){
            if(map_static_info_->LaneConnectivitys.PairConnectivity[index].FromLinkId.FromLinkId == link_id && map_static_info_->LaneConnectivitys.PairConnectivity[index].PathId.PathId == map_position_->PathId){
                next_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[index].ToLinkId.ToLink;
                if(map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum == right_lane.LaneNum.LaneNum){
                    ri_lane_next_lane_id_vec.push_back(std::make_pair(map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum,next_link_id));
                }else if(map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum == left_lane.LaneNum.LaneNum){
                    le_lane_next_lane_id_vec.push_back(std::make_pair(map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum,next_link_id));
                }
            }
        }

        message::map_map::s_LaneInfo_t left_lane_next_lane{};
        left_lane_next_lane.LaneNum.LaneNum = 0;
        message::map_map::s_LaneInfo_t right_lane_next_lane{}; 
        right_lane_next_lane.LaneNum.LaneNum = 0;   
        for(auto le_id:le_lane_next_lane_id_vec){
            message::map_map::s_LinkInfo_t next_link_infos{};
            if(!GetLinkInfos(le_id.second, next_link_infos)){
                continue;
            }

            for(auto ri_id:ri_lane_next_lane_id_vec){
                if(le_id.first == (ri_id.first+1) && ri_id.second == le_id.second){
                    for(auto lane:next_link_infos.LaneInfos.LaneInfos){
                        if(lane.LaneNum.LaneNum == le_id.first){
                            left_lane_next_lane = lane;
                        }
                        if(lane.LaneNum.LaneNum == ri_id.first){
                            right_lane_next_lane = lane;
                        }
                        if(right_lane_next_lane.LaneNum.LaneNum !=0 && left_lane_next_lane.LaneNum.LaneNum !=0){
                            found_lanes =true;
                            break;
                        }
                    }
                }
                if(found_lanes == true){
                    break;
                }
            }
            if(found_lanes == true){
                break;
            }    
        }
#ifdef MODIFY_ATTRIBUTE           
        std::cout<<"found_lanes: "<< found_lanes<<std::endl;
        std::cout<<"le_lane_next_lane_id_vec: ";
        for(auto le_id:le_lane_next_lane_id_vec){
            std::cout<<" ,< lane_num: "<<(int)le_id.first<<" ,link: "<<le_id.second<<">";
        }
        std::cout<<std::endl;
        std::cout<<"ri_lane_next_lane_id_vec: ";
        for(auto le_id:ri_lane_next_lane_id_vec){
            std::cout<<" ,< lane_num: "<<(int)le_id.first<<" ,link: "<<le_id.second<<">";
        }
        std::cout<<std::endl;        
        std::cout<<"left_lane_next_lane.LaneNum.LaneNum: "<< (int)left_lane_next_lane.LaneNum.LaneNum<<std::endl;
        std::cout<<"right_lane_next_lane.LaneNum.LaneNum: "<< (int)right_lane_next_lane.LaneNum.LaneNum<<std::endl;
#endif        
        // get one common line
        if(found_lanes == true){
            uint32_t line_index = left_lane_next_lane.RBound.RBound;
            // std::cout<<"left_lane_next_lane.RBound.RBound: "<< left_lane_next_lane.RBound.RBound<<std::endl;
            std::vector<message::map_map::s_GeometryPoint_t> geometry_points;
            if (!GetLine(line_index, geometry_points)) {
                // std::cout << "line not found 222" << std::endl;
                return false;
            }
            EFMRefLinePoints ref_line_points{};
            CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(geometry_points, ref_line_points, map_position_);

            std::vector<double> raw_accumulated_s, raw_kappas, raw_dkappas, raw_headings;
            CommonMathMethod::DiscretePointsMath::ComputePathProfile(ref_line_points, &raw_headings, &raw_accumulated_s,
                                                                     &raw_kappas, &raw_dkappas);
            // std::cout<<"  next lane common line ,raw_headings.size(): "<<raw_headings.size()<<std::endl; 
            if (raw_headings.size() > 0) {
                for (int i = 0; i < raw_headings.size(); i++) {
                    acumulate_heading_next_lane += raw_headings[i];
                }
                aver_heading_next_lane = acumulate_heading_next_lane / (raw_headings.size());
            }
        }else{
            //find last link, and last lane
            std::vector<int> link_staticmap_indices{};
            if (to_link_id_index_lane_connect_map_->find(link_id) != to_link_id_index_lane_connect_map_->end()) {
                link_staticmap_indices = to_link_id_index_lane_connect_map_->at(link_id);
            }
            if(link_staticmap_indices.empty()) {
                return false;
            } 
            uint32_t last_link_id = 0;
            uint8_t last_lane_num = 0;
            for(auto index : link_staticmap_indices){
                if(map_static_info_->LaneConnectivitys.PairConnectivity[index].ToLinkId.ToLink == link_id && map_static_info_->LaneConnectivitys.PairConnectivity[index].PathId.PathId == map_position_->PathId){
                    last_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[index].FromLinkId.FromLinkId;
                    if(map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum == right_lane.LaneNum.LaneNum){
                        last_lane_num = map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum;
                        break;
                    }
                }
            }

            message::map_map::s_LinkInfo_t last_link_infos;
            if(!GetLinkInfos(last_link_id, last_link_infos)){
                return false;
            }
            uint32_t line_index = 0;
            for(auto lane: last_link_infos.LaneInfos.LaneInfos){
                if(lane.LaneNum.LaneNum == last_lane_num){
                     line_index = lane.Centeline.Centeline;
                }
            }
#ifdef MODIFY_ATTRIBUTE               
            std::cout<<"last_lane.LaneNum.LaneNum: "<< (int)last_lane_num<<std::endl;
            std::cout<<"last_lane.Centeline.Centeline: "<< line_index<<std::endl;
#endif            
            std::vector<message::map_map::s_GeometryPoint_t> geometry_points;
            if (!GetLine(line_index, geometry_points)) {
                // std::cout << "line not found 222" << std::endl;
                return false;
            }
            EFMRefLinePoints ref_line_points{};
            CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(geometry_points, ref_line_points, map_position_); 

            std::vector<double> raw_accumulated_s, raw_kappas, raw_dkappas, raw_headings;
            CommonMathMethod::DiscretePointsMath::ComputePathProfile(ref_line_points, &raw_headings, &raw_accumulated_s,
                                                            &raw_kappas, &raw_dkappas);   
            // std::cout<<"  left lane right line ,raw_headings.size(): "<<raw_headings.size()<<std::endl;  
            if(raw_headings.size()>0){
                 for(int i =0;i<raw_headings.size();i++){
                     acumulate_heading_last_lane += raw_headings[i];
                 }
                  aver_heading_last_lane = acumulate_heading_last_lane/(raw_headings.size());
            }              
        }
    }
#ifdef MODIFY_ATTRIBUTE   
    std::cout<<"split acumulate_heading_ri_lane_center: "<< acumulate_heading_ri_lane_center<<std::endl;
    std::cout<<"split aver_heading_ri_lane_center: "<< aver_heading_ri_lane_center<<std::endl;
    std::cout<<"split acumulate_heading_le_lane_center: "<< acumulate_heading_le_lane_center<<std::endl;
    std::cout<<"split aver_heading_le_lane_center: "<< aver_heading_le_lane_center<<std::endl;
    std::cout<<"split acumulate_heading_next_lane: "<< acumulate_heading_next_lane<<std::endl;
    std::cout<<"split aver_heading_next_lane: "<< aver_heading_next_lane<<std::endl; 
    std::cout<<"split acumulate_heading_last_lane: "<< acumulate_heading_last_lane<<std::endl;
    std::cout<<"split aver_heading_last_lane: "<< aver_heading_last_lane<<std::endl; 
#endif    
    //计算平均heading，判断哪个直
    double ri_heading_cost = 0;
    double le_heading_cost = 0;
    if(found_lanes == true){
        ri_heading_cost = abs(aver_heading_next_lane - aver_heading_ri_lane_center);
        le_heading_cost = abs(aver_heading_next_lane - aver_heading_le_lane_center);   
    }else{
        ri_heading_cost = abs(aver_heading_last_lane - aver_heading_ri_lane_center);
        le_heading_cost = abs(aver_heading_last_lane - aver_heading_le_lane_center);         
    }
#ifdef MODIFY_ATTRIBUTE       
    std::cout<<"split ri_heading_cost: "<< ri_heading_cost<<std::endl;
    std::cout<<"split le_heading_cost: "<< le_heading_cost<<std::endl; 
    std::cout<<"p_heading_for_split_first_level: "<< p_heading_for_merge_first_level<<std::endl; 
    std::cout<<"p_heading_for_split_sec_level: "<< p_heading_for_merge_sec_level<<std::endl;
#endif    
    uint8_t ri_split = right_lane.Transit.data;
    uint8_t le_split = left_lane.Transit.data;
    if(ri_heading_cost>p_heading_for_merge_first_level && le_heading_cost> p_heading_for_merge_first_level){
       //do nothing    
    }else{
        if(ri_heading_cost - le_heading_cost > p_heading_for_merge_sec_level){
            ri_split = 3;
            le_split = 1;
        }else if(le_heading_cost - ri_heading_cost > p_heading_for_merge_sec_level){
            ri_split = 1;
            le_split = 3; 
        }
    }
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"ri_split: "<< (int)ri_split<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"le_split: "<< (int)le_split<<std::endl;    
    // std::cout<<"dir: "<< dir<<std::endl;
    if(dir == 1){
        raw_split = ri_split;
        side_split = le_split;
    }else if(dir ==2){
        raw_split = le_split;
        side_split = ri_split;
    }
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"raw_split: "<< (int)raw_split<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"side_split: "<< (int)side_split<<std::endl;
    // std::cout <<"ModifySplitAttribute end!!!: "<<std::endl; 
    return true;
}

bool CandidateLanesModel::StoreModifyedLaneAttribute(std::map<uint64_t,uint8_t>& map_res, std::deque<uint64_t>& deque_res, uint8_t attribute_val, uint8_t lane_id, uint32_t link_id){
    uint64_t key_temp = ((static_cast<uint64_t>(lane_id)) <<32) | (static_cast<uint64_t>(link_id));
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"StoreModifyedLaneAttribute: "<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"link_id: "<< link_id<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"lane_id: "<< (int)lane_id<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"key_temp: "<< key_temp<<std::endl;
    StoreStaticMapAndDeque(map_res, deque_res, key_temp, attribute_val, 100);
    return true;
}

bool CandidateLanesModel::GetModifyedLaneAttribute(const std::map<uint64_t,uint8_t>& map_res, const std::deque<uint64_t>& deque_res, uint8_t lane_id, uint32_t link_id,uint8_t& attribute_val){
    //查找元素
    uint64_t key_temp = ((static_cast<uint64_t>(lane_id)) <<32) | (static_cast<uint64_t>(link_id));
    bool res_b = GetStaticMapAndDeque(map_res,deque_res,key_temp,attribute_val);
    return res_b;
}

bool CandidateLanesModel::GetStoredSplitInfo(const std::map<uint64_t,SplitInfo_S>& map_res, const std::deque<uint64_t>& deque_res, uint8_t lane_id, uint32_t link_id, SplitInfo_S& split_info){
    //查找元素
    uint64_t key_temp = ((static_cast<uint64_t>(lane_id)) <<32) | (static_cast<uint64_t>(link_id));
    bool res_b = GetStaticMapAndDeque(map_res,deque_res,key_temp,split_info);
    return res_b;
}

bool CandidateLanesModel::ChangeStoredSplitInfo(std::map<uint64_t,SplitInfo_S>& map_res, std::deque<uint64_t>& deque_res, uint8_t lane_id, uint32_t link_id, SplitInfo_S split_info){
    //查找元素
    uint64_t key_temp = ((static_cast<uint64_t>(lane_id)) <<32) | (static_cast<uint64_t>(link_id));
    bool res_b = ChangeStaticMapAndDeque(map_res, deque_res, key_temp, split_info);
    return res_b;
}

bool CandidateLanesModel::GetBackLinkLane(uint32_t link_id, uint8_t lane_id, uint32_t& back_link_id,
                                       uint8_t& back_lane_id) {
    back_link_id = 0;
    back_lane_id = 0;
    int link_idx = -1;

    if (to_link_id_index_lane_connect_map_->find(link_id) != to_link_id_index_lane_connect_map_->end()) {
        uint32_t temp_link_id = 0;
        for (auto idx : to_link_id_index_lane_connect_map_->at(link_id)) {
            uint32_t temp_back_link_id =
                map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId;
            uint8_t temp_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].NewLaneNum.NewLaneNum;
            if (map_static_info_->LaneConnectivitys.PairConnectivity[idx].InitPath.InitPath == map_position_->PathId &&
                lane_id == temp_lane_id) {
                back_lane_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].InitLaneNum.InitLaneNum;
                back_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId;
                break;
            }
        }
    }

    if (0 == back_link_id || 0 == back_lane_id) {
        return false;
    }

    return true;
}

void CandidateLanesModel::GetMergeBackLinkLane(uint32_t& link_id, uint8_t& ego_lane_num, uint8_t& side_lane_num){
    /*获取自车前方50m内，自车道或旁车道存在merge的link和merge的两个lane_id*/
    uint8_t ego_lane_id = ego_lane_num;
    uint8_t side_lane_id = side_lane_num;
    uint32_t cur_link_id = link_id;
    bool is_left = false;
    merge_back_links_id_.clear();
    merge_ego_back_lanes_id_.clear();
    merge_side_back_lanes_id_.clear();
    message::map_map::s_LinkInfo_t link_infos{};
    if (!GetLinkInfos(cur_link_id, link_infos)) {
        return;
    }
    if(link_infos.EndOffset.EndOffset < map_position_->PathOffset + 50 * 100){
        // 保存merge且双侧虚拟车道存在的link和lane_id
        merge_back_links_id_.emplace_back(link_id);
        merge_ego_back_lanes_id_.emplace_back(ego_lane_id);
        merge_side_back_lanes_id_.emplace_back(side_lane_id);
    }else{
        return ;
    }

    uint32_t lanes_length = 0;
    while (lanes_length < 50 * 100) {
        // 保存merge后方50m内双侧虚拟车道的link和lane_id
        uint32_t ego_back_link_id = 0, side_back_link_id = 0;
        uint8_t ego_back_lane_id = 0, side_back_lane_id = 0;
         message::map_map::s_LaneInfo_t lane_info_raw{}, lane_info_side{};
         message::map_map::s_LinkInfo_t link_infos_raw{}, link_infos_side{};
        GetBackLinkLane(cur_link_id, ego_lane_id, ego_back_link_id, ego_back_lane_id);
        GetBackLinkLane(cur_link_id, side_lane_id, side_back_link_id, side_back_lane_id);
        if(ego_back_lane_id > side_back_lane_id){
            is_left = true;
        }else{
            is_left = false;
        }
        if (!GetLinkInfos(ego_back_link_id, link_infos_raw)) {
        return;
        }
        for(const auto& laneinfo : link_infos_raw.LaneInfos.LaneInfos){
            if(laneinfo.LaneNum.LaneNum == ego_back_lane_id){
                lane_info_raw = laneinfo;
            }
            else if(laneinfo.LaneNum.LaneNum == side_back_lane_id){
                lane_info_side = laneinfo;
            }

        }
        if(ego_back_link_id == side_back_link_id && IsVirtually(lane_info_raw, lane_info_side, is_left)){
            merge_back_links_id_.emplace_back(ego_back_link_id);
            merge_ego_back_lanes_id_.emplace_back(ego_back_lane_id);
            merge_side_back_lanes_id_.emplace_back(side_back_lane_id);
        }else{break;}
        
        message::map_map::s_LinkInfo_t back_link_infos;
        if (false == GetLinkInfos(ego_back_link_id, back_link_infos)) {
            // static data none
            break;
        }
        cur_link_id = ego_back_link_id;
        ego_lane_id = ego_back_lane_id;
        side_lane_id = side_back_lane_id;
        lanes_length += back_link_infos.EndOffset.EndOffset - back_link_infos.PathOffset.PathOffset;
    }
    return ;
}

void CandidateLanesModel::InitGlobalVal(){
      split_lind_id_lane_id_map.clear();
    merge_lind_id_lane_id_map.clear();//key: linkid低32位 laneid 高32位； 保存的是修改了split属性的属性
    split_deque.clear();//first-保存的所map的key; 
    merge_deque.clear();//first-保存的所map的key;
    split_info_map.clear(); //key:linkid低32位 laneid 高32位； 
    split_info_deque.clear(); //和split_info_map配套，保存的是split_info_map的key； 
    drive_lind_id_lane_id_map.clear();//key: linkid低32位 laneid 高32位； 保存的是自车走过的link id和link id; second-是path id
    drive_deque.clear();

    return;
}

bool CandidateLanesModel::get_toll_link() {
    if (nullptr == map_static_info_) {
        return true;
    }

    for (auto& geoface : map_static_info_->GeoFences.GeoFences) {
        if (4 == geoface.GeoFenceType.data) {
            // Tollgate_ = 4
            if (toll_link_id_set_.find(geoface.InstanceId_.InstanceId) == toll_link_id_set_.end()) {
                toll_link_id_set_.insert(geoface.InstanceId_.InstanceId);
            }
        }
    }

    return true;
}

double CandidateLanesModel::HeadingTransform(double in){
    // atans [-pi, pi] -> [0,2pi]
    if(in < 0){
        return 6.28 + in;
    }else{
        return in;
    }

}

bool CandidateLanesModel::IsRoadMerge(const uint32_t link_id, const uint8_t lane_num_raw, const uint8_t lane_num_side, uint32_t ego_offset, uint32_t ego_path_id,
                                       int32_t& merge_start_offset, int32_t& merge_end_offset){
    //向车屁股方向找，找到自车offset位置，如果是road merge, 在自车前方的start_offset 发实际值，自车过了start ,start发0， 
    //道路的merge，需要merge的两条路最终不同path id,
    //回找遇到连续merge的，只看continue一条！！！！！
    merge_start_offset = 0;
    merge_end_offset = 0;
    if(ego_offset<0){
        return false;
    }
    uint32_t path_offset = 0;
    uint32_t while_link_id_raw = link_id;
    uint32_t while_link_id_side = link_id;
    uint8_t while_lane_num_raw = lane_num_raw;
    uint8_t while_lane_num_side = lane_num_side;
    uint32_t while_path_offset_raw = 0;
    uint32_t while_path_offset_side = 0;
    int while_counter = 0;
    message::map_map::s_LinkInfo_t link_infos{};
    if(efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_, *link_id_index_lane_info_map_, link_id, link_infos)){
        path_offset = link_infos.PathOffset.PathOffset;
        while_path_offset_raw = path_offset;
        while_path_offset_side = path_offset;
        merge_end_offset = link_infos.EndOffset.EndOffset - ego_offset;
    }else{
        return false;
    }
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"ego_offset: "<<ego_offset<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"merge_end_offset: "<<merge_end_offset<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"link_id: "<<link_id<<" ,lane_num_raw: "<<(int)lane_num_raw<<" ,lane_num_side: "<<(int)lane_num_side<<std::endl;
    while(path_offset > ego_offset && while_link_id_raw == while_link_id_side && while_counter<20){
        // std::cout << __FILE__ << "," << __LINE__ << ","<<"while_counter: "<<while_counter<<" ,while_link_id_raw: "<<while_link_id_raw<<" ,while_link_id_side: "<<while_link_id_side
        //                      <<" ,while_lane_num_raw: "<<(int)while_lane_num_raw<<" ,while_lane_num_side: "<<(int)while_lane_num_side<<std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << ","<<"path_offset: "<<path_offset<<std::endl;
        std::vector<message::map_map::s_PairConnectivity_t> lane_Connectivitys_raw{};
        std::vector<message::map_map::s_PairConnectivity_t> lane_Connectivitys_side{};
        uint32_t path_id_raw = 0;
        uint32_t path_id_side = 0;
        bool find_lane_raw = false;
        bool find_lane_side = false;
        //找一条车道
        if(efm::MapCommonTool::GetInstance()->GetLaneToConnectivitys(map_static_info_,to_link_id_index_lane_connect_map_,while_link_id_raw,while_lane_num_raw,lane_Connectivitys_raw)){
            for(auto connect:lane_Connectivitys_raw){
                message::map_map::s_LinkInfo_t link_infos{};
                if(efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_, *link_id_index_lane_info_map_, connect.FromLinkId.FromLinkId, link_infos)){
                    message::map_map::s_LaneInfo_t lane_info{};
                    if(efm::MapCommonTool::GetInstance()->GetLaneInfo(link_infos, connect.InitLaneNum.InitLaneNum, lane_info)){
                        find_lane_raw = true;
                        if(lane_info.Transit.data == 1){
                            while_lane_num_raw = lane_info.LaneNum.LaneNum;
                            while_link_id_raw = connect.FromLinkId.FromLinkId;
                            path_id_raw = link_infos.PathId.PathId;
                            while_path_offset_raw = link_infos.PathOffset.PathOffset;
                            break;
                        }
                        //没有continue的话，就随便找一条
                        while_lane_num_raw = lane_info.LaneNum.LaneNum;
                        while_link_id_raw = connect.FromLinkId.FromLinkId;
                        path_id_raw = link_infos.PathId.PathId;
                        while_path_offset_raw = link_infos.PathOffset.PathOffset;
                    }
                }
            }
        }
        if(find_lane_raw == false){
            return false;
        }
        //找另一条车道
        if(efm::MapCommonTool::GetInstance()->GetLaneToConnectivitys(map_static_info_,to_link_id_index_lane_connect_map_,while_link_id_side,while_lane_num_side,lane_Connectivitys_side)){
            for(auto connect:lane_Connectivitys_side){
                message::map_map::s_LinkInfo_t link_infos{};
                if(efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_, *link_id_index_lane_info_map_, connect.FromLinkId.FromLinkId, link_infos)){
                    message::map_map::s_LaneInfo_t lane_info{};
                    if(efm::MapCommonTool::GetInstance()->GetLaneInfo(link_infos, connect.InitLaneNum.InitLaneNum, lane_info)){
                        find_lane_side = true;
                        if(lane_info.Transit.data == 1){
                            while_lane_num_side = lane_info.LaneNum.LaneNum;
                            while_link_id_side = connect.FromLinkId.FromLinkId;
                            path_id_side = link_infos.PathId.PathId;
                            while_path_offset_side = link_infos.PathOffset.PathOffset;
                            break;
                        }
                        //没有continue的话，就随便找一条
                        while_lane_num_side = lane_info.LaneNum.LaneNum;
                        while_link_id_side = connect.FromLinkId.FromLinkId;
                        path_id_side = link_infos.PathId.PathId;
                        while_path_offset_side = link_infos.PathOffset.PathOffset;
                    }
                }
            }
        }
        if(find_lane_side == false){
            return false;
        }

        if(path_id_side != path_id_raw || while_link_id_raw != while_link_id_side || while_path_offset_side != while_path_offset_raw){
        // std::cout << __FILE__ << "," << __LINE__ << ","<<"path_id_side: "<<path_id_side<<" ,path_id_raw: "<<path_id_raw<<" ,while_link_id_side: "<<while_link_id_side
        //                      <<" ,while_link_id_raw: "<<while_link_id_raw<<std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << ","<<"path_offset: "<<path_offset<<" ,while_path_offset_side:"<<while_path_offset_side<<" ,while_path_offset_raw:"<<while_path_offset_raw<<std::endl;
            merge_start_offset = path_offset - ego_offset;
            return true;
        }
        path_offset = while_path_offset_raw;
        while_counter++;        
    }    
    return true;    
}

bool CandidateLanesModel::StoreSplitInfo(std::map<uint64_t,SplitInfo_S>& map_res, std::deque<uint64_t>& deque_res, SplitInfo_S split_info, uint8_t lane_id, uint32_t link_id){
    uint64_t key_temp = ((static_cast<uint64_t>(lane_id)) <<32) | (static_cast<uint64_t>(link_id));
    return StoreStaticMapAndDeque(map_res, deque_res, key_temp, split_info, 200);
}

bool CandidateLanesModel::IsRoadSplit(const uint32_t link_id, const uint8_t lane_num_raw, const uint8_t lane_num_side, uint32_t ego_path_id,
                                       std::vector<std::pair<uint64_t,SplitInfo_S>>& split_info_vec_raw, std::vector<std::pair<uint64_t,SplitInfo_S>>& split_info_vec_side){
// 如果是road split, 那么之间的lane 的split info的s e offset 都是road split的
// 如果不是road split, 那么只存split start offset 前方200m的lane
// 如果有连续的分歧，那么第二个分歧之后的lane 不存储                                   
//判断是否有道路分歧，分歧点向自车行驶方向搜索，两条路最终到不同的link或path,就认为是道路分歧
// split_info_vec_raw ; fisrt 是低32位linkId, 高32是laneId; second是
    split_info_vec_raw.clear();
    split_info_vec_side.clear();
    uint32_t split_start_offset = 0;
    uint32_t split_end_offset = 0;
    uint32_t end_offset = 0;
    uint32_t while_link_id_raw = link_id;
    uint32_t while_link_id_side = link_id;
    uint8_t while_lane_num_raw = lane_num_raw;
    uint8_t while_lane_num_side = lane_num_side;
    uint32_t while_path_id_raw = 0;
    uint32_t while_path_id_side = 0;
    uint32_t while_end_offset_raw = 0;
    uint32_t while_end_offset_side = 0;
    int while_counter = 0;
    bool continuous_split_f_raw = false; //连续分歧的，那么连续分歧之后的道路都不保存
    bool continuous_split_f_side = false;
    bool is_road_split = false;
    message::map_map::s_LinkInfo_t link_infos{};
    if(efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_, *link_id_index_lane_info_map_, link_id, link_infos)){
        end_offset = link_infos.EndOffset.EndOffset;
        while_path_id_raw = link_infos.PathId.PathId;
        while_path_id_side = link_infos.PathId.PathId;
        split_start_offset = link_infos.PathOffset.PathOffset;
        SplitInfo_S split_info_raw(0,split_start_offset,end_offset,EFM_SplitType_NONE);//先保存第一个
        uint64_t key_temp_raw = ((static_cast<uint64_t>(lane_num_raw)) <<32) | (static_cast<uint64_t>(link_id));
        uint64_t key_temp_side = ((static_cast<uint64_t>(lane_num_side)) <<32) | (static_cast<uint64_t>(link_id));
        split_info_vec_raw.push_back(std::make_pair(key_temp_raw,split_info_raw));
        split_info_vec_side.push_back(std::make_pair(key_temp_side,split_info_raw));
    }else{
        return false;
    }

    while(while_path_id_raw == while_path_id_side && while_link_id_raw == while_link_id_side &&  while_counter<100){
        //向自车行驶方向找
#ifdef SPLIT_0930
        std::cout << __FILE__ << "," << __LINE__ << ","<<"while_counter: "<<while_counter<<" ,while_link_id_raw: "<<while_link_id_raw<<" ,while_link_id_side: "<<while_link_id_side
                             <<" ,while_lane_num_raw: "<<(int)while_lane_num_raw<<" ,while_lane_num_side: "<<(int)while_lane_num_side<<std::endl;
        std::cout << __FILE__ << "," << __LINE__ << ","<<"end_offset: "<<end_offset<<std::endl;
#endif
        std::vector<message::map_map::s_PairConnectivity_t> lane_Connectivitys_raw{};
        std::vector<message::map_map::s_PairConnectivity_t> lane_Connectivitys_side{};
        bool find_lane_raw = false;
        bool find_lane_side = false;
        //找一条车道
        if(efm::MapCommonTool::GetInstance()->GetLaneFromConnectivitys(map_static_info_,link_id_index_lane_connect_map_,while_link_id_raw,while_lane_num_raw,lane_Connectivitys_raw)){
            int connect_lane_size_raw = 0;
            bool find_continue_lane = false;
            for(auto connect:lane_Connectivitys_raw){
                message::map_map::s_LinkInfo_t link_infos{};
                if(efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_, *link_id_index_lane_info_map_, connect.ToLinkId.ToLink, link_infos)){
                    message::map_map::s_LaneInfo_t lane_info{};
                    if(efm::MapCommonTool::GetInstance()->GetLaneInfo(link_infos, connect.NewLaneNum.NewLaneNum, lane_info)){
                        find_lane_raw = true;
                        connect_lane_size_raw++;
                        if(connect_lane_size_raw > 1){
                            continuous_split_f_raw = true;
                        }
                        if(lane_info.Transit.data == 1){
                            while_lane_num_raw = lane_info.LaneNum.LaneNum;
                            while_link_id_raw = connect.ToLinkId.ToLink;
                            while_path_id_raw = link_infos.PathId.PathId;
                            while_end_offset_raw = link_infos.EndOffset.EndOffset;
                            find_continue_lane = true;
                        }else if(find_continue_lane == false){
                            //没有continue的话，就随便找一条
                            while_lane_num_raw = lane_info.LaneNum.LaneNum;
                            while_link_id_raw = connect.ToLinkId.ToLink;
                            while_path_id_raw = link_infos.PathId.PathId;
                            while_end_offset_raw = link_infos.EndOffset.EndOffset;                            
                        }

                    }
                }
            }
        }
        if(find_lane_raw == false){
            break;
        }
        //找另一条车道
        if(efm::MapCommonTool::GetInstance()->GetLaneFromConnectivitys(map_static_info_,link_id_index_lane_connect_map_,while_link_id_side,while_lane_num_side,lane_Connectivitys_side)){
            int connect_lane_size_side = 0;
            bool find_continue_lane = false;            
            for(auto connect:lane_Connectivitys_side){
                message::map_map::s_LinkInfo_t link_infos{};
                if(efm::MapCommonTool::GetInstance()->GetLinkInfos(*map_static_info_, *link_id_index_lane_info_map_, connect.ToLinkId.ToLink, link_infos)){
                    message::map_map::s_LaneInfo_t lane_info{};
                    if(efm::MapCommonTool::GetInstance()->GetLaneInfo(link_infos, connect.NewLaneNum.NewLaneNum, lane_info)){
                        find_lane_side = true;
                        connect_lane_size_side++;
                        if(connect_lane_size_side > 1){
                            continuous_split_f_side = true;
                        }
                        if(lane_info.Transit.data == 1){
                            while_lane_num_side = lane_info.LaneNum.LaneNum;
                            while_link_id_side = connect.ToLinkId.ToLink;
                            while_path_id_side = link_infos.PathId.PathId;
                            while_end_offset_side = link_infos.EndOffset.EndOffset;
                            find_continue_lane = true;
                        }else if(find_continue_lane == false){
                        //没有continue的话，就随便找一条
                            while_lane_num_side = lane_info.LaneNum.LaneNum;
                            while_link_id_side = connect.ToLinkId.ToLink;
                            while_path_id_side = link_infos.PathId.PathId;
                            while_end_offset_side = link_infos.EndOffset.EndOffset;                            
                        }

                    }
                }
            }
        }
        if(find_lane_side == false){
            break;
        }
        if(while_lane_num_raw == while_lane_num_side && while_link_id_raw == while_link_id_side){
            split_end_offset = end_offset;
            //merge 了
            break;
        }else if(while_path_id_side != while_path_id_raw || while_link_id_raw != while_link_id_side){
#ifdef SPLIT_0930
        std::cout << __FILE__ << "," << __LINE__ << ","<<"path_id_side: "<<while_path_id_side<<" ,path_id_raw: "<<while_path_id_raw<<" ,while_link_id_side: "<<while_link_id_side
                             <<" ,while_link_id_raw: "<<while_link_id_raw<<std::endl;
        std::cout << __FILE__ << "," << __LINE__ << ","<<"end_offset: "<<end_offset<<std::endl;
        std::cout << __FILE__ << "," << __LINE__ << ","<<"IsRoadSplit  ending yes!!!: " <<std::endl; 
#endif
            split_end_offset = end_offset;
            is_road_split = true;
            break;
        }else{
            end_offset = while_end_offset_raw;
            while_counter++;   
            //保存信息, 连续分歧的不保存，只保存到连续分歧前
            if(continuous_split_f_raw == false){
                SplitInfo_S split_info(0,split_start_offset,end_offset,EFM_SplitType_NONE);
                uint64_t key_temp = ((static_cast<uint64_t>(while_lane_num_raw)) <<32) | (static_cast<uint64_t>(while_link_id_raw));
                split_info_vec_raw.push_back(std::make_pair(key_temp,split_info));
            }
            if(continuous_split_f_side == false){
                SplitInfo_S split_info(0,split_start_offset,end_offset,EFM_SplitType_NONE);
                uint64_t key_temp = ((static_cast<uint64_t>(while_lane_num_side)) <<32) | (static_cast<uint64_t>(while_link_id_side));
                split_info_vec_side.push_back(std::make_pair(key_temp,split_info));
            }          
        }
       
    } 
#ifdef SPLIT_0930
    std::cout << __FILE__ << "," << __LINE__ << ","<< "ori split_info_vec_raw: ";
    for(auto iter:split_info_vec_raw){
       std::cout<<" ,<"<<(iter.first & (uint64_t)4294967295)<<", "<<((iter.first >> 32) & (((uint64_t)4294967295)))<<",|| "<<iter.second.is_road_split<<", "<<iter.second.s_offset<<", "<<iter.second.e_offset<<", "<<(int)iter.second.split_dir<<">";
    }
    std::cout <<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<< "ori split_info_vec_side: ";
    for(auto iter:split_info_vec_side){
       std::cout<<" ,<"<<(iter.first & (uint64_t)4294967295)<<", "<<((iter.first >> 32) & (((uint64_t)4294967295)))<<",|| "<<iter.second.is_road_split<<", "<<iter.second.s_offset<<", "<<iter.second.e_offset<<", "<<(int)iter.second.split_dir<<">";
    }
    std::cout <<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<< " is_road_split: "<< is_road_split<<std::endl;
#endif
    //对保存的信息进行修正
    if(is_road_split){
        for(auto& info: split_info_vec_raw){
            info.second.is_road_split = true;
            info.second.e_offset = split_end_offset;
        }
        for(auto& info: split_info_vec_side){
            info.second.is_road_split = true;
            info.second.e_offset = split_end_offset;
        }
    }else{
        //只要split start点后200m的split
        std::vector<std::pair<uint64_t,SplitInfo_S>> split_info_vec_tmp{};
        for(auto info : split_info_vec_raw){
            split_info_vec_tmp.push_back(info);
            split_info_vec_tmp.back().second.e_offset = split_start_offset+20000;
            if(info.second.e_offset >= split_start_offset+20000){
                
                break;
            }
        }
        split_info_vec_raw.clear();
        split_info_vec_raw = split_info_vec_tmp;

        split_info_vec_tmp.clear();
        for(auto info : split_info_vec_side){
            split_info_vec_tmp.push_back(info);
            if(info.second.e_offset >= split_start_offset+20000){
                split_info_vec_tmp.back().second.e_offset = split_start_offset+20000;
                break;
            }
        }
        split_info_vec_side.clear();
        split_info_vec_side = split_info_vec_tmp;
    }
#ifdef SPLIT_0930
    std::cout << __FILE__ << "," << __LINE__ << ","<< "split_info_vec_raw: ";
    for(auto iter:split_info_vec_raw){
       std::cout<<" ,<"<<(iter.first & (uint64_t)4294967295)<<", "<<((iter.first >> 32) & (((uint64_t)4294967295)))<<",|| "<<iter.second.is_road_split<<", "<<iter.second.s_offset<<", "<<iter.second.e_offset<<", "<<(int)iter.second.split_dir<<">";
    }
    std::cout <<std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","<< "split_info_vec_side: ";
    for(auto iter:split_info_vec_side){
       std::cout<<" ,<"<<(iter.first & (uint64_t)4294967295)<<", "<<((iter.first >> 32) & (((uint64_t)4294967295)))<<",|| "<<iter.second.is_road_split<<", "<<iter.second.s_offset<<", "<<iter.second.e_offset<<", "<<(int)iter.second.split_dir<<">";
    }
    std::cout <<std::endl;
#endif
    return true;
}

template<typename K,typename V>
bool CandidateLanesModel::StoreStaticMapAndDeque(std::map<K,V>& map_res, std::deque<K>& deque_res, K map_key, V map_val, int max_size){
    if(map_res.size()!=deque_res.size()){
        // std::cout << __FILE__ << "," << __LINE__ << ","<<"StoreStaticMapAndDeque size error!!!: "<<std::endl;
        return false;
    }
    if(deque_res.size()>=max_size){
        //删除先入的
        map_res.erase(deque_res.front());
        deque_res.pop_front();
    }
    //加入新元素
    // std::pair<std::map<K,V>::iterator, bool> res{};
    auto res = map_res.emplace(map_key, map_val);
    if(res.second == true){
        deque_res.push_back(map_key);    
    }else{
        return false;
    }

    return true;
}

template<typename K,typename V>
bool CandidateLanesModel::GetStaticMapAndDeque(const std::map<K,V>& map_res, const std::deque<K>& deque_res, K map_key, V& map_val){
    if(map_res.size()!=deque_res.size()){
        return false;
    }
    //查找元素
    if(map_res.find(map_key) !=map_res.end()){
        map_val = map_res.at(map_key);
    }else{
        return false;
    }
    return true;   
}

template<typename K,typename V>
bool CandidateLanesModel::ChangeStaticMapAndDeque(std::map<K,V>& map_res, std::deque<K>& deque_res, K map_key, V change_map_val){
    if(map_res.size()!=deque_res.size()){
        // std::cout << __FILE__ << "," << __LINE__ << ","<<"ChangeStaticMapAndDeque size error!!!: "<<std::endl;
        return false;
    }
    //查找元素,找不到那么就是异常
    auto iter = map_res.find(map_key);
    if(iter == map_res.end()){
        return false;
    }else{
        (*iter).second = change_map_val;
    }

    //要把deque里的值放到back
    for(int i = 0; i< deque_res.size(); i++){
        if(deque_res.at(i) == map_key){
           deque_res.erase(deque_res.begin()+i);
           deque_res.push_back(map_key); 
           break;
        }
    }      
    return true;
}

bool CandidateLanesModel::StoreEgoDrivePathLinkLane(std::map<uint64_t,uint8_t>& map_res, std::deque<uint64_t>& deque_res, uint32_t path_id, uint8_t lane_id, uint32_t link_id){
    if(map_res.size()!=deque_res.size()){
        return false;
    }
    if(deque_res.size()>0){
        //如果最新保存的数据的path id和新来的path id不同，那么就删掉原来保存的所有
        uint64_t key_val = deque_res.back();
        if(map_res.find(key_val) != map_res.end()){
            if(map_res[key_val] != path_id){
                map_res.clear();
                deque_res.clear();
            }
        }
    }
    if(deque_res.size()>=20){
        //删除先入的
        map_res.erase(deque_res.front());
        deque_res.pop_front();
    }
    //加入新元素
    uint64_t key_temp = ((static_cast<uint64_t>(lane_id)) <<32) | (static_cast<uint64_t>(link_id));
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"StoreModifyedLaneAttribute: "<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"link_id: "<< link_id<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"lane_id: "<< (int)lane_id<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","<<"key_temp: "<< key_temp<<std::endl;
    std::pair<std::map<uint64_t,uint8_t>::iterator, bool> res{};
    res = map_res.emplace(key_temp, path_id);
    if(res.second == true){
        deque_res.push_back(key_temp);    
    }

    return true;
}

bool CandidateLanesModel::GetEgoDrivePathLinkLane(const std::map<uint64_t,uint8_t>& map_res, const std::deque<uint64_t>& deque_res, uint8_t lane_id, uint32_t link_id,uint8_t& path_id){
    if(map_res.size()!=deque_res.size()){
        return false;
    }
    //查找元素
    uint64_t key_temp = ((static_cast<uint64_t>(lane_id)) <<32) | (static_cast<uint64_t>(link_id));
    if(map_res.find(key_temp) !=map_res.end()){
        path_id = map_res.at(key_temp);
    }else{
        return false;
    }

    return true;
}

bool CandidateLanesModel::CalculateMergeLinkLineHeading(const message::map_map::s_LaneInfo_t& left_lane_infos, const message::map_map::s_LaneInfo_t& right_lane_infos, 
                                                        const efm::MapStaticInfo& map_static_info, double& sum_heading_ri_lane_le, double& sum_heading_le_lane_ri,
                                                        double& aver_heading_ri_lane_le, double& aver_heading_le_lane_ri){
    sum_heading_ri_lane_le = 0.0;
    sum_heading_le_lane_ri = 0.0;
    aver_heading_ri_lane_le = 0.0;
    aver_heading_le_lane_ri = 0.0;
    uint32_t right_left_line_index = right_lane_infos.LBound.LBound;
    uint32_t right_center_line_index = right_lane_infos.Centeline.Centeline;
    uint32_t left_right_line_index = left_lane_infos.RBound.RBound;
    uint32_t left_center_line_index = left_lane_infos.Centeline.Centeline;
    std::vector<message::map_map::s_GeometryPoint_t> right_left_geometry_points{}, right_center_geometry_points{}, left_right_geometry_points{}, left_center_geometry_points{};
    EFMRefLinePoints right_left_line_points_utm{}, right_center_line_points_utm{}, left_right_line_points_utm{}, left_center_line_points_utm{};
    if ((false == efm::MapCommonTool::GetInstance()->GetLineGeometry(map_static_info, right_left_line_index, right_left_geometry_points)) ||
        (false == efm::MapCommonTool::GetInstance()->GetLineGeometry(map_static_info, right_center_line_index, right_center_geometry_points)) ||
        (false == efm::MapCommonTool::GetInstance()->GetLineGeometry(map_static_info, left_right_line_index, left_right_geometry_points)) ||
        (false == efm::MapCommonTool::GetInstance()->GetLineGeometry(map_static_info, left_center_line_index, left_center_geometry_points))) {
            // std::cout << "line not found 222" << std::endl;
            return false;
    }

    CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(right_left_geometry_points, right_left_line_points_utm, map_position_);
    CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(right_center_geometry_points, right_center_line_points_utm, map_position_);
    CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(left_right_geometry_points, left_right_line_points_utm, map_position_);
    CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(left_center_geometry_points, left_center_line_points_utm, map_position_);

    uint8_t tar_point_index = 0;
    double nearest_distance = 0.0, left_center_line_conincidece_dist = 0.0;
    // for(const auto& point : right_center_line_points_utm ){
    for(size_t pointidx = right_center_line_points_utm.size() - 1; pointidx > 0; pointidx--){
        CommonTool::CoordinateTool::GetInstance()->GetNearestPointV2(left_center_line_points_utm, right_center_line_points_utm[pointidx], nearest_distance);
        if(nearest_distance > p_merge_link_centerline_nearest_distance){
            tar_point_index = pointidx;
            // std::cout << __FILE__ << __LINE__ << ":" << "point_index: " << int(tar_point_index) << std::endl;
            break;
        }
        // std::cout << __FILE__ << __LINE__ << ":" << "nearest_distance: " << nearest_distance << std::endl;
        // std::cout << __FILE__ << __LINE__ << ":" << "pointidx: " << int(pointidx) << std::endl;

        double point_distance = 0;
        CommonTool::CoordinateTool::GetInstance()->calculateDistance(right_center_line_points_utm[pointidx].x, right_center_line_points_utm[pointidx].y, 0,
                                                                    right_center_line_points_utm[pointidx - 1].x, right_center_line_points_utm[pointidx - 1].y, 0, point_distance); 
        left_center_line_conincidece_dist += point_distance;
        // std::cout << __FILE__ << __LINE__ << "left_center_line_conincidece_dist: " << left_center_line_conincidece_dist << std::endl;
    }

    double right_left_line_dist = 0.0, left_right_line_dist = 0.0;
    uint8_t right_left_line_tar_point_index = 0, left_right_line_tar_point_index = 0;

    for(size_t pointidx = right_left_line_points_utm.size() - 1; pointidx > 0; pointidx--){
        double point_distance = 0;
        CommonTool::CoordinateTool::GetInstance()->calculateDistance(right_left_line_points_utm[pointidx].x, right_left_line_points_utm[pointidx].y, 0,
                                                                    right_left_line_points_utm[pointidx -1].x, right_left_line_points_utm[pointidx -1].y, 0, point_distance); 
        right_left_line_dist += point_distance;
        // std::cout << __FILE__ << __LINE__ << "right_left_line_dist: " << right_left_line_dist << std::endl;
        if(right_left_line_dist >= left_center_line_conincidece_dist){
            right_left_line_tar_point_index = pointidx > 255 ? 255 : pointidx;
            break;
        }
    }

    for(size_t pointidx = left_right_line_points_utm.size() - 1; pointidx > 0; pointidx--){
        double point_distance = 0;
        CommonTool::CoordinateTool::GetInstance()->calculateDistance(left_right_line_points_utm[pointidx].x, left_right_line_points_utm[pointidx].y, 0,
                                                                    left_right_line_points_utm[pointidx -1 ].x, left_right_line_points_utm[pointidx -1].y, 0, point_distance); 
        left_right_line_dist += point_distance;
        // std::cout << __FILE__ << __LINE__ << "left_right_line_dist: " << left_right_line_dist << std::endl;
        if(left_right_line_dist >= left_center_line_conincidece_dist){
            left_right_line_tar_point_index = pointidx > 255 ? 255 : pointidx;
            break;
        }
    }

    EFMRefLinePoints right_left_line_valid_points{}, left_right_line_valid_points{};
    for(size_t pointidx = 0; pointidx < right_left_line_tar_point_index && pointidx < right_left_line_points_utm.size(); pointidx++){
        right_left_line_valid_points.emplace_back(right_left_line_points_utm[pointidx]);
    }
    for(size_t pointidx = 0; pointidx < left_right_line_tar_point_index && pointidx < left_right_line_points_utm.size(); pointidx++){
        left_right_line_valid_points.emplace_back(left_right_line_points_utm[pointidx]);
    }

    std::vector<double> right_left_raw_accumulated_s{}, right_left_raw_kappas{}, right_left_raw_dkappas{}, right_left_raw_headings{};
    std::vector<double> left_right_raw_accumulated_s{}, left_right_raw_kappas{}, left_right_raw_dkappas{}, left_right_raw_headings{};
    CommonMathMethod::DiscretePointsMath::ComputePathProfile(right_left_line_valid_points, &right_left_raw_headings, &right_left_raw_accumulated_s,
                                                            &right_left_raw_kappas, &right_left_raw_dkappas);  
    CommonMathMethod::DiscretePointsMath::ComputePathProfile(left_right_line_valid_points, &left_right_raw_headings, &left_right_raw_accumulated_s,
                                                            &left_right_raw_kappas, &left_right_raw_dkappas);  


    if(right_left_raw_headings.size()>0){
            // std::cout<<"right_lane.Lbound heading: ";
             for(int i =0;i<right_left_raw_headings.size();i++){
                // std::cout<<" ,"<< raw_headings[i];
                double heading_tmp = right_left_raw_headings[i];
                heading_tmp = HeadingTransform(right_left_raw_headings[i]);
                sum_heading_ri_lane_le += heading_tmp;
             }
            //  std::cout<<std::endl;
              aver_heading_ri_lane_le = sum_heading_ri_lane_le/(right_left_raw_headings.size());
    }  

    if(left_right_raw_headings.size()>0){
            // std::cout<<"right_lane.Lbound heading: ";
             for(int i =0;i<left_right_raw_headings.size();i++){
                // std::cout<<" ,"<< raw_headings[i];
                double heading_tmp = left_right_raw_headings[i];
                heading_tmp = HeadingTransform(left_right_raw_headings[i]);
                sum_heading_le_lane_ri += heading_tmp;
             }
            //  std::cout<<std::endl;
              aver_heading_le_lane_ri = sum_heading_le_lane_ri/(left_right_raw_headings.size());
    }  
    
    // std::cout << __FILE__ << __LINE__ << "sum_heading_ri_lane_le: " << sum_heading_ri_lane_le << std::endl;
    // std::cout << __FILE__ << __LINE__ << "aver_heading_ri_lane_le: " << aver_heading_ri_lane_le << std::endl;
    // std::cout << __FILE__ << __LINE__ << "sum_heading_le_lane_ri: " << sum_heading_le_lane_ri << std::endl;
    // std::cout << __FILE__ << __LINE__ << "aver_heading_le_lane_ri: " << aver_heading_le_lane_ri << std::endl;

    return false;
}

bool CandidateLanesModel::GetEndOffsetCloseForArrowLane(std::vector<uint32_t> link_id_vec, std::vector<uint8_t> lane_num_vec,
                                                     int32_t close_idx, bool side_dir_is_left,uint32_t& end_offset) {
    //side_dir_is_left, side是左侧车道
    end_offset = 0;
    if ((close_idx + 1) > link_id_vec.size() || (close_idx + 1) > lane_num_vec.size()){
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
        // std::cout << __FILE__ << "," << __LINE__ << "," << " lane_id: " << (int)lane_num_vec[idx] << std::endl;

        uint32_t left_line = 0;
        uint32_t right_line = 0;
        if(side_dir_is_left){
            //取自车道的左和 左车道的左
            if(!GetLine(link_infos, lane_num_vec[idx], true, left_line)){
                return false;
            }
            if(!GetLine(link_infos, lane_num_vec[idx]+1, true, right_line)){
                return false;
            }            
        }else{
            //取自车道的右和 右车道的右
            if(!GetLine(link_infos, lane_num_vec[idx], false, left_line)){
                return false;
            }
            if(!GetLine(link_infos, lane_num_vec[idx]-1, false, right_line)){
                return false;
            }             
        }
        
        // std::cout << __FILE__ << "," << __LINE__ << "," << " left_line: " << left_line << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << "," << " right_line: " << right_line << std::endl;
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

        if (start_distance <= p_narrow_lane_width){
            //back
            end_offset = link_infos.PathOffset.PathOffset;
            continue;
        }

        if (end_distance >= p_narrow_lane_width){
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
            // if(1 == side_dir_is_left){//close to left, left line is base, 都是以自车道的线为基准，上面处理完后，自车道的线都是left_line
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
                   
                   if(static_cast<uint32_t>(sl.l*100)>p_narrow_lane_width){
                       gap_first = sl.l;
                       offset_first = point_offset;
                   }else{
                       gap_second = sl.l;
                       offset_second = point_offset;
                       break;
                   }
               }
            // }
            // std::cout << __FILE__ << "," << __LINE__ << "," << " gap_first: " << gap_first << std::endl;
            // std::cout << __FILE__ << "," << __LINE__ << "," << " gap_second: " << gap_second << std::endl;
            // std::cout << __FILE__ << "," << __LINE__ << "," << " offset_first: " << offset_first << std::endl;
            // std::cout << __FILE__ << "," << __LINE__ << "," << " offset_second: " << offset_second << std::endl;
            //根据gao 和offset， 按比例计算2m点的offset
            double ratio = (offset_second - offset_first)/(gap_first - gap_second);
            end_offset = offset_first;
            double temp_length = gap_first - static_cast<double>(p_narrow_lane_width)/100.0;
            if (temp_length > 0){
                end_offset += (int)(temp_length * ratio);
            }
            return true;
        }      
        
    }
    
    return true;
}

bool CandidateLanesModel::GetLine(message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id,bool is_left, uint32_t& line_id) {
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
    if(lane_id == 0){
        return false;
    }
    return true;
}

bool CandidateLanesModel::GetEgoLanePreCutInMerge(const std::vector<uint32_t>& link_id_vec, const std::vector<uint8_t>& lane_num_vec, 
                                                  const std::vector<LaneExtraInfo_s>& lane_extro_vec, std::vector<uint32_t>& end_offset_vec){
    end_offset_vec.clear();
    for(int i = 0; i <lane_num_vec.size() && i<lane_extro_vec.size();i++){
        uint32_t end_offset = 0;
        if(lane_extro_vec[i].merge_value == EFM_MergeType_FROM_LEFT){
            if(GetEndOffsetCloseForArrowLane(link_id_vec, lane_num_vec, i, true, end_offset)){
                end_offset_vec.push_back(end_offset);
            }
        }else if(lane_extro_vec[i].merge_value == EFM_MergeType_FROM_RIGHT){
            if(GetEndOffsetCloseForArrowLane(link_id_vec, lane_num_vec, i, false, end_offset)){
                end_offset_vec.push_back(end_offset);
            }
        }
    }
    return true;
}

bool CandidateLanesModel::GetLineGeometry(uint32_t line_id, std::vector<EFMPoint>& geometry_points) {

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

bool CandidateLanesModel::GetTwoLineDistance(uint32_t left_line, uint32_t right_line,
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
}  // namespace framework
}  // namespace shell
}  // namespace earth
