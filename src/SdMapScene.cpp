#include "SdMapScene.h"
#include "log_manager.h"
#define DEBUG_FLAG 0    

namespace NoMapEFM{

bool SdMapScene::Execute(SEhpOutputLoc& loc, SEhpOutputPathList& paths, SEhpOutputLinkList& links, 
                         SDLaneElementGroupSet& ele_group, MapRawDataMap& sd_index) {
    
    try{
        //更新link下面车道的信息
        UpdateLinksLanes(links);

        loc_   = loc;
        paths_ = paths;
        links_ = links;
        ele_group.clear();
        ele_group_.clear();
        sd_index_.lane_id_index_map.clear();
        sd_index_.link_path_offset_id_index_map.clear();
        sd_index_.path_id_index_map.clear();
        sd_index_ = sd_index;

        // //判断数据是否异常，定位是否成功
        // LOG_DEBUG("loc.path_id_: " + std::to_string(loc_.path_id_) + 
        //           ", loc.offset_: " + std::to_string(loc_.offset_) + 
        //           ", paths.ehp_output_path_list.size(): " + std::to_string(paths_.ehp_output_path_list.size()) +
        //           ", links.ehp_output_link_list.size(): " + std::to_string(links_.ehp_output_link_list.size()) +
        //           ", sd_index.link_path_offset_id_index_map.size(): " + std::to_string(sd_index_.link_path_offset_id_index_map.size()) +
        //           ", sd_index.path_id_index_map.size(): " + std::to_string(sd_index_.path_id_index_map.size()) +
        //           ", sd_index.lane_id_index_map.size(): " + std::to_string(sd_index_.lane_id_index_map.size()));

        if (!(loc.path_id_ > 0 && loc_.offset_ > 0 && 
            paths.ehp_output_path_list.size() > 0 && 
            links.ehp_output_link_list.size() > 0 &&
            sd_index.link_path_offset_id_index_map.size() > 0 &&
            sd_index.path_id_index_map.size() > 0 &&
            sd_index.lane_id_index_map.size() > 0)) {
            LOG_ERROR("loc.path_id_: " + std::to_string(loc_.path_id_));
            return false;
        }
        
        //获取自车所在的link和path
        if ((sd_index.link_path_offset_id_index_map.find(loc_.path_offset_id_) == sd_index.link_path_offset_id_index_map.end()) || 
            (sd_index.path_id_index_map.find(loc_.path_id_) == sd_index.path_id_index_map.end())){
                LOG_ERROR("loc.path_id_: " + std::to_string(loc_.path_id_));
            return false;
        }

        auto& loc_path = paths_.ehp_output_path_list[sd_index.path_id_index_map[loc_.path_id_]];
        auto& loc_link = links_.ehp_output_link_list[sd_index.link_path_offset_id_index_map[loc_.path_offset_id_]];
        if (loc_path.path_id_ != loc_.path_id_ || loc_link.path_offset_id_ != loc_.path_offset_id_) {
            LOG_ERROR("loc.path_id_: " + std::to_string(loc_.path_id_));
            return false;
        }

        //获取自车所在的path的link列表
        main_path_links_.clear();
        for (const auto& link_offset : loc_path.link_offsets_) {
            if (sd_index.link_path_offset_id_index_map.find(link_offset.path_offset_id_) != sd_index.link_path_offset_id_index_map.end()) {
                main_path_links_.push_back(link_offset.path_offset_id_);
            } else {
                LOG_ERROR("SdMapScene::Execute: link path offset id not found: " + std::to_string(link_offset.path_offset_id_));
            }
        }

        if (false == GetLocLane(loc_link)){
            LOG_ERROR("loc.path_id_: " + std::to_string(loc_.path_id_));
            return false;
        }

        // //打印自车所在的车道信息
        // LOG_DEBUG("loc.lane_ids_: ");
        // for (const auto& lane_id : loc_.lane_ids_) {
        //     LOG_DEBUG(std::to_string(lane_id) + " ");
        // }

        //根据自车所在的车道做成group
        for (const auto& lane_id : loc_.lane_ids_) {
            //如果车道id在link的车道信息中
            if (sd_index.lane_id_index_map.find(lane_id) != sd_index.lane_id_index_map.end()) {
                // LOG_DEBUG("SdMapScene::Execute: link_idx: " + std::to_string(sd_index.lane_id_index_map[lane_id][0]) +
                //           "lane_idx: " + std::to_string(sd_index.lane_id_index_map[lane_id][1]));
                auto& lane_info = links_.ehp_output_link_list[sd_index.lane_id_index_map[lane_id][0]]
                                  .link_lane_info_list_[sd_index.lane_id_index_map[lane_id][1]];
                auto& link_info = links_.ehp_output_link_list[sd_index.lane_id_index_map[lane_id][0]];
                if (DEBUG_FLAG == 1)
                {
                    LOG_DEBUG("SdMapScene::Execute: lane_id: " + std::to_string(lane_id) +
                              ", lane_num: " + std::to_string(lane_info.lane_num) +
                              ", link_id: " + std::to_string(link_info.id_) +
                              ", path_offset_id_: " + std::to_string(link_info.path_offset_id_));
                }
                SDLaneElementGroup one_ele_group;
                if (false == MakeSDGroup(link_info, lane_info, one_ele_group)) {
                    //如果MakeSDGroup失败，输出错误信息
                    //并继续处理下一个车道
                    LOG_ERROR("SdMapScene::Execute: MakeSDGroup failed for lane_id: " + std::to_string(lane_id));
                    continue; 
                }
                //遍历one_ele_group中的车道信息
                // LOG_DEBUG("SdMapScene::Execute: lane_id: " + std::to_string(lane_id));
                // for (const auto& ele : one_ele_group) {
                    // //遍历link_path_ids
                    // LOG_DEBUG("SdMapScene::Execute: link_path_id: ");
                    // for (const auto& link_path_id : ele.link_path_ids) {
                    //     LOG_DEBUG(std::to_string(link_path_id) + ",");
                    // }

                    // //遍历lane_in_link
                    // LOG_DEBUG("SdMapScene::Execute: lane_in_link: ");
                    // for (const auto& lane_in_link_id : ele.lane_in_link) {
                    //     LOG_DEBUG(std::to_string(lane_in_link_id) + ",");
                    // }
                    // //遍历lane_ids
                    // LOG_DEBUG("SdMapScene::Execute: lane_ids: ");
                    // for (const auto& lane_id : ele.lane_ids) {
                    //     LOG_DEBUG(std::to_string(lane_id) + ",");
                    // }                   

                    //遍历lane_nums
                    // LOG_DEBUG("SdMapScene::Execute: lane_nums: ");
                    // for (const auto& lane_num : ele.lane_nums) {
                    //     LOG_DEBUG(std::to_string(lane_num) + ",");
                    // }
                    // LOG_DEBUG("SdMapScene::Execute: lane_types: ");
                    // for (const auto& lane_num : ele.lane_types) {
                    //     LOG_DEBUG(std::to_string(lane_num) + ",");
                    // } 
                // }
                //将车道信息加入到车道组中
                ele_group_.push_back({lane_info.lane_num, one_ele_group});
            }
        }

        //对车道进行全局推荐判断
        if (false == MakeRefLane(ele_group_)) {
            LOG_ERROR("SdMapScene::Execute: MakeRefLane failed: " + std::to_string(ele_group.size()));
            return false; 
        }

        if (ele_group_.empty()){
            LOG_ERROR("loc.path_id_: " + std::to_string(loc_.path_id_));
            return false;
        }
        ele_filter_group_.clear();
        MakeFilterEleGroup();
        // LOG_DEBUG("loc.path_id_: " + std::to_string(loc_.path_id_));
        //ele_group = ele_group_;
        ele_group = ele_filter_group_;
        return true;
    }
    catch(const std::exception& e){
        LOG_ERROR(std::string(e.what()));
        return false;
    }

    return true;
}

void SdMapScene::UpdateLinksLanes(SEhpOutputLinkList& links) {
    for (auto& link: links.ehp_output_link_list) {
        std::vector<uint32_t> start_offsets;
        for (const auto& lane : link.link_lane_info_list_) {
            if (start_offsets.empty()) {
                start_offsets.push_back(lane.start_point_to_link_start_dis);
                continue;
            }
            
            auto it = std::find(start_offsets.begin(), start_offsets.end(), lane.start_point_to_link_start_dis);
            if (it == start_offsets.end()) {
                auto position = std::lower_bound(start_offsets.begin(), start_offsets.end(),
                    lane.start_point_to_link_start_dis);
                start_offsets.insert(position, lane.start_point_to_link_start_dis);
            }
        }

        if (start_offsets.empty()){
            continue;
        }
        

        //更新每个车道的length信息
        uint32_t last_lane_length = link.e_offset_ > (link.s_offset_ + link.link_lane_info_list_.back().start_point_to_link_start_dis) ? 
                                    link.e_offset_ - (link.s_offset_ + link.link_lane_info_list_.back().start_point_to_link_start_dis) : 0;
        for (auto& lane : link.link_lane_info_list_) {
            auto it = std::find(start_offsets.begin(), start_offsets.end(), lane.start_point_to_link_start_dis);
            if (it != start_offsets.end()) {
                size_t index = std::distance(start_offsets.begin(), it);
                lane.length = (index + 1 < start_offsets.size()) ? start_offsets[index + 1] - lane.start_point_to_link_start_dis : last_lane_length;
            }
        }

    }
}

void SdMapScene::MakeFilterEleGroup() {
    try {
        ele_filter_group_.clear();
        //获取is_group_dest的车道组
        if (ele_group_.empty()) {
            LOG_WARN("SdMapScene::MakeFilterEleGroup: ele_group_ is empty.");
            return;
        }
        //遍历所有的车道组
        for (auto& ele_group : ele_group_) {
            if (ele_group.second.empty()){
                LOG_WARN("SdMapScene::MakeFilterEleGroup: ele_group is empty.");
                continue;
            }

            SDLaneElementGroup one_ele_group;
            MakeFilterEleOneGroup(ele_group.second, one_ele_group);
            if (!one_ele_group.empty()) {
                ele_filter_group_.push_back({ele_group.first, one_ele_group});
            }
        }
    } catch (const std::exception& e) {
        LOG_ERROR("SdMapScene::MakeFilterEleGroup: " + std::string(e.what()));
    }
}

void SdMapScene::MakeFilterEleOneGroup(SDLaneElementGroup one_ele_group, SDLaneElementGroup& one_ele_filter_group) {
    try {
        one_ele_filter_group.clear();
        if (one_ele_group.empty()) {
            LOG_ERROR("SdMapScene::MakeFilterEleOneGroup: one_ele_group is empty.");
            return;
        }
        if (one_ele_group.size() == 1) {
            //如果车道组只有一个车道，直接加入到过滤后的车道组中
            one_ele_filter_group = one_ele_group;
            return;
        }
        

        // //遍历one_ele_group，打印
        // for (const auto& ele : one_ele_group) {
        //     LOG_DEBUG("one_ele_group.ele_id: " + std::to_string(ele.ele_id) + ", size: " + std::to_string(ele.lane_nums.size()));
        //     LOG_DEBUG("one_ele_group.is_group_dest: " + std::to_string(ele.is_group_dest) + ", one_ele_group.is_dest: " + std::to_string(ele.is_dest));
        //     //遍历ele.lane_nums，打印
        //     std::string lane_nums_str = "";
        //     for (const auto& lane_num : ele.lane_nums) {
        //         lane_nums_str += std::to_string(lane_num) + ",";
        //     }
        //     LOG_DEBUG("one_ele_group.lane_nums: " + lane_nums_str); 
        // }

        //找到is_group_dest车道
        SDLaneElement group_dest_ele = one_ele_group.front();
        auto it = std::find_if(one_ele_group.begin(), one_ele_group.end(), [](const SDLaneElement& ele) {
            return ele.is_group_dest;
        });
        if (it != one_ele_group.end()) {
            group_dest_ele = *it; //找到第一个is_group_dest的车道
        }else {
            LOG_ERROR("SdMapScene::MakeFilterEleOneGroup: no group dest lane found.");
            return;
        }
        //LOG_DEBUG("group_dest_ele.ele_id: " + std::to_string(group_dest_ele.ele_id) + ", size: " + std::to_string(group_dest_ele.lane_nums.size()));
        //获取bev_max_offset范围内含有几个lane
        int32_t lane_number = group_dest_ele.lane_e_offsets.size();
        for (size_t i = 0; i < group_dest_ele.lane_e_offsets.size(); i++){
            if (group_dest_ele.lane_e_offsets[i] >= bev_max_offset){
                lane_number = i + 1; //获取车道组中，车道的数量
                break;
            }
        }

        //LOG_DEBUG("lane_number: " + std::to_string(lane_number) + ", group_dest_ele.lane_nums: " + std::to_string(group_dest_ele.lane_nums.size()));
        
        std::pair<std::vector<uint8_t>, uint32_t> one_lanes_idx;
        if (lane_number == group_dest_ele.lane_nums.size()){
            one_lanes_idx.first.insert(one_lanes_idx.first.end(),
                group_dest_ele.lane_nums.begin(), group_dest_ele.lane_nums.end());
        }else{
            one_lanes_idx.first.insert(one_lanes_idx.first.end(),
                group_dest_ele.lane_nums.begin(), group_dest_ele.lane_nums.begin() + lane_number);
        }
        

        one_lanes_idx.second = group_dest_ele.ele_id;
        //LOG_DEBUG("one_lanes_idx: " + std::to_string(one_lanes_idx.first.size()) + ", ele_id: " + std::to_string(one_lanes_idx.second));
        std::vector<std::pair<std::vector<uint8_t>, uint32_t>> lanes_idxs;
        lanes_idxs.push_back(one_lanes_idx);

        //遍历车道组，获取bev_max_offset范围内的车道
        for (size_t i = 0; i < one_ele_group.size(); i++) {
            const SDLaneElement& one_ele = one_ele_group[i];
            if (one_ele.lane_nums.empty() || one_ele.ele_id == one_lanes_idx.second) {
                continue; 
            }
            //获取该车道组的车道信息
            std::pair<std::vector<uint8_t>, uint32_t> temp_lanes_idx;
            if (one_ele.lane_nums.size() <= lane_number){
                temp_lanes_idx.first.insert(temp_lanes_idx.first.end(), 
                    one_ele.lane_nums.begin(), one_ele.lane_nums.end());
            }else{
                temp_lanes_idx.first.insert(temp_lanes_idx.first.end(), 
                    one_ele.lane_nums.begin(), one_ele.lane_nums.begin() + lane_number);
            }
            one_lanes_idx.second = one_ele.ele_id;
            //如果该车道组的车道信息已经存在于lanes_idxs中，则跳过
            auto it_lanes = std::find_if(lanes_idxs.begin(), lanes_idxs.end(), 
                [&temp_lanes_idx](const std::pair<std::vector<uint8_t>, int32_t>& lanes_idx) {
                    return lanes_idx.first == temp_lanes_idx.first;
                });
            if (it_lanes != lanes_idxs.end()) {
                continue; 
            }else{
                auto position = std::lower_bound(lanes_idxs.begin(), lanes_idxs.end(), 
                    temp_lanes_idx, [](const std::pair<std::vector<uint8_t>, uint32_t>& a, 
                                       const std::pair<std::vector<uint8_t>, uint32_t>& b) {
                        for (size_t idx = 0; idx < a.first.size() && idx < b.first.size(); idx++){
                            if (a.first[idx] != b.first[idx]) {
                                return a.first[idx] < b.first[idx];
                            }
                        }
                        return a.first.size() < b.first.size();
                    });
                lanes_idxs.insert(position, temp_lanes_idx); //插入到合适的位置
            } 
        }

        //将车道组的车道信息做成group
        for (const auto& lanes_idx : lanes_idxs) {
            uint32_t max_remain_dist = 0;
            std::vector<int32_t> temp_idxs;
            for (int32_t i = 0; i < one_ele_group.size(); i++){
                SDLaneElement& one_ele = one_ele_group[i];
                if (lanes_idx.first.size() > one_ele.lane_nums.size()) {
                    continue; //如果车道组的车道信息比该车道组的车道信息多，则跳过
                }

                std::vector<uint8_t> temp_lane_nums;
                if (lanes_idx.first.size() == one_ele.lane_nums.size()){
                    temp_lane_nums.insert(temp_lane_nums.end(), 
                        one_ele.lane_nums.begin(), one_ele.lane_nums.end());
                }else{
                    temp_lane_nums.insert(temp_lane_nums.end(), 
                        one_ele.lane_nums.begin(), one_ele.lane_nums.begin() + lanes_idx.first.size());
                }

                if (temp_lane_nums == lanes_idx.first){
                    if (one_ele.is_group_dest) {
                        one_ele_filter_group.push_back(one_ele); //直接加入到过滤后的车道组中
                        temp_idxs.clear();
                        break;
                    }else{
                        temp_idxs.push_back(i); //记录该车道组的索引
                        max_remain_dist = max_remain_dist < one_ele.remain_dist ? one_ele.remain_dist : max_remain_dist; //获取最大剩余距离
                    }
                }
            }

            //如果该车道组的索引不为空，则将该车道组的车道信息做成group
            if (temp_idxs.empty() || max_remain_dist == 0) {
                continue; //如果该车道组的索引为空，则跳过
            }
            for (auto idx : temp_idxs){
                SDLaneElement& one_ele = one_ele_group[idx];
                if (one_ele.remain_dist == max_remain_dist){
                    one_ele_filter_group.push_back(one_ele);
                    break;
                }
                
            }
        }

        // //遍历one_ele_filter_group，打印
        // for (const auto& ele : one_ele_filter_group) {
        //     LOG_DEBUG("one_ele_group: " + std::to_string(ele.ele_id) + ", size: " + std::to_string(ele.lane_nums.size()));
        //     //遍历ele.lane_nums，打印
        //     std::string lane_nums_str = "";
        //     for (const auto& lane_num : ele.lane_nums) {
        //         lane_nums_str += std::to_string(lane_num) + ",";
        //     }
        //     LOG_DEBUG("one_ele_group.lane_nums: " + lane_nums_str); 
        // }
        

    } catch (const std::exception& e) {
        LOG_ERROR("SdMapScene::MakeFilterEleOneGroup: " + std::string(e.what()));
    }
}


bool SdMapScene::GetLocLane(SEhpOutputLink& loc_link) {
    try {
        //车道为空，或者link的车道信息为空，直接返回
        if (loc_link.link_lane_info_list_.empty()) {
            LOG_ERROR("SdMapScene::GetLocLane: loc_link.link_lane_info_list_ is empty.");
            return false;
        }

        //loc的offset_必须在link的s_offset_和e_offset_之间
        if (loc_.offset_ < loc_link.s_offset_ || loc_.offset_ > loc_link.e_offset_) {
            LOG_ERROR("SdMapScene::GetLocLane: loc_.offset_ is out of range: " + 
                      std::to_string(loc_.offset_) + 
                      ", loc_link.s_offset_: " + std::to_string(loc_link.s_offset_) + 
                      ", loc_link.e_offset_: " + std::to_string(loc_link.e_offset_) +
                      ", loc_link.id_: " + std::to_string(loc_link.id_));
            return false; //定位点不在link范围内
        }

        //对车道组进行排序
        auto laneinfo = loc_link.link_lane_info_list_;
        std::sort(laneinfo.begin(), laneinfo.end(), [](const LinkLaneInfo& a, const LinkLaneInfo& b) {
            // 先按 start_point_to_link_start_dis 升序排序，如果 start_point_to_link_start_dis 相同，则按 lane_num 升序排序
            if (a.start_point_to_link_start_dis == b.start_point_to_link_start_dis) {
                // 如果起点到link起点的距离相同，则按车道号升序排序

                return a.lane_num < b.lane_num;
            }
            return a.start_point_to_link_start_dis < b.start_point_to_link_start_dis;
        });

        //对车道的长度信息进行赋值
        // for (size_t i = 0; i < laneinfo.size(); i++){
        //     bool is_last_lane = true;
        //     for (size_t j = i+1; j < laneinfo.size(); j++){
        //         if (laneinfo[j].start_point_to_link_start_dis > laneinfo[i].start_point_to_link_start_dis){
        //             laneinfo[i].length = laneinfo[j].start_point_to_link_start_dis - laneinfo[i].start_point_to_link_start_dis;
        //             is_last_lane = false; //如果后面还有车道，则不是最后一个车道
        //             break;
        //         }
                
        //     }

        //     if (is_last_lane){
        //         if (loc_link.e_offset_ >= loc_link.s_offset_ + laneinfo[i].start_point_to_link_start_dis){
        //             laneinfo[i].length = loc_link.e_offset_ - loc_link.s_offset_ - 
        //                                  laneinfo[i].start_point_to_link_start_dis; //最后一个车道的长度
        //         }else{
        //             laneinfo[i].length = 0; //如果最后一个车道的长度小于0，则设置为0
        //         }
        //     }
        // }

        //获取自车所在的lane
        for (auto& lane : laneinfo) {
            //判断自车是否在lane的范围内
            // LOG_DEBUG("lane.start_point_to_link_start_dis: " + std::to_string(lane.start_point_to_link_start_dis) +
            //           ", lane.length: " + std::to_string(lane.length) +
            //           ", loc_link.s_offset_: " + std::to_string(loc_link.s_offset_) +
            //           ", loc_.offset_: " + std::to_string(loc_.offset_) + 
            //           ", loc_link.e_offset_: " + std::to_string(loc_link.e_offset_) +
            //           ", lane.lane_id: " + std::to_string(lane.lane_id));
            if (((lane.start_point_to_link_start_dis + loc_link.s_offset_ <= loc_.offset_) &&
                (lane.start_point_to_link_start_dis + lane.length + loc_link.s_offset_ > loc_.offset_)) ||
                ((loc_link.e_offset_ == loc_.offset_) && 
                 (lane.start_point_to_link_start_dis + lane.length + loc_link.s_offset_ == loc_.offset_))) {
                loc_.lane_ids_.push_back(lane.lane_id);
            }
        }
    } catch (const std::exception& e) {
        LOG_ERROR("SdMapScene::GetLocLane: " + std::string(e.what()));
        return false;
    }
    return true;
}

bool SdMapScene::MakeSDGroup(SEhpOutputLink& link_info, LinkLaneInfo& lane_info, SDLaneElementGroup& one_ele_group) {
    try {
        SDLaneElement one_lane_ele;
        one_lane_ele.lane_id = lane_info.lane_num;
        one_lane_ele.link_path_ids.push_back(link_info.path_offset_id_);
        one_lane_ele.lane_in_link.push_back(one_lane_ele.link_path_ids.size() - 1);
        one_lane_ele.lane_ids.push_back(lane_info.lane_id);
        one_lane_ele.lane_nums.push_back(lane_info.lane_num);
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("SdMapScene::MakeSDGroup: lane_id: " + std::to_string(lane_info.lane_id) +
                      ", lane_num: " + std::to_string(lane_info.lane_num) +
                      ", link_path_id: " + std::to_string(link_info.path_offset_id_));
        }

        
        //转换merge类型
        EfmMergeType one_merge_type = TransMergeType(lane_info.lane_change_type, lane_info);
        one_lane_ele.lane_merges.push_back(one_merge_type);
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("SdMapScene::MakeSDGroup: merge_type: " + std::to_string(one_merge_type));
        }

        //转换split类型
        EfmSplitType one_split_type = TransSplitType(lane_info.lane_change_type, lane_info);
        one_lane_ele.lane_splits.push_back(one_split_type);
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("SdMapScene::MakeSDGroup: split_type: " + std::to_string(one_split_type));
        }

        //转换lane类型
        LaneType lane_type = TransLaneType(lane_info.lane_types[0], link_info.fow_);
        one_lane_ele.lane_types.push_back(lane_type);
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("SdMapScene::MakeSDGroup: lane_type: " + std::to_string(lane_type));
        }

        //获取lane的起点和终点相对于自车的距离
        one_lane_ele.lane_s_offsets.push_back(loc_.lane_offset_ * (-1)); //起点相对于自车的距离，单位cm
        one_lane_ele.lane_e_offsets.push_back(lane_info.length - loc_.lane_offset_); //终点相对于自车的距离，单位cm
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("SdMapScene::MakeSDGroup: lane_s_offset: " + std::to_string(one_lane_ele.lane_s_offsets.back()));
            LOG_DEBUG("SdMapScene::MakeSDGroup: lane_e_offset: " + std::to_string(one_lane_ele.lane_e_offsets.back()));
            LOG_DEBUG("lane_info.is_route_lane: " + std::to_string(lane_info.is_route_lane));
        }

        //如果车道类型是应急车道或者应急停车带，则该车道组为不可行驶车道
        bool on_route = true;
        if (false == lane_info.is_route_lane || lane_type == LaneType::EnumLaneType_EMERGENCY_ || 
            lane_type == LaneType::EnumLaneType_EMERGENCY_PARKING_STRIP_) {
            on_route = false; //如果不是导航车道或者是应急停车带，则该车道组为不可行驶车道
        }
        one_lane_ele.lane_routes.push_back(on_route);
        one_lane_ele.all_length = (link_info.s_offset_ + lane_info.length + lane_info.start_point_to_link_start_dis) - loc_.offset_; //计算该车道组的总长度
        //LOG_DEBUG("SdMapScene::MakeSDGroup: all_length: " + std::to_string(one_lane_ele.all_length));
        //
        if (on_route) {
            one_lane_ele.remain_dist = one_lane_ele.all_length; //计算剩余距离
        } else {
            one_lane_ele.remain_dist = 0; //如果不是导航车道或者是应急停车带，则剩余距离为0
        }

        //判断one_merge_type是否为 EFM_MergeType_TO_LEFT、EFM_MergeType_TO_RIGHT
        if (one_merge_type == EfmMergeType::EFM_MergeType_TO_LEFT || 
            one_merge_type == EfmMergeType::EFM_MergeType_TO_RIGHT) {
            one_lane_ele.merge_count = 1; //如果是合流车道，则合流数量为1如果是合流车道，则合流数量为1
            one_lane_ele.first_merge_index = 0; //如果是合流车道，则第一个合流的index为0
        }

        one_ele_group.push_back(one_lane_ele);

        //循环向前找对应的连接车道，组成车道组
        if (false == GetNextLane(one_ele_group)) {
            LOG_ERROR("SdMapScene::MakeSDGroup: GetNextLane failed for lane_id: " + std::to_string(lane_info.lane_id));
            return false; //如果获取下一车道失败，返回false
        }

        std::vector<int32_t> ref_idxs;
        //对车道组内数据根据remain_dist进行排序
        std::sort(one_ele_group.begin(), one_ele_group.end(), [](const SDLaneElement& a, const SDLaneElement& b) {
            return a.remain_dist > b.remain_dist; // 按remain_dist降序排序
        });
        // //遍历one_ele_group，打印
        // for (const auto& ele : one_ele_group) {
        //     LOG_DEBUG("one_ele_group.ele_id: " + std::to_string(ele.ele_id) + ", size: " + std::to_string(ele.lane_nums.size()));
        //     LOG_DEBUG("one_ele_group.is_group_dest: " + std::to_string(ele.is_group_dest) + ", one_ele_group.is_dest: " + std::to_string(ele.is_dest));
        //     //遍历ele.lane_nums，打印
        //     std::string lane_nums_str = "";
        //     for (const auto& lane_num : ele.lane_nums) {
        //         lane_nums_str += std::to_string(lane_num) + ",";
        //     }
        //     LOG_DEBUG("one_ele_group.lane_nums: " + lane_nums_str);
        // }

        //判断本组内的目标车道
        if (false == GetRefEleForGroup(one_ele_group, ref_idxs)) {
            LOG_ERROR("SdMapScene::MakeSDGroup: GetRefForGroup failed for ref ele: " + std::to_string(one_ele_group[0].lane_id));
        }

        // //遍历ref_idxs，打印
        // for (const auto& idx : ref_idxs) {
        //     LOG_DEBUG("ref_idxs: " + std::to_string(idx));
        // }

        if (ref_idxs.empty()) {
            //如果ref_idxs为空，则默认第一个设置为组内推荐车道
            one_ele_group[0].is_group_dest = true;
        }else{
            //如果ref_idxs不为空，则将第一个设置为组内推荐车道
            one_ele_group[ref_idxs[0]].is_group_dest = true;
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("SdMapScene::MakeSDGroup: " + std::string(e.what()));
        return false;
    }
    return true;
}

bool SdMapScene::MakeRefLane(SDLaneElementGroupSet& ele_group) {
    try {
        if (ele_group.empty()) {
            LOG_ERROR("SdMapScene::MakeRefLane: conditions_ or ele_group is empty.");
            return false; //如果条件或车道组为空，返回false
        }
        
        //对车道组进行排序
        std::sort(ele_group.begin(), ele_group.end(), [](const auto& a, const auto& b) {
            return a.first < b.first; // 按照车道编号升序排序
        });

        //遍历车道组，赋值ele_id
        uint32_t max_ele_id = 0;
        for (auto& group : ele_group) {
            SDLaneElementGroup& one_ele_group = group.second;
            if (one_ele_group.empty()) {
                continue; //如果车道组为空，跳过
            }
            //获取当前组的最大ele_id
            for (size_t i = 0; i < one_ele_group.size(); ++i) {
                one_ele_group[i].ele_id = max_ele_id + 1;
                max_ele_id ++;
            }
        }
        // for (auto& one_ele_group : ele_group) {
        //     for (const auto& ele : one_ele_group.second) {
        //         LOG_DEBUG("one_ele_group.first: " + std::to_string(one_ele_group.first));
        //         LOG_DEBUG("one_ele_group.ele_id: " + std::to_string(ele.ele_id) + ", size: " + std::to_string(ele.lane_nums.size()));
        //         LOG_DEBUG("one_ele_group.is_group_dest: " + std::to_string(ele.is_group_dest) + ", one_ele_group.is_dest: " + std::to_string(ele.is_dest));
        //         //遍历ele.lane_nums，打印
        //         std::string lane_nums_str = "";
        //         for (const auto& lane_num : ele.lane_nums) {
        //             lane_nums_str += std::to_string(lane_num) + ",";
        //         }
        //         LOG_DEBUG("one_ele_group.lane_nums: " + lane_nums_str); 
        //     }
        // }

        std::vector<SDLaneElement> dest_group_eles;
        //遍历车道组，获取is_group_dest为true的车道组
        for (auto& group : ele_group) {
            SDLaneElementGroup& one_ele_group = group.second;
            if (one_ele_group.empty()) {
                continue; //如果车道组为空，跳过
            }

            if (one_ele_group.size() == 1){
                one_ele_group[0].is_group_dest = true;
                dest_group_eles.push_back(one_ele_group[0]);
                continue;
            }
            
            for (const auto& lane : one_ele_group) {
                if (lane.is_group_dest) {
                    dest_group_eles.push_back(lane);
                }
            }
        }
        // //遍历dest_group_eles，打印
        // for (const auto& ele : dest_group_eles) {
        //     LOG_DEBUG("dest_group_eles.ele_id: " + std::to_string(ele.ele_id) + ", size: " + std::to_string(ele.lane_nums.size()));
        //     LOG_DEBUG("dest_group_eles.is_group_dest: " + std::to_string(ele.is_group_dest) + ", dest_group_eles.is_dest: " + std::to_string(ele.is_dest));
        //     //遍历ele.lane_nums，打印
        //     std::string lane_nums_str = "";
        //     for (const auto& lane_num : ele.lane_nums) {
        //         lane_nums_str += std::to_string(lane_num) + ",";
        //     }
        //     LOG_DEBUG("dest_group_eles.lane_nums: " + lane_nums_str);
        // }

        std::vector<int32_t> ref_idxs;
        //对group_eles数据根据remain_dist进行排序
        std::sort(dest_group_eles.begin(), dest_group_eles.end(), [](const SDLaneElement& a, const SDLaneElement& b) {
            return a.remain_dist > b.remain_dist; // 按remain_dist降序排序
        });

        //获取推荐车道
        if (false == GetRefEleForGroup(dest_group_eles, ref_idxs)) {
            LOG_ERROR("SdMapScene::MakeRefLane: GetRefForGroup failed for ref ele: " + std::to_string(dest_group_eles[0].ele_id));
            return false; //如果获取推荐车道失败，返回false
        }
        // //遍历ref_idxs，打印
        // for (const auto& idx : ref_idxs) {
        //     LOG_DEBUG("ref_idxs: " + std::to_string(idx));
        // }

        std::vector<uint32_t> dest_ele_ids;
        //如果ref_idxs为空，则默认第一个设置为推荐车道
        if (ref_idxs.empty()) {
            dest_ele_ids.push_back(dest_group_eles[0].ele_id); //如果没有推荐车道，则默认第一个车道为目标车道
        } else {
            //如果ref_idxs不为空，则遍历group_eles，全部赋值为推荐车道
            for (const auto& idx : ref_idxs) {
                if (idx < dest_group_eles.size()) {
                    dest_ele_ids.push_back(dest_group_eles[idx].ele_id); //将推荐车道的ele_id加入到dst_ele_ids中
                } else {
                    LOG_ERROR("SdMapScene::MakeRefLane: ref index out of range: " + std::to_string(idx));
                }
            }
        }

        //将推荐车道信息加入到车道组中
        for (auto& group : ele_group) {
            SDLaneElementGroup& one_ele_group = group.second;
            if (one_ele_group.empty()) {
                continue; //如果车道组为空，跳过
            }
            //判断ele_id是否在dest_ele_ids中，是则设置is_dest为true
            for (auto& lane : one_ele_group) {
                if (std::find(dest_ele_ids.begin(), dest_ele_ids.end(), lane.ele_id) != dest_ele_ids.end()) {
                    lane.is_dest = true; //设置推荐车道标志
                } else {
                    lane.is_dest = false; //清空推荐车道标志
                }
            }
        }
        // for (auto& one_ele_group : ele_group) {
        //     for (const auto& ele : one_ele_group.second) {
        //         LOG_DEBUG("one_ele_group.first: " + std::to_string(one_ele_group.first));
        //         LOG_DEBUG("one_ele_group.ele_id: " + std::to_string(ele.ele_id) + ", size: " + std::to_string(ele.lane_nums.size()));
        //         LOG_DEBUG("one_ele_group.is_group_dest: " + std::to_string(ele.is_group_dest) + ", one_ele_group.is_dest: " + std::to_string(ele.is_dest));
        //         //遍历ele.lane_nums，打印
        //         std::string lane_nums_str = "";
        //         for (const auto& lane_num : ele.lane_nums) {
        //             lane_nums_str += std::to_string(lane_num) + ",";
        //         }
        //         LOG_DEBUG("one_ele_group.lane_nums: " + lane_nums_str); 
        //     }
        // }
    } catch (const std::exception& e) {
        LOG_ERROR("SdMapScene::MakeRefLane: " + std::string(e.what()));
        return false;
    }
    return true;
}

LaneType SdMapScene::TransLaneType(LANE_TYPE lane_type, LINK_FOW fow) {
    switch (lane_type) {
        case LANE_TYPE::LANE_TYPE_COMPOUND:
        case LANE_TYPE::LANE_TYPE_LEFT_DECELERATION:
        case LANE_TYPE::LANE_TYPE_RIGHT_DECELERATION:
            return LaneType::EnumLaneType_DECELERATE_;

        case LANE_TYPE::LANE_TYPE_LEFT_ACCELERATION:
        case LANE_TYPE::LANE_TYPE_RIGHT_ACCELERATION:
            return LaneType::EnumLaneType_ACCELERATE_;

        case LANE_TYPE::LANE_TYPE_SHOULDER:
        case LANE_TYPE::LANE_TYPE_DRIVABLE_SHOULDER:
        case LANE_TYPE::LANE_TYPE_CONTROL:
        case LANE_TYPE::LANE_TYPE_EMERGENCY_PARKING_STRIP:
        case LANE_TYPE::LANE_TYPE_BUS:
        case LANE_TYPE::LANE_TYPE_BICYCLE:
        case LANE_TYPE::LANE_TYPE_PARKING_ROAD:
        case LANE_TYPE::LANE_TYPE_DRIVABLE_PARKING_ROAD:
        case LANE_TYPE::LANE_TYPE_TAXI:
        case LANE_TYPE::LANE_TYPE_STRAIGHT_WAITING:
        case LANE_TYPE::LANE_TYPE_MOTOR:
        case LANE_TYPE::LANE_TYPE_DANGEROUS_ARTICLE:
        case LANE_TYPE::LANE_TYPE_FORBIDDEN_DRIVE:
        case LANE_TYPE::LANE_TYPE_THOUGH_LANE_ZONE:
        case LANE_TYPE::LANE_TYPE_STREET_RAILWAY:
        case LANE_TYPE::LANE_TYPE_BUS_BAY:
        case LANE_TYPE::LANE_TYPE_SPECIAL_CAR:
        case LANE_TYPE::LANE_TYPE_PEDESTRIANS:
        case LANE_TYPE::LANE_TYPE_HEDGING:
        case LANE_TYPE::LANE_TYPE_EMPTY:
        case LANE_TYPE::LANE_TYPE_REVERSE_NON_VEHICLE:
        case LANE_TYPE::LANE_TYPE_EDGE:
        case LANE_TYPE::LANE_TYPE_NONE:
            return LaneType::EnumLaneType_EMERGENCY_PARKING_STRIP_;

        case LANE_TYPE::LANE_TYPE_EMERGENCY:
            return LaneType::EnumLaneType_EMERGENCY_;

        default:
            // 通过fow判断车道类型
            if (fow == LINK_FOW::LINK_FOW_ENTRANCE_RAMP) {
                return LaneType::EnumLaneType_ON_RAMP_;
            }else if (fow == LINK_FOW::LINK_FOW_EXIT_RAMP) {
                return LaneType::EnumLaneType_OFF_RAMP_;
            } else if (fow == LINK_FOW::LINK_FOW_RAMP || fow == LINK_FOW::LINK_FOW_JCT) {
                return LaneType::EnumLaneType_CONNECT_RAMP_;
            } else {
                return LaneType::EnumLaneType_NORMAL_;
            }
    }
}

EfmMergeType SdMapScene::TransMergeType(LaneChangeType lane_change_type, LinkLaneInfo& lane_info) {
    //判断索引是否在车道id索引中
    if (sd_index_.lane_id_index_map.find(lane_info.lane_id) == sd_index_.lane_id_index_map.end()) {
        return EfmMergeType::EFM_MergeType_NONE; // 如果车道id不在索引中，返回默认类型
    }
    auto& lane_list = links_.ehp_output_link_list[sd_index_.lane_id_index_map[lane_info.lane_id][0]].link_lane_info_list_;

    switch (lane_change_type) {
        case LaneChangeType::RightTurnMergingLane:
            return EfmMergeType::EFM_MergeType_TO_LEFT;
        case LaneChangeType::LeftTurnMergingLane:
            return EfmMergeType::EFM_MergeType_TO_RIGHT;
        case LaneChangeType::BothDirectionMergingLane:
            //遍历lane_list，查找是否有左侧或者右侧车道是否也是 BothDirectionMergingLane 类型
            for (const auto& lane : lane_list) {
                if ((lane.start_point_to_link_start_dis == lane_info.start_point_to_link_start_dis) &&
                    ((lane.lane_num + 1) == lane_info.lane_num) && 
                    (lane.lane_change_type == LaneChangeType::BothDirectionMergingLane) &&
                    (lane.next_lane_ids.size() > 0 && lane_info.next_lane_ids.size() > 0 && 
                     lane.next_lane_ids[0] == lane_info.next_lane_ids[0])) {
                    return EfmMergeType::EFM_MergeType_RIGHT_TO_MIDDLE; // 如果左侧车道类型是BothDirectionMergingLane，返回右侧到中间类型
                }

                if ((lane.start_point_to_link_start_dis == lane_info.start_point_to_link_start_dis) &&
                    (lane.lane_num == (lane_info.lane_num + 1)) && 
                    (lane.lane_change_type == LaneChangeType::BothDirectionMergingLane)  &&
                    (lane.next_lane_ids.size() > 0 && lane_info.next_lane_ids.size() > 0 && 
                     lane.next_lane_ids[0] == lane_info.next_lane_ids[0])) {
                    return EfmMergeType::EFM_MergeType_LEFT_TO_MIDDLE; // 如果右侧车道类型是BothDirectionMergingLane，返回左侧到中间类型
                }             
            }
            return EfmMergeType::EFM_MergeType_TO_MIDDLE; // 如果没有找到旁边的BothDirectionMergingLane车道，返回中间类型

        default:
            //遍历lane_list，查找是否有左侧或者右侧车道是否也是 LaneChangeType::LeftTurnMergingLane 或者 LaneChangeType::RightTurnMergingLane 类型
            for (const auto& lane : lane_list) {
                if ((lane.start_point_to_link_start_dis == lane_info.start_point_to_link_start_dis) &&
                    ((lane.lane_num + 1) == lane_info.lane_num) && 
                    (lane.lane_change_type == LaneChangeType::LeftTurnMergingLane) &&
                    (lane.next_lane_ids.size() > 0 && lane_info.next_lane_ids.size() > 0 && 
                     lane.next_lane_ids[0] == lane_info.next_lane_ids[0])) {
                    return EfmMergeType::EFM_MergeType_FROM_LEFT; // 如果左侧车道类型是LeftTurnMergingLane，返回从左侧类型
                }

                if ((lane.start_point_to_link_start_dis == lane_info.start_point_to_link_start_dis) &&
                    (lane.lane_num == (lane_info.lane_num + 1)) && 
                    (lane.lane_change_type == LaneChangeType::RightTurnMergingLane) &&
                    (lane.next_lane_ids.size() > 0 && lane_info.next_lane_ids.size() > 0 && 
                     lane.next_lane_ids[0] == lane_info.next_lane_ids[0])) {
                    return EfmMergeType::EFM_MergeType_FROM_RIGHT; // 如果右侧车道类型是RightTurnMergingLane，返回从右侧类型
                }             
            }
            return EfmMergeType::EFM_MergeType_NONE; // 默认返回
    }
}

EfmSplitType SdMapScene::TransSplitType(LaneChangeType lane_change_type, LinkLaneInfo& lane_info) {
    //判断索引是否在车道id索引中
    if (sd_index_.lane_id_index_map.find(lane_info.lane_id) == sd_index_.lane_id_index_map.end()) {
        return EfmSplitType::EFM_SplitType_NONE; // 如果车道id不在索引中，返回默认类型
    }
    auto& lane_list = links_.ehp_output_link_list[sd_index_.lane_id_index_map[lane_info.lane_id][0]].link_lane_info_list_;

    switch (lane_change_type) {
        case LaneChangeType::RightTurnExpandingLane:
            return EfmSplitType::EFM_SplitType_FROM_LEFT;
        case LaneChangeType::LeftTurnExpandingLane:
            return EfmSplitType::EFM_SplitType_FROM_RIGHT;
        case LaneChangeType::BothDirectionExpandingLane:
            //遍历lane_list，查找是否有左侧或者右侧车道是否也是 BothDirectionExpandingLane 类型
            for (const auto& lane : lane_list) {
                if ((lane.start_point_to_link_start_dis == lane_info.start_point_to_link_start_dis) &&
                    ((lane.lane_num + 1) == lane_info.lane_num) && 
                    (lane.lane_change_type == LaneChangeType::BothDirectionExpandingLane) &&
                    (lane.pre_lane_ids.size() > 0 && lane_info.pre_lane_ids.size() > 0 && 
                     lane.pre_lane_ids[0] == lane_info.pre_lane_ids[0])) {
                    return EfmSplitType::EFM_SplitType_SPLIT_FROM_LEFT; // 如果左侧车道类型是BothDirectionExpandingLane
                }

                if ((lane.start_point_to_link_start_dis == lane_info.start_point_to_link_start_dis) &&
                    (lane.lane_num == (lane_info.lane_num + 1)) && 
                    (lane.lane_change_type == LaneChangeType::BothDirectionExpandingLane) &&
                    (lane.next_lane_ids.size() > 0 && lane_info.next_lane_ids.size() > 0 && 
                     lane.next_lane_ids[0] == lane_info.next_lane_ids[0])) {
                    return EfmSplitType::EFM_SplitType_SPLIT_FROM_RIGHT;
                }

                if ((lane.start_point_to_link_start_dis == lane_info.start_point_to_link_start_dis) &&
                    ((lane.lane_num + 1) == lane_info.lane_num) && 
                    (lane.lane_change_type == LaneChangeType::LeftTurnExpandingLane) &&
                    (lane.pre_lane_ids.size() > 0 && lane_info.pre_lane_ids.size() > 0 && 
                     lane.pre_lane_ids[0] == lane_info.pre_lane_ids[0])) {
                    return EfmSplitType::EFM_SplitType_CONTIUE_FROM_LEFT; // 如果左侧车道类型是BothDirectionExpandingLane
                }

                if ((lane.start_point_to_link_start_dis == lane_info.start_point_to_link_start_dis) &&
                    (lane.lane_num == (lane_info.lane_num + 1)) && 
                    (lane.lane_change_type == LaneChangeType::RightTurnExpandingLane) &&
                    (lane.next_lane_ids.size() > 0 && lane_info.next_lane_ids.size() > 0 && 
                     lane.next_lane_ids[0] == lane_info.next_lane_ids[0])) {
                    return EfmSplitType::EFM_SplitType_CONTIUE_FROM_RIGHT;
                }
            }

            return EfmSplitType::EFM_SplitType_CONTIUE_FROM_RIGHT; // 如果没有找到旁边的BothDirectionExpandingLane/LeftTurnExpandingLane/RightTurnExpandingLane车道，返回
        default:
            //遍历lane_list，查找是否有左侧或者右侧车道是否也是 LaneChangeType::LeftTurnExpandingLane 或者 LaneChangeType::RightTurnExpandingLane 类型
            for (const auto& lane : lane_list) {
                if ((lane.start_point_to_link_start_dis == lane_info.start_point_to_link_start_dis) &&
                    ((lane.lane_num + 1) == lane_info.lane_num) && 
                    (lane.lane_change_type == LaneChangeType::LeftTurnExpandingLane) &&
                    (lane.pre_lane_ids.size() > 0 && lane_info.pre_lane_ids.size() > 0 && 
                     lane.pre_lane_ids[0] == lane_info.pre_lane_ids[0])) {
                    return EfmSplitType::EFM_SplitType_TO_LEFT; // 如果左侧车道类型是LeftTurnExpandingLane，返回从左侧类型
                }

                if ((lane.start_point_to_link_start_dis == lane_info.start_point_to_link_start_dis) &&
                    (lane.lane_num == (lane_info.lane_num + 1)) && 
                    (lane.lane_change_type == LaneChangeType::RightTurnExpandingLane) &&
                    (lane.next_lane_ids.size() > 0 && lane_info.next_lane_ids.size() > 0 && 
                     lane.next_lane_ids[0] == lane_info.next_lane_ids[0])) {
                    return EfmSplitType::EFM_SplitType_TO_RIGHT; // 如果右侧车道类型是RightTurnExpandingLane，返回从右侧类型
                }             
            }
            return EfmSplitType::EFM_SplitType_NONE; // 默认返回
    }
    
}

bool SdMapScene::GetNextLane(SDLaneElementGroup& one_ele_group){
    try {
        //遍历one_ele_group中的车道
        if (one_ele_group.empty()) {
            return false; //如果车道组为空，返回false
        }
        uint32_t group_index = 0;
        uint32_t group_number = one_ele_group.size();
        while (group_index < group_number){
            auto & one_lane_ele = one_ele_group[group_index];
            //如果车道组已经结束，直接跳过
            if (one_lane_ele.is_end) {
                group_index += 1; //如果车道组已经结束，跳过该车道
                continue;
            }

            //获取最后一个车道对应的LinkLaneInfo信息
            if (sd_index_.lane_id_index_map.find(one_lane_ele.lane_ids.back()) == sd_index_.lane_id_index_map.end()) {
                LOG_ERROR("SdMapScene::GetNextLane: lane_id not found in index map: " + std::to_string(one_lane_ele.lane_ids.back()));
                one_lane_ele.is_end = true; //标记该车道组为结束
                group_index += 1; //如果车道id不在索引中，跳过该车道
                continue;
            }

            auto& lane_info = links_.ehp_output_link_list[sd_index_.lane_id_index_map[one_lane_ele.lane_ids.back()][0]]
                              .link_lane_info_list_[sd_index_.lane_id_index_map[one_lane_ele.lane_ids.back()][1]];
            auto& link_info = links_.ehp_output_link_list[sd_index_.lane_id_index_map[one_lane_ele.lane_ids.back()][0]];

            //如果没有后继车道，直接跳过
            if (lane_info.next_lane_ids.empty()) {
                one_lane_ele.is_end = true; //标记该车道组为结束
                group_index += 1; //如果没有后继车道，跳过该车道
                continue;
            }

            bool is_update = false; //是否需要更新车道组
            SDLaneElement befor_split_ele = one_lane_ele; //用于记录分歧前的车道信息
            //遍历后继车道id列表
            for (int next_lane_id_idx = 0; next_lane_id_idx < lane_info.next_lane_ids.size(); ++next_lane_id_idx) {
                auto& next_lane_id = lane_info.next_lane_ids[next_lane_id_idx];
                //如果后继车道id在link的车道信息中
                if (sd_index_.lane_id_index_map.find(next_lane_id) == sd_index_.lane_id_index_map.end()) {
                    LOG_ERROR("SdMapScene::GetNextLane: next lane_id not found in index map: " + std::to_string(next_lane_id));
                    continue; //如果后继车道id不在索引中，跳过该车道
                }
                
                //获取后继车道的LinkLaneInfo信息
                auto& next_lane_info = links_.ehp_output_link_list[sd_index_.lane_id_index_map[next_lane_id][0]]
                                       .link_lane_info_list_[sd_index_.lane_id_index_map[next_lane_id][1]];
                auto& next_link_info = links_.ehp_output_link_list[sd_index_.lane_id_index_map[next_lane_id][0]];
                if (DEBUG_FLAG == 1)
                {
                    LOG_DEBUG("SdMapScene::GetNextLane: next_lane_id: " + std::to_string(next_lane_id));
                    LOG_DEBUG("SdMapScene::GetNextLane: next_lane_num: " + std::to_string(next_lane_info.lane_num));
                    LOG_DEBUG("SdMapScene::GetNextLane: next_link_path_id: " + std::to_string(next_link_info.path_offset_id_));
                }

                //判断后继link是否在当前path的link列表中
                if (std::find(main_path_links_.begin(), main_path_links_.end(), next_link_info.path_offset_id_) == main_path_links_.end()) {
                    LOG_ERROR("SdMapScene::GetNextLane: next link not in main path links: " + std::to_string(next_link_info.path_offset_id_));
                    continue; //如果后继link不在当前path的link列表中，跳过该车道
                }
                if (is_update){
                    //新增分歧车道
                    SDLaneElement one_next_lane_ele = befor_split_ele;
                    //如果最后一个link和当前link相同，则不需要添加link_path_ids
                    if (one_next_lane_ele.link_path_ids.back() != next_link_info.path_offset_id_) {
                        one_next_lane_ele.link_path_ids.push_back(next_link_info.path_offset_id_);
                    }
                    one_next_lane_ele.lane_in_link.push_back(one_next_lane_ele.link_path_ids.size() - 1);
                    one_next_lane_ele.lane_ids.push_back(next_lane_info.lane_id);
                    one_next_lane_ele.lane_nums.push_back(next_lane_info.lane_num);
                    
                    //转换merge类型
                    EfmMergeType one_merge_type = TransMergeType(next_lane_info.lane_change_type, next_lane_info);
                    one_next_lane_ele.lane_merges.push_back(one_merge_type);

                    //转换split类型
                    EfmSplitType one_split_type = TransSplitType(next_lane_info.lane_change_type, next_lane_info);
                    one_next_lane_ele.lane_splits.push_back(one_split_type);

                    //转换lane类型
                    LaneType lane_type = TransLaneType(next_lane_info.lane_types[0], next_link_info.fow_);
                    one_next_lane_ele.lane_types.push_back(lane_type);

                    //获取lane的起点和终点相对于自车的距离
                    one_next_lane_ele.lane_s_offsets.push_back(one_next_lane_ele.lane_e_offsets.back()); //起点相对于自车的距离，单位cm
                    one_next_lane_ele.lane_e_offsets.push_back(one_next_lane_ele.lane_e_offsets.back() + next_lane_info.length); //终点点相对于自车的距离，单位cm

                    //如果车道类型是应急车道或者应急停车带，则该车道组为不可行驶车道
                    bool on_route = true;
                    //if (false == next_lane_info.is_route_lane || lane_type == LaneType::EnumLaneType_EMERGENCY_ || 
                    if (lane_type == LaneType::EnumLaneType_EMERGENCY_ || 
                        lane_type == LaneType::EnumLaneType_EMERGENCY_PARKING_STRIP_) {
                        on_route = false; //如果不是导航车道或者是应急停车带，则该车道组为不可行驶车道
                    }
                    one_next_lane_ele.lane_routes.push_back(on_route);
                    if (DEBUG_FLAG == 1)
                    {
                        LOG_DEBUG("SdMapScene::MakeSDGroup: lane_s_offset: " + std::to_string(one_next_lane_ele.lane_s_offsets.back()));
                        LOG_DEBUG("SdMapScene::MakeSDGroup: lane_e_offset: " + std::to_string(one_next_lane_ele.lane_e_offsets.back()));
                        LOG_DEBUG("lane_info.is_route_lane: " + std::to_string(lane_info.is_route_lane));
                        LOG_DEBUG("one_next_lane_ele.all_length: " + std::to_string(one_next_lane_ele.all_length));
                        LOG_DEBUG("one_next_lane_ele.remain_dist: " + std::to_string(one_next_lane_ele.remain_dist));
                        LOG_DEBUG("next_lane_info.lane_type: " + std::to_string(lane_type));
                        LOG_DEBUG("one_next_lane_ele.lane_merges: " + std::to_string(one_merge_type));
                        LOG_DEBUG("one_next_lane_ele.lane_splits: " + std::to_string(one_split_type));

                    }

                    if (on_route && one_next_lane_ele.all_length == one_next_lane_ele.remain_dist) {
                        one_next_lane_ele.remain_dist += next_lane_info.length; //计算剩余距离
                    }

                    one_next_lane_ele.all_length += next_lane_info.length; //计算该车道组的总长度

                    //判断one_merge_type是否为 EFM_MergeType_TO_LEFT、EFM_MergeType_TO_RIGHT
                    if (one_merge_type == EfmMergeType::EFM_MergeType_TO_LEFT || 
                        one_merge_type == EfmMergeType::EFM_MergeType_TO_RIGHT) {
                        one_next_lane_ele.merge_count += 1; //如果是合流车道，则合流数量为1
                        //如果first_merge_index为-1，则表示该车道组是第一个合流车道
                        if (one_next_lane_ele.first_merge_index == -1) {
                            one_next_lane_ele.first_merge_index = one_next_lane_ele.lane_merges.size() - 1; //如果是第一个合流车道，则记录该车道组的合流索引
                        }
                    }

                    //判断是否为EFM_SplitType_FROM_LEFT、EFM_SplitType_FROM_RIGHT、EFM_SplitType_SPLIT_FROM_LEFT、EFM_SplitType_SPLIT_FROM_RIGHT
                    if (one_split_type == EfmSplitType::EFM_SplitType_FROM_LEFT || 
                        one_split_type == EfmSplitType::EFM_SplitType_FROM_RIGHT) {
                        //如果first_split_index为-1，则表示该车道组是第一个分歧车道
                        if (one_next_lane_ele.first_split_index == -1) {
                            one_next_lane_ele.first_split_index = one_next_lane_ele.lane_splits.size() - 1; //如果是第一个分歧车道，则记录该车道组的分歧索引
                        }
                    }

                    one_ele_group.push_back(one_next_lane_ele); //将新增的车道组添加到车道组中
                    group_number += 1; //更新车道组数量
                }else{
                    //第一个车道分歧做法
                   //如果最后一个link和当前link相同，则不需要添加link_path_ids
                    if (one_lane_ele.link_path_ids.back() != next_link_info.path_offset_id_) {
                        one_lane_ele.link_path_ids.push_back(next_link_info.path_offset_id_);
                    }
                    one_lane_ele.lane_in_link.push_back(one_lane_ele.link_path_ids.size() - 1);
                    one_lane_ele.lane_ids.push_back(next_lane_info.lane_id);
                    one_lane_ele.lane_nums.push_back(next_lane_info.lane_num);
                    
                    //转换merge类型
                    EfmMergeType next_merge_type = TransMergeType(next_lane_info.lane_change_type, next_lane_info);
                    one_lane_ele.lane_merges.push_back(next_merge_type);

                    //转换split类型
                    EfmSplitType next_split_type = TransSplitType(next_lane_info.lane_change_type, next_lane_info);
                    one_lane_ele.lane_splits.push_back(next_split_type);

                    //转换lane类型
                    LaneType lane_type = TransLaneType(next_lane_info.lane_types[0], next_link_info.fow_);
                    one_lane_ele.lane_types.push_back(lane_type);

                    //获取lane的起点和终点相对于自车的距离
                    one_lane_ele.lane_s_offsets.push_back(one_lane_ele.lane_e_offsets.back()); //起点相对于自车的距离，单位cm
                    one_lane_ele.lane_e_offsets.push_back(one_lane_ele.lane_e_offsets.back() + next_lane_info.length); //终点点相对于自车的距离，单位cm

                    //如果车道类型是应急车道或者应急停车带，则该车道组为不可行驶车道
                    bool on_route = true;
                    //if (false == next_lane_info.is_route_lane || lane_type == LaneType::EnumLaneType_EMERGENCY_ || 
                    if (lane_type == LaneType::EnumLaneType_EMERGENCY_ ||
                        lane_type == LaneType::EnumLaneType_EMERGENCY_PARKING_STRIP_) {
                        on_route = false; //如果不是导航车道或者是应急停车带，则该车道组为不可行驶车道
                    }
                    one_lane_ele.lane_routes.push_back(on_route);
                    if (DEBUG_FLAG == 1)
                    {
                        LOG_DEBUG("SdMapScene::MakeSDGroup: lane_s_offset: " + std::to_string(one_lane_ele.lane_s_offsets.back()));
                        LOG_DEBUG("SdMapScene::MakeSDGroup: lane_e_offset: " + std::to_string(one_lane_ele.lane_e_offsets.back()));
                        LOG_DEBUG("lane_info.is_route_lane: " + std::to_string(lane_info.is_route_lane));
                        LOG_DEBUG("one_next_lane_ele.all_length: " + std::to_string(one_lane_ele.all_length));
                        LOG_DEBUG("one_next_lane_ele.remain_dist: " + std::to_string(one_lane_ele.remain_dist));
                        LOG_DEBUG("next_lane_info.lane_type: " + std::to_string(lane_type));
                        LOG_DEBUG("one_next_lane_ele.lane_merges: " + std::to_string(next_merge_type));
                        LOG_DEBUG("one_next_lane_ele.lane_splits: " + std::to_string(next_split_type));

                    }
                    if (on_route && one_lane_ele.all_length == one_lane_ele.remain_dist) {
                        one_lane_ele.remain_dist += next_lane_info.length; //计算剩余距离
                    }

                    one_lane_ele.all_length += next_lane_info.length; //计算该车道组的总长度

                    //判断one_merge_type是否为 EFM_MergeType_TO_LEFT、EFM_MergeType_TO_RIGHT
                    if (next_merge_type == EfmMergeType::EFM_MergeType_TO_LEFT || 
                        next_merge_type == EfmMergeType::EFM_MergeType_TO_RIGHT) {
                        one_lane_ele.merge_count += 1; //如果是合流车道，则合流数量为1
                        //如果first_merge_index为-1，则表示该车道组是第一个合流车道
                        if (one_lane_ele.first_merge_index == -1) {
                            one_lane_ele.first_merge_index = one_lane_ele.lane_merges.size() - 1; //如果是第一个合流车道，则记录该车道组的合流索引
                        }
                    }

                    //判断是否为EFM_SplitType_FROM_LEFT、EFM_SplitType_FROM_RIGHT、EFM_SplitType_SPLIT_FROM_LEFT、EFM_SplitType_SPLIT_FROM_RIGHT
                    if (next_split_type == EfmSplitType::EFM_SplitType_FROM_LEFT || 
                        next_split_type == EfmSplitType::EFM_SplitType_FROM_RIGHT) {
                        //如果first_split_index为-1，则表示该车道组是第一个分歧车道
                        if (one_lane_ele.first_split_index == -1) {
                            one_lane_ele.first_split_index = one_lane_ele.lane_splits.size() - 1; //如果是第一个分歧车道，则记录该车道组的分歧索引
                        }
                    }
                }
                is_update = true; //需要更新车道组
            }

            if (!is_update){
                one_lane_ele.is_end = true; //标记该车道组为结束
                group_index += 1; //如果没有更新车道组，需要更新下一个车道
            }
        }
    } catch (const std::exception& e) {
        LOG_ERROR("SdMapScene::GetNextLane: " + std::string(e.what()));
        return false;
    }
    return true; 
}

bool SdMapScene::GetRefEleForGroup(SDLaneElementGroup& one_ele_group, std::vector<int32_t>& ref_idxs) {
    try {
        if (one_ele_group.empty()) {
            return false; //如果车道组为空，返回false
        }

        //用于存储根据距离做成推荐车道的索引
        std::vector<int32_t> reg_ele_max_remain_dist_indexs; 
        {//通过剩余距离判断
            //获取最大剩余距离
            uint32_t max_remain_dist = one_ele_group[0].remain_dist;
            //遍历车道组，根据remain_dist最大做成推荐车索引
            for (int i = 0; i < one_ele_group.size(); ++i) {
                if (one_ele_group[i].remain_dist == max_remain_dist) { //如果剩余距离等于最大剩余距离，则认为是推荐车道
                    reg_ele_max_remain_dist_indexs.push_back(i);
                } else {
                    break; //如果剩余距离小于最大剩余距离，则认为不是推荐车道，直接终止 
                }
            }

            //判断推荐车道索引是否为空
            if (reg_ele_max_remain_dist_indexs.empty()) {
                ref_idxs.push_back(0); //如果没有推荐车道，返
                return true; 
            }
            //如果推荐车道索引数量为1，则认为该车道组是目标车道
            if (reg_ele_max_remain_dist_indexs.size() == 1) {
                ref_idxs.push_back(reg_ele_max_remain_dist_indexs[0]); //将推荐车道索引添加到ref_idxs中
                return true; //如果只有一个推荐车道，返回true
            }
        }
        //获取最小步数到目标车道的索引
        std::vector<int32_t> reg_ele_min_step_to_dest_lane_indexs;
        {//通过到目标车道的步数判断
            uint64_t last_link_path_id = one_ele_group[reg_ele_max_remain_dist_indexs[0]].link_path_ids.back(); //获取最后一个link_path_id
            //判断最后一个link_path_id是否为main_path_links_最后一个link
            if (last_link_path_id == main_path_links_.back()) {
                reg_ele_min_step_to_dest_lane_indexs = reg_ele_max_remain_dist_indexs; //如果最后一个link_path_id是main_path_links_的最后一个link，则认为推荐车道索引就是最大剩余距离的索引
            }else{
                //最小步数变量
                uint32_t min_step_to_dest_lane = UINT32_MAX; //初始化为最大值
                //遍历main_path_links_,获取last_link_path_id的索引
                auto it = std::find(main_path_links_.begin(), main_path_links_.end(), last_link_path_id);
                if (it == main_path_links_.end()) {
                    //数据异常，不进行变道距离计算
                    reg_ele_min_step_to_dest_lane_indexs = reg_ele_max_remain_dist_indexs;
                } else {
                    uint32_t next_link_index = std::distance(main_path_links_.begin(), it) + 1; //获取最后一个link的下一个link的索引
                    std::vector<uint8_t> connect_lanes; //用于存储连接车道
                    //获取last_link_path_id与下一个link的连接车道
                    if (false == GetConnectToNextLinkLanes(last_link_path_id, main_path_links_[next_link_index], connect_lanes)) {
                        LOG_ERROR("SdMapScene::GetRefEleForGroup: GetNextLane failed for last link path id: " + std::to_string(last_link_path_id));
                        reg_ele_min_step_to_dest_lane_indexs = reg_ele_max_remain_dist_indexs;
                    }

                    //判断connect_lanes数量，空则返回最大剩余距离的索引
                    if (connect_lanes.empty()) {
                        reg_ele_min_step_to_dest_lane_indexs = reg_ele_max_remain_dist_indexs; //如果没有连接车道，则认为推荐车道索引就是最大剩余距离的索引
                    } else {
                        //遍历reg_ele_max_remain_dist_indexs，获取最小步数到目标车道的索引
                        for (const auto& idx : reg_ele_max_remain_dist_indexs) {
                            
                            if (one_ele_group[idx].lane_nums.empty()) {
                                LOG_ERROR("SdMapScene::GetRefEleForGroup: lane_nums is empty for index: " + std::to_string(idx));
                                continue; 
                            }
                            uint8_t last_lane_num = one_ele_group[idx].lane_nums.back(); //获取最后一个lane_num
                            uint8_t min_step_to_dest_lane_tmp = UINT8_MAX; //初始化为最大值
                            //遍历连接车道，获取最小步数到目标车道
                            for (const auto& connect_lane : connect_lanes) {
                                uint8_t step_to_dest_lane_tmp = UINT8_MAX; //初始化为最大值
                                if (connect_lane == last_lane_num) {
                                    min_step_to_dest_lane_tmp = 0; //如果连接车道与最后一个lane_num相同，则步数为0
                                    break; //如果步数为0，直接跳出循环
                                } else if (connect_lane > last_lane_num) {
                                    step_to_dest_lane_tmp = connect_lane - last_lane_num; //如果连接车道大于最后一个lane_num，则步数为连接车道与最后一个lane_num的差值
                                } else {
                                    step_to_dest_lane_tmp = last_lane_num - connect_lane; //如果连接车道小于最后一个lane_num，则步数为最后一个lane_num与连接车道的差值
                                }
                                if (step_to_dest_lane_tmp < min_step_to_dest_lane_tmp) {
                                    min_step_to_dest_lane_tmp = step_to_dest_lane_tmp; //更新最小步数到目标车道
                                }
                            }
                            one_ele_group[idx].step_to_dest = min_step_to_dest_lane_tmp; //更新车道组的最小步数到目标车道

                            //min_step_to_dest_lane_tmp与min_step_to_dest_lane比较
                            if (min_step_to_dest_lane_tmp < min_step_to_dest_lane) {
                                min_step_to_dest_lane = min_step_to_dest_lane_tmp;
                            }
                        }

                        //遍历reg_ele_max_remain_dist_indexs，获取最小步数到目标车道的索引
                        for (const auto& idx : reg_ele_max_remain_dist_indexs) {
                            if (one_ele_group[idx].step_to_dest == min_step_to_dest_lane) {
                                reg_ele_min_step_to_dest_lane_indexs.push_back(idx); //如果步数到目标车道相同，则添加到最小步数到目标车道的索引中
                            }
                        }

                        //判断推荐车道索引是否为空
                        if (reg_ele_min_step_to_dest_lane_indexs.empty()) {
                            ref_idxs.push_back(0); //如果没有推荐车道，返回第一个车道
                            return true; 
                        }

                        //如果推荐车道索引数量为1，则认为该车道组是目标车道
                        if (reg_ele_min_step_to_dest_lane_indexs.size() == 1) {
                            ref_idxs.push_back(reg_ele_min_step_to_dest_lane_indexs[0]); //将推荐车道索引添加到ref_idxs中
                            return true; //如果只有一个推荐车道，返回true
                        }
                    }
                }
            }

        }

        std::vector<int32_t> reg_ele_min_merge_count_indexs; //最小merge的索引
        {//通过merge次数判断
            uint32_t min_merge_count = UINT32_MAX; //初始化为最大值
            //根据merge属性进行判断，merge的次数
            for (const auto& idx : reg_ele_max_remain_dist_indexs) {
                if (one_ele_group[idx].merge_count < min_merge_count) {
                    min_merge_count = one_ele_group[idx].merge_count; //更新最小的merge的次数
                } 
            }
            //遍历推荐车道索引，根据merge的次数进行判断
            for (const auto& idx : reg_ele_min_step_to_dest_lane_indexs) {
                if (one_ele_group[idx].merge_count == min_merge_count) {
                    reg_ele_min_merge_count_indexs.push_back(idx); //如果merge的次数相同，则添加到最小merge的索引中
                }
            }

            //判断推荐车道索引是否为空
            if (reg_ele_min_merge_count_indexs.empty()) {
                ref_idxs.push_back(0); //如果没有推荐车道，返回第一个车道
                return true; 
            }
            //如果推荐车道索引数量为1，则认为该车道组是目标车道
            if (reg_ele_min_merge_count_indexs.size() == 1) {
                ref_idxs.push_back(reg_ele_min_merge_count_indexs[0]); //将推荐车道索引添加到ref_idxs中
                return true; //如果只有一个推荐车道，返回true
            }
        }

        {//通过split属性，进行推荐车道排序
            //获取最小的first_split_index
            uint32_t min_split_idx = UINT32_MAX; //初始化为最大值
            for (const auto& idx : reg_ele_min_merge_count_indexs) {
                if (one_ele_group[idx].lane_splits.size() < min_split_idx) {
                    min_split_idx = one_ele_group[idx].first_split_index; //更新最小的split的次数
                }
            }

            //遍历推荐车道索引，与min_split_idx相等，优先放入推荐车道索引
            for (const auto& idx : reg_ele_min_merge_count_indexs) {
                if (one_ele_group[idx].first_split_index == min_split_idx) {
                    ref_idxs.push_back(idx); //如果split的次数相同，优先放入推荐车道索引
                }
            }

            //其他reg_ele_min_merge_count_indexs放入推荐车道索引
            for (const auto& idx : reg_ele_min_merge_count_indexs) {
                if (std::find(ref_idxs.begin(), ref_idxs.end(), idx) == ref_idxs.end()) {
                    ref_idxs.push_back(idx); //如果不在推荐车道索引中，则添加到推荐车道索引中
                }
            }
        }
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR("SdMapScene::GetRefEleForGroup: " + std::string(e.what()));
        return false;
    }
}

bool SdMapScene::GetConnectToNextLinkLanes(uint64_t last_link_path_id, uint64_t next_link_path_id, std::vector<uint8_t>& connect_lanes) {
    //获取last_link_path_id对应的LinkLaneInfo列表
    if (sd_index_.link_path_offset_id_index_map.find(last_link_path_id) == sd_index_.link_path_offset_id_index_map.end()) {
        LOG_ERROR("SdMapScene::GetConnectToNextLinkLanes: last link path id not found in index map: " + std::to_string(last_link_path_id));
        return false; //如果last_link_path_id不在索引中，返回false
    }
    auto& link_lane_info = links_.ehp_output_link_list[sd_index_.link_path_offset_id_index_map[last_link_path_id]];

    //获取next_link_path_id对应的LinkLaneInfo列表
    if (sd_index_.link_path_offset_id_index_map.find(next_link_path_id) == sd_index_.link_path_offset_id_index_map.end()) {
        LOG_ERROR("SdMapScene::GetConnectToNextLinkLanes: next link path id not found in index map: " + std::to_string(next_link_path_id));
        return false; //如果next_link_path_id不在索引中，返回false
    }
    auto& next_link_lane_info = links_.ehp_output_link_list[sd_index_.link_path_offset_id_index_map[next_link_path_id]];

    //遍历link_lane_info中的车道信息，查找与next_link_lane_info的连接车道
    for (const auto& lane_info : link_lane_info.link_lane_info_list_) {
        //如果车道的next_lane_ids中包含next_link_path_id对应的车道id，则认为是连接车道
        for (auto& next_lane_id : lane_info.next_lane_ids) {
            //判断next_lane_id是否在next_link_lane_info的车道id列表中
            if (std::find_if(next_link_lane_info.link_lane_info_list_.begin(), next_link_lane_info.link_lane_info_list_.end(), 
                            [next_lane_id](LinkLaneInfo a) { return a.lane_id == next_lane_id; }) != next_link_lane_info.link_lane_info_list_.end()) {
                connect_lanes.push_back(lane_info.lane_num); //如果连接车道，则添加到connect_lanes中
            }
        }
    }
    return true;
}

} // namespace NoMapEFM
