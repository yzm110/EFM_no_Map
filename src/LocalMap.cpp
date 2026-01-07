#include "LocalMap.h"
#include "log_manager.h"

#define SPECIAL_DIS_CAR 20000  //cm
#define OUT_POSITION_STEP 250  //cm
#define DEBUG_FLAG 0    

namespace NoMapEFM{

bool LocalMap::Execute(BevLaneElementGroupSet& bev_lane_group_set, SDLaneElementGroupSet& ele_group, 
                       NodeInfo& node_info, SEhpOutputLoc& lane_loc, SEhpOutputPathList& paths) {
    try
    {
        //保存上一帧边线和中心线信息，和本帧数据对比，以保证中心线不产生很大的跳动
        //该逻辑为预留，待后续优化
        static BevLaneElementGroupSet last_BevLaneElementGroupSet = BevLaneElementGroupSet{};

        node_info_ = NodeInfo{};
        prior_dir_ = 0;
        lane_loc_ = lane_loc;
        left_lane_idx_= LaneIdx{};
        right_lane_idx_ = LaneIdx{};
        left_left_lane_idx_ = LaneIdx{};
        right_right_lane_idx_ = LaneIdx{};
        ref_lane_idx_ = LaneIdx{};
        ego_lane_idx_ = LaneIdx{};

        InitBevEle(bev_lane_group_set);
        paths_ = paths;

        // //遍历ele_group，打印
        if (DEBUG_FLAG == 1)
        {
            for (const auto& pair : ele_group) {
                LOG_DEBUG("pair.second[0].lane_ids[0]: " + std::to_string(pair.second[0].lane_ids[0]));
                LOG_DEBUG("ele_group: " + std::to_string(pair.first) + ", size: " + std::to_string(pair.second.size()));
                for (const auto& ele : pair.second) {
                    LOG_DEBUG("ele.remain_dist: " + std::to_string(ele.remain_dist));
                    LOG_DEBUG("ele.lane_nums.size: " + std::to_string(ele.lane_nums.size()));
                    LOG_DEBUG("ele.is_group_dest: " + std::to_string(ele.is_group_dest));
                    LOG_DEBUG("ele.is_dest: " + std::to_string(ele.is_dest));
                    std::string lane_nums_str = "lane_nums: ";
                    for (const auto& lane_num : ele.lane_nums) {
                        lane_nums_str += std::to_string(lane_num) + ", ";
                    }
                    LOG_DEBUG(lane_nums_str);
                }
            }
            LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
            LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
        }



        //如果bev_lane_group_set为空，和英箭确认是否使用上一帧数据？
        // if (bev_lane_group_set.empty()){
        //     std::cout << __FILE__ << "," << __LINE__ << "," << "bev_lane_group_set is empty" << std::endl;
        //     return false;
        // }

        // 该部分移植到LaneModel，正民做成
        // static LaneIdx last_ego_lane_idx_ = LaneIdx{};
        // //判断自车所在车道
        // if (false == GetEgoLaneIdx(bev_lane_group_set, ele_group)){
        //     if (false == GetEgoLaneIdxForLastLane(bev_lane_group_set, ele_group, last_ego_lane_idx_)){
        //         return false;
        //     }
            
        // }
        // last_ego_lane_idx_ = ego_lane_idx_;
        
        //做成推荐信息
        if (false == MakeNodeInfoForChangeLane(ele_group)){
            return false;
        }
        //遍历node_info_，打印
        // LOG_DEBUG("node_info.StartPointOffset: " + std::to_string(node_info_.StartPointOffset));
        // LOG_DEBUG("node_info.EndPointOffset: " + std::to_string(node_info_.EndPointOffset));
        // LOG_DEBUG("node_info.dir: " + std::to_string(node_info_.dir));
        // LOG_DEBUG("node_info.LaneChgType: " + std::to_string(node_info_.LaneChgType));
        // LOG_DEBUG("node_info.LaneChgTimes: " + std::to_string(node_info_.LaneChgTimes));

        // LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
        // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
        
        for (int group_idx = 0; group_idx < bev_lane_group_set.size(); ++group_idx){
            //做成左右车道线重叠的部分
            if (false == MakeOverlapLane(bev_lane_group_set[group_idx].second)){
                LOG_ERROR("MakeOverlapLane error! error idx: " + std::to_string(bev_lane_group_set[group_idx].first));
                continue; //如果车道组为空，跳过
            }
            // std::string left_points_strx = "[";
            // std::string left_points_stry = "[";
            // for (const auto& point : bev_lane_group_set[group_idx].second[0].left_line_points) {
            //     left_points_strx += std::to_string(point.x) +  ",";
            //     left_points_stry += std::to_string(point.y) + ",";
            // }
            // LOG_DEBUG(left_points_strx + "]");
            // LOG_DEBUG(left_points_stry + "]");

            // //遍历right_points，打印
            // std::string right_points_strx = "[";
            // std::string right_points_stry = "[";
            // for (const auto& point : bev_lane_group_set[group_idx].second[0].right_line_points) {
            //     right_points_strx += std::to_string(point.x) +  ",";
            //     right_points_stry += std::to_string(point.y) + ",";
            // }
            // LOG_DEBUG(right_points_strx + "]");
            // LOG_DEBUG(right_points_stry + "]");

            // LOG_DEBUG("idx: " + std::to_string(bev_lane_group_set[group_idx].first));
            // LOG_DEBUG("bev_lane_group_set[group_idx].second.size: " + std::to_string(bev_lane_group_set[group_idx].second.size()));
            if (false == MakeCenterLine(bev_lane_group_set[group_idx].first, bev_lane_group_set[group_idx].second)){
                //未分配数据
                LOG_ERROR("MakeCenterLine error! error idx: " + std::to_string(bev_lane_group_set[group_idx].first));
                continue;
            }
            //LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));

            //车道线中心线生成后，判断与路沿、车道线的距离，如果不满足KPI，需要修正
            bool center_line_valid = false;
            std::vector<int32_t> target_line = {};
            if (false == CheckCenterLine(bev_lane_group_set[group_idx].second, target_line, center_line_valid)){
                LOG_ERROR("CheckCenterLine error! error idx: " + std::to_string(bev_lane_group_set[group_idx].first));
                return false;
            }
            //LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));

            if (center_line_valid){
                if (false == ChangeCenterLine(bev_lane_group_set[group_idx].second, target_line)){
                    LOG_ERROR("ChangeCenterLine error! error idx: " + std::to_string(bev_lane_group_set[group_idx].first));
                    return false;
                }
            }
            //LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));

            //对边线和中心进行重新采样
            // for (auto& bev_ele : bev_lane_group_set[group_idx].second){
            //     {
                    //遍历bev_ele.left_line_points，打印
                    // std::string left_points_strx = "[";
                    // std::string left_points_stry = "[";
                    // for (const auto& point : bev_ele.left_line_points) {
                    //     left_points_strx += std::to_string(point.x) +  ",";
                    //     left_points_stry += std::to_string(point.y) + ",";
                    // }
                    // LOG_DEBUG(left_points_strx + "]");
                    // LOG_DEBUG(left_points_stry + "]");

                    // //遍历right_points，打印
                    // std::string right_points_strx = "[";
                    // std::string right_points_stry = "[";
                    // for (const auto& point : bev_ele.right_line_points) {
                    //     right_points_strx += std::to_string(point.x) +  ",";
                    //     right_points_stry += std::to_string(point.y) + ",";
                    // }
                    // LOG_DEBUG(right_points_strx + "]");
                    // LOG_DEBUG(right_points_stry + "]");

                    //遍历center_line_points，打印
                    // std::string center_line_points_strx2 = "[";
                    // std::string center_line_points_stry2 = "[";
                    // for (const auto& point : bev_ele.center_line_points) {
                    //     center_line_points_strx2 += std::to_string(point.x) +  ",";
                    //     center_line_points_stry2 += std::to_string(point.y) + ",";
                    // }
                    // LOG_DEBUG(center_line_points_strx2 + "]");
                    // LOG_DEBUG(center_line_points_stry2 + "]");
                // }

                // if (false == ReSamplingPoint(bev_ele)){
                //     LOG_ERROR("ReSamplingPoint error! error idx: " + std::to_string(bev_lane_group_set[group_idx].first));
                //     return false;
                // }

                // {
                //     //遍历bev_ele.left_line_points，打印
                //     std::string left_points_strx = "[";
                //     std::string left_points_stry = "[";
                //     for (const auto& point : bev_ele.left_line_points) {
                //         left_points_strx += std::to_string(point.x) +  ",";
                //         left_points_stry += std::to_string(point.y) + ",";
                //     }
                //     LOG_DEBUG(left_points_strx + "]");
                //     LOG_DEBUG(left_points_stry + "]");

                //     //遍历right_points，打印
                //     std::string right_points_strx = "[";
                //     std::string right_points_stry = "[";
                //     for (const auto& point : bev_ele.right_line_points) {
                //         right_points_strx += std::to_string(point.x) +  ",";
                //         right_points_stry += std::to_string(point.y) + ",";
                //     }
                //     LOG_DEBUG(right_points_strx + "]");
                //     LOG_DEBUG(right_points_stry + "]");

                    //遍历center_line_points，打印
                    // std::string center_line_points_strx2 = "[";
                    // std::string center_line_points_stry2 = "[";
                    // for (const auto& point : bev_ele.center_line_points) {
                    //     center_line_points_strx2 += std::to_string(point.x) +  ",";
                    //     center_line_points_stry2 += std::to_string(point.y) + ",";
                    // }
                    // LOG_DEBUG(center_line_points_strx2 + "]");
                    // LOG_DEBUG(center_line_points_stry2 + "]");
                // }

            // }
            // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
        }
        // LOG_DEBUG("node_info.StartPointOffset: " + std::to_string(node_info_.StartPointOffset));
        // LOG_DEBUG("node_info.EndPointOffset: " + std::to_string(node_info_.EndPointOffset));
        // LOG_DEBUG("node_info.dir: " + std::to_string(node_info_.dir));
        // LOG_DEBUG("node_info.LaneChgType: " + std::to_string(node_info_.LaneChgType));
        // LOG_DEBUG("node_info.LaneChgTimes: " + std::to_string(node_info_.LaneChgTimes));
        //LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
        // LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
        // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
        //做成自车道，左右车道等信息，更新node信息
        //LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
        if (false == MakeRefLine(bev_lane_group_set, ele_group)){
            LOG_ERROR("MakeRefLine error");
            return false;
        }
        for (int group_idx = 0; group_idx < bev_lane_group_set.size(); ++group_idx){
            for (auto& bev_ele : bev_lane_group_set[group_idx].second){
                if (false == ReSamplingPoint(bev_ele)){
                    LOG_ERROR("ReSamplingPoint error! error idx: " + std::to_string(bev_lane_group_set[group_idx].first));
                    return false;
                }
            }
        }
        // LOG_DEBUG("ego_lane_idx_.sd_lane_idx_.first: " + std::to_string(ego_lane_idx_.sd_lane_idx_.first));
        // LOG_DEBUG("ego_lane_idx_.sd_lane_idx_.second: " + std::to_string(ego_lane_idx_.sd_lane_idx_.second));
        // LOG_DEBUG("ego_lane_idx_.bev_lane_idx_.first: " + std::to_string(ego_lane_idx_.bev_lane_idx_.first));
        // LOG_DEBUG("ego_lane_idx_.bev_lane_idx_.second: " + std::to_string(ego_lane_idx_.bev_lane_idx_.second));
        // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));

        node_info = node_info_;
        // LOG_DEBUG("node_info.StartPointOffset: " + std::to_string(node_info_.StartPointOffset));
        // LOG_DEBUG("node_info.EndPointOffset: " + std::to_string(node_info_.EndPointOffset));
        // LOG_DEBUG("node_info.dir: " + std::to_string(node_info_.dir));
        // LOG_DEBUG("node_info.LaneChgType: " + std::to_string(node_info_.LaneChgType));
        // LOG_DEBUG("node_info.LaneChgTimes: " + std::to_string(node_info_.LaneChgTimes));
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::Execute: " + std::string(e.what()));
        return false;
    }
    // LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
    // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
    // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
    return true;
}

bool LocalMap::ReSamplingPoint(BevLaneElement& bev_ele){
    try
    {
        ReSamplingPointOne(bev_ele.left_line_points);
        ReSamplingPointOne(bev_ele.right_line_points);
        ReSamplingPointOne(bev_ele.center_line_points);
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::ReSamplingPoint: " + std::string(e.what()));
        return false;
    }

    return true;
}

//ReSamplingPointOne
void LocalMap::ReSamplingPointOne(EFMRefLinePoints& line_points){
    try
    {
        if (line_points.empty()){
            return; //如果线点为空，直接返回
        }

        //重新采样点
        EFMRefLinePoints new_line_points;
        double arc_length = 0.0;
        std::vector<double> arcs;
        arcs.push_back(arc_length);
        for (int i = 1; i < line_points.size(); ++i) {
            double dx = line_points[i].x - line_points[i - 1].x;
            double dy = line_points[i].y - line_points[i - 1].y;
            arc_length += std::hypot(dx, dy);
            arcs.push_back(arc_length);
        }

        for (double s = 0; s <= arc_length; s += 2.5) {
            EFMPoint point = interpolateAtLength(line_points, arcs, s);
            new_line_points.push_back(point);
        }
        
        line_points = new_line_points; //更新线点
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::ReSamplingPointOne: " + std::string(e.what()));
    }
}

void LocalMap::InitBevEle(BevLaneElementGroupSet& bev_lane_group_set){
    try
    {
        //清空
        bev_ego_ele_.clear();
        bev_left_ele_.clear();
        bev_right_ele_.clear();
        bev_left_left_ele_.clear();
        bev_right_right_ele_.clear();

        //遍历bev_lane_group_set，获取自车所在车道的车道组
        for (const auto& group : bev_lane_group_set){
            if (group.second.empty()){
                continue; //如果车道组为空，跳过
            }

            if (group.first == 0){
                //自车所在车道
                bev_ego_ele_ = group.second;
            }else if (group.first == 1){
                //左侧车道
                bev_left_ele_ = group.second;
            }else if (group.first == 2){
                //右侧车道
                bev_right_ele_ = group.second;
            }else if (group.first == 3){
                //左侧左侧车道
                bev_left_left_ele_ = group.second;
            }else if (group.first == 4){
                //右侧右侧车道
                bev_right_right_ele_ = group.second;
            }
        }
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::InitBevEle: " + std::string(e.what()));
        return ;
    }

    return ;
}

bool LocalMap::GetEgoLaneIdx(const BevLaneElementGroupSet& bev_lane_group_set, SDLaneElementGroupSet& ele_group){
    try
    {
        //判断bev_lane_group_set与ele_group数量是否为空
        if (bev_lane_group_set.empty() || ele_group.empty()){
            LOG_ERROR("bev_lane_group_set or ele_group is empty");
            return false;
        }
        
        {//通过左右路沿判断自车所在车道
            //如果bev_lane_group_set的第0个元素的左右边线都为空，则认为自车所在车道不存在
            if (bev_lane_group_set[0].second.empty() || 
                bev_lane_group_set[0].second[0].left_line_points.empty() || 
                bev_lane_group_set[0].second[0].right_line_points.empty()){
                LOG_ERROR("bev_lane_group_set[0] is empty");
                return false;
            }

        }

    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetEgoLaneIdx: " + std::string(e.what()));
        return false;
    }

    return true;
}

bool LocalMap::MakeNodeInfoForChangeLane(SDLaneElementGroupSet& ele_group){
    try
    {
        //获取lane_loc_.lane_id_ 所在的车道组
        if (ele_group.empty() || lane_loc_.lane_id_ == 0){
            LOG_ERROR("ele_group is empty or lane_id_ is 0");
            return false;
        }

        uint8_t ego_lane_number = 0; //自车所在车道的数量
        std::vector<std::vector<uint32_t>> dest_lane_ele_idxs; //目标车道的索引
        SDLaneElement ego_ele = {};
        //遍历车道组，找到自车所在的车道组
        for (const auto& group : ele_group){
            if (group.second.empty() || group.second[0].lane_ids.empty()){
                continue; //如果车道组为空，跳过
            }

            if (group.second[0].lane_ids[0] == lane_loc_.lane_id_){
                //找到自车所在的车道组
                ego_lane_number = group.first;
                ego_sd_group_ = group.second; //保存自车所在的车道组
                //遍历车道组，获取自车所在车道
                for (size_t i = 0; i < group.second.size(); i++){
                    if (group.second[i].is_group_dest == true){
                        ego_ele = group.second[i]; //获取自车所在车道
                        break;
                    }
                }
            }

            //遍历车道组，如果车道组的is_dest为true，则认为该车道组是目标车道
            for (size_t i = 0; i < group.second.size(); i++){
                if (group.second[i].is_dest){
                    //如果车道组的is_dest为true，则认为该车道组是目标车道
                    std::vector<uint32_t> dest_lane_idxs;
                    dest_lane_idxs.push_back(group.first);
                    dest_lane_idxs.push_back(i);
                    dest_lane_ele_idxs.push_back(dest_lane_idxs);
                    break;
                }
            }
        }

        if (ego_lane_number == 0 || ego_ele.lane_ids.empty()){
            LOG_ERROR("ego_lane_number is 0");
            return false; //如果没有找到自车所在的车道组，返回false
        }

        //判断自车所在车道是否在目标车道中
        auto it = std::find_if(dest_lane_ele_idxs.begin(), dest_lane_ele_idxs.end(), 
                               [ego_lane_number](const std::vector<uint32_t>& lane_idxs) {
                                   return lane_idxs[0] == ego_lane_number;
                               });
        if (it != dest_lane_ele_idxs.end()){
            //如果自车所在车道在目标车道中，则认为自车所在车道是目标车道
            return true; 
        }else{
            //如果自车所在车道不在目标车道中，获取目标车道列表
            if (dest_lane_ele_idxs.empty()){
                LOG_ERROR("dest_lane_ele_idxs is empty");
                return false; //如果目标车道列表为空，返回false
            }
            //遍历目标车道，获取距离自车最近的目标车道
            uint8_t left_nearest_lane_number = UINT8_MAX; //左侧最近的目标车道
            uint8_t right_nearest_lane_number = 0; //右侧最近的目标车道
            for (const auto& dest_lane_idx : dest_lane_ele_idxs){
                if (dest_lane_idx[0] > ego_lane_number && dest_lane_idx[0] < left_nearest_lane_number){
                    //如果目标车道在自车左侧且小于当前左侧最近的目标车道，则更新左侧最近的目标车道
                    left_nearest_lane_number = dest_lane_idx[0];
                }else if (dest_lane_idx[0] < ego_lane_number && dest_lane_idx[0] > right_nearest_lane_number){
                    //如果目标车道在自车右侧且大于当前右侧最近的目标车道，则更新右侧最近的目标车道
                    right_nearest_lane_number = dest_lane_idx[0];
                }
            }

            //判断到达左侧最近的目标车道的距离
            uint8_t left_change_distance = UINT8_MAX; //左侧变道距离
            if (left_nearest_lane_number < UINT8_MAX){
                //如果左侧最近的目标车道不为最大值，则认为可以变道到左侧最近的目标车道
                //LOG_DEBUG("ego_lane_number: " + std::to_string(ego_lane_number));
                left_change_distance = left_nearest_lane_number - ego_lane_number;
            }
            //LOG_DEBUG("left_change_distance: " + std::to_string(left_change_distance));

            //判断到达右侧最近的目标车道的距离
            uint8_t right_change_distance = UINT8_MAX; //右侧变道距离
            if (right_nearest_lane_number > 0){
                //如果右侧最近的目标车道大于0，则认为可以变道到右侧最近的目标车道
                //LOG_DEBUG("ego_lane_number: " + std::to_string(ego_lane_number));
                right_change_distance = ego_lane_number - right_nearest_lane_number;
            }
            //LOG_DEBUG("right_change_distance: " + std::to_string(right_change_distance));

            //判断左侧变道距离和右侧变道距离，选择最小的变道距离
            if (left_change_distance <= right_change_distance){
                //如果左侧变道距离小于等于右侧变道距离，则认为可以变道到左侧最近的目标车道
                node_info_.dir = NodeDir::NODE_DIR_LEFT; //变道方向为左侧
                node_info_.LaneChgType = 3; //bit0- with control, bit1- with light
                node_info_.StartPointOffset = 0;
                node_info_.EndPointOffset =  static_cast<double>(ego_ele.remain_dist) / 100.0;//需要根据自车是否存在merge修改
                node_info_.LaneChgTimes = left_change_distance; //变道次数为左侧变道距离
                prior_dir_ = 1; //变道方向为左侧
            }else{
                //如果右侧变道距离小于左侧变道距离，则认为可以变道到右侧最近的目标车道
                node_info_.dir = NodeDir::NODE_DIR_RIGHT; //变道方向为右侧
                node_info_.LaneChgType = 3; //bit0- with control, bit1- with light
                node_info_.StartPointOffset = 0;
                node_info_.EndPointOffset = static_cast<double>(ego_ele.remain_dist) / 100.0;//需要根据自车是否存在merge修改
                node_info_.LaneChgTimes = right_change_distance; //变道次数为右侧变道距离
                prior_dir_ = 2; //变道方向为右侧
            }
        }
        
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::MakeNodeInfo: " + std::string(e.what()));
        return false;
    }

    return true;
}

bool LocalMap::MakeCenterLineCommon(BevLaneElement& bev_ele){
    try
    {
        //如果bev_ele的左右边线都为空，则认为该车道没有中心线
        if (bev_ele.left_line_points.empty() || bev_ele.right_line_points.empty()){
            LOG_ERROR("bev_ele left or right line points is empty");
            return false;
        }

        //生成中心线
        bev_ele.center_line_points = GenerateCenterLine(bev_ele.left_line_points, bev_ele.right_line_points, 0);
        
        //检查中心线是否有问题
        if (bev_ele.center_line_points.empty()){
            LOG_ERROR("bev_ele center line points is empty");
            return false;
        }

    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::MakeCenterLine: " + std::string(e.what()));
        return false;
    }

    return true;
}

bool LocalMap::MakeCenterLine(int32_t lane_idx, BevLaneElementGroup& bev_ele_group){
    try
    {
        //为空直接返回
        if (bev_ele_group.empty()) return false;

        if (!(0 == lane_idx || 1 == lane_idx || 2 == lane_idx)){
            //如果车道索引不在0,1,2中，则认为不在自车、左侧、右侧车道中，暂时不处理
            return false;
        }
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("MakeCenterLine: lane_idx: " + std::to_string(lane_idx));
            LOG_DEBUG("MakeCenterLine: bev_ele_group.size: " + std::to_string(bev_ele_group.size()));
        }
        //自车后方中心线做成，后面优化

        //自车前方做成
        if (1 == bev_ele_group.size()){
            uint8_t merge_dir = 0;//0: 非merge; 1: 向左merge；2：向右merge
            int32_t left_merge_point = -1;
            int32_t right_merge_point = -1;
            int32_t left_merge_narrow_pos = -1;
            int32_t right_merge_narrow_pos = -1;

            // //遍历left_points，打印
            // std::string left_points_strx = "[";
            // std::string left_points_stry = "[";
            // for (const auto& point : bev_ele_group[0].left_line_points) {
            //     left_points_strx += std::to_string(point.x) +  ",";
            //     left_points_stry += std::to_string(point.y) + ",";
            // }
            // LOG_DEBUG(left_points_strx + "]");
            // LOG_DEBUG(left_points_stry + "]");

            // //遍历right_points，打印
            // std::string right_points_strx = "[";
            // std::string right_points_stry = "[";
            // for (const auto& point : bev_ele_group[0].right_line_points) {
            //     right_points_strx += std::to_string(point.x) +  ",";
            //     right_points_stry += std::to_string(point.y) + ",";
            // }
            // LOG_DEBUG(right_points_strx + "]");
            // LOG_DEBUG(right_points_stry + "]");

            if (false == GetMergeDir(bev_ele_group[0], merge_dir, left_merge_point, right_merge_point, 
                                     left_merge_narrow_pos, right_merge_narrow_pos, lane_idx)){
                return false;
            }

            // //遍历left_points，打印
            // std::string left_points_strx1 = "[";
            // std::string left_points_stry1 = "[";
            // for (const auto& point : bev_ele_group[0].left_line_points) {
            //     left_points_strx1 += std::to_string(point.x) +  ",";
            //     left_points_stry1 += std::to_string(point.y) + ",";
            // }
            // LOG_DEBUG(left_points_strx1 + "]");
            // LOG_DEBUG(left_points_stry1 + "]");

            // //遍历right_points，打印
            // std::string right_points_strx1 = "[";
            // std::string right_points_stry1 = "[";
            // for (const auto& point : bev_ele_group[0].right_line_points) {
            //     right_points_strx1 += std::to_string(point.x) +  ",";
            //     right_points_stry1 += std::to_string(point.y) + ",";
            // }
            // LOG_DEBUG(right_points_strx1 + "]");
            // LOG_DEBUG(right_points_stry1 + "]");

            bev_ele_group[0].merge_dir = merge_dir;
            bev_ele_group[0].left_merge_point = left_merge_point;
            bev_ele_group[0].right_merge_point = right_merge_point;
            bev_ele_group[0].left_merge_narrow_pos = left_merge_narrow_pos;
            bev_ele_group[0].right_merge_narrow_pos = right_merge_narrow_pos;
            if (DEBUG_FLAG == 1)
            {
                LOG_DEBUG("merge_dir: " + std::to_string(merge_dir) + ", left_merge_point: " + std::to_string(left_merge_point) +
                          ", right_merge_point: " + std::to_string(right_merge_point) + ", left_merge_narrow_pos: " + std::to_string(left_merge_narrow_pos) +
                          ", right_merge_narrow_pos: " + std::to_string(right_merge_narrow_pos) + ", lane_idx: " + std::to_string(lane_idx) +
                          ", bev_left_ele_.size: " + std::to_string(bev_left_ele_.size()) + ", prior_dir_: " + std::to_string(prior_dir_));
            }

            if (1 == merge_dir && lane_idx == 0 && !bev_left_ele_.empty()){
                //自车道向左 merge
                MakeCenterLineForMerge(bev_ele_group[0], merge_dir, left_merge_point, right_merge_point, 
                                     left_merge_narrow_pos, right_merge_narrow_pos, bev_left_ele_[0]);
            }else if (1 == merge_dir && lane_idx == 2 && !bev_ego_ele_.empty()){
                //右车道向左 merge
                MakeCenterLineForMerge(bev_ele_group[0], merge_dir, left_merge_point, right_merge_point, 
                                     left_merge_narrow_pos, right_merge_narrow_pos, bev_ego_ele_[0]);
            }else if (2 == merge_dir && lane_idx == 0 && !bev_right_ele_.empty()){
                //自车道向右 merge
                MakeCenterLineForMerge(bev_ele_group[0], merge_dir, left_merge_point, right_merge_point, 
                                     left_merge_narrow_pos, right_merge_narrow_pos, bev_right_ele_[bev_right_ele_.size() - 1]);
            }else if (2 == merge_dir && lane_idx == 1 && !bev_ego_ele_.empty()){
                //左车道向右 merge
                MakeCenterLineForMerge(bev_ele_group[0], merge_dir, left_merge_point, right_merge_point, 
                                     left_merge_narrow_pos, right_merge_narrow_pos, bev_ego_ele_[bev_ego_ele_.size() - 1]);
            }else if (lane_idx == 0 && prior_dir_ != 0){
                //无merge，需要向左变道
                MakeCenterLineForChangeLane(bev_ele_group[0], prior_dir_);
            }else{
                // LOG_DEBUG("MakeCenterLine: lane_idx: " + std::to_string(lane_idx));
                // LOG_DEBUG("MakeCenterLine: bev_ele_group[0].left_line_points.size: " + std::to_string(bev_ele_group[0].left_line_points.size()));
                //LOG_DEBUG("MakeCenterLine: bev_ele_group[0].right_line_points.size: " + std::to_string(bev_ele_group[0].right_line_points.size()));
                bev_ele_group[0].center_line_points = GetCenterLineCommon(bev_ele_group[0].lane_width_last_cycle, bev_ele_group[0].left_line_points, bev_ele_group[0].right_line_points, 
                                                                          bev_ele_group[0].left_types, bev_ele_group[0].right_types, 0, 0, lane_idx, lane_idx == 0 ? false : true);
            }
        }else{
            //LOG_DEBUG("MakeCenterLine: lane_idx: " + std::to_string(lane_idx));
            //分歧路中找到含有线的主line
            EFMRefLinePoints main_left_line_points = {};
            EFMRefLinePoints main_right_line_points = {};
            //遍历bev_ele_group
            if (DEBUG_FLAG == 1)
            {
                for (int32_t i = 0; i < bev_ele_group.size(); ++i){
                    const auto& ele = bev_ele_group[i];
                    LOG_DEBUG("MakeCenterLine: i: " + std::to_string(i));
                    //遍历ele.left_line_points
                    std::string left_points_strx = "[";
                    std::string left_points_stry = "[";
                    for (const auto& point : ele.left_line_points) {
                        left_points_strx += std::to_string(point.x) +  ",";
                        left_points_stry += std::to_string(point.y) + ",";
                    }
                    LOG_DEBUG("left ix=" + std::to_string(i)  + ":"+ left_points_strx + "]");
                    LOG_DEBUG("left iy=" + std::to_string(i)  + ":"+ left_points_stry + "]");
                    //遍历ele.right_line_points
                    std::string right_points_strx = "[";
                    std::string right_points_stry = "[";
                    for (const auto& point : ele.right_line_points) {
                        right_points_strx += std::to_string(point.x) +  ",";
                        right_points_stry += std::to_string(point.y) + ",";
                    }
                    LOG_DEBUG("right ix=" + std::to_string(i)  + ":"+ right_points_strx + "]");
                    LOG_DEBUG("right iy=" + std::to_string(i)  + ":"+ right_points_stry + "]");
                }
            }

            bool is_all_wide_lane = false;
            std::vector<LineTypeInfo> main_left_types; //从起点开始
            std::vector<LineTypeInfo> main_right_types; //从起点开始
            uint8_t main_line = 0;
            if (false == GetMainLinePoint(bev_ele_group, main_left_line_points, main_right_line_points, lane_idx, is_all_wide_lane,
                                         main_left_types, main_right_types, main_line)){
                return false;
            }
            if (DEBUG_FLAG == 1)
            {
                LOG_DEBUG("main_line: " + std::to_string(main_line));
                LOG_DEBUG("is_all_wide_lane: " + std::to_string(is_all_wide_lane));
            }
            if (is_all_wide_lane){
                //判断主要道路
                int32_t near_idx = 0;
                if (0 == lane_idx){
                    GetNearIdxForMainLine(bev_ele_group, main_left_line_points, main_right_line_points, near_idx);
                }else if (1 == lane_idx){
                    near_idx = 0;
                }else{
                    near_idx = bev_ele_group.size() -1;
                }
                bev_ele_group[near_idx].is_main_ele = true; //最后一个车道认为是主车道
                if (DEBUG_FLAG == 1)
                {
                    LOG_DEBUG("near_idx: " + std::to_string(near_idx));
                }


                for (int32_t i = 0; i < bev_ele_group.size(); ++i){
                    bool is_neighbor_lane = false;
                    if(near_idx == i){
                        is_neighbor_lane = true;
                    }
                    BevLaneElement& bev_ele = bev_ele_group[i];
                    if (DEBUG_FLAG == 1)
                    {
                        LOG_DEBUG("MakeCenterLine: i: " + std::to_string(i));
                    }
                    if (bev_ele.left_line_points.empty() && bev_ele.right_line_points.empty()){
                        //如果左右边线都为空，则认为该车道没有中心线
                        LOG_ERROR("bev_ele left or right line points is empty");
                        continue;
                    }

                    EFMRefLinePoints center_line_points = GetCenterLineCommon(bev_ele.lane_width_last_cycle, bev_ele.left_line_points, bev_ele.right_line_points, bev_ele.left_types, bev_ele.right_types, 0, 0, lane_idx, is_neighbor_lane);
                    if (center_line_points.empty()){
                        LOG_ERROR("center_line_points is empty");
                        continue;
                    }
                    
                    bev_ele.center_line_points = center_line_points;
                    EFMRefLinePoints side_line_points = {};
                    bool is_left = false;
                    if (0 == i){
                        if (bev_ele.right_line_points.front().x > main_right_line_points.front().x){
                            side_line_points = main_right_line_points;
                            is_left = true;
                        }
                        
                    }else if(bev_ele_group.size() -1 == i){
                        if (bev_ele.left_line_points.front().x > main_left_line_points.front().x){
                            side_line_points = main_left_line_points;
                            is_left = false;
                        }
                    }else{
                        // if (1 == lane_idx){
                        //     side_line_points = main_right_line_points;
                        //     is_left = true;
                        // }else{
                        //     side_line_points = main_left_line_points;
                        //     is_left = false;
                        // }

                        // 在中心线计算完成后，添加从自车到center_line_points.front()的直线连接
                        if (!center_line_points.empty()) {
                            EFMPoint ego_point(0.0, 0.0); // 自车位置
                            EFMPoint first_point = center_line_points.front(); // 中心线第一个点
                            
                            // 如果第一个点不在自车位置，需要添加连接线
                            if (first_point.x > 0.1) { // 避免浮点数比较误差
                                EFMRefLinePoints connection_points;
                                
                                // 计算从自车到第一个点的距离
                                double total_distance = std::hypot(first_point.x - ego_point.x, 
                                                                first_point.y - ego_point.y);
                                
                                // 使用2.5m的间隔进行离散化
                                double step = 1.5;
                                int num_steps = static_cast<int>(std::ceil(total_distance / step));
                                
                                // 如果距离太小，至少插入一个点
                                if (num_steps == 0) {
                                    connection_points.push_back(ego_point);
                                } else {
                                    // 计算方向向量
                                    double dx = (first_point.x - ego_point.x) / total_distance;
                                    double dy = (first_point.y - ego_point.y) / total_distance;
                                    
                                    // 从自车位置开始，每隔step距离插入一个点
                                    for (int i = 0; i < num_steps; ++i) {
                                        double distance = i * step;
                                        EFMPoint new_point;
                                        new_point.x = ego_point.x + dx * distance;
                                        new_point.y = ego_point.y + dy * distance;
                                        connection_points.push_back(new_point);
                                    }
                                }
                                
                                // 将连接点插入到center_line_points的前面
                                center_line_points.insert(center_line_points.begin(), 
                                                        connection_points.begin(), 
                                                        connection_points.end());
                            }
                        }
                    }

                    if (side_line_points.empty()){
                        bev_ele.center_line_points = center_line_points;
                        continue;
                    }

                    //根据中心线位置，平移side_line_points
                    EFMRefLinePoints new_center_line_points = {};

                    EFMPoint tar_point = EFMPoint(0.0, 0.0);
                    bool is_inside = false;
                    int nearest_index = -1; 
                    int sign = -1;
                    if (false == CommonTool::DiscretePointsMath::GetInstance()->GetProjectPointBodyCoordinate(center_line_points.front(), side_line_points,
                                                                                                              tar_point, is_inside, nearest_index, sign)){
                        bev_ele.center_line_points = center_line_points;
                        continue;
                    }
                    if (is_inside == false || nearest_index < 0 || nearest_index >= side_line_points.size()){
                        bev_ele.center_line_points = center_line_points;
                        continue;
                    }
                    auto nearest_point = side_line_points[nearest_index];
                    if (nearest_point.x > tar_point.x){
                        new_center_line_points.insert(new_center_line_points.end(), side_line_points.begin(), side_line_points.begin() + nearest_index);
                    }else{
                        new_center_line_points.insert(new_center_line_points.end(), side_line_points.begin(), side_line_points.begin() + nearest_index + 1);
                    }
                    new_center_line_points.push_back(tar_point);
                    
                    double offset_len = std::hypot(center_line_points.front().x - tar_point.x, center_line_points.front().y - tar_point.y);
                    if (!is_left){
                        offset_len *= -1.0;
                    }
                    
                    center_line_points = offsetBevLine(new_center_line_points, offset_len);

                    bev_ele.center_line_points = new_center_line_points;
                    bev_ele.center_line_points.insert(bev_ele.center_line_points.end(), center_line_points.begin(), center_line_points.end());
                }
            }else{
                double temp = -1;
                EFMRefLinePoints main_center_line_points = GetCenterLineCommon(temp, main_left_line_points, main_right_line_points, main_left_types, main_right_types, 0, main_line, lane_idx, false);
                if (main_center_line_points.empty()){
                    LOG_ERROR("main_center_line_points is empty! ");
                    // return false;
                }
                if (DEBUG_FLAG == 1)
                {
                    LOG_DEBUG("main_left_line_points.size(): " + std::to_string(main_left_line_points.size()));
                    LOG_DEBUG("main_right_line_points.size(): " + std::to_string(main_right_line_points.size()));
                    LOG_DEBUG("main_center_line_points.size(): " + std::to_string(main_center_line_points.size()));
                    //遍历main_left_line_points
                    std::string main_left_line_points_str = "";
                    for (const auto& point : main_left_line_points) {
                        main_left_line_points_str += std::to_string(point.x) + " " + std::to_string(point.y) + ",";
                    }
                    LOG_DEBUG("main_left_line_points: " + main_left_line_points_str);

                    //遍历main_left_line_points
                    std::string main_right_line_points_str = "";
                    for (const auto& point : main_right_line_points) {
                        main_right_line_points_str += std::to_string(point.x) + " " + std::to_string(point.y) + ",";
                    }
                    LOG_DEBUG("main_right_line_points: " + main_right_line_points_str);
                    //遍历main_center_line_points
                    std::string main_center_line_points_strx = "[";
                    std::string main_center_line_points_stry = "[";
                    for (const auto& point : main_center_line_points) {
                        main_center_line_points_strx += std::to_string(point.x) +  ",";
                        main_center_line_points_stry += std::to_string(point.y) + ",";
                    }
                    LOG_DEBUG("main center ix=" + main_center_line_points_strx + "]");
                    LOG_DEBUG("main center iy=" + main_center_line_points_stry + "]");
                }

                //遍历bev_ele_group，做成中心线
                double min_split_dis = MAXFLOAT; //最小分歧距离
                double min_angle_diff = MAXFLOAT;
                int32_t min_split_dis_idx = -1; //最小分歧距离的索引
                for (int32_t i = 0; i < bev_ele_group.size(); ++i){


                    bool is_neighbor_lane = false;
                    if (0 == lane_idx){
                        is_neighbor_lane = false;;
                    }else if (1 == lane_idx && i == 0){
                        is_neighbor_lane = true;
                    }else if (2 == lane_idx && i == bev_ele_group.size() -1){
                        is_neighbor_lane = true;
                    }

                    if (DEBUG_FLAG == 1)
                    {
                        LOG_DEBUG("MakeCenterLine: i: " + std::to_string(i));
                    }
                    BevLaneElement& bev_ele = bev_ele_group[i];
                    if (bev_ele.left_line_points.empty() && bev_ele.right_line_points.empty()){
                        //如果左右边线都为空，则认为该车道没有中心线
                        LOG_ERROR("bev_ele left or right line points is empty");
                        continue;
                    }
                    bev_ele.is_main_ele = false; //默认不是主车道
                    bev_ele.center_line_points = main_center_line_points;
                    bev_ele.s_split_point = main_center_line_points.size() - 1; //分歧点在中心线的最后一个点
                    if (DEBUG_FLAG == 1)
                    {
                        LOG_DEBUG("bev_ele.s_split_point=" + std::to_string(bev_ele.s_split_point));
                        LOG_DEBUG("bev_ele.e_split_point=" + std::to_string(bev_ele.e_split_point));
                    }

                    EFMRefLinePoints left_line_points = {};
                    EFMRefLinePoints right_line_points = {};

                    if (main_left_line_points.empty()){
                        LOG_ERROR("main_center_line_points.size: i: " + std::to_string(main_center_line_points.size()));
                        LOG_ERROR("main_left_line_points.size: i: " + std::to_string(main_left_line_points.size()));
                        GetLinePointUpX(bev_ele.left_line_points, bev_ele.right_line_points, left_line_points, right_line_points, main_right_line_points.back().x);
                    }else{
                        GetLinePointUpX(bev_ele.left_line_points, bev_ele.right_line_points, left_line_points, right_line_points, main_left_line_points.back().x);
                    }
                    if (DEBUG_FLAG == 1)
                    {
                        std::string left_line_points_strx = "[";
                        std::string left_line_points_stry = "[";
                        for (const auto& point : left_line_points) {
                            left_line_points_strx += std::to_string(point.x) +  ",";
                            left_line_points_stry += std::to_string(point.y) + ",";
                        }
                        LOG_DEBUG("left line ix=" +  left_line_points_strx + "]");
                        LOG_DEBUG("left line iy=" +  left_line_points_stry + "]");

                        std::string right_line_points_strx = "[";
                        std::string right_line_points_stry = "[";
                        for (const auto& point : right_line_points) {
                            right_line_points_strx += std::to_string(point.x) +  ",";
                            right_line_points_stry += std::to_string(point.y) + ",";
                        }
                        LOG_DEBUG("right line ix=" +  right_line_points_strx + "]");
                        LOG_DEBUG("right line iy=" +  right_line_points_stry + "]");

                        std::string main_center_line_points_strx = "[";
                        std::string main_center_line_points_stry = "[";
                        for (const auto& point : main_center_line_points) {
                            main_center_line_points_strx += std::to_string(point.x) +  ",";
                            main_center_line_points_stry += std::to_string(point.y) + ",";
                        }
                        LOG_DEBUG("main_center_line_points ix=" +  main_center_line_points_strx + "]");
                        LOG_DEBUG("main_center_line_points iy=" +  main_center_line_points_stry + "]");
                    }

                    // DeleteLineForStartMerge(left_line_points, right_line_points);

                    // left_line_points_strx = "[";
                    // left_line_points_stry = "[";
                    // for (const auto& point : left_line_points) {
                    //     left_line_points_strx += std::to_string(point.x) +  ",";
                    //     left_line_points_stry += std::to_string(point.y) + ",";
                    // }
                    // LOG_DEBUG("left line ix=" +  left_line_points_strx + "]");
                    // LOG_DEBUG("left line iy=" +  left_line_points_stry + "]");

                    // right_line_points_strx = "[";
                    // right_line_points_stry = "[";
                    // for (const auto& point : right_line_points) {
                    //     right_line_points_strx += std::to_string(point.x) +  ",";
                    //     right_line_points_stry += std::to_string(point.y) + ",";
                    // }
                    // LOG_DEBUG("right line ix=" +  right_line_points_strx + "]");
                    // LOG_DEBUG("right line iy=" +  right_line_points_stry + "]");
                    if (DEBUG_FLAG == 1)
                    {
                        LOG_DEBUG("MakeCenterLine: lane_idx: " + std::to_string(lane_idx));
                    }
                    EFMRefLinePoints center_line_points = GetCenterLineCommon(bev_ele.lane_width_last_cycle, left_line_points, right_line_points, bev_ele.left_types, bev_ele.right_types, 0, 0, lane_idx, is_neighbor_lane);
                    if (!center_line_points.empty() && !main_center_line_points.empty()){
                        //处理接口处中心线,如果距离过短，直接拼接一起
                        double split_dis = std::hypot(center_line_points.front().x - main_center_line_points.back().x, 
                                                center_line_points.front().y - main_center_line_points.back().y);

                        // double split_dis_second_point = split_dis;
                        // if (center_line_points.size() >= 2){
                        //     split_dis_second_point = std::hypot(center_line_points[1].x - main_center_line_points.back().x, 
                        //                         center_line_points[1].y - main_center_line_points.back().y);
                        // }

                        // //如果分歧距离小于最小分歧距离，则更新最小分歧距离和索引
                        // if (split_dis_second_point < min_split_dis){
                        //     min_split_dis = split_dis_second_point;
                        //     min_split_dis_idx = i;
                        // }

                        // 计算角度差

                        double angle_diff = 0.0;
                        
                        // 创建临时线: main终点 -> center起点的连接线 + center_line_points
                        EFMRefLinePoints temp_combined_line;
                        
                        // 1. 添加从main终点到center起点的连接线(1m间隔离散化)
                        EFMPoint main_end = main_center_line_points.back();
                        EFMPoint center_start = center_line_points.front();
                        
                        double connection_dist = std::hypot(center_start.x - main_end.x, 
                                                           center_start.y - main_end.y);
                        
                        if (connection_dist > 0.1) { // 如果距离大于0.1m才需要连接
                            int num_points = static_cast<int>(std::ceil(connection_dist / 1.0)); // 1m间隔
                            
                            for (int i = 0; i <= num_points; ++i) {
                                double ratio = static_cast<double>(i) / num_points;
                                EFMPoint interp_point;
                                interp_point.x = main_end.x + ratio * (center_start.x - main_end.x);
                                interp_point.y = main_end.y + ratio * (center_start.y - main_end.y);
                                temp_combined_line.push_back(interp_point);
                            }
                        } else {
                            temp_combined_line.push_back(main_end);
                        }
                        
                        // 2. 添加center_line_points到临时线
                        temp_combined_line.insert(temp_combined_line.end(), 
                                                 center_line_points.begin(), 
                                                 center_line_points.end());
                        
                        // 3. 在组合线中找到与起点x方向距离>2.5m的点
                        int temp_idx = -1;
                        for (size_t i = 1; i < temp_combined_line.size(); ++i) {
                            if (fabs(temp_combined_line[i].x - temp_combined_line[0].x) > 15.0) {
                                temp_idx = i;
                                break;
                            }
                        }
                        
                        // 4. 在main_center_line_points中找到与终点x方向距离>2.5m的点
                        int main_idx = -1;
                        size_t n = main_center_line_points.size();
                        for (int i = n - 2; i >= 0; --i) {
                            if (fabs(main_center_line_points[n-1].x - main_center_line_points[i].x) > 2.5) {
                                main_idx = i;
                                break;
                            }
                        }
                        
                        if (temp_idx > 0 && main_idx >= 0) {
                            // 计算临时组合线的方向向量
                            double dx1 = temp_combined_line[temp_idx].x - temp_combined_line[0].x;
                            double dy1 = temp_combined_line[temp_idx].y - temp_combined_line[0].y;
                            double angle1 = std::atan2(dy1, dx1); // 弧度
                            
                            // 计算main_center_line_points的方向向量
                            double dx2 = main_center_line_points[n-1].x - main_center_line_points[main_idx].x;
                            double dy2 = main_center_line_points[n-1].y - main_center_line_points[main_idx].y;
                            double angle2 = std::atan2(dy2, dx2); // 弧度
                            
                            // 计算角度差的绝对值
                            angle_diff = fabs(angle2 - angle1);

                            if (DEBUG_FLAG == 1) {
                                LOG_DEBUG("temp_combined_line[0]: " + std::to_string(temp_combined_line[0].x) + ", " + std::to_string(temp_combined_line[0].y));
                                LOG_DEBUG("temp_combined_line[temp_idx]: " + std::to_string(temp_combined_line[temp_idx].x) + ", " + std::to_string(temp_combined_line[temp_idx].y));
                                LOG_DEBUG("main_center_line_points[main_idx]: " + std::to_string(main_center_line_points[main_idx].x) + ", " + std::to_string(main_center_line_points[main_idx].y));
                                LOG_DEBUG("main_center_line_points[n-1]: " + std::to_string(main_center_line_points[n-1].x) + ", " + std::to_string(main_center_line_points[n-1].y));
                                LOG_DEBUG("temp_idx: " + std::to_string(temp_idx) + 
                                        ", x_dist: " + std::to_string(fabs(temp_combined_line[temp_idx].x - temp_combined_line[0].x)));
                                LOG_DEBUG("main_idx: " + std::to_string(main_idx) + 
                                        ", x_dist: " + std::to_string(fabs(main_center_line_points[n-1].x - main_center_line_points[main_idx].x)));
                                LOG_DEBUG("angle1: " + std::to_string(angle1 * 180.0 / M_PI) + " deg");
                                LOG_DEBUG("angle2: " + std::to_string(angle2 * 180.0 / M_PI) + " deg");
                                LOG_DEBUG("angle_diff: " + std::to_string(angle_diff * 180.0 / M_PI) + " deg");
                                LOG_DEBUG("connection_dist: " + std::to_string(connection_dist) + " m");
                            }
                        }
                        // if ((split_dis < min_split_dis) && (angle_diff < min_angle_diff)){
                        if (angle_diff < min_angle_diff){
                            min_split_dis = split_dis;
                            min_angle_diff = angle_diff;
                            min_split_dis_idx = i;
                        }

                        if (split_dis < 5.0 && fabs(center_line_points.front().y - main_center_line_points.back().y) < 1.0) {
                            center_line_points = offsetBevLine(center_line_points, main_center_line_points.back().y - center_line_points.front().y);
                        }
                    }
                    if (DEBUG_FLAG == 1)
                    {
                        //遍历center_line_points
                        std::string center_line_points_strx = "[";
                        std::string center_line_points_stry = "[";
                        for (const auto& point : center_line_points) {
                            center_line_points_strx += std::to_string(point.x) +  ",";
                            center_line_points_stry += std::to_string(point.y) + ",";
                        }
                        LOG_DEBUG("center line ix=" +  center_line_points_strx + "]");
                        LOG_DEBUG("center line iy=" +  center_line_points_stry + "]");
                    }

                    //如果center_line_points为空，直接跳过
                    if (center_line_points.empty()){
                        LOG_DEBUG("bev_ele.s_split_point=" + std::to_string(bev_ele.s_split_point));
                        LOG_DEBUG("bev_ele.e_split_point=" + std::to_string(bev_ele.e_split_point));
                        LOG_ERROR("main_center_line_points or center_line_points is empty");
                        continue;
                    }
                    if (DEBUG_FLAG == 1)
                    {
                        //遍历center_line_points
                        std::string center_line_points_strx = "[";
                        std::string center_line_points_stry = "[";
                        for (const auto& point : bev_ele.center_line_points) {
                            center_line_points_strx += std::to_string(point.x) +  ",";
                            center_line_points_stry += std::to_string(point.y) + ",";
                        }
                        LOG_DEBUG("center line ix=" +center_line_points_strx + "]");
                        LOG_DEBUG("center line iy=" + center_line_points_stry + "]");
                    }

                    // //根据X、Y轴的位置，修正中心线
                    // if (false == AlterLineForMainLine(bev_ele, center_line_points, bev_ele_group)){
                    //     //如果中心线点数大于0，则认为需要修正
                    //     LOG_DEBUG("bev_ele.s_split_point=" + std::to_string(bev_ele.s_split_point));
                    //     LOG_DEBUG("bev_ele.e_split_point=" + std::to_string(bev_ele.e_split_point));
                    //     continue;
                    // }
                    if (DEBUG_FLAG == 1)
                    {
                        //遍历center_line_points
                        std::string center_line_points_strx = "[";
                        std::string center_line_points_stry = "[";
                        for (const auto& point : center_line_points) {
                            center_line_points_strx += std::to_string(point.x) +  ",";
                            center_line_points_stry += std::to_string(point.y) + ",";
                        }
                        LOG_DEBUG("center line ix=" +center_line_points_strx + "]");
                        LOG_DEBUG("center line iy=" + center_line_points_stry + "]");
                    }

                    bev_ele.e_split_point = bev_ele.s_split_point + 1;
                    if (DEBUG_FLAG == 1)
                    { 
                        LOG_DEBUG("bev_ele.s_split_point=" + std::to_string(bev_ele.s_split_point));
                        LOG_DEBUG("bev_ele.e_split_point=" + std::to_string(bev_ele.e_split_point));
                    }
                    bev_ele.center_line_points.insert(bev_ele.center_line_points.end(), center_line_points.begin(), center_line_points.end());
                }

                //如果最小分歧距离的索引大于等于0，则认为找到了主车道
                if (min_split_dis_idx >= 0 && min_split_dis_idx < bev_ele_group.size()){    
                    //如果找到了主车道，则认为该车道是主车道
                    bev_ele_group[min_split_dis_idx].is_main_ele = true;
                    //小于主车道的车道，认为是右侧车道
                    for (int32_t i = 0; i < min_split_dis_idx; ++i){
                        bev_ele_group[i].split_dir = 2;
                        bev_ele_group[i].to_main_lane_times = min_split_dis_idx - i;
                    }
                    //大于主车道的车道，认为是左侧车道
                    for (int32_t i = min_split_dis_idx + 1; i < bev_ele_group.size(); ++i){
                        bev_ele_group[i].split_dir = 1;
                        bev_ele_group[i].to_main_lane_times = i - min_split_dis_idx; 
                    }
                }
                for (int32_t i = 0; i < bev_ele_group.size(); ++i){
                    BevLaneElement& bev_ele = bev_ele_group[i];
                    if(!bev_ele.is_main_ele){
                        // 删除前面的 main_center_line_points 部分
                        // bev_ele.center_line_points 的前 main_center_line_points.size() 个点是主线部分
                        if (bev_ele.center_line_points.size() > main_center_line_points.size()){
                            // 删除前面的主线部分，保留分支部分
                            bev_ele.center_line_points.erase(bev_ele.center_line_points.begin(), 
                                                            bev_ele.center_line_points.begin() + main_center_line_points.size());
                            // 更新分歧点索引
                            bev_ele.s_split_point = 0;
                            bev_ele.e_split_point = 1;
                        }
                    }
                }

            }
        }
    
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::MakeCenterLineEgo: " + std::string(e.what()));
        return false;
    }

    return true;
}

bool LocalMap::MakeCenterLineForMerge(BevLaneElement& bev_ele, uint8_t merge_dir, int32_t left_merge_point, 
                                       int32_t right_merge_point, int32_t left_narrow_pos, int32_t right_narrow_pos, 
                                       BevLaneElement& side_bev_ele){

    try
    {
        //如果bev_ele的左右边线都为空，则认为该车道没有中心线
        if (bev_ele.left_line_points.empty() || bev_ele.right_line_points.empty()){
            LOG_ERROR("bev_ele left or right line points is empty");
            return false;
        }

        //根据merge_dir进行判断
        if (merge_dir == 1){
            //如果右侧的合流点或者收窄点未找到，直接返回
            if (right_merge_point < 0 || right_narrow_pos < 0){
                LOG_ERROR("right_merge_point or right_narrow_pos is negative");
                return false;
            }

            //获取收窄点之前的中心线
            EFMRefLinePoints right_points = EFMRefLinePoints();
            right_points.insert(right_points.begin(), bev_ele.right_line_points.begin(), bev_ele.right_line_points.begin() + right_narrow_pos + 1);
            EFMRefLinePoints left_points = EFMRefLinePoints();
            if (left_narrow_pos >= 0){
                left_points.insert(left_points.begin(), bev_ele.left_line_points.begin(), bev_ele.left_line_points.begin() + left_narrow_pos + 1);
            }else{
                left_points.insert(left_points.begin(), bev_ele.left_line_points.begin(), bev_ele.left_line_points.begin() + bev_ele.left_overlap_right_end_index + 1);
            }
            bev_ele.center_line_points = GetCenterLineCommon(bev_ele.lane_width_last_cycle, left_points, right_points, bev_ele.left_types, bev_ele.right_types, merge_dir, 0, -1, false);
            bev_ele.s_merge_point = bev_ele.center_line_points.size() - 1; //合流点在中心线的最后一个点

            //收窄点和合流点之间的中心线（如果两者之间有间隔）
            if (right_merge_point > right_narrow_pos + 1){
                EFMRefLinePoints mid_right_points = EFMRefLinePoints();
                mid_right_points.insert(mid_right_points.begin(), bev_ele.right_line_points.begin() + right_narrow_pos, 
                                        bev_ele.right_line_points.begin() + right_merge_point + 1);
                
                //找到左边线对应的部分
                EFMRefLinePoints mid_left_points = EFMRefLinePoints();
                int32_t left_start_idx = (left_narrow_pos >= 0) ? left_narrow_pos : 0;
                //通过x坐标找到左边线中对应合流点位置的索引
                double merge_x = bev_ele.right_line_points[right_merge_point].x;
                int32_t left_end_idx = left_start_idx;
                for (size_t i = left_start_idx; i < bev_ele.left_line_points.size(); ++i){
                    if (bev_ele.left_line_points[i].x >= merge_x){
                        left_end_idx = i;
                        break;
                    }
                    left_end_idx = i;
                }
                if (left_end_idx > left_start_idx && left_end_idx < static_cast<int32_t>(bev_ele.left_line_points.size())){
                    mid_left_points.insert(mid_left_points.begin(), bev_ele.left_line_points.begin() + left_start_idx, 
                                           bev_ele.left_line_points.begin() + left_end_idx + 1);
                }
                
                EFMRefLinePoints mid_center_points = EFMRefLinePoints();
                mid_center_points = GetCenterLineCommon(bev_ele.lane_width_last_cycle, mid_left_points, mid_right_points, 
                                                        bev_ele.left_types, bev_ele.right_types, merge_dir, 0, -1, false);
                //跳过第一个点避免重复
                if (!mid_center_points.empty()){
                    bev_ele.center_line_points.insert(bev_ele.center_line_points.end(), 
                                                      mid_center_points.begin() + 1, mid_center_points.end());
                }
                bev_ele.s_merge_point = bev_ele.center_line_points.size() - 1;
            }

            //获取合流点之后的中心线
            right_points.clear();
            right_points.insert(right_points.begin(), bev_ele.right_line_points.begin() + right_merge_point, bev_ele.right_line_points.end());
            
            PointSLd cur_sl =  PointSLd();
            bool cur_is_inside = false;
            int32_t cur_nearest_index = -1;
            if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(side_bev_ele.left_line_points, 
                                                                                                bev_ele.right_line_points[right_merge_point], 
                                                                                                cur_sl, cur_is_inside, cur_nearest_index)){
                LOG_ERROR("CalPointSLBodyCoordinate error");
                return false;
            }
            //如果cur_nearest_index在左边线范围内，
            left_points.clear();
            if (cur_nearest_index >= 0 && cur_nearest_index < side_bev_ele.left_line_points.size()){
                left_points.insert(left_points.begin(), side_bev_ele.left_line_points.begin() + cur_nearest_index, side_bev_ele.left_line_points.end());
            }

            EFMRefLinePoints merge_center_points = EFMRefLinePoints();
            //合并之后中心线，取中间
            merge_center_points = GetCenterLineCommon(side_bev_ele.lane_width_last_cycle, left_points, right_points, side_bev_ele.left_types, bev_ele.right_types, 0, 0, -1, false);
            bev_ele.e_merge_point = bev_ele.s_merge_point + 1; 
            bev_ele.center_line_points.insert(bev_ele.center_line_points.end(), merge_center_points.begin(), merge_center_points.end());

        }else if (merge_dir == 2){
            //如果左侧的合流点或者收窄点未找到，直接返回
            if (left_merge_point < 0 || left_narrow_pos < 0){
                LOG_ERROR("left_merge_point or left_narrow_pos is negative");
                return false;
            }

            //获取收窄点之前的中心线
            EFMRefLinePoints left_points = EFMRefLinePoints();
            left_points.insert(left_points.begin(), bev_ele.left_line_points.begin(), bev_ele.left_line_points.begin() + left_narrow_pos + 1);
            EFMRefLinePoints right_points = EFMRefLinePoints();
            if (right_narrow_pos >= 0){
                right_points.insert(right_points.begin(), bev_ele.right_line_points.begin(), bev_ele.right_line_points.begin() + right_narrow_pos + 1);
            }else{
                right_points.insert(right_points.begin(), bev_ele.right_line_points.begin(), bev_ele.right_line_points.begin()+ bev_ele.right_overlap_left_end_index + 1);
            }
            bev_ele.center_line_points = GetCenterLineCommon(bev_ele.lane_width_last_cycle, left_points, right_points, bev_ele.left_types, bev_ele.right_types, bev_ele.merge_dir, 0, -1, false);
            bev_ele.s_merge_point = bev_ele.center_line_points.size() - 1; //合流点在中心线的最后一个点

            //收窄点和合流点之间的中心线（如果两者之间有间隔）
            if (left_merge_point > left_narrow_pos + 1){
                EFMRefLinePoints mid_left_points = EFMRefLinePoints();
                mid_left_points.insert(mid_left_points.begin(), bev_ele.left_line_points.begin() + left_narrow_pos, 
                                       bev_ele.left_line_points.begin() + left_merge_point + 1);
                
                //找到右边线对应的部分
                EFMRefLinePoints mid_right_points = EFMRefLinePoints();
                int32_t right_start_idx = (right_narrow_pos >= 0) ? right_narrow_pos : 0;
                //通过x坐标找到右边线中对应合流点位置的索引
                double merge_x = bev_ele.left_line_points[left_merge_point].x;
                int32_t right_end_idx = right_start_idx;
                for (size_t i = right_start_idx; i < bev_ele.right_line_points.size(); ++i){
                    if (bev_ele.right_line_points[i].x >= merge_x){
                        right_end_idx = i;
                        break;
                    }
                    right_end_idx = i;
                }
                if (right_end_idx > right_start_idx && right_end_idx < static_cast<int32_t>(bev_ele.right_line_points.size())){
                    mid_right_points.insert(mid_right_points.begin(), bev_ele.right_line_points.begin() + right_start_idx, 
                                            bev_ele.right_line_points.begin() + right_end_idx + 1);
                }
                
                EFMRefLinePoints mid_center_points = EFMRefLinePoints();
                mid_center_points = GetCenterLineCommon(bev_ele.lane_width_last_cycle, mid_left_points, mid_right_points, 
                                                        bev_ele.left_types, bev_ele.right_types, merge_dir, 0, -1, false);
                //跳过第一个点避免重复
                if (!mid_center_points.empty()){
                    bev_ele.center_line_points.insert(bev_ele.center_line_points.end(), 
                                                      mid_center_points.begin() + 1, mid_center_points.end());
                }
                bev_ele.s_merge_point = bev_ele.center_line_points.size() - 1;
            }

            //获取合流点之后的中心线
            left_points.clear();
            left_points.insert(left_points.begin(), bev_ele.left_line_points.begin() + left_merge_point, bev_ele.left_line_points.end());
            
            PointSLd cur_sl =  PointSLd();
            bool cur_is_inside = false;
            int32_t cur_nearest_index = -1;
            if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(side_bev_ele.right_line_points, 
                                                                                                bev_ele.left_line_points[left_merge_point], 
                                                                                                cur_sl, cur_is_inside, cur_nearest_index)){
                LOG_ERROR("CalPointSLBodyCoordinate error");
                return false;
            }
            //如果cur_nearest_index在右边线范围内，
            right_points.clear();
            if (cur_nearest_index >= 0 && cur_nearest_index < side_bev_ele.right_line_points.size()){
                right_points.insert(right_points.begin(), side_bev_ele.right_line_points.begin() + cur_nearest_index, side_bev_ele.right_line_points.end());
            }

            EFMRefLinePoints merge_center = EFMRefLinePoints();
            //合并之后中心线，取中间
            merge_center = GetCenterLineCommon(bev_ele.lane_width_last_cycle, left_points, right_points, bev_ele.left_types, side_bev_ele.right_types, 0, 0, -1, false);
            bev_ele.e_merge_point = bev_ele.s_merge_point + 1; 
            bev_ele.center_line_points.insert(bev_ele.center_line_points.end(), merge_center.begin(), merge_center.end());
        }else{
            //如果不是向左或向右合流，则认为有问题
            return false;
        }
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::MakeCenterLineForMerge: " + std::string(e.what()));
        return false;
    }
    return true;
}

bool LocalMap::GetBevLineType(std::vector<LineTypeInfo>& types, double start_x, BstdsLineType& re_type){
    re_type = BstdsLineType::LINE_UNKNOWN;
    if (types.empty()){
        return false;
    }
    std::vector<BstdsLineType> sorted_types;
    std::vector<double> sorted_starts;
    for (const auto& type_info : types){
        if (type_info.is_valid == false) continue;

        if (sorted_types.empty()){
            sorted_types.push_back(static_cast<BstdsLineType>(type_info.line_type));
            sorted_starts.push_back(type_info.start_point);
        }else{
            if (sorted_types.back() != static_cast<BstdsLineType>(type_info.line_type)){
                sorted_types.push_back(static_cast<BstdsLineType>(type_info.line_type));
                sorted_starts.push_back(type_info.start_point);
            }
        }

        if (type_info.typ_chg_point < 10000.0){
            sorted_types.push_back(static_cast<BstdsLineType>(type_info.typ_aft_chg_point));
            sorted_starts.push_back(type_info.typ_chg_point);
        }
    }
    //LOG_DEBUG("start_x=" + std::to_string(start_x));

    // for (size_t i = 0; i < sorted_types.size(); i++)
    // {
    //     LOG_DEBUG("sorted_types[" + std::to_string(i) + "]=" + std::to_string(static_cast<int32_t>(sorted_types[i])));
    //     LOG_DEBUG("sorted_starts[" + std::to_string(i) + "]=" + std::to_string(sorted_starts[i]));
    // }

    if (sorted_starts.empty()){
        return false;
    }

    if (start_x < sorted_starts.front()){
        // LOG_DEBUG("sorted_types.front()=" + std::to_string(static_cast<int32_t>(sorted_types.front())));
        // LOG_DEBUG("sorted_starts.front()=" + std::to_string(sorted_starts.front()));
        re_type = sorted_types.front();
        return true;
    }   

    if (start_x >= sorted_starts.back()){
        // LOG_DEBUG("sorted_types.back()=" + std::to_string(static_cast<int32_t>(sorted_types.back())));
        // LOG_DEBUG("sorted_starts.back()=" + std::to_string(sorted_starts.back()));
        re_type = sorted_types.back();
        return true;
    }

    for (int32_t i = 0; i < sorted_starts.size() - 1; ++i){
        if (start_x >= sorted_starts[i] && start_x < sorted_starts[i + 1]){
            // LOG_DEBUG("sorted_types[i]=" + std::to_string(static_cast<int32_t>(sorted_types[i])));
            // LOG_DEBUG("sorted_starts[i]=" + std::to_string(sorted_starts[i]));
            // LOG_DEBUG("sorted_starts[i + 1]=" + std::to_string(sorted_starts[i + 1]));
            // LOG_DEBUG("start_x=" + std::to_string(start_x));
            if (fabs(start_x - sorted_starts[i+1]) < 10.0){
                re_type = sorted_types[i + 1];
            }else{
                re_type = sorted_types[i];
            }
            
            
            return true;
        }
    }

    return true;
}

EFMRefLinePoints LocalMap::GetCenterLineCommonOne(EFMRefLinePoints& left_points,
                                                  EFMRefLinePoints& right_points,
                                                  double all_right_dkappas,
                                                  double all_left_dkappas,
                                                  uint8_t main_line,
                                                  double left_min_distance,
                                                  double right_min_distance){
    if (DEBUG_FLAG == 1)
    {
        LOG_DEBUG("GetCenterLineCommonOne: left_points size: " + std::to_string(left_points.size()) + 
                ", right_points size: " + std::to_string(right_points.size()) + 
                ", all_right_dkappas: " + std::to_string(all_right_dkappas) + 
                ", all_left_dkappas: " + std::to_string(all_left_dkappas) + 
                ", main_line: " + std::to_string(main_line) + 
                ", left_min_distance: " + std::to_string(left_min_distance) + 
                ", right_min_distance: " + std::to_string(right_min_distance));
    }
    EFMRefLinePoints center_line_points = EFMRefLinePoints();
    if (main_line == 1){
        center_line_points = offsetBevLine(left_points, -1.0*left_min_distance);
    }else if (main_line == 2){
        center_line_points = offsetBevLine(right_points, right_min_distance);
    }else{
        // if (all_left_dkappas > all_right_dkappas){
        //     // center_line_points = offsetBevLine(right_points, right_min_distance);
        //     center_line_points = offsetBevLine(left_points, -1.0*left_min_distance);//cyj debug
        // }else{
        //     // center_line_points = offsetBevLine(right_points, right_min_distance);//cyj debug
        //     center_line_points = offsetBevLine(left_points, -1.0*left_min_distance);
        // }

        // 计算左右侧线的中线
        if (!left_points.empty() && !right_points.empty()) {
            // 计算左右边线的弧长
            std::vector<double> left_arc_lengths;
            std::vector<double> right_arc_lengths;
            double left_length = 0.0;
            double right_length = 0.0;
            
            left_arc_lengths.push_back(0.0);
            for (size_t i = 1; i < left_points.size(); ++i) {
                left_length += std::hypot(left_points[i].x - left_points[i-1].x, 
                                            left_points[i].y - left_points[i-1].y);
                left_arc_lengths.push_back(left_length);
            }
            
            right_arc_lengths.push_back(0.0);
            for (size_t i = 1; i < right_points.size(); ++i) {
                right_length += std::hypot(right_points[i].x - right_points[i-1].x,
                                            right_points[i].y - right_points[i-1].y);
                right_arc_lengths.push_back(right_length);
            }
            
            // 取较短的长度作为采样范围
            double max_length = std::min(left_length, right_length);
            
            // 清空之前的中心线，重新计算
            center_line_points.clear();
            
            // 每2.5m采样一次，计算中点
            for (double s = 0.0; s <= max_length; s += 2.5) {
                EFMPoint left_point = interpolateAtLength(left_points, left_arc_lengths, s);
                EFMPoint right_point = interpolateAtLength(right_points, right_arc_lengths, s);
                
                // 计算中点
                EFMPoint mid_point;
                mid_point.x = (left_point.x + right_point.x) / 2.0;
                mid_point.y = (left_point.y + right_point.y) / 2.0;
                
                center_line_points.push_back(mid_point);
            }
        }
        
    }    
    return center_line_points;
 }

EFMRefLinePoints LocalMap::GetCenterLineCommon(double& lane_width_last_cycle,
                                               EFMRefLinePoints& left_points, 
                                               EFMRefLinePoints& right_points, 
                                               std::vector<LineTypeInfo> left_types,
                                               std::vector<LineTypeInfo> right_types,
                                               uint8_t merge_dir,
                                               uint8_t main_line, 
                                               int32_t lane_idx, 
                                               bool is_neighbor_lane){
    EFMRefLinePoints center_line_points = EFMRefLinePoints();

    // //遍历left_points，打印
    // std::string left_points_strx = "[";
    // std::string left_points_stry = "[";
    // for (const auto& point : left_points) {
    //     left_points_strx += std::to_string(point.x) +  ",";
    //     left_points_stry += std::to_string(point.y) + ",";
    // }
    // LOG_DEBUG(left_points_strx + "]");
    // LOG_DEBUG(left_points_stry + "]");

    // //遍历right_points，打印
    // std::string right_points_strx = "[";
    // std::string right_points_stry = "[";
    // for (const auto& point : right_points) {
    //     right_points_strx += std::to_string(point.x) +  ",";
    //     right_points_stry += std::to_string(point.y) + ",";
    // }
    // LOG_DEBUG(right_points_strx + "]");
    // LOG_DEBUG(right_points_stry + "]");

    try
    {
        //如果左右边线都为空，直接返回空
        if (left_points.empty() && right_points.empty()){
            return center_line_points;
        }

        //如果是单边线，则直接通过偏移做成
        if (left_points.empty()){
            //如果左边线为空，则直接使用右边线
            //LOG_DEBUG("right_points" + std::to_string(static_cast<double>(offset_line_width) / 100.0));
            center_line_points = offsetBevLine(right_points, static_cast<double>(offset_line_width) / 100.0);
        }else if (right_points.empty()){
            //如果右边线为空，则直接使用左边线
            //LOG_DEBUG("left_points" + std::to_string(-1.0*static_cast<double>(offset_line_width) / 100.0));
            center_line_points = offsetBevLine(left_points, -1.0*static_cast<double>(offset_line_width) / 100.0);
        }else{
            //左右边线都不为空

            // cyj新增: 检查右侧线终点是否满足特殊条件（实际连线其实都不该生成，避免碰撞，激进一点先做出来，后面有问题再处理）
            // 条件: 右边线终点x < 15m 且 终点处宽度 < 2.0m
            double right_end_x = right_points.back().x;

            if (DEBUG_FLAG == 1) {
                LOG_DEBUG("Checking right line end: x=" + std::to_string(right_end_x));
            }

            // 判断右边线终点是否在25m以内
            if (right_end_x > 0 && right_end_x < 25.0) {
                // 计算右边线终点处的车道宽度
                double end_width = 0.0;
                EFMPoint right_end = right_points.back();

                // 在左边线上找到对应的点
                auto it = std::lower_bound(left_points.begin(), left_points.end(), right_end.x,
                                        [](const EFMPoint& point, double x) { return point.x < x; });

                if (it != left_points.end() && it != left_points.begin()) {
                    int idx = std::distance(left_points.begin(), it);
                    // 计算右边线终点到左边线的垂直距离
                    end_width = std::abs(PointLineDistance(left_points[idx - 1], left_points[idx], right_end));

                    if (DEBUG_FLAG == 1) {
                        LOG_DEBUG("Right line end check: x=" + std::to_string(right_end_x) +
                                "m, end_width=" + std::to_string(end_width) + "m");
                    }

                    // 如果终点宽度 < 2.0m，从右边线终点开始生成中心线
                    if (end_width > 0 && end_width < 2.0) {
                        // LOG_INFO("Right line short and narrow (x=" + std::to_string(right_end_x) +
                        //         "m, width=" + std::to_string(end_width) +
                        //         "m), using left line offset 1.75m from right end point");

                        // // 找到左边线中x >= right_end_x的部分
                        // EFMRefLinePoints left_points_after_right_end;
                        // for (const auto& point : left_points) {
                        //     if (point.x >= right_end_x) {
                        //         left_points_after_right_end.push_back(point);
                        //     }
                        // }

                        // // 如果找到了左边线的对应部分，使用左边线向右偏移1.75m生成中心线
                        // if (!left_points_after_right_end.empty()) {
                        //     center_line_points = offsetBevLine(left_points_after_right_end, -1.0 * offset_line_width * 0.01);
                        //     return center_line_points;
                        // }
                        return center_line_points;
                    }
                }
            }

            //获取左右边线的重叠部分
            double left_x_start = std::max(left_points.front().x, right_points.front().x);
            double left_x_end = std::min(left_points.back().x, right_points.back().x);
            if (left_x_start >= left_x_end){
                LOG_ERROR("left_x_start >= left_x_end, left_x_start: " + std::to_string(left_x_start) + 
                          ", left_x_end: " + std::to_string(left_x_end));
                //遍历left_points，打印
                LOG_ERROR("left_points.size: " + std::to_string(left_points.size()));
                for (const auto& point : left_points) {
                    LOG_ERROR("left_points: " + std::to_string(point.x) + ", " + std::to_string(point.y));
                }

                //遍历right_points，打印
                LOG_ERROR("right_points.size: " + std::to_string(right_points.size()));
                for (const auto& point : right_points) {
                    LOG_ERROR("right_points: " + std::to_string(point.x) + ", " + std::to_string(point.y));
                }
                return center_line_points;
            }
            // LOG_DEBUG("left_x_start: " + std::to_string(left_x_start));
            // LOG_DEBUG("left_x_end: " + std::to_string(left_x_end));

            //根据left_x_start和left_x_end，左车道插值出该点
            int32_t left_overlap_start_index = -1;
            int32_t left_overlap_end_index = -1;
            if (false == InterpolateBevLine(left_points, left_x_start, left_overlap_start_index,
                                            left_x_end, left_overlap_end_index, right_points)){
                LOG_ERROR("left_x_start: " + std::to_string(left_x_start) + ", left_x_end: " + std::to_string(left_x_end));
                LOG_ERROR("left_overlap_start_index: " + std::to_string(left_overlap_start_index) + ", left_overlap_end_index: " + std::to_string(left_overlap_end_index));
                center_line_points = offsetBevLine(right_points, static_cast<double>(offset_line_width) / 100.0);
                return center_line_points; //如果插值失败，直接返回右边线的偏移
            }

            //根据left_x_start和left_x_end，右车道插值出该点
            int32_t right_overlap_start_index = -1;
            int32_t right_overlap_end_index = -1;
            if (false == InterpolateBevLine(right_points, left_x_start, right_overlap_start_index,
                                            left_x_end, right_overlap_end_index, left_points)){
                LOG_ERROR("left_x_start: " + std::to_string(left_x_start) + ", left_x_end: " + std::to_string(left_x_end));
                LOG_ERROR("right_overlap_start_index: " + std::to_string(right_overlap_start_index) + ", right_overlap_end_index: " + std::to_string(right_overlap_end_index));
                center_line_points = offsetBevLine(left_points, -1.0*static_cast<double>(offset_line_width) / 100.0);
                return center_line_points; //如果插值失败，直接返回左边线的偏移
            }
            if (DEBUG_FLAG == 1)
            {
                LOG_DEBUG("left_overlap_start_index: " + std::to_string(left_overlap_start_index));
                LOG_DEBUG("left_overlap_end_index: " + std::to_string(left_overlap_end_index));
                LOG_DEBUG("right_overlap_start_index: " + std::to_string(right_overlap_start_index));
                LOG_DEBUG("right_overlap_end_index: " + std::to_string(right_overlap_end_index));
            }

            //如果重叠索引不在左右边线范围内，则认为有问题
            if (left_overlap_start_index < 0 || left_overlap_end_index < 0 || 
                right_overlap_start_index < 0 || right_overlap_end_index < 0 ||
                left_overlap_start_index >= static_cast<int32_t>(left_points.size()) ||
                left_overlap_end_index >= static_cast<int32_t>(left_points.size()) ||
                right_overlap_start_index >= static_cast<int32_t>(right_points.size()) ||
                right_overlap_end_index >= static_cast<int32_t>(right_points.size())){
                LOG_ERROR("overlap index is out of range");
                return center_line_points; //如果重叠索引不在范围内，返回空
            }

            //如果重叠部分的X距离大于60m，说明根本不准，直接用多长的边线进行偏移
            // if (left_points[left_overlap_start_index].x >= 60.0 && right_points[right_overlap_start_index].x >= 60.0) {
            //     if (left_overlap_start_index == right_overlap_start_index){
            //         if (left_points.size() >= right_points.size()){
            //             center_line_points = offsetBevLine(left_points, -1.0*static_cast<double>(offset_line_width) / 100.0);
            //         }else{
            //             center_line_points = offsetBevLine(right_points, static_cast<double>(offset_line_width) / 100.0);
            //         }
                    
            //     }else if (left_overlap_start_index > right_overlap_start_index){
            //         center_line_points = offsetBevLine(left_points, -1.0*static_cast<double>(offset_line_width) / 100.0);
            //     }
            //     else{
            //         center_line_points = offsetBevLine(right_points, static_cast<double>(offset_line_width) / 100.0);
            //     }
            //     return center_line_points;
            // }
            

            
            EFMRefLinePoints left_overlap_points = EFMRefLinePoints();
            EFMRefLinePoints right_overlap_points = EFMRefLinePoints();
            left_overlap_points.insert(left_overlap_points.begin(), left_points.begin() + left_overlap_start_index, 
                                       left_points.begin() + left_overlap_end_index + 1);
            right_overlap_points.insert(right_overlap_points.begin(), right_points.begin() + right_overlap_start_index, 
                                        right_points.begin() + right_overlap_end_index + 1);
            if (DEBUG_FLAG == 1)
            {
                //遍历left_overlap_points，打印
                std::string left_overlap_points_strx = "[";
                std::string left_overlap_points_stry = "[";
                for (const auto& point : left_overlap_points) {
                    left_overlap_points_strx += std::to_string(point.x) +  ",";
                    left_overlap_points_stry += std::to_string(point.y) + ",";
                }
                LOG_DEBUG(left_overlap_points_strx + "]");
                LOG_DEBUG(left_overlap_points_stry + "]");

                //遍历right_overlap_points，打印
                std::string right_overlap_points_strx = "[";
                std::string right_overlap_points_stry = "[";
                for (const auto& point : right_overlap_points) {
                    right_overlap_points_strx += std::to_string(point.x) +  ",";
                    right_overlap_points_stry += std::to_string(point.y) + ",";
                }
                LOG_DEBUG(right_overlap_points_strx + "]");
                LOG_DEBUG(right_overlap_points_stry + "]");
            }

            {//做重叠部分中心线
                //判断重叠部分的曲率
                std::vector<double> left_headings, left_accumulated_s, left_kappas, left_dkappas;
                CommonTool::DiscretePointsMath::GetInstance()->ComputePathProfile(left_overlap_points, 
                                                                                &left_headings, &left_accumulated_s, 
                                                                                &left_kappas, &left_dkappas);
                double all_left_dkappas = 0.0;
                for (size_t i = 1; i < left_kappas.size() - 1; ++i){
                    all_left_dkappas += fabs(left_kappas.at(i));
                }
                // //打印左边线的曲率
                // LOG_DEBUG("left all dkappas: " + std::to_string(all_left_dkappas));

                std::vector<double> right_headings, right_accumulated_s, right_kappas, right_dkappas;
                CommonTool::DiscretePointsMath::GetInstance()->ComputePathProfile(right_overlap_points, 
                                                                                &right_headings, &right_accumulated_s, 
                                                                                &right_kappas, &right_dkappas);
                double all_right_dkappas = 0.0;
                for (size_t i = 1; i < right_kappas.size() - 1; ++i){
                    all_right_dkappas += fabs(right_kappas.at(i));
                }
                // //打印右边线的曲率
                // LOG_DEBUG("right all dkappas: " + std::to_string(all_right_dkappas));

                //获取重叠部分的左边线和右边线的最小距离
                double min_distance = std::numeric_limits<double>::max();
                double unparallel_min = std::numeric_limits<double>::max();
                double unparallel_max = 0.0;
                double first_arrow_x = -200.0;
                double end_arrow_idx = -1;
                for (size_t i = 0; i < left_overlap_points.size() && left_overlap_points[i].x < 60.0; ++i){
                    //大于60m的点不计算
                    PointSLd cur_sl =  PointSLd();
                    if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(right_overlap_points, left_overlap_points[i], cur_sl)){
                        LOG_ERROR("CalPointSLBodyCoordinate error");
                        return center_line_points;
                    }
                    if (fabs(cur_sl.l) < static_cast<double>(offset_line_width)* 0.012 && first_arrow_x == -200.0){
                        first_arrow_x = left_overlap_points[i].x;
                    }

                    if (fabs(cur_sl.l) < 0.2 && end_arrow_idx == -1){
                        end_arrow_idx = i;
                    }
                    
                    if (fabs(cur_sl.l) < min_distance){
                        min_distance = fabs(cur_sl.l);
                    }

                    //cyj↓
                    if (left_overlap_points[i].x < 3 && left_overlap_points[i].x > -1){
                        min_distance = fabs(cur_sl.l);
                        break;
                    }
                    //cyj↑
                }

                for (size_t i = 0; i < left_overlap_points.size() && left_overlap_points[i].x < 60.0; ++i){
                    //大于60m的点不计算
                    PointSLd cur_sl =  PointSLd();
                    if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(right_overlap_points, left_overlap_points[i], cur_sl)){
                        LOG_ERROR("CalPointSLBodyCoordinate error");
                        return center_line_points;
                    }

                    if (fabs(cur_sl.l) < unparallel_min){
                        unparallel_min = fabs(cur_sl.l);
                    }

                    if (fabs(cur_sl.l) > unparallel_max){
                        unparallel_max = fabs(cur_sl.l);
                    }
                }

                
                double unparallel_distance = fabs(unparallel_max - unparallel_min);
                //判断是否是喇叭路，是的话放到超宽条件中
                bool is_horn_road = false;
                if (unparallel_distance > 1.0){
                    is_horn_road = true;
                }

                if (DEBUG_FLAG == 1)
                {
                    LOG_DEBUG("unparallel_max: " + std::to_string(unparallel_max) + " unparallel_min: " + std::to_string(unparallel_min) + " unparallel_distance: " + std::to_string(unparallel_distance));
                    LOG_DEBUG("is_horn_road: " + std::to_string(is_horn_road));
                }
                // //cyj 计算自车左右侧距离，相加的话作为当前道路的宽度↓
                // PointSLd ego_left_sl =  PointSLd();
                // PointSLd ego_right_sl =  PointSLd();
                // EFMPoint ego_point = EFMPoint(0.0, 0.0);
                // bool ego_left_sl_exist = true;
                // bool ego_right_sl_exist = true;
                // if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(left_overlap_points, ego_point, ego_left_sl)){

                //     ego_left_sl_exist = false;   
                // }
                // if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(right_overlap_points, ego_point, ego_right_sl)){

                //     ego_right_sl_exist = false;
                // }

                // if (ego_left_sl_exist && ego_right_sl_exist){ 
                //     min_distance = fabs(ego_left_sl.l) + fabs(ego_left_sl.l);//cyj debug use current l forever
                // }
                // //cyj 计算自车左右侧距离，相加的话作为当前道路的宽度↑

                if (first_arrow_x == -200.0){
                    first_arrow_x = left_overlap_points[left_overlap_points.size()/2].x;
                }
                
                min_distance = min_distance == std::numeric_limits<double>::max() ? static_cast<double>(offset_line_width)* 0.02 : min_distance;
                
                //对宽度进行滤波，滤波系数根据实际delta调整↓
                // 在类成员变量中添加滤波后的距离变量
                double filter_alpha ; // 滤波系数，范围0-1，值越大表示越依赖当前值
                if (fabs(lane_width_last_cycle-min_distance) > 0.5){
                    filter_alpha = 1.0;
                }else{
                    filter_alpha = 0.001;
                }

                // 添加一阶低通滤波
                if (lane_width_last_cycle < 0) {
                    // 第一次初始化
                    lane_width_last_cycle = min_distance;
                } else {
                    // 一阶低通滤波公式: y[n] = α * x[n] + (1-α) * y[n-1]
                    lane_width_last_cycle = filter_alpha * min_distance + (1 - filter_alpha) * lane_width_last_cycle;
                }

                // 使用滤波后的值替代原来的min_distance
                min_distance = lane_width_last_cycle;
                //对宽度进行滤波，滤波系数根据实际delta调整↑

                if (DEBUG_FLAG == 1)
                {
                    LOG_DEBUG("min_distance: " + std::to_string(min_distance));
                    LOG_DEBUG("offset_line_width: " + std::to_string(offset_line_width));
                }

                //获取重叠部分中心线
                double offset_distance = 0.0;
                if (min_distance <= static_cast<double>(offset_line_width)* 0.016 && is_horn_road == false){
                    //如果重叠部分的最小距离小于偏移线宽度的1.6%（2.8米），短的部分做成中心线
                    //获取重叠部分的左边线和右边线的车道线类型
                    BstdsLineType left_line_type = BstdsLineType::LINE_UNKNOWN;
                    BstdsLineType right_line_type = BstdsLineType::LINE_UNKNOWN;
                    GetBevLineType(left_types, first_arrow_x, left_line_type);
                    GetBevLineType(right_types, first_arrow_x, right_line_type);
                    if (DEBUG_FLAG == 1)
                    {
                        for (size_t i = 0; i < left_types.size(); i++)
                        {
                            LOG_DEBUG("left line type: start_point=" + std::to_string(left_types[i].start_point) + 
                                    ", line_type=" + std::to_string(static_cast<int32_t>(left_types[i].line_type)) + 
                                    ", typ_chg_point=" + std::to_string(left_types[i].typ_chg_point) + 
                                    ", typ_aft_chg_point=" + std::to_string(static_cast<int32_t>(left_types[i].typ_aft_chg_point)) + 
                                    ", is_valid=" + std::to_string(left_types[i].is_valid));
                        }

                        for (size_t i = 0; i < right_types.size(); i++)
                        {
                            LOG_DEBUG("right line type: start_point=" + std::to_string(right_types[i].start_point) + 
                                    ", line_type=" + std::to_string(static_cast<int32_t>(right_types[i].line_type)) + 
                                    ", typ_chg_point=" + std::to_string(right_types[i].typ_chg_point) + 
                                    ", typ_aft_chg_point=" + std::to_string(static_cast<int32_t>(right_types[i].typ_aft_chg_point)) + 
                                    ", is_valid=" + std::to_string(right_types[i].is_valid));
                        }
                        
                        LOG_DEBUG("left_line_type: " + std::to_string(static_cast<int32_t>(left_line_type)));
                        LOG_DEBUG("right_line_type: " + std::to_string(static_cast<int32_t>(right_line_type)));
                    }
                    if ((left_line_type == BstdsLineType::LINE_SOLID || left_line_type == BstdsLineType::LINE_DOUBLESOLID || 
                        left_line_type == BstdsLineType::LINE_DASHEDSOLID || left_line_type == BstdsLineType::LINE_FISHBONE ||
                        left_line_type == BstdsLineType::LINE_TEMPORARY || left_line_type == BstdsLineType::LINE_CURB ||
                        left_line_type == BstdsLineType::LINE_CONE || left_line_type == BstdsLineType::LINE_UNKNOWN) &&
                        (right_line_type == BstdsLineType::LINE_DASHED || right_line_type == BstdsLineType::LINE_DOUBLEDASHED || 
                        right_line_type == BstdsLineType::LINE_DASHEDSOLID)){
                        //左边不可跨，右边可跨，用左线偏移
                        center_line_points = offsetBevLine(left_overlap_points, -1.0*offset_line_width*0.01); 
                    }else if ((right_line_type == BstdsLineType::LINE_SOLID || right_line_type == BstdsLineType::LINE_DOUBLESOLID || 
                        right_line_type == BstdsLineType::LINE_SOLIDDASHED || right_line_type == BstdsLineType::LINE_FISHBONE ||
                        right_line_type == BstdsLineType::LINE_TEMPORARY || right_line_type == BstdsLineType::LINE_CURB ||
                        right_line_type == BstdsLineType::LINE_CONE || right_line_type == BstdsLineType::LINE_UNKNOWN) &&
                        (left_line_type == BstdsLineType::LINE_DASHED || left_line_type == BstdsLineType::LINE_DOUBLEDASHED || 
                        left_line_type == BstdsLineType::LINE_SOLIDDASHED)){
                        //右边不可跨，左边可跨，用右线偏移
                        center_line_points = offsetBevLine(right_overlap_points, offset_line_width*0.01); 
                    }else if ((left_line_type == BstdsLineType::LINE_SOLID || left_line_type == BstdsLineType::LINE_DOUBLESOLID || 
                        left_line_type == BstdsLineType::LINE_DASHEDSOLID || left_line_type == BstdsLineType::LINE_FISHBONE ||
                        left_line_type == BstdsLineType::LINE_TEMPORARY || left_line_type == BstdsLineType::LINE_CURB ||
                        left_line_type == BstdsLineType::LINE_CONE || left_line_type == BstdsLineType::LINE_UNKNOWN) &&
                        (right_line_type == BstdsLineType::LINE_SOLID || right_line_type == BstdsLineType::LINE_DOUBLESOLID || 
                        right_line_type == BstdsLineType::LINE_SOLIDDASHED || right_line_type == BstdsLineType::LINE_FISHBONE ||
                        right_line_type == BstdsLineType::LINE_TEMPORARY || right_line_type == BstdsLineType::LINE_CURB ||
                        right_line_type == BstdsLineType::LINE_CONE || right_line_type == BstdsLineType::LINE_UNKNOWN)){
                            center_line_points = GetCenterLineCommonOne(left_overlap_points,right_overlap_points,all_right_dkappas,all_left_dkappas,
                                                                        main_line, min_distance/2.0, min_distance/2.0);
                        // //左右都不可跨，取中间
                        // if (main_line == 1){
                        //     center_line_points = offsetBevLine(left_overlap_points, -1.0*min_distance/2.0);
                        // }else if (main_line == 2){
                        //     center_line_points = offsetBevLine(right_overlap_points, min_distance/2.0);
                        // }else{
                        //     if (all_left_dkappas > all_right_dkappas){
                        //         center_line_points = offsetBevLine(right_overlap_points, min_distance/2.0);
                        //     }else{
                        //         center_line_points = offsetBevLine(left_overlap_points, -1.0*min_distance/2.0);
                        //     }
                        // }
                    }else{
                        if (left_overlap_start_index > 0 || left_overlap_end_index < static_cast<int32_t>(left_points.size()) -1){
                            center_line_points = offsetBevLine(right_overlap_points, offset_line_width*0.01); 
                        }else{
                            center_line_points = offsetBevLine(left_overlap_points, -1.0*offset_line_width*0.01); 
                        }
                    }
                }else if (min_distance <= static_cast<double>(offset_line_width)* 0.023 && is_horn_road == false){
                    center_line_points = GetCenterLineCommonOne(left_overlap_points,right_overlap_points,all_right_dkappas,all_left_dkappas,
                                                                main_line,min_distance/2.0, min_distance/2.0);
                    // if (main_line == 1){
                    //     center_line_points = offsetBevLine(left_overlap_points, -1.0*min_distance/2.0);
                    // }else if (main_line == 2){
                    //     center_line_points = offsetBevLine(right_overlap_points, min_distance/2.0);
                    // }else{
                    //     if (all_left_dkappas > all_right_dkappas){
                    //         //LOG_DEBUG("min_distance: " + std::to_string(min_distance));
                    //     center_line_points = offsetBevLine(right_overlap_points, min_distance/2.0);
                    // }else{
                    //     //LOG_DEBUG("min_distance: " + std::to_string(min_distance));
                    //     center_line_points = offsetBevLine(left_overlap_points, -1.0*min_distance/2.0);
                    // }
                }else{
                    //如果重叠部分的最小距离大于偏移线宽度的2%，则选择靠近一边
                    if (merge_dir == 2){
                        center_line_points = GetCenterLineCommonOne(left_overlap_points,right_overlap_points,all_right_dkappas,all_left_dkappas,
                                                                main_line, min_distance - static_cast<double>(offset_line_width) / 100.0, static_cast<double>(offset_line_width) / 100.0);
                        // //向右合流
                        // if (all_left_dkappas > all_right_dkappas){
                        //     center_line_points = offsetBevLine(right_overlap_points, static_cast<double>(offset_line_width) / 100.0);
                        // }else{
                        //     center_line_points = offsetBevLine(left_overlap_points, static_cast<double>(offset_line_width) / 100.0 - min_distance);
                        // }
                    }else if (merge_dir == 1) {
                        center_line_points = GetCenterLineCommonOne(left_overlap_points,right_overlap_points,all_right_dkappas,all_left_dkappas,
                                                                main_line, static_cast<double>(offset_line_width) / 100.0, min_distance - static_cast<double>(offset_line_width) / 100.0);
                        // //向左合流靠近左侧
                        // if (all_left_dkappas > all_right_dkappas){
                        //     center_line_points = offsetBevLine(right_overlap_points, min_distance - static_cast<double>(offset_line_width) / 100.0);
                        // }else{
                        //     center_line_points = offsetBevLine(left_overlap_points, -1.0*static_cast<double>(offset_line_width) / 100.0);
                        // }
                    }else{
                        //如果不是向左或向右合流，且小于wide_lane_width_start取中间
                        if (min_distance*100 < wide_lane_width_start && is_horn_road == false){
                            center_line_points = GetCenterLineCommonOne(left_overlap_points,right_overlap_points,all_right_dkappas,all_left_dkappas,
                                                            main_line, min_distance / 2.0, min_distance / 2.0);
                            // if (all_left_dkappas > all_right_dkappas){
                            //     //LOG_DEBUG("wide_lane_width_start: " + std::to_string(wide_lane_width_start));
                            //     center_line_points = offsetBevLine(right_overlap_points, min_distance / 2.0);
                            //     // double temp_min_distance = std::numeric_limits<double>::max();
                            //     // for (size_t i = 0; i < right_overlap_points.size(); ++i){
                            //     //     PointSLd cur_sl =  PointSLd();
                            //     //     if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(center_line_points, right_overlap_points[i], cur_sl)){
                            //     //         LOG_ERROR("CalPointSLBodyCoordinate error");
                            //     //         return center_line_points;
                            //     //     }
                            //     //     if (fabs(cur_sl.l) < temp_min_distance){
                            //     //         temp_min_distance = fabs(cur_sl.l);
                            //     //     }
                            //     // }
                            //     //LOG_DEBUG("temp_min_distance: " + std::to_string(temp_min_distance));
                            // }else{
                            //     //LOG_DEBUG("wide_lane_width_start: " + std::to_string(wide_lane_width_start));
                            //     center_line_points = offsetBevLine(left_overlap_points, -1.0*min_distance / 2.0);
                            // }
                        }else{
                            //看自车距离哪个边线近，靠近哪一边
                            // LOG_DEBUG("left_overlap_points.back().y: " + std::to_string(left_overlap_points.back().y));
                            // LOG_DEBUG("right_overlap_points.back().y: " + std::to_string(right_overlap_points.back().y));

                            PointSLd left_cur_sl =  PointSLd();
                            bool left_cur_is_inside = false;
                            int32_t left_cur_nearest_index = -1;
                            
                            PointSLd right_cur_sl =  PointSLd();
                            bool right_cur_is_inside = false;
                            int32_t right_cur_nearest_index = -1;
                            EFMPoint ego_point = EFMPoint(0.0, 0.0);

                            if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(left_overlap_points, 
                                                                                                                ego_point, 
                                                                                                                left_cur_sl, left_cur_is_inside, left_cur_nearest_index)){
                                LOG_ERROR("CalPointSLBodyCoordinate error");
                            }
                            if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(right_overlap_points, 
                                                                                                                ego_point, 
                                                                                                                right_cur_sl, right_cur_is_inside, right_cur_nearest_index)){
                                LOG_ERROR("CalPointSLBodyCoordinate error");
                            }
                            
                            if (DEBUG_FLAG == 1)
                            {
                                LOG_DEBUG("left_cur_sl.l: " + std::to_string(left_cur_sl.l));
                                LOG_DEBUG("right_cur_sl.l: " + std::to_string(right_cur_sl.l));
                                LOG_DEBUG("is_horn_road: " + std::to_string(is_horn_road) + "lane_idx: " + std::to_string(lane_idx) + "is_neighbor_lane: " + std::to_string(is_neighbor_lane));
                            }

                            if (is_horn_road == true && lane_idx == 2 && is_neighbor_lane){
                                main_line = 1;
                                center_line_points = GetCenterLineCommonOne(left_overlap_points,right_overlap_points,all_right_dkappas,all_left_dkappas,
                                                                main_line, static_cast<double>(offset_line_width) / 100.0, min_distance - static_cast<double>(offset_line_width) / 100.0);
                            }else if (is_horn_road == true && lane_idx == 1 && is_neighbor_lane){
                                //LOG_DEBUG("is_neighbor_lane");
                                main_line = 2;
                                center_line_points = GetCenterLineCommonOne(left_overlap_points,right_overlap_points,all_right_dkappas,all_left_dkappas,
                                                                main_line, min_distance - static_cast<double>(offset_line_width) / 100.0, static_cast<double>(offset_line_width) / 100.0);
                            
                            }else if (is_horn_road == true && lane_idx == 0 && fabs(left_overlap_points.back().y) <= fabs(right_overlap_points.back().y)){
                                //距离左边车道更近
                                main_line = 1;
                                center_line_points = GetCenterLineCommonOne(left_overlap_points,right_overlap_points,all_right_dkappas,all_left_dkappas,
                                                            main_line, static_cast<double>(offset_line_width) / 100.0, min_distance - static_cast<double>(offset_line_width) / 100.0);
                            
                            }else if (is_horn_road == true && lane_idx == 0 && fabs(left_overlap_points.back().y) > fabs(right_overlap_points.back().y)){
                                //距离左边车道更近
                                main_line = 2;
                                center_line_points = GetCenterLineCommonOne(left_overlap_points,right_overlap_points,all_right_dkappas,all_left_dkappas,
                                                                main_line, min_distance - static_cast<double>(offset_line_width) / 100.0, static_cast<double>(offset_line_width) / 100.0);
                            
                            }else if (fabs(left_cur_sl.l) <= fabs(right_cur_sl.l)){
                                //距离左边车道更近
                                main_line = 1;
                                center_line_points = GetCenterLineCommonOne(left_overlap_points,right_overlap_points,all_right_dkappas,all_left_dkappas,
                                                                main_line, static_cast<double>(offset_line_width) / 100.0, min_distance - static_cast<double>(offset_line_width) / 100.0);
                                // if (all_left_dkappas > all_right_dkappas){
                                //     center_line_points = offsetBevLine(right_overlap_points, min_distance - static_cast<double>(offset_line_width) / 100.0);
                                // }else{
                                //     center_line_points = offsetBevLine(left_overlap_points, -1.0*static_cast<double>(offset_line_width) / 100.0);
                                // }
                            }else{
                                //距离右边车道更近
                                main_line = 2;
                                center_line_points = GetCenterLineCommonOne(left_overlap_points,right_overlap_points,all_right_dkappas,all_left_dkappas,
                                                                main_line, min_distance - static_cast<double>(offset_line_width) / 100.0, static_cast<double>(offset_line_width) / 100.0);
                                // if (all_left_dkappas > all_right_dkappas){
                                //     center_line_points = offsetBevLine(right_overlap_points, static_cast<double>(offset_line_width) / 100.0);
                                // }else{
                                //     center_line_points = offsetBevLine(left_overlap_points, static_cast<double>(offset_line_width) / 100.0 - min_distance);
                                // }
                            }   
                        }
                    }
                }
            }
            
            if (DEBUG_FLAG == 1)
            {
                //遍历center_line_points，打印
                std::string center_line_points_strx = "[";
                std::string center_line_points_stry = "[";
                for (const auto& point : center_line_points) {
                    center_line_points_strx += std::to_string(point.x) +  ",";
                    center_line_points_stry += std::to_string(point.y) + ",";
                }
                LOG_DEBUG(center_line_points_strx + "]");
                LOG_DEBUG(center_line_points_stry + "]");
            }

            {//重叠之前的车道偏移
                if (left_overlap_start_index > 0) {
                    EFMRefLinePoints left_front_points = EFMRefLinePoints();
                    left_front_points.insert(left_front_points.begin(), left_points.begin(), 
                                             left_points.begin() + left_overlap_start_index + 1);
                    if (left_front_points.size() > 1){
                        PointSLd cur_sl =  PointSLd();
                        if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(left_overlap_points, center_line_points[0], cur_sl)){
                            LOG_ERROR("CalPointSLBodyCoordinate error");
                            return center_line_points;
                        }
                        //LOG_DEBUG("cur_sl.l: " + std::to_string(cur_sl.l));
                        EFMRefLinePoints left_offset_points = EFMRefLinePoints();
                        left_offset_points = offsetBevLine(left_front_points, cur_sl.l);
                        center_line_points.insert(center_line_points.begin(), left_offset_points.begin(), left_offset_points.end() - 1);
                    }
                }else if (right_overlap_start_index > 0){
                    EFMRefLinePoints right_front_points = EFMRefLinePoints();
                    right_front_points.insert(right_front_points.begin(), right_points.begin(), 
                                              right_points.begin() + right_overlap_start_index + 1);
                    if (right_front_points.size() > 1){
                        PointSLd cur_sl =  PointSLd();
                        if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(right_overlap_points, center_line_points[0], cur_sl)){
                            LOG_ERROR("CalPointSLBodyCoordinate error");
                            return center_line_points;
                        }
                        //LOG_DEBUG("cur_sl.l: " + std::to_string(cur_sl.l));
                        EFMRefLinePoints right_offset_points = EFMRefLinePoints();
                        right_offset_points = offsetBevLine(right_front_points, cur_sl.l);
                        center_line_points.insert(center_line_points.begin(), right_offset_points.begin(), right_offset_points.end() - 1);
                    }
                } 
            }
            if (DEBUG_FLAG == 1)
            {
                //遍历center_line_points，打印
                std::string center_line_points_strx1 = "[";
                std::string center_line_points_stry1 = "[";
                for (const auto& point : center_line_points) {
                    center_line_points_strx1 += std::to_string(point.x) +  ",";
                    center_line_points_stry1 += std::to_string(point.y) + ",";
                }
                LOG_DEBUG(center_line_points_strx1 + "]");
                LOG_DEBUG(center_line_points_stry1 + "]");
            }

            {//重叠之后的车道偏移
                if (left_overlap_end_index < static_cast<int32_t>(left_points.size()) - 1){
                    EFMRefLinePoints left_back_points = EFMRefLinePoints();
                    left_back_points.insert(left_back_points.begin(), left_points.begin() + left_overlap_end_index, 
                                            left_points.end());
                    if (left_back_points.size() > 1){
                        PointSLd cur_sl =  PointSLd();
                        if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(left_overlap_points, center_line_points.back(), cur_sl)){
                            LOG_ERROR("CalPointSLBodyCoordinate error");
                            return center_line_points;
                        }
                        //LOG_DEBUG("cur_sl.l: " + std::to_string(cur_sl.l));
                        EFMRefLinePoints left_offset_points = EFMRefLinePoints();
                        left_offset_points = offsetBevLine(left_back_points, cur_sl.l);
                        center_line_points.insert(center_line_points.end(), left_offset_points.begin() + 1, left_offset_points.end());
                    }
                }else{
                    EFMRefLinePoints right_back_points = EFMRefLinePoints();
                    right_back_points.insert(right_back_points.begin(), right_points.begin() + right_overlap_end_index, 
                                             right_points.end());
                    if (right_back_points.size() > 1){
                        PointSLd cur_sl =  PointSLd();
                        if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(right_overlap_points, center_line_points.back(), cur_sl)){
                            LOG_ERROR("CalPointSLBodyCoordinate error");
                            return center_line_points;
                        }
                        //LOG_DEBUG("cur_sl.l: " + std::to_string(cur_sl.l));
                        EFMRefLinePoints right_offset_points = EFMRefLinePoints();
                        right_offset_points = offsetBevLine(right_back_points, cur_sl.l);
                        center_line_points.insert(center_line_points.end(), right_offset_points.begin() + 1, right_offset_points.end());
                    }
                }
            }
            if (DEBUG_FLAG == 1)
            {
                //遍历center_line_points，打印
                std::string center_line_points_strx2 = "[";
                std::string center_line_points_stry2 = "[";
                for (const auto& point : center_line_points) {
                    center_line_points_strx2 += std::to_string(point.x) +  ",";
                    center_line_points_stry2 += std::to_string(point.y) + ",";
                }
                LOG_DEBUG(center_line_points_strx2 + "]");
                LOG_DEBUG(center_line_points_stry2 + "]");
            }
        }

    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetCenterLineCommon: " + std::string(e.what()));
        return EFMRefLinePoints(); //返回空
    }

    return center_line_points;
}

bool LocalMap::MakeCenterLineForChangeLane(BevLaneElement& bev_ele, uint8_t prior_dir){

    try
    {
        //如果bev_ele的左右边线都为空，则认为该车道没有中心线
        if (bev_ele.left_line_points.empty() && bev_ele.right_line_points.empty()){
            LOG_ERROR("bev_ele left or right line points is empty");
            return false;
        }

        //如果是向左变道
        if (1 == prior_dir){
            //向左变道，使用左边线的中心线
            bev_ele.center_line_points = GetCenterLineCommon(bev_ele.lane_width_last_cycle, bev_ele.left_line_points, bev_ele.right_line_points, bev_ele.left_types, bev_ele.right_types, 1, 0, -1, false);
        }else if (2 == prior_dir){
            //向右变道，使用右边线的中心线
            bev_ele.center_line_points = GetCenterLineCommon(bev_ele.lane_width_last_cycle, bev_ele.left_line_points, bev_ele.right_line_points, bev_ele.left_types, bev_ele.right_types, 2, 0, -1, false);
        }else{
            //如果不是变道，则认为有问题
            return false;
        }
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::MakeCenterLineForChangeLane: " + std::string(e.what()));
        return false;
    }

    return true;
}

bool LocalMap::GetDistEgoLines(const BevLaneElementGroup& bev_ele_group, std::map<double, 
                               std::vector<EFMRefLinePoints>>& dist_ego_lines_map, 
                               double& min_dist, double& max_dist){

    try
    {
        dist_ego_lines_map.clear();
        min_dist = MAXFLOAT; //最小距离
        max_dist = -MAXFLOAT; //最大距离
        for (const auto& bev_ele : bev_ele_group){
            //判断左侧车道线是否过自车，一个group下所以过自车的线应该是重叠的
            if (!bev_ele.left_line_points.empty() && bev_ele.left_line_points.front().x <= 0 && bev_ele.left_line_points.back().x >= 0){
                //遍历bev_ele, 判断左边线过自车时，Y值正负，来判断加入左车道或者右车道
                for (size_t i = 0; i < bev_ele.left_line_points.size() - 1; i++){
                    if (bev_ele.left_line_points[i].x == 0){
                        dist_ego_lines_map[bev_ele.left_line_points[i].y].push_back(bev_ele.left_line_points);
                        min_dist = std::min(min_dist, bev_ele.left_line_points[i].y);
                        max_dist = std::max(max_dist, bev_ele.left_line_points[i].y);
                        break; //找到后退出循环
                    }else if (bev_ele.left_line_points[i + 1].x == 0){
                        dist_ego_lines_map[bev_ele.left_line_points[i + 1].y].push_back(bev_ele.left_line_points);
                        min_dist = std::min(min_dist, bev_ele.left_line_points[i + 1].y);
                        max_dist = std::max(max_dist, bev_ele.left_line_points[i + 1].y);
                        break; //找到后退出循环
                    }else if (bev_ele.left_line_points[i].x < 0 && bev_ele.left_line_points[i + 1].x > 0){
                        EFMPoint ego_point = EFMPoint(0.0,0.0);
                        EFMRefLinePoints temp_line = EFMRefLinePoints();
                        temp_line.push_back(bev_ele.left_line_points[i]);
                        temp_line.push_back(bev_ele.left_line_points[i + 1]);
                        EFMPoint tar_point = EFMPoint(0.0, 0.0);
                        bool is_inside = false;
                        int nearest_index = -1; 
                        int sign = -1;
                        if (false == CommonTool::DiscretePointsMath::GetInstance()->GetProjectPointBodyCoordinate(ego_point, temp_line, tar_point, is_inside,nearest_index,sign)){
                            LOG_ERROR("GetProjectPointBodyCoordinate error" );
                            continue;
                        }
                        dist_ego_lines_map[tar_point.y].push_back(bev_ele.left_line_points);
                        min_dist = std::min(min_dist, tar_point.y);
                        max_dist = std::max(max_dist, tar_point.y);
                        break; //找到后退出循环
                    }
                    
                }
            }

            //判断右侧车道线是否过自车，一个group下所以过自车的线应该是重叠的
            if (!bev_ele.right_line_points.empty() && bev_ele.right_line_points.front().x <= 0 && bev_ele.right_line_points.back().x >= 0){
                //遍历bev_ele, 判断右边线过自车时，Y值正负，来判断加入左车道或者右车道
                for (size_t i = 0; i < bev_ele.right_line_points.size() - 1; i++){
                    if (bev_ele.right_line_points[i].x == 0){
                        dist_ego_lines_map[bev_ele.right_line_points[i].y].push_back(bev_ele.right_line_points);
                        min_dist = std::min(min_dist, bev_ele.right_line_points[i].y);
                        max_dist = std::max(max_dist, bev_ele.right_line_points[i].y);
                        break; //找到后退出循环
                    }else if (bev_ele.right_line_points[i + 1].x == 0){
                        dist_ego_lines_map[bev_ele.right_line_points[i+1].y].push_back(bev_ele.right_line_points);
                        min_dist = std::min(min_dist, bev_ele.right_line_points[i + 1].y);
                        max_dist = std::max(max_dist, bev_ele.right_line_points[i + 1].y);
                        break; //找到后退出循环
                    }else if (bev_ele.right_line_points[i].x < 0 && bev_ele.right_line_points[i + 1].x > 0){
                        EFMPoint ego_point = EFMPoint(0.0,0.0);
                        EFMRefLinePoints temp_line = EFMRefLinePoints();
                        temp_line.push_back(bev_ele.right_line_points[i]);
                        temp_line.push_back(bev_ele.right_line_points[i + 1]);
                        EFMPoint tar_point = EFMPoint(0.0, 0.0);
                        bool is_inside = false;
                        int nearest_index = -1; 
                        int sign = -1;
                        if (false == CommonTool::DiscretePointsMath::GetInstance()->GetProjectPointBodyCoordinate(ego_point, temp_line, tar_point, is_inside,nearest_index,sign)){
                            LOG_ERROR("GetProjectPointBodyCoordinate error" );
                            continue;
                        }
                        dist_ego_lines_map[tar_point.y].push_back(bev_ele.right_line_points);
                        min_dist = std::min(min_dist, tar_point.y);
                        max_dist = std::max(max_dist, tar_point.y);
                        break; //找到后退出循环
                    }
                }
            }
        }
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("min_dist: " + std::to_string(min_dist) + ", max_dist: " + std::to_string(max_dist));
            LOG_DEBUG("dist_ego_lines_map size: " + std::to_string(dist_ego_lines_map.size()));
            for (auto iter : dist_ego_lines_map){
                LOG_DEBUG("dist_ego_lines_map key: " + std::to_string(iter.first) + ", value size: " + std::to_string(iter.second.size()));
                if (!iter.second.empty()){
                    LOG_DEBUG("First line points size: " + std::to_string(iter.second[0].size()));
                    for (size_t i = 0; i < iter.second.size(); i++){
                        LOG_DEBUG("i=" + std::to_string(i) + ": " + std::to_string(iter.second[i][0].x) + "," + std::to_string(iter.second[i][0].y) + " ... " +
                                std::to_string(iter.second[i].back().x) + "," + std::to_string(iter.second[i].back().y));
                    }
                }
                
            }
        }
        

        if (dist_ego_lines_map.size() > 2){
            //遍历dist_ego_lines_map
            for (const auto& [y, lines] : dist_ego_lines_map) {
                if (y == min_dist || y == max_dist){
                    continue; //跳过
                }else{
                    if (std::abs(y - min_dist) < std::abs(y - max_dist)){
                        //如果y距离最小距离更近，则认为是左边线
                        dist_ego_lines_map[min_dist].insert(dist_ego_lines_map[min_dist].end(), lines.begin(), lines.end());
                    }else{
                        dist_ego_lines_map[max_dist].insert(dist_ego_lines_map[max_dist].end(), lines.begin(), lines.end());
                    }
                }
            }
        }

        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("dist_ego_lines_map size: " + std::to_string(dist_ego_lines_map.size()));
            for (auto iter : dist_ego_lines_map){
                LOG_DEBUG("dist_ego_lines_map key: " + std::to_string(iter.first) + ", value size: " + std::to_string(iter.second.size()));
                if (!iter.second.empty()){
                    LOG_DEBUG("First line points size: " + std::to_string(iter.second[0].size()));
                    for (size_t i = 0; i < iter.second.size(); i++){
                        LOG_DEBUG("i=" + std::to_string(i) + ": " + std::to_string(iter.second[i][0].x) + "," + std::to_string(iter.second[i][0].y) + " ... " +
                                std::to_string(iter.second[i].back().x) + "," + std::to_string(iter.second[i].back().y));
                    }
                }
                
            }
        }
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetDistEgoLines: " + std::string(e.what()));
        return false;
    }

    return true; 
}

bool LocalMap::GetNearIdxForMainLine(const BevLaneElementGroup& bev_ele_group, EFMRefLinePoints& main_left_line_points, 
                                     EFMRefLinePoints& main_right_line_points, int32_t& cur_nearest_index){
    try
    {
        cur_nearest_index = 0;
        if (main_left_line_points.empty() || main_right_line_points.empty()) return false;

        auto left_it = std::find_if(main_left_line_points.begin(), main_left_line_points.end(), 
                    [&](const EFMPoint& point){ return point.x > 0; });
        auto right_it = std::find_if(main_right_line_points.begin(), main_right_line_points.end(), 
                    [&](const EFMPoint& point){ return point.x > 0; });
        if (left_it == main_left_line_points.end() || right_it == main_right_line_points.end() ||
            left_it == main_left_line_points.begin() || right_it == main_right_line_points.begin()){
            LOG_ERROR("left_it or right_it is end");
            return false;
        }
        int32_t left_index = static_cast<int32_t>(std::distance(main_left_line_points.begin(), left_it));
        int32_t right_index = static_cast<int32_t>(std::distance(main_right_line_points.begin(), right_it));

        EFMRefLinePoints left_temp_points = EFMRefLinePoints();
        left_temp_points.push_back(*(left_it - 1));
        left_temp_points.push_back(*left_it);
        EFMRefLinePoints right_temp_points = EFMRefLinePoints();
        right_temp_points.push_back(*(right_it - 1));
        right_temp_points.push_back(*right_it);
        EFMPoint ego_point = EFMPoint(0.0, 0.0);

        PointSLd left_sl =  PointSLd();
        if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(left_temp_points, ego_point, left_sl)){
            LOG_ERROR("CalPointSLBodyCoordinate error");
            return false;
        }

        PointSLd right_sl =  PointSLd();
        if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(right_temp_points, ego_point, right_sl)){
            LOG_ERROR("CalPointSLBodyCoordinate error");
            return false;
        }

        double lane_width = (fabs(left_sl.l) + fabs(right_sl.l)) / (static_cast<double>(bev_ele_group.size()));
        if (fabs(left_sl.l) <= lane_width + 0.1){
            cur_nearest_index = static_cast<int32_t>(left_temp_points.size()) - 1;
        }else if (fabs(right_sl.l) <= lane_width + 0.1){
            cur_nearest_index = 0;
        }else{
            uint32_t left_num = std::ceil(fabs(left_sl.l)/lane_width);
            cur_nearest_index = left_num > static_cast<int32_t>(left_temp_points.size()) ? 0 : static_cast<int32_t>(left_temp_points.size()) - std::ceil(fabs(left_sl.l)/lane_width);
        }
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetNearIdxForMainLine: " + std::string(e.what()));
        return false;
    }
    return true;
}


bool LocalMap::GetMainLinePoint(const BevLaneElementGroup& bev_ele_group, EFMRefLinePoints& main_left_line_points, 
                                EFMRefLinePoints& main_right_line_points, int32_t lane_idx, bool& is_wide,
                                std::vector<LineTypeInfo>& main_left_types, std::vector<LineTypeInfo>& main_right_types,
                                uint8_t& main_line){

    try
    {
        is_wide = false;
        main_left_line_points.clear();
        main_right_line_points.clear();
        main_left_types.clear();
        main_right_types.clear();
        //获取主路线的左右边线，默认两边都是实线
        // LineTypeInfo one_type;
        // one_type.is_valid = true;
        // one_type.start_point = 0.0;
        // one_type.line_type = 1;
        // one_type.typ_chg_point = MAXFLOAT;
        // one_type.typ_aft_chg_point = 1;
        // main_left_types.push_back(one_type);
        // main_right_types.push_back(one_type);

        main_line = 0;

        //如果bev_ele_group为空，直接返回
        if (bev_ele_group.empty() || bev_ele_group.size() < 2) return false;

        //遍历bev_ele_group，找到主线的左右边线
        // std::map<double, std::vector<EFMRefLinePoints>> dist_ego_lines_map;
        // double min_dist = MAXFLOAT; //最小距离
        // double max_dist = -MAXFLOAT; //最大距离
        // if (false == GetDistEgoLines(bev_ele_group, dist_ego_lines_map, min_dist, max_dist)){
        //     LOG_ERROR("GetDistEgoLines error");
        //     return false;
        // }
        // LOG_DEBUG("min_dist: " + std::to_string(min_dist) + ", max_dist: " + std::to_string(max_dist));

        // std::vector<EFMRefLinePoints> left_lines;
        // std::vector<EFMRefLinePoints> right_lines;

        // if (dist_ego_lines_map.empty()) return false;

        // if (dist_ego_lines_map.size() == 1){
        //     if (lane_idx == 1){
        //         left_lines = dist_ego_lines_map.begin()->second;
        //         if (bev_ego_ele_.empty() || bev_ego_ele_.back().left_line_points.empty()){
        //             return false;
        //         }
        //         right_lines.push_back(bev_ego_ele_.back().left_line_points); //如果只有一条线，则认为右边线是自车所在车道的左边线
        //     }else if (lane_idx == 2){
        //         right_lines = dist_ego_lines_map.begin()->second;
        //         if (bev_ego_ele_.empty() || bev_ego_ele_.front().right_line_points.empty()){
        //             return false;
        //         }
        //         left_lines.push_back(bev_ego_ele_.front().right_line_points); //如果只有一条线，则认为左边线是自车所在车道的右边线
        //     }else{
        //         LOG_ERROR("group only one line, lane_idx=" + std::to_string(lane_idx));
        //         return false;
        //     }
            
        // }else{
        //     if (dist_ego_lines_map.find(min_dist) == dist_ego_lines_map.end() || 
        //         dist_ego_lines_map.find(max_dist) == dist_ego_lines_map.end()){
        //         LOG_ERROR("min_dist or max_dist not found in dist_ego_lines_map");
        //         return false; //如果没有找到最小距离和最大距离，直接返回
        //     }

        //     left_lines = dist_ego_lines_map[max_dist];
        //     right_lines = dist_ego_lines_map[min_dist];

        //     LOG_DEBUG("left_lines size: " + std::to_string(left_lines.size()));
        //     for (size_t i = 0; i < left_lines.size(); i++){
        //         LOG_DEBUG("i=" + std::to_string(i) + ": " + std::to_string(left_lines[i][0].x) + "," + std::to_string(left_lines[i][0].y) + " ... " +
        //                 std::to_string(left_lines[i].back().x) + "," + std::to_string(left_lines[i].back().y));
        //     }

        //     LOG_DEBUG("right_lines size: " + std::to_string(right_lines.size()));
        //     for (size_t i = 0; i < right_lines.size(); i++){
        //         LOG_DEBUG("i=" + std::to_string(i) + ": " + std::to_string(right_lines[i][0].x) + "," + std::to_string(right_lines[i][0].y) + " ... " +
        //                 std::to_string(right_lines[i].back().x) + "," + std::to_string(right_lines[i].back().y));
        //     }



        //     //如果没有找到左边线和右边线，直接返回
        //     if (left_lines.empty() && right_lines.empty()){
        //         LOG_ERROR("left_lines and right_lines is empty");
        //         return false;
        //     }
        // }
        
        // //如果左边线不为空，找出左边重叠部分
        // if (!left_lines.empty()){
        //     //获取左边线的重叠部分
        //     main_left_line_points.insert(main_left_line_points.begin(), left_lines[0].begin(), left_lines[0].end());
        //     for (size_t i = 1; i < left_lines.size(); ++i){
        //         //遍历left_lines[i],找到与main_left_line_points重叠的部分
        //         for (size_t j = 0; j < left_lines[i].size(); ++j){
        //             auto it = std::find_if(main_left_line_points.begin(), main_left_line_points.end(), 
        //                              [&](const EFMPoint& point){
        //                                  return point.x == left_lines[i][j].x && point.y == left_lines[i][j].y; //判断X轴位置是否相近
        //                              });
        //             if (it != main_left_line_points.end()){
        //                 //删除main_left_line_points中找到的之前的点
        //                 main_left_line_points.erase(main_left_line_points.begin(), it);
        //                 //比较left_lines[i]的剩余部分与main_left_line_points的重叠部分
        //                 for (size_t idx = 0; (idx + j + 1 < left_lines[i].size()) && (idx < main_left_line_points.size()); idx++){
        //                     if (left_lines[i][j + idx + 1].x == main_left_line_points[idx].x && 
        //                         left_lines[i][j + idx + 1].y == main_left_line_points[idx].y){
        //                         //如果找到重叠部分，直接退出循环
        //                         continue;
        //                     }else{
        //                         main_left_line_points.erase(main_left_line_points.begin() + idx, main_left_line_points.end());
        //                         break;
        //                     }
        //                 }
        //                 break; //找到后退出循环
        //             }
        //         }
        //     }
        // }

        // //如果右边线不为空，找出右边重叠部分
        // if (!right_lines.empty()){
        //     //获取右边线的重叠部分
        //     main_right_line_points.insert(main_right_line_points.begin(), right_lines[0].begin(), right_lines[0].end());
        //     for (size_t i = 1; i < right_lines.size(); ++i){
        //         //遍历main_right_line_points,删除与right_lines[i]不重叠的部分
        //         for (int32_t j = main_right_line_points.size() - 1; j >= 0; j--){
        //             auto it = std::find_if(right_lines[i].begin(), right_lines[i].end(), 
        //                              [&](const EFMPoint& point){
        //                                  return point.x == main_right_line_points[j].x && point.y == main_right_line_points[j].y; //判断X轴位置是否相近
        //                              });
        //             if (it != right_lines[i].end()){
        //                 break; //找到后退出循环
        //             }else{
        //                 main_right_line_points.erase(main_right_line_points.begin() + j);
        //             }
        //         }
                
        //         // for (size_t j = 0; j < right_lines[i].size(); ++j){
        //         //     auto it = std::find_if(main_right_line_points.begin(), main_right_line_points.end(), 
        //         //                      [&](const EFMPoint& point){
        //         //                          return point.x == right_lines[i][j].x && point.y == right_lines[i][j].y; //判断X轴位置是否相近
        //         //                      });
        //         //     if (it != main_right_line_points.end()){
        //         //         //删除main_right_line_points中找到的之前的点
        //         //         main_right_line_points.erase(main_right_line_points.begin(), it);
        //         //         //比较right_lines[i]的剩余部分与main_right_line_points的重叠部分
        //         //         for (size_t idx = 0; (idx + j + 1 < right_lines[i].size()) && (idx < main_right_line_points.size()); idx++){
        //         //             if (right_lines[i][j + idx + 1].x == main_right_line_points[idx].x && 
        //         //                 right_lines[i][j + idx + 1].y == main_right_line_points[idx].y){
        //         //                 //如果找到重叠部分，直接退出循环
        //         //                 continue;
        //         //             }else{
        //         //                 main_right_line_points.erase(main_right_line_points.begin() + idx, main_right_line_points.end());
        //         //                 break;
        //         //             }
        //         //         }
        //         //         break; //找到后退出循环
        //         //     }
        //         // }
        //     }
        // }

        //根据数据新的特性，修改做成方法
        EFMRefLinePoints org_main_left_line_points = bev_ele_group[bev_ele_group.size() - 1].left_line_points;
        EFMRefLinePoints org_main_right_line_points = bev_ele_group[0].right_line_points;
        main_left_types = bev_ele_group[bev_ele_group.size() - 1].left_types;
        main_right_types = bev_ele_group[0].right_types;
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("bev_ele_group[bev_ele_group.size() - 1].left_line_split_index=" + std::to_string(bev_ele_group[bev_ele_group.size() - 1].left_line_split_index));
            LOG_DEBUG("bev_ele_group[bev_ele_group.size() - 1].left_line_split_x=" + std::to_string(bev_ele_group[bev_ele_group.size() - 1].left_line_split_x));
        }

        if (bev_ele_group[bev_ele_group.size() - 1].left_line_split_index >= 0 &&
            bev_ele_group[bev_ele_group.size() - 1].left_line_split_index < org_main_left_line_points.size()){
                main_line = 2; //左边线为主线
                auto iter = std::find_if(org_main_left_line_points.begin(), org_main_left_line_points.end(), 
                                            [&](const EFMPoint& point){ return point.x >= bev_ele_group[bev_ele_group.size() - 1].left_line_split_x; });
                if (iter != org_main_left_line_points.end()){
                    int32_t insert_num = std::distance(org_main_left_line_points.begin(), iter);
                    main_left_line_points.insert(main_left_line_points.begin(), 
                                            org_main_left_line_points.begin(),
                                            org_main_left_line_points.begin() + insert_num + 1);
                }
        }else{
            main_left_line_points = org_main_left_line_points;
        }

        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("bev_ele_group[0].right_line_split_index=" + std::to_string(bev_ele_group[0].right_line_split_index));
            LOG_DEBUG("bev_ele_group[0].right_line_split_x=" + std::to_string(bev_ele_group[0].right_line_split_x));
        }
        if (bev_ele_group[0].right_line_split_index >= 0 && 
            bev_ele_group[0].right_line_split_index < org_main_right_line_points.size() - 1){
                if (main_line == 2){
                    //如果左边线为主线，则认为双侧都是分歧点，不设置主线
                    main_line = 0;
                }else{
                    main_line = 1; //右边线为主线
                }
                
                auto iter = std::find_if(org_main_right_line_points.begin(), org_main_right_line_points.end(), 
                                            [&](const EFMPoint& point){ return point.x >= bev_ele_group[0].right_line_split_x; });
                if (iter != org_main_right_line_points.end()){
                    int32_t insert_num = std::distance(org_main_right_line_points.begin(), iter);
                    main_right_line_points.insert(main_right_line_points.begin(), 
                                            org_main_right_line_points.begin(),
                                            org_main_right_line_points.begin() + insert_num + 1);
                }
        }else{
            main_right_line_points = org_main_right_line_points;
        }
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("Before delete, main_left_line_points size=" + std::to_string(main_left_line_points.size()) +
                ", main_right_line_points size=" + std::to_string(main_right_line_points.size()));
        }

        double right_line_length = CommonTool::DiscretePointsMath::GetInstance()->LineLength(main_right_line_points);
        double left_line_length = CommonTool::DiscretePointsMath::GetInstance()->LineLength(main_left_line_points);
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("right_line_length=" + std::to_string(right_line_length) + ", left_line_length=" + std::to_string(left_line_length));
        }

        org_main_left_line_points = main_left_line_points;
        org_main_right_line_points = main_right_line_points;
        //比较距离，删除大于virtual_split_start_width的点
        if (!main_left_line_points.empty() && !main_right_line_points.empty()){
            //遍历左边，删除距离大于virtual_split_start_width的点
            
            for (int32_t i = main_left_line_points.size() - 1; i >= 0; --i){
                if (main_left_line_points[i].x <= main_right_line_points.back().x && bev_ele_group[0].right_line_split_index != -1){
                    //另一遍是分歧点，且当前点的X轴位置小于等于右边线的最后一个点的X轴位置，退出循环
                    break;
                }
                
                PointSLd cur_sl =  PointSLd();
                if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(main_right_line_points, main_left_line_points[i], cur_sl)){
                    //遍历ele.right_line_points
                    std::string right_points_strx = "[";
                    std::string right_points_stry = "[";
                    for (const auto& point : main_right_line_points) {
                        right_points_strx += std::to_string(point.x) +  ",";
                        right_points_stry += std::to_string(point.y) + ",";
                    }
                    LOG_DEBUG("right ix=" + std::to_string(i)  + ":"+ right_points_strx + "]");
                    LOG_DEBUG("right iy=" + std::to_string(i)  + ":"+ right_points_stry + "]");
                    LOG_DEBUG("left x=" + std::to_string(main_left_line_points[i].x) + ", y=" + std::to_string(main_left_line_points[i].y));
                    LOG_ERROR("CalPointSLBodyCoordinate error");
                    return false;
                }
                //LOG_DEBUG("i=" + std::to_string(i) + ", sl.l=" + std::to_string(cur_sl.l) + ", cur_sl.s=" + std::to_string(cur_sl.s));
                //如果当前点的X轴位置大于virtual_split_start_width，则认为距离过大，删除该点
                if (fabs(cur_sl.l *100.0) > virtual_split_start_width || cur_sl.s < -1.5 || cur_sl.s > right_line_length + 1.5){
                    //LOG_DEBUG("i=" + std::to_string(i));
                    main_left_line_points.erase(main_left_line_points.begin() + i);
                }else{
                    break; //如果当前点的X轴位置小于等于virtual_split_start_width，则认为距离足够，退出循环
                }
            }
            if (main_left_line_points.size() < 2){
                main_left_line_points = org_main_left_line_points;
                main_right_line_points = org_main_right_line_points;
                is_wide = true;
                return true;
            }
            
            if (DEBUG_FLAG == 1)
            {
                LOG_DEBUG("Before delete, main_left_line_points size=" + std::to_string(main_left_line_points.size()) +
                        ", main_right_line_points size=" + std::to_string(main_right_line_points.size()));
            }
            right_line_length = CommonTool::DiscretePointsMath::GetInstance()->LineLength(main_right_line_points);
            left_line_length = CommonTool::DiscretePointsMath::GetInstance()->LineLength(main_left_line_points);
            //遍历右边，删除距离大于virtual_split_start_width的点
            for (int32_t i = main_right_line_points.size() - 1; i >= 0 && bev_ele_group[0].right_line_split_index == -1; --i){
                if (main_right_line_points[i].x <= main_left_line_points.back().x && bev_ele_group[bev_ele_group.size() - 1].left_line_split_index >= 0){
                    //另一遍是分歧点，且当前点的X轴位置小于等于左边线的最后一个点的X轴位置，退出循环
                    break;
                }

                PointSLd cur_sl =  PointSLd();
                if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(main_left_line_points, main_right_line_points[i], cur_sl)){
                    //遍历ele.right_line_points
                    std::string right_points_strx = "[";
                    std::string right_points_stry = "[";
                    for (const auto& point : main_left_line_points) {
                        right_points_strx += std::to_string(point.x) +  ",";
                        right_points_stry += std::to_string(point.y) + ",";
                    }
                    LOG_DEBUG("left ix=" + std::to_string(i)  + ":"+ right_points_strx + "]");
                    LOG_DEBUG("left iy=" + std::to_string(i)  + ":"+ right_points_stry + "]");
                    LOG_DEBUG("right x=" + std::to_string(main_right_line_points[i].x) + ", y=" + std::to_string(main_right_line_points[i].y));
                    LOG_ERROR("CalPointSLBodyCoordinate error");
                    return false;
                }
                //LOG_DEBUG("i=" + std::to_string(i) + ", sl.l=" + std::to_string(cur_sl.l) + ", cur_sl.s=" + std::to_string(cur_sl.s));
                //如果当前点的X轴位置大于virtual_split_start_width，则认为距离过大，删除该点
                if (fabs(cur_sl.l *100.0) > virtual_split_start_width || cur_sl.s < -1.5 || cur_sl.s > left_line_length + 1.5){
                    //LOG_DEBUG("i=" + std::to_string(i));
                    main_right_line_points.erase(main_right_line_points.begin() + i);
                }else{
                    break; //如果当前点的X轴位置小于等于virtual_split_start_width，则认为距离足够，退出循环
                }
            }
            if (DEBUG_FLAG == 1)
            {
                LOG_DEBUG("After delete, main_left_line_points size=" + std::to_string(main_left_line_points.size()) +
                        ", main_right_line_points size=" + std::to_string(main_right_line_points.size()));
            }

            if (main_right_line_points.size() < 2){
                main_left_line_points = org_main_left_line_points;
                main_right_line_points = org_main_right_line_points;
                is_wide = true;
                return true;
            }

        }
        return true; //如果找到了主线，返回true
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetMainLinePoint: " + std::string(e.what()));
        return false;
    }

    return true; //如果没有找到主线，返回false
}

void LocalMap::DeleteLineForStartMerge(EFMRefLinePoints& left_line_points, EFMRefLinePoints& right_line_points){

    try
    {
        //如果左右边线都为空，直接返回
        if (left_line_points.empty() || right_line_points.empty()){
            return;
        }

        double right_line_length = CommonTool::DiscretePointsMath::GetInstance()->LineLength(right_line_points);
        double left_line_length = CommonTool::DiscretePointsMath::GetInstance()->LineLength(left_line_points);

        //遍历左边线，删除宽度小于merge_lane_start_width的点
        for (int32_t i = 0; i < left_line_points.size(); ++i){
            PointSLd cur_sl =  PointSLd();
            if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(right_line_points, left_line_points[i], cur_sl)){
                LOG_ERROR("CalPointSLBodyCoordinate error");
                return;
            }
            //如果当前点的X轴位置小于merge_lane_start_width，则认为距离短，删除该点
            if (fabs(cur_sl.l *100.0) < merge_lane_start_width || cur_sl.s < -1.5 || cur_sl.s > right_line_length + 1.5){
                left_line_points.erase(left_line_points.begin() + i);
                --i; //删除后需要回退索引
            }else{
                break; //如果当前点的X轴位置大于等于merge_lane_start_width，则认为距离足够，退出循环
            }
        }

        //遍历右边线，删除宽度小于merge_lane_start_width的点
        for (int32_t i = 0; i < right_line_points.size(); ++i){
            PointSLd cur_sl =  PointSLd();
            if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(left_line_points, right_line_points[i], cur_sl)){
                LOG_ERROR("x: " + std::to_string(right_line_points[i].x) + ", y: " + std::to_string(right_line_points[i].y));
                //遍历left_line_points，打印每个点的X和Y值
                std::string ss_x = "i: " + std::to_string(i) + ":";
                std::string ss_y = "i: " + std::to_string(i) + ":";
                for (const auto& point : left_line_points) {
                    ss_x += std::to_string(point.x) + ",";
                    ss_y += std::to_string(point.y) + ",";
                }
                LOG_ERROR(ss_x);
                LOG_ERROR(ss_y);
                LOG_ERROR("CalPointSLBodyCoordinate error");
                return;
            }
            //如果当前点的X轴位置小于merge_lane_start_width，则认为距离短，删除该点
            if (fabs(cur_sl.l *100.0) < merge_lane_start_width || cur_sl.s < -1.5 || cur_sl.s > left_line_length + 1.5){
                right_line_points.erase(right_line_points.begin() + i);
                --i; //删除后需要回退索引
            }else{
                break; //如果当前点的X轴位置大于等于merge_lane_start_width，则认为距离足够，退出循环
            }
        }
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::DeleteLineForStartMerge: " + std::string(e.what()));
    }
}


bool LocalMap::AlterLineForMainLine(BevLaneElement& bev_ele, EFMRefLinePoints& center_line_points, 
                                     const BevLaneElementGroup& bev_ele_group){
    //做成步骤
    //1，向后延长中心线到bev_ele.center_line_points.back().x位置
    //2，判断延长后的中心线和bev_ele.center_line_points.back().x横向距离
    //3，如果横向距离小于一个车道则移动center_line_points到ele.center_line_points
    //4，根据横向位置或者中间是否经过实际车道线，判断查几个车道，根据查几个车道，判断横向步长，1个车道给lane_offset_step_length的长度
    //5，缩短延长后center_line_points
    try
    {
        //如果中心线点为空，则认为不需要修正
        if (center_line_points.size() < 2 || bev_ele.center_line_points.size() < 2){
            return false;
        }

        //向后延长中心线到bev_ele.center_line_points.back().x位置
        double dir_length = std::hypot(center_line_points[0].x - center_line_points[1].x, 
                                       center_line_points[0].y - center_line_points[1].y);
        EFMPoint p_extern_dir((center_line_points[0].x - center_line_points[1].x)/dir_length,  (center_line_points[0].y - center_line_points[1].y)/dir_length);                               
        double max_length = std::hypot(center_line_points[0].x - bev_ele.center_line_points.back().x, 
                                       center_line_points[0].y - bev_ele.center_line_points.back().y);
        EFMPoint extern_p=CommonTool::DiscretePointsMath::GetInstance()-> ExtendPoint(center_line_points[0], p_extern_dir, max_length);
        center_line_points.insert(center_line_points.begin(), extern_p);

        //判断延长后的中心线和bev_ele.center_line_points.back().x横向距离
        PointSLd start_sl = PointSLd();
        bool is_inside = false;
        EFMPoint point_front,  point_back, proj_point_res;
        if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(center_line_points, 
                                                                                             bev_ele.center_line_points.back(), start_sl,
                                                                                             is_inside, point_front, point_back, proj_point_res)){
            LOG_ERROR("CalPointSLBodyCoordinate error");
            return false;
        }
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("start_sl.l=" + std::to_string(start_sl.l) + ", start_sl.s=" + std::to_string(start_sl.s) +
                    ", extern_p=(" + std::to_string(extern_p.x) + "," + std::to_string(extern_p.y) + ")" + ", proj_point_res=" + std::to_string(proj_point_res.x) + "," + std::to_string(proj_point_res.y));
        }

        //根据横向距离进行判断，中心线移动距离
        if (fabs(start_sl.l * 100.0) < offset_line_width){
            //如果横向距离小于offset_line_width,同一个车道，直接返回
            center_line_points.erase(center_line_points.begin()); //删除延长的点
        }else if (fabs(start_sl.l * 100.0) < offset_line_width + virtual_split_start_width){
            //如果横向距离大于offset_line_width，小于 wide_lane_width_start，说明是差一个车道
            center_line_points[0] = proj_point_res;
            //遍历center_line_points，找到距离center_line_points[0]等于lane_offset_step_length的点
            double acc_length = 0.0;

            for (size_t i = 1; i < center_line_points.size(); ++i){
                //计算当前点和前一个点的距离
                double temp_length = std::hypot(center_line_points[i].x - center_line_points[i-1].x, 
                                       center_line_points[i].y - center_line_points[i-1].y);
                acc_length += temp_length;
                if (acc_length * 100.0 == lane_offset_step_length){
                    center_line_points.erase(center_line_points.begin(), center_line_points.begin() + i);
                    break;
                }else if (acc_length * 100.0 > lane_offset_step_length){
                    //如果超过了lane_offset_step_length，则删除多余的点
                    // double dir_length = std::hypot(center_line_points[i-1].x - center_line_points[i].x, 
                    //                 center_line_points[i-1].y - center_line_points[i].y);
                    EFMPoint p_extern_dir((center_line_points[i-1].x - center_line_points[i].x)/temp_length,  (center_line_points[i-1].y - center_line_points[i].y)/temp_length);
                    double max_length = acc_length - lane_offset_step_length/100.0 ;
                    // double max_length = std::hypot(center_line_points[0].x - bev_ele.center_line_points.back().x,
                    //                             center_line_points[0].y - bev_ele.center_line_points.back().y);
                    EFMPoint temp_extern_p=CommonTool::DiscretePointsMath::GetInstance()-> ExtendPoint(center_line_points[i], p_extern_dir, max_length);
                    center_line_points.erase(center_line_points.begin(), center_line_points.begin() + i);
                    center_line_points.insert(center_line_points.begin(), temp_extern_p);
                    break;
                }
                
            }
            if (DEBUG_FLAG == 1)
            {
                LOG_DEBUG(", acc_length=" + std::to_string(acc_length));
            }
            if (acc_length * 100.0 < lane_offset_step_length){
                return false; //如果没有找到满足条件的点，返回false
            }
        }else if (fabs(start_sl.l * 100.0) < offset_line_width + 2*virtual_split_start_width){
            //如果横向距离大于wide_lane_width_start，小于merge_lane_start_width，说明是差两个车道
            center_line_points[0] = proj_point_res;
            //遍历center_line_points，找到距离center_line_points[0]等于2*lane_offset_step_length的点
            double acc_length = 0.0;
            for (size_t i = 1; i < center_line_points.size(); ++i){
                // acc_length += std::hypot(center_line_points[i].x - center_line_points[i-1].x, 
                //                        center_line_points[i].y - center_line_points[i-1].y);
                // if (acc_length * 100.0 >= lane_offset_step_length * 2){
                //     center_line_points.erase(center_line_points.begin() + i, center_line_points.end());
                //     break;
                // }
                                //计算当前点和前一个点的距离
                double temp_length = std::hypot(center_line_points[i].x - center_line_points[i-1].x, 
                                       center_line_points[i].y - center_line_points[i-1].y);
                acc_length += temp_length;
                if (acc_length * 100.0 == 2.0*lane_offset_step_length){
                    center_line_points.erase(center_line_points.begin(), center_line_points.begin() + i);
                    break;
                }else if (acc_length * 100.0 > lane_offset_step_length*2.0){
                    //如果超过了lane_offset_step_length，则删除多余的点
                    // double dir_length = std::hypot(center_line_points[i-1].x - center_line_points[i].x, 
                    //                 center_line_points[i-1].y - center_line_points[i].y);
                    EFMPoint p_extern_dir((center_line_points[i-1].x - center_line_points[i].x)/temp_length,  (center_line_points[i-1].y - center_line_points[i].y)/temp_length);
                    double max_length = acc_length - 2.0*lane_offset_step_length/100.0 ;
                    // double max_length = std::hypot(center_line_points[0].x - bev_ele.center_line_points.back().x,
                    //                             center_line_points[0].y - bev_ele.center_line_points.back().y);
                    EFMPoint temp_extern_p=CommonTool::DiscretePointsMath::GetInstance()-> ExtendPoint(center_line_points[i], p_extern_dir, max_length);
                    center_line_points.erase(center_line_points.begin(), center_line_points.begin() + i);
                    center_line_points.insert(center_line_points.begin(), temp_extern_p);
                    break;
                }
            }
            if (acc_length * 100.0 < lane_offset_step_length * 2){
                return false; //如果没有找到满足条件的点，返回false
            }
        }else{
            return false; //横向距离大于2个车道，直接返回
        }
        return true;
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::AlterLineForMainLine: " + std::string(e.what()));
        return false;
    }

 }

// EFMRefLinePoints LocalMap::GenerateCenterLine(const EFMRefLinePoints& left_line_points, 
//                                                 const EFMRefLinePoints& right_line_points, int32_t offset){

//     EFMRefLinePoints center_line_points;
//     //如果左右边线都为空，直接返回空
//     if (left_line_points.empty() || right_line_points.empty()){
//         return center_line_points;
//     }

//     //生成中心线
//     for (size_t i = 0; i < left_line_points.size(); ++i){
//         EFMPoint center_point;
//         center_point.x = (left_line_points[i].x + right_line_points[i].x) / 2 + offset;
//         center_point.y = (left_line_points[i].y + right_line_points[i].y) / 2 + offset;
//         center_line_points.push_back(center_point);
//     }

//     return center_line_points;
// }

//GetLinePointUpX
bool LocalMap::GetLinePointUpX(const EFMRefLinePoints& left_line_points, const EFMRefLinePoints& right_line_points, 
                                EFMRefLinePoints& up_left_line_points, EFMRefLinePoints& up_right_line_points, double x){

    try
    {
        up_left_line_points.clear();
        up_right_line_points.clear();

        //如果左右边线都为空，直接返回空
        if (left_line_points.empty() && right_line_points.empty()){
            return false;
        }

        //获取左边线和右边线在x处的点
        for (int32_t i = 0; i < static_cast<int32_t>(left_line_points.size()); ++i){
            if (left_line_points[i].x >= x){
                up_left_line_points.insert(up_left_line_points.end(), left_line_points.begin() + i, left_line_points.end());
                break;
            }
        }

        for (int32_t i = 0; i < static_cast<int32_t>(right_line_points.size()); ++i){
            if (right_line_points[i].x >= x){
                up_right_line_points.insert(up_right_line_points.end(), right_line_points.begin() + i, right_line_points.end());
                break;
            }
        }

        return true;
    }
    catch(const std::exception& e)
    {
        std::cout << "LocalMap::GetLinePointUpX: "<< e.what() << std::endl;
        return false;
    }
}

bool LocalMap::MakeCenterLineSide(BevLaneElementGroup& bev_ele_group, int32_t lane_idx){

    try
    {
        //为空直接返回,不用做
        if (bev_ele_group.empty()) return true;



    
    }
    catch(const std::exception& e)
    {
        std::cout << "LocalMap::MakeCenterLineSide: " << lane_idx << e.what() << std::endl;
        return false;
    }

    return true;
}

bool LocalMap::GetMergeDir(BevLaneElement& bev_ele, uint8_t& merge_dir, int32_t& left_merge_point, int32_t& right_merge_point, 
                           int32_t& left_narrow_pos, int32_t& right_narrow_pos, int32_t lane_idx){

    try
    {
        merge_dir = 0;
        left_merge_point = -1;
        right_merge_point = -1;
        left_narrow_pos = -1;
        right_narrow_pos = -1;
        if (bev_ele.left_line_points.empty() || bev_ele.right_line_points.empty()){
            return false;
        }

        //判断左右车道线前方是否存在left_line_merge_index、right_line_merge_index
        if (bev_ele.left_line_merge_index < 0 || bev_ele.right_line_merge_index < 0){
            //如果左右车道线的合流点都小于0，说明没有直接向交的合流点
            //通过左右线中更长的车道边线和旁车道边线比较，判断是否是虚拟合流
            if ((bev_ele.left_line_points.back().x < bev_ele.right_line_points.back().x) && 
                lane_idx == 0 && !bev_left_ele_.empty()){
                auto& left_bev_ele = bev_left_ele_[0];
                if (IsVirtualMerge(bev_ele, left_bev_ele, right_merge_point, right_narrow_pos, true)){
                    merge_dir = 1; //向左merge
                }
            }

            if ((bev_ele.left_line_points.back().x < bev_ele.right_line_points.back().x) && 
                lane_idx == 2 && !bev_ego_ele_.empty()){
                auto& left_bev_ele = bev_ego_ele_[0];
                if (IsVirtualMerge(bev_ele, left_bev_ele, right_merge_point, right_narrow_pos, true)){
                    merge_dir = 1; //向左merge
                }
            }

            if ((bev_ele.left_line_points.back().x > bev_ele.right_line_points.back().x) && 
                lane_idx == 0 && !bev_right_ele_.empty()){
                auto& right_bev_ele = bev_right_ele_[bev_right_ele_.size() - 1];
                if (IsVirtualMerge(bev_ele, right_bev_ele, left_merge_point, left_narrow_pos, false)){
                    merge_dir = 2; //向右merge
                }
            }

            if ((bev_ele.left_line_points.back().x > bev_ele.right_line_points.back().x) && 
                lane_idx == 1 && !bev_ego_ele_.empty()){
                auto& right_bev_ele = bev_ego_ele_[bev_ego_ele_.size() - 1];
                if (IsVirtualMerge(bev_ele, right_bev_ele, left_merge_point, left_narrow_pos, false)){
                    merge_dir = 2; //向右merge
                }
            }

            return true;
        }else {
            auto left_iter = std::find_if(bev_ele.left_line_points.begin(), bev_ele.left_line_points.end(), 
                                        [&](const EFMPoint& point){ return point.x >= bev_ele.left_line_merge_x; });
            auto right_iter = std::find_if(bev_ele.right_line_points.begin(), bev_ele.right_line_points.end(), 
                                        [&](const EFMPoint& point){ return point.x >= bev_ele.right_line_merge_x; });
            if (right_iter != bev_ele.right_line_points.end() && bev_ele.right_line_merge_index >= 0 && 
                left_iter != bev_ele.left_line_points.end() && bev_ele.left_line_merge_index >= 0){
                right_merge_point = std::distance(bev_ele.right_line_points.begin(), right_iter);
                left_merge_point = std::distance(bev_ele.left_line_points.begin(), left_iter);
            }else{
                right_merge_point = -1;
                left_merge_point = -1; 
                return true;
            }

            // left_merge_point = bev_ele.left_line_merge_index; //获取左车道线的合流点
            // right_merge_point = bev_ele.right_line_merge_index; //获取右车道线的合流点
            //获取左右车道收窄点：从合流点向自车方向遍历，找到宽度从窄变宽的位置
            for (int32_t i = left_merge_point; i > 0; i--){
                PointSLd cur_sl =  PointSLd();
                bool cur_is_inside = false;
                int32_t cur_nearest_index = -1;
                if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(bev_ele.right_line_points, 
                                                                                                     bev_ele.left_line_points[i], 
                                                                                                     cur_sl, cur_is_inside, cur_nearest_index)){
                    LOG_ERROR("CalPointSLBodyCoordinate error");
                    return false;
                }

                PointSLd next_sl =  PointSLd();
                bool next_is_inside = false;
                int32_t next_nearest_index = -1;
                if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(bev_ele.right_line_points, 
                                                                                                     bev_ele.left_line_points[i-1], 
                                                                                                     next_sl, next_is_inside, next_nearest_index)){
                    LOG_ERROR("CalPointSLBodyCoordinate error");
                    return false;
                }

                //比较两次距离，判断是否是收窄点
                //收窄点定义：当前点宽度 <= 阈值，前一点宽度 > 阈值（宽度开始收窄的位置）
                if (100.0 *fabs(cur_sl.l) <= merge_lane_start_width && 100.0 *fabs(next_sl.l) > merge_lane_start_width && 100.0 *fabs(cur_sl.l) > 0.0 &&
                    cur_nearest_index < right_merge_point && cur_nearest_index >= 0){
                    //找到左右车道线的收窄点
                    left_narrow_pos = i; //获取左车道线的收窄点
                    right_narrow_pos = cur_nearest_index; //获取右车道线的收窄点
                    break;
                }
                
                //如果遍历到自车位置附近（x接近0）还没找到收窄点，使用当前位置作为收窄点
                if (bev_ele.left_line_points[i].x <= 5.0 && left_narrow_pos < 0 && 
                    100.0 *fabs(cur_sl.l) > merge_lane_start_width && cur_nearest_index >= 0){
                    left_narrow_pos = i;
                    right_narrow_pos = cur_nearest_index;
                    break;
                }
            }
            //如果左侧未判断成功，用右侧判断
            if (left_narrow_pos < 0 || right_narrow_pos < 0){
                for (int32_t i = right_merge_point; i > 0; i--){
                    PointSLd cur_sl =  PointSLd();
                    bool cur_is_inside = false;
                    int32_t cur_nearest_index = -1;
                    if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(bev_ele.left_line_points, 
                                                                                                         bev_ele.right_line_points[i], 
                                                                                                         cur_sl, cur_is_inside, cur_nearest_index)){
                        LOG_ERROR("CalPointSLBodyCoordinate error");
                        return false;
                    }

                    PointSLd next_sl =  PointSLd();
                    bool next_is_inside = false;
                    int32_t next_nearest_index = -1;
                    if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(bev_ele.left_line_points, 
                                                                                                         bev_ele.right_line_points[i-1], 
                                                                                                         next_sl, next_is_inside, next_nearest_index)){
                        LOG_ERROR("CalPointSLBodyCoordinate error");
                        return false;
                    }

                    //比较两次距离，判断是否是收窄点
                    //收窄点定义：当前点宽度 <= 阈值，前一点宽度 > 阈值
                    if (100.0 *fabs(cur_sl.l) <= merge_lane_start_width && 100.0 *fabs(next_sl.l) > merge_lane_start_width && 100.0 *fabs(cur_sl.l) > 0.0 &&
                        cur_nearest_index < left_merge_point && cur_nearest_index >= 0){
                        //找到左右车道线的收窄点
                        right_narrow_pos = i; //获取右车道线的收窄点
                        left_narrow_pos = cur_nearest_index; //获取左车道线的收窄点
                        break;
                    }
                    
                    //如果遍历到自车位置附近（x接近0）还没找到收窄点，使用当前位置作为收窄点
                    if (bev_ele.right_line_points[i].x <= 5.0 && right_narrow_pos < 0 && 
                        100.0 *fabs(cur_sl.l) > merge_lane_start_width && cur_nearest_index >= 0){
                        right_narrow_pos = i;
                        left_narrow_pos = cur_nearest_index;
                        break;
                    }
                }
                
            }
            //如果左右车道线的收窄点都小于0，说明没有收窄点，直接取第一个点
            if (left_narrow_pos < 0 && right_narrow_pos < 0){
                return true; //如果左右边线没有交叉点，返回true
            }

            {
                //判断方向
                //lane_idx为0时，表示自车所在车道,判断左右是否有车道
                if (lane_idx == 0){
                    //判断左车道存在右车道不存在
                    if (!bev_left_ele_.empty() && bev_right_ele_.empty()){
                        merge_dir = 1; //向左merge
                        return true;
                    }

                    //判断右车道存在左车道不存在
                    if (!bev_right_ele_.empty() && bev_left_ele_.empty()){
                        merge_dir = 2; //向右merge
                        return true;
                    }
                }

                //根据左右车道线的车道线类型信息，获取merge方向
                LineTypeInfo left_line_type  = GetlineTypeForIdx(bev_ele.left_line_points, bev_ele.left_types, left_narrow_pos, left_merge_point);
                LineTypeInfo right_line_type = GetlineTypeForIdx(bev_ele.right_line_points, bev_ele.right_types, right_narrow_pos, right_merge_point);

                //如果左车道线类型为实线，右车道线类型为虚线，则认为是向左merge
                if (left_line_type.line_type == 2 && right_line_type.line_type != 2){
                    merge_dir = 1; //向左merge
                    return true;
                }
                //如果右车道线类型为实线，左车道线类型为虚线，则认为是向右merge
                if (right_line_type.line_type == 2 && left_line_type.line_type != 2){
                    merge_dir = 2; //向右merge
                    return true;
                }

                //如果左车道线类型和右车道线类型都不为虚线，则返回
                if (left_line_type.line_type != 2 && right_line_type.line_type != 2){
                    merge_dir = 0; //不merge
                    return true;
                }
                

                //根据变道方向做成
                if (node_info_.dir == NodeDir::NODE_DIR_LEFT && lane_idx == 0){
                    //向左变道
                    merge_dir = 1; //向左merge
                    return true;
                }
                else if (node_info_.dir == NodeDir::NODE_DIR_RIGHT && lane_idx == 0){
                    //向右变道
                    merge_dir = 2; //向右merge
                    return true;
                }

                //根据山下匝道属性做成
                //遍历path，找到自车对应的path数据
                for (const auto& path : paths_.ehp_output_path_list){
                    if (path.path_id_ == lane_loc_.path_id_ && path.link_offsets_.size() > 0){
                        //遍历SSpecialData
                        for (const auto& special_data : path.special_datas_){
                            if (special_data.type_ == PATH_SPECIAL_TYPE::PATH_SPECIAL_TYPE_TO_RAMP && 
                                special_data.s_offset_ >= lane_loc_.offset_ && special_data.s_offset_ <= lane_loc_.offset_ + SPECIAL_DIS_CAR){
                                merge_dir = 2; //向左merge
                                return true;
                            }
                            else if (special_data.type_ == PATH_SPECIAL_TYPE::PATH_SPECIAL_TYPE_TO_MAIN && 
                                    special_data.s_offset_ >= lane_loc_.offset_ && special_data.s_offset_ <= lane_loc_.offset_ + SPECIAL_DIS_CAR){
                                merge_dir = 1; //向右merge
                                return true;
                            }
                        }
                        break;
                    }
                }

                //都没有做成则优先merge左侧
                merge_dir = 1; //向左merge
            }
        }
        

        // //获取交叉点信息
        // if (false == GetMergePoint(bev_ele, left_merge_point, right_merge_point, 
        //                            left_narrow_pos, right_narrow_pos)){

        // }

        // //如果merge的起点和终点都小于0，说明没有merge，直接返回
        // if (left_merge_point < 0 && right_merge_point < 0 && left_narrow_pos < 0 && right_narrow_pos < 0){
        //     return true; //如果左右边线没有交叉点，返回false
        // }
        
        return true;

    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetMergeDir: " + std::to_string(merge_dir) + " " + e.what());
        return false;
    }
    return true;
}

bool LocalMap::IsVirtualMerge(BevLaneElement& bev_ele, BevLaneElement& side_bev_ele, 
                              int32_t& merge_point, int32_t& narrow_pos, bool is_left){
    //如果narrow_pos与merge_point距离过短，可能存在中心线曲率过大，过长可能存在压边线，一直不居中，后期再改进
    try
    {
        merge_point = -1;
        narrow_pos = -1;
        //如果bev_ele和side_bev_ele的车道线点数都小于2，直接返回false
        if (bev_ele.left_line_points.size() < 2 || bev_ele.right_line_points.size() < 2 ||
            side_bev_ele.left_line_points.size() < 2 || side_bev_ele.right_line_points.size() < 2){
            return false;
        }

        
        if (is_left){
            //如果是左侧车道线，判断自车道的左侧车道线和旁车道的右侧车道线是否一样
            if (!(bev_ele.left_line_points.size() == side_bev_ele.right_line_points.size() && 
                bev_ele.left_line_points.back().x == side_bev_ele.right_line_points.back().x &&
                bev_ele.left_line_points.back().y == side_bev_ele.right_line_points.back().y)){
                return false; //如果车道线点数不一样，直接返回false
            }
            //判断旁车道的左车道线是否大于旁车道的右车道线
            if (side_bev_ele.left_line_points.size() <= side_bev_ele.right_line_points.size()){
                return false;
            }

            //遍历自车道边线点，查找对应距离小于virtual_merge_end_width米的点
            for (int32_t i = bev_ele.right_overlap_left_end_index + 1; i < bev_ele.right_line_points.size(); ++i){
                //遍历旁车道的左侧车道线点，查找对应距离小于virtual_merge_end_width米的点
                PointSLd cur_sl =  PointSLd();
                if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(side_bev_ele.left_line_points, 
                                                                                                     bev_ele.right_line_points[i], 
                                                                                                     cur_sl)){
                    LOG_ERROR("CalPointSLBodyCoordinate error");
                    return false;
                }
                if (100.0 * fabs(cur_sl.l) > 0.0 && 100.0 * fabs(cur_sl.l) <= virtual_merge_end_width){
                    //如果自车道的右侧车道线点和旁车道的左侧车道线点的距离小于virtual_merge_end_width米，则认为是虚拟合流
                    merge_point = i; //设置合流点
                    narrow_pos = bev_ele.right_overlap_left_end_index; //设置收窄点
                    return true; //如果满足条件，返回true
                }
                
            }
        }else{
            //如果是右侧车道线，判断自车道的右侧车道线和旁车道的左侧车道线是否一样
            if (!(bev_ele.right_line_points.size() == side_bev_ele.left_line_points.size() && 
                bev_ele.right_line_points.back().x == side_bev_ele.left_line_points.back().x &&
                bev_ele.right_line_points.back().y == side_bev_ele.left_line_points.back().y)){
                return false; //如果车道线点数不一样，直接返回false
            }

            //判断旁车道的右车道线是否大于旁车道的左车道线
            if (side_bev_ele.right_line_points.size() <= side_bev_ele.left_line_points.size()){
                return false;
            }

            //遍历自车道边线点，查找对应距离小于virtual_merge_end_width米的点
            for (int32_t i = bev_ele.left_overlap_right_end_index + 1; i < bev_ele.left_line_points.size(); ++i){
                //遍历旁车道的右侧车道线点，查找对应距离小于virtual_merge_end_width米的点
                PointSLd cur_sl =  PointSLd();
                if (false == CommonTool::DiscretePointsMath::GetInstance()->CalPointSLBodyCoordinate(side_bev_ele.right_line_points, 
                                                                                                     bev_ele.left_line_points[i], 
                                                                                                     cur_sl)){
                    LOG_ERROR("CalPointSLBodyCoordinate error");
                    return false;
                }
                if (100.0 * fabs(cur_sl.l) > 0.0 && 100.0 * fabs(cur_sl.l) <= virtual_merge_end_width){
                    //如果自车道的左侧车道线点和旁车道的右侧车道线点的距离小于virtual_merge_end_width米，则认为是虚拟合流
                    merge_point = i; //设置合流点
                    narrow_pos = bev_ele.left_overlap_right_end_index; //设置收窄点
                    return true; //如果满足条件，返回true
                }
            }
        }
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::IsVirtualMerge: " + std::string(e.what()));
        return false;
    }
    return false; //如果没有满足以上条件，则认为不是虚拟合流
}

LineTypeInfo LocalMap::GetlineTypeForIdx(const EFMRefLinePoints& line_points, std::vector<LineTypeInfo>& line_type_infos,
                                         int32_t s_pos, int32_t e_pos){
    LineTypeInfo re_line_type_info = {};
    try
    {
        //如果line_points为空，直接返回
        if (line_points.empty()){
            return re_line_type_info;
        }

        //如果s_pos和e_pos都小于0，直接返回
        if (s_pos < 0 && e_pos < 0){
            return re_line_type_info;
        }

        //如果s_pos和e_pos都大于line_points.size()，直接返回
        if (s_pos >= line_points.size() && e_pos >= line_points.size()){
            return re_line_type_info;
        }

        //如果s_pos小于0，设置为0
        if (s_pos < 0){
            s_pos = 0;
        }

        //如果e_pos大于line_points.size()，设置为line_points.size()
        if (e_pos >= line_points.size()){
            e_pos = line_points.size() - 1;
        }

        //如果line_type_infos为空，直接返回
        if (line_type_infos.empty()){
            return re_line_type_info;
        }

        //判断s_pos_length和e_pos_length是否在line_type_infos中
        for (auto& line_type_info : line_type_infos){
            //判断是否有效
            if (line_type_info.is_valid == false){
                continue; //如果无效，跳过
            }

            if (line_type_info.line_type == line_type_info.typ_aft_chg_point){
                if (line_points[s_pos].x >= static_cast<double>(line_type_info.start_point) && 
                    line_points[s_pos].x < static_cast<double>(line_type_info.typ_chg_point)){
                    re_line_type_info.is_valid = true; //设置有效
                    re_line_type_info.line_type = line_type_info.line_type; //设置车道线类型
                    re_line_type_info.start_point = static_cast<float>(line_points[s_pos].x); //设置起点
                    re_line_type_info.typ_chg_point = line_type_info.typ_chg_point; //设置类型变更点
                    re_line_type_info.typ_aft_chg_point = line_type_info.typ_aft_chg_point; //设置类型变更点
                }
                continue;
            }else{
                if (line_points[s_pos].x >= static_cast<double>(line_type_info.start_point) && 
                    line_points[s_pos].x < static_cast<double>(line_type_info.typ_chg_point)){
                    re_line_type_info.is_valid = true; //设置有效
                    re_line_type_info.line_type = line_type_info.line_type; //设置车道线类型
                    re_line_type_info.start_point = static_cast<float>(line_points[s_pos].x); //设置起点
                    if (line_points[e_pos].x >= static_cast<double>(line_type_info.start_point) && 
                        line_points[e_pos].x < static_cast<double>(line_type_info.typ_chg_point) &&
                        line_points[e_pos].x >= line_points[s_pos].x){
                        re_line_type_info.typ_chg_point = MAXFLOAT; //设置类型变更点
                        re_line_type_info.typ_aft_chg_point = line_type_info.line_type; //设置类型变更点
                    }else{
                        re_line_type_info.typ_chg_point = line_type_info.typ_chg_point; //设置类型变更点
                        re_line_type_info.typ_aft_chg_point = line_type_info.typ_aft_chg_point; //设置类型变更点 
                    }
                    break;
                }
            }
        }
        
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetlineTypeForIdx: " + std::string(e.what()));
    }

    return re_line_type_info;
}


bool LocalMap::GetSplitDir(BevLaneElement& bev_ele, uint8_t& split_dir, int32_t& left_split_point, 
                           int32_t& right_split_point, int32_t& left_split_end_pos, int32_t& right_split_end_pos){

    try
    {
        split_dir = 0;
        if (bev_ele.left_line_points.empty() && bev_ele.right_line_points.empty()){
            return false;
        }

        //获取交叉点信息
        if (false == GetSplitPoint(bev_ele.left_line_points, bev_ele.right_line_points, 
                                   left_split_point, right_split_point)){
            return false;
        }
        
        
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetMergeDir: " + std::to_string(split_dir) + " " + e.what());
        return false;
    }

    return true;
}

bool LocalMap::GetMergePoint(BevLaneElement& bev_ele, int32_t& left_merge_point, 
                             int32_t& right_merge_point, int32_t& left_narrow_pos, int32_t& right_narrow_pos){
    try
    {
        left_merge_point = -1;
        right_merge_point = -1;
        left_narrow_pos = -1;
        right_narrow_pos = -1;

        if (bev_ele.left_line_points.empty() && bev_ele.right_line_points.empty()){
            return false;
        }

        //判断left_overlap_right_start_index、left_overlap_right_end_index、right_overlap_left_start_index、right_overlap_left_end_index是否有效
        if (bev_ele.left_overlap_right_start_index < 0 || bev_ele.left_overlap_right_start_index >= bev_ele.left_line_points.size() ||
            bev_ele.left_overlap_right_end_index < 0 || bev_ele.left_overlap_right_end_index >= bev_ele.left_line_points.size() ||
            bev_ele.left_overlap_right_start_index > bev_ele.left_overlap_right_end_index ||
            bev_ele.right_overlap_left_start_index < 0 || bev_ele.right_overlap_left_start_index >= bev_ele.right_line_points.size() ||
            bev_ele.right_overlap_left_end_index < 0 || bev_ele.right_overlap_left_end_index >= bev_ele.right_line_points.size() ||
            bev_ele.right_overlap_left_start_index > bev_ele.right_overlap_left_end_index ||
            (bev_ele.left_overlap_right_end_index - bev_ele.left_overlap_right_start_index) != (bev_ele.right_overlap_left_end_index - bev_ele.right_overlap_left_start_index)){
            return false; 
        }

        //判断左右边线在重叠部分是否存在越来越小的情况，说明是merge
        auto& left_points = bev_ele.left_line_points;
        auto& right_points = bev_ele.right_line_points;
        double e_distance = std::hypot(left_points.back().x - right_points.back().x, 
                                                     left_points.back().y - right_points.back().y);
        //如果e_distance大于merge_lane_start_width，认为不是merge
        if (e_distance*100 > merge_lane_start_width){
            return false; //如果距离大于merge_lane_start_width，则认为不是merge
        }

        double s_distance = std::hypot(left_points.begin()->x - right_points.begin()->x, 
                                                left_points.begin()->y - right_points.begin()->y);
        //如果s_distanced大于e_distance，且s_distance小于merge_lane_end_width，则认为是merge
        if (s_distance*100 > e_distance*100 && s_distance*100 < merge_lane_end_width){
            left_merge_point = bev_ele.left_overlap_right_start_index;
            right_merge_point = bev_ele.right_overlap_left_start_index;
            left_narrow_pos = bev_ele.left_overlap_right_start_index;
            right_narrow_pos = bev_ele.right_overlap_left_start_index;
            return true; //如果满足条件，则认为是merge
        }

        bool is_merge_end = false; //如果左右边线在重叠部分存在越来越小的情况，说明是merge的终点
        //如果左右边线在重叠部分存在越来越小的情况，说明是merge
        for (int32_t left_idx = bev_ele.left_overlap_right_end_index - 1, right_idx = bev_ele.right_overlap_left_end_index - 1;
             left_idx >= bev_ele.left_overlap_right_start_index && right_idx >= bev_ele.right_overlap_left_start_index;
             --left_idx, --right_idx){
            //计算左右边线的距离
            double distance = std::hypot(left_points[left_idx].x - right_points[right_idx].x, 
                                                     left_points[left_idx].y - right_points[right_idx].y);
            if (distance*100 > merge_lane_start_width){ //如果距离小于merge_lane_end_width，则认为是merge的终点
                left_narrow_pos = left_idx + 1; //记录左边线的merge点
                right_narrow_pos = right_idx + 1; //记录右边线的merge点
                break; //如果已经到达终点位置，则跳出循环

            }else if (distance*100 > merge_lane_end_width && false == is_merge_end) { //如果距离小于merge_lane_start_width，则认为是merge的起点
                left_merge_point = left_idx + 1; //记录左边线的merge点
                right_merge_point = right_idx + 1; //记录右边线的merge点
                is_merge_end = true; //标记已经找到merge的终点
            }
        }

        //如果左右边线的merge点都大于等于0，且未找到merge起点，则把初始重叠部分的起始点作为merge点
        if ((left_merge_point >= 0 && right_merge_point >= 0) && 
            (left_narrow_pos < 0 && right_narrow_pos < 0)){
            left_narrow_pos = bev_ele.left_overlap_right_start_index;
            right_narrow_pos = bev_ele.right_overlap_left_start_index;
            return true;
        }

        //如果左右边线的merge点都小于0，且找到merge起点，则重叠部分终点做为merge的终点
        if (left_merge_point < 0 && right_merge_point < 0 && 
            (left_narrow_pos >= 0 && right_narrow_pos >= 0)){
            left_merge_point = bev_ele.left_overlap_right_end_index;
            right_merge_point = bev_ele.right_overlap_left_end_index;
            return true;
        }

        //如果和终点都小于0，且未找到merge起点，则认为不是merge
        if (left_merge_point < 0 && right_merge_point < 0 && 
            (left_narrow_pos < 0 && right_narrow_pos < 0)){
            return false; //如果没有找到merge点，则认为不是merge
        }
    }
    catch(const std::exception& e)
    {
        std::cout << "LocalMap::GetCrossPoint: "  << e.what() << std::endl;
        return false;
    }

    return true;
}

bool LocalMap::MakeRefLine(BevLaneElementGroupSet& bev_lane_group_set, SDLaneElementGroupSet& ele_group){
    try
    {
        if (DEBUG_FLAG == 1)
        {
            //遍历bev_lane_group_set，打印
            for (const auto& pair : bev_lane_group_set) {
                LOG_DEBUG("bev_lane_group_set: " + std::to_string(pair.first) + ", size: " + std::to_string(pair.second.size()));
            }
            //遍历ele_group，打印
            for (const auto& pair : ele_group) {
                LOG_DEBUG("ele_group: " + std::to_string(pair.first) + ", size: " + std::to_string(pair.second.size()));
            }
        }
        
        if (bev_lane_group_set.empty() || ele_group.empty()){
            return false; //如果车道组为空，直接返回false
        }
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
            LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
        }

        //遍历bev、ele_group车道组，找到自车所在车道组
        auto bev_ego_it =  std::find_if(bev_lane_group_set.begin(), bev_lane_group_set.end(), 
            [](auto& pair) { return pair.first == 0 && !pair.second.empty(); }) ;
        auto ele_ego_it =  std::find_if(ele_group.begin(), ele_group.end(), 
            [this](auto& pair) { return !pair.second.empty() && pair.second[0].lane_ids[0] == lane_loc_.lane_id_; }) ;

        if (bev_ego_it == bev_lane_group_set.end() || ele_ego_it == ele_group.end()){
            return false; //如果没有找到自车所在车道组，直接返回false
        }
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
            LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
        }
        ego_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_ego_it); //设置自车所在车道组的索引
        ego_lane_idx_.sd_lane_idx_.first = std::distance(ele_group.begin(), ele_ego_it); //设置自车所在车道组的索引

        auto bev_ego = bev_ego_it->second;
        auto ele_ego = ele_ego_it->second;
        int8_t ele_ego_lane_id = ele_ego_it->first; //获取自车所在车道组的车道ID
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
            LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
            //遍历bev_lane_group_set，打印
            for (const auto& pair : bev_lane_group_set) {
                LOG_DEBUG("bev_lane_group_set: " + std::to_string(pair.first) + ", size: " + std::to_string(pair.second.size()));
            }
            //遍历ele_group，打印
            LOG_DEBUG("ele_ego_lane_id: " + std::to_string(ele_ego_lane_id));
            for (auto& pair : ele_group) {
                LOG_DEBUG("ele_group: " + std::to_string(pair.first) + ", size: " + std::to_string(pair.second.size()));
            }
        }
        //遍历bev、ele_group车道组，找到左车所在车道组
        auto bev_left_it =  std::find_if(bev_lane_group_set.begin(), bev_lane_group_set.end(), 
            [](auto& pair) { return pair.first == 1 && !pair.second.empty(); }) ;
        if (DEBUG_FLAG == 1)
        {
            if (bev_left_it != bev_lane_group_set.end())
            {
                LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
            }
            LOG_DEBUG("ele_ego_lane_id: " + std::to_string(ele_ego_lane_id));
        }

        auto ele_left_it =  std::find_if(ele_group.begin(), ele_group.end(), 
            [&ele_ego_lane_id](auto& pair) { return !pair.second.empty() && pair.first == ele_ego_lane_id + 1; }) ;
        if (DEBUG_FLAG == 1)
        {
            LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
            LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
        }
        //遍历bev、ele_group车道组，找到右车所在车道组
        auto bev_right_it =  std::find_if(bev_lane_group_set.begin(), bev_lane_group_set.end(), 
            [](auto& pair) { return pair.first == 2 && !pair.second.empty(); }) ;
        auto ele_right_it =  std::find_if(ele_group.begin(), ele_group.end(), 
            [&ele_ego_lane_id](auto& pair) { return !pair.second.empty() && pair.first == ele_ego_lane_id - 1; }) ;
        // LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
        // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
        //比较bev_ego和ele_ego的车道组大小，如果不一致，直接做成
        if (bev_ego.size() == 1){
            auto bev_ego_ele = bev_ego[0]; //获取自车所在车道组的车道元素
            ego_lane_idx_.bev_lane_idx_.second = 0; //设置自车所在车道组的索引
            //ele_ego中查找is_group_dest=true的车道元素
            SDLaneElement ele_ego_ele = ele_ego[0]; //获取自车所在车道组的车道元素
            auto it = std::find_if(ele_ego.begin(), ele_ego.end(), 
                [](const SDLaneElement& ele) { return ele.is_group_dest; });
            if (it != ele_ego.end()){
                ele_ego_ele = *it; //如果找到了，获取该车道元素
                ego_lane_idx_.sd_lane_idx_.second = std::distance(ele_ego.begin(), it); //设置自车所在车道组的索引
            }else{
                ego_lane_idx_.sd_lane_idx_.second = 0; //如果没有找到，设置为0
            }
            // LOG_DEBUG("ego_lane_idx_.sd_lane_idx_.first: " + std::to_string(ego_lane_idx_.sd_lane_idx_.first));
            // LOG_DEBUG("ego_lane_idx_.sd_lane_idx_.second: " + std::to_string(ego_lane_idx_.sd_lane_idx_.second));
            // LOG_DEBUG("ego_lane_idx_.bev_lane_idx_.first: " + std::to_string(ego_lane_idx_.bev_lane_idx_.first));
            // LOG_DEBUG("ego_lane_idx_.bev_lane_idx_.second: " + std::to_string(ego_lane_idx_.bev_lane_idx_.second));

            //判断bev_left_it、ele_left_it是否为空，赋值left_lane_idx_
            if (bev_left_it != bev_lane_group_set.end()){
                left_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_left_it); //设置左车道组的索引
                left_lane_idx_.bev_lane_idx_.second = 0; //设置左车道组的索引
            }
            if (ele_left_it != ele_group.end()){
                left_lane_idx_.sd_lane_idx_.first = std::distance(ele_group.begin(), ele_left_it); //设置左车道组的索引
                left_lane_idx_.sd_lane_idx_.second = 0; //设置左车道组的索引
            }

            //判断bev_right_it、ele_right_it是否为空，赋值right_lane_idx_
            if (bev_right_it != bev_lane_group_set.end()){
                right_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_right_it); //设置右车道组的索引
                right_lane_idx_.bev_lane_idx_.second = bev_right_it->second.size() - 1; //设置右车道组的索引
            }
            if (ele_right_it != ele_group.end()){
                right_lane_idx_.sd_lane_idx_.first = std::distance(ele_group.begin(), ele_right_it); //设置左车道组的索引
                right_lane_idx_.sd_lane_idx_.second = ele_right_it->second.size() - 1; //设置右车道组的索引
            }
            // LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
            // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));

            //通过node_info_，做成ref_lane_idx_
            if (node_info_.dir == NodeDir::NODE_DIR_LEFT){
                ref_lane_idx_ = left_lane_idx_;
            }else if (node_info_.dir == NodeDir::NODE_DIR_RIGHT){
                ref_lane_idx_ = right_lane_idx_;
            }else{
                ref_lane_idx_ = ego_lane_idx_; //如果没有方向，默认是自车所在车道组
            }

            // LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
            // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
            //通过merge做成node_info
            if (bev_ego_ele.s_merge_point >= 0 && bev_ego_ele.e_merge_point > 0 && bev_ego_ele.merge_dir != 0){
                //获取s_merge_point、e_merge_point与自车的距离
                double s_merge_offset, e_merge_offset;
                GetPointOffset(bev_ego_ele.center_line_points, bev_ego_ele.s_merge_point, bev_ego_ele.e_merge_point,
                               s_merge_offset, e_merge_offset);
                MakeNodeInfoForMerge(bev_ego_ele.merge_dir, s_merge_offset, e_merge_offset); 
                // LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
                // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
            }else if (ele_ego_ele.first_merge_index >= 0){
                int32_t s_merge_offset = ele_ego_ele.lane_e_offsets[ele_ego_ele.first_merge_index];
                int32_t e_merge_offset = ele_ego_ele.lane_e_offsets[ele_ego_ele.first_merge_index];

                uint8_t merge_dir = 0;
                switch (ele_ego_ele.lane_merges[ele_ego_ele.first_merge_index]){
                    case EFM_MergeType_TO_LEFT:
                    case EFM_MergeType_RIGHT_TO_MIDDLE:
                        merge_dir = 1; //向左merge
                        break;
                    case EFM_MergeType_TO_RIGHT:
                    case EFM_MergeType_LEFT_TO_MIDDLE:
                        merge_dir = 2; //向右merge
                        break;
                    default:
                        merge_dir = 0; //默认不merge
                        break;
                }
                // LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
                // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
                MakeNodeInfoForMerge(bev_ego_ele.merge_dir, s_merge_offset, e_merge_offset); 
            }
            return true; //返回true，表示做成成功
        }else{
            //匹配bev_ego和ele_ego的车道组，明确对应关系
            BevLaneElement bev_ego_ele = bev_ego[0]; //获取自车所在车道组的车道元素
            int32_t bev_ego_ele_idx = 0;
            auto it = std::find_if(bev_ego.begin(), bev_ego.end(), 
                [](BevLaneElement& bev_ego) { return bev_ego.is_main_ele; });
            if (it != bev_ego.end()){
                bev_ego_ele = *it; //如果找到了，获取该车道元素
                bev_ego_ele_idx = std::distance(bev_ego.begin(), it);
            }
            if (DEBUG_FLAG == 1)
            {
                for (size_t i = 0; i < bev_ego.size(); i++)
                {
                    LOG_DEBUG("bev_ego[" + std::to_string(i) + "]: " + std::to_string(bev_ego[i].is_main_ele));
                }
                
                LOG_DEBUG("bev_ego.size: " + std::to_string(bev_ego.size()) + ", bev_ego_ele_idx: " + std::to_string(bev_ego_ele_idx));
            }
            int32_t ele_ego_ele_idx = 0; //自车所在车道组的车道元素索引
            GetEleGroupMainLane(ele_ego, ele_ego_ele_idx, bev_ego, bev_ego_ele_idx);
            if (DEBUG_FLAG == 1)
            {
                LOG_DEBUG("ele_ego.size: " + std::to_string(ele_ego.size()) + ", ele_ego_ele_idx: " + std::to_string(ele_ego_ele_idx));
            }
            auto ele_ego_ele = ele_ego[ele_ego_ele_idx]; //获取自车所在车道组的车道元素
            std::vector<std::pair<int32_t, int32_t>> bev_mapping_eles = {}; //ele和bev的映射关系，数量和ele_ego一致，和ele_ego对应
            int32_t ele_group_dest_idx = -1; 
            int32_t bev_group_dest_idx = -1; 
            GetBevMappingEle(ele_ego, ele_ego_ele_idx, bev_ego, bev_ego_ele_idx, bev_mapping_eles, bev_group_dest_idx, ele_group_dest_idx);
            if (DEBUG_FLAG == 1)
            {
                LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
                LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
            }
            if (ele_group_dest_idx < 0 || bev_group_dest_idx < 0){
                return false; //如果没有找到目的车道组，直接返回false
            }
            if (DEBUG_FLAG == 1)
            {
                LOG_DEBUG("bev_group_dest_idx: " + std::to_string(bev_group_dest_idx) + ", ele_group_dest_idx: " + std::to_string(ele_group_dest_idx));
            }


            //获取s_split_point、e_split_point与自车的距离
            double s_split_offset, e_split_offset;
            GetPointOffset(bev_ego_ele.center_line_points, bev_ego_ele.s_split_point, bev_ego_ele.e_split_point, 
                           s_split_offset, e_split_offset);

            //todo 临时方案，e_split_point的计算逻辑需要看一下,理论上不需要下面这行。
            e_split_offset = fmax(bev_ego_ele.left_line_points.front().x, bev_ego_ele.right_line_points.front().x);
            if (DEBUG_FLAG == 1)
            {
                LOG_DEBUG("bev_ego_ele.s_split_point: " + std::to_string(bev_ego_ele.s_split_point) + ", bev_ego_ele.e_split_point: " + std::to_string(bev_ego_ele.e_split_point));
                LOG_DEBUG("s_split_offset: " + std::to_string(s_split_offset) + ", e_split_offset: " + std::to_string(e_split_offset));
            }

            // LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
            // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
            if (s_split_offset*100.0 >= car_offset_split && e_split_offset*100.0 >= car_offset_split ){
                //如果s_split_offset和e_split_offset都大于等于car_offset_split，说明自车在分道线前面
                //判断bev_left_it、ele_left_it是否为空，赋值left_lane_idx_
                if (bev_left_it != bev_lane_group_set.end()){
                    left_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_left_it); //设置左车道组的索引
                    left_lane_idx_.bev_lane_idx_.second = 0; //设置左车道组的索引
                }
                if (ele_left_it != ele_group.end()){
                    left_lane_idx_.sd_lane_idx_.first = std::distance(ele_group.begin(), ele_left_it); //设置左车道组的索引
                    left_lane_idx_.sd_lane_idx_.second = 0; //设置左车道组的索引
                }

                //判断bev_right_it、ele_right_it是否为空，赋值right_lane_idx_
                if (bev_right_it != bev_lane_group_set.end()){
                    right_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_right_it); //设置右车道组的索引
                    right_lane_idx_.bev_lane_idx_.second = bev_right_it->second.size() - 1; //设置右车道组的索引
                }
                if (ele_right_it != ele_group.end()){
                    right_lane_idx_.sd_lane_idx_.first = std::distance(ele_group.begin(), ele_right_it); //设置左车道组的索引
                    right_lane_idx_.sd_lane_idx_.second = ele_right_it->second.size() - 1; //设置右车道组的索引
                }

                ego_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx; //设置自车所在车道组的索引
                ego_lane_idx_.sd_lane_idx_.second = ele_ego_ele_idx; //设置自车所在车道组的索引

                //通过node_info_，做成ref_lane_idx_
                if (node_info_.dir == NodeDir::NODE_DIR_LEFT){
                    ref_lane_idx_ = left_lane_idx_;
                }else if (node_info_.dir == NodeDir::NODE_DIR_RIGHT){
                    ref_lane_idx_ = right_lane_idx_;
                }else{
                    ref_lane_idx_ = ego_lane_idx_; //如果没有方向，默认是自车所在车道组
                }

                // LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
                // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
                MakeNodeInfoForSplit(ele_ego, ele_ego_ele_idx, ele_group_dest_idx, s_split_offset, e_split_offset); //做成node_info_

            }else if (e_split_offset > 0 ){
                ego_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx; //设置自车所在车道组的索引
                ego_lane_idx_.sd_lane_idx_.second = ele_ego_ele_idx; //设置自车所在车道组的索引
                if (DEBUG_FLAG == 1)
                {
                    LOG_DEBUG("bev_ego_ele_idx: " + std::to_string(bev_ego_ele_idx) + ", ele_ego_ele_idx: " + std::to_string(ele_ego_ele_idx));
                }

                if (ele_ego_ele_idx == ele_group_dest_idx){
                    bool is_left = false;
                    int32_t side_lane_idx = -1;
                    GetSideLane(ele_ego_ele_idx, bev_mapping_eles, side_lane_idx, is_left); //获取旁车道的索引和方向
                    if (DEBUG_FLAG == 1)
                    {
                        LOG_DEBUG("bev_mapping_eles.size: " + std::to_string(bev_mapping_eles.size()) + ", side_lane_idx: " + std::to_string(side_lane_idx) + ", is_left: " + std::to_string(is_left));
                        for (size_t i = 0; i < bev_mapping_eles.size(); i++)
                        {
                            LOG_DEBUG("bev_mapping_eles[" + std::to_string(i) + "]: " + std::to_string(bev_mapping_eles[i].first) + ", " + std::to_string(bev_mapping_eles[i].second));
                        }
                    }
                    

                    if (side_lane_idx == -1){
                        // if (bev_left_it != bev_lane_group_set.end()){
                        //     left_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_left_it); //设置左车道组的索引
                        //     left_lane_idx_.bev_lane_idx_.second = 0; //设置左车道组的索引
                        // }else{
                        //     if (bev_ego_ele_idx < bev_ego.size() - 1){
                        //         left_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first; //设置左车道组的索引
                        //         left_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx + 1; //设置左车道组的索引
                        //     }
                        // }

                        if (bev_ego_ele_idx < bev_ego.size() - 1){
                                left_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first; //设置左车道组的索引
                                left_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx + 1; //设置左车道组的索引
                        }else{
                            if (bev_left_it != bev_lane_group_set.end()){
                                left_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_left_it); //设置左车道组的索引
                                left_lane_idx_.bev_lane_idx_.second = 0; //设置左车道组的索引
                            }
                        }

                        if (ele_left_it != ele_group.end()){
                            left_lane_idx_.sd_lane_idx_.first = std::distance(ele_group.begin(), ele_left_it); //设置左车道组的索引
                            left_lane_idx_.sd_lane_idx_.second = 0; //设置左车道组的索引
                        }

                        //判断bev_right_it、ele_right_it是否为空，赋值right_lane_idx_
                        // if (bev_right_it != bev_lane_group_set.end()){
                        //     right_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_right_it); //设置右车道组的索引
                        //     right_lane_idx_.bev_lane_idx_.second = bev_right_it->second.size() - 1; //设置右车道组的索引
                        // }else{
                        //     if (bev_ego_ele_idx > 0){
                        //         right_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first; //设置右车道组的索引
                        //         right_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx - 1; //设置右车道组的索引
                        //     }
                        // }

                        if (bev_ego_ele_idx > 0){
                                right_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first; //设置右车道组的索引
                                right_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx - 1; //设置右车道组的索引
                        }else{
                            if (bev_right_it != bev_lane_group_set.end()){
                                right_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_right_it); //设置右车道组的索引
                                right_lane_idx_.bev_lane_idx_.second = bev_right_it->second.size() - 1; //设置右车道组的索引
                            }
                        }


                        if (ele_right_it != ele_group.end()){
                            right_lane_idx_.sd_lane_idx_.first = std::distance(ele_group.begin(), ele_right_it); //设置左车道组的索引
                            right_lane_idx_.sd_lane_idx_.second = ele_right_it->second.size() - 1; //设置右车道组的索引
                        }
                        if (DEBUG_FLAG == 1)
                        {
                            LOG_DEBUG("left_lane_idx_.sd_lane_idx_.first: " + std::to_string(left_lane_idx_.sd_lane_idx_.first));
                            LOG_DEBUG("left_lane_idx_.sd_lane_idx_.second: " + std::to_string(left_lane_idx_.sd_lane_idx_.second));
                            LOG_DEBUG("right_lane_idx_.sd_lane_idx_.first: " + std::to_string(right_lane_idx_.sd_lane_idx_.first));
                            LOG_DEBUG("right_lane_idx_.sd_lane_idx_.second: " + std::to_string(right_lane_idx_.sd_lane_idx_.second));
                            LOG_DEBUG("left_lane_idx_.bev_lane_idx_.first: " + std::to_string(left_lane_idx_.bev_lane_idx_.first));
                            LOG_DEBUG("left_lane_idx_.bev_lane_idx_.second: " + std::to_string(left_lane_idx_.bev_lane_idx_.second));
                            LOG_DEBUG("right_lane_idx_.bev_lane_idx_.first: " + std::to_string(right_lane_idx_.bev_lane_idx_.first));
                            LOG_DEBUG("right_lane_idx_.bev_lane_idx_.second: " + std::to_string(right_lane_idx_.bev_lane_idx_.second));
                        }
                    }else if (is_left){
                        left_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first; //设置左车道组的索引
                        left_lane_idx_.bev_lane_idx_.second = bev_mapping_eles[side_lane_idx].first; //设置左车道组的索引
                        left_lane_idx_.sd_lane_idx_.first = ego_lane_idx_.sd_lane_idx_.first; //设置左车道组的索引
                        left_lane_idx_.sd_lane_idx_.second = bev_mapping_eles[side_lane_idx].second; //设置左车道组的索引

                        //判断bev_right_it、ele_right_it是否为空，赋值right_lane_idx_
                        if (bev_right_it != bev_lane_group_set.end()){
                            right_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_right_it); //设置右车道组的索引
                            right_lane_idx_.bev_lane_idx_.second = bev_right_it->second.size() - 1; //设置右车道组的索引
                        }else{
                            if (bev_ego_ele_idx > 0){
                                right_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first; //设置右车道组的索引
                                right_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx - 1; //设置右车道组的索引
                            }
                        }
                        if (ele_right_it != ele_group.end()){
                            right_lane_idx_.sd_lane_idx_.first = std::distance(ele_group.begin(), ele_right_it); //设置左车道组的索引
                            right_lane_idx_.sd_lane_idx_.second = ele_right_it->second.size() - 1; //设置右车道组的索引
                        }
                        if (DEBUG_FLAG == 1)
                        {
                            LOG_DEBUG("left_lane_idx_.sd_lane_idx_.first: " + std::to_string(left_lane_idx_.sd_lane_idx_.first));
                            LOG_DEBUG("left_lane_idx_.sd_lane_idx_.second: " + std::to_string(left_lane_idx_.sd_lane_idx_.second));
                            LOG_DEBUG("right_lane_idx_.sd_lane_idx_.first: " + std::to_string(right_lane_idx_.sd_lane_idx_.first));
                            LOG_DEBUG("right_lane_idx_.sd_lane_idx_.second: " + std::to_string(right_lane_idx_.sd_lane_idx_.second));
                            LOG_DEBUG("left_lane_idx_.bev_lane_idx_.first: " + std::to_string(left_lane_idx_.bev_lane_idx_.first));
                            LOG_DEBUG("left_lane_idx_.bev_lane_idx_.second: " + std::to_string(left_lane_idx_.bev_lane_idx_.second));
                            LOG_DEBUG("right_lane_idx_.bev_lane_idx_.first: " + std::to_string(right_lane_idx_.bev_lane_idx_.first));
                            LOG_DEBUG("right_lane_idx_.bev_lane_idx_.second: " + std::to_string(right_lane_idx_.bev_lane_idx_.second));
                        }
                    }else{
                        if (bev_left_it != bev_lane_group_set.end()){
                            left_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_left_it); //设置左车道组的索引
                            left_lane_idx_.bev_lane_idx_.second = 0; //设置左车道组的索引
                        }else{
                            if (bev_ego_ele_idx < bev_ego.size() - 1){
                                left_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first; //设置左车道组的索引
                                left_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx + 1; //设置左车道组的索引
                            }
                        }
                        if (ele_left_it != ele_group.end()){
                            left_lane_idx_.sd_lane_idx_.first = std::distance(ele_group.begin(), ele_left_it); //设置左车道组的索引
                            left_lane_idx_.sd_lane_idx_.second = 0; //设置左车道组的索引
                        }

                        right_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first;  //设置右车道组的索引
                        right_lane_idx_.bev_lane_idx_.second = bev_mapping_eles[side_lane_idx].first; //设置右车道组的索引
                        right_lane_idx_.sd_lane_idx_.first = ego_lane_idx_.sd_lane_idx_.first; //设置左车道组的索引
                        right_lane_idx_.sd_lane_idx_.second = bev_mapping_eles[side_lane_idx].second; //设置右车道组的索引
                        if (DEBUG_FLAG == 1)
                        {
                            LOG_DEBUG("left_lane_idx_.sd_lane_idx_.first: " + std::to_string(left_lane_idx_.sd_lane_idx_.first));
                            LOG_DEBUG("left_lane_idx_.sd_lane_idx_.second: " + std::to_string(left_lane_idx_.sd_lane_idx_.second));
                            LOG_DEBUG("right_lane_idx_.sd_lane_idx_.first: " + std::to_string(right_lane_idx_.sd_lane_idx_.first));
                            LOG_DEBUG("right_lane_idx_.sd_lane_idx_.second: " + std::to_string(right_lane_idx_.sd_lane_idx_.second));
                            LOG_DEBUG("left_lane_idx_.bev_lane_idx_.first: " + std::to_string(left_lane_idx_.bev_lane_idx_.first));
                            LOG_DEBUG("left_lane_idx_.bev_lane_idx_.second: " + std::to_string(left_lane_idx_.bev_lane_idx_.second));
                            LOG_DEBUG("right_lane_idx_.bev_lane_idx_.first: " + std::to_string(right_lane_idx_.bev_lane_idx_.first));
                            LOG_DEBUG("right_lane_idx_.bev_lane_idx_.second: " + std::to_string(right_lane_idx_.bev_lane_idx_.second));
                        }
                    }
                }else if (ele_ego_ele_idx > ele_group_dest_idx){
                    if (DEBUG_FLAG == 1)
                    {
                        LOG_DEBUG("ele_ego_ele_idx > ele_group_dest_idx");
                    }
                    if (bev_left_it != bev_lane_group_set.end()){
                        left_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_left_it); //设置左车道组的索引
                        left_lane_idx_.bev_lane_idx_.second = 0; //设置左车道组的索引
                    }else{
                        if (bev_ego_ele_idx < bev_ego.size() - 1){
                            left_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first; //设置左车道组的索引
                            left_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx + 1; //设置左车道组的索引
                        }
                    }
                    if (ele_left_it != ele_group.end()){
                        left_lane_idx_.sd_lane_idx_.first = std::distance(ele_group.begin(), ele_left_it); //设置左车道组的索引
                        left_lane_idx_.sd_lane_idx_.second = 0; //设置左车道组的索引
                    }

                    right_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first;  //设置右车道组的索引
                    if (bev_ego_ele_idx > 0){
                        right_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first;  //设置右车道组的索引
                        right_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx - 1; //设置右车道组的索引
                    }else{
                        if (bev_right_it != bev_lane_group_set.end()){
                            right_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_right_it); //设置右车道组的索引
                            right_lane_idx_.bev_lane_idx_.second = bev_right_it->second.size() - 1; //设置右车道组的索引
                        }
                    }
                    
                    right_lane_idx_.sd_lane_idx_.first = ego_lane_idx_.sd_lane_idx_.first; //设置左车道组的索引
                    right_lane_idx_.sd_lane_idx_.second = ele_ego_ele_idx - 1; //设置右车道组的索引
                    if (DEBUG_FLAG == 1)
                    {
                        LOG_DEBUG("right_lane_idx_.sd_lane_idx_.first: " + std::to_string(right_lane_idx_.sd_lane_idx_.first));
                        LOG_DEBUG("right_lane_idx_.sd_lane_idx_.second: " + std::to_string(right_lane_idx_.sd_lane_idx_.second)); 
                    } 
                }else{
                    if (bev_ego_ele_idx < bev_ego.size() - 1){
                        left_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first; //设置左车道组的索引
                        left_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx + 1; //设置左车道组的索引
                    }else{
                        if (bev_left_it != bev_lane_group_set.end()){
                            left_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_left_it); //设置左车道组的索引
                            left_lane_idx_.bev_lane_idx_.second = 0; //设置左车道组的索引
                        }
                    }
                    left_lane_idx_.sd_lane_idx_.first = ego_lane_idx_.sd_lane_idx_.first; //设置左车道组的索引
                    left_lane_idx_.sd_lane_idx_.second = ele_ego_ele_idx + 1; //设置左车道组的索引

                    //判断bev_right_it、ele_right_it是否为空，赋值right_lane_idx_
                    if (bev_right_it != bev_lane_group_set.end()){
                        right_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_right_it); //设置右车道组的索引
                        right_lane_idx_.bev_lane_idx_.second = bev_right_it->second.size() - 1; //设置右车道组的索引
                    }else{
                        if (bev_ego_ele_idx > 0){
                            right_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first; //设置右车道组的索引
                            right_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx - 1; //设置右车道组的索引
                        }
                    }
                    if (ele_right_it != ele_group.end()){
                        right_lane_idx_.sd_lane_idx_.first = std::distance(ele_group.begin(), ele_right_it); //设置左车道组的索引
                        right_lane_idx_.sd_lane_idx_.second = ele_right_it->second.size() - 1; //设置右车道组的索引
                    }
                    if (DEBUG_FLAG == 1)
                    {
                        LOG_DEBUG("right_lane_idx_.sd_lane_idx_.first: " + std::to_string(right_lane_idx_.sd_lane_idx_.first));
                        LOG_DEBUG("right_lane_idx_.sd_lane_idx_.second: " + std::to_string(right_lane_idx_.sd_lane_idx_.second));
                    }
                }
                // LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
                // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));
                //通过node_info_，做成ref_lane_idx_
                if (node_info_.dir == NodeDir::NODE_DIR_LEFT){
                    ref_lane_idx_ = left_lane_idx_;
                }else if (node_info_.dir == NodeDir::NODE_DIR_RIGHT){
                    ref_lane_idx_ = right_lane_idx_;
                }else{
                    ref_lane_idx_ = ego_lane_idx_; //如果没有方向，默认是自车所在车道组
                }
                MakeNodeInfoForSplit(ele_ego, ele_ego_ele_idx, ele_group_dest_idx, s_split_offset, e_split_offset); //做成node_info_
            }else{
                if (DEBUG_FLAG == 1)
                {
                    LOG_DEBUG("s_split_offset: " + std::to_string(s_split_offset) + ", e_split_offset: " + std::to_string(e_split_offset));
                }
                //如果s_split_offset和e_split_offset都大于等于car_offset_split，说明自车在分道线前面
                //判断bev_left_it、ele_left_it是否为空，赋值left_lane_idx_
                if (bev_left_it != bev_lane_group_set.end()){
                    left_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_left_it); //设置左车道组的索引
                    left_lane_idx_.bev_lane_idx_.second = 0; //设置左车道组的索引
                }else{
                    if (bev_ego_ele_idx < bev_ego.size() - 1){
                        left_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first; //设置左车道组的索引
                        left_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx + 1; //设置左车道组的索引
                    }
                }
                if (ele_left_it != ele_group.end()){
                    left_lane_idx_.sd_lane_idx_.first = std::distance(ele_group.begin(), ele_left_it); //设置左车道组的索引
                    left_lane_idx_.sd_lane_idx_.second = 0; //设置左车道组的索引
                }

                //判断bev_right_it、ele_right_it是否为空，赋值right_lane_idx_
                if (bev_right_it != bev_lane_group_set.end()){
                    right_lane_idx_.bev_lane_idx_.first = std::distance(bev_lane_group_set.begin(), bev_right_it); //设置右车道组的索引
                    right_lane_idx_.bev_lane_idx_.second = bev_right_it->second.size() - 1; //设置右车道组的索引
                }else{
                    if (bev_ego_ele_idx > 0){
                        right_lane_idx_.bev_lane_idx_.first = ego_lane_idx_.bev_lane_idx_.first; //设置右车道组的索引
                        right_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx - 1; //设置右车道组的索引
                    }
                }
                if (ele_right_it != ele_group.end()){
                    right_lane_idx_.sd_lane_idx_.first = std::distance(ele_group.begin(), ele_right_it); //设置左车道组的索引
                    right_lane_idx_.sd_lane_idx_.second = ele_right_it->second.size() - 1; //设置右车道组的索引
                }
                // LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
                // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));

                ego_lane_idx_.bev_lane_idx_.second = bev_ego_ele_idx; //设置自车所在车道组的索引
                ego_lane_idx_.sd_lane_idx_.second = ele_ego_ele_idx; //设置自车所在车道组的索引
                if (DEBUG_FLAG == 1)
                {
                    LOG_DEBUG("ego_lane_idx_.bev_lane_idx_.second: " + std::to_string(ego_lane_idx_.bev_lane_idx_.second));
                    LOG_DEBUG("ego_lane_idx_.sd_lane_idx_.second: " + std::to_string(ego_lane_idx_.sd_lane_idx_.second));
                }

                //通过node_info_，做成ref_lane_idx_
                if (node_info_.dir == NodeDir::NODE_DIR_LEFT){
                    ref_lane_idx_ = left_lane_idx_;
                }else if (node_info_.dir == NodeDir::NODE_DIR_RIGHT){
                    ref_lane_idx_ = right_lane_idx_;
                }else{
                    ref_lane_idx_ = ego_lane_idx_; //如果没有方向，默认是自车所在车道组
                } 
            }
        }


    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::MakeRefLine: "  +  std::string(e.what()));
        return false;
    }
    // LOG_DEBUG("bev_lane_group_set.size: " + std::to_string(bev_lane_group_set.size()));
    // LOG_DEBUG("ele_group.size: " + std::to_string(ele_group.size()));

    return true;
}

bool LocalMap::GetEleGroupMainLane(const SDLaneElementGroup& ele_group, int32_t& main_lane_idx, 
                                   const BevLaneElementGroup& bev_group, int32_t bev_main_lane_idx){
    try
    {
        main_lane_idx = -1;
        if (ele_group.empty() || bev_group.empty() || bev_main_lane_idx < 0 || bev_main_lane_idx >= bev_group.size()){
            LOG_ERROR("ele_group.size: " + std::to_string(ele_group.size()) + ", bev_group.size: " + std::to_string(bev_group.size())
                      + ", bev_main_lane_idx: " + std::to_string(bev_main_lane_idx));

            return false; //如果车道组为空，直接返回false
        }

        //如果ele_group的size等于1，直接使用
        if (ele_group.size() == 1){
            main_lane_idx = 0; //设置主车道索引为0
            //LOG_DEBUG("main_lane_idx: " + std::to_string(main_lane_idx));
            return true; //返回true，表示找到主车道
        }else if (ele_group.size() == bev_group.size()){
            //LOG_DEBUG("bev_main_lane_idx: " + std::to_string(bev_main_lane_idx));
            main_lane_idx = bev_main_lane_idx; //如果ele_group和bev_group的size相等，直接使用bev_main_lane_idx
            return true; //返回true，表示找到主车道
        }else{
            //获取最小的first_split_index对应的idx
            int32_t min_first_split_index = INT32_MAX; //初始化最小的min_first_split_index为INT32_MAX
            int32_t min_first_split_lane_number = 0;
            int32_t max_first_split_index = INT32_MIN; //初始化最小的max_first_split_index为INT32_MIN 
            std::vector<int32_t> max_first_splits;
            for (int32_t i = 0; i < ele_group.size(); ++i){
                if (ele_group[i].first_split_index > 0 && ele_group[i].first_split_index < min_first_split_index){
                    min_first_split_index = ele_group[i].first_split_index; //更新最小的first_split_index
                    min_first_split_lane_number = ele_group[i].lane_nums[min_first_split_index];
                }
                

                if (ele_group[i].first_split_index == -1){
                    if (max_first_split_index > -1){
                        max_first_splits.clear(); //如果当前的first_split_index小于最大值，清空最大值对应的idx
                        max_first_splits.push_back(i); //添加当前索引
                        max_first_split_index = -1; //更新最大值为-1
                    }else{
                        max_first_split_index = -1; //更新最大值为-1
                        max_first_splits.push_back(i); //添加当前索引
                    }
                    
                }else if (max_first_split_index == -1){
                    continue;
                }else if (ele_group[i].first_split_index == max_first_split_index){
                    max_first_splits.push_back(i); //添加当前索引
                }else if (ele_group[i].first_split_index > max_first_split_index){
                    max_first_splits.clear(); //如果当前的first_split_index大于最大值，清空最大值对应的idx
                    max_first_splits.push_back(i); //添加当前索引
                    max_first_split_index = ele_group[i].first_split_index; //更新最大值为当前的first_split_index
                }
                
            }

            if (max_first_splits.size() == 1){
                main_lane_idx = max_first_splits[0]; //如果只有一个最小的first_split_index，直接使用
                //LOG_DEBUG("main_lane_idx: " + std::to_string(main_lane_idx));
                return true; //返回true，表示找到主车道
            }else{
                if (min_first_split_index == INT32_MAX){
                    if ((bev_main_lane_idx + 1) *2 <= bev_group.size()){
                        int32_t tmp_idx = max_first_splits.size() < 3 ? 0 : (max_first_splits.size() / 2) - 1; //如果max_first_splits.size()小于3，取1，否则取max_first_splits.size() / 2
                        main_lane_idx = max_first_splits[tmp_idx]; 
                    }else{
                        int32_t tmp_idx = max_first_splits.size() < 3 ? 1 : (max_first_splits.size() / 2) + 1; //如果max_first_splits.size()小于2，取0，否则取max_first_splits.size() / 2
                        main_lane_idx = max_first_splits[tmp_idx]; 
                    }
                }else{
                    if (ele_group[max_first_splits[0]].lane_nums.size() <= min_first_split_index){
                        main_lane_idx = max_first_splits[0]; //如果第一个最大值的first_split_index等于最小值，直接使用
                    }else{
                        if (ele_group[max_first_splits[0]].lane_nums[min_first_split_index] > min_first_split_lane_number){
                            main_lane_idx = max_first_splits[0]; //如果第一个最大值的first_split_index等于最小值，直接使用
                        }else{
                            main_lane_idx = max_first_splits.back(); //如果第一个最大值的first_split_index不等于最小值，使用最后一个最大值
                        }
                    }
                }
            }   
        }

        // if (main_lane_idx == -1){
        //     LOG_DEBUG("bev_main_lane_idx: " + std::to_string(main_lane_idx));
        //     LOG_DEBUG("bev_group.size: " + std::to_string(bev_group.size()) + ", bev_main_lane_idx: " + std::to_string(main_lane_idx));
        //     //遍历ele_group，打印
        //     for (int i = 0; i < ele_group.size(); ++i) {
        //         auto& ele = ele_group[i];
        //         LOG_DEBUG("i:" + std::to_string(i) +
        //                   ", ele.lane_id:" + std::to_string(ele.lane_id) + ", ele.ele_id:" + std::to_string(ele.ele_id) + 
        //                   ", ele.link_path_ids:" + std::to_string(ele.link_path_ids[0]) + 
        //                   ", ele.lane_in_link:" + std::to_string(ele.lane_in_link[0]) + 
        //                   ", ele.lane_ids:" + std::to_string(ele.lane_ids[0]) + 
        //                   ", ele.lane_nums:" + std::to_string(ele.lane_nums[0]) +
        //                   ", ele.is_group_dest:" + std::to_string(ele.is_group_dest) +
        //                   ", ele.is_end:" + std::to_string(ele.is_end) +
        //                   ", ele.first_merge_index:" + std::to_string(ele.first_merge_index) +
        //                   ", ele.first_split_index:" + std::to_string(ele.first_split_index) +
        //                   ", ele.merge_count:" + std::to_string(ele.merge_count) +
        //                   ", ele.step_to_dest:" + std::to_string(ele.step_to_dest));
        //     }

        //     //遍历bev_group，打印
        //     for (int i = 0; i < bev_group.size(); ++i) {
        //         auto& bev = bev_group[i];
        //         LOG_DEBUG("i:" + std::to_string(i) +
        //                   ", bev.left_line_points.size:" + std::to_string(bev.left_line_points.size()) +
        //                   ", bev.right_line_points.size:" + std::to_string(bev.right_line_points.size()) +
        //                   ", bev.is_main_merge:" + std::to_string(bev.is_main_merge));
        //     }

        // }

        

    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetEleGroupMainLane: "  + std::string(e.what()));
        return false;
    }

    return true;
}

bool LocalMap::GetBevMappingEle(SDLaneElementGroup& ele_group, int32_t ele_main_lane_idx, 
                                const BevLaneElementGroup& bev_group, int32_t bev_main_lane_idx, 
                                std::vector<std::pair<int32_t, int32_t>>& bev_mapping_eles,
                                int32_t& bev_group_dest_idx, int32_t& ele_group_dest_idx){
    try
    {
        bev_mapping_eles.clear();
        ele_group_dest_idx = ele_main_lane_idx;
        bev_group_dest_idx = bev_main_lane_idx;
        if (ele_group.empty() || bev_group.empty()){
            return false; //如果车道组为空，直接返回false
        }

        //如果ele_group的size和bev_group的size相等，且ele_main_lane_idx=bev_main_lane_idx，直接使用
        if (ele_group.size() == bev_group.size() && ele_main_lane_idx == bev_main_lane_idx){
            for (size_t i = 0; i < ele_group.size(); i++){
                std::pair<int32_t, int32_t> bev_mapping_ele = {};
                bev_mapping_ele.first = i; //设置bev的索引
                bev_mapping_ele.second = i; //设置ele的索引
                bev_mapping_eles.push_back(bev_mapping_ele); //添加到映射关系中
                if (ele_group[i].is_group_dest){
                    ele_group_dest_idx = i; //如果是group_dest，设置ele_group_dest_idx
                    bev_group_dest_idx = i; //如果是group_dest，设置bev_group_dest_idx
                } 
            }
            
            return true; //返回true，表示找到映射关系
        }
        
        //查找ele_group中is_group_dest=true的车道元素
        auto it = std::find_if(ele_group.begin(), ele_group.end(), 
            [](const SDLaneElement& ele) { return ele.is_group_dest; });
        if (it != ele_group.end()){
            ele_group_dest_idx = std::distance(ele_group.begin(), it); //如果找到了，获取该车道元素的索引
        }else{
            ele_group[ele_group_dest_idx].is_group_dest = true; //如果没有找到，设置当前车道元素为group_dest
        }

        std::pair<int32_t, int32_t> bev_mapping_ele = {};
        bev_mapping_ele.first = bev_main_lane_idx;
        bev_mapping_ele.second = ele_main_lane_idx; //设置bev和ele的映射关系
        bev_mapping_eles.push_back(bev_mapping_ele); //添加到映射关系中

        //group_dest_idx加入到映射关系中
        if (ele_group_dest_idx < ele_main_lane_idx){
            if (bev_main_lane_idx == 0){
                ele_group[ele_group_dest_idx].is_group_dest = false; //如果没有找到，设置当前车道元素为group_dest
                ele_group_dest_idx = ele_main_lane_idx;
                ele_group[ele_main_lane_idx].is_group_dest = true; //如果没有找到，设置当前车道元素为group_dest
            }else if (ele_main_lane_idx - ele_group_dest_idx > bev_main_lane_idx){
                bev_group_dest_idx = 0;
                bev_mapping_ele.first = 0;
                bev_mapping_ele.second = ele_group_dest_idx;
                bev_mapping_eles.push_back(bev_mapping_ele); //添加到映射关系中
            }else{
                bev_group_dest_idx = bev_main_lane_idx - (ele_main_lane_idx - ele_group_dest_idx); //计算bev的索引
                bev_mapping_ele.first = bev_group_dest_idx; //设置bev的索引
                bev_mapping_ele.second = ele_group_dest_idx; //设置ele的索引
                bev_mapping_eles.push_back(bev_mapping_ele); //添加到映射关系中
            }
        }else if (ele_group_dest_idx > ele_main_lane_idx){
            if (bev_main_lane_idx == bev_group.size() - 1){
                ele_group[ele_group_dest_idx].is_group_dest = false;
                ele_group_dest_idx = ele_main_lane_idx;
                ele_group[ele_group_dest_idx].is_group_dest = true; //如果没有找到，设置当前车道元素为group_dest
            }else if (ele_group_dest_idx - ele_main_lane_idx > bev_group.size() - 1 - bev_main_lane_idx){
                bev_group_dest_idx = bev_group.size() - 1; //设置bev的索引
                bev_mapping_ele.first = bev_group_dest_idx;
                bev_mapping_ele.second = ele_group_dest_idx;
                bev_mapping_eles.push_back(bev_mapping_ele); //添加到映射关系中
            }else{
                bev_group_dest_idx = bev_main_lane_idx + (ele_group_dest_idx - ele_main_lane_idx); //计算bev的索引
                bev_mapping_ele.first = bev_group_dest_idx; //设置bev的索引
                bev_mapping_ele.second = ele_group_dest_idx; //设置ele的索引
                bev_mapping_eles.push_back(bev_mapping_ele); //添加到映射关系中
            }
        }

        //遍历ele_group\bev_group，找到ele_main_lane_idx\bev_main_lane_idx之前的对应关系
        for (int32_t ele_idx = ele_main_lane_idx - 1, bev_idx = bev_main_lane_idx -1 ; 
             ele_idx >= 0 && bev_idx >= 0; --ele_idx, --bev_idx){
            if (ele_idx == ele_group_dest_idx || bev_idx == bev_group_dest_idx){
                continue; 
            }
            bev_mapping_ele.first = bev_idx; //设置bev的索引
            bev_mapping_ele.second = ele_idx; //设置ele的索引
            bev_mapping_eles.push_back(bev_mapping_ele); //添加到映射关系中
        }

        //遍历ele_group\bev_group，找到ele_main_lane_idx\bev_main_lane_idx之后的对应关系
        for (int32_t ele_idx = ele_main_lane_idx + 1, bev_idx = bev_main_lane_idx + 1; 
             ele_idx < ele_group.size() && bev_idx < bev_group.size(); ++ele_idx, ++bev_idx){
            if (ele_idx == ele_group_dest_idx || bev_idx == bev_group_dest_idx){
                continue; 
            }
            bev_mapping_ele.first = bev_idx; //设置bev的索引
            bev_mapping_ele.second = ele_idx; //设置ele的索引
            bev_mapping_eles.push_back(bev_mapping_ele); //添加到映射关系中
        }

    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetBevMappingEle: "  + std::string(e.what()));
        return false;
    }

    return true;
}

bool LocalMap::GetSideLane(int32_t ele_ego_ele_idx, const std::vector<std::pair<int32_t, int32_t>>& bev_mapping_eles, 
                           int32_t& side_lane_idx, bool& is_left){
    try
    {
        side_lane_idx = -1;
        is_left = false;
        if (bev_mapping_eles.empty()){
            return false; //如果映射关系为空，直接返回false
        }

        //遍历映射关系，找到ele_group_dest_idx对应的bev_mapping_ele
        for (size_t i = 0; i < bev_mapping_eles.size(); ++i){
            if (bev_mapping_eles[i].second == ele_ego_ele_idx){
                continue;
            }else if (bev_mapping_eles[i].second < ele_ego_ele_idx){
                side_lane_idx = i;
                is_left = false; //设置方向为左
                return true; //返回true，表示找到旁车道
            }else{
                side_lane_idx = i;
                is_left = true; //设置方向为左
                return true; //返回true，表示找到旁车道
            }
        }
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetSideLane: "  + std::string(e.what()));
        return false;
    }

    return false; //如果没有找到，返回false
}

bool LocalMap::MakeNodeInfoForSplit(const SDLaneElementGroup& ele_group, int32_t ele_main_lane_idx, 
                                     int32_t ele_group_dest_idx, double s_split_offset, double e_split_offset){
    try
    {
        if (ele_group.empty() || ele_main_lane_idx < 0 || ele_main_lane_idx >= ele_group.size()){
            return false; //如果车道组为空，直接返回false
        }

        if (node_info_.dir == NodeDir::NODE_DIR_UNKNOWN){
            if (ele_main_lane_idx > ele_group_dest_idx){
                node_info_.dir = NodeDir::NODE_DIR_RIGHT; //如果主车道索引大于group_dest索引，设置方向为右
                node_info_.StartPointOffset = s_split_offset;
                node_info_.EndPointOffset = e_split_offset;
                node_info_.LaneChgType = 3; //设置车道变更类型为3
                node_info_.LaneChgTimes = 1;
            }else if (ele_main_lane_idx < ele_group_dest_idx){
                node_info_.dir = NodeDir::NODE_DIR_LEFT; //如果主车道索引小于group_dest索引，设置方向为左
                node_info_.StartPointOffset = s_split_offset;
                node_info_.EndPointOffset = e_split_offset;
                node_info_.LaneChgType = 3; //设置车道变更类型为3
                node_info_.LaneChgTimes = 1;
            }
            
        }else{
            node_info_.EndPointOffset = static_cast<double>(ele_group[ele_main_lane_idx].remain_dist) / 100.0;
        }

    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::MakeNodeInfoForSplit: "  + std::string(e.what()));
        return false;
    }

    return true;
}


bool LocalMap::GetSplitPoint(EFMRefLinePoints& left_line, EFMRefLinePoints& right_line, int32_t& left_split_point, int32_t& right_split_point){
    try
    {
        left_split_point = -1;
        right_split_point = -1;

        if (left_line.empty() && right_line.empty()){
            return false;
        }
        
        
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetSplitPoint: "  + std::string(e.what()));
        return false;
    }

    return true;
}

bool LocalMap::GetMainLaneIdxForGroup(BevLaneElementGroup& bev_ele_group, int32_t& main_lane_idx){
    try
    {
        main_lane_idx = -1;
        
        
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetMainLaneIdxForGroup: "  + std::string(e.what()));
        return false;
    }

    return true;
}

bool LocalMap::CheckCenterLine(BevLaneElementGroup& bev_ele_group, std::vector<int32_t>& target_line, bool& center_line_valid){
    try
    {
        center_line_valid = false;
        if (bev_ele_group.empty()){
            return false;
        } 
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::CheckCenterLine: "  + std::string(e.what()));
        return false;
    }

    return true;
}

bool LocalMap::ChangeCenterLine(BevLaneElementGroup& bev_ele_group, std::vector<int32_t>& target_line){
    try
    {
        if (bev_ele_group.empty()){
            return false;
        } 
        
        
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::ChangeCenterLine: "  + std::string(e.what()));
        return false;
    }

    return true;
}

bool LocalMap::MakeOverlapLane(BevLaneElementGroup& bev_ele_group){
    try
    {
        if (bev_ele_group.empty()){
            return false;
        }

        //遍历车道组，找到左右边线重叠的车道
        for (size_t i = 0; i < bev_ele_group.size(); i++){
            BevLaneElement& bev_ele = bev_ele_group[i];
            if (bev_ele.left_line_points.empty() || bev_ele.right_line_points.empty()){
                continue; //如果左右边线都为空，跳过
            }

            //判断左右边线是否有重叠部分
            if (bev_ele.left_line_points.back().x < bev_ele.right_line_points.front().x || 
                bev_ele.right_line_points.back().x < bev_ele.left_line_points.front().x){
                continue; //如果没有重叠部分，跳过
            }

            //获取左右边线的重叠部分
            double left_x_start = std::max(bev_ele.left_line_points.front().x, bev_ele.right_line_points.front().x);
            double left_x_end = std::min(bev_ele.left_line_points.back().x, bev_ele.right_line_points.back().x);

            //根据left_x_start和left_x_end，左车道插值出该点
            if (false == InterpolateBevLine(bev_ele.left_line_points, left_x_start, bev_ele.left_overlap_right_start_index,
                                            left_x_end, bev_ele.left_overlap_right_end_index, bev_ele.right_line_points)){
                LOG_ERROR("InterpolateBevLine left failed");
                continue;
            }

            //根据left_x_start和left_x_end，右车道插值出该点
            if (false == InterpolateBevLine(bev_ele.right_line_points, left_x_start, bev_ele.right_overlap_left_start_index,
                                            left_x_end, bev_ele.right_overlap_left_end_index, bev_ele.left_line_points)){
                LOG_ERROR("InterpolateBevLine right failed");
                continue;
            }
        }
        
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::MakeOverlapLane: "  + std::string(e.what()));
        return false;
    }

    return true;


}

bool LocalMap::GetPointOffset(const EFMRefLinePoints& line_points, int32_t s_merge_point, int32_t e_merge_point,
                              double& s_merge_offset, double& e_merge_offset) {
    try
    {
        s_merge_offset = -1;
        e_merge_offset = -1;
        LOG_DEBUG("line_points.size(): " + std::to_string(line_points.size()) + ", s_merge_point: " + std::to_string(s_merge_point) + ", e_merge_point: " + std::to_string(e_merge_point));
        if (line_points.size() < 2 || s_merge_point < 0 || e_merge_point < 0 ||
            s_merge_point >= line_points.size() || e_merge_point >= line_points.size()) {
            return false; //如果车道线点数为空，或者起始点、终止点索引不合法，返回false
        }

        LOG_DEBUG("line_points.front(): (" + std::to_string(line_points.front().x) + ", " + std::to_string(line_points.front().y) + ")");
        //判断line_points与自车的关系
        if (line_points.back().x <= 0.0) {
            //线上所有点在自车后方
            return true; //如果车道线点都在自车后方，直接返回true
        }else if (line_points.front().x >= 0.0){
            //线上所有点在自车前方
            double s_x = line_points[0].x; //获取起始点的x坐标
            for (int32_t i = 1; i <= s_merge_point; i++){
                s_x += std::hypot(line_points[i].x - line_points[i - 1].x, 
                                  line_points[i].y - line_points[i - 1].y);
            }
            s_merge_offset = s_x;
            for (int32_t i = s_merge_point + 1; i <= e_merge_point; i++){
                s_x += std::hypot(line_points[i].x - line_points[i - 1].x, 
                                  line_points[i].y - line_points[i - 1].y);
            }
            e_merge_offset = s_x;
            return true; //如果车道线点都在自车前方，直接返回true
        }else{
            //找到X=0的点
            EFMPoint car_point ;
            int32_t befor_car_point_idx = -1;
            for (int32_t i = 0; i < (line_points.size() - 1); ++i) {
                if (line_points[i].x <= 0.0 && line_points[i + 1].x >= 0.0) {
                    befor_car_point_idx = i; //记录自车前方的点的索引
                    break;
                }
            }
            if (befor_car_point_idx < 0) {
                LOG_DEBUG("befor_car_point_idx=" + std::to_string(befor_car_point_idx));
                return false; //如果没有找到自车前方的点，返回false
            }

            if (befor_car_point_idx >= e_merge_point){
                LOG_DEBUG("befor_car_point_idx=" + std::to_string(befor_car_point_idx));
                return true; //如果自车前方的点在终止点之后，直接返回true
            }else if (befor_car_point_idx < s_merge_point){
                double s_x = std::hypot(car_point.x - line_points[befor_car_point_idx + 1].x, 
                                        car_point.y - line_points[befor_car_point_idx + 1].y);
                for (int32_t i = befor_car_point_idx + 1; i < s_merge_point; i++){
                    s_x += std::hypot(line_points[i].x - line_points[i + 1].x, 
                                    line_points[i].y - line_points[i + 1].y);
                }
                s_merge_offset = s_x;
                for (int32_t i = s_merge_point; i < e_merge_point; i++){
                    s_x += std::hypot(line_points[i].x - line_points[i + 1].x, 
                                    line_points[i].y - line_points[i + 1].y);
                }
                e_merge_offset = s_x;
            }else{
                double s_x = std::hypot(car_point.x - line_points[befor_car_point_idx].x, 
                                        car_point.y - line_points[befor_car_point_idx].y);
                for (int32_t i = befor_car_point_idx; i > s_merge_point; i--){
                    s_x += std::hypot(line_points[i].x - line_points[i - 1].x, 
                                    line_points[i].y - line_points[i - 1].y);
                }
                s_merge_offset = -1.0*s_x;

                s_x = std::hypot(car_point.x - line_points[befor_car_point_idx + 1].x, 
                                car_point.y - line_points[befor_car_point_idx + 1].y);
                for (int32_t i = s_merge_point; i < e_merge_point; i++){
                    s_x += std::hypot(line_points[i].x - line_points[i + 1].x, 
                                    line_points[i].y - line_points[i + 1].y);
                }
                e_merge_offset = s_x;
            }
            LOG_DEBUG("s_merge_offset=" + std::to_string(s_merge_offset) + ", e_merge_offset=" + std::to_string(e_merge_offset));
            return true;
        }
    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::GetMergeOffset: "  + std::string(e.what()));
        return false;
    }

    return true;
}

bool LocalMap::MakeNodeInfoForMerge(uint8_t merge_dir, double s_merge_offset, double e_merge_offset) {
    try
    {
        if (merge_dir == 0 || e_merge_offset <= 0){
            return false; //如果合并方向为0，或者起始偏移、终止偏移小于0，返回false
        }
        if (node_info_.dir == NodeDir::NODE_DIR_UNKNOWN) {
            node_info_.dir = (merge_dir == 1) ? NodeDir::NODE_DIR_LEFT : NodeDir::NODE_DIR_RIGHT; //设置方向
            node_info_.StartPointOffset = s_merge_offset; //设置起点偏移
            node_info_.EndPointOffset = e_merge_offset; //设置终点偏移
            node_info_.LaneChgType = 3; //设置车道变更类型
            node_info_.LaneChgTimes = 1; //设置车道变更次数
        }else{
            if (node_info_.dir == NodeDir::NODE_DIR_LEFT && merge_dir == 2) {
                node_info_.dir = NodeDir::NODE_DIR_RIGHT; //如果当前方向是左，且合并方向是右，则设置为右
            } else if (node_info_.dir == NodeDir::NODE_DIR_RIGHT && merge_dir == 1) {
                node_info_.dir = NodeDir::NODE_DIR_LEFT; //如果当前方向是右，且合并方向是左，则设置为左
            }
            node_info_.StartPointOffset = std::min(node_info_.StartPointOffset, s_merge_offset); //设置起点偏移
            node_info_.EndPointOffset = std::max(node_info_.EndPointOffset, e_merge_offset); //设置终点偏移
        }

    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::MakeNodeInfoForMerge: "  + std::string(e.what()));
        return false;
    }

    return true;
}

bool LocalMap::InterpolateBevLine(EFMRefLinePoints& line_points, double left_x_start, int32_t& start_index,
                                  double left_x_end, int32_t& end_index, EFMRefLinePoints& side_line_points) {
    try
    {
        if (line_points.empty()) {
            return false;
        }

        if (left_x_start < line_points.front().x || left_x_end > line_points.back().x) {
            return false; //如果left_x_start小于第一个点的x，或者left_x_end大于最后一个点的x，返回false
        }
        //LOG_DEBUG("InterpolateBevLine: left_x_start=" + std::to_string(left_x_start) + ", left_x_end=" + std::to_string(left_x_end));
        //LOG_DEBUG("InterpolateBevLine: line_points[0].x=" + std::to_string(line_points[0].x) + ", line_points.back().x=" + std::to_string(line_points.back().x));

        //left_x_start和第一个进行比较，相等则直接给start_index赋值，否则进行插值
        start_index = -1;
        if (line_points[0].x == left_x_start){
            start_index = 0;
        }else{
            //如果left_x_start不等于第一个点的x，则进行插值
            for (size_t i = 1; i < line_points.size(); ++i) {
                if (line_points[i].x == left_x_start) {
                    start_index = i;
                    break;
                }else if (line_points[i].x > left_x_start) {
                    //插值
                    double ratio = (left_x_start - line_points[i - 1].x) / (line_points[i].x - line_points[i - 1].x);
                    EFMPoint new_point;
                    new_point.x = left_x_start;
                    new_point.y = line_points[i - 1].y + ratio * (line_points[i].y - line_points[i - 1].y);
                    line_points.insert(line_points.begin() + i, new_point);
                    start_index = i;
                    break;
                }
            }
        }

        //left_x_end和最后一个进行比较，想等则直接给end_index赋值，否则进行插值
        end_index = -1;
        if (line_points.back().x == left_x_end){
            end_index = line_points.size() - 1;
        }else{
            //如果left_x_end不等于最后一个点的x，则进行插值
            for (size_t i = line_points.size() - 2; i >= 0; --i) {
                if (line_points[i].x == left_x_end) {
                    end_index = i;
                    break;
                }else if (line_points[i].x < left_x_end) {
                    //插值
                    double ratio = (left_x_end - line_points[i].x) / (line_points[i + 1].x - line_points[i].x);
                    EFMPoint new_point;
                    new_point.x = left_x_end;
                    new_point.y = line_points[i].y + ratio * (line_points[i + 1].y - line_points[i].y);
                    line_points.insert(line_points.begin() + i + 1, new_point);
                    end_index = i + 1;
                    break;
                }
            }
        }

        if (start_index < 0 || end_index < 0 || start_index >= end_index) {
            LOG_ERROR("start_index: " + std::to_string(start_index) + ", end_index: " + std::to_string(end_index));
            return false; //索引不合法
        }

        //用side_line_points的点进行插值
        if (side_line_points.empty()) {
            return true;
        }

        for (size_t i = start_index + 1; i <= end_index; ++i) {
            //在side_line_points中查找大于line_points[i-1]的x，小于line_points[i]的x的点
            //如果找到了，则直接使用该点，否则进行插值
            if (i >= line_points.size()) {
                break; //防止越界
            }
            if (line_points[i].x < line_points[i - 1].x) {
                continue; //如果当前点的x小于前一个点的x，跳过
            }

            //这里可以添加一个条件判断，防止死循环
            int side_idx = 0;
            while (side_idx < side_line_points.size()) {
                auto it = std::find_if(side_line_points.begin(), side_line_points.end(),
                                    [x = line_points[i].x, y = line_points[i-1].x](const EFMPoint& point) { return point.x < x && point.x > y; });
                if (it == side_line_points.end()) {
                    break; //找到后跳出循环
                }
                //如果找到了，则直接使用该点进行插值
                double ratio = (it->x - line_points[i-1].x) / (line_points[i].x - line_points[i-1].x);
                EFMPoint new_point;
                new_point.x = it->x;
                new_point.y = line_points[i-1].y + ratio * (line_points[i].y - line_points[i-1].y);
                line_points.insert(line_points.begin() + i, new_point);
                i = i + 1; //插入后，i需要加1，因为新插入的点会影响后面的点
                end_index = end_index + 1; //end_index也需要加1，因为新插入的点会影响后面的点
                ++side_idx;
            }
        }

    }
    catch(const std::exception& e)
    {
        LOG_ERROR("LocalMap::InterpolateBevLine: "  + std::string(e.what()));
        return false;
    }

    return true;
}


bool LocalMap::MakeCenterLines(BevLaneElement& ego_lane, BevLaneElement& left_lane, BevLaneElement& right_lane, uint8_t prior_dir, bool is_off_ramp) {
    ego_lane.center_line_points.clear();
    left_lane.center_line_points.clear();
    right_lane.center_line_points.clear();

    int lane_merge_dir = 0; //map_lane.lanes[2].lane_merge[0].dir.data_;
    int lane_split_dir = 0; //map_lane.lanes[2].lane_split[0].dir.data_;
    float merge_s_start = 0; //map_lane.lanes[2].lane_merge[0].s_start;
    float merge_s_end = 0; //map_lane.lanes[2].lane_merge[0].s_end;
    float split_s_start = 0; //map_lane.lanes[2].lane_split[0].s_start;
    float split_s_end = 0; //map_lane.lanes[2].lane_split[0].s_end;
    //以上接口待地图ok后重新匹配
    
    static uint8_t prior_dir_old = 0;
    
    if ((prior_dir == 1 || prior_dir == 3) && is_off_ramp) {
        //车辆需要向左变道，且属于ramp范围
        if (!ego_lane.left_line_points.empty() && !ego_lane.right_line_points.empty()) {
            //两侧都有线，需要走通用逻辑
            ego_lane.center_line_points = GenerateCenterLine(ego_lane.left_line_points, ego_lane.right_line_points, 0);
        } else if (!ego_lane.left_line_points.empty()) {
            //一侧偏移
            ego_lane.center_line_points = offsetBevLine(ego_lane.left_line_points, -1.75);
        } else if (!ego_lane.right_line_points.empty()) {
            //一侧偏移
            ego_lane.center_line_points = offsetBevLine(ego_lane.right_line_points, 1.75);
        }

        MergeCdnEgoLaneStitch(ego_lane, lane_merge_dir, merge_s_end, ego_lane.center_line_points);

        //左侧车道线处理
        if (!left_lane.right_line_points.empty()) {
            //优先用左侧车道右线偏移
            left_lane.center_line_points = offsetBevLine(left_lane.right_line_points, 1.75);
        } else if (!left_lane.left_line_points.empty()) {
            //右侧线没有，用左线偏移
            left_lane.center_line_points = offsetBevLine(left_lane.left_line_points, -1.75);
        }

        //右侧车道线处理，右侧车道如果没有右车道边线，就不补右车道的中心线
        if (!right_lane.left_line_points.empty() && !right_lane.right_line_points.empty()) {
            //通用处理
            right_lane.center_line_points = GenerateCenterLine(right_lane.left_line_points, right_lane.right_line_points, 2);
        } else if (!right_lane.right_line_points.empty()) {
            //右侧线偏移
            right_lane.center_line_points = offsetBevLine(right_lane.right_line_points, 1.75);
        } 

        // DownRampCenterLineStitch(left_lane, right_lane, prior_dir, left_lane.center_line_points, right_lane.center_line_points);
    } else if (prior_dir == 0 && (prior_dir_old == 1 || prior_dir_old == 3) && is_off_ramp) {
        //已经进入匝道，距离split点50米以内，做法同上
        if (!ego_lane.left_line_points.empty() && !ego_lane.right_line_points.empty()) {
            ego_lane.center_line_points = GenerateCenterLine(ego_lane.left_line_points, ego_lane.right_line_points, 0);
        } else if (!ego_lane.left_line_points.empty()) {
            ego_lane.center_line_points = offsetBevLine(ego_lane.left_line_points, -1.75);
        } else if (!ego_lane.right_line_points.empty()) {
            ego_lane.center_line_points = offsetBevLine(ego_lane.right_line_points, 1.75);
        }

        MergeCdnEgoLaneStitch(ego_lane, lane_merge_dir, merge_s_end, ego_lane.center_line_points);

        //左侧车道线处理，左侧车道如果没有左车道边线，就不补左车道的中心线
        if (!left_lane.left_line_points.empty() && !left_lane.right_line_points.empty()) {
            left_lane.center_line_points = GenerateCenterLine(left_lane.left_line_points, left_lane.right_line_points, 1);
        } else if (!left_lane.left_line_points.empty()) {
            left_lane.center_line_points = offsetBevLine(left_lane.left_line_points, -1.75);
        }

        //右侧车道线处理，右侧车道如果没有右车道边线，就不补右车道的中心线
        if (!right_lane.left_line_points.empty() && !right_lane.right_line_points.empty()) {
            right_lane.center_line_points = GenerateCenterLine(right_lane.left_line_points, right_lane.right_line_points, 2);
        } else if (!right_lane.right_line_points.empty()) {
            right_lane.center_line_points = offsetBevLine(right_lane.right_line_points, 1.75);
        }
    } else if ((prior_dir == 2 || prior_dir == 4) && is_off_ramp) {
        //和左侧下匝道相反做法
        if (!ego_lane.left_line_points.empty() && !ego_lane.right_line_points.empty()) {
            ego_lane.center_line_points = GenerateCenterLine(ego_lane.left_line_points, ego_lane.right_line_points, 0);
        } else if (!ego_lane.left_line_points.empty()) {
            ego_lane.center_line_points = offsetBevLine(ego_lane.left_line_points, -1.75);
        } else if (!ego_lane.right_line_points.empty()) {
            ego_lane.center_line_points = offsetBevLine(ego_lane.right_line_points, 1.75);
        }

        MergeCdnEgoLaneStitch(ego_lane, lane_merge_dir, merge_s_end, ego_lane.center_line_points);

        if (!left_lane.left_line_points.empty() && !left_lane.right_line_points.empty()) {
            left_lane.center_line_points = GenerateCenterLine(left_lane.left_line_points, left_lane.right_line_points, 1);
        } else if (!left_lane.left_line_points.empty()) {
            left_lane.center_line_points = offsetBevLine(left_lane.left_line_points, -1.75);
        }

        if (!right_lane.left_line_points.empty()) {
            right_lane.center_line_points = offsetBevLine(right_lane.left_line_points, -1.75);
        } else if (!right_lane.right_line_points.empty()) {
            right_lane.center_line_points = offsetBevLine(right_lane.right_line_points, 1.75);
        }

        // DownRampCenterLineStitch(left_lane, right_lane, prior_dir, left_lane.center_line_points, right_lane.center_line_points);
    } else if (prior_dir == 0 && (prior_dir_old == 2 || prior_dir_old == 4) && is_off_ramp) {
        //同理
        if (!ego_lane.left_line_points.empty() && !ego_lane.right_line_points.empty()) {
            ego_lane.center_line_points = GenerateCenterLine(ego_lane.left_line_points, ego_lane.right_line_points, 0);
        } else if (!ego_lane.right_line_points.empty()) {
            ego_lane.center_line_points = offsetBevLine(ego_lane.right_line_points, 1.75);
        } else if (!ego_lane.left_line_points.empty()) {
            ego_lane.center_line_points = offsetBevLine(ego_lane.left_line_points, -1.75);
        }

        MergeCdnEgoLaneStitch(ego_lane, lane_merge_dir, merge_s_end, ego_lane.center_line_points);

        if (!left_lane.left_line_points.empty() && !left_lane.right_line_points.empty()) {
            left_lane.center_line_points = GenerateCenterLine(left_lane.left_line_points, left_lane.right_line_points, 1);
        } else if (!left_lane.left_line_points.empty()) {
            left_lane.center_line_points = offsetBevLine(left_lane.left_line_points, -1.75);
        }

        if (!right_lane.left_line_points.empty() && !right_lane.right_line_points.empty()) {
            right_lane.center_line_points = GenerateCenterLine(right_lane.left_line_points, right_lane.right_line_points, 2);
        } else if (!right_lane.right_line_points.empty()) {
            right_lane.center_line_points = offsetBevLine(right_lane.right_line_points, 1.75);
        }
    } else {
        //通用做法
        if (!ego_lane.left_line_points.empty() && !ego_lane.right_line_points.empty()) {
            ego_lane.center_line_points = GenerateCenterLine(ego_lane.left_line_points, ego_lane.right_line_points, 0);
        } else if (!ego_lane.left_line_points.empty()) {
            ego_lane.center_line_points = offsetBevLine(ego_lane.left_line_points, -1.75);
        } else if (!ego_lane.right_line_points.empty()) {
            ego_lane.center_line_points = offsetBevLine(ego_lane.right_line_points, 1.75);
        }

        MergeCdnEgoLaneStitch(ego_lane, lane_merge_dir, merge_s_end, ego_lane.center_line_points);

        MergeSplitCdnEgoLaneOffset(ego_lane, lane_merge_dir, merge_s_start, merge_s_end, lane_split_dir, split_s_start, split_s_end, ego_lane.center_line_points);

        if (!left_lane.left_line_points.empty() && !left_lane.right_line_points.empty()) {
            left_lane.center_line_points = GenerateCenterLine(left_lane.left_line_points, left_lane.right_line_points, 1);
        } else if (!left_lane.left_line_points.empty()) {
            left_lane.center_line_points = offsetBevLine(left_lane.left_line_points, -1.75);
        }

        if (!right_lane.left_line_points.empty() && !right_lane.right_line_points.empty()) {
            right_lane.center_line_points = GenerateCenterLine(right_lane.left_line_points, right_lane.right_line_points, 2);
        } else if (!right_lane.right_line_points.empty()) {
            right_lane.center_line_points = offsetBevLine(right_lane.right_line_points, 1.75);
        }
    }
    if (prior_dir != 0) {
        prior_dir_old = prior_dir;
    } else if (!is_off_ramp) {
        prior_dir_old = 0;
    }

    if (!ego_lane.center_line_points.empty() && ego_lane.center_line_points[0].x > 5.0) {
        //自车车道中心线在前方，沿到自车位置
        EFMRefLinePoints egolane_Points_front;
        for (double x = ego_lane.center_line_points[0].x - 2.5; x >= -2.5; x -= 2.5) {
            EFMPoint pnt = LinearInterp(ego_lane.center_line_points[0], ego_lane.center_line_points[1], x);
            egolane_Points_front.push_back(pnt);
        }
        std::reverse(egolane_Points_front.begin(), egolane_Points_front.end());
        ego_lane.center_line_points.insert(ego_lane.center_line_points.begin(), egolane_Points_front.begin(), egolane_Points_front.end());
    }
    if (!left_lane.center_line_points.empty() && left_lane.center_line_points[0].x > 5.0) {
        //左侧车道中心线在前方，沿到自车位置
        EFMRefLinePoints leftlane_Points_front;
        for (double x = left_lane.center_line_points[0].x - 2.5; x >= -2.5; x -= 2.5) {
            EFMPoint pnt = LinearInterp(left_lane.center_line_points[0], left_lane.center_line_points[1], x);
            leftlane_Points_front.push_back(pnt);
        }
        std::reverse(leftlane_Points_front.begin(), leftlane_Points_front.end());
        left_lane.center_line_points.insert(left_lane.center_line_points.begin(), leftlane_Points_front.begin(), leftlane_Points_front.end());
    }
    if (!right_lane.center_line_points.empty() && right_lane.center_line_points[0].x > 5.0) {
        //右侧车道中心线在前方，沿到自车位置
        EFMRefLinePoints rightlane_Points_front;
        for (double x = right_lane.center_line_points[0].x - 2.5; x >= -2.5; x -= 2.5) {
            EFMPoint pnt = LinearInterp(right_lane.center_line_points[0], right_lane.center_line_points[1], x);
            rightlane_Points_front.push_back(pnt);
        }
        std::reverse(rightlane_Points_front.begin(), rightlane_Points_front.end());
        right_lane.center_line_points.insert(right_lane.center_line_points.begin(), rightlane_Points_front.begin(), rightlane_Points_front.end());
    }

    // 杰哥代码，待地图ok后重新匹配
    // if (follow_lane_flag_ > 0){
    //     bool right_split_flag = false;
    //     if (!right_lane.center_line_points.empty() && !ego_lane.center_line_points.empty()){
    //         if (ChangeEgoLaneForVirtualSplit(ego_lane, right_lane, false)){
    //             right_split_flag = true;
    //         }
    //     }

    //     if (!left_lane.center_line_points.empty() && !ego_lane.center_line_points.empty() && false == right_split_flag) {
    //         ChangeEgoLaneForVirtualSplit(left_lane, ego_lane, true);
    //     }        
        
    // }
    return true;
}

EFMRefLinePoints LocalMap::GenerateCenterLine(const EFMRefLinePoints& leftline, const EFMRefLinePoints& rightline, int lane_id) {
    //lane_id ： 0: 自车道，1: 左侧车道，2: 右侧车道
    EFMRefLinePoints Centerline;

    if (leftline.empty() || rightline.empty() || (leftline.back().x <= 0.0 && rightline.back().x <= 0.0)) {
        // 左右车道线有一条为空或都在自车后方，返回空
        return Centerline;
    } else if (!leftline.empty() && leftline.back().x <= 0.0) {
        // 左侧车道线不为空且左侧车道线在自车后方，拿右侧偏移
        Centerline = offsetBevLine(rightline, 1.75);
        return Centerline;
    } else if (!rightline.empty() && rightline.back().x <= 0.0) {
        // 右侧车道线不为空且右侧车道线在自车后方，拿左侧偏移
        Centerline = offsetBevLine(leftline, -1.75);
        return Centerline;
    } else {
        static int ego_offset_dir = 0;  //偏移方向
        static int left_offset_dir = 0;
        static int right_offset_dir = 0;
        static bool ego_wide_lane_ = false;
        static bool left_wide_lane = false;
        static bool right_wide_lane = false;

        if (lane_id == 0) {
            //超宽判断
            IsWideLane(leftline, rightline, change_lane_flag_, ego_wide_lane_, ego_offset_dir);
            // ZTEXT("EFM_INFO", "ego_wide_lane: ", 120, -65, "ego_wide_lane: {}", ego_wide_lane_);
            // ZTEXT("EFM_INFO", "ego_offset_dir: ", 120, -70, "ego_offset_dir: {}", ego_offset_dir);
            if (ego_wide_lane_ && ego_offset_dir == 1) {
                //自车道超宽且自车距离左车道线较近，则将左车道线向右偏1.75m作为右车道中心线
                Centerline = offsetBevLine(leftline, -1.75);
                CenterLineStitch(leftline, rightline, Centerline);
                return Centerline;
            } else if (ego_wide_lane_ && ego_offset_dir == 2) {
                //自车道超宽且自车距离右车道线较近，则将右车道线向左偏1.75m作为左车道中心线
                Centerline = offsetBevLine(rightline, 1.75);
                CenterLineStitch(leftline, rightline, Centerline);
                return Centerline;
            }
        } else if (lane_id == 1) {
            IsWideLane(leftline, rightline, 0, left_wide_lane, left_offset_dir);
            // ZTEXT("EFM_INFO", "left_wide_lane: ", 120, -75, "left_wide_lane: {}", left_wide_lane);
            // ZTEXT("EFM_INFO", "left_offset_dir: ", 120, -80, "left_offset_dir: {}", left_offset_dir);
            if (left_wide_lane && left_offset_dir == 1) {
                //自车道超宽且自车距离左车道线较近，则将左车道线向右偏1.75m作为右车道中心线
                Centerline = offsetBevLine(leftline, -1.75);
                CenterLineStitch(leftline, rightline, Centerline);
                return Centerline;
            } else if (left_wide_lane && left_offset_dir == 2) {
                //自车道超宽且自车距离右车道线较近，则将右车道线向左偏1.75m作为左车道中心线
                Centerline = offsetBevLine(rightline, 1.75);
                CenterLineStitch(leftline, rightline, Centerline);
                return Centerline;
            }
        } else {
            IsWideLane(leftline, rightline, 0, right_wide_lane, right_offset_dir);
            // ZTEXT("EFM_INFO", "right_wide_lane: ", 120, -85, "right_wide_lane: {}", right_wide_lane);
            // ZTEXT("EFM_INFO", "right_offset_dir: ", 120, -90, "right_offset_dir: {}", right_offset_dir);
            if (right_wide_lane && right_offset_dir == 1) {
                //自车道超宽且自车距离左车道线较近，则将左车道线向右偏1.75m作为右车道中心线
                Centerline = offsetBevLine(leftline, -1.75);
                CenterLineStitch(leftline, rightline, Centerline);
                return Centerline;
            } else if (right_wide_lane && right_offset_dir == 2) {
                //自车道超宽且自车距离右车道线较近，则将右车道线向左偏1.75m作为左车道中心线
                Centerline = offsetBevLine(rightline, 1.75);
                CenterLineStitch(leftline, rightline, Centerline);
                return Centerline;
            } 
        }

        if (leftline[0].x >= 0 || rightline[0].x >= 0) {
            // 左右车道线起点至少有一条在自车前方
            if (leftline[0].x >= rightline[0].x) {
                // 左侧车道线起点较右侧车道线在自车前方,找到右侧车道大于等于左侧车道线起点的点
                auto it = std::lower_bound(rightline.begin(), rightline.end(), leftline[0].x, 
                                           [](const EFMPoint& point, double x) { return point.x < x; });
                int idx = std::distance(rightline.begin(), it);

                // 对右侧车道线进行线性插值找到对应点
                EFMPoint startpoint;
                if ((idx == 0)) {
                    startpoint = rightline[0];
                } else if (idx < rightline.size()) {
                    startpoint = LinearInterp(rightline[idx - 1], rightline[idx], leftline[0].x);
                } else {
                    startpoint = LinearInterp(rightline[idx - 2], rightline[idx - 1], leftline[0].x);
                }

                double left_arc_length = 0.0;
                double right_arc_length = 0.0;
                std::vector<double> left_arc;
                std::vector<double> right_arc;
                EFMRefLinePoints rightline_new;
                left_arc.push_back(left_arc_length);

                for (int i = 1; i < leftline.size(); ++i) {
                    double dx = leftline[i].x - leftline[i - 1].x;
                    double dy = leftline[i].y - leftline[i - 1].y;
                    left_arc_length += std::hypot(dx, dy);
                    left_arc.push_back(left_arc_length);
                }

                rightline_new.push_back(startpoint);
                right_arc.push_back(right_arc_length);
                if (idx > 0 && idx < rightline.size()) {
                    rightline_new.push_back(rightline[idx]);
                    right_arc_length += std::hypot(rightline[idx].x - startpoint.x, rightline[idx].y - startpoint.y);
                    right_arc.push_back(right_arc_length);
                }

                for (int i = idx + 1; i < rightline.size(); ++i) {
                    double dx = rightline[i].x - rightline[i - 1].x;
                    double dy = rightline[i].y - rightline[i - 1].y;
                    right_arc_length += std::hypot(dx, dy);
                    right_arc.push_back(right_arc_length);
                    rightline_new.push_back(rightline[i]);
                }
                double max_length = std::min(left_arc_length, right_arc_length);
                // 间隔2.5m重新采样，计算中心线
                for (double s = 0; s <= max_length; s += 2.5) {
                    // 双车道同步采样
                    EFMPoint left = interpolateAtLength(leftline, left_arc, s);
                    EFMPoint right = interpolateAtLength(rightline_new, right_arc, s);

                    // 计算中心点
                    Centerline.push_back((left + right) / 2.0);
                }

                CenterLineStitch(leftline, rightline, Centerline);
            } else {
                // 右侧车道线起点较左侧车道线在自车前方,找到左侧车道大于等于右侧车道线起点的点
                auto it = std::lower_bound(leftline.begin(), leftline.end(), rightline[0].x, 
                                           [](const EFMPoint& point, double x) { return point.x < x; });
                int idx = std::distance(leftline.begin(), it);
                if (idx == 0) LOG_ERROR("std::distance Out of range");
                // 对左侧车道线进行线性插值找到对应点
                EFMPoint startpoint;
                if (idx < leftline.size()) {
                    startpoint = LinearInterp(leftline[idx - 1], leftline[idx], rightline[0].x);
                } else {
                    startpoint = LinearInterp(leftline[idx - 2], leftline[idx - 1], rightline[0].x);
                }

                double left_arc_length = 0.0;
                double right_arc_length = 0.0;
                std::vector<double> left_arc;
                std::vector<double> right_arc;
                EFMRefLinePoints leftline_new;
                right_arc.push_back(right_arc_length);

                for (int i = 1; i < rightline.size(); ++i) {
                    double dx = rightline[i].x - rightline[i - 1].x;
                    double dy = rightline[i].y - rightline[i - 1].y;
                    right_arc_length += std::hypot(dx, dy);
                    right_arc.push_back(right_arc_length);
                }

                left_arc.push_back(left_arc_length);
                leftline_new.push_back(startpoint);
                if (idx < leftline.size()) {
                    leftline_new.push_back(leftline[idx]);
                    left_arc_length += std::hypot(leftline[idx].x - startpoint.x, leftline[idx].y - startpoint.y);
                    left_arc.push_back(left_arc_length);
                }

                for (int i = idx + 1; i < leftline.size(); ++i) {
                    double dx = leftline[i].x - leftline[i - 1].x;
                    double dy = leftline[i].y - leftline[i - 1].y;
                    left_arc_length += std::hypot(dx, dy);
                    left_arc.push_back(left_arc_length);
                    leftline_new.push_back(leftline[i]);
                }
                double max_length = std::min(left_arc_length, right_arc_length);
                // 间隔2.5m重新采样，计算中心线
                for (double s = 0; s <= max_length; s += 2.5) {
                    // 双车道同步采样
                    EFMPoint left = interpolateAtLength(leftline_new, left_arc, s);
                    EFMPoint right = interpolateAtLength(rightline, right_arc, s);

                    // 计算中心点
                    Centerline.push_back((left + right) / 2.0);
                }

                CenterLineStitch(leftline, rightline, Centerline);
            }
        } else {
            // 左右车道线起点均在自车后方
            auto itl = std::lower_bound(leftline.begin(), leftline.end(), 0.0, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
            int left_zero_index = std::distance(leftline.begin(), itl) - 1;

            auto itr = std::lower_bound(rightline.begin(), rightline.end(), 0.0, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
            int right_zero_index = std::distance(rightline.begin(), itr) - 1;
            if (left_zero_index < 0 || left_zero_index > leftline.size() - 2) LOG_ERROR("std::distance Out of range");
            if (right_zero_index < 0 || right_zero_index > rightline.size() - 2) LOG_ERROR("std::distance Out of range");
            // 找到左右车道线x为0的位置作为起点
            EFMPoint leftstartpoint;
            leftstartpoint = LinearInterp(leftline[left_zero_index], leftline[left_zero_index + 1], 0.0);
          
            EFMPoint rightstartpoint;
            rightstartpoint = LinearInterp(rightline[right_zero_index], rightline[right_zero_index + 1], 0.0);
            // 分别对左右车道线后方点计算弧长
            double left_arc_length = 0.0;
            double right_arc_length = 0.0;
            std::vector<double> left_arc;
            std::vector<double> right_arc;
            left_arc.push_back(left_arc_length);
            left_arc_length = -std::hypot(leftline[left_zero_index].x - leftstartpoint.x,
                                          leftline[left_zero_index].y - leftstartpoint.y);
            left_arc.push_back(left_arc_length);
            for (int i = left_zero_index; i > 0; --i) {
                double dx = leftline[i].x - leftline[i - 1].x;
                double dy = leftline[i].y - leftline[i - 1].y;
                left_arc_length -= std::hypot(dx, dy);
                left_arc.push_back(left_arc_length);
            }
            std::reverse(left_arc.begin(), left_arc.end());
            right_arc.push_back(right_arc_length);
            right_arc_length = -std::hypot(rightline[right_zero_index].x - rightstartpoint.x,
                                           rightline[right_zero_index].y - rightstartpoint.y);
            right_arc.push_back(right_arc_length);
            for (int i = right_zero_index; i > 0; --i) {
                double dx = rightline[i].x - rightline[i - 1].x;
                double dy = rightline[i].y - rightline[i - 1].y;
                right_arc_length -= std::hypot(dx, dy);
                right_arc.push_back(right_arc_length);
            }
            std::reverse(right_arc.begin(), right_arc.end());
            double min_length = std::max(left_arc_length, right_arc_length);
            // 间隔2.5m重采样计算自车后方车道中心线
            Centerline.push_back({0.0, (leftstartpoint.y + rightstartpoint.y) / 2.0});
            for (double s = -2.5; s >= min_length; s -= 2.5) {
                // 双车道同步采样
                EFMPoint left = interpolateAtLength(leftline, left_arc, s);
                EFMPoint right = interpolateAtLength(rightline, right_arc, s);

                // 计算中心点
                Centerline.push_back((left + right) / 2.0);
            }
            std::reverse(Centerline.begin(), Centerline.end());
            // 分别对左右车道线前方点计算弧长
            left_arc.pop_back();
            right_arc.pop_back();
            left_arc_length = std::hypot(leftline[left_zero_index + 1].x - leftstartpoint.x,
                                         leftline[left_zero_index + 1].y - leftstartpoint.y);
            left_arc.push_back(left_arc_length);
            for (int i = left_zero_index + 1; i < leftline.size() - 1; ++i) {
                double dx = leftline[i + 1].x - leftline[i].x;
                double dy = leftline[i + 1].y - leftline[i].y;
                left_arc_length += std::hypot(dx, dy);
                left_arc.push_back(left_arc_length);
            }
            right_arc_length = std::hypot(rightline[right_zero_index + 1].x - rightstartpoint.x,
                                          rightline[right_zero_index + 1].y - rightstartpoint.y);
            right_arc.push_back(right_arc_length);
            for (int i = right_zero_index + 1; i < rightline.size() - 1; ++i) {
                double dx = rightline[i + 1].x - rightline[i].x;
                double dy = rightline[i + 1].y - rightline[i].y;
                right_arc_length += std::hypot(dx, dy);
                right_arc.push_back(right_arc_length);
            }
            double max_length = std::min(left_arc_length, right_arc_length);
            // 间隔2.5m重采样计算自车前方车道中心线
            for (double s = 2.5; s <= max_length; s += 2.5) {
                // 双车道同步采样
                EFMPoint left = interpolateAtLength(leftline, left_arc, s);
                EFMPoint right = interpolateAtLength(rightline, right_arc, s);

                // 计算中心点
                Centerline.push_back((left + right) / 2.0);
            }

            CenterLineStitch(leftline, rightline, Centerline);
        }
    }
    return Centerline;
}

EFMRefLinePoints LocalMap::offsetBevLine(const EFMRefLinePoints& BevLine, double offset) {
    if (BevLine.size() < 2) {
        return BevLine;
    }

    EFMRefLinePoints new_line;
    //过滤相同点
    for (size_t i = 0; i < BevLine.size() - 1; i++)
    {
        if (fabs(BevLine[i].x - BevLine[i + 1].x) < 1e-6 && fabs(BevLine[i].y - BevLine[i + 1].y) < 1e-6) {
            continue;
        } else {
            new_line.push_back(BevLine[i]);
        }
    }
    new_line.push_back(BevLine.back());

    if (new_line.size() < 2) {
        return new_line;
    }

    EFMRefLinePoints Centerline;
    // 处理第一个点
    EFMPoint tangent = new_line[1] - new_line[0];
    EFMPoint normal;
    normal = EFMPoint((new_line[0].y - new_line[1].y)/sqrt(tangent.InnerProduct(tangent)), (new_line[1].x - new_line[0].x)/sqrt(tangent.InnerProduct(tangent)));

    Centerline.push_back(new_line[0] + normal * offset);

    // 处理中间点
    for (size_t i = 1; i < new_line.size() - 1; ++i) {
        EFMPoint prevTangent = new_line[i] - new_line[i-1];
        EFMPoint nextTangent = new_line[i+1] - new_line[i];

        // 平均两个切线得到更平滑的法向量
        EFMPoint avgTangent((prevTangent + nextTangent) / 2.0);
        normal = EFMPoint(- (prevTangent.y + nextTangent.y) / 2.0 / sqrt(avgTangent.InnerProduct(avgTangent)),
                           (prevTangent.x + nextTangent.x) / 2.0 / sqrt(avgTangent.InnerProduct(avgTangent)));
        Centerline.push_back(new_line[i] + normal * offset);
    }
    
    // 处理最后一个点
    tangent = new_line[new_line.size()-1] - new_line[new_line.size()-2];
    normal = EFMPoint((new_line[new_line.size()-2].y - new_line[new_line.size()-1].y)/sqrt(tangent.InnerProduct(tangent)),
                      (new_line[new_line.size()-1].x - new_line[new_line.size()-2].x)/sqrt(tangent.InnerProduct(tangent)));

    Centerline.push_back(new_line[new_line.size()-1] + normal * offset);

    return Centerline;
}

void LocalMap::MergeCdnEgoLaneStitch(const BevLaneElement& ego_lane, int lane_merge_dir, float s_end, EFMRefLinePoints& egolane_Points) {
    //merge尾部，用边线偏移拼接，保证merge场景可以寻线上去
    if (lane_merge_dir == 1 && s_end < 100.0f) {
        for (int i = 0; i < egolane_Points.size(); i++) {
            auto it = std::lower_bound(ego_lane.right_line_points.begin(), ego_lane.right_line_points.end(), egolane_Points[i].x, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx = std::distance(ego_lane.right_line_points.begin(), it);

            double offset;
            if (idx == 0) continue;
            else if (idx < ego_lane.right_line_points.size()) offset = PointLineDistance(ego_lane.right_line_points[idx - 1], ego_lane.right_line_points[idx], egolane_Points[i]);
            else break;

            if (std::abs(offset) >= 1.7) {
                continue;
            } else {
                egolane_Points.erase(egolane_Points.begin() + i, egolane_Points.end());
            }

            EFMRefLinePoints egolane_Points_front;
            egolane_Points_front = offsetBevLine(ego_lane.right_line_points, 1.75);
            auto start = egolane_Points_front.begin() + idx;
            auto end = egolane_Points_front.end();
            egolane_Points.insert(egolane_Points.end(), start, end);
            break;
        }
    } else if (lane_merge_dir == 4 && s_end < 100.0f) {
        for (int i = 0; i < egolane_Points.size(); i++) {
            auto it = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), egolane_Points[i].x, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx = std::distance(ego_lane.left_line_points.begin(), it);

            double offset;
            if (idx == 0) continue;
            else if (idx < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx - 1], ego_lane.left_line_points[idx], egolane_Points[i]);
            else break;

            if (std::abs(offset) >= 1.7) {
                continue;
            } else {
                egolane_Points.erase(egolane_Points.begin() + i, egolane_Points.end());
            }

            EFMRefLinePoints egolane_Points_front;
            egolane_Points_front = offsetBevLine(ego_lane.left_line_points, -1.75);
            auto start = egolane_Points_front.begin() + idx;
            auto end = egolane_Points_front.end();
            egolane_Points.insert(egolane_Points.end(), start, end);
            break;
        }
    }
}

void LocalMap::MergeSplitCdnEgoLaneOffset(const BevLaneElement& ego_lane, int lane_merge_dir, float merge_s_start, float merge_s_end,
                        int lane_split_dir, float split_s_start, float split_s_end, EFMRefLinePoints& egolane_Points) {
    //经过旁边有分歧和合流的地方，用一侧线进行偏移

    static float merge_s_end_old = merge_s_end;
    static float split_s_end_old = split_s_end;
    static int offset_dir = 0;
    static double offset;
    static bool offset_flag = false;
    static int offset_count = 0;

    if (offset_dir == 0 && (!ego_wide_lane_ || change_lane_flag_ == 0)) {
        if (lane_merge_dir == 5 && ((merge_s_start > 0.0f && merge_s_start < 50.0f) || (merge_s_start < 1e-6f && merge_s_end < 100.0f))) {
            //左线右偏，根据位置merge点位置
            offset_dir = 2;

            auto it = std::lower_bound(ego_lane.right_line_points.begin(), ego_lane.right_line_points.end(), 0.0, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx = std::distance(ego_lane.right_line_points.begin(), it);

            if (idx < ego_lane.right_line_points.size()) {
                auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx].x, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
                int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

                if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx]);
                else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx]);
                else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx]);
            } else {
                auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx - 1].x, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
                int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

                if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx - 1]);
                else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
                else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
            }
        } else if (lane_split_dir == 4 && ((split_s_start > 0.0f && split_s_start < 50.0f) || (split_s_start < 1e-6f && split_s_end < 100.0f))) {
            //左线右偏
            offset_dir = 4;

            auto it = std::lower_bound(ego_lane.right_line_points.begin(), ego_lane.right_line_points.end(), 0.0, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx = std::distance(ego_lane.right_line_points.begin(), it);

            if (idx < ego_lane.right_line_points.size()) {
                auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx].x, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
                int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

                if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx]);
                else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], egolane_Points[idx]);
                else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx]);
            } else {
                auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx - 1].x, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
                int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

                if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx - 1]);
                else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
                else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
            }
        } else if (lane_merge_dir == 2 && ((merge_s_start > 0.0f && merge_s_start < 50.0f) || (merge_s_start < 1e-6f && merge_s_end < 100.0f))) {
            //右线左偏
            offset_dir = 1;

            auto it = std::lower_bound(ego_lane.right_line_points.begin(), ego_lane.right_line_points.end(), 0.0, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx = std::distance(ego_lane.right_line_points.begin(), it);

            if (idx < ego_lane.right_line_points.size()) {
                auto it_right = std::lower_bound(ego_lane.right_line_points.begin(), ego_lane.right_line_points.end(), ego_lane.right_line_points[idx].x, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
                int idx_right = std::distance(ego_lane.right_line_points.begin(), it_right);

                if (idx_right == 0) offset = PointLineDistance(ego_lane.right_line_points[0], ego_lane.right_line_points[1], ego_lane.right_line_points[idx]);
                else if (idx_right < ego_lane.right_line_points.size()) offset = PointLineDistance(ego_lane.right_line_points[idx_right - 1], ego_lane.right_line_points[idx_right], egolane_Points[idx]);
                else offset = PointLineDistance(ego_lane.right_line_points[idx_right - 1], ego_lane.right_line_points[idx_right], ego_lane.right_line_points[idx]);
            } else {
                auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx - 1].x, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
                int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

                if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx - 1]);
                else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
                else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
            }
        } else if (lane_split_dir == 1 && ((split_s_start > 0.0f && split_s_start < 50.0f) || (split_s_start < 1e-6f && split_s_end < 100.0f))) {
            //右线左偏
            offset_dir = 3;

            auto it = std::lower_bound(ego_lane.right_line_points.begin(), ego_lane.right_line_points.end(), 0.0, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx = std::distance(ego_lane.right_line_points.begin(), it);

            if (idx < ego_lane.right_line_points.size()) {
                auto it_right = std::lower_bound(ego_lane.right_line_points.begin(), ego_lane.right_line_points.end(), ego_lane.right_line_points[idx].x, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
                int idx_right = std::distance(ego_lane.right_line_points.begin(), it_right);

                if (idx_right == 0) offset = PointLineDistance(ego_lane.right_line_points[0], ego_lane.right_line_points[1], ego_lane.right_line_points[idx]);
                else if (idx_right < ego_lane.right_line_points.size()) offset = PointLineDistance(ego_lane.right_line_points[idx_right - 1], ego_lane.right_line_points[idx_right], egolane_Points[idx]);
                else offset = PointLineDistance(ego_lane.right_line_points[idx_right - 1], ego_lane.right_line_points[idx_right], ego_lane.right_line_points[idx]);
            } else {
                auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx - 1].x, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
                int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

                if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx - 1]);
                else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
                else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
            }
        }
    } else if (offset_dir == 1) {
        //实时计算车道宽度
        auto it = std::lower_bound(ego_lane.right_line_points.begin(), ego_lane.right_line_points.end(), 0.0, 
                                [](const EFMPoint& point, double x) { return point.x < x; });
        int idx = std::distance(ego_lane.right_line_points.begin(), it);

        if (idx < ego_lane.right_line_points.size()) {
            auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx].x, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

            if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx]);
            else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx]);
            else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx]);
        } else {
            auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx - 1].x, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

            if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx - 1]);
            else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
            else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
        }
        egolane_Points.clear();
        egolane_Points = offsetBevLine(ego_lane.right_line_points, -offset/2.0);
        if (merge_s_end > merge_s_end_old || merge_s_end < 1e-6f) offset_flag = true;
        if (offset_flag) offset_count++;
        if (offset_count > 60) {
            offset_dir = 0;
            offset_flag = false;
            offset_count = 0;
        }
    } else if (offset_dir == 2) {
        auto it = std::lower_bound(ego_lane.right_line_points.begin(), ego_lane.right_line_points.end(), 0.0, 
                                [](const EFMPoint& point, double x) { return point.x < x; });
        int idx = std::distance(ego_lane.right_line_points.begin(), it);

        if (idx < ego_lane.right_line_points.size()) {
            auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx].x, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

            if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx]);
            else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx]);
            else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx]);
        } else {
            auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx - 1].x, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

            if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx - 1]);
            else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
            else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
        }

        egolane_Points.clear();
        egolane_Points = offsetBevLine(ego_lane.left_line_points, offset/2.0);
        if (merge_s_end > merge_s_end_old || merge_s_end < 1e-6f) offset_flag = true;
        if (offset_flag) offset_count++;
        if (offset_count > 60) {
            offset_dir = 0;
            offset_flag = false;
            offset_count = 0;
        }
    } else if (offset_dir == 3) {
        auto it = std::lower_bound(ego_lane.right_line_points.begin(), ego_lane.right_line_points.end(), 0.0, 
                                [](const EFMPoint& point, double x) { return point.x < x; });
        int idx = std::distance(ego_lane.right_line_points.begin(), it);

        if (idx < ego_lane.right_line_points.size()) {
            auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx].x, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

            if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx]);
            else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx]);
            else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx]);
        } else {
            auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx - 1].x, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

            if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx - 1]);
            else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
            else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
        }

        egolane_Points.clear();
        egolane_Points = offsetBevLine(ego_lane.right_line_points, -offset/2.0);
        if (split_s_end > split_s_end_old || split_s_end < 1e-6f) offset_flag = true;
        if (offset_flag) offset_count++;
        if (offset_count > 60) {
            offset_dir = 0;
            offset_flag = false;
            offset_count = 0;
        }
    } else if (offset_dir == 4) {
        auto it = std::lower_bound(ego_lane.right_line_points.begin(), ego_lane.right_line_points.end(), 0.0, 
                                [](const EFMPoint& point, double x) { return point.x < x; });
        int idx = std::distance(ego_lane.right_line_points.begin(), it);

        if (idx < ego_lane.right_line_points.size()) {
            auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx].x, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

            if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx]);
            else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx]);
            else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx]);
        } else {
            auto it_left = std::lower_bound(ego_lane.left_line_points.begin(), ego_lane.left_line_points.end(), ego_lane.right_line_points[idx - 1].x, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
            int idx_left = std::distance(ego_lane.left_line_points.begin(), it_left);

            if (idx_left == 0) offset = PointLineDistance(ego_lane.left_line_points[0], ego_lane.left_line_points[1], ego_lane.right_line_points[idx - 1]);
            else if (idx_left < ego_lane.left_line_points.size()) offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
            else offset = PointLineDistance(ego_lane.left_line_points[idx_left - 1], ego_lane.left_line_points[idx_left], ego_lane.right_line_points[idx - 1]);
        }

        egolane_Points.clear();
        egolane_Points = offsetBevLine(ego_lane.left_line_points, offset/2.0);
        if (split_s_end > split_s_end_old || split_s_end < 1e-6f) offset_flag = true;
        if (offset_flag) offset_count++;
        if (offset_count > 60) {
            offset_dir = 0;
            offset_flag = false;
            offset_count = 0;
        }
    }
    // ZTEXT("EFM_INFO", "offset_dir: ", 120, -55, "offset_dir: {}", offset_dir);
    // ZTEXT("EFM_INFO", "offset: ", 120, -60, "offset: {}", offset);
    merge_s_end_old = merge_s_end;
    split_s_end_old = split_s_end;
}

EFMPoint LocalMap::LinearInterp(EFMPoint p1, EFMPoint p2, double x) {
  if (std::fabs(p2.x - p1.x) < 1e-6) return p1;
  return EFMPoint(x, (p1.y + (p2.y - p1.y) / (p2.x - p1.x) * (x - p1.x)));
}

void LocalMap::IsWideLane(const EFMRefLinePoints& leftline, const EFMRefLinePoints& rightline, int change_lane_flag, bool& is_wide_lane, int& offset_dir) {
    if (leftline[0].x > 0.0 || rightline[0].x > 0.0) {
        //左右车道线起点至少有一侧在自车前方，则需找到对应的点插值
        if (leftline[0].x < rightline[0].x) {
            auto it = std::lower_bound(leftline.begin(), leftline.end(), rightline[0].x, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
            int idx = std::distance(leftline.begin(), it);
            if (idx == 0) LOG_ERROR("std::distance Out of range");
            EFMPoint left_line_point;
            if (idx < leftline.size()) left_line_point = LinearInterp(leftline[idx - 1], leftline[idx], rightline[0].x);
            else left_line_point = LinearInterp(leftline[idx - 2], leftline[idx - 1], rightline[0].x);

            //找到两侧均距离自车最近的点判断是否超宽
            if (fabs(rightline[0].y - left_line_point.y) > 5.0) is_wide_lane = true;
            else if (fabs(rightline[0].y - left_line_point.y) < 4.5) is_wide_lane = false;

            if (is_wide_lane && offset_dir == 0) {
                if (change_lane_flag == 1) offset_dir = 1;
                else if (change_lane_flag == 2) offset_dir = 2;
                else if (fabs(rightline[0].y) < fabs(left_line_point.y)) offset_dir = 2;
                else offset_dir = 1;
            } else if (!is_wide_lane) offset_dir = 0;
        } else if (leftline[0].x > rightline[0].x) {
            auto it = std::lower_bound(rightline.begin(), rightline.end(), leftline[0].x, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
            int idx = std::distance(rightline.begin(), it);
            if (idx == 0) LOG_ERROR("std::distance Out of range");
            EFMPoint right_line_point;
            if (idx < rightline.size()) right_line_point = LinearInterp(rightline[idx - 1], rightline[idx], leftline[0].x);
            else right_line_point = LinearInterp(rightline[idx - 2], rightline[idx - 1], leftline[0].x);

            //找到两侧均距离自车最近的点判断是否超宽
            if (fabs(leftline[0].y - right_line_point.y) > 5.0) is_wide_lane = true;
            else if (fabs(leftline[0].y - right_line_point.y) < 4.5) is_wide_lane = false;

            if (is_wide_lane && offset_dir == 0) {
                if (change_lane_flag == 1) offset_dir = 1;
                else if (change_lane_flag == 2) offset_dir = 2;
                else if (fabs(leftline[0].y) < fabs(right_line_point.y)) offset_dir = 1;
                else offset_dir = 2;
            } else if (!is_wide_lane) offset_dir = 0;
        } else {
            //找到两侧均距离自车最近的点判断是否超宽
            if (fabs(leftline[0].y - rightline[0].y) > 5.0) is_wide_lane = true;
            else if (fabs(leftline[0].y - rightline[0].y) < 4.5) is_wide_lane = false;

            if (is_wide_lane && offset_dir == 0) {
                if (change_lane_flag == 1) offset_dir = 1;
                else if (change_lane_flag == 2) offset_dir = 2;
                else if (fabs(leftline[0].y) < fabs(rightline[0].y)) offset_dir = 1;
                else offset_dir = 2;
            } else if (!is_wide_lane) offset_dir = 0;
        }
    } else {
        //左右车道线起点均在自车后方，则需找到左右车道线x为0的点插值
        auto itl = std::lower_bound(leftline.begin(), leftline.end(), 0.0, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
        int left_index = std::distance(leftline.begin(), itl);

        auto itr = std::lower_bound(rightline.begin(), rightline.end(), 0.0, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
        int right_index = std::distance(rightline.begin(), itr);

        EFMPoint left_line_point;
        if (left_index == 0) left_line_point = LinearInterp(leftline[0], leftline[1], 0.0);
        else if (left_index < leftline.size()) left_line_point = LinearInterp(leftline[left_index - 1], leftline[left_index], 0.0);
        else left_line_point = LinearInterp(leftline[left_index - 2], leftline[left_index - 1], 0.0);

        EFMPoint right_line_point;
        if (right_index == 0) right_line_point = LinearInterp(rightline[0], rightline[1], 0.0);
        else if (right_index < leftline.size()) right_line_point = LinearInterp(rightline[right_index - 1], rightline[right_index], 0.0);
        else right_line_point = LinearInterp(rightline[right_index - 2], rightline[right_index - 1], 0.0);

        //找到两侧均距离自车最近的点判断是否超宽
        if (fabs(left_line_point.y - right_line_point.y) > 5.0) is_wide_lane = true;
        else if (fabs(left_line_point.y - right_line_point.y) < 4.5) is_wide_lane = false;

        if (is_wide_lane && offset_dir == 0) {
            if (change_lane_flag == 1) offset_dir = 1;
            else if (change_lane_flag == 2) offset_dir = 2;
            else if (fabs(left_line_point.y) < fabs(right_line_point.y)) offset_dir = 1;
            else offset_dir = 2;
        } else if (!is_wide_lane) offset_dir = 0;
    }
}

void LocalMap::CenterLineStitch(const EFMRefLinePoints& leftline, const EFMRefLinePoints& rightline, EFMRefLinePoints& Centerline) {
    //把较长的边线，偏移拼接到中心线
    double offset = 0.0;
    if (leftline[0].x < rightline[0].x) {
        auto it_front = std::lower_bound(leftline.begin(), leftline.end(), Centerline[0].x, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
        int idx_front = std::distance(leftline.begin(), it_front);

        if (idx_front == 0) {

        } else if (idx_front < leftline.size()) {
            offset = PointLineDistance(leftline[idx_front - 1], leftline[idx_front], Centerline[0]);
            EFMRefLinePoints Centerline_front;
            Centerline_front = offsetBevLine(leftline, offset);
            auto start = Centerline_front.begin();
            auto end = Centerline_front.begin() + idx_front;
            Centerline.insert(Centerline.begin(), start, end);
        } else {
            offset = PointLineDistance(leftline[idx_front - 2], leftline[idx_front - 1], Centerline[0]);
            EFMRefLinePoints Centerline_front;
            Centerline_front = offsetBevLine(leftline, offset);
            auto start = Centerline_front.begin();
            auto end = Centerline_front.begin() + idx_front;
            Centerline.insert(Centerline.begin(), start, end);
        }
    } else if (leftline[0].x > rightline[0].x) {
        auto it_front = std::lower_bound(rightline.begin(), rightline.end(), Centerline[0].x, 
                                        [](const EFMPoint& point, double x) { return point.x < x; });
        int idx_front = std::distance(rightline.begin(), it_front);

        if (idx_front == 0) {

        } else if (idx_front < rightline.size()) {
            offset = PointLineDistance(rightline[idx_front - 1], rightline[idx_front], Centerline[0]);
            EFMRefLinePoints Centerline_front;
            Centerline_front = offsetBevLine(rightline, offset);
            auto start = Centerline_front.begin();
            auto end = Centerline_front.begin() + idx_front;
            Centerline.insert(Centerline.begin(), start, end);
        } else {
            offset = PointLineDistance(rightline[idx_front - 2], rightline[idx_front - 1], Centerline[0]);
            EFMRefLinePoints Centerline_front;
            Centerline_front = offsetBevLine(rightline, offset);
            auto start = Centerline_front.begin();
            auto end = Centerline_front.begin() + idx_front;
            Centerline.insert(Centerline.begin(), start, end);
        }
    }

    if (leftline[leftline.size() - 1].x > rightline[rightline.size() - 1].x) {
        auto it_rear = std::lower_bound(leftline.begin(), leftline.end(), Centerline[Centerline.size() - 1].x, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
        int idx_rear = std::distance(leftline.begin(), it_rear);

        if (idx_rear == 0) {
            offset = PointLineDistance(leftline[0], leftline[1], Centerline[Centerline.size() - 1]);
            EFMRefLinePoints Centerline_rear;
            Centerline_rear = offsetBevLine(leftline, offset);
            auto start = Centerline_rear.begin() + idx_rear + 1;
            auto end = Centerline_rear.end();
            Centerline.insert(Centerline.end(), start, end);
        } else if (idx_rear < leftline.size()) {
            offset = PointLineDistance(leftline[idx_rear - 1], leftline[idx_rear], Centerline[Centerline.size() - 1]);
            EFMRefLinePoints Centerline_rear;
            Centerline_rear = offsetBevLine(leftline, offset);
            auto start = Centerline_rear.begin() + ((Centerline_rear[idx_rear].x - Centerline[Centerline.size() - 1].x) > 1.25 ? idx_rear :
                                (idx_rear + 1) < Centerline_rear.size() ? (idx_rear + 1) : idx_rear);
            auto end = Centerline_rear.end();
            Centerline.insert(Centerline.end(), start, end);
        } else {
            offset = PointLineDistance(leftline[idx_rear - 2], leftline[idx_rear - 1], Centerline[Centerline.size() - 1]);
            EFMRefLinePoints Centerline_rear;
            Centerline_rear = offsetBevLine(leftline, offset);
            auto start = Centerline_rear.begin() + idx_rear;
            auto end = Centerline_rear.end();
            Centerline.insert(Centerline.end(), start, end);
        }
    } else if (leftline[leftline.size() - 1].x < rightline[rightline.size() - 1].x) {
        auto it_rear = std::lower_bound(rightline.begin(), rightline.end(), Centerline[Centerline.size() - 1].x, 
                                    [](const EFMPoint& point, double x) { return point.x < x; });
        int idx_rear = std::distance(rightline.begin(), it_rear);

        if (idx_rear == 0) {
            offset = PointLineDistance(rightline[0], rightline[1], Centerline[Centerline.size() - 1]);
            EFMRefLinePoints Centerline_rear;
            Centerline_rear = offsetBevLine(rightline, offset);
            auto start = Centerline_rear.begin() + idx_rear + 1;
            auto end = Centerline_rear.end();
            Centerline.insert(Centerline.end(), start, end);
        } else if (idx_rear < rightline.size()) {
            offset = PointLineDistance(rightline[idx_rear - 1], rightline[idx_rear], Centerline[Centerline.size() - 1]);
            EFMRefLinePoints Centerline_rear;
            Centerline_rear = offsetBevLine(rightline, offset);
            auto start = Centerline_rear.begin() + ((Centerline_rear[idx_rear].x - Centerline[Centerline.size() - 1].x) > 1.25 ? idx_rear :
                                (idx_rear + 1) < Centerline_rear.size() ? (idx_rear + 1) : idx_rear);
            auto end = Centerline_rear.end();
            Centerline.insert(Centerline.end(), start, end);
        } else {
            offset = PointLineDistance(rightline[idx_rear - 2], rightline[idx_rear - 1], Centerline[Centerline.size() - 1]);
            EFMRefLinePoints Centerline_rear;
            Centerline_rear = offsetBevLine(rightline, offset);
            auto start = Centerline_rear.begin() + idx_rear;
            auto end = Centerline_rear.end();
            Centerline.insert(Centerline.end(), start, end);
        }
    }
}

EFMPoint LocalMap::interpolateAtLength(const EFMRefLinePoints& line, const std::vector<double>& arc_lengths,
                                          double target_length) {
    if (arc_lengths.size() <= 1){
        return EFMPoint{0.0, 0.0};
    }
    
    // 根据弧长进行重采样线性插值
    auto it = std::lower_bound(arc_lengths.begin(), arc_lengths.end(), target_length);
    size_t idx = std::distance(arc_lengths.begin(), it) - 1;

    double seg_start = 0.0;
    double seg_end = 0.0;
    if (idx == -1) {
        return line[0];
    } else if (idx == arc_lengths.size() - 1) {
        seg_start = arc_lengths[idx - 1];
        seg_end = arc_lengths[idx];
    } else {
        seg_start = arc_lengths[idx];
        seg_end = arc_lengths[idx + 1];
    }

    double seg_tmp = seg_end - seg_start;
    if (seg_tmp < 1e-6) {
        return line[idx];
    }

    double t = (target_length - seg_start) / seg_tmp;
    return line[idx] * (1.0 - t) + line[idx + 1] * t;
}

double LocalMap::PointLineDistance(EFMPoint p1, EFMPoint p2, EFMPoint p3) {
  double line_length = std::hypot(p1.x - p2.x, p1.y - p2.y);
  if (line_length < 1e-6) {
    return std::hypot(p1.x - p3.x, p1.y - p3.y);
  }

  double distance = ((p2 - p1).CrossProduct(p3 - p1)) / line_length;
  return distance;
}

bool LocalMap::SmoothCenterLineForLastCycle(SEhpOutputLoc last_cycle_loc, 
                                            std::deque<std::pair<int, EFMRefLinePoints>> last_cycle_center_lines,
                                            SEhpOutputLoc& curr_cycle_loc, 
                                            std::deque<std::pair<int, EFMRefLinePoints>>& curr_cycle_center_lines) {
    // 平滑处理上一帧的中心线
    if (last_cycle_center_lines.size() == 0 || curr_cycle_center_lines.size() == 0) {
        return true;
    }
    
    // 找到上一帧和当前帧的匹配中心线
    for (size_t i = 0; i < curr_cycle_center_lines.size(); i++){
        int curr_lane_id = curr_cycle_center_lines[i].first;
        auto it = std::find_if(last_cycle_center_lines.begin(), last_cycle_center_lines.end(),
                               [curr_lane_id](const std::pair<int, EFMRefLinePoints>& lane_pair) {
                                   return lane_pair.first == curr_lane_id;
                               });
        if (it != last_cycle_center_lines.end()){
            // 找到匹配的中心线，进行平滑处理
            EFMRefLinePoints& last_center_line = it->second;
            EFMRefLinePoints& curr_center_line = curr_cycle_center_lines[i].second;

            // 计算弧长
            // std::vector<double> last_arc_lengths;
            // std::vector<double> curr_arc_lengths;
            // double last_length = 0.0;
            // double curr_length = 0.0;
            // last_arc_lengths.push_back(0.0);
            // curr_arc_lengths.push_back(0.0);
            // for (size_t j = 1; j < last_center_line.size(); j++){
            //     last_length += std::hypot(last_center_line[j].x - last_center_line[j - 1].x,
            //                               last_center_line[j].y - last_center_line[j - 1].y);
            //     last_arc_lengths.push_back(last_length);
            // }
            // for (size_t j = 1; j < curr_center_line.size(); j++){
            //     curr_length += std::hypot(curr_center_line[j].x - curr_center_line[j - 1].x,
            //                               curr_center_line[j].y - curr_center_line[j - 1].y);
            //     curr_arc_lengths.push_back(curr_length);
            // }

            // // 对当前中心线进行重采样插值
            // EFMRefLinePoints smoothed_center_line;
            // double step = 0.5; // 重采样步长
            // for (double len = 0.0; len <= curr_length; len += step){
            //     EFMPoint interp_point = interpolateAtLength(curr_center_line, curr_arc_lengths, len);
            //     smoothed_center_line.push_back(interp_point);
            // }
            curr_cycle_center_lines[i].second = curr_center_line;
        }
    }
    
    return true;
}
}
