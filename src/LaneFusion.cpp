#include "LaneFusion.h"
#include <algorithm>
#include <unordered_set>
#include <cstring> 
#include "LaneFusion.h"
#include "trans_coordinate.h"

namespace NoMapEFM{

bool LaneFusion::Execute(std::vector<BevLineInnerS>& bev_lines, const datatype_ehp::s_SDPosition_t& position ,const datatype_fusion::s_FusionLanes_t& lanes_msg) {

    // PlotOriginLine(lanes_msg);//plot BEV origin line
    // if (!bev_lines.empty()){
    //     for(auto& line: bev_lines){
    //         if (!line.total_line.empty()){
    //             std::cout<< "origin===========" << line.id << " | " << line.lane_location_type << " | " << line.total_line.front().x << " | " << line.total_line.front().y << " | " << line.total_line.back().x << " | " << line.total_line.back().y << std::endl;
    //         }    
    //     }
    // }
    // lane_fusion_(bev_lines, position, lanes_msg);
    // if (!bev_lines.empty()){
    //     for(auto& line: bev_lines){
    //         if (!line.total_line.empty() && !line.lane_fusion_process_info.LaneFusionPoints.empty()){
    //             std::cout<< "before===========" << line.id << " | " << line.lane_location_type << " | " << line.total_line.front().x << " | " << line.total_line.front().y << " | " << line.total_line.back().x << " | " << line.total_line.back().y << std::endl;
    //         }    
    //     }
    // }

    // inhibit_out_road_edge_line(bev_lines);

    // if (!bev_lines.empty()){
    //     for(auto& line: bev_lines){
    //         if (!line.total_line.empty()){

    //             std::cout<< "after===========" << line.id << " | " << line.lane_location_type << " | " << line.total_line.front().x << " | " << line.total_line.front().y << " | " << line.total_line.back().x << " | " << line.total_line.back().y << std::endl;
    //         }    
    //     }
    // }
    
    return true;
}


void LaneFusion::lane_fusion_(std::vector<BevLineInnerS>& bev_lines_, const datatype_ehp::s_SDPosition_t& position_msg, const datatype_fusion::s_FusionLanes_t& lanes_msg) {

    //预预处理：BEV的线，在有值的时候，也有可能出现total_line为空的情况，这种情况导致融合判断错误。在最前端，删除这种线。这是BST的一个bug

    if (!bev_lines_.empty()) {
        // 遍历 bev_lines_，删除 total_line 为空的线
        for (auto it = bev_lines_.begin(); it != bev_lines_.end();) {
            if (it->total_line.empty()) {
                it = bev_lines_.erase(it); // 删除当前元素，并更新迭代器
            } else {
                ++it; // 继续下一个元素
            }
        }
    }

    //预处理，将event涉及到的车道线的id全部找出来，这些线不参与融合，且用于消弭僵尸轨迹，即关联上了僵尸轨迹则将僵尸轨迹删除而不融合。
    std::vector<int> lane_event_line_id;
    lane_event_line_id.clear();

    // 遍历 FusionLanes 的 Array_LaneEventGroupCam1Vccs_5
    for (const auto& lane_event_group : lanes_msg.FusionLanes.Array_LaneEventGroupCam1Vccs_5) {
        // 检查 inner_lane_id
        if (lane_event_group.inner_lane_id != 0 && lane_event_group.inner_lane_id != -1) {
            lane_event_line_id.push_back(lane_event_group.inner_lane_id);
        }
        // 检查 outer_lane_id
        if (lane_event_group.outer_lane_id != 0 && lane_event_group.outer_lane_id != -1) {
            lane_event_line_id.push_back(lane_event_group.outer_lane_id);
        }
        // 检查 common_lane_id
        if (lane_event_group.common_lane_id != 0 && lane_event_group.common_lane_id != -1) {
            lane_event_line_id.push_back(lane_event_group.common_lane_id);
        }
    }

    CommonTool::DiscretePointsMath* discrete_points_math = CommonTool::DiscretePointsMath::GetInstance();
    CommonTool::CoordinateTool* coordinate_tool_instance = CommonTool::CoordinateTool::GetInstance();

    //标定参数
    //todo 标定参数都整合到一起
    float k_EFM_lane_fusion_cycle_time = 0.05F;//系统的运行周期

    //依据僵尸轨迹长度查表计算保持的时间
    float k_EFM_lane_fusion_zombie_delete_age_threshold_high = 5.0;//僵尸线的Age阈值，超过该值就将其删除
    float k_EFM_lane_fusion_zombie_delete_age_threshold_low = 2.0;//僵尸线的Age阈值，超过该值就将其删除
    float k_EFM_lane_fusion_zombie_delete_age_lenth_high = 60.0;//僵尸线的Age阈值，超过该值就将其删除(僵尸轨迹长度大于60m，保持5s)
    float k_EFM_lane_fusion_zombie_delete_age_lenth_low = 20.0;//僵尸线的Age阈值，超过该值就将其删除(僵尸轨迹长度小于20m，保持2s)

    float k_EFM_lane_fusion_bev_age_threshold = 0.15;//该线至少出现了多长时间，才有资格被识别为僵尸轨迹。即闪两下的线，没有资格成为僵尸轨迹，该丢弃就丢弃。
    double k_EFM_lane_fusion_x_threshold = -60.0;//僵尸线纵行距离的阈值，小于该值就将其删除
    double k_EFM_lane_fusion_x_threshold_zombie = 60;//远处的识别精度不高，起点大于60m的僵尸线不进行保持。
    // bool k_EFM_overlap_check = false;//同id线是否开启重叠检查
    bool k_EFM_overlap_check = true;//同id线是否开启重叠检查
    int k_EFM_zombie_min_size_threshold = 5;//僵尸轨迹小于10m就干掉
    bool k_EFM_delete_prev_line = true;//id虽然匹配上了，但是偏差太大，不能继续进行滤波和保持，是否将其删除（true:删除；false:变为僵尸轨迹参与接下来的处理）
    float k_EFM_lane_fusion_max_age = 10;//僵尸轨迹参与融合之后，保留其原始id的时间，超过这个时间就将该id删除
    double k_EFM_resample_distance = 2.5;//重采样的间距，单位m

    // bool k_EFM_enable_intersection_topology = false;//是否启用路口 topology

    DoublePosePoint ego_pos_wgs84;
    // ego_pos_wgs84.x = position_msg.Lon* 360.0 / (4294967296);
    // ego_pos_wgs84.y = position_msg.Lat* 360.0 / (4294967296);
    ego_pos_wgs84.x = position_msg.Lon;
    ego_pos_wgs84.y = position_msg.Lat;//c1236不需要上述转化
    ego_pos_wgs84.yaw = position_msg.Heading;

    // step 将所有感知到的线添加gps信息,只要出现，就定死在当前的gps位置上
    if (!bev_lines_.empty()) {
        for (auto& current_line : bev_lines_) {
            std::vector<Point2Dd> line_body;
            std::vector<Point2Dd> line_wgs84;

            for (const auto& point : current_line.total_line) {
                Point2Dd body_point;
                body_point.x = point.x;  // 将 current_line 中的 x 赋值给 line_body 的 x
                body_point.y = point.y;  // 将 current_line 中的 y 赋值给 line_body 的 y
                line_body.push_back(body_point);
            }
            
            coordinate_tool_instance->LineBodyToWGS84(line_body, ego_pos_wgs84, line_wgs84);

            for (const auto& point : line_wgs84) {
                GPSLinePoint body_point;
                body_point.Longitude = point.x;  // 将 current_line 中的 x 赋值给 line_body 的 x
                body_point.Latitude = point.y;  // 将 current_line 中的 y 赋值给 line_body 的 y
                current_line.lane_fusion_process_info.total_line_gps_points.push_back(body_point);
            }
            
            //将点的坐标赋值给融合要用的离散的点的坐标形式，插值处理
            LaneFusionPointsConvert(current_line);  
            //更新量测的值以及方差（只有僵尸轨迹和实时感知轨迹需要更新）
            LaneFusionMeanProcess(current_line);
            LaneFusionVarProcess(current_line);
            current_line.line_fusion_type = 0; // 0表示实时感知轨迹
        }
    }

    //step 对上一帧的轨迹进行补偿（不包含僵尸轨迹, 是最关键的，最终维护的轨迹），是为了接下来的关联和滤波（不管接下来会不会成僵尸，因为即使关联也是在转化之后的基础上进行的）   
    if (!prev_frame_active_lines_.empty()) {
        for (auto& prev_line : prev_frame_active_lines_) {

            std::vector<Point2Dd> line_body;
            std::vector<Point2Dd> line_wgs84;

            for (const auto& point : prev_line.lane_fusion_process_info.total_line_gps_points) {
                Point2Dd wgs84_point;
                wgs84_point.x = point.Longitude;  // 将 current_line 中的 x 赋值给 line_body 的 x
                wgs84_point.y = point.Latitude;  // 将 current_line 中的 y 赋值给 line_body 的 y
                line_wgs84.push_back(wgs84_point);
            }

            coordinate_tool_instance->LineWGS84ToBody(line_wgs84, ego_pos_wgs84, line_body);

            // 清空原来的点
            prev_line.total_line.clear();

            // 插入补偿后的点
            for (const auto& point : line_body) {
                EFMPoint body_point;
                body_point.x = point.x;  // 将 current_line 中的 x 赋值给 line_body 的 x
                body_point.y = point.y;  // 将 current_line 中的 y 赋值给 line_body 的 y
                prev_line.total_line.push_back(body_point);
            }
            //将点的坐标赋值给融合要用的点的坐标形式，并计算切线斜率（上个时刻的轨迹）
            LaneFusionPointsConvert(prev_line);//更新的是x y,kalman的不可以更新（理论上x y可以作为预测！！！----但不能直接赋值，可以后面通过方差的计算来影响）
            LaneFusionMeanProcess(prev_line);//更新量测的值（就是将坐标点转化一下，但是方差不能动）
           
            prev_line.lane_fusion_process_info.bev_line_age += k_EFM_lane_fusion_cycle_time; // 增加pre维持的时间
            prev_line.lane_fusion_process_info.bev_line_age = fmaxf(0, fminf(prev_line.lane_fusion_process_info.bev_line_age, 100));// 对 bev_line_age 进行限幅

            if (prev_line.line_fusion_type == 1 && !prev_line.fused_line_sts.empty()) {  // 确保条件判断为比较操作
                for (auto& fused_sts : prev_line.fused_line_sts) {
                    if (fused_sts.line_fused_age <= k_EFM_lane_fusion_max_age) {
                        // 不满足删除条件，累加时间
                        fused_sts.line_fused_age += k_EFM_lane_fusion_cycle_time;
                        fused_sts.line_fused_age = fmaxf(0, fminf(fused_sts.line_fused_age, 100));
                    }
                }

                // 删除满足条件的元素
                for (auto it = prev_line.fused_line_sts.begin(); it != prev_line.fused_line_sts.end(); ) {
                    if (it->line_fused_age > k_EFM_lane_fusion_max_age) {
                        it = prev_line.fused_line_sts.erase(it);  // 删除满足条件的元素
                    } else {
                        ++it;  // 仅在未删除时递增迭代器
                    }
                }
            }
        }  
    }

    //  step 对上一时刻的僵尸轨迹进行补偿
    if (!data_base_zombie_lines_.empty()) {
        for (auto& zombie_line : data_base_zombie_lines_) {

            std::vector<Point2Dd> line_body;
            std::vector<Point2Dd> line_wgs84;

            for (const auto& point : zombie_line.lane_fusion_process_info.total_line_gps_points) {
                Point2Dd wgs84_point;
                wgs84_point.x = point.Longitude;  // 将 current_line 中的 x 赋值给 line_body 的 x
                wgs84_point.y = point.Latitude;  // 将 current_line 中的 y 赋值给 line_body 的 y
                line_wgs84.push_back(wgs84_point);
            }

            coordinate_tool_instance->LineWGS84ToBody(line_wgs84, ego_pos_wgs84, line_body);

            // 清空原来的点
            zombie_line.total_line.clear();

            // 插入补偿后的点
            for (const auto& point : line_body) {
                EFMPoint body_point;
                body_point.x = point.x;  // 将 current_line 中的 x 赋值给 line_body 的 x
                body_point.y = point.y;  // 将 current_line 中的 y 赋值给 line_body 的 y
                zombie_line.total_line.push_back(body_point);
            }
            //将点的坐标赋值给融合要用的点的坐标形式，并计算切线斜率（僵尸轨迹）
            LaneFusionPointsConvert(zombie_line);
            //更新量测的值（只有僵尸轨迹和实时感知轨迹需要更新）
            LaneFusionMeanProcess(zombie_line);
            LaneFusionVarProcess(zombie_line);

            //计算僵尸轨迹的长度（或在死的时候计算一次就行，不用每次计算）
            if(!zombie_line.total_line.empty() && zombie_line.lane_fusion_process_info.zombie_line_total_length < 0.1) {
                for(int i =1;i<zombie_line.total_line.size();i++){
                    double dist = sqrt(pow(zombie_line.total_line[i].x-zombie_line.total_line[i-1].x,2)+ pow(zombie_line.total_line[i].y-zombie_line.total_line[i-1].y,2));
                    zombie_line.lane_fusion_process_info.zombie_line_total_length+= dist;
                }
            }
        }  
    }

    // step  寻找新的僵尸轨迹（id以及是否有重合的部分），该轨迹（注意：要分配id，在分配之前要找到未被使用的id：10000 开始；要将融合属性给赋值）
    // 是否要添加一个虽然id相同，但是偏差太大的情况？（可共用车道线关联逻辑）--Done LaneFusionCurrentLineAssociation 函数实现
    // 连续多帧丢失（也不能太多帧）再判定为僵尸轨迹，否则先保持一下--Done
    // 僵尸轨迹的原始id保存一下（我死之前是谁，记住我的名字），融合判断后输出，下游要用--Done
    //todo 僵尸轨迹的保持时间与长度也关联一下，使短的尽快消失。或者干脆直接干掉

    //debug↓
    // if (!prev_frame_active_lines_.empty()) {

    //     for (auto& prev_line : prev_frame_active_lines_) {
    //         std::cout << "prev_line.id_raw" << prev_line.id <<std::endl;
    //         std::cout << "prev_line.bev_line_age" << prev_line.lane_fusion_process_info.bev_line_age <<std::endl;
    //     }
    // }

    // if (!bev_lines_.empty()) {

    //     for (auto& bev_line : bev_lines_) {
    //         std::cout << "bev_line.id_raw" << bev_line.id <<std::endl;
    //         std::cout << "bev_line.bev_line_age" << bev_line.lane_fusion_process_info.bev_line_age <<std::endl;
    //     }
    // }
    //debug↑

    if (!prev_frame_active_lines_.empty() && !bev_lines_.empty()) {

        std::vector<int> to_delete_ids; // 用于存储待删除的 prev_line.id
        for (auto& prev_line : prev_frame_active_lines_) {
            // std::cout << "==prev_line.id==: " << prev_line.id << std::endl;
            // std::cout << "prev_line.lane_fusion_process_info.bev_line_age: " << prev_line.lane_fusion_process_info.bev_line_age << std::endl;

            bool line_be_refreshed = false;
            bool line_be_deleted = false;
            for (const auto& current_line : bev_lines_) {
                if (prev_line.id == current_line.id) {
                    bool line_overlap_ = false;
                    bool Associated_ = false;

                    if (k_EFM_overlap_check) {

                        if (!current_line.total_line.empty() && !prev_line.total_line.empty()) {

                            if(current_line.total_line.size() >= 2 && prev_line.total_line.size() >= 2){

                                Associated_ = LaneFusionCurrentLineAssociation(prev_line, current_line, lane_event_line_id);
                            }else{
                                Associated_ = false;
                            }
                            // std::cout << "prev_line.id==: " << prev_line.id << std::endl;
                            // std::cout << "Associated_: " << Associated_ << std::endl;

                            if (Associated_) {
                                line_overlap_ = true;
                            } else if (!Associated_ && k_EFM_delete_prev_line) {
                                to_delete_ids.push_back(prev_line.id); // 标记待删除的 id
                                line_be_deleted = true;
                            }
                        }
                    } else {
                        line_overlap_ = true;
                    }

                    if (line_overlap_) {
                        line_be_refreshed = true;
                    }
                    break;
                }
            }

            if (!line_be_refreshed) { // pre轨迹是没有被更新的

                if (line_be_deleted == false) { // 已经因为前后帧跳变太大被删除了，就不需要再进行僵尸轨迹的处理了

                    if (prev_line.lane_fusion_process_info.bev_line_age > k_EFM_lane_fusion_bev_age_threshold) { // pre没有被更新，且之前是存在了

                        prev_line.lane_fusion_process_info.zombie_line_origin_id = prev_line.id; // 保存原始id,融合后将其发出

                        std::unordered_set<int> used_ids;
                        used_ids.clear();
                        for (const auto& zombie_line : data_base_zombie_lines_) {
                            used_ids.insert(zombie_line.id);
                        }

                        bool found_unused_id = false;
                        for (int i = 10000; i < 20000; ++i) {
                            if (used_ids.find(i) == used_ids.end()) {
                                prev_line.id = i;
                                found_unused_id = true;
                                break;
                            }
                        }

                        if (!found_unused_id) {
                            prev_line.id = -1;
                        }

                        prev_line.line_fusion_type = 2;
                        data_base_zombie_lines_.push_back(prev_line);
                        to_delete_ids.push_back(prev_line.id); // 标记待删除的 id
                    } else { // 没有被更新，且存在的时间比较短，且没有因为跳变太大导致的，那就将其直接删除
                        to_delete_ids.push_back(prev_line.id); // 标记待删除的 id
                    }
                }
            }
        }

        // 遍历结束后统一删除待删除的元素
        prev_frame_active_lines_.erase(
            std::remove_if(
                prev_frame_active_lines_.begin(),
                prev_frame_active_lines_.end(),
                [&to_delete_ids](const auto& line) {
                    return std::find(to_delete_ids.begin(), to_delete_ids.end(), line.id) != to_delete_ids.end();
                }
            ),
            prev_frame_active_lines_.end()
        );
    }

    // step  Age累加，删除不需要的僵尸线（Age 或距离）
    //todo 依据线长度，自适应调整Age阈值
    if (!data_base_zombie_lines_.empty()) {
        for (auto it = data_base_zombie_lines_.begin(); it != data_base_zombie_lines_.end();) {
            //作为僵尸的时间
            it->lane_fusion_process_info.zombie_line_age += k_EFM_lane_fusion_cycle_time;  // 增加 Age
            it->lane_fusion_process_info.zombie_line_age = fmaxf(0.0, fminf(it->lane_fusion_process_info.zombie_line_age + k_EFM_lane_fusion_cycle_time, 100.0));//限幅
            //作为僵尸也要更新自古以来的时间,否则容易被出现几帧但是关联上了的小鲜肉带没
            it->lane_fusion_process_info.bev_line_age += k_EFM_lane_fusion_cycle_time;  // 增加 Age
            it->lane_fusion_process_info.bev_line_age = fmaxf(0.0, fminf(it->lane_fusion_process_info.bev_line_age + k_EFM_lane_fusion_cycle_time, 100.0));//限幅

            // 添加删除条件：如果 Age 超过阈值或最后一个点的 x 坐标小于阈值，则删除该僵尸线

            // 由僵尸轨迹的长度线性插值计算Age的阈值
            float x = it->lane_fusion_process_info.zombie_line_total_length;
            float x_low = k_EFM_lane_fusion_zombie_delete_age_lenth_low;
            float x_high = k_EFM_lane_fusion_zombie_delete_age_lenth_high;
            float y_low = k_EFM_lane_fusion_zombie_delete_age_threshold_low;
            float y_high = k_EFM_lane_fusion_zombie_delete_age_threshold_high;
            float y;
            if (x <= x_low) {
                y = y_low;
            } else if (x >= x_high) {
                y = y_high;
            } else {
                y = y_low + (x - x_low) / (x_high - x_low) * (y_high - y_low);
            }

            //针对路口拓扑的僵尸轨迹，要保持的更久一点
            if (it->id == 20003 || it->id == 20004){
                y = 500;//即过路口的时候，僵尸轨迹至少保持的时间，其实就是靠60m来干掉了。
            }

            //条件判断
            if (it->lane_fusion_process_info.zombie_line_age > y ||
                (it->total_line.size() > 0 && it->total_line.back().x < k_EFM_lane_fusion_x_threshold) || (it->total_line.size() < k_EFM_zombie_min_size_threshold) || (it->total_line.front().x > k_EFM_lane_fusion_x_threshold_zombie)) {
                it = data_base_zombie_lines_.erase(it);  // 删除当前元素，并更新迭代器
            } else {
                ++it;  // 继续下一个元素
            }
            //不可以用it打印，注意！
        }
    }

    LaneFusionLineFusion(bev_lines_, data_base_zombie_lines_, prev_frame_active_lines_, ego_pos_wgs84, lane_event_line_id);//此时输入全部是时间同步后的。僵尸轨迹与之前轨迹关联；持续不变的轨迹也与上一时刻关联。
    
   //在此之前要将输出的 bev_lines_ 处理完毕（关联及滤波之后的，不包含僵尸轨迹）
    prev_frame_active_lines_.clear();//上一帧活的线：即除了僵尸线之外的线，关联、滤波之后的线，保存进来，用于下一帧的僵尸线关联和滤波（把僵尸轨迹减除，把新出现的轨迹加入，kalman模型初始化或更新）

    if (!bev_lines_.empty()) {

        prev_frame_active_lines_.insert(prev_frame_active_lines_.end(), bev_lines_.begin(), bev_lines_.end());
    }

    if (!data_base_zombie_lines_.empty()) {

        bev_lines_.insert(bev_lines_.end(), data_base_zombie_lines_.begin(), data_base_zombie_lines_.end());
    }

    //路口拓扑=============================================================================================================
    if (1 == k_EFM_enable_intersection_topology){
        intersection_topology_(bev_lines_, ego_pos_wgs84);
    }

    //resample & refresh overlap index
    if (!bev_lines_.empty()) {

        for (auto& bev_line : bev_lines_) {
            // std::cout << "sample_bev_line.id: " << bev_line.id <<std::endl;
            // std::cout << "sample_bev_line.start_overlap_point.x: " << bev_line.line_overlap_points.start_overlap_point.x <<std::endl;
            // std::cout << "sample_bev_line.end_overlap_point.x: " << bev_line.line_overlap_points.end_overlap_point.x <<std::endl;
            int start_index = -1, end_index = -1;
            EFMRefLinePoints fixed_reference_line;
            bool resample_success = discrete_points_math -> FixPathDensity(bev_line.total_line, k_EFM_resample_distance, fixed_reference_line);
            // std::cout << "sample_resample_success: " << resample_success <<std::endl;
            if (resample_success){
                bev_line.total_line = fixed_reference_line;
                // std::cout << "i am here: " <<std::endl;
                //refresh overlap index
                // std::cout << "sample_bev_line.line_overlap_points.is_valid: " << bev_line.line_overlap_points.is_valid <<std::endl;
                // std::cout << "sample_bev_line.total_line.size(): " << bev_line.total_line.size() <<std::endl;
                if (bev_line.line_overlap_points.is_valid && bev_line.total_line.size() > 1) {
                    

                    // 遍历 total_line 找到 start_overlap_point.x 所在的区间
                    for (size_t i = 1; i < bev_line.total_line.size(); ++i) {
                        // 找到 start_overlap_point.x 落在两个点区间内
                        if (bev_line.total_line[0].x >= bev_line.line_overlap_points.start_overlap_point.x){

                            start_index = 0;
                            break;
                        }else if (bev_line.total_line[i - 1].x < bev_line.line_overlap_points.start_overlap_point.x && bev_line.line_overlap_points.start_overlap_point.x <= bev_line.total_line[i].x) {

                            start_index = i;
                            break;
                        }
                    }

                    // 遍历 total_line 找到 end_overlap_point.x 所在的区间
                    for (size_t i = 1; i < bev_line.total_line.size(); ++i) {
                        // 找到 end_overlap_point.x 落在两个点区间内
                        if (bev_line.line_overlap_points.end_overlap_point.x >= bev_line.total_line.back().x){

                            end_index = bev_line.total_line.size()-1;
                            break;
                        }else if (bev_line.total_line[i - 1].x <= bev_line.line_overlap_points.end_overlap_point.x && bev_line.line_overlap_points.end_overlap_point.x < bev_line.total_line[i].x) {

                            end_index = i-1;
                            break;
                        }
                    }
                }
            }
            if(start_index == -1 || end_index == -1){
                bev_line.line_overlap_points.is_valid = false;
            }else{
                bev_line.line_overlap_points.start_overlap_point.index = start_index;
                bev_line.line_overlap_points.end_overlap_point.index = end_index;
            }
             
            // std::cout << "sample_bev_line.start_overlap_point.index: " << bev_line.line_overlap_points.start_overlap_point.index <<std::endl;
            // std::cout << "sample_bev_line.send_overlap_point.index: " << bev_line.line_overlap_points.end_overlap_point.index <<std::endl;
        }
    }

    //是否抑制路沿外侧的车道线
    inhibit_out_road_edge_line(bev_lines_);
}

//车道线关联(只要BEV新数据没有被关联上，就将其加入僵尸轨迹，在本周期就进行关联，所以输入不需要BEV的实时输入轨迹
bool LaneFusion::LaneFusionZombieLineAssociation(std::vector<BevLineInnerS>& data_base_Associated_lines_, std::vector<BevLineInnerS>& data_base_zombie_lines_, const BevLineInnerS& prev_line_) {
    // 车道线关联  关联上了要删除僵尸轨迹？--是否要删除？还是最好要一直存在？--删除  
    //todo 将僵尸轨迹维持的时间考虑进来进行关联阈值的标定（时间越久，越允许将阈值放大一点，于此同时要把其方差放大，这样对融合后的结果影响较小。要不要考虑自车运动状态即换道时放大？）
    //todo 有可能多条轨迹与某一僵尸轨迹是可以关联上的，但是现在会是先到先得，是不是不太好？需要遍历完所有pre吗？可能是需要大改一下。
    float k_EFM_lane_fusion_zombie_associated_average_distance_threshold = 0.5F;//关联的阈值(要根据点数来计算平均的)（可根据Type是否一致来进行自适应调整）
    float k_EFM_lane_fusion_zombie_associated_single_distance_threshold = 1.0F;// 单个点的距离阈值
    int k_EFM_lane_fusion_zombie_associated_overlap_num_threshold = 5; //关联的重合点数量阈值（可根据Type是否一致来进行自适应调整）5个点是4段，2.5m一段就是10m
    // std::cout << "prev_line_.id: " << prev_line_.id << std::endl;
    bool is_associated = false;
    data_base_Associated_lines_.clear(); // 清空关联的线    

    if (!data_base_zombie_lines_.empty()){
        // std::cout << "data_base_zombie_lines_.size()" << data_base_zombie_lines_.size() << std::endl;

        for (auto it = data_base_zombie_lines_.begin(); it != data_base_zombie_lines_.end(); ) {

            float total_distance_ = 0.0f; // 累计距离
            int overlap_number_ = 0; // 重合点数量

            for (int i = 0; i < prev_line_.lane_fusion_process_info.LaneFusionPoints.size(); ++i) {

                if ((prev_line_.lane_fusion_process_info.LaneFusionPoints[i].valid == true) &&
                    (it->lane_fusion_process_info.LaneFusionPoints[i].valid == true)) {

                    float single_distance_ = fabsf(prev_line_.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32 - it->lane_fusion_process_info.LaneFusionPoints[i].y);

                    if (single_distance_ > k_EFM_lane_fusion_zombie_associated_single_distance_threshold) {

                        overlap_number_ = 0;
                        total_distance_ = 100;
                        break; // 如果单个点的距离超过阈值，则认为没有关联上，直接跳出循环
                    }
                    overlap_number_ += 1; // 有重合点
                    total_distance_ += single_distance_; // 计算重合点的距离
                }
            }
            float average_distance_ = 100.0f; // 平均距离初始化
            if(overlap_number_ > 0){

                average_distance_ = total_distance_ / overlap_number_; // 计算平均距离
            }

            if (overlap_number_ >= k_EFM_lane_fusion_zombie_associated_overlap_num_threshold &&
                average_distance_ <= k_EFM_lane_fusion_zombie_associated_average_distance_threshold) {

                data_base_Associated_lines_.push_back(*it); // 将关联的线添加到结果中
                // 删除当前 zombie_line
                it = data_base_zombie_lines_.erase(it); // 安全删除元素并更新迭代器
            } else {
                ++it; // 如果不删除，正常推进迭代器
            }
        }   
        if (!data_base_Associated_lines_.empty()){

            is_associated = true;
        }
    }
    return is_associated;
}

//当前轨迹和已有的pre的轨迹是否有较大的偏差（id虽然相同），如果较大，则直接将pre的删除，完全相信新过来的轨迹，即reset机制
bool LaneFusion::LaneFusionCurrentLineAssociation(const BevLineInnerS& prev_line_, const BevLineInnerS& Bev_Lane_, const std::vector<int>& event_line_id_vec_) {
    float k_EFM_lane_fusion_current_associated_average_distance_threshold = 1.0F;//关联的阈值(要根据点数来计算平均的)（可根据Type是否一致来进行自适应调整）
    float k_EFM_lane_fusion_current_associated_single_distance_threshold = 1.0F;//关联的阈值(单点的判断，单个点如果差很多，也认为没有关联上)
    int k_EFM_lane_fusion_current_associated_overlap_num_threshold = 2; //关联的重合点数量阈值（可根据Type是否一致来进行自适应调整）5个点是4段，2.5m一段就是10m
    bool k_EFM_enable_lane_event_inhibit_fusion_ego = true; //是否允许用split和merge的lane event来抑制融合（即event里面的线不参与融合），当前轨迹是event里面的，直接reset

    bool is_associated = false;
    float total_distance_ = 0.0f; // 累计距离
    int overlap_number_ = 0; // 重合点数量

    for (int i = 0; i < prev_line_.lane_fusion_process_info.LaneFusionPoints.size(); ++i) {

        if ((prev_line_.lane_fusion_process_info.LaneFusionPoints[i].valid == true) &&
            (Bev_Lane_.lane_fusion_process_info.LaneFusionPoints[i].valid == true)) {
            overlap_number_ += 1; // 有重合点

            float single_distance_ = fabsf(prev_line_.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32 - Bev_Lane_.lane_fusion_process_info.LaneFusionPoints[i].y);
            
            if (single_distance_ > k_EFM_lane_fusion_current_associated_single_distance_threshold) {
                overlap_number_ = 0;
                total_distance_ = 100;
                break; // 如果单个点的距离超过阈值，则认为没有关联上，直接跳出循环
            }
            total_distance_ += single_distance_; // 计算重合点的距离
        }
    }

    float average_distance_ = 100.0f; // 平均距离初始化
    if(overlap_number_ > 0){
        average_distance_ = total_distance_ / overlap_number_; // 计算平均距离
    }    

    if (overlap_number_ >= k_EFM_lane_fusion_current_associated_overlap_num_threshold &&
        average_distance_ <= k_EFM_lane_fusion_current_associated_average_distance_threshold) {

        is_associated = true;
    }

    //判断是否是lane event成员，是的话，直接要reset
    if(k_EFM_enable_lane_event_inhibit_fusion_ego == true){
        if (!event_line_id_vec_.empty()){
            for (int i = 0; i < event_line_id_vec_.size(); i++){
                if (Bev_Lane_.id == event_line_id_vec_[i]){
                    is_associated = false;
                    break;
                }
            }
        }
    }
    return is_associated;
}

//车道线关联(在两条当前车道线之间寻找关联性，如果有，则将二者合并为一个车道线，此时的关联要严苛一点)
bool LaneFusion::LaneFusionCloseLineAssociation(std::vector<BevLineInnerS>& data_base_Associated_lines_, std::vector<BevLineInnerS>& new_lines_, const BevLineInnerS& prev_line_) {
    // 车道线关联  关联上了要删除僵尸轨迹？--是否要删除？还是最好要一直存在？--删除  
    //todo 将僵尸轨迹维持的时间考虑进来进行关联阈值的标定（时间越久，越允许将阈值放大一点，于此同时要把其方差放大，这样对融合后的结果影响较小。要不要考虑自车运动状态即换道时放大？）
    //todo 有可能多条轨迹与某一僵尸轨迹是可以关联上的，但是现在会是先到先得，是不是不太好？需要遍历完所有pre吗？可能是需要大改一下。
    float k_EFM_lane_fusion_close_associated_average_distance_threshold = 0.2F;//关联的阈值(要根据点数来计算平均的)（可根据Type是否一致来进行自适应调整）
    float k_EFM_lane_fusion_close_associated_single_distance_threshold = 0.5F;// 单个点的距离阈值
    int k_EFM_lane_fusion_close_associated_overlap_num_threshold = 5; //关联的重合点数量阈值（可根据Type是否一致来进行自适应调整）5个点是4段，2.5m一段就是10m
    // std::cout << "prev_line_.id: " << prev_line_.id << std::endl;
    bool is_associated = false;
    data_base_Associated_lines_.clear(); // 清空关联的线    

    if (!new_lines_.empty()){

        for (auto it = new_lines_.begin(); it != new_lines_.end(); ) {

            float total_distance_ = 0.0f; // 累计距离
            int overlap_number_ = 0; // 重合点数量
            for (int i = 0; i < prev_line_.lane_fusion_process_info.LaneFusionPoints.size(); ++i) {

                if ((prev_line_.lane_fusion_process_info.LaneFusionPoints[i].valid == true) &&
                    (it->lane_fusion_process_info.LaneFusionPoints[i].valid == true)) {

                    float single_distance_ = fabsf(prev_line_.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32 - it->lane_fusion_process_info.LaneFusionPoints[i].y);
                    if (single_distance_ > k_EFM_lane_fusion_close_associated_single_distance_threshold) {

                        overlap_number_ = 0;
                        total_distance_ = 100;
                        break; // 如果单个点的距离超过阈值，则认为没有关联上，直接跳出循环
                    }
                    overlap_number_ += 1; // 有重合点
                    total_distance_ += single_distance_; // 计算重合点的距离
                }
            }
            float average_distance_ = 100.0f; // 平均距离初始化
            if(overlap_number_ > 0){

                average_distance_ = total_distance_ / overlap_number_; // 计算平均距离
            }

            if (overlap_number_ >= k_EFM_lane_fusion_close_associated_overlap_num_threshold &&
                average_distance_ <= k_EFM_lane_fusion_close_associated_average_distance_threshold &&
                it->lane_location_type != 21 && it->lane_location_type != 22 &&
                prev_line_.lane_location_type != 21 && prev_line_.lane_location_type != 22) {

                data_base_Associated_lines_.push_back(*it); // 将关联的线添加到结果中
                // 删除当前 zombie_line
                it = new_lines_.erase(it); // 安全删除元素并更新迭代器
            } else {
                ++it; // 如果不删除，正常推进迭代器
            }
        }   
        if (!data_base_Associated_lines_.empty()){

            is_associated = true;
        }
    }
    return is_associated;
}


//车道线滤波
void LaneFusion::LaneFusionLineFusion(std::vector<BevLineInnerS>& bev_lines_, std::vector<BevLineInnerS>& data_base_zombie_lines_, std::vector<BevLineInnerS>& prev_frame_active_lines_, const DoublePosePoint& ego_pos_wgs84_, const std::vector<int>& event_line_id_vec_) {
    //车道线滤波
    // 1. 对 bev_lines_ 中的每一条线进行滤波处理
    // 2. 对于每一条线，先与上一帧的活线进行关联（如果有相同的 id），然后进行卡尔曼滤波
    // 3. 再尝试与僵尸轨迹进行关联，并于关联上了的线进行滤波
    // 4. 最后将滤波后的结果存储到 bev_lines_ 中

    //注意：bev_lines_ 是当前帧的感知轨迹，prev_frame_active_lines_ 是上一帧的活的轨迹（即除了僵尸轨迹之外的线），data_base_zombie_lines_ 是上一帧的僵尸轨迹。
    // step 对僵尸轨迹进行关联，在僵尸轨迹转化之后关联，是合理的。关联上之后更新bev_lines_（该线的上一帧以及该僵尸轨迹均参与到滤波过程）
    //标定参数
    float k_EFM_initial_variance_f32 = 0.001f;//初始方差
    float k_EFM_kalman_process_variance_f32 = 0.001f;//过程噪声方差
    float k_EFM_kalman_step_x_f32 = 2.5f; // kalman滤波离散步长，假设为2.5
    bool k_EFM_enable_lane_event_inhibit_fusion_zombie = true; //是否允许用split和merge的lane event来抑制融合（即event里面的线不参与融合）.zombie轨迹一旦和event的线关联上，不融合且还会将该僵尸轨迹删除，正义的光，僵尸见到即灰飞烟灭。

    CommonTool::CoordinateTool* coordinate_tool_instance = CommonTool::CoordinateTool::GetInstance();

    //局部变量
    std::vector<BevLineInnerS> data_base_Associated_lines_;
    std::vector<BevLineInnerS> new_lines_;

    if (!prev_frame_active_lines_.empty() && !bev_lines_.empty()){

        for (auto& prev_line_ : prev_frame_active_lines_) {

            prev_line_.line_overlap_points = {};//overlap点全删除
        }

        for (auto& current_line : bev_lines_) {
            
            bool line_be_refreshed = false;
            // 遍历 prev_frame_active_line_ 查找是否有相同的 id
            for (auto& prev_line : prev_frame_active_lines_) {
                // std::cout << "Overlap_current_line.id=: " << current_line.id << std::endl;
                // std::cout << "Overlap_prev_line.id=: " << prev_line.id << std::endl;
                if (prev_line.id == current_line.id) {
                    // std::cout << "Overlap_debug 1: " <<  std::endl;
                    line_be_refreshed = true;
                    // 滤波处理
                    int start_overlap_point_index = 10000;//相交的起点
                    float delta_start_point_pre = 0;
                    float delta_start_point_cur = 0;
                    int end_overlap_point_index = -1;//相交的终点
                    float delta_end_point_pre = 0;
                    float delta_end_point_cur = 0;
                    float start_y_filtered = 0;
                    float end_y_filtered = 0;
                    // 自车道线融合，也要检验一下，一条线突然变短且末端偏差很大，末端之后的点要干掉，即加一个滤波器的重置机制！！！或者就用相似性判断；或者是终点判断--Done
                    for (int i = 0; i < prev_line.lane_fusion_process_info.LaneFusionPoints.size(); ++i) {

                        if (    (prev_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true)
                             && (current_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true)) { 

                                start_overlap_point_index = std::min(start_overlap_point_index, i);
                                float start_y_raw_pre = prev_line.lane_fusion_process_info.LaneFusionPoints[start_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;
                                float start_y_raw_cur = current_line.lane_fusion_process_info.LaneFusionPoints[start_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;
                                end_overlap_point_index = std::max(end_overlap_point_index, i);
                                float end_y_raw_pre = prev_line.lane_fusion_process_info.LaneFusionPoints[end_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;
                                float end_y_raw_cur = current_line.lane_fusion_process_info.LaneFusionPoints[end_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;
                            
                            // 重合部分，应用卡尔曼滤波
                            Kalman_filter_prediction_OneDim( &prev_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct, k_EFM_kalman_process_variance_f32 );
                         
                            Kalman_filter_update_OneDim(&prev_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct, &current_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct); // 卡尔曼滤波

                            start_y_filtered = prev_line.lane_fusion_process_info.LaneFusionPoints[start_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;
                            end_y_filtered = prev_line.lane_fusion_process_info.LaneFusionPoints[end_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;

                            //注意BST是佐证有福
                            delta_start_point_pre = start_y_filtered - start_y_raw_pre;
                            delta_start_point_cur = start_y_filtered - start_y_raw_cur;
                            delta_end_point_pre = end_y_filtered - end_y_raw_pre;
                            delta_end_point_cur = end_y_filtered - end_y_raw_cur;
                        }
                    }

                    prev_line.line_overlap_points.start_overlap_point.x = fmaxf(-100, fminf((start_overlap_point_index * k_EFM_kalman_step_x_f32)-100, 150.0));
                    prev_line.line_overlap_points.start_overlap_point.y = start_y_filtered;
                    prev_line.line_overlap_points.end_overlap_point.x = fmaxf(-100, fminf((end_overlap_point_index * k_EFM_kalman_step_x_f32)-100, 150.0));
                    prev_line.line_overlap_points.end_overlap_point.y = end_y_filtered;
                    prev_line.line_overlap_points.is_valid = true;

                    //处理没有重合的点
                    LaneFusionNoneOverlapProcess(start_overlap_point_index,
                                             delta_start_point_pre,
                                             delta_start_point_cur,
                                             end_overlap_point_index,
                                             delta_end_point_pre,
                                             delta_end_point_cur,
                                             prev_line,
                                             current_line);
                    
                    //对于更新的线，将线型和locationType都设置为当前实时更新的
                     prev_line.line_type = current_line.line_type;
                     prev_line.lane_location_type = current_line.lane_location_type;
                     prev_line.typ_chg_point = current_line.typ_chg_point;
                     prev_line.typ_aft_chg_point = current_line.typ_aft_chg_point;
                     prev_line.md_qly = current_line.md_qly;
                     
                    //再去僵尸轨迹池里面看看是否有可关联上的轨迹====================================================================================
                    data_base_Associated_lines_.clear(); // 清空之前的关联线
                    bool AssociationLineExist = false; // 是否有关联的线

                    AssociationLineExist = LaneFusionZombieLineAssociation(data_base_Associated_lines_, data_base_zombie_lines_, prev_line);


                    // std::cout << "Association_prev_line.id: " << prev_line.id << std::endl;
                    // std::cout << "Association_AssociationLineExist: " << AssociationLineExist << std::endl;

                    // std::cout << "AssociationLineExist: " << AssociationLineExist << std::endl;
                    // 僵尸轨迹没有像上面那样将历史的轨迹拼上，要做！！！--Done
                    // 融合上了的车道线，要将与哪个车道线进行融合的，原始的id给到下游--Done
                    if(true == AssociationLineExist){

                        for (auto& associated_line : data_base_Associated_lines_) {

                            // 滤波处理
                            int start_overlap_point_index = 10000;//相交的起点
                            float delta_start_point_pre = 0;
                            float delta_start_point_cur = 0;
                            int end_overlap_point_index = -1;//相交的终点
                            float delta_end_point_pre = 0;
                            float delta_end_point_cur = 0;

                            for (int i = 0; i < associated_line.lane_fusion_process_info.LaneFusionPoints.size(); ++i) {

                                if(    (prev_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true)
                                    && (associated_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true)){

                                    start_overlap_point_index = std::min(start_overlap_point_index, i);
                                    float start_y_raw_pre = prev_line.lane_fusion_process_info.LaneFusionPoints[start_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;
                                    float start_y_raw_cur = associated_line.lane_fusion_process_info.LaneFusionPoints[start_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;
                                    end_overlap_point_index = std::max(end_overlap_point_index, i);
                                    float end_y_raw_pre = prev_line.lane_fusion_process_info.LaneFusionPoints[end_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;
                                    float end_y_raw_cur = associated_line.lane_fusion_process_info.LaneFusionPoints[end_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;

                                    Kalman_filter_prediction_OneDim( &associated_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct, k_EFM_kalman_process_variance_f32 );
                                    Kalman_filter_update_OneDim(&prev_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct, &associated_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct); // 卡尔曼滤波
                                    
                                    float start_y_filtered = prev_line.lane_fusion_process_info.LaneFusionPoints[start_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;
                                    float end_y_filtered = prev_line.lane_fusion_process_info.LaneFusionPoints[end_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;

                                    //注意BST是佐证有福
                                    delta_start_point_pre = start_y_filtered - start_y_raw_pre;
                                    delta_start_point_cur = start_y_filtered - start_y_raw_cur;
                                    delta_end_point_pre = end_y_filtered - end_y_raw_pre;
                                    delta_end_point_cur = end_y_filtered - end_y_raw_cur;

                                }
                            }  
                            //处理没有重合的点
                            LaneFusionNoneOverlapProcess(start_overlap_point_index,
                                                        delta_start_point_pre,
                                                        delta_start_point_cur,
                                                        end_overlap_point_index,
                                                        delta_end_point_pre,
                                                        delta_end_point_cur,
                                                        prev_line,
                                                        associated_line);

                            prev_line.lane_fusion_process_info.bev_line_age = fmaxf(prev_line.lane_fusion_process_info.bev_line_age, associated_line.lane_fusion_process_info.bev_line_age);
                            EFMFusedLineStsType fused_line_sts_;
                            fused_line_sts_.line_fused_age = 0; // 初始化融合线的年龄
                            fused_line_sts_.line_id = associated_line.lane_fusion_process_info.zombie_line_origin_id; // 设置融合线的原始 ID
                            prev_line.fused_line_sts.push_back(fused_line_sts_);
                        }
                        prev_line.line_fusion_type = 1; // 设置融合属性为 1 融合线
                    }
                    //对于融合后的线，要将kalman的点到融合的点转化(融合的点到输出的点的转化在后面进行)
                    for (int i = 0; i < prev_line.lane_fusion_process_info.LaneFusionPoints.size(); ++i) {

                        if (prev_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true){

                            prev_line.lane_fusion_process_info.LaneFusionPoints[i].y = prev_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32; // 更新 y 坐标为卡尔曼滤波后的值
                        }
                    }
                    break;  // 找到后跳出循环
                }
            }

            //新的轨迹，kalman参数进行初始化
            if(line_be_refreshed == false){

                for (int i = 0; i < current_line.lane_fusion_process_info.LaneFusionPoints.size(); ++i) {

                    if (current_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true) {//不管是否valid，全部赋值

                        SCALAR_STATE_STRUCT initial_state_s;
                        initial_state_s.mean_f32     = current_line.lane_fusion_process_info.LaneFusionPoints[i].y;
                        initial_state_s.variance_f32 = k_EFM_initial_variance_f32;
                        Kalman_filter_init_OneDim(&current_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct, &initial_state_s);
                    }
                }
                new_lines_.push_back(current_line); // 将当前线添加到上一帧活的线中


                    //处理僵尸轨迹，将能与lane event关联上的僵尸轨迹都删除====
                if (k_EFM_enable_lane_event_inhibit_fusion_zombie == true){

                    bool is_lane_event_line = false;
                    if (!event_line_id_vec_.empty()){

                        for (int i = 0; i < event_line_id_vec_.size(); i++){

                            if (current_line.id == event_line_id_vec_[i]){

                                is_lane_event_line = true;
                                break;
                            }
                        }
                    }
                    if (is_lane_event_line == true){

                        data_base_Associated_lines_.clear(); // 清空之前的关联线
                        bool AssociationLineExist = false; // 是否有关联的线

                        AssociationLineExist = LaneFusionZombieLineAssociation(data_base_Associated_lines_, data_base_zombie_lines_, current_line);

                        if (AssociationLineExist == true){

                            for (auto& associated_line : data_base_Associated_lines_) {

                                if (!data_base_zombie_lines_.empty()) {

                                    for (auto it = data_base_zombie_lines_.begin(); it != data_base_zombie_lines_.end();) {
                                        
                                        //条件判断
                                        if (it->id == associated_line.id) {
                                            it = data_base_zombie_lines_.erase(it);  // 删除当前元素，并更新迭代器
                                        } else {
                                            ++it;  // 继续下一个元素
                                        }
                                        //不可以用it打印，注意！
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        //将prev_frame_active_lines_中，kalman的位置，放到total_line的结构体中
        if(!prev_frame_active_lines_.empty()) {
            
            for (auto& prev_line : prev_frame_active_lines_) {
                
                prev_line.total_line.clear();
                for (int i = 0; i < prev_line.lane_fusion_process_info.LaneFusionPoints.size(); ++i) {
                    
                    if (prev_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true) {
                        
                        EFMPoint prev_point;
                        prev_point.x = prev_line.lane_fusion_process_info.LaneFusionPoints[i].x;
                        prev_point.y = prev_line.lane_fusion_process_info.LaneFusionPoints[i].y;
                        prev_line.total_line.push_back(prev_point);
                    }
                }

                //对于融合上了的线，要利用融合之后的自车坐标系下的坐标，将融合上了的点的gps信息更新。
                std::vector<Point2Dd> line_body;
                std::vector<Point2Dd> line_wgs84;

                prev_line.lane_fusion_process_info.total_line_gps_points.clear(); // 清空之前的 GPS 点

                for (const auto& point : prev_line.total_line) {

                    Point2Dd body_point;
                    body_point.x = point.x;  // 将 current_line 中的 x 赋值给 line_body 的 x
                    body_point.y = point.y;  // 将 current_line 中的 y 赋值给 line_body 的 y
                    line_body.push_back(body_point);
                }

                coordinate_tool_instance->LineBodyToWGS84(line_body, ego_pos_wgs84_, line_wgs84);

                for (const auto& point : line_wgs84) {

                    GPSLinePoint body_point;
                    body_point.Longitude = point.x;  // 将 current_line 中的 x 赋值给 line_body 的 x
                    body_point.Latitude = point.y;  // 将 current_line 中的 y 赋值给 line_body 的 y
                    prev_line.lane_fusion_process_info.total_line_gps_points.push_back(body_point);
                }
            }
        }

        // 清空 bev_lines_
        bev_lines_.clear();

        if (!new_lines_.empty() && !prev_frame_active_lines_.empty()) {

            //对new_lines_进行关联，将两个关联度比较高的线删除一个（一定要将其在new_lines_中删除）--第一帧就能关联上的可以，因为是newline,多帧的暂不处理，后面再说

            for (auto& prev_line : prev_frame_active_lines_) {

                data_base_Associated_lines_.clear(); // 清空之前的关联线
                bool AssociationLineExist = false; // 是否有关联的线
                // AssociationLineExist = LaneFusionCloseLineAssociation(data_base_Associated_lines_, new_lines_, prev_line);//对所有当前的实时线进行关联，存在event属性不对问题，暂关闭（后面可以考虑将属性mapping一下，继续使用）
                //std::cout << "Closeprev_line.id===================: " << prev_line.id << std::endl;
                //std::cout << "CloseAssociationLineExist: " << AssociationLineExist << std::endl;
                if(true == AssociationLineExist){

                    for (auto& associated_line : data_base_Associated_lines_) {
                        std::cout << "associated_line.id=: " << associated_line.id << std::endl;

                        // 滤波处理
                        int start_overlap_point_index = 10000;//相交的起点
                        float delta_start_point_pre = 0;
                        float delta_start_point_cur = 0;
                        int end_overlap_point_index = -1;//相交的终点
                        float delta_end_point_pre = 0;
                        float delta_end_point_cur = 0;

                        for (int i = 0; i < associated_line.lane_fusion_process_info.LaneFusionPoints.size(); ++i) {

                            if(    (prev_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true)
                                && (associated_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true)){

                                start_overlap_point_index = std::min(start_overlap_point_index, i);
                                float start_y_raw_pre = prev_line.lane_fusion_process_info.LaneFusionPoints[start_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;
                                float start_y_raw_cur = associated_line.lane_fusion_process_info.LaneFusionPoints[start_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;
                                end_overlap_point_index = std::max(end_overlap_point_index, i);
                                float end_y_raw_pre = prev_line.lane_fusion_process_info.LaneFusionPoints[end_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;
                                float end_y_raw_cur = associated_line.lane_fusion_process_info.LaneFusionPoints[end_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;

                                Kalman_filter_prediction_OneDim( &associated_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct, k_EFM_kalman_process_variance_f32 );
                                Kalman_filter_update_OneDim(&prev_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct, &associated_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct); // 卡尔曼滤波
                                
                                float start_y_filtered = prev_line.lane_fusion_process_info.LaneFusionPoints[start_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;
                                float end_y_filtered = prev_line.lane_fusion_process_info.LaneFusionPoints[end_overlap_point_index].kalman_model_onedim_struct.state_s.mean_f32;

                                //注意BST是佐证有福
                                delta_start_point_pre = start_y_filtered - start_y_raw_pre;
                                delta_start_point_cur = start_y_filtered - start_y_raw_cur;
                                delta_end_point_pre = end_y_filtered - end_y_raw_pre;
                                delta_end_point_cur = end_y_filtered - end_y_raw_cur;
                            }
                        }

                        //处理没有重合的点
                        LaneFusionNoneOverlapProcess(start_overlap_point_index,
                                                    delta_start_point_pre,
                                                    delta_start_point_cur,
                                                    end_overlap_point_index,
                                                    delta_end_point_pre,
                                                    delta_end_point_cur,
                                                    prev_line,
                                                    associated_line);
                        
                        prev_line.lane_fusion_process_info.bev_line_age = fmaxf(prev_line.lane_fusion_process_info.bev_line_age, associated_line.lane_fusion_process_info.bev_line_age);
                        EFMFusedLineStsType fused_line_sts_;
                        fused_line_sts_.line_fused_age = 0; // 初始化融合线的年龄
                        fused_line_sts_.line_id = associated_line.id; // 设置融合线的原始 ID
                        prev_line.fused_line_sts.push_back(fused_line_sts_);
                    }
                    prev_line.line_fusion_type = 1; // 设置融合属性为 1 融合线

                    //对于融合后的线，要将kalman的点到融合的点转化(融合的点到输出的点的转化在后面进行)==
                    for (int i = 0; i < prev_line.lane_fusion_process_info.LaneFusionPoints.size(); ++i) {

                        if (prev_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true){

                            prev_line.lane_fusion_process_info.LaneFusionPoints[i].y = prev_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32; // 更新 y 坐标为卡尔曼滤波后的值
                        }
                    }

                    //将关联了的线转到total中，并对其GPS的坐标进行更新==
                    prev_line.total_line.clear();

                    for (int i = 0; i < prev_line.lane_fusion_process_info.LaneFusionPoints.size(); ++i) {
                        
                        if (prev_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true) {
                            
                            EFMPoint prev_point;
                            prev_point.x = prev_line.lane_fusion_process_info.LaneFusionPoints[i].x;
                            prev_point.y = prev_line.lane_fusion_process_info.LaneFusionPoints[i].y;
                            prev_line.total_line.push_back(prev_point);
                        }
                    }

                    //对于融合上了的线，要利用融合之后的自车坐标系下的坐标，将融合上了的点的gps信息更新。
                    std::vector<Point2Dd> line_body;
                    std::vector<Point2Dd> line_wgs84;

                    prev_line.lane_fusion_process_info.total_line_gps_points.clear(); // 清空之前的 GPS 点

                    for (const auto& point : prev_line.total_line) {

                        Point2Dd body_point;
                        body_point.x = point.x;  // 将 current_line 中的 x 赋值给 line_body 的 x
                        body_point.y = point.y;  // 将 current_line 中的 y 赋值给 line_body 的 y
                        line_body.push_back(body_point);
                    }

                    coordinate_tool_instance->LineBodyToWGS84(line_body, ego_pos_wgs84_, line_wgs84);

                    for (const auto& point : line_wgs84) {

                        GPSLinePoint body_point;
                        body_point.Longitude = point.x;  // 将 current_line 中的 x 赋值给 line_body 的 x
                        body_point.Latitude = point.y;  // 将 current_line 中的 y 赋值给 line_body 的 y
                        prev_line.lane_fusion_process_info.total_line_gps_points.push_back(body_point);
                    }

                }
                // break;  // 找到后跳出循环
                //todo 要再进行2.5m一个的重分配，类型之类的再考虑一下怎么放
                //todo 将融合的起止点的index处理一下，输出给下游（待2.5离散的逻辑处理完毕之后做）
            }
            
            // 将 new_lines_ 中的线添加到 bev_lines_==
            bev_lines_.insert(bev_lines_.end(), new_lines_.begin(), new_lines_.end());
        }

        // 将 prev_line 添加到 bev_lines_
        if(!prev_frame_active_lines_.empty()) {
            // 将 prev_frame_active_lines_ 中的线添加到 bev_lines_
            bev_lines_.insert(bev_lines_.end(), prev_frame_active_lines_.begin(), prev_frame_active_lines_.end());
        }
    }
}

void LaneFusion::LaneFusionNoneOverlapProcess(const int start_overlap_point_index,
                                             const float delta_start_point_pre,
                                             const float delta_start_point_cur,
                                             const int end_overlap_point_index,
                                             const float delta_end_point_pre,
                                             const float delta_end_point_cur,
                                             BevLineInnerS& prev_line,
                                             const BevLineInnerS& current_line){
//处理没有重合的点===============================================================
    //处理起始点前面的点
    if(start_overlap_point_index < 10000 && start_overlap_point_index > 0){
        // std::cout << "Overlap_debug 3: " <<  std::endl;
        
        if(prev_line.lane_fusion_process_info.LaneFusionPoints[start_overlap_point_index-1].valid == true ){//起点前面的是pre线

            // std::cout << "Overlap_debug 8: " <<  std::endl;
            for(int i = 0; i < start_overlap_point_index; ++i){
                // std::cout << "prev_line.lane_fusion_process_info.LaneFusionPoints[" <<i<< "].valid: " <<  prev_line.lane_fusion_process_info.LaneFusionPoints[i].valid << std::endl;
                if(prev_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true){
                    prev_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32 += delta_start_point_pre;
                }
            }
        }else{//起点前面是current线
            // std::cout << "Overlap_debug 4: " <<  std::endl;
            for(int i = 0; i < start_overlap_point_index; ++i){

                if(current_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true){

                    prev_line.lane_fusion_process_info.LaneFusionPoints[i].valid = true;
                    prev_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32 = current_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32 + delta_start_point_cur;
                    prev_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.variance_f32 = current_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.variance_f32;
                }
            }
        }
    }

    //处理终止点后面的点
    if(end_overlap_point_index > -1 && end_overlap_point_index < current_line.lane_fusion_process_info.LaneFusionPoints.size() && end_overlap_point_index < prev_line.lane_fusion_process_info.LaneFusionPoints.size()){
        // std::cout << "Overlap_debug 5: " <<  std::endl;
        if(prev_line.lane_fusion_process_info.LaneFusionPoints[end_overlap_point_index + 1].valid == true ){//终点后面的是pre线
            // std::cout << "Overlap_debug 7: " <<  std::endl;
            for(int i = end_overlap_point_index+1; i < prev_line.lane_fusion_process_info.LaneFusionPoints.size(); ++i){

                if(prev_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true){

                    prev_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32 += delta_end_point_pre;
                }
            }
        }else{//终点后面是current线
            // std::cout << "prev_line_inter: " << prev_line.id << std::endl;
            for(int i = end_overlap_point_index+1; i < current_line.lane_fusion_process_info.LaneFusionPoints.size(); ++i){
                // std::cout << "current_line.lane_fusion_process_info.LaneFusionPoints[" <<i<< "].valid: " <<  current_line.lane_fusion_process_info.LaneFusionPoints[i].valid << std::endl;
                // std::cout << "current_line.lane_fusion_process_info.LaneFusionPoints[" <<i<< "].kalman_model_onedim_struct.state_s.mean_f32: " <<  current_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32 << std::endl;
                
                if(current_line.lane_fusion_process_info.LaneFusionPoints[i].valid == true){

                    prev_line.lane_fusion_process_info.LaneFusionPoints[i].valid = true;
                    prev_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32 = current_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32 + delta_end_point_cur;
                    prev_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.variance_f32 = current_line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.variance_f32;
                }
            }
        }
    }
}

void LaneFusion::Kalman_filter_init_OneDim( KALMAN_MODEL_ONEDIM_STRUCT* const p_Kalman_model_s, const SCALAR_STATE_STRUCT* const p_initial_state_s )
{
    if ( NULL != p_Kalman_model_s )
    {
        /* reset the whole Kalman model struct */
        (void)memset(p_Kalman_model_s, 0, sizeof(KALMAN_MODEL_ONEDIM_STRUCT));

        if ( NULL != p_initial_state_s )
        {
            /* initialize state mean and variance*/
            p_Kalman_model_s->state_s.mean_f32          = p_initial_state_s->mean_f32;
            p_Kalman_model_s->state_s.variance_f32      = p_initial_state_s->variance_f32;

            /* set has_been_initialised_b to true*/
            p_Kalman_model_s->has_been_initialised_b    = true;
        }
        else
        {
            /* set has_been_initialised_b to false*/
            p_Kalman_model_s->has_been_initialised_b    = false;
        }
    }
    else
    {
        /* Do nothing*/
    }
}   

void LaneFusion::Kalman_filter_prediction_OneDim( KALMAN_MODEL_ONEDIM_STRUCT* const p_Kalman_model_s,
                                      const float process_variance_f32 )
{
    //todo 用 x y 和kalman的对比设置方差的大小
    if ( NULL != p_Kalman_model_s )
    {
        /* the calculation are based on following
         * A_f32 = 1.0F;
         * Ppred_f32 = A_f32 * p_state_s->variance_vf32 * A_f32;
         * Q_f32 = dt_f32 * dt_f32 * process_variance_f32;
         * p_state_s->variance_vf32 = Ppred_f32 + Q_f32;
         */

        /* predict mean */
        /* p_Kalman_model_s->state_s.mean_vf32 = p_Kalman_model_s->state_s.mean_vf32;  -> do nothing! */

        /* Predict state variance  only if process_variance_f32 is a positive value */
        if ( process_variance_f32 > 0.0F )
        {
            /* predict variance */
            p_Kalman_model_s->state_s.variance_f32      += process_variance_f32;
        }
        else
        {
            /* Do nothing */
        }

    }
    else
    {
        /* Do nothing */
    }
}

void LaneFusion::Kalman_filter_update_OneDim(KALMAN_MODEL_ONEDIM_STRUCT* const p_Kalman_model_s, const KALMAN_MODEL_ONEDIM_STRUCT* const p_meas_feature_s)
{
    /*  DEVIATION:     781
        DESCRIPTION:   'innovation_variance_f32' and 'innovation_value_f32' are being used as a structure/union member as well as being a label, tag or ordinary identifier.
        JUSTIFICATION: For readability and maintainability, identical signals should have identical names. */
    /* PRQA S 781 01 */

    SCALAR_STATE_STRUCT pred_state_s;
    SCALAR_STATE_STRUCT innovation_s;
    SCALAR_STATE_STRUCT pred_meas_s;

    float                 Kalman_gain_f32;

    if (   ( NULL != p_Kalman_model_s )
        && ( NULL != p_meas_feature_s )  )
    {
        /* the calculation are based on following
            * C_row_f32 = c_f32;
            * innovation_variance_f32 = C_row_f32 * p_state_s->variance_vf32 * C_row_f32;
            * pred_feature_input_noise_f32 = 0.0F;
            * innovation_variance_f32 += p_meas_feature_s->variance_vf32 + pred_feature_input_noise_f32;
            */

        /* save the predicted state */
        pred_state_s       = p_Kalman_model_s->state_s;

        pred_meas_s.variance_f32      = pred_state_s.variance_f32;

        /* calculate innovation variance */
        innovation_s.variance_f32 = pred_meas_s.variance_f32 + p_meas_feature_s->state_s.variance_f32;

        /* calculate Kalman gain */
        if (    ( innovation_s.variance_f32           > 0.0000001F )
            &&  ( pred_meas_s.variance_f32            > 0.0F )
            &&  ( p_meas_feature_s->state_s.variance_f32      > 0.0F )    )
        {
            Kalman_gain_f32 = ( pred_state_s.variance_f32 * 1.0F ) / innovation_s.variance_f32;
        }
        else /* if measurement or predicted state variance is not positive,
                set Kalman gain to zero */
        {
            Kalman_gain_f32         = 0.0F;

        }

        pred_meas_s.mean_f32 = pred_state_s.mean_f32 * 1.0F;

        innovation_s.mean_f32 = p_meas_feature_s->state_s.mean_f32- pred_meas_s.mean_f32;

        /* state variance update */
        p_Kalman_model_s->state_s.variance_f32  = pred_state_s.variance_f32 * ( 1.0F - ( Kalman_gain_f32 * 1.0F ) );

        /* state mean update */
        p_Kalman_model_s->state_s.mean_f32      = pred_state_s.mean_f32 + ( Kalman_gain_f32 * innovation_s.mean_f32 );
    }
}

void LaneFusion::LaneFusionPointsConvert(BevLineInnerS& line) {
    // 定义离散范围
    float start_x = -100.0f;
    float end_x = 150.0f;
    float step_x = 2.5f;

    // 清空原有的 LaneFusionPoints
    line.lane_fusion_process_info.LaneFusionPoints.clear();

    // 离散化处理
    std::vector<float> discrete_x;
    for (float x = start_x; x <= end_x; x += step_x) {
        discrete_x.push_back(x);
    }

    // 遍历离散后的点
    for (const auto& x : discrete_x) {
        LaneFusionPoint fusion_point;
        fusion_point.x = x;

        bool valid = false;
        float y_value = 0.0f;
        size_t n = line.total_line.size();

        if (n >= 2) {
            // 查找 x 落在的区间
            for (size_t i = 0; i < n - 1; ++i) {
                if (static_cast<float>(line.total_line[i].x) <= x && x <= static_cast<float>(line.total_line[i + 1].x)) {
                    valid = true;

                    // 获取区间两点
                    auto& p0 = line.total_line[i];
                    auto& p1 = line.total_line[i + 1];

                    // 检查 p1.x 和 p0.x 是否相等，避免除以零
                    if (p1.x == p0.x) {
                        y_value = p0.y; // 如果 x 坐标相等，直接取 p0 的 y 值
                        break;
                    }

                    // 获取两点的斜率（导数）
                    float m0 = 0.0f, m1 = 0.0f;
                    if (i == 0) {
                        m0 = (p1.x != p0.x) ? static_cast<float>((p1.y - p0.y) / (p1.x - p0.x)) : 0.0f;
                    } else {
                        m0 = (p1.x != line.total_line[i - 1].x) ? static_cast<float>((p1.y - line.total_line[i - 1].y) / (p1.x - line.total_line[i - 1].x)) : 0.0f;
                    }

                    if (i == n - 2) {
                        m1 = (p1.x != p0.x) ? static_cast<float>((p1.y - p0.y) / (p1.x - p0.x)) : 0.0f;
                    } else {
                        m1 = (line.total_line[i + 2].x != p0.x) ? static_cast<float>((line.total_line[i + 2].y - p0.y) / (line.total_line[i + 2].x - p0.x)) : 0.0f;
                    }

                    // 计算 t
                    float t = static_cast<float>((x - p0.x) / (p1.x - p0.x));

                    // 三次 Hermite 插值公式
                    float h00 = (1 + 2 * t) * (1 - t) * (1 - t);
                    float h10 = t * (1 - t) * (1 - t);
                    float h01 = t * t * (3 - 2 * t);
                    float h11 = t * t * (t - 1);

                    y_value = static_cast<float>(h00 * p0.y + h10 * (p1.x - p0.x) * m0 + h01 * p1.y + h11 * (p1.x - p0.x) * m1);
                    break;
                }
            }
        }

        fusion_point.y = y_value;
        fusion_point.valid = valid ? 1 : 0;
        // 添加到 LaneFusionPoints,将kalman结构中的值也同步更新了
        line.lane_fusion_process_info.LaneFusionPoints.push_back(fusion_point);
    }
}

//将量测信号填充到对应的位置，供kalman滤波使用（注意：只有僵尸轨迹和实时量测轨迹需要。僵尸轨迹的方差可以随时间搞大一点，感知的可以随纵向距离绝对值搞大一点）
void LaneFusion::LaneFusionMeanProcess(BevLineInnerS& line) {

    for (int i = 0; i < line.lane_fusion_process_info.LaneFusionPoints.size(); ++i) {
        // 如果点是有效的，设置量测值和方差
        if (line.lane_fusion_process_info.LaneFusionPoints[i].valid == 1) {
            line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32 = line.lane_fusion_process_info.LaneFusionPoints[i].y;
            line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.has_been_initialised_b = false; 
        } else {
            line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.mean_f32 = 0.0f;
            line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.has_been_initialised_b = false; 
        }
    }
}

//将量测信号填充到对应的位置，供kalman滤波使用（注意：只有僵尸轨迹和实时量测轨迹需要。僵尸轨迹的方差可以随时间搞大一点，感知的可以随纵向距离绝对值搞大一点）
void LaneFusion::LaneFusionVarProcess(BevLineInnerS& line) {

    float k_EFM_Meas_variance_f32 = 0.001f; // 量测方差(后续要改为自适应)

    for (int i = 0; i < line.lane_fusion_process_info.LaneFusionPoints.size(); ++i) {
        // 如果点是有效的，设置量测值和方差
        if (line.lane_fusion_process_info.LaneFusionPoints[i].valid == 1) {
            line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.variance_f32 = k_EFM_Meas_variance_f32;
        } else {
            line.lane_fusion_process_info.LaneFusionPoints[i].kalman_model_onedim_struct.state_s.variance_f32 = 1000.0f;// 设置一个较大的方差，表示不确定性
        }
    }
}

//十字路口：在BEV中添加拼接后的车道线
void LaneFusion::intersection_topology_(std::vector<BevLineInnerS>& bev_lines_, const DoublePosePoint& position_msg) 
{

    //对历史中心扩展线进行转化（类似僵尸轨迹的周期转化）
    LF_LineWGS84ToBody(extended_center_line_lastcycle, position_msg);
    //将点的坐标赋值给融合要用的点的坐标形式，并计算切线斜率（僵尸轨迹）
    LaneFusionPointsConvert(extended_center_line_lastcycle);
    if(!extended_center_line_lastcycle.total_line.empty()){
        if(extended_center_line_lastcycle.total_line.front().x < -500 || extended_center_line_lastcycle.total_line.back().x > 500){
            extended_center_line_lastcycle = {}; // 如果中心线的x坐标超出范围，清空中心线
        }
    }

    //对左右侧线保持
    LF_LineWGS84ToBody(bev_line_left_lastcycle, position_msg);
    LaneFusionPointsConvert(bev_line_left_lastcycle);
    if(!bev_line_left_lastcycle.total_line.empty()){
        if(bev_line_left_lastcycle.total_line.front().x < -500 || bev_line_left_lastcycle.total_line.back().x > 500){
            bev_line_left_lastcycle = {}; // 如果左侧线的x坐标超出范围，清空左侧线
        }
    }

    LF_LineWGS84ToBody(bev_line_right_lastcycle, position_msg);
    LaneFusionPointsConvert(bev_line_right_lastcycle);
    if(!bev_line_right_lastcycle.total_line.empty()){
        if(bev_line_right_lastcycle.total_line.front().x < -500 || bev_line_right_lastcycle.total_line.back().x > 500){
            bev_line_right_lastcycle = {}; // 如果右侧线的x坐标超出范围，清空右侧线
        }
    }

    //calibration values
    float k_EFM_IT_lanewidth_threshold = 2.5;//超过该宽度的线，不考虑是自车左右侧的线
    float k_EFM_IT_dis2endpoint_threshold = 15.0; //自车左右侧线都小于这个距离的时候，开始进行拓扑（不能过早，因为自车存在换道的可能性）
    float k_EFM_IT_default_lanewidth = 3.5; // 默认车道线宽度
    float k_EFM_LF_IT_curv_limit = 0.1;
    float k_EFM_IT_min_valid_length_threshold = 12; // 路口对向车道线的最小长度，小于该值的话不拓扑

    //local parameter
    int closest_left_id = -1;  // 最近左侧车道线的id
    int closest_right_id = -1; // 最近右侧车道线的id
    float min_left_distance = 10000;  // 左侧横向最小距离
    float min_right_distance = 10000; // 右侧横向最小距离
    // float distance_to_endpoint_left = 10000;  // 左侧线终点到自车的x向距离
    // float distance_to_endpoint_right = 10000; // 右侧线终点到自车的x向距离

    BevLineInnerS bev_line_left = {};  // 左侧车道线
    BevLineInnerS bev_line_right = {}; // 右侧车道线
    BevLineInnerS extended_line_left = {};  // 左侧车道线
    BevLineInnerS extended_line_right = {}; // 右侧车道线

    if(!bev_lines_.empty()){
        
        //判断左右侧是否存在,id取出来
        for(const auto& bev_line: bev_lines_){

            float y = bev_line.lane_fusion_process_info.LaneFusionPoints[40].y;//第40个点是自车当前的位置（-100-150，2.5间隔，如有调整，需更改）。
            if(bev_line.lane_fusion_process_info.LaneFusionPoints[40].valid && fabsf(y) <= k_EFM_IT_lanewidth_threshold){

                float distance = fabsf(y); // 距离计算
                if (y > 0) { // 左侧车道线

                    if (distance < min_left_distance) {

                        bev_line_left = {};
                        min_left_distance = distance;
                        bev_line_left = bev_line; // 记录左侧车道线
                        closest_left_id = bev_line_left.id; // 假设 bev_line 有 id 属性
                        // distance_to_endpoint_left = bev_line_left.total_line.back().x; // 假设 bev_line 有 id 属性
                    }
                } else if (y < 0) { // 右侧车道线

                    if (distance < min_right_distance) {

                        bev_line_right = {};
                        min_right_distance = distance;
                        bev_line_right = bev_line;
                        closest_right_id = bev_line_right.id; // 假设 bev_line 有 id 属性
                        // distance_to_endpoint_right = bev_line_right.total_line.back().x; // 假设 bev_line 有 id 属性
                    }
                }
            }
        }
    }

    if(bev_line_left.id == 0){

        bev_line_left = bev_line_left_lastcycle; // 保存上一个周期的自车左边线
    }
    bev_line_left_lastcycle = bev_line_left;

    if(bev_line_right.id == 0){

        bev_line_right = bev_line_right_lastcycle; // 保存上一个周期的自车右边线
    }
    bev_line_right_lastcycle = bev_line_right;

    //judgement of enable intersection topology logic situation: both line are shorter than threshold
    //plot extended center line
    std::vector<double> extended_center_line_x;
    std::vector<double> extended_center_line_y;
    extended_center_line_x.clear();
    extended_center_line_y.clear();
    BevLineInnerS extended_center_line = {};

    //process the center line : noline; two lines; single line （单线可能存在待转线拉偏，后期再处理待转线或者只取两侧都有时候的，有一侧没有就不用）
    //todo: 后期根据道路行点构造居中线，逻辑在这里添加。
    if((!bev_line_left.total_line.empty() && bev_line_left.total_line.back().x > 60) || (!bev_line_right.total_line.empty() && bev_line_right.total_line.back().x > 60)){

        extended_center_line_lastcycle = {};
    }else{

        if(-1 != closest_left_id && -1 != closest_right_id){//both left and right
            
            LaneFusionPointsType bev_line_left_points = bev_line_left.lane_fusion_process_info.LaneFusionPoints;
            LaneFusionPointsType bev_line_right_points = bev_line_right.lane_fusion_process_info.LaneFusionPoints;

            extended_center_line.total_line = IT_calculateExtendedCenterLine(bev_line_left_points, bev_line_right_points, 100.0);
            
            extended_center_line.id = 20000; // 设置一个特殊的id，表示这是扩展的中心线
            //将extended_center_line的点转换为GPS坐标(提前填充total_line)
            LF_LineBodyToWGS84(extended_center_line, position_msg);
            //将点的坐标赋值给融合要用的离散的点的坐标形式，插值处理
            LaneFusionPointsConvert(extended_center_line);
            extended_center_line_lastcycle = extended_center_line;
            
            // std::cout << "extended_center_line.lane_fusion_process_info.LaneFusionPoints.size():" << extended_center_line.lane_fusion_process_info.LaneFusionPoints.size() << std::endl;
            // std::cout << "extended_center_line.total_line.size()1:" << extended_center_line.total_line.size() << std::endl;
        }else if(-1 != closest_left_id){ //only left

            LaneFusionPointsType bev_line_left_points = bev_line_left.lane_fusion_process_info.LaneFusionPoints;
            LaneFusionPointsType bev_line_right_points = bev_line_left.lane_fusion_process_info.LaneFusionPoints;//copy left to right
            for (auto& point : bev_line_right_points){

                if(true == point.valid){
                    point.y = point.y - k_EFM_IT_default_lanewidth; // 假设右侧车道线在左侧车道线的右边
                }
            }
            extended_center_line.total_line = IT_calculateExtendedCenterLine(bev_line_left_points, bev_line_right_points, 100.0);
            
            extended_center_line.id = 20000; // 设置一个特殊的id，表示这是扩展的中心线
            //将extended_center_line的点转换为GPS坐标(提前填充total_line)
            LF_LineBodyToWGS84(extended_center_line, position_msg);
            //将点的坐标赋值给融合要用的离散的点的坐标形式，插值处理
            LaneFusionPointsConvert(extended_center_line);
            extended_center_line_lastcycle = extended_center_line;

        }else if(-1 != closest_right_id){//only right
            LaneFusionPointsType bev_line_left_points = bev_line_right.lane_fusion_process_info.LaneFusionPoints;
            LaneFusionPointsType bev_line_right_points = bev_line_right.lane_fusion_process_info.LaneFusionPoints;
            for (auto& point : bev_line_left_points){

                if(true == point.valid){
                    point.y = point.y + k_EFM_IT_default_lanewidth; // 假设右侧车道线在左侧车道线的右边
                }
            }
            extended_center_line.total_line = IT_calculateExtendedCenterLine(bev_line_left_points, bev_line_right_points, 100.0);
            
            extended_center_line.id = 20000; // 设置一个特殊的id，表示这是扩展的中心线
            //将extended_center_line的点转换为GPS坐标(提前填充total_line)
            LF_LineBodyToWGS84(extended_center_line, position_msg);
            //将点的坐标赋值给融合要用的离散的点的坐标形式，插值处理
            LaneFusionPointsConvert(extended_center_line);
            extended_center_line_lastcycle = extended_center_line;
        }
        else{
            //no line, use history
            //todo 历史的轨迹，要有和僵尸轨迹一样的清空条件

            extended_center_line = extended_center_line_lastcycle; // 保存上一个周期的中心线
        }
    }
    
    //debug↓
    if (!extended_center_line.lane_fusion_process_info.LaneFusionPoints.empty()){
        // std::cout << "extended_center_line.lane_fusion_process_info.LaneFusionPoints.size()2:" << extended_center_line.lane_fusion_process_info.LaneFusionPoints.size() << std::endl;
    }
    if (!extended_center_line_lastcycle.lane_fusion_process_info.LaneFusionPoints.empty()){
        // std::cout << "extended_center_line_lastcycle.lane_fusion_process_info.LaneFusionPoints.size()2:" << extended_center_line_lastcycle.lane_fusion_process_info.LaneFusionPoints.size() << std::endl;
    }

    if (!extended_center_line_lastcycle.total_line.empty()){
        // std::cout << "extended_center_line_lastcycle.total_line.empty():" << extended_center_line_lastcycle.total_line.empty() << std::endl;
        // std::cout << "extended_center_line_lastcycle.total_line.back().x:" << extended_center_line_lastcycle.total_line.back().x << std::endl;
    }
    //debug↑
    
    
    //历史中心线的删除
    if (!extended_center_line_lastcycle.total_line.empty() &&  extended_center_line_lastcycle.total_line.back().x < 0){

        extended_center_line_lastcycle = {};
    }

    //画中心线
    // std::cout << "extended_center_line.lane_fusion_process_info.LaneFusionPoints.empty():" << extended_center_line.total_line.empty() << std::endl;
    if(!extended_center_line.total_line.empty()){
        for (auto p : extended_center_line.total_line) {
            extended_center_line_x.push_back(p.x);
            extended_center_line_y.push_back(p.y);
            // std::cout << "p.x: " << p.x << "p.y: " << p.y << std::endl;
        }
    }

    //todo 锁定左右侧线，锁定中心线（要在离路口比较近的时候）,持续的进行最近线的寻找
    // 找对向左右侧的线====
    BevLineInnerS left_line_ahead;
    BevLineInnerS right_line_ahead;
    float min_y_distance_left = 10000;
    float min_y_distance_right = 10000;
    
    // std::cout << "extended_center_line.empty(): " << extended_center_line.total_line.empty() << std::endl;
    if (!bev_lines_.empty() && !extended_center_line.total_line.empty()) {

        // 遍历 bev_lines_ 中的每条线，找到符合条件的线
        for (const auto& line : bev_lines_) {
            if (line.total_line.empty()) {
                continue; // 跳过没有点的线
            }
            // std::cout << "line.id: " << line.id << std::endl;
            const EFMPoint& start_point = line.total_line.front(); // 获取线的起始点
            const EFMPoint& end_point = line.total_line.back(); // 获取线的终止点
            float line_length = std::abs(end_point.x - start_point.x);
            // std::cout << "start_point: " << start_point.x << ", " << start_point.y << std::endl;
            // 找到 start_point 所处的 extended_center_line 中的最近点
            size_t closest_index = 0;
            float min_x_distance = 10000;
            for (size_t i = 0; i < extended_center_line.total_line.size(); ++i) {
                float x_distance = std::abs(start_point.x - extended_center_line.total_line[i].x);
                if (x_distance < min_x_distance) {
                    min_x_distance = x_distance;
                    closest_index = i;
                }
            }
            // std::cout << "closest_index: " << closest_index << std::endl;

            //todo Heading的校验等逻辑，可以在这里做
            // 获取最近点的 y 值
            float closest_y = extended_center_line.total_line[closest_index].y;
            // std::cout << "closest_y: " << closest_y << std::endl;

            // 获取 extended_center_line 在 closest_index 处的斜率
            float extended_slope = 0.0f;
            if (closest_index < extended_center_line.total_line.size() - 1) {
                const auto& curr_point = extended_center_line.total_line[closest_index];
                const auto& next_point = extended_center_line.total_line[closest_index + 1];
                if (next_point.x != curr_point.x) { // 避免除以零
                    extended_slope = (next_point.y - curr_point.y) / (next_point.x - curr_point.x);
                }
            }

            // 判断是否在 extended_center_line 左侧且 x > distance_to_endpoint_left
            if (!bev_line_left.total_line.empty()) {
                if (start_point.x > bev_line_left.total_line.back().x && start_point.y > closest_y && start_point.y < (closest_y + k_EFM_IT_default_lanewidth) && line_length > k_EFM_IT_min_valid_length_threshold) {
                    // 获取 line 在 closest_index 处的斜率
                    float line_slope = 0.0f;
                    if (closest_index < line.total_line.size() - 1) {
                        const auto& curr_point = line.total_line[closest_index];
                        const auto& next_point = line.total_line[closest_index + 1];
                        if (next_point.x != curr_point.x) { // 避免除以零
                            line_slope = (next_point.y - curr_point.y) / (next_point.x - curr_point.x);
                        }
                    }

                    // 判断斜率之差是否小于 k_EFM_LF_IT_curv_limit
                    if (std::abs(extended_slope - line_slope) < k_EFM_LF_IT_curv_limit) {
                        float y_distance = std::abs(start_point.y - closest_y);
                        if (y_distance < min_y_distance_left) {
                            min_y_distance_left = y_distance;
                            left_line_ahead = line;
                        }
                    }
                }
            }

            // 判断是否在 extended_center_line 右侧且 x > distance_to_endpoint_right
            if (!bev_line_right.total_line.empty()) {
                if (start_point.x > bev_line_right.total_line.back().x && start_point.y < closest_y && start_point.y > (closest_y - k_EFM_IT_default_lanewidth) && line_length > k_EFM_IT_min_valid_length_threshold) {
                    // 获取 line 在 closest_index 处的斜率
                    float line_slope = 0.0f;
                    if (closest_index < line.total_line.size() - 1) {
                        const auto& curr_point = line.total_line[closest_index];
                        const auto& next_point = line.total_line[closest_index + 1];
                        if (next_point.x != curr_point.x) { // 避免除以零
                            line_slope = (next_point.y - curr_point.y) / (next_point.x - curr_point.x);
                        }
                    }

                    // 判断斜率之差是否小于 k_EFM_LF_IT_curv_limit
                    if (std::abs(extended_slope - line_slope) < k_EFM_LF_IT_curv_limit) {
                        float y_distance = std::abs(start_point.y - closest_y);
                        if (y_distance < min_y_distance_right) {
                            min_y_distance_right = y_distance;
                            right_line_ahead = line;
                        }
                    }
                }
            }
        }
    }

    //车道线拓扑连接
    // 连接 left_line_ahead 的第一个点和 bev_line_left 的最后一个点，并离散为 2.5m 一个的点
    BevLineInnerS left_topology_line = LF_ConnectIntersectionLine(bev_line_left, left_line_ahead, position_msg);
    EFMRefLinePoints left_discrete_points;
    if (!left_topology_line.total_line.empty()) {
        left_topology_line.id = 20001;//left
        left_topology_line.line_fusion_type = 3;
        left_topology_line.lane_location_type = 1;//left
        left_discrete_points = left_topology_line.total_line;

        // 将topology_line.total_line中x小于0的点放到topology_line.minus_line中，将x大于等于0的点放到topology_line.first_line中
        for (const auto& point : left_topology_line.total_line) {
            if (point.x < 0) {
                left_topology_line.minus_line.push_back(point);
            } else {
                left_topology_line.first_line.push_back(point);
            }
        }

        if (!left_topology_line.minus_line.empty()){

            left_topology_line.minus_valid = true;
        }else{
            left_topology_line.minus_valid = false;
        }

        if (!left_topology_line.first_line.empty()){

            left_topology_line.first_valid = true;
        }
        left_topology_line.md_qly = 0.9;
        left_topology_line.typ_chg_point = 20000;
    }

    // 连接 right_line_ahead 的第一个点和 bev_line_right 的最后一个点，并按照 x 方向间隔 2.5m 离散
    BevLineInnerS right_topology_line = LF_ConnectIntersectionLine(bev_line_right, right_line_ahead, position_msg);
    EFMRefLinePoints right_discrete_points;
    if (!right_topology_line.total_line.empty()) {
        right_topology_line.id = 20002;//right
        right_topology_line.line_fusion_type = 3;//topo line
        right_topology_line.lane_location_type = 2;//right
        right_discrete_points = right_topology_line.total_line;

        // 将topology_line.total_line中x小于0的点放到topology_line.minus_line中，将x大于等于0的点放到topology_line.first_line中
        for (const auto& point : right_topology_line.total_line) {
            if (point.x < 0) {
                right_topology_line.minus_line.push_back(point);
            } else {
                right_topology_line.first_line.push_back(point);
            }
        }

        if (!right_topology_line.minus_line.empty()){

            right_topology_line.minus_valid = true;
        }else{
            right_topology_line.minus_valid = false;
        }

        if (!right_topology_line.first_line.empty()){

            right_topology_line.first_valid = true;
        }
        right_topology_line.md_qly = 0.9;
        right_topology_line.typ_chg_point = 20000;
    }

    //删除bev_lines_中id为20000,20001,20002等的线
    if (!bev_lines_.empty()) {
        // 遍历 bev_lines_，删除 total_line 为空的线
        for (auto it = bev_lines_.begin(); it != bev_lines_.end();) {
            if (it->id == 20000 || it->id == 20001 || it->id == 20002 || it->id == 20003 | it->id == 20004) {
                it = bev_lines_.erase(it); // 删除当前元素，并更新迭代器
            } else {
                ++it; // 继续下一个元素
            }
        }
    }

    //填充到BEV中
    if (!left_topology_line.total_line.empty()){

        bev_lines_.push_back(left_topology_line);
    }else if (!left_topology_line_lastcycle.total_line.empty()){

        left_topology_line_lastcycle.id = 20003;//left zombie
        left_topology_line_lastcycle.line_fusion_type = 2;
        bev_lines_.push_back(left_topology_line_lastcycle);
        data_base_zombie_lines_.push_back(left_topology_line_lastcycle);
    }
    left_topology_line_lastcycle = left_topology_line;

    //填充到BEV中
    if (!right_topology_line.total_line.empty()){

        bev_lines_.push_back(right_topology_line);
    }else if (!right_topology_line_lastcycle.total_line.empty()){

        right_topology_line_lastcycle.id = 20004;//right zombie
        right_topology_line_lastcycle.line_fusion_type = 2;
        bev_lines_.push_back(right_topology_line_lastcycle);
        data_base_zombie_lines_.push_back(right_topology_line_lastcycle);
    }
    right_topology_line_lastcycle = right_topology_line;

}

// 计算中心线并延长
EFMRefLinePoints LaneFusion::IT_calculateExtendedCenterLine(

    const LaneFusionPointsType& left_points,
    const LaneFusionPointsType& right_points,
    float extension_length) {

    EFMRefLinePoints center_points;

    // 找到有效点并计算中心点
    size_t num_points = std::min(left_points.size(), right_points.size());
    for (size_t i = 0; i < num_points; ++i) {
        const LaneFusionPoint& left_point = left_points[i];
        const LaneFusionPoint& right_point = right_points[i];

        // 判断两点是否有效
        if (left_point.valid && right_point.valid) {
            EFMPoint center_point;
            center_point.x = (left_point.x + right_point.x) / 2.0;
            center_point.y = (left_point.y + right_point.y) / 2.0;
            center_points.push_back(center_point);
        }
    }

    // 如果没有有效点，直接返回空结果
    if (center_points.size() < 2) {
        return {};
    }

    // 根据最后两个点计算方向向量
    EFMPoint last_point = center_points[center_points.size() - 1];
    EFMPoint second_last_point = center_points[center_points.size() - 2];
    float dx = last_point.x - second_last_point.x;
    float dy = last_point.y - second_last_point.y;

    // 归一化方向向量
    float magnitude = sqrt(dx * dx + dy * dy);
    dx /= magnitude;
    dy /= magnitude;

    // 按照 x 方向每隔 2.5m 采样点进行延伸
    float remaining_length = extension_length;
    while (remaining_length > 0) {
        EFMPoint extended_point;
        extended_point.x = last_point.x + 2.5; // 仅在 x 方向延伸
        extended_point.y = last_point.y;      // y 方向保持不变
        center_points.push_back(extended_point);

        // 更新最后点和剩余长度
        last_point = extended_point;
        remaining_length -= 2.5;
    }

    return center_points;
}

void LaneFusion::LF_LineBodyToWGS84(BevLineInnerS& line_, const DoublePosePoint& ego_pos_wgs84){
    CommonTool::CoordinateTool* coordinate_tool_instance = CommonTool::CoordinateTool::GetInstance();

    std::vector<Point2Dd> line_body;
    std::vector<Point2Dd> line_wgs84;

    for (const auto& point : line_.total_line) {
        Point2Dd body_point;
        body_point.x = point.x;  // 将 current_line 中的 x 赋值给 line_body 的 x
        body_point.y = point.y;  // 将 current_line 中的 y 赋值给 line_body 的 y
        line_body.push_back(body_point);
    }

    coordinate_tool_instance->LineBodyToWGS84(line_body, ego_pos_wgs84, line_wgs84);

    for (const auto& point : line_wgs84) {
        GPSLinePoint body_point;
        body_point.Longitude = point.x;  // 将 current_line 中的 x 赋值给 line_body 的 x
        body_point.Latitude = point.y;  // 将 current_line 中的 y 赋值给 line_body 的 y
        line_.lane_fusion_process_info.total_line_gps_points.push_back(body_point);
    }
}

void LaneFusion::LF_LineWGS84ToBody(BevLineInnerS& line_, const DoublePosePoint& ego_pos_wgs84){

    CommonTool::CoordinateTool* coordinate_tool_instance = CommonTool::CoordinateTool::GetInstance();

    if (line_.id != 0) {

        std::vector<Point2Dd> line_body;
        std::vector<Point2Dd> line_wgs84;

        for (const auto& point : line_.lane_fusion_process_info.total_line_gps_points) {
            Point2Dd wgs84_point;
            wgs84_point.x = point.Longitude;  // 将 current_line 中的 x 赋值给 line_body 的 x
            wgs84_point.y = point.Latitude;  // 将 current_line 中的 y 赋值给 line_body 的 y
            line_wgs84.push_back(wgs84_point);
        }

        coordinate_tool_instance->LineWGS84ToBody(line_wgs84, ego_pos_wgs84, line_body);

        // 清空原来的点
        line_.total_line.clear();

        // 插入补偿后的点
        for (const auto& point : line_body) {
            EFMPoint body_point;
            body_point.x = point.x;  // 将 current_line 中的 x 赋值给 line_body 的 x
            body_point.y = point.y;  // 将 current_line 中的 y 赋值给 line_body 的 y
            line_.total_line.push_back(body_point);
        }
    }
}

BevLineInnerS LaneFusion::LF_ConnectIntersectionLine(const BevLineInnerS& line_location, const BevLineInnerS& line_ahead, const DoublePosePoint& ego_pos_wgs84){

    BevLineInnerS line_res = {};

    if (!line_ahead.total_line.empty() && !line_location.total_line.empty() && line_location.total_line.size() > 1 && line_ahead.total_line.size() > 1) {
        const EFMPoint& end_point = line_ahead.total_line.front();
        const EFMPoint& start_point = line_location.total_line.back();

        // 获取起点的前一个点和终点的后一个点
        const EFMPoint& start_prev_point = line_location.total_line[line_location.total_line.size() - 2];
        const EFMPoint& end_next_point = line_ahead.total_line[1];

        // 定义贝塞尔曲线的控制点
        EFMPoint control_point1, control_point2;

        // 控制点1：基于起点和前一个点计算
        control_point1.x = start_point.x + 0.5f * (start_point.x - start_prev_point.x);
        control_point1.y = start_point.y + 0.5f * (start_point.y - start_prev_point.y);

        // 控制点2：基于终点和后一个点计算
        control_point2.x = end_point.x + 0.5f * (end_next_point.x - end_point.x);
        control_point2.y = end_point.y + 0.5f * (end_next_point.y - end_point.y);

        // 生成贝塞尔曲线上的离散点
        float x_interval = 2.5f; // x 方向的间隔
        float last_x = start_point.x; // 记录上一个点的 x 坐标

        size_t num_segments = 100; // 分段数，越大曲线越平滑

        // 清空结果并插入起点
        line_res.total_line.clear();
        line_res.total_line.push_back(start_point);

        for (size_t i = 1; i < num_segments; ++i) { // 从 1 开始，避免重复插入起点
            float t = static_cast<float>(i) / num_segments; // 参数 t，范围 [0, 1]

            // 三次贝塞尔曲线公式
            float one_minus_t = 1.0f - t;
            EFMPoint discrete_point;
            discrete_point.x = one_minus_t * one_minus_t * one_minus_t * start_point.x +
                            3 * one_minus_t * one_minus_t * t * control_point1.x +
                            3 * one_minus_t * t * t * control_point2.x +
                            t * t * t * end_point.x;

            discrete_point.y = one_minus_t * one_minus_t * one_minus_t * start_point.y +
                            3 * one_minus_t * one_minus_t * t * control_point1.y +
                            3 * one_minus_t * t * t * control_point2.y +
                            t * t * t * end_point.y;

            // 按 x 方向每隔 2.5m 取一个点
            if (std::abs(discrete_point.x - last_x) >= x_interval) {
                line_res.total_line.push_back(discrete_point);
                last_x = discrete_point.x; // 更新上一个点的 x 坐标
            }
        }

        // 插入终点
        line_res.total_line.push_back(end_point);
    }

    if (!line_res.total_line.empty()){

        LF_LineBodyToWGS84(line_res, ego_pos_wgs84);
    }

    return line_res;
}

void LaneFusion::inhibit_out_road_edge_line(std::vector<BevLineInnerS>& bev_lines_) {

    if (!bev_lines_.empty()) {

        for (auto& current_line : bev_lines_){
            LaneFusionPointsConvert(current_line); 
        }
        
        float closed_left_edge = 100.0f;
        float closed_right_edge = 100.0f;
        BevLineInnerS left_edge_line = {};
        BevLineInnerS right_edge_line = {};

        // 找距离自车最近的边缘线
        for (auto& line : bev_lines_) {
            // 找距离自车最近的边缘线--左侧
            if (line.lane_location_type == 21 && line.lane_fusion_process_info.LaneFusionPoints[40].valid) {
                if (fabsf(line.lane_fusion_process_info.LaneFusionPoints[40].y) < fabsf(closed_left_edge)) {
                    closed_left_edge = line.lane_fusion_process_info.LaneFusionPoints[40].y;
                    left_edge_line = line;
                }
            }
            // 找距离自车最近的边缘线--右侧
            if (line.lane_location_type == 22 && line.lane_fusion_process_info.LaneFusionPoints[40].valid) {
                if (fabsf(line.lane_fusion_process_info.LaneFusionPoints[40].y) < fabsf(closed_right_edge)) {
                    closed_right_edge = line.lane_fusion_process_info.LaneFusionPoints[40].y;
                    right_edge_line = line;
                }
            }
        }

        // std::cout << "left_edge_line:" << left_edge_line.id << std::endl;
        // std::cout << "right_edge_line:" << right_edge_line.id << std::endl;

        // 检查 left_edge_line 和 right_edge_line 是否有值
        bool has_left_edge = !left_edge_line.lane_fusion_process_info.LaneFusionPoints.empty();
        bool has_right_edge = !right_edge_line.lane_fusion_process_info.LaneFusionPoints.empty();

        // std::cout << "has_ledge:" << has_left_edge << " | " << has_right_edge << std::endl;

        // 遍历 bev_lines_，删除位于 left_edge_line 左边或 right_edge_line 右边的线
        for (auto it = bev_lines_.begin(); it != bev_lines_.end();) {

            bool should_erase = false;

            //检查与左侧是否有overlap
            bool is_left_overlap = false;
            for (size_t j = 0; j < left_edge_line.lane_fusion_process_info.LaneFusionPoints.size(); ++j) {
                if (left_edge_line.lane_fusion_process_info.LaneFusionPoints[j].valid && it->lane_fusion_process_info.LaneFusionPoints[j].valid) {
                    is_left_overlap = true;
                    break;
                }
            }

            // 检查是否在 left_edge_line 左边
            if (has_left_edge && is_left_overlap && it->lane_location_type != 21 && it->lane_location_type != 22) {
                bool is_left_out = true;

                // 计算 left_edge_line 中 valid 的最小和最大索引
                size_t left_min_index = left_edge_line.lane_fusion_process_info.LaneFusionPoints.size();
                size_t left_max_index = 0;
                for (size_t j = 0; j < left_edge_line.lane_fusion_process_info.LaneFusionPoints.size(); ++j) {
                    if (left_edge_line.lane_fusion_process_info.LaneFusionPoints[j].valid) {
                        left_min_index = std::min(left_min_index, j);
                        left_max_index = std::max(left_max_index, j);
                    }
                }

                for (size_t i = 0; i < it->lane_fusion_process_info.LaneFusionPoints.size(); ++i) {
                    if (it->lane_fusion_process_info.LaneFusionPoints[i].valid) {
                        if (i >= left_min_index && i <= left_max_index) {

                            // 当 i 在 left_min_index 和 left_max_index 之间时
                            if (it->lane_fusion_process_info.LaneFusionPoints[i].y < left_edge_line.lane_fusion_process_info.LaneFusionPoints[i].y) {

                                is_left_out = false;
                                break;
                            }
                        } else if (i < left_min_index) {

                            // 当 i 小于 left_min_index 时
                            if ((left_min_index - i > 25 && left_min_index > 35) || 
                                it->lane_fusion_process_info.LaneFusionPoints[left_min_index].y < left_edge_line.lane_fusion_process_info.LaneFusionPoints[left_min_index].y) {

                                is_left_out = false;
                                break;
                            }
                        } else if (i > left_max_index) {

                            // 当 i 大于 left_max_index 时
                            if (i - left_max_index > 20 && left_max_index < 50 || 
                                it->lane_fusion_process_info.LaneFusionPoints[left_max_index].y < left_edge_line.lane_fusion_process_info.LaneFusionPoints[left_max_index].y) {

                                is_left_out = false;
                                break;
                            }
                        }
                    }
                }
                if (is_left_out) {
                    should_erase = true;
                }
            }

            //检查与右侧是否有overlap
            bool is_right_overlap = false;
            for (size_t j = 0; j < right_edge_line.lane_fusion_process_info.LaneFusionPoints.size(); ++j) {
                if (right_edge_line.lane_fusion_process_info.LaneFusionPoints[j].valid && it->lane_fusion_process_info.LaneFusionPoints[j].valid) {
                    is_right_overlap = true;
                    break;
                }
            }

            // 检查是否在 right_edge_line 右边
            if (has_right_edge && is_right_overlap && it->lane_location_type != 21 && it->lane_location_type != 22) {
                bool is_right_out = true;

                // 计算 right_edge_line 中 valid 的最小和最大索引
                size_t right_min_index = right_edge_line.lane_fusion_process_info.LaneFusionPoints.size();
                size_t right_max_index = 0;
                for (size_t j = 0; j < right_edge_line.lane_fusion_process_info.LaneFusionPoints.size(); ++j) {
                    if (right_edge_line.lane_fusion_process_info.LaneFusionPoints[j].valid) {
                        right_min_index = std::min(right_min_index, j);
                        right_max_index = std::max(right_max_index, j);
                    }
                }

                // for (size_t i = 0; i < it->lane_fusion_process_info.LaneFusionPoints.size(); ++i) {
                //     if (it->lane_fusion_process_info.LaneFusionPoints[i].valid) {
                //         if (i < right_min_index || i > right_max_index ||
                //             it->lane_fusion_process_info.LaneFusionPoints[i].y > right_edge_line.lane_fusion_process_info.LaneFusionPoints[i].y) {
                //             is_right_out = false;
                //             break;
                //         }
                //     }
                // }

                for (size_t i = 0; i < it->lane_fusion_process_info.LaneFusionPoints.size(); ++i) {
                    if (it->lane_fusion_process_info.LaneFusionPoints[i].valid) {

                        if (i >= right_min_index && i <= right_max_index) {

                            // 当 i 在 right_min_index 和 right_max_index 之间时
                            if (it->lane_fusion_process_info.LaneFusionPoints[i].y > right_edge_line.lane_fusion_process_info.LaneFusionPoints[i].y) {

                                is_right_out = false;
                                break;
                            }
                        } else if (i < right_min_index) {

                            // 当 i 小于 right_min_index 时
                            if ((right_min_index - i > 25 && right_min_index > 35) || 
                                it->lane_fusion_process_info.LaneFusionPoints[right_min_index].y > right_edge_line.lane_fusion_process_info.LaneFusionPoints[right_min_index].y) {

                                is_right_out = false;
                                break;
                            }
                        } else if (i > right_max_index) {
                            // 当 i 大于 right_max_index 时
                            if ((i - right_max_index > 20 && right_max_index < 50) || 
                                it->lane_fusion_process_info.LaneFusionPoints[right_max_index].y > right_edge_line.lane_fusion_process_info.LaneFusionPoints[right_max_index].y) {
                                is_right_out = false;
                                break;
                            }
                        }
                    }
                }

                if (is_right_out) {
                    should_erase = true;
                }
                // std::cout << "min_max_index " << right_min_index << " | " << right_max_index << std::endl;

            }

            // 删除或继续
            if (should_erase) {
                // std::cout << "erase line " << it->id << std::endl;
                it = bev_lines_.erase(it); // 删除当前元素，并更新迭代器
            } else {
                ++it; // 继续下一个元素
            }
        }
    }
}
    
}//namespace
