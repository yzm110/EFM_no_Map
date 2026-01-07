#include <iostream>
#include <unistd.h>

#include "ComponentEFMTaskApp.h"
#include "log_manager.h"

using namespace frrt::task;

void ComponentEFMTaskApp::Init()
{
    LOG_DEBUG("ComponentEFMTaskApp init test ok");
}

void ComponentEFMTaskApp::Release()
{
    LOG_DEBUG("ComponentEFMTaskApp release test ok");
}

const char* ComponentEFMTaskApp::Name()
{
    return "ComponentEFMTaskApp";
}

ComponentEFMTaskApp::ComponentEFMTaskApp(){
    wrapper_input_ = std::make_shared<NoMapEFM::WrapperInput>();
    lane_fusion_ = std::make_shared<NoMapEFM::LaneFusion>();
    lane_model_ = std::make_shared<NoMapEFM::LaneModel>();
    local_map_ = std::make_shared<NoMapEFM::LocalMap>();
    sd_map_scene_ = std::make_shared<NoMapEFM::SdMapScene>();
    wrapper_output_ = std::make_shared<NoMapEFM::WrapperOutput>();
}

bool ComponentEFMTaskApp::LoadConfig(const std::string file_name){
    if (config_file_.load(file_name)){
        p_production_parameter = std::stoi(config_file_.getValue("ProductionMode", "mode"));
        wide_lane_width_start = std::stoi(config_file_.getValue("centerline", "wide_lane_width_start"));
        wide_lane_width_end = std::stoi(config_file_.getValue("centerline", "wide_lane_width_end"));
        wide_lane_width_step = std::stoi(config_file_.getValue("centerline", "wide_lane_width_step"));
        merge_lane_start_width = std::stoi(config_file_.getValue("centerline", "merge_lane_start_width"));
        merge_lane_end_width = std::stoi(config_file_.getValue("centerline", "merge_lane_end_width"));
        split_lane_start_width = std::stoi(config_file_.getValue("centerline", "split_lane_start_width"));
        split_lane_end_width = std::stoi(config_file_.getValue("centerline", "split_lane_end_width"));
        virtual_merge_end_width = std::stoi(config_file_.getValue("centerline", "virtual_merge_end_width"));
        virtual_split_start_width = std::stoi(config_file_.getValue("centerline", "virtual_split_start_width"));
        offset_line_width = std::stoi(config_file_.getValue("centerline", "offset_line_width"));
        lane_offset_step_length = std::stoi(config_file_.getValue("centerline", "lane_offset_step_length"));
        car_offset_split = std::stoi(config_file_.getValue("centerline", "car_offset_split"));
        bev_max_offset = std::stoi(config_file_.getValue("centerline", "bev_max_offset"));
        k_EFM_enable_intersection_topology = std::stoi(config_file_.getValue("lanefusion", "k_EFM_enable_intersection_topology"));
        return true;
    }

    return false;
}

int32_t ComponentEFMTaskApp::compute(const datatype_perception::s_PerceptionFront_t& perception_data, 
                                     const datatype_fusion::s_FusionLanes_t& fusion_lane_data, 
                                     const datatype_ehp::s_SDPosition_t& sd_position_data, 
                                     const datatype_ehp::s_SDPaths_t& sd_paths_data, 
                                     const datatype_ehp::s_SDLinks_t& sd_links_data, 
                                     datatype_efm::s_MapLane_t& output_map_lane)
{
    LOG_TRACE("ComponentEFMTaskApp compute test ok");
    output_map_lane.header.cntr = sd_position_data.Header.cntr;
    // std::cout << __FILE__ << "," << __LINE__ << "sd_position_data.PositionTimeStamp " << sd_position_data.PositionTimeStamp<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << "sd_position_data.Header.timestamp " << sd_position_data.Header.timestamp<<std::endl;
    output_map_lane.header.timestamp = sd_position_data.PositionTimeStamp;
    LOG_DEBUG(std::string("efm_counter===================================: ") + std::to_string(output_map_lane.header.cntr));
    // std::cout << " " << std::endl;
    // std::cout << "fusion_lane_data.header.cntr " << fusion_lane_data.header.cntr<<std::endl;
    // std::cout << "sd_position_data.header.cntr " << sd_position_data.Header.cntr<<std::endl;
    // ZTEXT("EFM_INFO", "efm_counter: ", 10, 24, "efm_counter: {}", efm_counter);
    // ZTEXT("EFM_INFO", "position_counter: ", 16, 21, "position_counter: {}", position_msg.header.cntr);
#ifdef EM_COUT
    std::cout << __FILE__ << "," << __LINE__ << "," << "EFM is running " << std::endl;
#endif
    PlotBevLines(fusion_lane_data);
    //给EHP_loc_, EHP_paths_, EHP_links_赋值
    GetSDInfo(sd_position_data, sd_paths_data, sd_links_data);
    CreateSDDateIndex();
    //读取配置
    std::string config_file = "./conf/user/efm_config.ini";
#ifdef __aarch64__
    config_file = "/mnt/appfs/pack/erte/efm_exe/etc/user/efm_config.ini";
#endif
    LOG_DEBUG(std::string("sd_position_data.cnt: ") + std::to_string(sd_position_data.Header.cntr));
    LOG_DEBUG(std::string("config_file: ") + config_file);
    if(LoadConfig(config_file) == false){
        LOG_DEBUG("config_file return");
	    // return 0;
    }

    //产品功能切换
    // p_production_mode

    //处理输入
    if(wrapper_input_->Execute(EHP_links_, EHP_paths_, EHP_loc_, sd_index_,
                                fusion_lane_data, bev_lines_, path_point_body_with_offset_, road_split_dir_vec_) == false){
        LOG_DEBUG("wrapper_input_ return");
        //error code log
        return 0;
    }
    // std::cout << __FILE__ << "," << __LINE__ << "bev_lines_.size() " << bev_lines_.size()<<std::endl;
    //L2输出
    if(wrapper_output_->Execute(bev_lines_, output_map_lane) == false){
        LOG_DEBUG("wrapper_output_ return");
        //error code log
        return 0;
    }
    LOG_DEBUG(std::string("p_production_parameter: ") + std::to_string(p_production_parameter));
    // if(p_production_parameter == 0){
    //     //  return 0;
    // }
    if(NotNOA()== true){
        return 0; 
    }
    //前后帧融合
    // 历史的车道线数据拿哪个？，历史帧放在lane_fusion模块, position_msg->heading旋转， 经纬度变化做平移
    if(lane_fusion_->Execute(bev_lines_, sd_position_data, fusion_lane_data) == false){
        LOG_DEBUG("lane_fusion_ return");
        //error code log
        return 0;
    }

    //sd信号处理
    if(sd_map_scene_->Execute(EHP_loc_, EHP_paths_, EHP_links_, EHP_ele_group_, sd_index_) == false){
        LOG_DEBUG("sd_map_scene_ return");
        //error code log
        return 0;
    }
    auto& lane_loc = sd_map_scene_->GetLoc();  

    //车道线拼接，排序，车道生成
    if(lane_model_->Execute(bev_lines_, bev_lane_group_set_, lane_loc, EHP_ele_group_, loc_res_info_,last_cycle_info_, sd_index_, path_point_body_with_offset_, road_split_dir_vec_) == false){
        LOG_DEBUG("lane_model_ return");
        //error code log
        return 0;
    } 

    //中心线生成，车道级定位等, 
    NodeInfo node_info = NodeInfo{};
    if(local_map_->Execute(bev_lane_group_set_, EHP_ele_group_, node_info, lane_loc, EHP_paths_) == false){
        LOG_DEBUG("local_map_ return");
        //error code log
        return 0;
    } 

    //输出
    if(wrapper_output_->Execute(bev_lane_group_set_, EHP_ele_group_, EHP_links_,local_map_->GetEgoLaneIdx(), local_map_->GetLeftLaneIdx(), local_map_->GetRightLaneIdx(),local_map_->GetLeftLeftLaneIdx(),local_map_->GetRightRightLaneIdx(), 
                                lane_loc, local_map_->GetRefLaneIdx(), node_info, EHP_paths_, loc_res_info_, output_map_lane) == false){
        LOG_DEBUG("2wrapper_output_ return");
        //error code log
        return 0;
    }

    const BevLaneElementGroup *ego_bev_lane_group = nullptr;
    for(auto& lane_group : bev_lane_group_set_){
        if(lane_group.second.size() > 0){
            if(lane_group.first == 0){
                ego_bev_lane_group = &lane_group.second;
            }
        }
    } 
    StoreInfo(lane_loc, ego_bev_lane_group);
    // LOGE("EnvironmentModel::after plot bev");
    // std::cout << "end:!!!!!!!" << std::endl;
    return 0;
}

void ComponentEFMTaskApp::PlotBevLines(const datatype_fusion::s_FusionLanes_t& lanes_msg){
if (lanes_msg.FusionLanes.Array_Lanes_50[0].valid) {
    std::vector<float> lane0;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[0].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[0].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[0].MinusEndPoint;a += 2.5f) {
            lane0.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[0].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[0].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[0].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[0].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[0].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[0].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[0].FirstStartPoint) continue;
            lane0.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[0].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[0].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[0].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[0].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[0].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[0].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[0].SecStartPoint) continue;
            lane0.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[0].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[0].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[0].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[0].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[0].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[0].points[i].y) < 1e-6) break;
    //     lane0.push_back(lanes_msg.FusionLanes.Array_Lanes_50[0].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[0].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane0, laney);
    if(lane0.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id0: ", lane0[0], laney[0], "id0: {}", lanes_msg.FusionLanes.Array_Lanes_50[0].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id0: ", 0, -100, "id0: {}", lanes_msg.FusionLanes.Array_Lanes_50[0].CamObjId);
    }    
} else {
    std::vector<float> lane0;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane0, laney);
    // ZTEXT("BEV_LINE", "id0: ", 0, -100, "id0: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[1].valid) {
    std::vector<float> lane1;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[1].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[1].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[1].MinusEndPoint;a += 2.5f) {
            lane1.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[1].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[1].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[1].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[1].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[1].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[1].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[1].FirstStartPoint) continue;
            lane1.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[1].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[1].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[1].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[1].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[1].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[1].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[1].SecStartPoint) continue;
            lane1.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[1].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[1].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[1].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[1].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[1].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[1].points[i].y) < 1e-6) break;
    //     lane1.push_back(lanes_msg.FusionLanes.Array_Lanes_50[1].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[1].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane1, laney);
    if(lane1.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id1: ", lane1[0], laney[0], "id1: {}", lanes_msg.FusionLanes.Array_Lanes_50[1].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id1: ", 0, -100, "id1: {}", lanes_msg.FusionLanes.Array_Lanes_50[1].CamObjId);
    }
} else {
    std::vector<float> lane1;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane1, laney);
    // ZTEXT("BEV_LINE", "id1: ", 0, -100, "id1: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[2].valid) {
    std::vector<float> lane2;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[2].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[2].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[2].MinusEndPoint;a += 2.5f) {
            lane2.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[2].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[2].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[2].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[2].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[2].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[2].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[2].FirstStartPoint) continue;
            lane2.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[2].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[2].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[2].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[2].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[2].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[2].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[2].SecStartPoint) continue;
            lane2.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[2].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[2].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[2].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[2].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[2].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[2].points[i].y) < 1e-6) break;
    //     lane2.push_back(lanes_msg.FusionLanes.Array_Lanes_50[2].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[2].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane2, laney);
    
    if(lane2.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id2: ", lane2[0], laney[0], "id2: {}", lanes_msg.FusionLanes.Array_Lanes_50[2].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id2: ", 0, -100, "id2: {}", lanes_msg.FusionLanes.Array_Lanes_50[2].CamObjId);
    }
} else {
    std::vector<float> lane2;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane2, laney);
    // ZTEXT("BEV_LINE", "id2: ", 0, -100, "id2: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[3].valid) {
    std::vector<float> lane3;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[3].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[3].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[3].MinusEndPoint;a += 2.5f) {
            lane3.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[3].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[3].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[3].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[3].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[3].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[3].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[3].FirstStartPoint) continue;
            lane3.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[3].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[3].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[3].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[3].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[3].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[3].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[3].SecStartPoint) continue;
            lane3.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[3].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[3].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[3].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[3].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[3].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[3].points[i].y) < 1e-6) break;
    //     lane3.push_back(lanes_msg.FusionLanes.Array_Lanes_50[3].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[3].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane3, laney);
    
    if(lane3.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id3: ", lane3[0], laney[0], "id3: {}", lanes_msg.FusionLanes.Array_Lanes_50[3].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id3: ", 0, -100, "id3: {}", lanes_msg.FusionLanes.Array_Lanes_50[3].CamObjId);
    }
} else {
    std::vector<float> lane3;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane3, laney);
    // ZTEXT("BEV_LINE", "id3: ", 0, -100, "id3: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[4].valid) {
    std::vector<float> lane4;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[4].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[4].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[4].MinusEndPoint;a += 2.5f) {
            lane4.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[4].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[4].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[4].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[4].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[4].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[4].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[4].FirstStartPoint) continue;
            lane4.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[4].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[4].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[4].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[4].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[4].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[4].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[4].SecStartPoint) continue;
            lane4.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[4].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[4].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[4].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[4].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[4].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[4].points[i].y) < 1e-6) break;
    //     lane4.push_back(lanes_msg.FusionLanes.Array_Lanes_50[4].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[4].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane4, laney);
    
    if(lane4.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id4: ", lane4[0], laney[0], "id4: {}", lanes_msg.FusionLanes.Array_Lanes_50[4].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id4: ", 0, -100, "id4: {}", lanes_msg.FusionLanes.Array_Lanes_50[4].CamObjId);
    }
} else {
    std::vector<float> lane4;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane4, laney);
    // ZTEXT("BEV_LINE", "id4: ", 0, -100, "id4: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[5].valid) {
    std::vector<float> lane5;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[5].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[5].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[5].MinusEndPoint;a += 2.5f) {
            lane5.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[5].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[5].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[5].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[5].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[5].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[5].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[5].FirstStartPoint) continue;
            lane5.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[5].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[5].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[5].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[5].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[5].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[5].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[5].SecStartPoint) continue;
            lane5.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[5].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[5].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[5].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[5].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[5].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[5].points[i].y) < 1e-6) break;
    //     lane5.push_back(lanes_msg.FusionLanes.Array_Lanes_50[5].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[5].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane5, laney);
    
    if(lane5.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id5: ", lane5[0], laney[0], "id5: {}", lanes_msg.FusionLanes.Array_Lanes_50[5].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id5: ", 0, -100, "id5: {}", lanes_msg.FusionLanes.Array_Lanes_50[5].CamObjId);
    }
} else {
    std::vector<float> lane5;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane5, laney);
    // ZTEXT("BEV_LINE", "id5: ", 0, -100, "id5: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[6].valid) {
    std::vector<float> lane6;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[6].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[6].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[6].MinusEndPoint;a += 2.5f) {
            lane6.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[6].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[6].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[6].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[6].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[6].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[6].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[6].FirstStartPoint) continue;
            lane6.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[6].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[6].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[6].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[6].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[6].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[6].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[6].SecStartPoint) continue;
            lane6.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[6].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[6].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[6].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[6].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[6].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[6].points[i].y) < 1e-6) break;
    //     lane6.push_back(lanes_msg.FusionLanes.Array_Lanes_50[6].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[6].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane6, laney);
    
    if(lane6.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id6: ", lane6[0], laney[0], "id6: {}", lanes_msg.FusionLanes.Array_Lanes_50[6].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id6: ", 0, -100, "id6: {}", lanes_msg.FusionLanes.Array_Lanes_50[6].CamObjId);
    }
} else {
    std::vector<float> lane6;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane6, laney);
    // ZTEXT("BEV_LINE", "id6: ", 0, -100, "id6: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[7].valid) {
    std::vector<float> lane7;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[7].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[7].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[7].MinusEndPoint;a += 2.5f) {
            lane7.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[7].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[7].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[7].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[7].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[7].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[7].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[7].FirstStartPoint) continue;
            lane7.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[7].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[7].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[7].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[7].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[7].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[7].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[7].SecStartPoint) continue;
            lane7.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[7].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[7].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[7].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[7].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[7].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[7].points[i].y) < 1e-6) break;
    //     lane7.push_back(lanes_msg.FusionLanes.Array_Lanes_50[7].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[7].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane7, laney);
    
    if(lane7.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id7: ", lane7[0], laney[0], "id7: {}", lanes_msg.FusionLanes.Array_Lanes_50[7].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id7: ", 0, -100, "id7: {}", lanes_msg.FusionLanes.Array_Lanes_50[7].CamObjId);
    }
} else {
    std::vector<float> lane7;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane7, laney);
    // ZTEXT("BEV_LINE", "id7: ", 0, -100, "id7: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[8].valid) {
    std::vector<float> lane8;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[8].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[8].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[8].MinusEndPoint;a += 2.5f) {
            lane8.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[8].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[8].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[8].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[8].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[8].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[8].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[8].FirstStartPoint) continue;
            lane8.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[8].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[8].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[8].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[8].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[8].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[8].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[8].SecStartPoint) continue;
            lane8.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[8].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[8].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[8].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[8].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[8].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[8].points[i].y) < 1e-6) break;
    //     lane8.push_back(lanes_msg.FusionLanes.Array_Lanes_50[8].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[8].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane8, laney);
    
    if(lane8.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id8: ", lane8[0], laney[0], "id8: {}", lanes_msg.FusionLanes.Array_Lanes_50[8].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id8: ", 0, -100, "id8: {}", lanes_msg.FusionLanes.Array_Lanes_50[8].CamObjId);
    }
} else {
    std::vector<float> lane8;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane8, laney);
    // ZTEXT("BEV_LINE", "id8: ", 0, -100, "id8: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[9].valid) {
    std::vector<float> lane9;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[9].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[9].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[9].MinusEndPoint;a += 2.5f) {
            lane9.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[9].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[9].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[9].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[9].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[9].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[9].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[9].FirstStartPoint) continue;
            lane9.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[9].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[9].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[9].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[9].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[9].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[9].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[9].SecStartPoint) continue;
            lane9.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[9].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[9].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[9].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[9].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[9].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[9].points[i].y) < 1e-6) break;
    //     lane9.push_back(lanes_msg.FusionLanes.Array_Lanes_50[9].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[9].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane9, laney);
    
    if(lane9.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id9: ", lane9[0], laney[0], "id9: {}", lanes_msg.FusionLanes.Array_Lanes_50[9].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id9: ", 0, -100, "id9: {}", lanes_msg.FusionLanes.Array_Lanes_50[9].CamObjId);
    }
} else {
    std::vector<float> lane9;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane9, laney);
    // ZTEXT("BEV_LINE", "id9: ", 0, -100, "id9: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[10].valid) {
    std::vector<float> lane10;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[10].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[10].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[10].MinusEndPoint;a += 2.5f) {
            lane10.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[10].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[10].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[10].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[10].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[10].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[10].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[10].FirstStartPoint) continue;
            lane10.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[10].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[10].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[10].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[10].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[10].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[10].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[10].SecStartPoint) continue;
            lane10.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[10].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[10].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[10].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[10].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[10].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[10].points[i].y) < 1e-6) break;
    //     lane10.push_back(lanes_msg.FusionLanes.Array_Lanes_50[10].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[10].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane10, laney);
    
    if(lane10.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id10: ", lane10[0], laney[0], "id10: {}", lanes_msg.FusionLanes.Array_Lanes_50[10].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id10: ", 0, -100, "id10: {}", lanes_msg.FusionLanes.Array_Lanes_50[10].CamObjId);
    }
} else {
    std::vector<float> lane10;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane10, laney);
    // ZTEXT("BEV_LINE", "id10: ", 0, -100, "id10: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[11].valid) {
    std::vector<float> lane11;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[11].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[11].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[11].MinusEndPoint;a += 2.5f) {
            lane11.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[11].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[11].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[11].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[11].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[11].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[11].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[11].FirstStartPoint) continue;
            lane11.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[11].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[11].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[11].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[11].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[11].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[11].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[11].SecStartPoint) continue;
            lane11.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[11].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[11].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[11].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[11].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[11].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[11].points[i].y) < 1e-6) break;
    //     lane11.push_back(lanes_msg.FusionLanes.Array_Lanes_50[11].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[11].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane11, laney);
    
    if(lane11.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id11: ", lane11[0], laney[0], "id11: {}", lanes_msg.FusionLanes.Array_Lanes_50[11].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id11: ", 0, -100, "id11: {}", lanes_msg.FusionLanes.Array_Lanes_50[11].CamObjId);
    }
} else {
    std::vector<float> lane11;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane11, laney);
    // ZTEXT("BEV_LINE", "id11: ", 0, -100, "id11: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[12].valid) {
    std::vector<float> lane12;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[12].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[12].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[12].MinusEndPoint;a += 2.5f) {
            lane12.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[12].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[12].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[12].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[12].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[12].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[12].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[12].FirstStartPoint) continue;
            lane12.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[12].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[12].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[12].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[12].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[12].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[12].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[12].SecStartPoint) continue;
            lane12.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[12].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[12].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[12].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[12].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[12].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[12].points[i].y) < 1e-6) break;
    //     lane12.push_back(lanes_msg.FusionLanes.Array_Lanes_50[12].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[12].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane12, laney);
    
    if(lane12.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id12: ", lane12[0], laney[0], "id12: {}", lanes_msg.FusionLanes.Array_Lanes_50[12].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id12: ", 0, -100, "id12: {}", lanes_msg.FusionLanes.Array_Lanes_50[12].CamObjId);
    }
} else {
    std::vector<float> lane12;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane12, laney);
    // ZTEXT("BEV_LINE", "id12: ", 0, -100, "id12: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[13].valid) {
    std::vector<float> lane13;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[13].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[13].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[13].MinusEndPoint;a += 2.5f) {
            lane13.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[13].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[13].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[13].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[13].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[13].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[13].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[13].FirstStartPoint) continue;
            lane13.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[13].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[13].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[13].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[13].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[13].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[13].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[13].SecStartPoint) continue;
            lane13.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[13].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[13].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[13].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[13].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[13].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[13].points[i].y) < 1e-6) break;
    //     lane13.push_back(lanes_msg.FusionLanes.Array_Lanes_50[13].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[13].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane13, laney);
    
    if(lane13.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id13: ", lane13[0], laney[0], "id13: {}", lanes_msg.FusionLanes.Array_Lanes_50[13].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id13: ", 0, -100, "id13: {}", lanes_msg.FusionLanes.Array_Lanes_50[13].CamObjId);
    }
} else {
    std::vector<float> lane13;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane13, laney);
    // ZTEXT("BEV_LINE", "id13: ", 0, -100, "id13: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[14].valid) {
    std::vector<float> lane14;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[14].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[14].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[14].MinusEndPoint;a += 2.5f) {
            lane14.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[14].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[14].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[14].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[14].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[14].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[14].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[14].FirstStartPoint) continue;
            lane14.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[14].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[14].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[14].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[14].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[14].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[14].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[14].SecStartPoint) continue;
            lane14.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[14].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[14].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[14].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[14].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[14].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[14].points[i].y) < 1e-6) break;
    //     lane14.push_back(lanes_msg.FusionLanes.Array_Lanes_50[14].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[14].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane14, laney);
    
    if(lane14.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id14: ", lane14[0], laney[0], "id14: {}", lanes_msg.FusionLanes.Array_Lanes_50[14].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id14: ", 0, -100, "id14: {}", lanes_msg.FusionLanes.Array_Lanes_50[14].CamObjId);
    }
} else {
    std::vector<float> lane14;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane14, laney);
    // ZTEXT("BEV_LINE", "id14: ", 0, -100, "id14: {}", 0);
}
if (lanes_msg.FusionLanes.Array_Lanes_50[15].valid) {
    std::vector<float> lane15;
    std::vector<float> laney;
    float a = int(lanes_msg.FusionLanes.Array_Lanes_50[15].MinusStartPoint / 2.5f) * 2.5f;
    if (lanes_msg.FusionLanes.Array_Lanes_50[15].MinusStartPoint < 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[15].MinusEndPoint;a += 2.5f) {
            lane15.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[15].C3Minus * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[15].C2Minus * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[15].C1Minus * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[15].C0Minus);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[15].C0First) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[15].FirstEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[15].FirstStartPoint) continue;
            lane15.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[15].C3First * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[15].C2First * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[15].C1First * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[15].C0First);
        }
    }
    if (abs(lanes_msg.FusionLanes.Array_Lanes_50[15].C0Sec) > 0.0f) {
        for (; a <= lanes_msg.FusionLanes.Array_Lanes_50[15].SecEndPoint;a += 2.5f) {
            if (a < lanes_msg.FusionLanes.Array_Lanes_50[15].SecStartPoint) continue;
            lane15.push_back(a);
            laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[15].C3Sec * a * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[15].C2Sec * a * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[15].C1Sec * a +
                            lanes_msg.FusionLanes.Array_Lanes_50[15].C0Sec);
        }
    }
    // for (int i = 0; i < 400; ++i) {
    //     if (fabs(lanes_msg.FusionLanes.Array_Lanes_50[15].points[i].x) < 1e-6 &&
    //         fabs(lanes_msg.FusionLanes.Array_Lanes_50[15].points[i].y) < 1e-6) break;
    //     lane15.push_back(lanes_msg.FusionLanes.Array_Lanes_50[15].points[i].x);
    //     laney.push_back(lanes_msg.FusionLanes.Array_Lanes_50[15].points[i].y);
    // }
    // ZPLOTXYF("BEV_LINE", ".blue4", lane15, laney);
    
    if(lane15.size()>0 && laney.size()>0){
        // ZTEXT("BEV_LINE", "id15: ", lane15[0], laney[0], "id15: {}", lanes_msg.FusionLanes.Array_Lanes_50[15].CamObjId);
    }else{
        // ZTEXT("BEV_LINE", "id15: ", 0, -100, "id15: {}", lanes_msg.FusionLanes.Array_Lanes_50[15].CamObjId);
    }
} else {
    std::vector<float> lane15;
    std::vector<float> laney;
    // ZPLOTXYF("BEV_LINE", ".blue4", lane15, laney);
    // ZTEXT("BEV_LINE", "id15: ", 0, -100, "id15: {}", 0);
}
}

void ComponentEFMTaskApp::GetSDInfo(const datatype_ehp::s_SDPosition_t& sd_position_data, 
                                    const datatype_ehp::s_SDPaths_t& sd_paths_data, 
                                    const datatype_ehp::s_SDLinks_t& sd_links_data){
    //location
    EHP_loc_ = NoMapEFM::SEhpOutputLoc();
    EHP_loc_.id_ = sd_position_data.PositionID;
    EHP_loc_.lat_ = sd_position_data.Lat;
    EHP_loc_.lon_ = sd_position_data.Lon;
    EHP_loc_.heading_ = sd_position_data.Heading;
    EHP_loc_.speed_ = sd_position_data.Speed;
    EHP_loc_.path_offset_id_ = sd_position_data.PathOffsetID;
    EHP_loc_.offset_ = sd_position_data.Offset;
    EHP_loc_.link_id_ = sd_position_data.LinkID;
    EHP_loc_.path_id_ = sd_position_data.PathID;
    EHP_loc_.time_stmp_ = sd_position_data.PositionTimeStamp;

    //paths
    EHP_paths_.ehp_output_path_list.clear();
    for (const auto& path : sd_paths_data.Vec_Paths) {
        NoMapEFM::SEhpOutputPath ehp_output_path;
        ehp_output_path.path_id_ = path.PathID;
        ehp_output_path.parent_path_id_ = path.ParentPathID;
        ehp_output_path.InParentOffset_ = path.InParentOffset;
        ehp_output_path.is_end_ = path.IsEnd;

        for (const auto& special_data : path.Vec_SpecialDatas) {
            NoMapEFM::SSpecialData sd;
            sd.type_ = static_cast<NoMapEFM::PATH_SPECIAL_TYPE>(special_data.SpecialDataType);
            sd.dir_ = special_data.SpecialDataDir;
            sd.s_offset_ = special_data.SOffset;
            sd.e_offset_ = special_data.EOffset;
            sd.child_path_ = special_data.ChildPath;
            ehp_output_path.special_datas_.push_back(sd);
        }

        for (const auto& link_offset : path.Vec_LinkOffsets) {
            NoMapEFM::SLinkOffset lo;
            lo.path_offset_id_ = link_offset.PathOffsetID;
            lo.s_offset_ = link_offset.SOffset;
            lo.e_offset_ = link_offset.EOffset;
            ehp_output_path.link_offsets_.push_back(lo);
        }
        EHP_paths_.ehp_output_path_list.push_back(ehp_output_path);
    }
    //links
    EHP_links_.ehp_output_link_list.clear();
    for (const auto& link : sd_links_data.Vec_Links) {
        NoMapEFM::SEhpOutputLink ehp_output_link;
        ehp_output_link.id_ = link.ID;
        ehp_output_link.path_id_ = link.PathID;
        ehp_output_link.path_offset_id_ = link.PathOffsetID;
        ehp_output_link.s_offset_ = link.SOffset;
        ehp_output_link.e_offset_ = link.EOoffset;
        ehp_output_link.frc_ = static_cast<NoMapEFM::LINK_FRC>(link.Frc);
        ehp_output_link.fow_ = static_cast<NoMapEFM::LINK_FOW>(link.Fow);
        ehp_output_link.struct_ = static_cast<NoMapEFM::LINK_STRUCT>(link.LinkStruct);
        ehp_output_link.dir_ = static_cast<NoMapEFM::LINK_DIRECTION>(link.LinkDir);
        ehp_output_link.speed_unit_ = link.SpeedUnit;
        ehp_output_link.s_lane_num_ = link.SLaneNum;
        ehp_output_link.e_lane_num_ = link.ELaneNum;
        ehp_output_link.is_highway_city = link.IsHighwayCity;
        ehp_output_link.last_in_link_ = link.LastInLink;
        ehp_output_link.next_out_link_ = link.NextOutLinks;
        // ehp_output_link.speeds_ = link.Vec_Speeds;

        for (const auto& speed : link.Vec_Speeds) {
            NoMapEFM::SOffsetValue temp_speed;
            temp_speed.offset_ = speed.Offset;
            temp_speed.value_ = speed.Value;
            ehp_output_link.speeds_.push_back(temp_speed);
        }


        for (const auto& lane_change : link.Vec_LaneChanges) {
            NoMapEFM::SLaneChange lc;
            lc.num_ = lane_change.Num;
            lc.offset_ = lane_change.Offset;
            lc.split_dir_ = lane_change.SplitDir;
            lc.meger_dir_ = lane_change.MegerDir;
            ehp_output_link.lane_changes_.push_back(lc);
        }

        for (const auto& lane_info : link.Vec_LinkLaneInfoList) {
            NoMapEFM::LinkLaneInfo lli;
            lli.lane_id = lane_info.LaneId;
            lli.lane_num = lane_info.LaneNum;
            lli.is_route_lane = !lane_info.IsPathEnd;
            lli.is_lane_num_change = lane_info.IsLaneNumChange;
            lli.lane_change_type = static_cast<NoMapEFM::LaneChangeType>(lane_info.LaneChangeType);
            lli.start_point_to_link_start_dis = lane_info.StartPointToLinkStartDis;
            lli.feature_point_type = static_cast<NoMapEFM::FEATUREPOINT_TYPE>(lane_info.FeaturePointType);
            lli.next_lane_ids = lane_info.Vec_NextLaneIds;
            lli.pre_lane_ids = lane_info.Vec_PrevLaneIds;
            lli.lane_lsl_type = static_cast<NoMapEFM::LANE_LSLTYPE>(lane_info.LaneLslType);
            lli.lane_types.push_back(static_cast<NoMapEFM::LANE_TYPE>(lane_info.LaneTypes));
            // std::transform(lane_info.LaneTypes.begin(), lane_info.LaneTypes.end(), std::back_inserter(lli.lane_types),
            //     [](datatype_ehp::e_LaneType_t_ref laneType) {
            //         return static_cast<NoMapEFM::LANE_TYPE>(laneType);
            //     });

            std::transform(lane_info.Vec_LeftLineBorderTypes.begin(), lane_info.Vec_LeftLineBorderTypes.end(), std::back_inserter(lli.left_line_border_types),
                [](datatype_ehp::e_LaneBorderType_t_ref lineType) {
                    return static_cast<NoMapEFM::LANE_BORDERTYPE>(lineType);
                });

            std::transform(lane_info.Vec_RightLineBorderTypes.begin(), lane_info.Vec_RightLineBorderTypes.end(), std::back_inserter(lli.right_line_border_types),
                [](datatype_ehp::e_LaneBorderType_t_ref lineType) {
                    return static_cast<NoMapEFM::LANE_BORDERTYPE>(lineType);
                });

            ehp_output_link.link_lane_info_list_.push_back(lli);
        }

        for (const auto& curv : link.Vec_Curvs) {
            NoMapEFM::SOffsetValue temp_curv;
            temp_curv.offset_ = curv.Offset;
            temp_curv.value_ = curv.Value;
            ehp_output_link.curvs_.push_back(temp_curv);
        }

        for (const auto& slope : link.Vec_Slopes) {
            NoMapEFM::SOffsetValue temp_slope;
            temp_slope.offset_ = slope.Offset;
            temp_slope.value_ = slope.Value;
            ehp_output_link.slopes_.push_back(temp_slope);
        }        
        ehp_output_link.all_in_links_ = link.Vec_AllInLinks;
        ehp_output_link.all_out_links_ = link.Vec_AllOutLinks;

        for (const auto& geom : link.Vec_Geoms) {
            EFMPoint temp_point;
            temp_point.x = geom.Lon;
            temp_point.y = geom.Lat;
            ehp_output_link.geoms_.push_back(temp_point);
        } 

        EHP_links_.ehp_output_link_list.push_back(ehp_output_link);
    }

    return;

}

bool ComponentEFMTaskApp::CreateSDDateIndex(){
    sd_index_.path_id_index_map.clear();
    sd_index_.link_path_offset_id_index_map.clear();
    sd_index_.lane_id_index_map.clear();

    for (int i = 0; i < EHP_links_.ehp_output_link_list.size(); i++){
        uint64_t link_id = EHP_links_.ehp_output_link_list[i].path_offset_id_;
        if (sd_index_.link_path_offset_id_index_map.find(link_id) == sd_index_.link_path_offset_id_index_map.end()) {
            sd_index_.link_path_offset_id_index_map[link_id] = i;
        }else{
            std::cout << "EnvironmentModel::CreateSDDateIndex: link_id is repeat:"<< link_id << std::endl;
        }
        
        for (int j = 0; j < EHP_links_.ehp_output_link_list[i].link_lane_info_list_.size(); j++){
            uint64_t lane_id = EHP_links_.ehp_output_link_list[i].link_lane_info_list_[j].lane_id; 
            if (sd_index_.lane_id_index_map.find(lane_id) == sd_index_.lane_id_index_map.end()) {
                sd_index_.lane_id_index_map[lane_id] = {i, j};
            } else {
                std::cout << "EnvironmentModel::CreateSDDateIndex: lane id is repeat:"<< lane_id << std::endl;
            }
        }
        
    }

    for (int i = 0; i < EHP_paths_.ehp_output_path_list.size(); i++){
        uint32_t path_id = EHP_paths_.ehp_output_path_list[i].path_id_;
        if (sd_index_.path_id_index_map.find(path_id) == sd_index_.path_id_index_map.end()) {
            sd_index_.path_id_index_map[path_id] = i;
        }else{
            std::cout << "EnvironmentModel::CreateSDDateIndex: path_id is repeat:"<< path_id << std::endl;
        }
    }
    
    return true;
}

bool ComponentEFMTaskApp::StoreInfo(const NoMapEFM::SEhpOutputLoc& lane_loc, const BevLaneElementGroup *ego_bev_lane_group){
    last_cycle_info_.lane_id =lane_loc.lane_id_;
    last_cycle_info_.lane_num = lane_loc.lane_ids_.size();

    if(ego_bev_lane_group != nullptr&&ego_bev_lane_group->size()>0){
        if(ego_bev_lane_group->front().right_front_connect_id != 0){
            last_cycle_info_.ego_right_line_ids.push_back(ego_bev_lane_group->front().right_front_connect_id) ;
        }
        if(ego_bev_lane_group->front().right_line_base_id != 0){
            last_cycle_info_.ego_right_line_ids.push_back(ego_bev_lane_group->front().right_line_base_id) ;
        }
        if(ego_bev_lane_group->front().right_back_connect_id != 0){
            last_cycle_info_.ego_right_line_ids.push_back(ego_bev_lane_group->front().right_back_connect_id) ;
        }

        if(ego_bev_lane_group->back().left_front_connect_id != 0){
            last_cycle_info_.ego_left_line_ids.push_back(ego_bev_lane_group->front().left_front_connect_id) ;
        }
        if(ego_bev_lane_group->back().left_line_base_id != 0){
            last_cycle_info_.ego_left_line_ids.push_back(ego_bev_lane_group->front().left_line_base_id) ;
        }
        if(ego_bev_lane_group->back().left_back_connect_id != 0){
            last_cycle_info_.ego_left_line_ids.push_back(ego_bev_lane_group->front().left_back_connect_id) ;
        }
    }

    return true;
}

bool ComponentEFMTaskApp::NotNOA(){
    bool res = false;
    for(auto& path: EHP_paths_.ehp_output_path_list){
        if(path.path_id_ == EHP_loc_.path_id_){
            if(path.parent_path_id_ == UINT32_MAX){
                res = true;
            }
        }
    }
    return res;
}

void ComponentEFMTaskApp::PlotGroup(){
#ifdef LM_LANEGROUP_RES
    for(auto& group: bev_lane_group_set_){
        if(group.first == 0){
            auto ego_lane_group = group.second;
    for(int i = 0; i< ego_lane_group.size(); i++){
        std::cout<< "ego_lane_group["<<i<<" ]: "<<" left_line_base_id: "<<ego_lane_group[i].left_line_base_id<< " ,left_line_back_con_id: "<< ego_lane_group[i].left_back_connect_id <<" left_line_front_conn_id: "<<ego_lane_group[i].left_front_connect_id
                                                <<" right_line_base_id: "<<ego_lane_group[i].right_line_base_id<< " ,right_line_back_con_id: "<< ego_lane_group[i].right_back_connect_id <<" right_line_front_conn_id: "<<ego_lane_group[i].right_front_connect_id
                                                 <<" ,left_line_size"<<ego_lane_group[i].left_line_points.size()<<" ,right_line_size"<<ego_lane_group[i].right_line_points.size()<<std::endl;
        std::cout<<"            left_line_split_index: "<<ego_lane_group[i].left_line_split_index << " ,left_line_merge_index: "<<ego_lane_group[i].left_line_merge_index<<
        " ,right_line_split_index: "<<ego_lane_group[i].right_line_split_index << " ,right_line_merge_index: "<<ego_lane_group[i].right_line_merge_index<<std::endl;
        std::cout<<"            left_line_split_x: "<<ego_lane_group[i].left_line_split_x << " ,left_line_merge_x: "<<ego_lane_group[i].left_line_merge_x<<
        " ,right_line_split_x: "<<ego_lane_group[i].right_line_split_x << " ,right_line_merge_x: "<<ego_lane_group[i].right_line_merge_x<<std::endl;
        std::cout<<" left_line_type:";
        for(auto& type: ego_lane_group[i].left_types){
            std::cout<<",<is_v:"<<(int)type.is_valid<<" ,type:"<<type.line_type<<" ,start_s:"<<type.start_point<<" ,typ_aft_chg_point:"<<type.typ_aft_chg_point<<" ,typ_chg_point:"<<type.typ_chg_point<<">";
        }
        std::cout<<std::endl;
        std::cout<<" right_line_type:";
        for(auto& type: ego_lane_group[i].right_types){
            std::cout<<",<is_v:"<<(int)type.is_valid<<" ,type:"<<type.line_type<<" ,start_s:"<<type.start_point<<" ,typ_aft_chg_point:"<<type.typ_aft_chg_point<<" ,typ_chg_point:"<<type.typ_chg_point<<">";
        }
        std::cout<<std::endl;
        {
        std::stringstream ss, ss1;
        ss<<"ego_left_line_x = [";
        ss1<<"ego_left_line_y = [";
        for(int j=0; j<ego_lane_group[i].left_line_points.size();j++){
            if(j == ego_lane_group[i].left_line_points.size()-1){
                ss<< ego_lane_group[i].left_line_points[j].x;
            }else{
                ss<< ego_lane_group[i].left_line_points[j].x<<" ,";
            }            
        }
        for(int j=0; j<ego_lane_group[i].left_line_points.size();j++){
            if(j == ego_lane_group[i].left_line_points.size()-1){
                ss1<< ego_lane_group[i].left_line_points[j].y;
            }else{
                ss1<< ego_lane_group[i].left_line_points[j].y<<" ,";
            }            
        }
        ss<<"]"<<std::endl;
        ss1<<"]"<<std::endl; 
        std::cout<<ss.str();   
        std::cout<<ss1.str();    
        }
        {
        std::stringstream ss, ss1;
        ss<<"ego_right_line_x = [";
        ss1<<"ego_right_line_y = [";
        for(int j=0; j<ego_lane_group[i].right_line_points.size();j++){
            if(j == ego_lane_group[i].right_line_points.size()-1){
                ss<< ego_lane_group[i].right_line_points[j].x;
            }else{
                ss<< ego_lane_group[i].right_line_points[j].x<<" ,";
            }            
        }
        for(int j=0; j<ego_lane_group[i].right_line_points.size();j++){
            if(j == ego_lane_group[i].right_line_points.size()-1){
                ss1<< ego_lane_group[i].right_line_points[j].y;
            }else{
                ss1<< ego_lane_group[i].right_line_points[j].y<<" ,";
            }            
        }
        ss<<"]"<<std::endl;
        ss1<<"]"<<std::endl; 
        std::cout<<ss.str();   
        std::cout<<ss1.str();    
        }
        {
        std::vector<double> headings{};
        std::vector<double> accumulated_s{};
        std::vector<double> kappas{}; 
        std::vector<double> dkappas{};
        CommonTool::DiscretePointsMath::GetInstance()->ComputePathProfile(ego_lane_group[i].left_line_points,&headings, &accumulated_s,&kappas, &dkappas);
        std::stringstream ss, ss1;
        ss<<"ego_left_line_kappas = [";
        for(int j=0; j<kappas.size() && j<ego_lane_group[i].left_line_points.size();j++){
                ss<<" ,<x:"<<ego_lane_group[i].left_line_points[j].x<<",k:"<<kappas[j]<<">";           
        }
        ss<<"]"<<std::endl;
        std::cout<<ss.str();     
        }
        {
        std::vector<double> headings{};
        std::vector<double> accumulated_s{};
        std::vector<double> kappas{}; 
        std::vector<double> dkappas{};
        CommonTool::DiscretePointsMath::GetInstance()->ComputePathProfile(ego_lane_group[i].right_line_points,&headings, &accumulated_s,&kappas, &dkappas);
        std::stringstream ss, ss1;
        ss<<"ego_right_line_kappas = [";
        for(int j=0; j<kappas.size() && j<ego_lane_group[i].right_line_points.size();j++){
                ss<<" ,<x:"<<ego_lane_group[i].right_line_points[j].x<<",k:"<<kappas[j]<<">";           
        }
        ss<<"]"<<std::endl;
        std::cout<<ss.str();      
        }     
        {
        std::vector<double> headings{};
        std::vector<double> accumulated_s{};
        std::vector<double> kappas{}; 
        std::vector<double> dkappas{};
        CommonTool::DiscretePointsMath::GetInstance()->ComputePathProfile(ego_lane_group[i].center_line_points,&headings, &accumulated_s,&kappas, &dkappas);
        std::stringstream ss, ss1;
        ss<<"ego_center_line_kappas = [";
        for(int j=0; j<kappas.size() && j<ego_lane_group[i].center_line_points.size();j++){
                ss<<" ,<x:"<<ego_lane_group[i].center_line_points[j].x<<",k:"<<kappas[j]<<">";           
        }
        ss<<"]"<<std::endl;
        std::cout<<ss.str();      
        }     
    }
        }
    }
#endif
}
