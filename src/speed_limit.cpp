/*
 * @Author: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @Date: 2023-12-15 10:35:03
 * @LastEditors: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @LastEditTime: 2024-06-06 17:44:09
 * @FilePath: /repo/code/dx11_noa/application/environmentmodelfunction/src/speed_limit.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "speed_limit.h"

#define TOLL_LANE_MAX_SPEED 60           // km/H
#define RAMP_LANE_MAX_SPEED 50           // km/H
#define CONNECT_ACCELERATE_LANE_MAX_SPEED 80  // km/H
#define CONNECT_DECELERATE_LANE_MAX_SPEED 100  // km/H
#define NORMAL_LANE_MAX_SPEED 60         // km/H
#define SPEED_INTERPOLATION 20           // km/H
#define LOWER_SPEED_INTERPOLATION 10     // km/H
#define SPEED_DIST_INTERPOLATION 35000   // cm
#define RAMP_LANE_MAX_SPEED_TOP 70       // cm
#define RAMP_40_BACK_STEP 3200           // cm

namespace noa {
namespace environmentmodelfunction {
uint64_t SpeedLimit::on_ramp_pos_offset_ = 0;
uint8_t SpeedLimit::last_cycle_ego_lane_type_ = 0;
bool SpeedLimit::on_ramp_lock_start_f_ = false;
bool SpeedLimit::last_cycle_tunnel_f_ = false;
bool SpeedLimit::just_pass_tunnel_f_ = false;
uint32_t SpeedLimit::just_pass_tunnel_offset_ = 0;
uint8_t SpeedLimit::last_speed_limit_final_value_ = 0;
uint8_t SpeedLimit::last_speed_limit_source_ = 0;
bool SpeedLimit::main_road_use_hd_ = false;
bool SpeedLimit::Execute(const message::map_position::s_Position_t& map_position,
                         const earth::shell::framework::TopicTrait::MapMapMsg& map_static_info,
                         const std::unordered_map<uint32_t, int>& link_id_index_lane_info_map,
                         const std::shared_ptr<earth::shell::framework::CandidateLanesModel> candidate_lanes_model,
                         const std::shared_ptr<earth::shell::framework::ScenarioJudgeModel> scenario_judge_model,
                         const std::shared_ptr<earth::shell::framework::ExtractRefLine> extract_ref_line,
                         std::shared_ptr<const earth::shell::framework::TopicTrait::TrafficInfoMsg> traffic_info,
                         std::shared_ptr<const earth::shell::framework::TopicTrait::ComInfo> com_info) {
    // link_id_vec_ = std::make_shared<const std::vector<uint32_t>>(link_id_vec);
    map_position_ = std::make_shared<const message::map_position::s_Position_t>(map_position);
    map_static_info_ = std::make_shared<const earth::shell::framework::TopicTrait::MapMapMsg>(map_static_info);
    link_id_index_lane_info_map_ =
        std::make_shared<const std::unordered_map<uint32_t, int>>(link_id_index_lane_info_map);
    com_info_ = com_info;
    traffic_info_ = traffic_info;
    tar_link_id_vec_ = scenario_judge_model->tar_link_list();
    tar_link_max_speed_ = std::max(scenario_judge_model->tar_link_max_speed(), static_cast<uint8_t>(60));
    // std::cout << __FILE__ << "," << __LINE__ << ",raw cur_speed_: " << (int)cur_speed_<<std::endl;
    speed_limit(candidate_lanes_model, extract_ref_line);
    // std::cout << __FILE__ << "," << __LINE__ << ",speed_limit cur_speed_: " << (int)cur_speed_<<std::endl;
    exit_scene(candidate_lanes_model, scenario_judge_model, extract_ref_line);
    // std::cout << __FILE__ << "," << __LINE__ << ",exit_scene cur_speed_: " << (int)cur_speed_<<std::endl;
    speed_limmit_sd(candidate_lanes_model, scenario_judge_model, extract_ref_line);
    // std::cout << __FILE__ << "," << __LINE__ << ",speed_limmit_sd cur_speed_: " << (int)cur_speed_<<std::endl;
    speed_ZTEXT();
    // std::cout << __FILE__ << "," << __LINE__ << ",speed_ZTEXT cur_speed_: " << (int)cur_speed_<<std::endl;

    last_speed_limit_final_value_ = cur_speed_;
    last_speed_limit_source_ = speed_limit_source_;
    ZTEXT("EFM_INFO", "speed_limmit_source_: ", 140, 65, "speed_limmit_source_: {}", int(speed_limit_source_));
    return true;
}

bool SpeedLimit::get_lanetype_and_speed(uint8_t& curlane_spd, uint8_t& cur_lane_type, uint8_t& mainroad_max_spd,
                                        uint32_t link_id, uint8_t lane_id, uint32_t& speed_dist, bool is_endoffset,
                                        uint8_t& curlane_next_spd, uint32_t& speed_dist_next) {
    curlane_next_spd = 0;
    message::map_map::s_LinkInfo_t link_infos;
    if (false == GetLinkInfos(link_id, link_infos)) {
        // static data none
        return false;
    }
    //
    int temp_link_dist = 0;
    speed_dist_next = 0;
    if (is_endoffset) {
        temp_link_dist = link_infos.EndOffset.EndOffset - map_position_->PathOffset;
    } else {
        temp_link_dist = link_infos.EndOffset.EndOffset - link_infos.PathOffset.PathOffset;
    }
    speed_dist = temp_link_dist > 0 ? temp_link_dist : 0;
    for (auto& link_LaneInfo : link_infos.LaneInfos.LaneInfos) {
        if (link_LaneInfo.LaneNum.LaneNum == lane_id) {
            cur_lane_type = link_LaneInfo.LaneType.data;
            break;
        }
    }

    bool find_flag = false;
    for (auto& lanespeedinfo : map_static_info_->LaneSpeedLimitis.LaneSpeedLimits) {
        if (lanespeedinfo.InstanceId.InstanceId == link_id) {
            find_flag = true;
            mainroad_max_spd = std::max(mainroad_max_spd, lanespeedinfo.ValueH.ValueH);
            if (lanespeedinfo.LaneNum.LaneNum == lane_id || 0 == lanespeedinfo.LaneNum.LaneNum) {
                if (cur_lane_type == 4 || cur_lane_type == 5 || cur_lane_type == 6) {
                    curlane_spd = (lanespeedinfo.ValueH.ValueH & 0xF) * 10;
                    curlane_next_spd = (lanespeedinfo.ValueL.ValueL & 0xF) * 10;
                    if ((((lanespeedinfo.ValueL.ValueL >> 4) & 0xF) <= 1) && (((lanespeedinfo.ValueH.ValueH >> 4) & 0xF) < 1)){
                        curlane_spd = curlane_next_spd;
                    } else {
                        uint32_t temp_next_speed_offset = 0;
                        temp_next_speed_offset =
                            ((lanespeedinfo.ValueH.ValueH & 0xF0) | (lanespeedinfo.ValueL.ValueL >> 4)) * 1000;
                        if (is_endoffset) {
                            if (link_infos.PathOffset.PathOffset + temp_next_speed_offset > map_position_->PathOffset) {
                                speed_dist = link_infos.PathOffset.PathOffset + temp_next_speed_offset -
                                             map_position_->PathOffset;
                                speed_dist_next = temp_link_dist > speed_dist ? temp_link_dist - speed_dist : 0;
                            } else {
                                curlane_spd = curlane_next_spd;
                                speed_dist = temp_link_dist;
                                curlane_next_spd = 0;
                                speed_dist_next = 0;
                            }

                        } else {
                            speed_dist = temp_next_speed_offset;
                            speed_dist_next =
                                temp_link_dist > temp_next_speed_offset ? temp_link_dist - temp_next_speed_offset : 0;
                        }
                    }

                    if (curlane_spd > 150 || curlane_next_spd > 150) {
                        curlane_spd = 50;
                        curlane_next_spd = 50;
                        speed_dist_next = 0;
                    }

                } else {
                    curlane_spd = std::max(curlane_spd, lanespeedinfo.ValueH.ValueH);
                }
                return true;
                // curlane_spd = lanespeedinfo.ValueH.ValueH;
            }
        } else {
            if (find_flag) {
                break;
            }
        }
    }

    return true;
}

bool SpeedLimit::converte_speed(uint8_t& cur_speed, uint8_t& tar_speed, uint8_t lane_type, uint8_t mainroad_max_spd,
                                uint32_t link_id) {
    if (toll_link_id_set_.find(link_id) != toll_link_id_set_.end()) {
        tar_speed = std::min(TOLL_LANE_MAX_SPEED, static_cast<int>(cur_speed));
    } else if(std::find(tar_link_id_vec_.begin(), tar_link_id_vec_.end(), link_id) != tar_link_id_vec_.end()){
        tar_speed = std::max(last_speed_limit_final_value_, tar_link_max_speed_);
    }else if (4 == lane_type || 5 == lane_type || 6 == lane_type) {
        // message::map_map::e_LaneType_t::ONRAMP_ / message::map_map::e_LaneType_t::OFFRAMP_ /
        // message::map_map::e_LaneType_t::CONNECTRAMP_
        // if (cur_speed == 30) {
        //     tar_speed = cur_speed;
        // } else {
        tar_speed = std::max(RAMP_LANE_MAX_SPEED, static_cast<int>(cur_speed));
        // }

    } else if (2 == lane_type || 8 == lane_type) {
        tar_speed = std::min(CONNECT_ACCELERATE_LANE_MAX_SPEED, static_cast<int>(mainroad_max_spd));
    } else if (1 == lane_type ||  7 == lane_type) {
        tar_speed = std::min(CONNECT_ACCELERATE_LANE_MAX_SPEED, static_cast<int>(mainroad_max_spd));
    }else {
        // other
        tar_speed = mainroad_max_spd;
    }

    return true;
}

bool SpeedLimit::get_toll_link() {
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

uint8_t SpeedLimit::converte_lanetype(uint8_t lane_type) {
    uint8_t tar_lane_type = 0;
    switch (lane_type) {
        case 4:  // ONRAMP_
        case 5:  // OFFRAMP_
        case 6:  // CONNECTRAMP_
            tar_lane_type = 4;
            break;
        case 1:  // ENTRY_
        case 7:  // ACCELERATE_
            tar_lane_type = 1;
            break;
        case 2:  // ENTRY_
        case 8:  // ACCELERATE_
            tar_lane_type = 2;
            break;
        default:
            tar_lane_type = lane_type;
            break;
    }

    return tar_lane_type;
}

bool SpeedLimit::get_ref_lane_info(
    const std::shared_ptr<earth::shell::framework::CandidateLanesModel> candidate_lanes_model,
    const std::shared_ptr<earth::shell::framework::ExtractRefLine> extract_ref_line,
    std::vector<lane_type_speed>& lane_type_vec) {
    std::vector<uint32_t> link_id_vec = candidate_lanes_model->link_id_vec();
    std::vector<uint8_t> lane_num_vec{};
    // int prior_index = extract_ref_line->prior_path_index();  // 0-ego;1-left;2-right
    // switch (prior_index) {
    //     case 0:
    //         lane_num_vec = candidate_lanes_model->ego_path_lane_num();
    //         break;
    //     case 1:
    //         lane_num_vec = candidate_lanes_model->left_path_lane_num();
    //         break;
    //     case 2:
    //         lane_num_vec = candidate_lanes_model->right_path_lane_num();
    //         break;
    //     default:
    //         lane_num_vec = candidate_lanes_model->ego_path_lane_num();
    //         break;
    // }
    // std::cout << __FILE__ << __LINE__ << ",is_ramp_to_main_road: " << int(is_ramp_to_main_road_) << std::endl;
    if(is_ramp_to_main_road_ == true){
        lane_num_vec = candidate_lanes_model->ego_path_lane_num();
    }else{
        lane_num_vec = candidate_lanes_model->ref_route_path_lane_num();
    }

    if (lane_num_vec.size() <= link_id_vec.size()) {
        for (int i = 1; i < lane_num_vec.size(); i++) {
            uint32_t next_link_id = link_id_vec[i];
            uint8_t next_lane_num = lane_num_vec[i];
            uint8_t next_lane_spd = RAMP_LANE_MAX_SPEED;
            uint8_t next_lane_spd_next = RAMP_LANE_MAX_SPEED;
            uint8_t next_link_spd = NORMAL_LANE_MAX_SPEED;
            uint8_t next_lane_type = 0;
            uint32_t next_speed_dist = 0;
            uint32_t next_speed_dist_next = 0;

            get_lanetype_and_speed(next_lane_spd, next_lane_type, next_link_spd, next_link_id, next_lane_num,
                                   next_speed_dist, false, next_lane_spd_next, next_speed_dist_next);
            if (4 == next_lane_type || 5 == next_lane_type || 6 == next_lane_type) {
                next_speed_ = next_lane_spd;
            } else {
                converte_speed(next_lane_spd, next_speed_, next_lane_type, next_link_spd, next_link_id);
            }

            lane_type_speed next_lane_type_speed;
            next_lane_type_speed.lane_type_ = converte_lanetype(next_lane_type);
            next_lane_type_speed.speed_ = next_speed_;
            next_lane_type_speed.link_dist_ = next_speed_dist;

            auto& back_data = lane_type_vec.back();
            if (back_data.speed_ == next_lane_type_speed.speed_ &&
                back_data.lane_type_ == next_lane_type_speed.lane_type_) {
                back_data.link_dist_ += next_lane_type_speed.link_dist_;
            } else {
                lane_type_vec.push_back(next_lane_type_speed);
            }

            if (next_speed_dist_next > 0) {
                next_lane_type_speed.lane_type_ = converte_lanetype(next_lane_type);
                next_lane_type_speed.speed_ = next_lane_spd_next;
                next_lane_type_speed.link_dist_ = next_speed_dist_next;
                lane_type_vec.push_back(next_lane_type_speed);
            }
        }
    }

    return true;
}

bool SpeedLimit::get_speed_change_info(std::vector<lane_type_speed> lane_type_vec,
                                       std::vector<uint8_t>& speed_change_vec) {
    speed_change_vec.clear();
    if (lane_type_vec.size() <= 1) {
        return true;
    }

    for (size_t i = 1; i < lane_type_vec.size(); i++) {
        if (lane_type_vec[i].speed_ > 30 &&
            ((lane_type_vec[i - 1].speed_ > lane_type_vec[i].speed_ + SPEED_INTERPOLATION))) {
            speed_change_vec.push_back(i);
        } else if (lane_type_vec[i].speed_ <= 30 &&
                   ((lane_type_vec[i - 1].speed_ > lane_type_vec[i].speed_ + LOWER_SPEED_INTERPOLATION))) {
            speed_change_vec.push_back(i);
        }
    }

    return true;
}

uint32_t SpeedLimit::get_before_data_dist(std::vector<lane_type_speed> lane_type_vec, uint32_t next_idx) {
    uint32_t all_dist = 0;
    if (next_idx > lane_type_vec.size()) {
        return all_dist;
    }

    for (size_t i = 0; i < next_idx; i++) {
        all_dist += lane_type_vec[i].link_dist_;
    }
    return all_dist;
}

int32_t SpeedLimit::get_lower_speed_dist(std::vector<lane_type_speed> lane_type_vec, uint32_t begin_idx,
                                         uint32_t has_dist) {
    if (begin_idx == (lane_type_vec.size() - 1)) {
        return SPEED_DIST_INTERPOLATION;
    }

    int32_t lower_speed_dist = lane_type_vec[begin_idx].link_dist_ + has_dist;
    if (lower_speed_dist >= SPEED_DIST_INTERPOLATION) {
        return lower_speed_dist;
    }
    uint8_t cur_speed = lane_type_vec[begin_idx].speed_;
    if (lane_type_vec[begin_idx].speed_ == lane_type_vec[begin_idx + 1].speed_) {
        lower_speed_dist = get_lower_speed_dist(lane_type_vec, begin_idx + 1, lower_speed_dist);
    } else if (lane_type_vec[begin_idx].speed_ < lane_type_vec[begin_idx + 1].speed_) {
        return SPEED_DIST_INTERPOLATION;
    } else {
        lower_speed_dist = (lower_speed_dist + get_lower_speed_dist(lane_type_vec, begin_idx + 1, lower_speed_dist)) -
                           SPEED_DIST_INTERPOLATION;
    }

    return lower_speed_dist;
}

bool SpeedLimit::get_next_speed(std::vector<lane_type_speed> lane_type_vec, uint8_t& cur_speed, uint8_t& next_speed,
                                double& next_speed_dist) {
    if (0 == lane_type_vec.size() || 1 == lane_type_vec.size()) {
        return true;
    }

    cur_speed = lane_type_vec[0].speed_;
    next_speed = cur_speed;
    next_speed_dist = 0.0;
    for (size_t i = 0; i < lane_type_vec.size(); i++) {
        if (lane_type_vec[i].speed_ == cur_speed) {
            next_speed_dist += lane_type_vec[i].link_dist_;
            continue;
        }
        next_speed = lane_type_vec[i].speed_;
        break;
    }
    return true;

    // uint32_t all_dist = lane_type_vec[0].link_dist_;
    // for (size_t i = 1; i < lane_type_vec.size(); i++) {
    //     if (lane_type_vec[i].speed_ == lane_type_vec[i - 1].speed_) {
    //         all_dist += lane_type_vec[i].link_dist_;
    //     } else {
    //         if (lane_type_vec[i].speed_ > lane_type_vec[i - 1].speed_) {
    //             // no need change dist
    //             next_speed = lane_type_vec[i].speed_;
    //             next_speed_dist = all_dist;
    //             break;
    //         } else {
    //             // jude dist
    //             int32_t lower_speed_dist = get_lower_speed_dist(lane_type_vec, i, 0);
    //             if (lower_speed_dist >= SPEED_DIST_INTERPOLATION) {
    //                 next_speed = lane_type_vec[i].speed_;
    //                 next_speed_dist = all_dist;
    //                 break;
    //             } else {
    //                 if ((all_dist + lower_speed_dist) > SPEED_DIST_INTERPOLATION) {
    //                     next_speed = lane_type_vec[i].speed_;
    //                     next_speed_dist = all_dist + lower_speed_dist - SPEED_DIST_INTERPOLATION;
    //                     break;
    //                 } else {
    //                     cur_speed = lane_type_vec[i].speed_;
    //                     if (lane_type_vec.size() == (i + 1)) {
    //                         next_speed = lane_type_vec[i].speed_;
    //                         next_speed_dist = 0;

    //                     } else {
    //                         next_speed = lane_type_vec[i + 1].speed_;
    //                         next_speed_dist = all_dist + lower_speed_dist;
    //                     }
    //                     break;
    //                 }
    //             }
    //         }
    //     }
    // }
    return true;
}

bool SpeedLimit::change_speed_back(std::vector<lane_type_speed>& lane_type_vec, uint8_t next_speed) {
    if (0 == lane_type_vec.size()) {
        return false;
    }

    uint8_t cur_speed = 0;
    uint32_t surplus_dist = 0;
    GetInsertDist(next_speed, surplus_dist, cur_speed);
    std::vector<lane_type_speed> temp_lane_type_vec;
    bool speed_chage_stop = false;
    lane_type_speed waiting_short_lane;
    for (int32_t i = (lane_type_vec.size() - 1); i >= 0; i--) {
        if (speed_chage_stop) {
            temp_lane_type_vec.push_back(lane_type_vec[i]);
            continue;
        }

        if (lane_type_vec[i].speed_ <= cur_speed) {
            lane_type_vec[i].link_dist_ += waiting_short_lane.link_dist_;
            temp_lane_type_vec.push_back(lane_type_vec[i]);
            speed_chage_stop = true;
            waiting_short_lane.link_dist_ = 0;
            continue;
        }

        if (lane_type_vec[i].link_dist_ >= surplus_dist) {
            waiting_short_lane.link_dist_ += surplus_dist;
            waiting_short_lane.lane_type_ = lane_type_vec[i].lane_type_;
            waiting_short_lane.speed_ = cur_speed;
            temp_lane_type_vec.push_back(waiting_short_lane);

            if (lane_type_vec[i].link_dist_ > surplus_dist) {
                // cut off lane
                lane_type_vec[i].link_dist_ -= surplus_dist;
                i++;
            }

            GetInsertDist(waiting_short_lane.speed_, surplus_dist, cur_speed);
            waiting_short_lane.link_dist_ = 0;
        } else {
            surplus_dist -= lane_type_vec[i].link_dist_;
            if (0 == waiting_short_lane.link_dist_) {
                // first
                waiting_short_lane.lane_type_ = lane_type_vec[i].lane_type_;
                waiting_short_lane.speed_ = cur_speed;
                waiting_short_lane.link_dist_ = lane_type_vec[i].link_dist_;
            } else {
                waiting_short_lane.link_dist_ += lane_type_vec[i].link_dist_;
            }
        }
    }
    if (false == speed_chage_stop && waiting_short_lane.link_dist_ > 0) {
        temp_lane_type_vec.push_back(waiting_short_lane);
    }

    lane_type_vec.clear();
    lane_type_vec.assign(temp_lane_type_vec.rbegin(), temp_lane_type_vec.rend());

    return true;
}

bool SpeedLimit::change_speed_in_ramp(std::vector<lane_type_speed>& lane_type_vec) {
    // change speed if in ramp 40, back 20m

    std::vector<lane_type_speed> temp_lane_type_vec;
    temp_lane_type_vec.assign(lane_type_vec.begin(), lane_type_vec.end());
    lane_type_vec.clear();

    lane_type_vec.push_back(temp_lane_type_vec[0]);
    bool first_flag = true;
    for (size_t i = 1; i < temp_lane_type_vec.size(); i++) {
        if (false == first_flag) {
            lane_type_vec.push_back(temp_lane_type_vec[i]);
            continue;
        }

        if ((temp_lane_type_vec[i].speed_ == 40 || temp_lane_type_vec[i].speed_ == 30) &&
            (temp_lane_type_vec[i].lane_type_ == 4 || temp_lane_type_vec[i].lane_type_ == 5 ||
             temp_lane_type_vec[i].lane_type_ == 6) &&
            (!(temp_lane_type_vec[i - 1].lane_type_ == 4 || temp_lane_type_vec[i - 1].lane_type_ == 5 ||
               temp_lane_type_vec[i - 1].lane_type_ == 6))) {
            first_flag = false;
            uint32_t temp_length = RAMP_40_BACK_STEP;
            int in_vec_size = lane_type_vec.size() - 1;
            while (temp_length > 0 && in_vec_size >= 0) {
                if (temp_length == lane_type_vec[in_vec_size].link_dist_) {
                    lane_type_vec[in_vec_size].speed_ = temp_lane_type_vec[i].speed_;
                    temp_length = 0;
                    break;
                } else if (temp_length > lane_type_vec[in_vec_size].link_dist_) {
                    lane_type_vec[in_vec_size].speed_ = temp_lane_type_vec[i].speed_;
                    temp_length -= lane_type_vec[in_vec_size].link_dist_;
                    in_vec_size--;
                    continue;
                } else {
                    lane_type_vec[in_vec_size].link_dist_ = lane_type_vec[in_vec_size].link_dist_ - temp_length;
                    lane_type_speed temp_new_data;
                    temp_new_data.speed_ = temp_lane_type_vec[i].speed_;
                    temp_new_data.lane_type_ = lane_type_vec[in_vec_size].lane_type_;
                    temp_new_data.link_dist_ = temp_length;
                    lane_type_vec.insert(lane_type_vec.begin() + in_vec_size + 1, temp_new_data);
                    break;
                }
            }
            lane_type_vec.push_back(temp_lane_type_vec[i]);
        } else {
            lane_type_vec.push_back(temp_lane_type_vec[i]);
        }
    }

    return true;
}

bool SpeedLimit::change_speed_for_exit(std::vector<lane_type_speed>& lane_type_vec) {
    // change speed if 2 or 8

    std::vector<lane_type_speed> temp_lane_type_vec;
    temp_lane_type_vec.assign(lane_type_vec.begin(), lane_type_vec.end());
    lane_type_vec.clear();

    for (size_t i = 1; i < temp_lane_type_vec.size(); i++) {
        if (2 == temp_lane_type_vec[i].lane_type_){
            uint8_t back_speed = temp_lane_type_vec[i].speed_ - SPEED_INTERPOLATION;
            uint8_t cur_speed = 0;
            uint32_t surplus_dist = 0;
            GetInsertDist(back_speed, surplus_dist, cur_speed);
            if (temp_lane_type_vec[i].link_dist_ < surplus_dist){
                temp_lane_type_vec[i].speed_ = temp_lane_type_vec[i -1].speed_;
            }
        }
    }

    lane_type_vec.push_back(temp_lane_type_vec[0]);
    for (size_t i = 1; i < temp_lane_type_vec.size(); i++){
        auto& back_data = lane_type_vec[lane_type_vec.size() - 1];
        if (temp_lane_type_vec[i].speed_ == back_data.speed_){
            back_data.link_dist_ += temp_lane_type_vec[i].link_dist_;
        }else{
            lane_type_vec.push_back(temp_lane_type_vec[i]);
        }
    }
    
    return true;
}

bool SpeedLimit::speed_limit(const std::shared_ptr<earth::shell::framework::CandidateLanesModel> candidate_lanes_model,
                             const std::shared_ptr<earth::shell::framework::ExtractRefLine> extract_ref_line) {
    get_toll_link();

    std::vector<uint8_t> ref_lane_num_vec{};
    ref_lane_num_vec = candidate_lanes_model->ref_route_path_lane_num();
    if (ref_lane_num_vec.empty()){
        return false;
    }
    
    uint8_t cur_lane_spd = RAMP_LANE_MAX_SPEED;
    uint8_t cur_lane_spd_next = 0;
    uint8_t cur_link_spd = NORMAL_LANE_MAX_SPEED;
    uint8_t cur_lane_type = 0;
    uint32_t cur_speed_dist = 0;
    uint32_t cur_speed_dist_next = 0;
    uint8_t egp_lane_speed = 255;
    is_ramp_to_main_road_ = false;
    ramp_to_main_road(map_position_->LinkId, ref_lane_num_vec[0], map_position_->LaneId, is_ramp_to_main_road_);
    if(is_ramp_to_main_road_ == true){
        get_lanetype_and_speed(cur_lane_spd, cur_lane_type, cur_link_spd, map_position_->LinkId,  map_position_->LaneId,
                           cur_speed_dist, true, cur_lane_spd_next, cur_speed_dist_next);
    }else{
        get_lanetype_and_speed(cur_lane_spd, cur_lane_type, cur_link_spd, map_position_->LinkId, ref_lane_num_vec[0],
                           cur_speed_dist, true, cur_lane_spd_next, cur_speed_dist_next);
    }

    lane_type_vec_.clear();
    if (4 == cur_lane_type || 5 == cur_lane_type || 6 == cur_lane_type) {
        cur_speed_ = cur_lane_spd;
    } else {
        converte_speed(cur_lane_spd, cur_speed_, cur_lane_type, cur_link_spd, map_position_->LinkId);
    }

    lane_type_speed cur_lane_type_speed;
    cur_lane_type_speed.lane_type_ = converte_lanetype(cur_lane_type);
    cur_lane_type_speed.speed_ = cur_speed_;
    cur_lane_type_speed.link_dist_ = cur_speed_dist;
    lane_type_vec_.push_back(cur_lane_type_speed);
    if (cur_speed_dist_next > 0) {
        cur_lane_type_speed.lane_type_ = converte_lanetype(cur_lane_type);
        cur_lane_type_speed.speed_ = cur_lane_spd_next;
        cur_lane_type_speed.link_dist_ = cur_speed_dist_next;
        lane_type_vec_.push_back(cur_lane_type_speed);
    }

    // get ego lane speed info
    get_ref_lane_info(candidate_lanes_model, extract_ref_line, lane_type_vec_);
    change_speed_in_ramp(lane_type_vec_);

    change_speed_for_exit(lane_type_vec_);
    std::vector<uint8_t> speed_change_vec;
    get_speed_change_info(lane_type_vec_, speed_change_vec);

    if (lane_type_vec_.size() > 0) {
        cur_speed_ = lane_type_vec_[0].speed_;
        cur_lane_type_ = lane_type_vec_[0].lane_type_;
    }

    next_speed_ = cur_speed_;
    next_speed_dist_ = 0;
    if (0 == speed_change_vec.size()) {
        get_next_speed(lane_type_vec_, cur_speed_, next_speed_, next_speed_dist_);
    } else {
        auto& before_data = lane_type_vec_[speed_change_vec[0] - 1];
        auto& after_data = lane_type_vec_[speed_change_vec[0]];

        std::vector<lane_type_speed> before_lane_type_vec;
        // std::cout << __FILE__ << "," << __LINE__ << " speed_change_vec[0] " << int(speed_change_vec[0]) << std::endl;
        before_lane_type_vec.assign(lane_type_vec_.begin(), lane_type_vec_.begin() + speed_change_vec[0]);
        if (before_data.speed_ > after_data.speed_) {
            next_speed_ = after_data.speed_;
            change_speed_back(before_lane_type_vec, after_data.speed_);
        } else {
            next_speed_ = before_data.speed_ + SPEED_INTERPOLATION;
        }

        next_speed_dist_ = get_before_data_dist(lane_type_vec_, speed_change_vec[0]);
        cur_speed_ = before_lane_type_vec[0].speed_;
        next_speed_ = cur_speed_;
        next_speed_dist_ = 0;
        before_lane_type_vec.push_back(lane_type_vec_[speed_change_vec[0]]);
        get_next_speed(before_lane_type_vec, cur_speed_, next_speed_, next_speed_dist_);
        lane_type_vec_.erase(lane_type_vec_.begin(), lane_type_vec_.begin() + speed_change_vec[0]);
        lane_type_vec_.insert(lane_type_vec_.begin(), before_lane_type_vec.begin(), before_lane_type_vec.end());
    }

    // if (40 == next_speed_ && cur_speed_ > next_speed_ && next_speed_dist_ <= 10000){
    //     cur_speed_ = next_speed_;
    // }

    // if (30 == next_speed_ && cur_speed_ > next_speed_ && next_speed_dist_ <= 7000){
    //     cur_speed_ = next_speed_;
    // }
    // ZTEXT("EFM_INFO", "cur_speed_: ", 80, -53, "cur_speed_: {}", int(cur_speed_));
    //  ZTEXT("EFM_INFO", "next_speed_dist_: ", 80, -38, "next_speed_dist_: {}", int(next_speed_dist_));
    //  ZTEXT("EFM_INFO", "next_speed_: ", 80, -41, "next_speed_: {}", int(next_speed_));
    cur_speed_ = cur_speed_ == 30 ? 35 : cur_speed_;
    next_speed_ = next_speed_ == 30 ? 35 : next_speed_;

    ZTEXT("EFM_INFO", "cur_hd_speed_: ", 80, -53, "cur_hd_speed_: {}", int(cur_speed_));
    ZTEXT("EFM_INFO", "next_hd_speed_dist_: ", 80, -59, "next_hd_speed_dist_: {}", int(next_speed_dist_));
    ZTEXT("EFM_INFO", "next_hd_speed_: ", 80, -56, "next_hd_speed_: {}", int(next_speed_));
    ZTEXT("EFM_INFO", "PathOffset: ", 80, -44, "PathOffset: {}", int(map_position_->PathOffset));
    ZTEXT("EFM_INFO", "next_speed_dist_idx: ", 80, -47, "next_speed_dist_idx: {}", int(next_speed_dist_ / 250.0));
    double next_speed_dist_tmp = next_speed_dist_ > (250.0 * 255) ? 255 : next_speed_dist_ / 250.0;
    ZTEXT("EFM_INFO", "next_speed_dist_tmp: ", 80, -50, "next_speed_dist_tmp: {}", pntIdx());

#ifdef SPEEDLIMIT_COUT
    std::cout << __FILE__ << "," << __LINE__ << " car_link_id: " << map_position_->LinkId << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << " PathOffset: " << map_position_->PathOffset << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << " car_in_link: " << car_in_link << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << " cur_speed_: " << (int)cur_speed_ << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << " next_speed_limit_dist_: " << next_speed_dist_ << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << " next_speed_limit_: " << static_cast<int>(next_speed_) << std::endl;
#endif
    return true;
}

bool SpeedLimit::ramp_to_main_road(const uint32_t link_id, const uint8_t ref_lane_id, const uint8_t ego_lane_id, bool& is_ramp_to_main_road) {
    message::map_map::s_LinkInfo_t link_infos;
    if (false == GetLinkInfos(link_id, link_infos)) {
        // static data none
        return false;
    }

    uint8_t ref_lane_type = 0;
    uint8_t ego_lane_type = 0;
    for (auto& link_LaneInfo : link_infos.LaneInfos.LaneInfos) {
        if (link_LaneInfo.LaneNum.LaneNum == ref_lane_id) {
            ref_lane_type = link_LaneInfo.LaneType.data;
            break;
        }
    }

    for (auto& link_LaneInfo : link_infos.LaneInfos.LaneInfos) {
        if (link_LaneInfo.LaneNum.LaneNum == ego_lane_id) {
            ego_lane_type = link_LaneInfo.LaneType.data;
            break;
        }
    }

    if(ref_lane_type == 0 && (ego_lane_type == 4 || ego_lane_type == 5 || ego_lane_type == 6 || ego_lane_type == 7 || ego_lane_type == 1)){
        is_ramp_to_main_road = true;
    }
    return true;

}

void SpeedLimit::exit_scene(const std::shared_ptr<earth::shell::framework::CandidateLanesModel> candidate_lanes_model,
                            const std::shared_ptr<earth::shell::framework::ScenarioJudgeModel> scenario_judge_model,
                            const std::shared_ptr<earth::shell::framework::ExtractRefLine> extract_ref_line) {
    exit_scene_speed_limit_ = {{99, 1}, {99, 1}};
    double exit_end_dist = scenario_judge_model->exit_end_dist();
    double exit_start_dist = scenario_judge_model->exit_start_dist();
    std::vector<uint32_t> link_id_vec = candidate_lanes_model->link_id_vec();
    uint8_t exit_start_road_class = scenario_judge_model->exit_start_road_class();
    if (exit_end_dist > 0) {
        double lane_speed_limit = 0;
        double lane_speed_limit2 = 0;
        // LOGI("p_exit_start_end_gap: {}", p_exit_start_end_gap);
        if ((exit_end_dist > 0 && exit_start_dist < 0) || (exit_end_dist - exit_start_dist > p_exit_start_end_gap)){
            //找起点后第一段限速
            double dist = 0;
            for(auto iter : lane_type_vec_){
                dist += static_cast<double>(iter.link_dist_)/100.0;
                if(dist+0.0001 >= exit_end_dist){
                    lane_speed_limit = iter.speed_;
                    break;
                }
            }
                    //     std::cout << __FILE__ << "," << __LINE__ << ","
                    //   << " lane_speed_limit: " << lane_speed_limit << std::endl;
            double speed_limit_temp = std::max(60.0, lane_speed_limit) / 3.6;
            exit_scene_speed_limit_[0] = std::make_pair(speed_limit_temp, exit_end_dist);
        } else if (exit_end_dist > 0 && exit_start_dist > 0 && exit_end_dist > exit_start_dist &&
                   exit_start_dist < 300) {
            //找起点后第一段限速
            double dist = 0;
            for(auto iter : lane_type_vec_){
                dist += static_cast<double>(iter.link_dist_)/100.0;
                if(scenario_judge_model->exit_start_speed_limit_no_connect() != 0){
                    lane_speed_limit = scenario_judge_model->exit_start_speed_limit_no_connect();
                    break;
                }else{
                    if(dist+0.0001 >= exit_start_dist){
                        lane_speed_limit = iter.speed_;
                        break;
                    }                    
                }

            }
                    //     std::cout << __FILE__ << "," << __LINE__ << ","
                    //   << "222 lane_speed_limit: " << lane_speed_limit << std::endl;
            double speed_limit_temp = std::max(60.0, lane_speed_limit) / 3.6;
            exit_scene_speed_limit_[0] = std::make_pair(speed_limit_temp , exit_start_dist);

            //找起点后第一段限速
            double dist2 = 0;
            for(auto iter : lane_type_vec_){
                dist2 += static_cast<double>(iter.link_dist_)/100.0;
                if(dist2+0.0001 >= exit_end_dist){
                    lane_speed_limit2 = iter.speed_;
                    break;
                }
            }
                    //     std::cout << __FILE__ << "," << __LINE__ << ","
                    //   << " lane_speed_limit2: " << lane_speed_limit2 << std::endl;
            double speed_limit_temp2 = std::max(60.0, lane_speed_limit2) / 3.6;
            exit_scene_speed_limit_[1] = std::make_pair(speed_limit_temp2, exit_end_dist);

            //如果是高架，那么取下匝道终点里的限速和 65的大者
            if(exit_start_road_class == 2){//2-高架；1-高速
                if(exit_scene_speed_limit_[1].first < 95){//终点限速存在                   
                    double speed_limit_change = std::min(p_exit_start_speed_urb_expway/3.6, exit_scene_speed_limit_[1].first);
                    exit_scene_speed_limit_[0] = std::make_pair(speed_limit_change , exit_start_dist);

                }
            }

        }
    }
                    //     std::cout << __FILE__ << "," << __LINE__ << ","
                    //   << " exit_scene_speed_limit_ 0: " << exit_scene_speed_limit_[0].first << std::endl;    
                    //                           std::cout << __FILE__ << "," << __LINE__ << ","
                    //   << " exit_scene_speed_limit_ 0 dist: " << exit_scene_speed_limit_[0].second << std::endl;  
                    //     std::cout << __FILE__ << "," << __LINE__ << ","
                    //   << " exit_scene_speed_limit_ 1: " << exit_scene_speed_limit_[1].first << std::endl;    
                    //                           std::cout << __FILE__ << "," << __LINE__ << ","
                    //   << " exit_scene_speed_limit_ 1 dist: " << exit_scene_speed_limit_[1].second << std::endl;                       
}

void SpeedLimit::speed_limmit_sd(
    const std::shared_ptr<earth::shell::framework::CandidateLanesModel> candidate_lanes_model,
    const std::shared_ptr<earth::shell::framework::ScenarioJudgeModel> scenario_judge_model,
    const std::shared_ptr<earth::shell::framework::ExtractRefLine> extract_ref_line) {
    speed_limit_source_ = 0;
    uint8_t cur_HD_speed = cur_speed_;
    uint8_t next_HD_speed = next_speed_;
    double next_HD_speed_dist = next_speed_dist_;
    uint8_t ego_lane_type = extract_ref_line->ego_path().LaneType;
    double ego_offset = static_cast<double>(map_position_->PathOffset) / 100;
    if(scenario_judge_model->is_ramp_to_main_road()){
        // std::cout << "left_lane_spd_limit: " << int(scenario_judge_model->left_lane_spd_limit()) << std::endl;
        cur_HD_speed = std::max(last_speed_limit_final_value_, scenario_judge_model->left_lane_spd_limit()) ;
    }
    // std::cout << __FILE__ << "," << __LINE__ << ",  "
    //           << " com_info_->OSM2_St_SpdLimOfRoad: " << (int)com_info_->OSM2_St_SpdLimOfRoad << std::endl;  

    uint8_t cur_SD_speed = 0; 
    if (com_info_->OSM2_St_SpdLimOfRoad != 0) {
        cur_speed_ = std::max(static_cast<int>(com_info_->OSM2_St_SpdLimOfRoad), 60);
        cur_SD_speed = cur_speed_;
        speed_limit_source_ = 2;//sd
    } else {
        cur_speed_ = std::max(static_cast<int>(cur_HD_speed), 50);
        speed_limit_source_ = 1; //hd
    }
    // std::vector<uint32_t> link120 = {101720086,101720084,101720077,101720109,109800159,101720295,107407849};
    // cur_SD_speed = 80;
    // for(auto link: link120){
    //     if(map_position_->LinkId == link){
    //        cur_SD_speed = 120;
    //        break;
    //     }
    // }
    // ZTEXT("EFM_INFO", "cur_SD_speed_t: ", 140, 90, "cur_SD_speed_t: {}", cur_SD_speed);
    // std::cout << __FILE__ << "," << __LINE__ << ",use sd, cur_speed_: " << (int)cur_speed_<<std::endl;
    next_speed_ = 0;
    next_speed_dist_ = 0;
    std::vector<uint8_t> lane_num_vec{};
    std::vector<uint32_t> link_id_vec{};
    // switch (extract_ref_line->prior_path_index()) {
    //     case 0:
    //         lane_num_vec = candidate_lanes_model->ego_path_lane_num();
    //         break;
    //     case 1:
    //         lane_num_vec = candidate_lanes_model->left_path_lane_num();
    //         break;
    //     case 2:
    //         lane_num_vec = candidate_lanes_model->right_path_lane_num();
    //         break;
    //     default:
    //         lane_num_vec = candidate_lanes_model->ego_path_lane_num();
    //         break;
    // }
    lane_num_vec = candidate_lanes_model->ref_route_path_lane_num();
    link_id_vec = candidate_lanes_model->link_id_vec();
    if (lane_num_vec.size() > link_id_vec.size()) {
        return;
    }
    // 用sd的时候，不能有hd的next speed
    bool need_next_speed_f = false;
    for (int i = 0; i < lane_num_vec.size(); i++) {
        uint8_t lane_num = lane_num_vec[i];
        uint32_t link_id = link_id_vec[i];
        message::map_map::s_LinkInfo_t link_infos;
        if (link_id_index_lane_info_map_->find(link_id) != link_id_index_lane_info_map_->end()) {
            int link_index = link_id_index_lane_info_map_->at(link_id);
            link_infos = map_static_info_->LinkInfos.LinkInfos[link_index];
        } else {
            // std::cout << __FILE__ << "," << __LINE__ << ",link not found: " << link_id << std::endl;
            break;
        }
        if (i == 0) {
            for (auto lane : link_infos.LaneInfos.LaneInfos) {
                if (lane.LaneNum.LaneNum == lane_num) {
                    uint8_t prior_lane_type = lane.LaneType.data;
                    if (prior_lane_type == 0 &&
                        (ego_lane_type == 5 || ego_lane_type == 2 || ego_lane_type == 1 || ego_lane_type == 4 ||
                         ego_lane_type == 6 || ego_lane_type == 7 || ego_lane_type == 8)) {
                        // on ramp, use hd
                        cur_speed_ = cur_HD_speed;
                        need_next_speed_f = true;
                        speed_limit_source_ = 1; //hd
                        break;
                    } else {
                        // just on ramp, lock hd for a while, 但是不需要下一段限速
                        if ((last_cycle_ego_lane_type_ == 5 || last_cycle_ego_lane_type_ == 2 ||
                             last_cycle_ego_lane_type_ == 1 || last_cycle_ego_lane_type_ == 4 ||
                             last_cycle_ego_lane_type_ == 6 || last_cycle_ego_lane_type_ == 7 ||
                             last_cycle_ego_lane_type_ == 8) &&
                            (ego_lane_type == 0)) {
                            on_ramp_lock_start_f_ = true;
                            on_ramp_pos_offset_ = map_position_->PathOffset;
                            cur_speed_ = cur_HD_speed;
                            speed_limit_source_ = 1;//hd
                            // need_next_speed_f = true;
                        } else if (on_ramp_lock_start_f_ == true) {
                            if (map_position_->PathOffset > on_ramp_pos_offset_) {
                                if ((map_position_->PathOffset - on_ramp_pos_offset_) < 5000) {
                                    cur_speed_ = cur_HD_speed;
                                    speed_limit_source_  = 1; //hd
                                    // need_next_speed_f = true;
                                } else {
                                    on_ramp_lock_start_f_ = false;
                                    on_ramp_pos_offset_ = 0;
                                }
                            } else {
                                // errror
                                on_ramp_lock_start_f_ = false;
                                on_ramp_pos_offset_ = 0;
                            }
                        }
                    }
                }
            }
            last_cycle_ego_lane_type_ = ego_lane_type;
        }
        // 下匝道前1km(可标)，当前和下一段发HD
        double dist_to_link = static_cast<double>(link_infos.PathOffset.PathOffset) / 100 - ego_offset;
        //std::cout << __FILE__ << "," << __LINE__ << ", " << "i: " << i << " dist_to_link: " << dist_to_link
        //          << " ,link_offset:" << static_cast<double>(link_infos.PathOffset.PathOffset) / 100
        //          << " ,ego_offset:" << ego_offset << " ,link_id: " << link_id << " ,lane_num: " << (int)lane_num
        //          << std::endl;
        if (dist_to_link <= p_offramp_speed_lmt_dist) {
            for (auto lane : link_infos.LaneInfos.LaneInfos) {
                if (lane.LaneNum.LaneNum == lane_num) {
                    if (lane.LaneType.data == 5 || lane.LaneType.data == 2 || lane.LaneType.data == 1 ||
                        lane.LaneType.data == 4 || lane.LaneType.data == 6 || lane.LaneType.data == 7 ||
                        lane.LaneType.data == 8) {
                        cur_speed_ = cur_HD_speed;
                        speed_limit_source_ = 1;//hd
                        need_next_speed_f = true;
                        break;
                    }
                }
            }
        }

        if (need_next_speed_f == false) {
            
        } else {
            next_speed_ = next_HD_speed;
            next_speed_dist_ = next_HD_speed_dist;
            break;
        }
    }
    // std::cout << __FILE__ << "," << __LINE__ << ",before tunel logic, cur_speed_: " << (int)cur_speed_<<std::endl;
    // tunnel speed limit, 隧道前300m(标定)，当前车速切成hd, 下一段限速发出hd的下一段限速；出隧道后保持200m；
    double start_dist = 100000;
    double end_dist = 100000;
    uint8_t special_type = SpecialScene(scenario_judge_model, start_dist, end_dist);//1-toll, 3- tunnel
    bool has_tunnel_f =false;
    bool has_toll_f = false;
    // std::cout << __FILE__ << "," << __LINE__ << "has_tunnel_f: " << (int)has_tunnel_f<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ",tunnel_end_dist: " << tunnel_end_dist<<std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ",tunnel_start_dist: " << tunnel_start_dist<<std::endl;
    if (special_type == 3 && p_tunnel_use_hd_speed !=0) {
        if ((start_dist < 0.0001 && end_dist > 0 && end_dist < 100000 - 0.0001) ||
            (start_dist >= 0.0001 && start_dist <= p_tunnel_speed_lmt_front_dist &&
             end_dist >= start_dist && end_dist < 100000 - 0.0001)) {
            has_tunnel_f = true;
        } else {
            has_tunnel_f = false;
        }
    }else if(special_type == 1){
        if ((start_dist < 0.0001 && end_dist > 0 && end_dist < 100000 - 0.0001) ||
            (start_dist >= 0.0001 && start_dist <= p_toll_speed_lmt_front_dist  &&
             end_dist >= start_dist && end_dist < 100000 - 0.0001)) {
            has_toll_f = true;
        } else {
            has_toll_f = false;
        }
    }

    // std::cout << __FILE__ << "," << __LINE__ << "has_tunnel_f: " << (int)has_tunnel_f<<std::endl;
    ZTEXT("EFM_INFO", "just_pass_tunnel_f_: ", 140, 80, "just_pass_tunnel_f_: {}", just_pass_tunnel_f_);
    ZTEXT("EFM_INFO", "last_cycle_tunnel_f_: ", 140, 83, "last_cycle_tunnel_f_: {}", last_cycle_tunnel_f_);
    
    int tunnel_logic = 0;
    if (has_tunnel_f == true) {
        // 当前隧道，或距离隧道p_tunnel_speed_lmt_front_dist m
        if((last_speed_limit_source_ ==2 || last_speed_limit_source_ ==3)
            && last_speed_limit_final_value_<cur_HD_speed && last_speed_limit_final_value_>0){
            cur_speed_ = last_speed_limit_final_value_;
            next_speed_ = next_HD_speed;
            next_speed_dist_ = next_HD_speed_dist;
            speed_limit_source_ = 3;//last value
            tunnel_logic = 1;
        }else{
            cur_speed_ = cur_HD_speed;
            next_speed_ = next_HD_speed;
            next_speed_dist_ = next_HD_speed_dist;
            speed_limit_source_ = 1; //hd
            tunnel_logic = 2;
        }
        just_pass_tunnel_offset_ = 0;
        just_pass_tunnel_f_ = false;
        
    } else {
// 出隧道场景限速策略变更内容如下：
// 1）如果HD与SD一致，则切SD；
// 2）HD与SD不一致，则继续使用HD限速，待一致后再切为SD
        if (last_cycle_tunnel_f_ == true && has_tunnel_f == false) {
            just_pass_tunnel_f_ = true;
            if(cur_SD_speed == cur_HD_speed){
                cur_speed_ = cur_SD_speed;
                speed_limit_source_ = 2;//sd
                just_pass_tunnel_f_ = false;
                tunnel_logic = 3;
            }else{
                cur_speed_ = cur_HD_speed;
                next_speed_ = next_HD_speed;
                next_speed_dist_ = next_HD_speed_dist;
                speed_limit_source_ = 1;//hd  
                tunnel_logic = 4;              
            }
            // just_pass_tunnel_offset_ = map_position_->PathOffset;

        } else if (just_pass_tunnel_f_ == true) {
            if(cur_SD_speed == cur_HD_speed){
                cur_speed_ = cur_SD_speed;
                speed_limit_source_ = 2;//sd
                just_pass_tunnel_f_ = false;
                tunnel_logic = 5;
            }else{
                cur_speed_ = cur_HD_speed;
                next_speed_ = next_HD_speed;
                next_speed_dist_ = next_HD_speed_dist;
                speed_limit_source_ = 1;//hd  
                tunnel_logic = 6;               
            }
            // if (map_position_->PathOffset - just_pass_tunnel_offset_ <= p_tunnel_speed_lmt_keep_dist * 100 &&
            //     map_position_->PathOffset >= just_pass_tunnel_offset_) {
            //     cur_speed_ = cur_HD_speed;
            //     next_speed_ = next_HD_speed;
            //     next_speed_dist_ = next_HD_speed_dist;
            //     speed_limit_source_ = 1;//hd
            // } else if (map_position_->PathOffset - just_pass_tunnel_offset_ > p_tunnel_speed_lmt_keep_dist * 100) {
            //     just_pass_tunnel_f_ = false;
            //     just_pass_tunnel_offset_ = 0;
            // } else {
            //     // error
            //     just_pass_tunnel_f_ = false;
            //     just_pass_tunnel_offset_ = 0;
            // }
        }
    }
    last_cycle_tunnel_f_ = has_tunnel_f;
    ZTEXT("EFM_INFO", "tunnel_logic: ", 140, 86, "tunnel_logic: {}", tunnel_logic);
    //std::cout << __FILE__ << "," << __LINE__ << "," << " last_cycle_tunnel_f_: " << last_cycle_tunnel_f_ << std::endl;
    //std::cout << __FILE__ << "," << __LINE__ << "," << " has_tunnel_f: " << has_tunnel_f << std::endl;
    //std::cout << __FILE__ << "," << __LINE__ << "," << " just_pass_tunnel_offset_: " << just_pass_tunnel_offset_ << std::endl;

    //收费站处理，收费站1.2km前，用hd,因为sd在1000m就会给40，太早
    if (has_toll_f == true) {
        // 收费站前限速用sd的和hd的取小
        if (com_info_->OSM2_St_SpdLimOfRoad != 0 && cur_HD_speed>(std::max(static_cast<int>(com_info_->OSM2_St_SpdLimOfRoad), 50))) {
            cur_speed_ = std::max(static_cast<int>(com_info_->OSM2_St_SpdLimOfRoad), 50);
            speed_limit_source_ = 2;//sd
        }else{
            cur_speed_ = cur_HD_speed;
            next_speed_ = next_HD_speed;
            next_speed_dist_ = next_HD_speed_dist;
            speed_limit_source_ = 1;//hd
        } 
    }

    //1126 new
        ZTEXT("EFM_INFO", "main_split_two_main: ", 140, 48, "main_split_two_main: {}", scenario_judge_model->main_split_two_main());
        ZTEXT("EFM_INFO", "end_dist: ", 140, 45, "end_dist: {}", scenario_judge_model->road_split_end_dist());
        ZTEXT("EFM_INFO", "start_dist: ", 140, 42, "start_dist: {}", scenario_judge_model->road_split_start_dist());

        // std::cout << __FILE__ << "," << __LINE__ << " main_split_two_main: " << scenario_judge_model->main_split_two_main() << std::endl;
    if(scenario_judge_model->main_split_two_main() == true){
        uint32_t link_id = scenario_judge_model->road_split_end_link_id();
        uint8_t lane_num = scenario_judge_model->road_split_lane_num_split();
        double start_dist = scenario_judge_model->road_split_start_dist();
        message::map_map::s_LaneSpeedLimit_t lane_speed_limit{};
        uint8_t road_main_road_split_hd_speed = 0;
        if(efm::MapCommonTool::GetInstance()->GetLaneSpeed(map_static_info_, link_id,lane_num, lane_speed_limit)){
            road_main_road_split_hd_speed = lane_speed_limit.ValueH.ValueH;
        }
        // std::cout << __FILE__ << "," << __LINE__ << " road_main_road_split_hd_speed: " << (int)road_main_road_split_hd_speed << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << " cur_SD_speed: " << (int)cur_SD_speed << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << " start_dist: " << start_dist << std::endl;
        if(road_main_road_split_hd_speed!=0 && (cur_SD_speed > (road_main_road_split_hd_speed +20)) && start_dist<1000.0){
            cur_speed_ = cur_HD_speed;
            next_speed_ = next_HD_speed;
            next_speed_dist_ = next_HD_speed_dist;
            speed_limit_source_ = 1;//hd 
            main_road_use_hd_ = true;           
        }
    }
    // std::cout << __FILE__ << "," << __LINE__ << " main_road_use_hd_: " << main_road_use_hd_ << std::endl;
    if(main_road_use_hd_ == true){
        if(cur_HD_speed == cur_SD_speed){
            main_road_use_hd_ = false;
        }else{
            cur_speed_ = cur_HD_speed;
            next_speed_ = next_HD_speed;
            next_speed_dist_ = next_HD_speed_dist;
            speed_limit_source_ = 1;//hd 
        }
    }
    // std::cout << __FILE__ << "," << __LINE__ << "final main_road_use_hd_: " << main_road_use_hd_ << std::endl;
    return;
}

void SpeedLimit::speed_ZTEXT() {
    // for (int i = 0; i < 8; i++) {
    //     uint8_t data = traffic_info_->FrntCam_Bus_TrfcSign[i].MainType.data;
    //     if (data != 0 && hist_camera_spd_limit_[i] != data) {
    //         // store hist
    //         for (int k = 0; k < 8; k++) {
    //             uint8_t data2 = traffic_info_->FrntCam_Bus_TrfcSign[k].MainType.data;
    //             hist_camera_spd_limit_[k] = data2;
    //         }
    //         break;
    //     }
    // }

    // for (int i = 0; i < 8; i++) {
    //     uint8_t data = hist_camera_spd_limit_[i];
    //     switch (i) {
    //         case 0:
    //             ZTEXT("EFM_INFO", "cameraSpeed0: ", 140, 27, "cameraSpeed0: {}", data);
    //             break;
    //         case 1:
    //             ZTEXT("EFM_INFO", "cameraSpeed1: ", 140, 30, "cameraSpeed1: {}", data);
    //             break;
    //         case 2:
    //             ZTEXT("EFM_INFO", "cameraSpeed2: ", 140, 33, "cameraSpeed2: {}", data);
    //             break;
    //         case 3:
    //             ZTEXT("EFM_INFO", "cameraSpeed3: ", 140, 36, "cameraSpeed3: {}", data);
    //             break;
    //         case 4:
    //             ZTEXT("EFM_INFO", "cameraSpeed4: ", 140, 39, "cameraSpeed4: {}", data);
    //             break;
    //         case 5:
    //             ZTEXT("EFM_INFO", "cameraSpeed5: ", 140, 42, "cameraSpeed5: {}", data);
    //             break;
    //         case 6:
    //             ZTEXT("EFM_INFO", "cameraSpeed6: ", 140, 45, "cameraSpeed6: {}", data);
    //             break;
    //         case 7:
    //             ZTEXT("EFM_INFO", "cameraSpeed7: ", 140, 48, "cameraSpeed7: {}", data);
    //             break;
    //     }
    // }
    ZTEXT("EFM_INFO", "just_pass_tunnel_f_: ", 140, 39, "just_pass_tunnel_f_: {}", just_pass_tunnel_f_);
    ZTEXT("EFM_INFO", "main_road_use_hd_: ", 140, 36, "main_road_use_hd_: {}", main_road_use_hd_);
    ZTEXT("EFM_INFO", "SD_Speed: ", 140, 52, "SD_Speed: {}", com_info_->OSM2_St_SpdLimOfRoad);
    // ZTEXT("EFM_INFO", "cur_speed_final: ", 140, 55, "cur_speed_final: {}", int(cur_speed_));
    // ZTEXT("EFM_INFO", "next_speed_final: ", 140, 58, "next_speed_final: {}", int(next_speed_));
    // ZTEXT("EFM_INFO", "next_speed_dist_final: ", 140, 61, "next_speed_dist_final: {}", next_speed_dist_);
}

void SpeedLimit::init_global_var() {
    on_ramp_pos_offset_ = 0;
    last_cycle_ego_lane_type_ = 0;
    on_ramp_lock_start_f_ = false;
    last_cycle_tunnel_f_ = false;
    just_pass_tunnel_f_ = false;
    just_pass_tunnel_offset_ = 0;
    main_road_use_hd_ = false;    
    return;
}

bool SpeedLimit::GetLinkInfos(uint32_t link_id, message::map_map::s_LinkInfo_t& link_infos) {
    if (link_id_index_lane_info_map_->find(link_id) != link_id_index_lane_info_map_->end()) {
        int link_index = link_id_index_lane_info_map_->at(link_id);
        link_infos = map_static_info_->LinkInfos.LinkInfos[link_index];
        return true;
    } else {
        return false;
    }
}

bool SpeedLimit::get_lane_curv(std::vector<lane_curv>& CurvPoints, uint32_t link_id, uint8_t lane_id,
                               bool is_first_link, uint32_t begin_length) {
    CurvPoints.clear();
    int relative_offset = 0;
    if (is_first_link) {
        message::map_map::s_LinkInfo_t link_infos;
        if (false == GetLinkInfos(link_id, link_infos)) {
            // static data none
            return false;
        }

        if (map_position_->PathOffset >= link_infos.EndOffset.EndOffset ||
            map_position_->PathOffset < link_infos.PathOffset.PathOffset) {
            return false;
        }
        relative_offset = map_position_->PathOffset - link_infos.PathOffset.PathOffset;
    }
    // std::cout << __FILE__ << "," << __LINE__ << " link_id: " << link_id << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << " relative_offset: " << relative_offset << std::endl;

    for (auto curvature_info : map_static_info_->LinkCurvatures.LinkCurvatures) {
        if (link_id == curvature_info.InstanceId.InstanceId && lane_id == curvature_info.LaneNum.LaneNum) {
            for (int32_t i = 0; i < curvature_info.CurvCount.CurvCount; i++) {
                lane_curv temp_curv;
                temp_curv.offset_ = curvature_info.CurvPoints.CurvPoints[i].PathOffset.PathOffset;
                temp_curv.curv_ = curvature_info.CurvPoints.CurvPoints[i].CurvPointValue.CurvPointValue;

                if (temp_curv.offset_ < relative_offset) {
                    uint32_t end_offset = 0;
                    if (i == curvature_info.CurvCount.CurvCount - 1) {
                        temp_curv.offset_ = 0 + begin_length;
                        CurvPoints.push_back(temp_curv);
                    } else if (curvature_info.CurvPoints.CurvPoints[i + 1].PathOffset.PathOffset > relative_offset) {
                        temp_curv.offset_ = 0 + begin_length;
                        CurvPoints.push_back(temp_curv);
                    } else {
                        continue;
                    }
                } else {
                    temp_curv.offset_ = temp_curv.offset_ - relative_offset + begin_length;
                    CurvPoints.push_back(temp_curv);
                }
            }
            break;
        }
    }
    return true;
}

bool SpeedLimit::change_speed_for_curv(std::vector<lane_type_speed>& lane_type_vec,
                                       std::vector<uint8_t>& speed_change_vec,
                                       std::vector<std::vector<lane_curv>> CurvPoints_list) {
    if (0 == lane_type_vec.size() || lane_type_vec.size() != CurvPoints_list.size()) {
        return false;
    }

    std::vector<lane_type_speed> temp_lane_type;
    temp_lane_type.assign(lane_type_vec.begin(), lane_type_vec.end());
    lane_type_vec.clear();

    // change speed for curv
    lane_type_speed last_lane_type_data;
    std::vector<lane_curv> last_lane_curv_data;
    for (size_t i = 0; i < temp_lane_type.size(); i++) {
        if (4 != temp_lane_type[i].lane_type_) {
            if (last_lane_type_data.link_dist_ > 0) {
                insert_lane_type(lane_type_vec, last_lane_type_data, last_lane_curv_data, temp_lane_type[i].speed_);
            }
            last_lane_type_data.speed_ = 0;
            last_lane_type_data.lane_type_ = 0;
            last_lane_type_data.link_dist_ = 0;
            last_lane_curv_data.clear();
            lane_type_vec.push_back(temp_lane_type[i]);
        } else {
            if (0 == last_lane_type_data.link_dist_) {
                last_lane_type_data = temp_lane_type[i];
                last_lane_curv_data.assign(CurvPoints_list[i].begin(), CurvPoints_list[i].end());
            } else {
                for (size_t idx = 0; idx < CurvPoints_list[i].size(); idx++) {
                    lane_curv temp_curv_data = CurvPoints_list[i][idx];
                    temp_curv_data.offset_ += last_lane_type_data.link_dist_;
                    last_lane_curv_data.push_back(temp_curv_data);
                }
                last_lane_type_data.link_dist_ += temp_lane_type[i].link_dist_;
                last_lane_type_data.speed_ = std::max(last_lane_type_data.speed_, temp_lane_type[i].speed_);
            }
        }
    }

    if (last_lane_type_data.link_dist_ > 0) {
        insert_lane_type(lane_type_vec, last_lane_type_data, last_lane_curv_data, RAMP_LANE_MAX_SPEED_TOP);
    }

    if (lane_type_vec.size() < 2) {
        return true;
    }

    // get speed_change_vec
    speed_change_vec.clear();
    for (size_t i = 1; i < lane_type_vec.size(); i++) {
        int cur_speed = lane_type_vec[i].speed_;
        int back_speed = lane_type_vec[i - 1].speed_;
        if ((abs((int)lane_type_vec[i].speed_ - (int)lane_type_vec[i - 1].speed_) >= SPEED_INTERPOLATION) &&
            0 != lane_type_vec[i].speed_ && 0 != lane_type_vec[i - 1].speed_) {
            speed_change_vec.push_back(i);
        }
    }
    return true;
}

bool SpeedLimit::insert_lane_type(std::vector<lane_type_speed>& lane_type_vec, lane_type_speed lane_type,
                                  std::vector<lane_curv> CurvPoints, uint8_t max_speed) {
    if (0 == lane_type.link_dist_) {
        return false;
    }

    if (0 == CurvPoints.size()) {
        lane_type_vec.push_back(lane_type);
        return false;
    }

    lane_type_speed temp_lane_type = lane_type;
    // lane_curv last_curv;
    uint32_t last_curv_length = 0;
    uint8_t last_speed = 0;
    // std::cout << __FILE__ << "," << __LINE__ << " size: " << CurvPoints.size() << std::endl;
    std::vector<uint8_t> all_speeds;
    std::vector<int> all_lengths;
    for (size_t i = 0; i < CurvPoints.size(); i++) {
        uint8_t temp_speed = 0;
        converte_curv_speed(CurvPoints[i].curv_, temp_speed, max_speed);
        // if (0 == i && temp_speed > 50) {
        //     temp_speed = 50;
        // }

        int temp_lane_curv_lenth = 0;
        if (i == CurvPoints.size() - 1) {
            temp_lane_curv_lenth = lane_type.link_dist_ - CurvPoints[i].offset_;
        } else {
            temp_lane_curv_lenth = CurvPoints[i + 1].offset_ - CurvPoints[i].offset_;
        }
        temp_lane_curv_lenth = temp_lane_curv_lenth > 0 ? temp_lane_curv_lenth : 0;

        // std::cout << __FILE__ << "," << __LINE__ << " i: " << i << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << " curv_: " << CurvPoints[i].curv_ << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << " temp_speed: " << (int)temp_speed<< std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << " max_speed: " << (int)max_speed<< std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << " temp_lane_curv_lenth: " << temp_lane_curv_lenth<< std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << " last_curv_length: " << last_curv_length<< std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << " last_speed: " << (int)last_speed<< std::endl;

        if (0 == last_curv_length || temp_speed == last_speed) {
            // last_curv = CurvPoints[i];
            last_curv_length += temp_lane_curv_lenth;
            last_speed = temp_speed;
        } else {
            all_speeds.push_back(last_speed);
            all_lengths.push_back(last_curv_length);
            last_curv_length = temp_lane_curv_lenth;
            last_speed = temp_speed;
            // if (last_curv_length < ((double)temp_speed/3.600)*3 || 0 == last_speed){//30m
            //     last_curv_length += temp_lane_curv_lenth;
            //     // last_speed = temp_speed;
            // }
            // else{
            //     temp_lane_type.speed_ = last_speed;
            //     temp_lane_type.link_dist_ = last_curv_length;
            //     lane_type_vec.push_back(temp_lane_type);
            //     last_curv_length = temp_lane_curv_lenth;
            //     last_speed = temp_speed;
            // }
        }
    }
    if (last_curv_length > 0 && last_speed > 0) {
        // std::cout << __FILE__ << "," << __LINE__ << " last_curv_length: " << last_curv_length<< std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << " last_speed: " << (int)last_speed<< std::endl;
        // temp_lane_type.speed_ = last_speed;
        // temp_lane_type.link_dist_ = last_curv_length;
        // lane_type_vec.push_back(temp_lane_type);
        all_speeds.push_back(last_speed);
        all_lengths.push_back(last_curv_length);
    }

    if (all_speeds.size() != all_lengths.size() || 0 == all_speeds.size()) {
        lane_type_vec.push_back(lane_type);
        return false;
    }
    if (false == smooth_curv_speed(all_speeds, all_lengths)) {
        lane_type_vec.push_back(lane_type);
        return false;
    }

    int min_speed_idx = -1;
    uint8_t min_speed = 200;
    for (size_t i = 0; i < all_speeds.size(); i++) {
        temp_lane_type.speed_ = all_speeds[i];
        temp_lane_type.link_dist_ = all_lengths[i];
        lane_type_vec.push_back(temp_lane_type);
        // std::cout << __FILE__ << "," << __LINE__ << " speed_: " << temp_lane_type.speed_ << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << " link_dist_: " << temp_lane_type.link_dist_ << std::endl;
    }

    return true;
}

bool SpeedLimit::smooth_curv_speed(std::vector<uint8_t>& all_speeds, std::vector<int>& all_lengths) {
    if (all_speeds.size() != all_lengths.size() || 0 == all_lengths.size()) {
        return false;
    }

    uint8_t temp_speed = 0;
    int idx = 0;
    // 50
    temp_speed = 50;
    while (idx < all_speeds.size()) {
        int first_exit = -1;
        int second_exit = -1;
        if (false == smooth_curv_speed_rang(all_speeds, all_lengths, first_exit, second_exit, temp_speed)) {
            break;
        }
        // std::cout << __FILE__ << "," << __LINE__ << " first_exit: " << first_exit << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << " second_exit: " << second_exit << std::endl;
        smooth_curv_speed_combine(all_speeds, all_lengths, first_exit, second_exit);
        idx++;
    }

    // 40
    temp_speed = 40;
    idx = 0;
    while (idx < all_speeds.size()) {
        int first_exit = -1;
        int second_exit = -1;
        if (false == smooth_curv_speed_rang(all_speeds, all_lengths, first_exit, second_exit, temp_speed)) {
            break;
        }
        smooth_curv_speed_combine(all_speeds, all_lengths, first_exit, second_exit);
        // std::cout << __FILE__ << "," << __LINE__ << " first_exit: " << first_exit << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << " second_exit: " << second_exit << std::endl;
        idx++;
    }

    // 30
    temp_speed = 30;
    idx = 0;
    while (idx < all_speeds.size()) {
        int first_exit = -1;
        int second_exit = -1;
        if (false == smooth_curv_speed_rang(all_speeds, all_lengths, first_exit, second_exit, temp_speed)) {
            break;
        }
        smooth_curv_speed_combine(all_speeds, all_lengths, first_exit, second_exit);
        // std::cout << __FILE__ << "," << __LINE__ << " first_exit: " << first_exit << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << " second_exit: " << second_exit << std::endl;
        idx++;
    }

    return true;
}

bool SpeedLimit::smooth_curv_speed_combine(std::vector<uint8_t>& all_speeds, std::vector<int>& all_lengths,
                                           int& first_exit, int& second_exit) {
    if (all_speeds.size() != all_lengths.size() || 0 == all_lengths.size() || first_exit >= second_exit ||
        first_exit >= all_speeds.size() || second_exit >= all_speeds.size()) {
        return false;
    }

    std::vector<uint8_t> temp_all_speeds;
    temp_all_speeds.assign(all_speeds.begin(), all_speeds.end());
    std::vector<int> temp_all_lengths;
    temp_all_lengths.assign(all_lengths.begin(), all_lengths.end());
    all_speeds.clear();
    all_lengths.clear();

    int need_length = 0;
    uint8_t need_speed = temp_all_speeds[first_exit];
    for (size_t i = 0; i < temp_all_speeds.size(); i++) {
        if (i < first_exit) {
            all_speeds.push_back(temp_all_speeds[i]);
            all_lengths.push_back(temp_all_lengths[i]);
        } else if (i >= first_exit && i <= second_exit) {
            need_length += temp_all_lengths[i];
        } else {
            if (need_length > 0) {
                all_speeds.push_back(need_speed);
                all_lengths.push_back(need_length);
            }

            all_speeds.push_back(temp_all_speeds[i]);
            all_lengths.push_back(temp_all_lengths[i]);

            need_speed = 0;
            need_length = 0;
        }
    }

    if (need_length > 0) {
        all_speeds.push_back(need_speed);
        all_lengths.push_back(need_length);
    }

    return true;
}

bool SpeedLimit::smooth_curv_speed_rang(std::vector<uint8_t>& all_speeds, std::vector<int>& all_lengths,
                                        int& first_exit, int& second_exit, uint8_t speed) {
    first_exit = -1;
    second_exit = -1;
    if (all_speeds.size() != all_lengths.size() || 0 == all_lengths.size()) {
        return false;
    }

    bool is_exit = false;
    for (size_t i = 0; i < all_speeds.size(); i++) {
        if (all_speeds[i] < speed) {
            is_exit = false;
            first_exit = -1;
        } else if (all_speeds[i] == speed) {
            if (is_exit && first_exit >= 0) {
                second_exit = i;
                break;
            } else {
                is_exit = true;
                first_exit = i;
            }
        }
    }
    if (first_exit >= 0 && second_exit > first_exit) {
        return true;
    } else {
        return false;
    }

    return true;
}

bool SpeedLimit::converte_curv_speed(float curv, uint8_t& tar_speed, uint8_t max_speed) {
    if (std::fabs(curv) >= (float)1428.6) {
        //<70m
        tar_speed = 30;
    } else if (std::fabs(curv) >= (float)1000) {
        // 71-100m
        tar_speed = 40;
        // }else if (std::fabs(curv) >= (float)125.0){
        //     //151-800m
        //     tar_speed = 50;
        // }else if (std::fabs(curv) >= (float)167.0){
        //     //201-600m
        //     tar_speed = 50;
        // }else if (std::fabs(curv) >= (float)125.0){
        //     //201-600m
        //     tar_speed = 60;
        // }else if (std::fabs(curv) >= (float)200.0){
        //     //500m
        //     tar_speed = std::min(70, (int)max_speed);
    } else {
        tar_speed = 50;
    }
    return true;
}

void SpeedLimit::GetInsertDist(uint8_t lower_speed, uint32_t& insert_dist, uint8_t& insert_val) {
    if (lower_speed <= 30) {
        insert_dist = 2000;
        insert_val = lower_speed + LOWER_SPEED_INTERPOLATION;
    } else if (lower_speed <= 40) {
        insert_dist = 12000;
        insert_val = lower_speed + SPEED_INTERPOLATION;
    } else if (lower_speed <= 50) {
        insert_dist = 15000;
        insert_val = lower_speed + SPEED_INTERPOLATION;
    } else if (lower_speed <= 60) {
        insert_dist = 15000;
        insert_val = lower_speed + SPEED_INTERPOLATION;
    } else if (lower_speed <= 70) {
        insert_dist = 30000;
        insert_val = lower_speed + SPEED_INTERPOLATION;
    } else if (lower_speed <= 80) {
        insert_dist = 30000;
        insert_val = lower_speed + SPEED_INTERPOLATION;
    } else if (lower_speed <= 90) {
        insert_dist = 35000;
        insert_val = lower_speed + SPEED_INTERPOLATION;
    } else if (lower_speed <= 100) {
        insert_dist = 40000;
        insert_val = lower_speed + SPEED_INTERPOLATION;
    } else {
        insert_dist = 20000;
        insert_val = lower_speed + SPEED_INTERPOLATION;
    }
    return;
}

uint8_t SpeedLimit::SpecialScene(const std::shared_ptr<earth::shell::framework::ScenarioJudgeModel> scenario_judge_model,
                              double& start_dist, double& end_dist) {
    // copy from bool EnvironmentModel::MakeOutputMapLppInfoOdd(TopicTrait::MapLppInfoMsg& map_lpp_info)
    uint8_t special_type = 0; //1-toll, 3- tunnel
    start_dist = 100000;
    end_dist = 100000;
    std::vector<GeoFenceSegment> valid_geofence_segments;
    valid_geofence_segments = scenario_judge_model->valid_geofence_segments();

    if (valid_geofence_segments.empty()) {
        special_type = 0;
    } else {
        bool isLinkIDPresent = false;
        for (const auto& linkid : valid_geofence_segments[0].linkids) {
            if (linkid == map_position_->LinkId) {
                isLinkIDPresent = true;
                break;
            }
        }
        if (isLinkIDPresent) {
            if (valid_geofence_segments[0].geo_fence_type == 3) {
                start_dist = 0;
                end_dist = (valid_geofence_segments[0].last_endoffset - map_position_->PathOffset) / 100;
                special_type = 3;
            }else
            if (valid_geofence_segments[0].geo_fence_type == 4) {
                start_dist = 0;
                end_dist = (valid_geofence_segments[0].last_endoffset - map_position_->PathOffset) / 100;
                special_type = 1;
            }            
        } else {
            if (valid_geofence_segments[0].geo_fence_type == 3) {
                start_dist = (valid_geofence_segments[0].first_pathoffset - map_position_->PathOffset) / 100;
                end_dist = (valid_geofence_segments[0].last_endoffset - map_position_->PathOffset) / 100;
                special_type = 3;
            }else if(valid_geofence_segments[0].geo_fence_type == 4){
                start_dist = (valid_geofence_segments[0].first_pathoffset - map_position_->PathOffset) / 100;
                end_dist = (valid_geofence_segments[0].last_endoffset - map_position_->PathOffset) / 100;
                special_type = 1;
            }
        }
    }

    return special_type;
}

}  // namespace environmentmodelfunction
}  // namespace noa
