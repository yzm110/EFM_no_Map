#include "ScenarioJudgeModel.h"

namespace earth {
namespace shell {
namespace framework {
uint32_t ScenarioJudgeModel::static_last_link_id_ = 0;
bool ScenarioJudgeModel::is_has_big_curvature_ = 0;
uint32_t ScenarioJudgeModel::big_curve_position_offset_ = 0;
bool ScenarioJudgeModel::is_exit_start_dist_f_ = false;
double ScenarioJudgeModel::last_cycle_exit_end_dist_ = -255;
std::vector<uint32_t> ScenarioJudgeModel::tollgate_back_linkids_{};
std::map<uint32_t, uint8_t> ScenarioJudgeModel::main_road_nearby_ramp_lanenum_{};
std::vector<uint32_t> ScenarioJudgeModel::last_merge_back_links_id_{};
std::vector<uint8_t> ScenarioJudgeModel::last_merge_ego_back_lanes_id_{};
std::vector<uint8_t> ScenarioJudgeModel::last_merge_side_back_lanes_id_{};
std::vector<uint32_t>  ScenarioJudgeModel::tar_link_list_{};
uint32_t ScenarioJudgeModel::tar_link_max_endoffset_ = 0;
uint8_t ScenarioJudgeModel::tar_link_max_speed_ = 60;
bool ScenarioJudgeModel::Execute(
    const int cur_rp_index, const std::shared_ptr<CandidateLanesModel> candidate_lanes_model,
    const std::shared_ptr<ExtractRefLine> extract_ref_line, const message::map_position::s_Position_t& map_position,
    const TopicTrait::MapRouteListMsg& map_route_list, const TopicTrait::MapMapMsg& map_static_info,
    const std::unordered_map<uint32_t, int>& link_id_index_lane_info_map,
    const std::unordered_map<uint32_t, std::vector<int>>& link_id_index_lane_connect_map,
    const std::unordered_map<uint32_t, std::vector<int>>& to_link_id_index_lane_connect_map) {
    cur_rp_index_ = cur_rp_index;
    // link_id_vec_ = std::make_shared<const std::vector<uint32_t>>(link_id_vec);
    // candidate_lanes_vec_vec_ = std::make_shared<const std::vector<std::vector<uint8_t>>>(candidate_lanes_vec_vec);
    map_position_ = std::make_shared<const message::map_position::s_Position_t>(map_position);
    map_route_list_ = std::make_shared<const TopicTrait::MapRouteListMsg>(map_route_list);
    map_static_info_ = std::make_shared<const TopicTrait::MapMapMsg>(map_static_info);
    link_id_index_lane_info_map_ =
        std::make_shared<const std::unordered_map<uint32_t, int>>(link_id_index_lane_info_map);
    link_id_index_lane_connect_map_ =
        std::make_shared<const std::unordered_map<uint32_t, std::vector<int>>>(link_id_index_lane_connect_map);
    to_link_id_index_lane_connect_map_ =
        std::make_shared<const std::unordered_map<uint32_t, std::vector<int>>>(to_link_id_index_lane_connect_map);

    if (link_id_index_lane_connect_map_->empty() || link_id_index_lane_info_map_->empty()) {
        return false;
    }

    // init
    entry_dist_ = -255;  //
    entry_end_dist_ = -255;
    entry_start_dist_ = -255;
    exit_dist_ = -255;
    exit_start_dist_ = -255;
    exit_end_dist_ = -255;
    odd_end_dist_ = -255;
    in_odd_type_ = false;
    exit_start_link_index_ = -1;
    exit_end_link_index_ = -1;
    exit_end_speed_limit_ = 255;
    // std::cout << "EntrySceneByLaneSize 1" << std::endl;

    ego_wgs84_x_ = map_position_->Lon.Lon * 360.0 / (4294967296);
    ego_wgs84_y_ = map_position_->Lat.Lat * 360.0 / (4294967296);
    CommonTool::CoordinateTool::GetInstance()->wgs84toUTM2(ego_wgs84_y_, ego_wgs84_x_, ego_utm_x_, ego_utm_y_,
                                                           ego_wgs84_y_, ego_wgs84_x_);

    EntrySceneByLaneSize();
    EntrySceneByCandidateLanes(candidate_lanes_model);
#ifdef SCENARIOJUDGEMODEL_COUT
    std::cout << "EntrySceneByLaneSize 2" << std::endl;
#endif
    ExitSceneByLink(candidate_lanes_model, cur_rp_index_);
    MainRoadExitSceneByLink(candidate_lanes_model, cur_rp_index_);
    DirectExitScene(exit_start_dist_, exit_end_dist_);
#ifdef SCENARIOJUDGEMODEL_COUT
    std::cout << "EntrySceneByLaneSize 3" << std::endl;
    std::cout << "ScenarioJudgeModel: entry_dist_ " << entry_dist_ << std::endl;
    std::cout << "ScenarioJudgeModel: exit_dist_ " << exit_dist_ << std::endl;
#endif
    EnterGeoFence();
    ExtractValidGeoFenceSegments();
    ODDSceneJudge();

#ifdef SCENARIOJUDGEMODEL_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "exit_start_dist_: " << exit_start_dist_ << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "exit_end_dist_: " << exit_end_dist_ << std::endl;
#endif
    StoreLastLinkId();
#ifdef SCENARIOJUDGEMODEL_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "static_last_link_id_:  " << static_last_link_id_ << std::endl;
#endif
    ByPassMerge(candidate_lanes_model);
    YShapeLink();
    MainRoadWideLane(candidate_lanes_model, extract_ref_line);
    BigCurvature(candidate_lanes_model);
    RampToMainRoad(candidate_lanes_model);
    RampToMainRoadToRamp(candidate_lanes_model);
    IsVirtualLineMerge(candidate_lanes_model);
    return true;
}

bool ScenarioJudgeModel::EntrySceneByLaneSize() {
    // new logic
    // use lane_size , lanetype, lane num to judge ramp
    // when lane_size get more, then if rightest lane type is entry, turn right
    // uint32_t cur_path_id = map_position_->PathId; todo: add path logic????????

    int last_rp_index = 0;
    size_t last_lane_size = 0;
    uint32_t last_link_id = 0;
    int last_link_index = 0;
    size_t cur_lane_size = 0;
    uint32_t cur_link_id = 0;
    int cur_link_index = 0;
    bool find_entry_on_route_f = false;
    uint8_t ego_position_lane_type = 0;
    message::map_map::s_LaneInfos_t ego_position_link_LaneInfos{};
    int position_curlink_instaticmap_index = 0;
    uint32_t ego_position_path_id = map_position_->PathId;

    // 获取当前link在静态地图中的的laneinfo
    position_curlink_instaticmap_index = link_id_index_lane_info_map_->at(map_position_->LinkId);
    ego_position_link_LaneInfos = map_static_info_->LinkInfos.LinkInfos[position_curlink_instaticmap_index].LaneInfos;
    for (const auto& cur_link_LaneInfo : ego_position_link_LaneInfos.LaneInfos) {
        if (cur_link_LaneInfo.LaneNum.LaneNum == map_position_->LaneId) {
            ego_position_lane_type = cur_link_LaneInfo.LaneType.data;
            break;
        }
    }

// for (const auto& cur_link_LaneInfo : cur_link_LaneInfos.LaneInfos) {
//     std::cout << "当前link的车道信息:"
//               << "车道lanenum: " << static_cast<int>(cur_link_LaneInfo.LaneNum.LaneNum) << std::endl;
//     std::cout << "当前link的车道信息:"
//               << "车道lanetype: " << static_cast<int>(cur_link_LaneInfo.LaneType.data) << std::endl;
// }
#ifdef SCENARIOJUDGEMODEL_COUT
    std::cout << "EntrySceneByLaneSize, 111111" << std::endl;
    std::cout << "map_route_list_->LinkCount.LinkCount: " << map_route_list_->LinkCount.LinkCount << std::endl;
    std::cout << "map_route_list_->LinkIds.LinkIds.size(): " << map_route_list_->LinkIds.LinkIds.size() << std::endl;
    std::cout << "cur_rp_index_ : " << cur_rp_index_ << std::endl;
#endif
    for (int i = cur_rp_index_; i < (map_route_list_->LinkCount.LinkCount - cur_rp_index_); i++) {
        // std::cout << "i : " << i << std::endl;
        if (i == cur_rp_index_ && cur_rp_index_ > 0) {
            last_rp_index = cur_rp_index_ - 1;
#ifdef SCENARIOJUDGEMODEL_COUT
            std::cout << "first last_rp_index : " << last_rp_index << std::endl;
#endif
            last_link_id = map_route_list_->LinkIds.LinkIds[last_rp_index].LinkId;
            // last_link_index = link_id_index_lane_info_map_->at(last_link_id);
            if (link_id_index_lane_info_map_->find(last_link_id) != link_id_index_lane_info_map_->end()) {
                last_link_index = link_id_index_lane_info_map_->at(last_link_id);
                last_lane_size = map_static_info_->LinkInfos.LinkInfos[last_link_index].LaneInfos.LaneInfos.size();
            } else {
#ifdef SCENARIOJUDGEMODEL_COUT
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "first last_link_index abnormal" << std::endl;
#endif
                last_lane_size = 0;
            }
            // last_lane_size = map_static_info_->LinkInfos.LinkInfos[last_link_index].LaneInfos.LaneInfos.size();
        } else if (i == cur_rp_index_ && cur_rp_index_ == 0) {
            last_link_id = static_last_link_id_;
            if (link_id_index_lane_info_map_->find(last_link_id) != link_id_index_lane_info_map_->end()) {
                last_link_index = link_id_index_lane_info_map_->at(last_link_id);
                last_lane_size = map_static_info_->LinkInfos.LinkInfos[last_link_index].LaneInfos.LaneInfos.size();
            } else {
#ifdef SCENARIOJUDGEMODEL_COUT
                std::cout << __FILE__ << "," << __LINE__ << ","
                          << "first last_link_index abnormal22" << std::endl;
#endif
                last_lane_size = 0;
            }
        }
        cur_link_id = map_route_list_->LinkIds.LinkIds[i].LinkId;
#ifdef SCENARIOJUDGEMODEL_COUT
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "cur_link_id : " << cur_link_id << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "last_link_id : " << last_link_id << std::endl;
#endif
        //  cur_link_index = link_id_index_lane_info_map_->at(cur_link_id);
        if (link_id_index_lane_info_map_->find(cur_link_id) != link_id_index_lane_info_map_->end()) {
            cur_link_index = link_id_index_lane_info_map_->at(cur_link_id);
        } else {
#ifdef SCENARIOJUDGEMODEL_COUT
            std::cout << "search end" << std::endl;
#endif
            break;
        }
#ifdef SCENARIOJUDGEMODEL_COUT
        // std::cout << "cur_link_index : " << cur_link_index << std::endl;
#endif
        cur_lane_size = map_static_info_->LinkInfos.LinkInfos[cur_link_index].LaneInfos.LaneInfos.size();
#ifdef SCENARIOJUDGEMODEL_COUT
        // std::cout << "cur_lane_size : " << cur_lane_size << std::endl;
        // std::cout << "last_lane_size : " << last_lane_size << std::endl;
#endif
        //   lane size get more
        if ((last_lane_size != 0 && cur_lane_size > last_lane_size && cur_lane_size >= 1) ||
            ego_position_lane_type == 1) {
            // judge direction of lane nmber change
            bool is_right_get_more = false;
            std::vector<int> last_link_connect_index{};
            if (link_id_index_lane_connect_map_->find(last_link_id) != link_id_index_lane_connect_map_->end()) {
                last_link_connect_index = link_id_index_lane_connect_map_->at(last_link_id);
            } else {
#ifdef SCENARIOJUDGEMODEL_COUT
                std::cout << "judge direction of lane nmber change failure!!!!" << std::endl;
#endif
                break;
            }
            for (auto index : last_link_connect_index) {
                if (map_static_info_->LaneConnectivitys.PairConnectivity[index].NewPath.NewPath ==
                    ego_position_path_id) {
                    if (map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum <
                        map_static_info_->LaneConnectivitys.PairConnectivity[index].NewLaneNum.NewLaneNum) {
                        // right lane get more
                        is_right_get_more = true;
                        break;
                    }
                }
            }
#ifdef SCENARIOJUDGEMODEL_COUT
            std::cout << "is_right_get_more: " << is_right_get_more << std::endl;
#endif
            uint8_t righest_available_lane_num = 1;
            if (!is_right_get_more) {
                if (!GetRightestAvailableLaneNum(cur_link_id, righest_available_lane_num)) {
#ifdef SCENARIOJUDGEMODEL_COUT
                    std::cout << "could not find righest_available_lane_num, EntrySceneByLaneSize" << std::endl;
#endif
                    righest_available_lane_num = 1;
                }
#ifdef SCENARIOJUDGEMODEL_COUT
                std::cout << "righest_available_lane_num: " << static_cast<int>(righest_available_lane_num)
                          << std::endl;
#endif
                for (int j = 0; j < cur_lane_size; j++) {
                    // std::cout << "j : " << j << std::endl;
                    //  rightest lane
                    if (map_static_info_->LinkInfos.LinkInfos[cur_link_index].LaneInfos.LaneInfos[j].LaneNum.LaneNum ==
                            righest_available_lane_num &&
                        (ego_position_lane_type == 4 || ego_position_lane_type == 6 || ego_position_lane_type == 1 ||
                         ego_position_lane_type == 7)) {
                        if (map_static_info_->LinkInfos.LinkInfos[cur_link_index]
                                    .LaneInfos.LaneInfos[j]
                                    .LaneType.data == 1 ||
                            map_static_info_->LinkInfos.LinkInfos[cur_link_index]
                                    .LaneInfos.LaneInfos[j]
                                    .LaneType.data == 17) {
                            // ENTRY_
                            find_entry_on_route_f = true;
                            break;
                        }
                    }
                }
            }

            if (find_entry_on_route_f) {
                entry_dist_ = (map_static_info_->LinkInfos.LinkInfos[cur_link_index].PathOffset.PathOffset -
                               map_position_->PathOffset) /
                              100;
                entry_start_dist_ =
                    (static_cast<double>(map_static_info_->LinkInfos.LinkInfos[cur_link_index].PathOffset.PathOffset) -
                     static_cast<double>(map_position_->PathOffset)) /
                    100.0;
                if (entry_start_dist_ > 300) {
                    entry_start_dist_ = -255;
                }
#ifdef SCENARIOJUDGEMODEL_COUT
                std::cout << "EFMdebugforExitSceneByLink-"
                          << "entry_link_EndOffset: "
                          << map_static_info_->LinkInfos.LinkInfos[cur_link_index].EndOffset.EndOffset << std::endl;
                std::cout << "EFMdebugforExitSceneByLink-"
                          << "find_entry_on_route_f-" << find_entry_on_route_f << std::endl;
                std::cout << "EFMdebugforExitSceneByLink-"
                          << "entry_dist_-" << entry_dist_ << std::endl;
                std::cout << "EFMdebugforExitSceneByLink-"
                          << "entry_link_id-" << cur_link_id << std::endl;
                std::cout << "EFMdebugforExitSceneByLink-"
                          << "entry_link_righest_available_lane_num-" << static_cast<int>(righest_available_lane_num)
                          << std::endl;
                std::cout << "EFMdebugforExitSceneByLink-"
                          << "map_position_link_id-" << map_position_->LinkId << std::endl;
                std::cout << "EFMdebugforExitSceneByLink-"
                          << "map_position_lane_id-" << static_cast<int>(map_position_->LaneId) << std::endl;
                std::cout << "EFMdebugforExitSceneByLink-"
                          << "map_position_path_id-" << map_position_->PathId << std::endl;
                std::cout << "EFMdebugforExitSceneByLink-"
                          << "map_position_path_offset-" << map_position_->PathOffset << std::endl;
                // std::cout << "EFMdebugforExitSceneByLink-"
                //  << "cur_lane_type-" << static_cast<int>(cur_lane_type) << std::endl;
                std::cout << std::cout.precision(12) << "EFMdebugforExitSceneByLink-"
                          << "map_position_lon-" << map_position_->Lon.Lon * 360.0 / (4294967296) << std::endl;
                std::cout << std::cout.precision(12) << "EFMdebugforExitSceneByLink-"
                          << "map_position_lat-" << map_position_->Lat.Lat * 360.0 / (4294967296) << std::endl;
#endif
                break;
            }
        }
        last_lane_size = cur_lane_size;
        last_link_id = cur_link_id;
    }
#ifdef SCENARIOJUDGEMODEL_COUT
    std::cout << "EntrySceneByLaneSize, 2222222222222" << std::endl;
#endif
    return true;
}

/* bool ScenarioJudgeModel::ExitSceneByLaneSize() {
    // new logic
    // use lane_size , lanetype, lane num to judge ramp
    // when lane_size get more, then if rightest lane type is exist or on ramp, turn left
    // uint32_t cur_path_id = map_position_->PathId; todo: add path logic????????

    int last_rp_index = -1;
    size_t last_lane_size = 0;
    uint32_t last_link_id = 0;
    int last_link_index = -1;
    size_t cur_lane_size = 0;
    uint32_t cur_link_id = 0;
    int cur_link_index = -1;
    bool find_exit_on_route_f = false;

    for (int i = cur_rp_index_; i < (map_route_list_->LinkCount.LinkCount - cur_rp_index_); i++) {
        if (i == cur_rp_index_ && cur_rp_index_ > 0) {
            last_rp_index = cur_rp_index_ - 1;
            last_link_id = map_route_list_->LinkIds.LinkIds[last_rp_index].LinkId;
            last_link_index = link_id_index_lane_info_map_->at(last_link_id);
            last_lane_size = map_static_info_->LinkInfos.LinkInfos[last_link_index].LaneInfos.LaneInfos.size();
        }
        cur_link_id = map_route_list_->LinkIds.LinkIds[i].LinkId;
        cur_link_index = link_id_index_lane_info_map_->at(cur_link_id);
        cur_lane_size = map_static_info_->LinkInfos.LinkInfos[cur_link_index].LaneInfos.LaneInfos.size();

        // lane size get more
        if (last_lane_size != 0 && cur_lane_size > last_lane_size && cur_lane_size >= 1) {
            for (int j = 0; j < cur_lane_size; j++) {
                // rightest lane
                if (map_static_info_->LinkInfos.LinkInfos[cur_link_index].LaneInfos.LaneInfos[j].LaneNum.LaneNum == 1) {
                    if (map_static_info_->LinkInfos.LinkInfos[cur_link_index].LaneInfos.LaneInfos[j].LaneType.data ==
                        2) {
                        // exit
                        find_exit_on_route_f = true;
                        break;
                    }
                }
            }

            if (find_exit_on_route_f) {
                exit_dist_ = map_static_info_->LinkInfos.LinkInfos[cur_link_index].PathOffset.PathOffset -
                             map_position_->PathOffset;
                break;
            }
        }
    }
    return true;
} */

bool ScenarioJudgeModel::ExitSceneByLink(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model,
                                         int cur_rp_index) {
    // when a link get two connected link, means split, then rightest lane type is exit
    // step 1, get link connect on route
    exit_start_road_class_ = 0;
    bool find_exit_on_route_f = false;
#ifdef SCENARIOJUDGEMODEL_COUT
    std::cout << "initial find_exit_on_route_f: " << find_exit_on_route_f << std::endl;
#endif
    uint32_t ego_path_id = map_position_->PathId;
    // for debug
    // double map_static_link_PathOffset = 0.0;
    double map_position_PathOffset = 0.0;
    std::vector<int> cur_lane_connectivity_indexs;
    uint32_t exit_start_offset = 0;
    uint32_t exit_end_offset = 0;
    int exit_link_rp_index = -1;
    uint32_t exit_link_id = 0;
#ifdef SCENARIOJUDGEMODEL_COUT
    std::cout << "ExitSceneByLink: #################" << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ", "
              << "map_route_list_->LinkIds.LinkIds.size(): " << map_route_list_->LinkIds.LinkIds.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ", "
              << "cur_rp_index_: " << cur_rp_index_ << std::endl;
#endif
    // for (int i = 0; i < (candidate_lanes_model->link_id_vec().size() - 1); i++) {
    //    uint32_t cur_search_link = candidate_lanes_model->link_id_vec()[i];
    //    uint32_t next_link_id = candidate_lanes_model->link_id_vec()[i + 1];
    for (int i = cur_rp_index_; i < (map_route_list_->LinkIds.LinkIds.size() - 1); i++) {
        uint32_t cur_search_link = map_route_list_->LinkIds.LinkIds[i].LinkId;
        uint32_t next_link_id = map_route_list_->LinkIds.LinkIds[i + 1].LinkId;
        // std::vector<int> cur_lane_connectivity_indexs = link_id_index_lane_connect_map_->at(cur_search_link);
        if (link_id_index_lane_connect_map_->find(cur_search_link) != link_id_index_lane_connect_map_->end()) {
            cur_lane_connectivity_indexs = link_id_index_lane_connect_map_->at(cur_search_link);
        } else {
#ifdef SCENARIOJUDGEMODEL_COUT
            std::cout << "ExitSceneByLink 11111111" << std::endl;
#endif
            break;
        }
        // if ToLinkId more than 1, means split
        int to_link_num = 0;
        int to_link_id = 0;
        // std::cout << i << ", cur_search_link: " << cur_search_link << std::endl;
        for (int cur_connect_index : cur_lane_connectivity_indexs) {
            if (cur_connect_index < map_static_info_->LaneConnectivitys.PairConnectivity.size()) {
                if (map_static_info_->LaneConnectivitys.PairConnectivity[cur_connect_index].ToLinkId.ToLink !=
                    to_link_id) {
                    to_link_num++;
                    to_link_id =
                        map_static_info_->LaneConnectivitys.PairConnectivity[cur_connect_index].ToLinkId.ToLink;
                    // std::cout << "EFM的debug信号-" << "to_link_id-" << to_link_id<<std::endl;
                    if (to_link_num > 1) {
                        // exist link split, get rightest lane type to judge ramp
                        // maybe next lane???
                        //  std::cout << "cur_search_link more to_link: " << cur_search_link << std::endl;
                        // int cur_link_index = link_id_index_lane_info_map_->at(cur_search_link);
                        int cur_link_index = 0;
                        if (link_id_index_lane_info_map_->find(cur_search_link) !=
                            link_id_index_lane_info_map_->end()) {
                            cur_link_index = link_id_index_lane_info_map_->at(cur_search_link);
                        } else {
                            // std::cout << "ExitSceneByLink 22222222" << std::endl;
                            break;
                        }
                        uint8_t righest_available_lane_num = 1;
                        if (!GetRightestAvailableLaneNum(cur_search_link, righest_available_lane_num)) {
                            // std::cout << "could not find righest_available_lane_num, ExitSceneByLink" << std::endl;
                            righest_available_lane_num = 1;
                        }
                        size_t cur_lane_size =
                            map_static_info_->LinkInfos.LinkInfos[cur_link_index].LaneInfos.LaneInfos.size();
#ifdef SCENARIOJUDGEMODEL_COUT
// std::cout << "cur_lane_size: " << cur_lane_size << std::endl;
// std::cout << "righest_available_lane_num: " << static_cast<int>(righest_available_lane_num)
//<< std::endl;
#endif
                        for (int j = 0; j < cur_lane_size; j++) {
#ifdef SCENARIOJUDGEMODEL_COUT
                            std::cout << "lane_num: "
                                      << static_cast<int>(map_static_info_->LinkInfos.LinkInfos[cur_link_index]
                                                              .LaneInfos.LaneInfos[j]
                                                              .LaneNum.LaneNum)
                                      << std::endl;
                            std::cout << "LaneType: "
                                      << static_cast<int>(map_static_info_->LinkInfos.LinkInfos[cur_link_index]
                                                              .LaneInfos.LaneInfos[j]
                                                              .LaneType.data)
                                      << std::endl;
#endif
                            uint8_t lane_num_temp = map_static_info_->LinkInfos.LinkInfos[cur_link_index]
                                                        .LaneInfos.LaneInfos[j]
                                                        .LaneNum.LaneNum;
                            if (lane_num_temp >= righest_available_lane_num) {
                                // if (map_static_info_->LinkInfos.LinkInfos[cur_link_index]
                                //             .LaneInfos.LaneInfos[j]
                                //             .LaneType.data == 2 ||
                                //     map_static_info_->LinkInfos.LinkInfos[cur_link_index]
                                //             .LaneInfos.LaneInfos[j]
                                //             .LaneType.data == 17) {
                                // //for all exit lane, and type 17 lane
                                // then to search  lane's connect info,
                                std::vector<int> cur_lane_connect_index_vec;
                                if (link_id_index_lane_connect_map_->find(cur_search_link) !=
                                    link_id_index_lane_connect_map_->end()) {
                                    cur_lane_connect_index_vec = link_id_index_lane_connect_map_->at(cur_search_link);
                                } else {
                                    // std::cout << "ExitSceneByLink 333333333" << std::endl;
                                    break;
                                }
                                // std::cout << "get lane type 2 or 17" << std::endl;
                                for (int index : cur_lane_connect_index_vec) {
                                    // for all connect. pick rightest lane's connectivitys
                                    if (map_static_info_->LaneConnectivitys.PairConnectivity[index]
                                            .InitLaneNum.InitLaneNum == lane_num_temp) {
                                        int to_link_id_2 =
                                            map_static_info_->LaneConnectivitys.PairConnectivity[index].ToLinkId.ToLink;
                                        // std::cout << "to_link_id_2: " << to_link_id_2 << std::endl;
                                        uint8_t to_lane_num_2 =
                                            map_static_info_->LaneConnectivitys.PairConnectivity[index]
                                                .NewLaneNum.NewLaneNum;

                                        if (to_link_id_2 == next_link_id) {
                                            int to_link_index = 0;
                                            if (link_id_index_lane_info_map_->find(to_link_id_2) !=
                                                link_id_index_lane_info_map_->end()) {
                                                to_link_index = link_id_index_lane_info_map_->at(to_link_id_2);
                                            } else {
                                                // std::cout << "ExitSceneByLink 22222222" << std::endl;
                                                break;
                                            }
                                            for (auto lane : map_static_info_->LinkInfos.LinkInfos[to_link_index]
                                                                 .LaneInfos.LaneInfos) {
                                                if (lane.LaneNum.LaneNum == to_lane_num_2 &&
                                                    (lane.LaneType.data == 5 || lane.LaneType.data == 2 ||
                                                     lane.LaneType.data == 4 || lane.LaneType.data == 6)) {
                                                    find_exit_on_route_f = true;
                                                    break;
                                                }
                                            }
                                            if (find_exit_on_route_f == true) {
                                                break;
                                            }
                                        }
                                        // if there's one to_link's path id == ego path id, means to link is on route,
                                        // exist exit on route
                                        // int link_index = link_id_index_lane_info_map_->at(to_link_id_2);
                                        // if (map_static_info_->LinkInfos.LinkInfos[link_index].PathId.PathId ==
                                        //     ego_path_id) {
                                        //     find_exit_on_route_f = true;
                                        //     break;
                                        // }
                                    }
                                }
                                if (find_exit_on_route_f) {
                                    break;
                                }
                                // }
                            }
                        }
                        if (find_exit_on_route_f) {
                            exit_dist_ = (map_static_info_->LinkInfos.LinkInfos[cur_link_index].EndOffset.EndOffset -
                                          map_position_->PathOffset) /
                                         100.0;
                            exit_end_offset = map_static_info_->LinkInfos.LinkInfos[cur_link_index].EndOffset.EndOffset;
                            exit_link_rp_index = i - cur_rp_index_;
                            exit_link_id = cur_search_link;
                            exit_end_link_index_ = exit_link_rp_index;

                            // GetLinkMinSpeedLimit(next_link_id, exit_end_speed_limit_);
#ifdef SCENARIOJUDGEMODEL_COUT
                            std::cout << __FILE__ << "," << __LINE__ << ","
                                      << "EFMdebugforExitSceneByLink-"
                                      << "exit_link_EndOffset: " << exit_end_offset << std::endl;
                            std::cout << __FILE__ << "," << __LINE__ << ","
                                      << "exit_link_rp_index: " << exit_link_rp_index << std::endl;
                            std::cout << "EFMdebugforExitSceneByLink-"
                                      << "exit_link_PathOffset.PathOffset: "
                                      << map_static_info_->LinkInfos.LinkInfos[cur_link_index].PathOffset.PathOffset
                                      << std::endl;
                            std::cout << "EFMdebugforExitSceneByLink-"
                                      << "find_exit_on_route_f-" << find_exit_on_route_f << std::endl;
                            std::cout << "EFMdebugforExitSceneByLink-"
                                      << "exit_dist_-" << exit_dist_ << std::endl;
                            std::cout << "EFMdebugforExitSceneByLink-"
                                      << "exit_link_id-" << cur_search_link << std::endl;
                            std::cout << "EFMdebugforExitSceneByLink-"
                                      << "exit_link_righest_available_lane_num-"
                                      << static_cast<int>(righest_available_lane_num) << std::endl;
                            std::cout << "EFMdebugforExitSceneByLink-"
                                      << "map_position_link_id-" << map_position_->LinkId << std::endl;
                            std::cout << "EFMdebugforExitSceneByLink-"
                                      << "map_position_path_id-" << map_position_->PathId << std::endl;
                            std::cout << "EFMdebugforExitSceneByLink-"
                                      << "map_position_lane_id-" << static_cast<int>(map_position_->LaneId)
                                      << std::endl;
                            std::cout << "EFMdebugforExitSceneByLink-"
                                      << "map_position_path_offset-" << map_position_->PathOffset << std::endl;
                            std::cout << std::cout.precision(12) << "EFMdebugforExitSceneByLink-"
                                      << "map_position_lon-" << map_position_->Lon.Lon * 360.0 / (4294967296)
                                      << std::endl;
                            std::cout << std::cout.precision(12) << "EFMdebugforExitSceneByLink-"
                                      << "map_position_lat-" << map_position_->Lat.Lat * 360.0 / (4294967296)
                                      << std::endl;
// map_static_link_PathOffset =
//     map_static_info_->LinkInfos.LinkInfos[cur_link_index].PathOffset.PathOffset;
// map_position_PathOffset = map_position_->PathOffset;
// std::cout << "EFMdebugforExitSceneByLink-"
//           << "map_position_PathId-" << map_position_->PathId << std::endl;
// std::cout << "EFMdebugforExitSceneByLink-"
//           << "map_position_LinkId-" << map_position_->LinkId << std::endl;
#endif
                            break;
                        }
                    }
                }
            }
        }

        // std::cout << "EFM的debug信号-" << "to_link_num-" << to_link_num<<std::endl;

        if (find_exit_on_route_f) {
            //            history_find_entry_on_route_f_ =true;
            break;
        }
    }
    // cal exit start dist, end dist
    if (!find_exit_on_route_f) {
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "exit none" << std::endl;
        return true;
    }

    std::vector<uint32_t> link_id_vec = candidate_lanes_model->link_id_vec();
    std::map<uint32_t, SplitNode> candidate_lanes_split_nodes = candidate_lanes_model->candidate_lanes_split_nodes();
    std::vector<bool> is_contain_split_vec = candidate_lanes_model->is_contain_split_vec();
    std::vector<std::vector<uint8_t>> all_lanes_vec_vec = candidate_lanes_model->all_lanes_vec_vec();
#ifdef SCENARIOJUDGEMODEL_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "exit_link_rp_index  " << exit_link_rp_index << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "link_id_vec.size()  " << link_id_vec.size() << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "is_contain_split_vec.size() " << is_contain_split_vec.size() << std::endl;
#endif
    bool to_ramp = false;
    bool to_main_road = false;
    // bool get_exit_start_point = false;
    for (int j = exit_link_rp_index - 1; j >= 0 && j < link_id_vec.size() && j < is_contain_split_vec.size(); j--) {
        uint32_t link_id = link_id_vec[j];
        //最右侧的可行驶车道num
        uint8_t rightest_drv_lane_num = 255;
        message::map_map::s_LinkInfo_t cur_search_link_infos{};
        if(false == GetLinkInfos(link_id, cur_search_link_infos)){
            break;
        }
        for(auto lane:cur_search_link_infos.LaneInfos.LaneInfos){
            if(lane.LaneType.data !=3 && lane.LaneType.data !=9 && lane.LaneType.data !=17){
                if(rightest_drv_lane_num> lane.LaneNum.LaneNum){
                    rightest_drv_lane_num = lane.LaneNum.LaneNum;
                }
            }
        }
        if(rightest_drv_lane_num == 255){
            break;
        }
        if (is_contain_split_vec[j] == true) {          
            if (candidate_lanes_split_nodes.find(link_id) != candidate_lanes_split_nodes.end()) {
                uint8_t origin_lane_num = candidate_lanes_split_nodes[link_id].origin_node.lane_num; 
                for (auto split_node : candidate_lanes_split_nodes[link_id].split_nodes) {
                    // j+1 is split, j is origin
                    auto iter = std::find_if(all_lanes_vec_vec.begin(), all_lanes_vec_vec.end(),
                                             [&](const std::vector<uint8_t>& it) {
                                                 if (it.size() > (j + 1)) {
                                                     if (it[j] == origin_lane_num && it[j + 1] == split_node.lane_num) {
                                                         return true;
                                                     }
                                                 }
                                                 return false;
                                             });
                    if (iter != all_lanes_vec_vec.end()) {
                        if ((*iter).size() < exit_link_rp_index + 1) {
                            // not this lane
                        } else {
                            uint8_t end_lane_num = (*iter)[exit_link_rp_index];
                            int link_index = link_id_index_lane_info_map_->at(exit_link_id);
                            for (auto lane : map_static_info_->LinkInfos.LinkInfos[link_index].LaneInfos.LaneInfos) {
                                if (lane.LaneNum.LaneNum == end_lane_num) {
                                    if (lane.LaneType.data == 2 || lane.LaneType.data == 8) {
                                        // if (*iter)[exit_link_rp_index +1] exist,means lane is on route
                                        if ((*iter).size() > exit_link_rp_index + 1 && rightest_drv_lane_num == origin_lane_num) {
                                            to_ramp = true;
                                        }
                                    } else if (lane.LaneType.data == 0) {
                                        // continue to check whether the next is on route, main road may not in route on
                                        // road split
                                        if ((*iter).size() == exit_link_rp_index + 1) {
                                            to_main_road = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if (to_ramp && to_main_road) {
            int link_index = link_id_index_lane_info_map_->at(link_id);
            exit_start_offset = map_static_info_->LinkInfos.LinkInfos[link_index].EndOffset.EndOffset;
            exit_start_link_index_ = j;
            exit_start_road_class_ = 0;
            GetRoadClass(map_static_info_,link_id,exit_start_road_class_);
            break;
        }
        if (exit_start_offset != 0) {
            break;
        }
    }

#ifdef SCENARIOJUDGEMODEL_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "to_ramp: " << (int)to_ramp << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "to_main_road: " << (int)to_main_road << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_position_->PathOffset: " << map_position_->PathOffset << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "exit_end_offset: " << exit_end_offset << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "exit_start_offset: " << exit_start_offset << std::endl;
#endif
    //针对没有连接关系的，再判断下,有没有 多个link连接一个link的
    int connect_lane_size = candidate_lanes_model->link_id_vec().size();
    int route_list_size = map_route_list_->LinkIds.LinkIds.size() - cur_rp_index;
    //std::cout << __FILE__ << "," << __LINE__ << "," << "connect_lane_size: " << connect_lane_size << std::endl;
    //std::cout << __FILE__ << "," << __LINE__ << "," << "route_list_size: " << route_list_size << std::endl;
    bool no_connect_lane_ahead = false;
    if (connect_lane_size > 0 && route_list_size > connect_lane_size) {
        if (link_id_index_lane_connect_map_->find(candidate_lanes_model->link_id_vec().back()) !=
            link_id_index_lane_connect_map_->end()) {
            int index_temp = std::max(0, connect_lane_size + cur_rp_index);
            //std::cout << __FILE__ << "," << __LINE__ << "," << "index_temp: " << index_temp << std::endl;
            //std::cout << __FILE__ << "," << __LINE__ << ","
            //          << "candidate_lanes_model->link_id_vec().back(): " << candidate_lanes_model->link_id_vec().back()
            //          << std::endl;
            if (map_route_list_->LinkIds.LinkIds.size() > index_temp) {
                if (link_id_index_lane_info_map_->find(map_route_list_->LinkIds.LinkIds[index_temp].LinkId) !=
                    link_id_index_lane_info_map_->end()) {
                    no_connect_lane_ahead = true;
                }
            }
        }
    }
    exit_start_lane_type_no_connect_ = 0;
    exit_start_speed_limit_no_connect_ = 0;
    if (exit_start_offset == 0 && no_connect_lane_ahead) {
        for (int j = exit_link_rp_index - 1; j >= 0 && j < link_id_vec.size(); j--) {
            uint32_t link_id = link_id_vec[j];
            if (to_link_id_index_lane_connect_map_->find(link_id) != to_link_id_index_lane_connect_map_->end()) {
                uint32_t temp_link_id = 0;
                for (auto idx : to_link_id_index_lane_connect_map_->at(link_id)) {
                    uint32_t temp_back_link_id =
                        map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId;
                    if (temp_link_id != 0 && temp_back_link_id != temp_link_id) {
                        if (link_id_index_lane_info_map_->find(link_id) != link_id_index_lane_info_map_->end()) {
                            int link_index = link_id_index_lane_info_map_->at(link_id);
                            exit_start_offset = map_static_info_->LinkInfos.LinkInfos[link_index].PathOffset.PathOffset;
                            //取最右侧非应急临停车道的类型，以及限速
                            int while_counter = std::min(static_cast<int>(map_static_info_->LinkInfos.LinkInfos[link_index].LaneInfos.LaneInfos.size()),10);
                            uint8_t lane_id =0;
                            message::map_map::s_LaneInfo_t lane_info{};
                            message::map_map::s_LinkInfo_t link_infos = map_static_info_->LinkInfos.LinkInfos[link_index];
                            message::map_map::s_LaneSpeedLimit_t lane_speed_limit{};
                            while(while_counter>0){//从最右侧开始   
                                lane_id++;                    
                                if(efm::MapCommonTool::GetInstance()->GetLaneInfo(link_infos, lane_id, lane_info)){
                                    if(lane_info.LaneType.data != 3 && lane_info.LaneType.data != 17 && lane_info.LaneType.data != 9){
                                        exit_start_lane_type_no_connect_ = lane_info.LaneType.data;
                                        if(efm::MapCommonTool::GetInstance()->GetLaneSpeed(map_static_info_, link_id,lane_id, lane_speed_limit)){
                                            exit_start_speed_limit_no_connect_ = lane_speed_limit.ValueH.ValueH;
                                            if(exit_start_speed_limit_no_connect_ == 0){
                                                exit_start_speed_limit_no_connect_ = 80;
                                            }
                                            if(exit_start_lane_type_no_connect_!=0){
                                                exit_start_speed_limit_no_connect_ = std::min(static_cast<uint8_t>(80),exit_start_speed_limit_no_connect_);
                                            }
                                            break;                                            
                                        }

                                    }
                                }  
                                while_counter--;                             
                            }
                            break;
                        }
                    }
                    temp_link_id = temp_back_link_id;
                }
            }
            if (exit_start_offset != 0) {
                break;
            }
        }
    }

    if (map_position_->PathOffset > exit_end_offset) {
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "exit_end_offset < position offset: " << std::endl;
        exit_end_dist_ = -255;
    } else {
        exit_end_dist_ =
            (static_cast<double>(exit_end_offset) - static_cast<double>(map_position_->PathOffset)) / 100.0;
    }

    if ((map_position_->PathOffset > exit_start_offset) || (exit_start_offset == 0 && exit_end_offset != 0)) {
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "exit_start_offset < position offset: " << std::endl;
        exit_start_dist_ = -255;
    } else {
        exit_start_dist_ =
            (static_cast<double>(exit_start_offset) - static_cast<double>(map_position_->PathOffset)) / 100.0;
    }

    // if (exit_start_dist_ > -255 && exit_end_dist_ > -255 && (exit_end_dist_ - exit_start_dist_) > 300) {
    //     exit_start_dist_ = exit_end_dist_ - 300;
    // }
    return true;
}

bool ScenarioJudgeModel::GetRightestAvailableLaneNum(uint32_t search_link_id, uint8_t& righest_lane_num) {
    // std::cout << "GetRightestLaneNum start" << std::endl;
    int cur_link_index = link_id_index_lane_info_map_->at(search_link_id);
    size_t cur_lane_size = map_static_info_->LinkInfos.LinkInfos[cur_link_index].LaneInfos.LaneInfos.size();
    int while_counter = 0;
    bool get_rightest_lane_f = false;
    righest_lane_num = 1;
    while (!get_rightest_lane_f) {
        while_counter++;
        for (int j = 0; j < cur_lane_size; j++) {
            // get righest lane, and not emergency lane type
            if (map_static_info_->LinkInfos.LinkInfos[cur_link_index].LaneInfos.LaneInfos[j].LaneNum.LaneNum ==
                    righest_lane_num &&
                map_static_info_->LinkInfos.LinkInfos[cur_link_index].LaneInfos.LaneInfos[j].LaneType.data != 3) {
                get_rightest_lane_f = true;
            }
            if (map_static_info_->LinkInfos.LinkInfos[cur_link_index].LaneInfos.LaneInfos[j].LaneNum.LaneNum ==
                    righest_lane_num &&
                map_static_info_->LinkInfos.LinkInfos[cur_link_index].LaneInfos.LaneInfos[j].LaneType.data == 3) {
                righest_lane_num++;
                get_rightest_lane_f = false;
            }
        }
        if (while_counter > cur_lane_size) {
            return false;
        }
    }
    // std::cout << "while_counter: " << while_counter << std::endl;
    return true;
}

bool ScenarioJudgeModel::StoreLastLinkId() {
    if (cur_rp_index_ > 0) {
        if (map_route_list_->LinkIds.LinkIds.size() > 0) {
            static_last_link_id_ = map_route_list_->LinkIds.LinkIds[cur_rp_index_ - 1].LinkId;
        }
    }

    return true;
}

void ScenarioJudgeModel::EntrySceneByCandidateLanes(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model) {
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "EntrySceneByCandidateLanes start  " << std::endl;
    std::vector<uint32_t> link_id_vec = candidate_lanes_model->link_id_vec();
    std::vector<std::vector<uint8_t>> all_lanes_vec_vec = candidate_lanes_model->all_lanes_vec_vec();
    uint8_t min_lane_num = 255;
    uint8_t max_lane_num = 0;
    for (int i = 0; i < all_lanes_vec_vec.size(); i++) {
        if (!all_lanes_vec_vec[i].empty()) {
            if (min_lane_num > all_lanes_vec_vec[i].front()) {
                min_lane_num = all_lanes_vec_vec[i].front();
            }
            if (max_lane_num < all_lanes_vec_vec[i].front()) {
                max_lane_num = all_lanes_vec_vec[i].front();
            }
        }
    }
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "min_lane_num  " << static_cast<int>(min_lane_num) << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "max_lane_num  " << static_cast<int>(max_lane_num) << std::endl;
    for (int lane_num = min_lane_num; lane_num < max_lane_num; lane_num++) {
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "lane_num  " << lane_num << std::endl;
        auto iter1 =
            std::find_if(all_lanes_vec_vec.begin(), all_lanes_vec_vec.end(), [&](const std::vector<uint8_t>& it) {
                if (!it.empty()) {
                    return (it.front() == lane_num);
                } else {
                    return false;
                }
            });
        auto iter2 =
            std::find_if(all_lanes_vec_vec.begin(), all_lanes_vec_vec.end(), [&](const std::vector<uint8_t>& it) {
                if (!it.empty()) {
                    return (it.front() == (lane_num + 1));
                } else {
                    return false;
                }
            });
        // int index1 = std::distance(all_lanes_vec_vec.begin(), iter1);
        // int index2 = std::distance(all_lanes_vec_vec.begin(), iter2);
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "index1  " << index1 << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "index2  " << index2 << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "(*iter1).size()  " << (*iter1).size() << std::endl;
        // std::cout << __FILE__ << "," << __LINE__ << ","
        //           << "(*iter2).size()  " << (*iter2).size() << std::endl;
        if (iter1 != all_lanes_vec_vec.end() && iter2 != all_lanes_vec_vec.end()) {
            for (int i = 1; i < (*iter1).size() && i < (*iter2).size() && i < link_id_vec.size(); i++) {
                if ((*iter1)[i] == (*iter2)[i]) {
                    // merge
                    uint32_t link_id = link_id_vec[i - 1];
                    message::map_map::s_LinkInfo_t link_infos;
                    if (link_id_index_lane_info_map_->find(link_id) != link_id_index_lane_info_map_->end()) {
                        int link_index = link_id_index_lane_info_map_->at(link_id);
                        link_infos = map_static_info_->LinkInfos.LinkInfos[link_index];
                    } else {
                        break;
                    }
                    bool one_on_main_road = false;
                    bool one_on_ramp = false;
                    for (auto lane : link_infos.LaneInfos.LaneInfos) {
                        if (lane.LaneNum.LaneNum == (*iter1)[i - 1] || lane.LaneNum.LaneNum == (*iter2)[i - 1]) {
                            if (lane.LaneType.data == 1 || lane.LaneType.data == 2 || lane.LaneType.data == 4 ||
                                lane.LaneType.data == 5 || lane.LaneType.data == 6 || lane.LaneType.data == 7 ||
                                lane.LaneType.data == 8) {
                                one_on_ramp = true;
                            } else if (lane.LaneType.data == 0) {
                                one_on_main_road = true;
                            }
                        }
                    }
                    if (one_on_main_road && one_on_ramp) {
                        entry_end_dist_ = (static_cast<double>(link_infos.EndOffset.EndOffset) -
                                           static_cast<double>(map_position_->PathOffset)) /
                                          100.0;
                        break;
                    }
                }
            }
        }
    }
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "EntrySceneByCandidateLanes end  " << std::endl;
}
bool ScenarioJudgeModel::ODDSceneJudge() {
    uint32_t routelist_end_dis = 5000;
    uint32_t routelist_start_dis = 5000;
    uint32_t highway_end_dis = 5000;
    uint32_t highway_start_dis = 5000;
    uint32_t tollgate_end_dis = 5000;
    uint32_t tollgate_start_dis = 5000;
    std::vector<uint32_t> tollgate_linkids{};
    uint32_t tollgate_last_linkid = 0;
    uint32_t odd_dis_tmp = 5000;
    uint32_t odd_dis_tmp1 = 5000;
    std::vector<uint32_t> link_id_vec = {};
    message::map_map::s_LinkInfo_t tollgate_last_link_infos;
    message::map_map::s_LinkInfo_t cur_use_link_infos;
    for (int map_route_list_linkid_index = 0; map_route_list_linkid_index < map_route_list_->LinkIds.LinkIds.size();
         map_route_list_linkid_index++) {
        link_id_vec.push_back(map_route_list_->LinkIds.LinkIds[map_route_list_linkid_index].LinkId);
    }
    int last_routelist_linkid_index = 0;
    uint32_t ego_pathoffset = 0;
    uint32_t ego_linkid = 0;
    ego_linkid = map_position_->LinkId;
    ego_pathoffset = map_position_->PathOffset;
    if (ego_linkid == 0) {
        in_odd_type_ = 0;
        odd_end_dist_ = 0;
        out_odd_reason_ = 1;
        return false;
    }

    if ((std::find(link_id_vec.begin(), link_id_vec.end(), ego_linkid) != link_id_vec.end()) && !link_id_vec.empty()) {
        uint32_t last_routelist_linkid = 0;
        last_routelist_linkid = link_id_vec.back();
        // std::cout << "last_routelist_linkid: " << last_routelist_linkid << std::endl;
        if (link_id_index_lane_info_map_->find(last_routelist_linkid) != link_id_index_lane_info_map_->end()) {
            last_routelist_linkid_index = link_id_index_lane_info_map_->at(last_routelist_linkid);
            routelist_end_dis =
                std::max((map_static_info_->LinkInfos.LinkInfos[last_routelist_linkid_index].EndOffset.EndOffset -
                          ego_pathoffset) /
                             100,
                         0U);
            routelist_start_dis =
                std::max((map_static_info_->LinkInfos.LinkInfos[last_routelist_linkid_index].PathOffset.PathOffset -
                          ego_pathoffset) /
                             100,
                         0U);
        } else {
            routelist_end_dis = 2500;
            routelist_start_dis = 2000;
        }

    } else {
        routelist_end_dis = 0;
        routelist_start_dis = 0;
    }

    for (const auto& segment : valid_geofence_segments_) {
        if (segment.geo_fence_type == 4) {
            tollgate_end_dis = (segment.last_endoffset - ego_pathoffset) / 100;
            tollgate_start_dis = (segment.first_pathoffset - ego_pathoffset) / 100;
            tollgate_linkids = segment.linkids;
        }
    }

    if (false == tollgate_linkids.empty()) {
        tollgate_last_linkid = tollgate_linkids.back();
    }

    GetLinkInfos(tollgate_last_linkid, tollgate_last_link_infos);
    auto it = std::find(link_id_vec.begin(), link_id_vec.end(), tollgate_last_linkid);
    // 检查是否找到元素
    if (it != link_id_vec.end() && tollgate_start_dis < 200) {
        tollgate_back_linkids_.clear();
        size_t index = std::distance(link_id_vec.begin(), it);
        for (int i = index; i < link_id_vec.size(); i++) {
            GetLinkInfos(link_id_vec[i], cur_use_link_infos);
            if (cur_use_link_infos.PathOffset.PathOffset <
                tollgate_last_link_infos.PathOffset.PathOffset + p_tollback_odd_distance * 100) {
                tollgate_back_linkids_.push_back(link_id_vec[i]);
            }
        }
    }
    // std::cout << "tollgate_back_linkids_ size: " << tollgate_back_linkids_.size() << "{ ";
    // for (int i = 0; i < tollgate_back_linkids_.size(); i++) {
    //     std::cout << tollgate_back_linkids_[i] << " ";
    // }
    // std::cout << "}" << std::endl;

    if ((std::find(tollgate_back_linkids_.begin(), tollgate_back_linkids_.end(), ego_linkid) !=
         tollgate_back_linkids_.end()) &&
        false == tollgate_back_linkids_.empty() && routelist_end_dis <= p_tollback_odd_distance) {
        odd_dis_tmp1 = 0;
    } else if ((std::find(tollgate_back_linkids_.begin(), tollgate_back_linkids_.end(), ego_linkid) !=
                tollgate_back_linkids_.end()) &&
               false == tollgate_back_linkids_.empty() && routelist_end_dis > p_tollback_odd_distance) {
        odd_dis_tmp1 = 5000;
        tollgate_back_linkids_.clear();
    }
    // std::cout << "odd_dis_tmp1: " << odd_dis_tmp1 << std::endl;

    odd_dis_tmp = std::min({routelist_end_dis, highway_end_dis, odd_dis_tmp1});
    ZTEXT("EFM_INFO", "routelist_start_dis: ", -30, -44, "routelist_start_dis: {}", routelist_start_dis);
    ZTEXT("EFM_INFO", "routelist_end_dis: ", -30, -47, "routelist_end_dis: {}", routelist_end_dis);
    if (odd_dis_tmp >= p_odd_to2_distance) {
        in_odd_type_ = 1;
        out_odd_reason_ = 0;
    } else if (odd_dis_tmp > p_odd_to0_distance) {
        in_odd_type_ = 2;
        out_odd_reason_ = 0;
    } else {
        in_odd_type_ = 0;
        out_odd_reason_ = 1;
    }
    odd_end_dist_ = (odd_dis_tmp > p_odd_to0_distance) ? odd_dis_tmp - p_odd_to0_distance : 0;

    return false;
}

bool ScenarioJudgeModel::EnterGeoFence() {
    message::map_map::s_GeoFences_t geo_fences{};
    std::vector<uint32_t> link_id_vec = {};
    for (int map_route_list_linkid_index = 0; map_route_list_linkid_index < map_route_list_->LinkIds.LinkIds.size();
         map_route_list_linkid_index++) {
        link_id_vec.push_back(map_route_list_->LinkIds.LinkIds[map_route_list_linkid_index].LinkId);
    }
    std::vector<GeoFenceSegment> all_type_geofence_segment{};
    geo_fences = map_static_info_->GeoFences;

    if (!geo_fences.GeoFences.empty()) {
        for (const auto& geo_fence : geo_fences.GeoFences) {
            if ((all_type_geofence_segment.empty() ||
                 //  all_type_geofence_segment.back().last_endoffset != geo_fence.PathOffset_.PathOffset ||
                 (std::abs(static_cast<int>(geo_fence.PathOffset_.PathOffset) -
                           static_cast<int>(all_type_geofence_segment.back().last_endoffset)) > 100) ||
                 all_type_geofence_segment.back().geo_fence_type != geo_fence.GeoFenceType.data) &&
                (std::find(link_id_vec.begin(), link_id_vec.end(), geo_fence.InstanceId_.InstanceId) !=
                 link_id_vec.end())) {
                GeoFenceSegment new_geo_fence_segment;
                new_geo_fence_segment.linkids.push_back(geo_fence.InstanceId_.InstanceId);
                new_geo_fence_segment.first_pathoffset = geo_fence.PathOffset_.PathOffset;
                new_geo_fence_segment.last_endoffset = geo_fence.EndOffset_.EndOffset;
                new_geo_fence_segment.geo_fence_type = geo_fence.GeoFenceType.data;
                all_type_geofence_segment.push_back(std::move(new_geo_fence_segment));
            } else if ((!all_type_geofence_segment.empty() &&
                        // all_type_geofence_segment.back().last_endoffset == geo_fence.PathOffset_.PathOffset) &&
                        (std::abs(static_cast<int>(geo_fence.PathOffset_.PathOffset) -
                                  static_cast<int>(all_type_geofence_segment.back().last_endoffset)) <= 100)) &&
                       all_type_geofence_segment.back().geo_fence_type == geo_fence.GeoFenceType.data &&
                       (std::find(link_id_vec.begin(), link_id_vec.end(), geo_fence.InstanceId_.InstanceId) !=
                        link_id_vec.end())) {
                all_type_geofence_segment.back().linkids.push_back(geo_fence.InstanceId_.InstanceId);
                all_type_geofence_segment.back().last_endoffset = geo_fence.EndOffset_.EndOffset;
                all_type_geofence_segment.back().geo_fence_type = geo_fence.GeoFenceType.data;
            } else {
                continue;
            }
        }

    } else {
        // 处理geo_fences.GeoFences为空的情况
    }
    geofence_segments_ = all_type_geofence_segment;
    return true;
}

bool ScenarioJudgeModel::ExtractValidGeoFenceSegments() {
    valid_geofence_segments_.clear();
    tollgate_links_.clear();
    for (const auto& segment : geofence_segments_) {
        if (segment.geo_fence_type == 3 || segment.geo_fence_type == 22) {
            valid_geofence_segments_.push_back(std::move(segment));
        } else if (segment.geo_fence_type == 4) {
            message::map_map::s_LinkInfo_t near_tool_link_infos;
            tollgate_links_ = segment.linkids;
            for (int linkidx = 0; linkidx < map_route_list_->LinkIds.LinkIds.size(); linkidx++) {
                uint32_t cur_use_linkid = map_route_list_->LinkIds.LinkIds[linkidx].LinkId;
                GetLinkInfos(cur_use_linkid, near_tool_link_infos);
                if (std::abs(static_cast<int>(near_tool_link_infos.PathOffset.PathOffset - segment.first_pathoffset)) <
                        p_tollahead_bigcurv_distance * 100 ||
                    std::abs(static_cast<int>(near_tool_link_infos.EndOffset.EndOffset - segment.last_endoffset)) <
                        p_tollback_bigcurv_distance * 100) {
                    tollgate_links_.push_back(cur_use_linkid);
                }
            }
            valid_geofence_segments_.push_back(std::move(segment));
        }
    }

    return true;
}

bool ScenarioJudgeModel::ByPassMerge(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model) {
    uint32_t close_link_id = 0;
    uint32_t close_link_pathoffset = 0;
    message::map_map::s_LinkInfo_t close_link_infos;
    uint32_t cur_position_offset = map_position_->PathOffset;
    std::vector<uint32_t> linklist_vec{0};

    linklist_vec = candidate_lanes_model->link_id_vec();

    // // // 遍历 unordered_map
    // for (const auto& pair : *to_link_id_index_lane_connect_map_) {
    //     uint32_t link_id = pair.first;
    //     const std::vector<int>& lane_connect = pair.second;

    //     // 打印 link_id 和其对应的 vector<int>
    //     std::cout << "Link ID: " << link_id << " is connected to lanes: ";
    //     for (int lane_index : lane_connect) {
    //         std::cout << lane_index << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // std::cout << "link_id_vec:[ ";
    // for (int i = 0; i < linklist_vec.size(); i++) {
    //     std::cout << linklist_vec[i] << ",";
    // }
    // std::cout << "]" << std::endl;

    for (int linkidx = 0; linkidx < map_route_list_->LinkIds.LinkIds.size(); linkidx++) {
        uint32_t cur_use_linkid = map_route_list_->LinkIds.LinkIds[linkidx].LinkId;
        uint32_t from_link_id1 = 0;
        uint32_t from_link_id2 = 0;
        uint32_t ego_conect_link_id = 0;
        uint32_t other_conect_link_id = 0;
        uint8_t cur_use_linkid_ego_lanenum = 0;
        uint8_t ego_conect_max_lane_num = 0;
        uint8_t other_conect__max_lane_num = 0;
        message::map_map::s_LinkInfo_t ego_conect_link_infos;
        message::map_map::s_LinkInfo_t other_conect_link_infos;
        if (IsByPassLink(cur_use_linkid, from_link_id1, from_link_id2)) {
            // std::cout << "close_link_id: " << cur_use_linkid << std::endl;
            // std::cout << "from_link_id1: " << from_link_id1 << std::endl;
            // std::cout << "from_link_id2: " << from_link_id2 << std::endl;
            auto it = std::find(linklist_vec.begin(), linklist_vec.end(), cur_use_linkid);
            // 检查是否找到元素
            if (it != linklist_vec.end()) {
                // 计算索引
                size_t index = std::distance(linklist_vec.begin(), it);
                (index > 0) ? (ego_conect_link_id = linklist_vec[index - 1]) : (ego_conect_link_id = 0);
            }
        }
        if (ego_conect_link_id == from_link_id1 && from_link_id2 != 0 && from_link_id1 != 0) {
            other_conect_link_id = from_link_id2;
        } else if (ego_conect_link_id == from_link_id2 && from_link_id2 != 0 && from_link_id1 != 0) {
            other_conect_link_id = from_link_id1;
        } else {
            continue;
        }
        GetLinkInfos(ego_conect_link_id, ego_conect_link_infos);
        GetLinkInfos(other_conect_link_id, other_conect_link_infos);
        for (auto& PairConnectivity : map_static_info_->LaneConnectivitys.PairConnectivity) {
            if (ego_conect_link_id == PairConnectivity.FromLinkId.FromLinkId &&
                cur_use_linkid == PairConnectivity.ToLinkId.ToLink) {
                // std::cout << "PairConnectivity.NewLaneNum.NewLaneNum: " <<
                // int(PairConnectivity.NewLaneNum.NewLaneNum)
                //           << std::endl;
                ego_conect_max_lane_num = std::max(ego_conect_max_lane_num, PairConnectivity.NewLaneNum.NewLaneNum);
            }
            if (other_conect_link_id == PairConnectivity.FromLinkId.FromLinkId &&
                cur_use_linkid == PairConnectivity.ToLinkId.ToLink) {
                // std::cout << "PairConnectivity.NewLaneNum.NewLaneNum: " <<
                // int(PairConnectivity.NewLaneNum.NewLaneNum)
                //           << std::endl;
                other_conect__max_lane_num =
                    std::max(other_conect__max_lane_num, PairConnectivity.NewLaneNum.NewLaneNum);
            }
        }
        // std::cout << "ego_conect_max_lane_num: " << int(ego_conect_max_lane_num) << std::endl;
        // std::cout << "other_conect__max_lane_num: " << int(other_conect__max_lane_num) << std::endl;

        if (ego_conect_max_lane_num > other_conect__max_lane_num) {
            close_link_id = cur_use_linkid;
            // std::cout << "ByPassMerge" << std::endl;
            break;
        }
    }
    if (false == GetLinkInfos(close_link_id, close_link_infos)) {
        // static data none
        dDistanceByPassMerge_ = 5000.0;
        return true;
    }
    close_link_pathoffset = close_link_infos.PathOffset.PathOffset;
    dDistanceByPassMerge_ =
        (close_link_pathoffset > cur_position_offset) ? ((close_link_pathoffset - cur_position_offset) / 100) : 5000.0;
    // std::cout << "dDistanceByPassMerge_: " << dDistanceByPassMerge_ << std::endl;
    return true;
}

bool ScenarioJudgeModel::YShapeLink() {
    is_y_shape_link_ = false;
    bool is_split_link = false;
    bool is_split_lane = false;
    bool is_by_pass_ramplink = false;
    uint32_t cur_link_id = map_position_->LinkId;
    uint32_t ego_pathoffset = map_position_->PathOffset;
    uint32_t from_link_id1 = 0;
    uint32_t from_link_id2 = 0;
    for (int i = cur_rp_index_; i < (map_route_list_->LinkCount.LinkCount - cur_rp_index_); i++) {
        message::map_map::s_LinkInfo_t cur_search_link_infos{};
        uint32_t cur_search_link_id = map_route_list_->LinkIds.LinkIds[i].LinkId;
        GetLinkInfos(cur_search_link_id, cur_search_link_infos);
        // if (cur_search_link_infos.EndOffset.EndOffset > ego_pathoffset + 150 * 100) {
        //     break;
        // }
        if (IsByPassLink(cur_search_link_id, from_link_id1, from_link_id2) &&
            (cur_search_link_infos.PathOffset.PathOffset < ego_pathoffset + p_y_shape_distance * 100)) {
            is_by_pass_ramplink = true;
            // std::cout << __FILE__ << __LINE__ << "cur_search_link_id: " << cur_search_link_id << std::endl;
            for (auto lane : cur_search_link_infos.LaneInfos.LaneInfos) {
                if (lane.LaneType.data != 3 && lane.LaneType.data != 4 && lane.LaneType.data != 5 &&
                    lane.LaneType.data != 6) {
                    is_by_pass_ramplink = false;
                    break;
                }
            }
            // std::cout << "is_by_pass_ramplink: " << int(is_by_pass_ramplink) << std::endl;
        }
        is_split_link = IsSplitLink(cur_search_link_id) &&
                        (cur_search_link_infos.EndOffset.EndOffset < ego_pathoffset + p_y_shape_distance * 100);
        // is_split_lane = LinkHasSplitLane(cur_search_link_id) && (cur_search_link_infos.EndOffset.EndOffset <
        // ego_pathoffset + p_y_shape_distance * 100);
        if (is_split_link || is_split_lane || is_by_pass_ramplink) {
            is_y_shape_link_ = true;
            // std::cout << "YShapeLink" << std::endl;
            // std::cout << "cur_search_link_id: " << cur_search_link_id << std::endl;
            return true;
        }
    }
    // std::cout << __FILE__ << __LINE__ << "is_y_shape_link_: " << int(is_y_shape_link_) << std::endl;
    return false;
}

bool ScenarioJudgeModel::IsByPassLink(uint32_t link_id, uint32_t& from_link_id1, uint32_t& from_link_id2) {
    from_link_id1 = 0;
    from_link_id2 = 0;
    if (to_link_id_index_lane_connect_map_->find(link_id) != to_link_id_index_lane_connect_map_->end()) {
        uint32_t temp_link_id = 0;
        for (auto idx : to_link_id_index_lane_connect_map_->at(link_id)) {
            if (0 == temp_link_id) {
                temp_link_id = map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId;
                from_link_id1 = temp_link_id;
            } else {
                if (temp_link_id != map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId) {
                    from_link_id2 = map_static_info_->LaneConnectivitys.PairConnectivity[idx].FromLinkId.FromLinkId;
                    return true;
                }
            }
        }
    }

    return false;
}

bool ScenarioJudgeModel::IsSplitLink(uint32_t link_id) {
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

bool ScenarioJudgeModel::LinkHasSplitLane(uint32_t link_id) {
    auto link_it = link_id_index_lane_connect_map_->find(link_id);
    if (link_it != link_id_index_lane_connect_map_->end()) {
        std::vector<uint8_t> InitLaneNum{};
        for (auto index : link_it->second) {
            InitLaneNum.push_back(map_static_info_->LaneConnectivitys.PairConnectivity[index].InitLaneNum.InitLaneNum);
        }
        std::set<uint8_t> unique_elements(InitLaneNum.begin(), InitLaneNum.end());
        if (unique_elements.size() < InitLaneNum.size()) {
            // std::cout << __FILE__ << __LINE__ << "LinkHasSplitLane" << std::endl;
            return true;
        }
    }

    return false;
}

bool ScenarioJudgeModel::GetLinkInfos(uint32_t link_id, message::map_map::s_LinkInfo_t& link_infos) {
    if (link_id_index_lane_info_map_->find(link_id) != link_id_index_lane_info_map_->end()) {
        int link_index = link_id_index_lane_info_map_->at(link_id);
        link_infos = map_static_info_->LinkInfos.LinkInfos[link_index];
        return true;
    } else {
        return false;
    }
}

bool ScenarioJudgeModel::GetLinkMinSpeedLimit(uint32_t link_id, uint8_t& speed_limit) {
    speed_limit = 255;
    uint8_t min_speed = 255;
    for (auto spd_limit : map_static_info_->LaneSpeedLimitis.LaneSpeedLimits) {
        if (spd_limit.InstanceId.InstanceId == link_id) {
            if (spd_limit.ValueH.ValueH != 0 && spd_limit.ValueH.ValueH < min_speed) {
                min_speed = spd_limit.ValueH.ValueH;
            }
        }
    }
    speed_limit = min_speed;

    return true;
}

bool ScenarioJudgeModel::MainRoadWideLane(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model,
                                          const std::shared_ptr<ExtractRefLine> extract_ref_line) {
    uint8_t range_idx = 0;
    std::vector<uint32_t> linklist_vec = candidate_lanes_model->link_id_vec();
    std::vector<uint8_t> prior_lanenum_vec{0};
    std::vector<uint16_t> prior_lane_width_max_vec{0};
    int32_t ego_lane_idx = candidate_lanes_model->ego_lane_prior_index();
    int32_t left_lane_idx = candidate_lanes_model->left_lane_prior_index();
    int32_t right_lane_idx = candidate_lanes_model->right_lane_prior_index();
    int32_t ref_lane_idx = candidate_lanes_model->get_ref_lane_prior_index();

    if (ref_lane_idx == left_lane_idx) {
        prior_lanenum_vec = candidate_lanes_model->left_path_lane_num();
        prior_lane_width_max_vec = candidate_lanes_model->left_lane_width_max_vec();
    } else if (ref_lane_idx == right_lane_idx) {
        prior_lanenum_vec = candidate_lanes_model->right_path_lane_num();
        prior_lane_width_max_vec = candidate_lanes_model->right_lane_width_max_vec();
    } else {
        prior_lanenum_vec = candidate_lanes_model->ego_path_lane_num();
        prior_lane_width_max_vec = candidate_lanes_model->ego_lane_width_max_vec();
    }

    std::vector<BigWidthLane> big_width_lanes{0};
    uint32_t temp_line_id = 0;
    is_has_virtual_lane_ = 0;

    for (uint8_t linkidx = 0; linkidx < prior_lanenum_vec.size() && linkidx < linklist_vec.size() && linkidx < prior_lane_width_max_vec.size(); linkidx++) {
        uint32_t cur_search_link_id = linklist_vec[linkidx];
        uint8_t cur_search_prior_lane_num = prior_lanenum_vec[linkidx];
        uint8_t cur_search_prior_left_lane_num = prior_lanenum_vec[linkidx] + 1;
        uint8_t cur_search_prior_right_lane_num = (prior_lanenum_vec[linkidx] > 1) ? prior_lanenum_vec[linkidx] - 1 : 0;
        double cur_ego_lane_width_max = prior_lane_width_max_vec[linkidx];
        uint8_t left_virtual_line_case = 0;
        uint8_t right_virtual_line_case = 0;
        message::map_map::s_LinkInfo_t cur_search_link_infos{};
        GetLinkInfos(cur_search_link_id, cur_search_link_infos);
        // std::cout << "cur_link_id: " << cur_search_link_id << "cur_ego_lane_width: " << cur_ego_lane_width_max
        //           << std::endl;
        // std::cout << "cur_link_pathoffset: " << cur_search_link_infos.PathOffset.PathOffset << std::endl;
        // std::cout << "map_position_PathOffset: " << map_position_->PathOffset << std::endl;
        // std::cout << "cur_lane_type_break " << std::endl;
        // std::cout << "offset_dis: " << cur_search_link_infos.PathOffset.PathOffset - cur_position_offset <<
        // std::endl;
        if (cur_search_link_infos.PathOffset.PathOffset > (map_position_->PathOffset + 300 * 100)) {
            break;
        }

        IsVirtuallyLine(cur_search_link_infos, cur_search_prior_left_lane_num, cur_search_prior_lane_num, true, left_virtual_line_case);
        IsVirtuallyLine(cur_search_link_infos, cur_search_prior_lane_num, cur_search_prior_right_lane_num, false, right_virtual_line_case);
        if (cur_ego_lane_width_max > p_lane_width_max || left_virtual_line_case == 1 || right_virtual_line_case == 1) {
            BigWidthLane big_width_lane;
            big_width_lane.link_id = cur_search_link_id;
            big_width_lane.link_pathoffset = cur_search_link_infos.PathOffset.PathOffset;
            big_width_lane.link_endoffset = cur_search_link_infos.EndOffset.EndOffset;
            big_width_lanes.push_back(std::move(big_width_lane));
        }  
    }

    // std::cout << "big_width_lanes_linkids:[ ";
    // for (int i = 0; i < big_width_lanes.size(); i++) {
    //     std::cout << "link_id: " << big_width_lanes[i].link_id << ", ";
    // }
    // std::cout << "]" << std::endl;

    if (false == big_width_lanes.empty()) {
        for (const auto& lane : big_width_lanes) {
            if (lane.link_id == map_position_->LinkId) {
                is_has_virtual_lane_ = true;
                // std::cout << "map_position_->LinkId: " << map_position_->LinkId << std::endl;
            }
        }
        std::sort(big_width_lanes.begin(), big_width_lanes.end(), CompareDis);
        if (map_position_->PathOffset + p_bigwidth_forward_distance * 100 > big_width_lanes[0].link_pathoffset ||
            big_width_lanes.back().link_endoffset + p_bigwidth_back_distance * 100 < map_position_->PathOffset) {
            // std::cout << "map_position_->PathOffset: " << map_position_->PathOffset << std::endl;
            // std::cout << "big_width_lanes[0].link_pathoffset: " << big_width_lanes[0].link_pathoffset << std::endl;
            // std::cout << "big_width_lanes[0].link_id: " << big_width_lanes[0].link_id << std::endl;
            // std::cout << "big_width_lanes.back().link_endoffset: " << big_width_lanes.back().link_endoffset <<std::endl; 
            // std::cout << "big_width_lanes.back().link_id: " << big_width_lanes.back().link_id <<std::endl;
            is_has_virtual_lane_ = true;
        }
    }
    // std::cout << "is_has_virtual_lane_: " << is_has_virtual_lane_ << std::endl;

    return true;
}

bool ScenarioJudgeModel::BigCurvature(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model) {
    // std::cout << __FILE__ << __LINE__ << "bigcurvature start: " << std::endl;
    std::vector<uint32_t> static_map_linkid_vec{0};
    std::vector<uint8_t> prior_lanenum_vec{0};
    int32_t ego_lane_idx = candidate_lanes_model->ego_lane_prior_index();
    int32_t left_lane_idx = candidate_lanes_model->left_lane_prior_index();
    int32_t right_lane_idx = candidate_lanes_model->right_lane_prior_index();
    int32_t ref_lane_idx = candidate_lanes_model->get_ref_lane_prior_index();
    static_map_linkid_vec = candidate_lanes_model->link_id_vec();
    bool is_has_big_curvature_tmp = false;
    std::vector<uint32_t> big_curvature_links{};
    uint32_t position_forward_dist = 0;
    uint8_t cur_lane_num = map_position_->LaneId;
    uint8_t cur_lane_type = 4;
    uint32_t cur_link_id = map_position_->LinkId;
    std::vector<CurvPoint> all_curvature_vec{};
    all_curvature_vec.clear();
    uint8_t big_curv_point_num = 0;
    message::map_map::s_LinkInfo_t cur_link_infos{};
    GetLinkInfos(cur_link_id, cur_link_infos);
    prior_lanenum_vec = candidate_lanes_model->ref_route_path_lane_num();

    if ((static_map_linkid_vec.size() < prior_lanenum_vec.size()) || (static_map_linkid_vec.size() == 0) ||
        (prior_lanenum_vec.size() == 0)) {
        is_has_big_curvature_ = false;
        is_has_big_curvature_tmp = false;
        return false;
    }

    for (const auto& lane : cur_link_infos.LaneInfos.LaneInfos) {
        if (lane.LaneNum.LaneNum == cur_lane_num) {
            cur_lane_type = lane.LaneType.data;
        }
    }

    if (exit_end_dist_ < p_bigcurve_oddto2_distance && exit_end_dist_ >= 0) {
        bool prior_lane_exits_ramp = false;
        GetBackRoutelistLinks();
        for (int linkididx = 0; linkididx < prior_lanenum_vec.size(); linkididx++) {
            message::map_map::s_LinkInfo_t link_infos{};
            bool prior_lane_is_ramp = false;
            bool ramp_in_tolgate = false;
            bool link_in_back_routelist = false;
            GetLinkInfos(static_map_linkid_vec[linkididx], link_infos);
            uint32_t cur_search_link_id = static_map_linkid_vec[linkididx];
            uint8_t cur_search_lane_num = prior_lanenum_vec[linkididx];
            for (const auto& lane : link_infos.LaneInfos.LaneInfos) {
                if (lane.LaneNum.LaneNum == cur_search_lane_num) {
                    if (lane.LaneType.data == 4 || lane.LaneType.data == 5 || lane.LaneType.data == 6) {
                        prior_lane_is_ramp = true;
                        prior_lane_exits_ramp = true;
                    }
                }
            }
            if (true == prior_lane_exits_ramp && false == prior_lane_is_ramp) {
                break;
            }
            if (false == tollgate_links_.empty()) {
                auto it = std::find(tollgate_links_.begin(), tollgate_links_.end(), cur_search_link_id);
                if (it != tollgate_links_.end()) {
                    ramp_in_tolgate = true;
                }
            }
            if (false == back_routelist_links_.empty()) {
                if (std::find(back_routelist_links_.begin(), back_routelist_links_.end(), cur_search_link_id) != back_routelist_links_.end()) {
                    link_in_back_routelist = true;
                }
            }
            if (true == prior_lane_is_ramp && false == ramp_in_tolgate && false == link_in_back_routelist) {
                for (const auto& linkcurvpoint : map_static_info_->LinkCurvatures.LinkCurvatures) {
                    if (linkcurvpoint.InstanceId.InstanceId == cur_search_link_id &&
                        linkcurvpoint.LaneNum.LaneNum == cur_search_lane_num) {
                        for (const auto& lanecurvpoint : linkcurvpoint.CurvPoints.CurvPoints) {
                            CurvPoint curv_point_info;
                            curv_point_info.CurvPointPathOffset =
                                (link_infos.PathOffset.PathOffset + lanecurvpoint.PathOffset.PathOffset) / 100;
                            curv_point_info.CurvPointValue =
                                std::abs(100000 / lanecurvpoint.CurvPointValue.CurvPointValue);
                            curv_point_info.linkid = cur_search_link_id;
                            all_curvature_vec.push_back(curv_point_info);
                        }
                    }
                }
            }
        }
    }

    // std::cout << __FILE__ << __LINE__ << "bigcurvature end1111: " << std::endl;
    if (all_curvature_vec.size() >= 2) {
        for (int i = 0; i < all_curvature_vec.size() - 1; ++i) {
            for (int j = i + 1; j < all_curvature_vec.size(); ++j) {
                int curv_point_dis = std::abs(static_cast<int>(all_curvature_vec[j].CurvPointPathOffset -
                                                               all_curvature_vec[i].CurvPointPathOffset));
                if (all_curvature_vec[i].CurvPointValue < p_bigcurve_odd_radius &&
                    all_curvature_vec[j].CurvPointValue < p_bigcurve_odd_radius &&
                    all_curvature_vec[i].CurvPointValue > p_bigcurve_odd_abnomal_radius &&
                    all_curvature_vec[j].CurvPointValue > p_bigcurve_odd_abnomal_radius &&
                    curv_point_dis > p_curv_point_dis_min && curv_point_dis < p_curv_point_dis_max) {
                    is_has_big_curvature_ = true;
                    is_has_big_curvature_tmp = true;
                    big_curve_position_offset_ = map_position_->PathOffset;
                    // std::cout << "big_curve_linkid1: " << all_curvature_vec[i].linkid << std::endl;
                    // std::cout << "big_curve_linkid2: " << all_curvature_vec[j].linkid << std::endl;
                    // std::cout << "all_curvature_vec[i].CurvPointValue: " << all_curvature_vec[i].CurvPointValue << std::endl;
                    // std::cout << "all_curvature_vec[j].CurvPointValue: " << all_curvature_vec[j].CurvPointValue << std::endl;
                    // std::cout << "big_curve_position_offset_: " << big_curve_position_offset_ << std::endl;
                    break;
                }
            }
            if (is_has_big_curvature_tmp) {
                break;
            }
        }
    }

    // std::cout << __FILE__ << __LINE__ << "bigcurvature end222: " << std::endl;

    // std::cout << "curvature pathoffset:[ ";
    // for (int i = 0; i < all_curvature_vec.size(); i++) {
    //     std::cout << all_curvature_vec[i].CurvPointPathOffset << ",";
    // }
    // std::cout << "]" << std::endl;

    // std::cout << "curvature value:[ ";
    // for (int i = 0; i < all_curvature_vec.size(); i++) {
    //     std::cout << all_curvature_vec[i].CurvPointValue << ",";
    // }
    // std::cout << "]" << std::endl;

    // position_forward_dist = (map_position_->PathOffset - big_curve_position_offset_) / 100;
    if (false == is_has_big_curvature_tmp && true == is_has_big_curvature_) {
        if ((map_position_->PathOffset > (big_curve_position_offset_ + 100 * 1000)) ||
            !(cur_lane_type == 4 || cur_lane_type == 5 || cur_lane_type == 6)) {
            is_has_big_curvature_ = false;
        }
    }

    // std::cout << "big_curv_point_num: " << big_curv_point_num << std::endl;
    // std::cout << "is_has_big_curvature_tmp: " << int(is_has_big_curvature_tmp) << std::endl;
    // std::cout << "is_has_big_curvature_: " << int(is_has_big_curvature_) << std::endl;
    // std::cout << __FILE__ << __LINE__ << "bigcurvature end: " << std::endl;
    return true;
}

bool ScenarioJudgeModel::DirectExitScene(double exit_start_dist, double exit_end_dist) {
    // std::cout << __FILE__ << __LINE__ << "is_exit_start_dist_f_: " <<is_exit_start_dist_f_<< std::endl;
    if (last_cycle_exit_end_dist_ < 10 && exit_end_dist > last_cycle_exit_end_dist_) {
        // next exit
        is_exit_start_dist_f_ = false;
    }

    if (exit_start_dist > 0 && exit_end_dist > 0) {
        is_exit_start_dist_f_ = true;
        exit_type_ = 0;  // not direct off ramp
    } else if (is_exit_start_dist_f_ == true && exit_end_dist > 0) {
        exit_type_ = 0;  // not direct off ramp
    } else if (is_exit_start_dist_f_ == false && exit_end_dist > 0) {
        exit_type_ = 1;
        is_exit_start_dist_f_ = false;
    } else {
        exit_type_ = 0;
        is_exit_start_dist_f_ = false;
    }
    last_cycle_exit_end_dist_ = exit_end_dist;
    return true;
}

bool ScenarioJudgeModel::GetLineType(const message::map_map::s_LinkInfo_t& link_infos, uint8_t lane_id, uint8_t& line_type,
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

bool ScenarioJudgeModel::RampToMainRoad(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model) {
    auto node = candidate_lanes_model->GetNOdeINfo();
    is_ramp_to_main_road_ = false;
    left_lane_spd_limit_ = 80;
    uint8_t lane_chg_dirdec_type = node.DirectionType.data_;
    uint32_t ego_link_id = map_position_->LinkId;
    uint8_t ego_lane_num = map_position_->LaneId;
    uint8_t ego_lane_type = 21, left_lane_type = 21;
    double node_start_dist = node.StartPointOffset, node_end_dist = node.EndPointOffset;
    std::vector<uint32_t> linklist_vec = candidate_lanes_model->link_id_vec();
    std::vector<uint8_t> ego_path_lane_num = candidate_lanes_model->ego_path_lane_num();
    std::vector<uint8_t> left_path_lane_num = candidate_lanes_model->left_path_lane_num();
    message::map_map::s_LinkInfo_t ego_link_infos;

    if (false == GetLinkInfos(ego_link_id, ego_link_infos)) {
        return false;
    }

    for (const auto& lane : ego_link_infos.LaneInfos.LaneInfos) {
        if (lane.LaneNum.LaneNum == ego_lane_num) {
            ego_lane_type = lane.LaneType.data;
        } else if (lane.LaneNum.LaneNum == ego_lane_num + 1) {
            left_lane_type = lane.LaneType.data;
        }
    }

    for(const auto& spd_limit_info: map_static_info_->LaneSpeedLimitis.LaneSpeedLimits){
        if(spd_limit_info.InstanceId.InstanceId == ego_link_id && spd_limit_info.LaneNum.LaneNum == ego_lane_num + 1){
            left_lane_spd_limit_ = spd_limit_info.ValueH.ValueH;
        }

    }

    // std::cout << "node_start_dist: " << node_start_dist << std::endl;
    // std::cout << "ego_lane_type: " << int(ego_lane_type) << std::endl;
    // std::cout << "left_lane_type: " << int(left_lane_type) << std::endl;
    // std::cout << "left_path_lane_num.size(): " << left_path_lane_num.size() << std::endl;
    // std::cout << "ego_path_lane_num.size(): " << ego_path_lane_num.size() << std::endl;
    // std::cout << "main_road_nearby_ramp_lanenum_.size(): " << main_road_nearby_ramp_lanenum_.size() << std::endl;
    // std::cout << "lane_chg_dirdec_type: " << int(lane_chg_dirdec_type) << std::endl;
    if (node_start_dist > -0.1 && node_start_dist < 0.1 && (ego_lane_type == 7 || ego_lane_type == 1) &&
        left_lane_type == 0 && true == main_road_nearby_ramp_lanenum_.empty() &&
        (left_path_lane_num.size() >= ego_path_lane_num.size()) && lane_chg_dirdec_type == 1) {
        for (int lanenumidx = 0; lanenumidx < ego_path_lane_num.size(); lanenumidx++) {
            uint8_t cur_search_ego_lane_num = ego_path_lane_num[lanenumidx];
            uint8_t cur_search_left_lane_num = left_path_lane_num[lanenumidx];
            uint32_t cur_search_link_id = linklist_vec[lanenumidx];
            uint8_t cur_search_ego_lane_type = 21, cur_search_left_lane_type = 21;
            message::map_map::s_LinkInfo_t cur_search_link_infos;
            if (false == GetLinkInfos(cur_search_link_id, cur_search_link_infos)) {
                return false;
            }
            for (const auto& lane : cur_search_link_infos.LaneInfos.LaneInfos) {
                if (lane.LaneNum.LaneNum == cur_search_ego_lane_num) {
                    cur_search_ego_lane_type = lane.LaneType.data;
                } else if (lane.LaneNum.LaneNum == cur_search_left_lane_num) {
                    cur_search_left_lane_type = lane.LaneType.data;
                }
            }
            if ((cur_search_ego_lane_type == 7 || cur_search_ego_lane_type == 1) && cur_search_left_lane_type == 0) {
                main_road_nearby_ramp_lanenum_.insert({cur_search_link_id, cur_search_left_lane_num});
            }
            if (cur_search_ego_lane_type != 7 && cur_search_ego_lane_type != 1 && cur_search_left_lane_type == 0) {
                break;
            }
        }
    }

    // std::cout << "ego_link_id: " << ego_link_id << std::endl;
    // std::cout << "ego_lane_num: " << int(ego_lane_num) << std::endl;
    // std::cout << "main_road_nearby_ramp_lanenum_.size(): ";
    // for (const auto& pair : main_road_nearby_ramp_lanenum_) {
    //     std::cout << "link_id: " << pair.first << ", lane_num: " << int(pair.second) << std::endl;
    // }
    auto ego_link_it = main_road_nearby_ramp_lanenum_.find(ego_link_id);
    if (ego_link_it != main_road_nearby_ramp_lanenum_.end() && ego_link_it->second == ego_lane_num) {
        is_ramp_to_main_road_ = true;
    }
    // ZTEXT("EFM_INFO", "is_ramp_to_main_road: ", -30, -53, "is_ramp_to_main_road: {}", int(is_ramp_to_main_road_));
    if (ego_link_it == main_road_nearby_ramp_lanenum_.end()) {
        main_road_nearby_ramp_lanenum_.clear();
    }
    return false;
}

void ScenarioJudgeModel::RampToMainRoadToRamp(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model){
    uint32_t ego_link_id = map_position_->LinkId;
    uint8_t ego_lane_num = map_position_->LaneId;
    uint8_t ego_lane_type = 21;
    std::vector<uint32_t> linklist_vec = candidate_lanes_model->link_id_vec();
    std::vector<uint8_t> ref_route_path_lane_num = candidate_lanes_model->ref_route_path_lane_num();
    std::vector<std::pair<uint32_t, uint8_t>> ref_path_link_lanetype{};
    std::vector<std::pair<uint32_t, uint32_t>> ego_path_link_offset{};
    size_t ref_route_path_lane_num_size = std::min(ref_route_path_lane_num.size(), linklist_vec.size());
    message::map_map::s_LinkInfo_t ego_link_infos;
    uint32_t ego_link_pathoffset = 0;

    // std::cout << __FILE__ << __LINE__ << "tar_link_list_: {";
    // for(const auto& link_id: tar_link_list_){
    //     std::cout << link_id << ", ";
    // }
    // std::cout << "}" << std::endl;
    // std::cout << __FILE__ << __LINE__ << "tar_link_max_endoffset_: " << tar_link_max_endoffset_ << std::endl;

    if(tar_link_list_.empty()){
        ZTEXT("EFM_INFO", "is_ramp_main_ramp: ", -30, -62, "is_ramp_main_ramp: {}", 0);
    }else{
        ZTEXT("EFM_INFO", "is_ramp_main_ramp: ", -30, -62, "is_ramp_main_ramp: {}", 1);
    }

    if((map_position_->PathOffset + 5* 100 < tar_link_max_endoffset_) 
        || (false == GetLinkInfos(ego_link_id, ego_link_infos))){
        return ;
    }

    if(map_position_->PathOffset > tar_link_max_endoffset_ + 10 * 100){
        tar_link_list_.clear();
        tar_link_max_endoffset_ = 0;
        tar_link_max_speed_ = 60;
    }

    for (const auto& lane : ego_link_infos.LaneInfos.LaneInfos) {
        if (lane.LaneNum.LaneNum == ego_lane_num) {
            ego_lane_type = lane.LaneType.data;
            ego_link_pathoffset = ego_link_infos.PathOffset.PathOffset;
        }
    }
    std::unordered_set<uint8_t>ramp_lanetype{4,5,6,7,1,8,2};
    if(exit_end_dist_ < 1000 && exit_end_dist_ > 100 && tar_link_list_.empty() 
        && ramp_lanetype.find(ego_lane_type) != ramp_lanetype.end()){
        for(size_t i = 0 ; i < ref_route_path_lane_num_size; i++){
            uint32_t cur_search_link_id = linklist_vec[i];
            uint8_t cur_search_ego_lane_num = ref_route_path_lane_num[i];
            message::map_map::s_LinkInfo_t cur_search_link_infos;
            if (false == GetLinkInfos(cur_search_link_id, cur_search_link_infos)) {
                return ;
            }
            if(cur_search_link_infos.PathOffset.PathOffset < ego_link_pathoffset + 1500 *100){
                for (const auto& lane : cur_search_link_infos.LaneInfos.LaneInfos) {
                    if (lane.LaneNum.LaneNum == cur_search_ego_lane_num) {
                        ref_path_link_lanetype.emplace_back(cur_search_link_id, lane.LaneType.data);
                        ego_path_link_offset.emplace_back(cur_search_link_id, cur_search_link_infos.PathOffset.PathOffset);
                    }
                }
            }

        }
    }

    if(true == ref_path_link_lanetype.empty()){
        return ;
    }

    int ramp_to_main_raod_index = -1, main_road_to_ramp_index = -1;
    uint8_t pre_linklane_lanetype = 99;
    int index = 0;
    for (const auto& item : ref_path_link_lanetype) {
        uint8_t value = item.second;
        if(ramp_to_main_raod_index == -1 && ramp_lanetype.find(pre_linklane_lanetype) != ramp_lanetype.end() && value == 0){
            ramp_to_main_raod_index = index;
        }
        if(main_road_to_ramp_index == -1 && pre_linklane_lanetype == 0 && ramp_lanetype.find(value) != ramp_lanetype.end()){
            main_road_to_ramp_index = index;
        }
        pre_linklane_lanetype = value;
        index++;
    }

    // std::cout << __FILE__ << __LINE__ << "ramp_to_main_raod_index: " << ramp_to_main_raod_index << std::endl;
    // std::cout << __FILE__ << __LINE__ << "main_road_to_ramp_index: " << main_road_to_ramp_index << std::endl;

    if(ramp_to_main_raod_index == -1 || main_road_to_ramp_index == -1){
        return ;
    }
    // else if((main_road_to_ramp_index == -1 && ramp_to_main_raod_index != -1) ||
    //          (main_road_to_ramp_index != -1 && ramp_to_main_raod_index == -1)) {
    //     int valid_index = (main_road_to_ramp_index != -1) ? main_road_to_ramp_index : ramp_to_main_raod_index;
    //     tar_link_list_.emplace_back(ref_path_link_lanetype[valid_index].first);
    //     tar_link_max_endoffset_ = std::max(tar_link_max_endoffset_, ego_path_link_offset[valid_index].second);
    // }
    else{
        for(int i = ramp_to_main_raod_index; i <= main_road_to_ramp_index; i++){
            tar_link_list_.emplace_back(ref_path_link_lanetype[i].first);
            tar_link_max_endoffset_ = std::max(tar_link_max_endoffset_, ego_path_link_offset[i].second);
        }
    }

    for(size_t link_idx = 0; link_idx < tar_link_list_.size(); link_idx++){
        message::map_map::s_LinkInfo_t tar_link_info;
        if (false == GetLinkInfos(tar_link_list_[link_idx], tar_link_info)) {
            return ;
        }
        for(int i = 0 ; i < tar_link_info.LaneInfos.LaneInfos.size(); i++){
            if(tar_link_info.LaneInfos.LaneInfos[i].LaneType.data == 0){
                for(const auto& spd_limit_info: map_static_info_->LaneSpeedLimitis.LaneSpeedLimits){
                    if(spd_limit_info.InstanceId.InstanceId == tar_link_list_[link_idx] && spd_limit_info.LaneNum.LaneNum == i + 1){
                        tar_link_max_speed_ = std::max(tar_link_max_speed_, spd_limit_info.ValueH.ValueH);
                        break;
                    }
                }
                break;
            }

        }
    }

    return ;
}

bool ScenarioJudgeModel::CompareDis(const BigWidthLane& a, const BigWidthLane& b) {
    return a.link_pathoffset < b.link_pathoffset;
}

bool ScenarioJudgeModel::GetBackRoutelistLinks() { 
    
    uint16_t back_search_length = 0;
    for(int i = map_route_list_->LinkCount.LinkCount - 1; i >= 0; i--){
        if (back_search_length >= p_lastlink_back_search_distance) {
            break;
        }
        uint32_t cur_search_link_id = map_route_list_->LinkIds.LinkIds[i].LinkId;
        message::map_map::s_LinkInfo_t cur_search_link_infos{};
        if(false == GetLinkInfos(cur_search_link_id, cur_search_link_infos)){
            return false;
        }
        uint16_t cur_search_link_length = (cur_search_link_infos.EndOffset.EndOffset > cur_search_link_infos.PathOffset.PathOffset) ? 
            (cur_search_link_infos.EndOffset.EndOffset - cur_search_link_infos.PathOffset.PathOffset)/100 : 0;
        back_search_length += cur_search_link_length;
        back_routelist_links_.push_back(map_route_list_->LinkIds.LinkIds[i].LinkId);
    }
    return true; 
}

void ScenarioJudgeModel::IsVirtuallyLine(const message::map_map::s_LinkInfo_t& link_infos,
                                       const uint8_t& left_lane_id, const uint8_t& right_lane_id, bool is_left, uint8_t& virtual_line_case) {
    // virtual_line_case: 0: not virtual line, 1: valid virtual line, 2: non-valid virtual line
    uint8_t left_line_type = 0;
    uint8_t right_line_type = 0;
    uint32_t temp_line_id = 0;
    GetLineType(link_infos, left_lane_id, right_line_type, false, temp_line_id);
    GetLineType(link_infos, right_lane_id, left_line_type, true, temp_line_id);

    if (8 == left_line_type && 8 == right_line_type){
        uint32_t ego_bound_line_idx = 0, side_bound_line_idx = 0;
        std::vector<message::map_map::s_GeometryPoint_t> ego_bound_geometry_points_wgs{}, side_bound_geometry_points_wgs{};
        EFMRefLinePoints ego_bound_geometry_points_ego{}, side_bound_geometry_points_ego{};
        std::vector<double> ego_bounds{}, side_bounds{};
        int points_num = 0;
        double points_sl_sum = 0;
        if(is_left){
            GetLaneLeftLine(link_infos, right_lane_id, ego_bound_line_idx);
            GetLaneRightLine(link_infos, left_lane_id, side_bound_line_idx);
        }
        else{
            GetLaneRightLine(link_infos, left_lane_id, ego_bound_line_idx);
            GetLaneLeftLine(link_infos, right_lane_id, side_bound_line_idx);
        }
       
        GetLineGeometry(ego_bound_line_idx, ego_bound_geometry_points_wgs);
        GetLineGeometry(side_bound_line_idx, side_bound_geometry_points_wgs);
       
        auto map_position = *map_position_;
        CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBodyV2(
            ego_bound_geometry_points_wgs, ego_bound_geometry_points_ego, map_position, map_position.Heading.Heading);
        CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBodyV2(
            side_bound_geometry_points_wgs,side_bound_geometry_points_ego, map_position, map_position.Heading.Heading);

        SLPoint sl(0.0, 0.0);
        for (auto point : ego_bound_geometry_points_ego) {
            CommonMathMethod::DiscretePointsMath::CalPointSLBodyCoordinateV2(side_bound_geometry_points_ego, point, sl);
            side_bounds.push_back(sl.l);
        }

        // std::cout << "side_bounds.size(): " << side_bounds.size() << " {";
        // for(const auto& l:side_bounds){
        //     std::cout << l << ",";
        // }
        // std::cout << "}" << std::endl;
        // points_num = (ego_bounds.size() < side_bounds.size()) ? ego_bounds.size() : side_bounds.size();

        int side_bounds_size = side_bounds.size();
        for (int i = 0; i < side_bounds_size; i++) {
            points_sl_sum +=  side_bounds[i];
        }

        if (side_bounds_size > 0 && points_sl_sum > 0.1 * side_bounds_size) {
            virtual_line_case = 1;
        } else if (side_bounds_size > 0 && points_sl_sum  < 0.1 * side_bounds_size && ego_bound_line_idx == side_bound_line_idx) {
            virtual_line_case = 1;
        } else if (side_bounds_size > 0 && points_sl_sum   < 0.1 * side_bounds_size && ego_bound_line_idx != side_bound_line_idx) {
            virtual_line_case = 2;
        } else {
            virtual_line_case = 0;
        }
        // std::cout << "is_parallel_line: " << int(is_parallel_virtual_line) << std::endl;
    }

    return ;
}

bool ScenarioJudgeModel::GetLaneCenterLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                                       uint32_t& center_line_index) {
    center_line_index = 0;
    for (auto lane : link_infos.LaneInfos.LaneInfos) {
        if (lane.LaneNum.LaneNum == lane_num) {
            center_line_index = lane.Centeline.Centeline;
            break;
        }
    }
    if (center_line_index == 0) {
        return false;
    } else {
        return true;
    }
}

bool ScenarioJudgeModel::GetLaneRightLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                                      uint32_t& line_index) {
    line_index = 0;
    for (auto lane : link_infos.LaneInfos.LaneInfos) {
        if (lane.LaneNum.LaneNum == lane_num) {
            line_index = lane.RBound.RBound;
            break;
        }
    }
    if (line_index == 0) {
        return false;
    } else {
        return true;
    }
}

bool ScenarioJudgeModel::GetLaneLeftLine(const message::map_map::s_LinkInfo_t& link_infos, const uint8_t lane_num,
                                     uint32_t& line_index) {
    line_index = 0;
    for (auto lane : link_infos.LaneInfos.LaneInfos) {
        if (lane.LaneNum.LaneNum == lane_num) {
            line_index = lane.LBound.LBound;
            break;
        }
    }
    if (line_index == 0) {
        return false;
    } else {
        return true;
    }
}

bool ScenarioJudgeModel::GetLineGeometry(uint32_t line_id, std::vector<message::map_map::s_GeometryPoint_t>& geometry_points) {

    geometry_points.clear();
    if (0 == line_id){
        return false;
    }
    
    for (auto& lineinfo : map_static_info_->LinearObjects.LinearObjects) {
        if (line_id == lineinfo.IDLinearObject.IDLinearObject) {
            geometry_points = lineinfo.GeometryPoints.GeometryPoints;
            return true;
        }
    }

    if (geometry_points.empty()){
        return false;
    }
    
    return true;
}

bool ScenarioJudgeModel::IsVirtualLineMerge(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model){
    int ego_idx = candidate_lanes_model->ego_lane_prior_index();
    int left_idx = candidate_lanes_model->left_lane_prior_index();
    int right_idx = candidate_lanes_model->right_lane_prior_index();
    std::vector<uint32_t> merge_back_links_id = candidate_lanes_model->merge_back_links_id();
    std::vector<uint32_t> linklist_vec = candidate_lanes_model->link_id_vec();
    std::vector<uint8_t> merge_one_back_lanes_id = candidate_lanes_model->merge_ego_back_lanes_id();
    std::vector<uint8_t> merge_two_back_lanes_id = candidate_lanes_model->merge_side_back_lanes_id();
    LaneElementGroupSets lane_element_group_sets = candidate_lanes_model->get_lane_element_group_sets();
    last_merge_back_links_id_.clear();
    last_merge_ego_back_lanes_id_.clear();
    last_merge_side_back_lanes_id_.clear();
    // std::cout << "IsVirtualLineMerge: " << std::endl;
    // std::cout << "get_lane_element_group_sets.size(): " << int(lane_element_group_sets.size()) << std::endl;

    uint32_t merge_link_id = 0;
    uint8_t merge_ego_lane_num = 0;
    std::vector<uint8_t>merge_ego_back_lanes_id{}, merge_side_back_lanes_id{};
    for (size_t lane_idx = 0; lane_idx < lane_element_group_sets.size(); lane_idx++) {
        /*根据拼好的车道确认merge处自车的link以及lane_id*/
        for (size_t lane_dir_idx = 0; lane_dir_idx < lane_element_group_sets[lane_idx].size(); lane_dir_idx++) {
            auto& lane_element = lane_element_group_sets[lane_idx][lane_dir_idx];
            uint8_t merge_num = 0;
            if(lane_element.candidate_index == ego_idx){
                for(size_t lane_idx = 0; lane_idx < lane_element.lane_num_vec.size(); lane_idx++){
                    if(lane_element.lane_extra_infos[lane_idx].merge_value != 0 && lane_element.lane_extra_infos[lane_idx].e_offset < 50 * 100 && merge_num < 1){
                        merge_link_id = linklist_vec[lane_idx];
                        merge_ego_lane_num = lane_element.lane_num_vec[lane_idx];
                        merge_num++;
                    }
                }
            }
        }
    }
    // std::cout << __FILE__ << __LINE__ << "merge_link_id: " << merge_link_id << std::endl;
    // std::cout << __FILE__ << __LINE__ << "merge_ego_lane_num: " << int(merge_ego_lane_num) << std::endl;
    for(size_t link_idx = 0; link_idx < merge_back_links_id.size(); link_idx++){
        /*确认哪一个车道列表是自车道列表*/
        if(merge_back_links_id[link_idx] == merge_link_id){
            if(merge_ego_lane_num == merge_one_back_lanes_id[link_idx]){
                merge_ego_back_lanes_id = merge_one_back_lanes_id;
                merge_side_back_lanes_id = merge_two_back_lanes_id;
            }else if(merge_ego_lane_num == merge_two_back_lanes_id[link_idx]){
                merge_ego_back_lanes_id = merge_two_back_lanes_id;
                merge_side_back_lanes_id = merge_one_back_lanes_id;
            }
        }
    }

    if(false == merge_back_links_id.empty()){
        last_merge_back_links_id_ = merge_back_links_id;
        last_merge_ego_back_lanes_id_ = merge_ego_back_lanes_id;
        last_merge_side_back_lanes_id_ = merge_side_back_lanes_id;
    }

    // std::cout << __FILE__ << __LINE__ << "merge_back_links_id:[ " ;
    // for(auto& link_id : merge_back_links_id){
    //     std::cout << int(link_id) << ", ";
    // }
    // std::cout << "]" << std::endl;
    // std::cout << __FILE__ << __LINE__ << "merge_ego_back_lanes_id:[ " ;
    // for(auto& lane_num : merge_ego_back_lanes_id){
    //     std::cout << int(lane_num) << ", ";
    // }
    // std::cout << "]" << std::endl;

    return false;
}

void ScenarioJudgeModel::InitGlobalVar() {
    is_exit_start_dist_f_ = false;
    last_cycle_exit_end_dist_ = -255;
    tar_link_list_.clear();
    tar_link_max_endoffset_ = 0;
    tar_link_max_speed_ = 60;
}

void ScenarioJudgeModel::GetRoadClass(std::shared_ptr<const TopicTrait::MapMapMsg> map_msg, uint32_t link_id,uint8_t& road_class){
    road_class = 0;
        auto iter = std::find_if(
            map_msg->FormOfWays.FormOfWays.begin(), map_msg->FormOfWays.FormOfWays.end(),
            [&](const message::map_map::s_FormOfWay_t& it) { return it.InstanceId.InstanceId == link_id; });

        if (iter != map_msg->FormOfWays.FormOfWays.end()) {
            road_class = (*iter).FunctionRoadClass.data;
        }
}

bool ScenarioJudgeModel::MainRoadExitSceneByLink(const std::shared_ptr<CandidateLanesModel> candidate_lanes_model,
                                         int cur_rp_index) {
    bool find_exit_on_route_f = false;
    uint32_t ego_path_id = map_position_->PathId;
    double map_position_PathOffset = 0.0;
    std::vector<int> cur_lane_connectivity_indexs;
    road_split_end_link_id_ = 0;//end
    road_split_end_link_index_ = -1;//end
    road_split_lane_num_continue_ = 0;
    road_split_lane_num_split_ = 0;
    road_split_end_dist_ = 9999;
    road_split_start_dist_ = 9999;//一条主路分成两条主路
    road_split_start_lane_num_ =0;
    road_split_start_link_id_ = 0;
    main_split_two_main_ = false;

    for (int i = cur_rp_index_; i < (map_route_list_->LinkIds.LinkIds.size() - 1); i++) {
        find_exit_on_route_f = false;
        uint32_t cur_search_link = map_route_list_->LinkIds.LinkIds[i].LinkId;
        uint32_t next_link_id = map_route_list_->LinkIds.LinkIds[i + 1].LinkId;
        // std::vector<int> cur_lane_connectivity_indexs = link_id_index_lane_connect_map_->at(cur_search_link);
        if (link_id_index_lane_connect_map_->find(cur_search_link) != link_id_index_lane_connect_map_->end()) {
            cur_lane_connectivity_indexs = link_id_index_lane_connect_map_->at(cur_search_link);
        } else {
            break;
        }
        // if ToLinkId more than 1, means split
        int to_link_num = 0;
        int to_link_id = 0;
        // std::cout << i << ", cur_search_link: " << cur_search_link << std::endl;
        for (int cur_connect_index : cur_lane_connectivity_indexs) {
            if (cur_connect_index < map_static_info_->LaneConnectivitys.PairConnectivity.size()) {
                if (map_static_info_->LaneConnectivitys.PairConnectivity[cur_connect_index].ToLinkId.ToLink !=
                    to_link_id) {
                    to_link_num++;
                    to_link_id =
                        map_static_info_->LaneConnectivitys.PairConnectivity[cur_connect_index].ToLinkId.ToLink;
                    // std::cout << "EFM的debug信号-" << "to_link_id-" << to_link_id<<std::endl;
                    if (to_link_num > 1) {
                        // exist link split, get rightest lane type to judge ramp
                        // maybe next lane???
                        //  std::cout << "cur_search_link more to_link: " << cur_search_link << std::endl;
                        // int cur_link_index = link_id_index_lane_info_map_->at(cur_search_link);
                        int cur_link_index = 0;
                        if (link_id_index_lane_info_map_->find(cur_search_link) !=
                            link_id_index_lane_info_map_->end()) {
                            cur_link_index = link_id_index_lane_info_map_->at(cur_search_link);
                        } else {
                            // std::cout << "ExitSceneByLink 22222222" << std::endl;
                            break;
                        }
                        uint8_t righest_available_lane_num = 1;
                        if (!GetRightestAvailableLaneNum(cur_search_link, righest_available_lane_num)) {
                            // std::cout << "could not find righest_available_lane_num, ExitSceneByLink" << std::endl;
                            righest_available_lane_num = 1;
                        }
                        size_t cur_lane_size =
                            map_static_info_->LinkInfos.LinkInfos[cur_link_index].LaneInfos.LaneInfos.size();

                        for (int j = 0; j < cur_lane_size; j++) {

                            uint8_t lane_num_temp = map_static_info_->LinkInfos.LinkInfos[cur_link_index]
                                                        .LaneInfos.LaneInfos[j]
                                                        .LaneNum.LaneNum;
                            if (lane_num_temp >= righest_available_lane_num) {

                                std::vector<int> cur_lane_connect_index_vec;
                                if (link_id_index_lane_connect_map_->find(cur_search_link) !=
                                    link_id_index_lane_connect_map_->end()) {
                                    cur_lane_connect_index_vec = link_id_index_lane_connect_map_->at(cur_search_link);
                                } else {
                                    // std::cout << "ExitSceneByLink 333333333" << std::endl;
                                    break;
                                }
                                // std::cout << "get lane type 2 or 17" << std::endl;
                                for (int index : cur_lane_connect_index_vec) {
                                    // for all connect. pick rightest lane's connectivitys
                                    if (map_static_info_->LaneConnectivitys.PairConnectivity[index]
                                            .InitLaneNum.InitLaneNum == lane_num_temp) {
                                        int to_link_id_2 =
                                            map_static_info_->LaneConnectivitys.PairConnectivity[index].ToLinkId.ToLink;
                                        // std::cout << "to_link_id_2: " << to_link_id_2 << std::endl;
                                        uint8_t to_lane_num_2 =
                                            map_static_info_->LaneConnectivitys.PairConnectivity[index]
                                                .NewLaneNum.NewLaneNum;

                                        if (to_link_id_2 == next_link_id) {
                                            int to_link_index = 0;
                                            if (link_id_index_lane_info_map_->find(to_link_id_2) !=
                                                link_id_index_lane_info_map_->end()) {
                                                to_link_index = link_id_index_lane_info_map_->at(to_link_id_2);
                                            } else {
                                                // std::cout << "ExitSceneByLink 22222222" << std::endl;
                                                break;
                                            }
                                            for (auto lane : map_static_info_->LinkInfos.LinkInfos[to_link_index]
                                                                 .LaneInfos.LaneInfos) {
                                                if (lane.LaneNum.LaneNum == to_lane_num_2 &&
                                                    lane.LaneType.data == 0 ) {
                                                    find_exit_on_route_f = true;
                                                    break;
                                                }
                                            }
                                            if (find_exit_on_route_f == true) {
                                                break;
                                            }
                                        }
                                        // if there's one to_link's path id == ego path id, means to link is on route,
                                        // exist exit on route
                                        // int link_index = link_id_index_lane_info_map_->at(to_link_id_2);
                                        // if (map_static_info_->LinkInfos.LinkInfos[link_index].PathId.PathId ==
                                        //     ego_path_id) {
                                        //     find_exit_on_route_f = true;
                                        //     break;
                                        // }
                                    }
                                }
                                if (find_exit_on_route_f) {
                                    break;
                                }
                                // }
                            }
                        }
                        if (find_exit_on_route_f) {
                            road_split_end_link_id_ = cur_search_link;
                            road_split_end_link_index_ = i - cur_rp_index_;
                            road_split_end_dist_ =  static_cast<double>(map_static_info_->LinkInfos.LinkInfos[cur_link_index].EndOffset.EndOffset)/100.0 -
                                          static_cast<double>(map_position_->PathOffset) /100.0;
                            break;
                        }
                    }
                }
            }
        }

        // std::cout << "EFM的debug信号-" << "to_link_num-" << to_link_num<<std::endl;

        if (find_exit_on_route_f) {
            std::vector<uint32_t> link_id_vec = candidate_lanes_model->link_id_vec();
            std::map<uint32_t, SplitNode> candidate_lanes_split_nodes = candidate_lanes_model->candidate_lanes_split_nodes();
            std::vector<bool> is_contain_split_vec = candidate_lanes_model->is_contain_split_vec();
            std::vector<std::vector<uint8_t>> all_lanes_vec_vec = candidate_lanes_model->all_lanes_vec_vec();
#ifdef SCENARIOJUDGEMODEL_COUT
            std::cout << __FILE__ << "," << __LINE__ << ","
              << "road_split_end_link_index_  " << road_split_end_link_index_ << std::endl;
            std::cout << __FILE__ << "," << __LINE__ << ","
              << "link_id_vec.size()  " << link_id_vec.size() << std::endl;
            std::cout << __FILE__ << "," << __LINE__ << ","
              << "is_contain_split_vec.size() " << is_contain_split_vec.size() << std::endl;
#endif

            main_split_two_main_ = false;
    // bool get_exit_start_point = false;
            for (int j = road_split_end_link_index_ - 1; j >= 0 && j < link_id_vec.size() && j < is_contain_split_vec.size(); j--) {
                uint32_t link_id = link_id_vec[j];
                bool to_main_road_continue= false;// road split直行的路
                bool to_main_road_split = false;//road split 下去的路
                uint8_t origin_lane_num = 0;
                if (is_contain_split_vec[j] == true) {
                    if (candidate_lanes_split_nodes.find(link_id) != candidate_lanes_split_nodes.end()) {
                       origin_lane_num = candidate_lanes_split_nodes[link_id].origin_node.lane_num;
                       for (auto split_node : candidate_lanes_split_nodes[link_id].split_nodes) {
                    // j+1 is split, j is origin
                            auto iter = std::find_if(all_lanes_vec_vec.begin(), all_lanes_vec_vec.end(),
                                             [&](const std::vector<uint8_t>& it) {
                                                 if (it.size() > (j + 1)) {
                                                     if (it[j] == origin_lane_num && it[j + 1] == split_node.lane_num) {
                                                         return true;
                                                     }
                                                 }
                                                 return false;
                                             });
                            if (iter != all_lanes_vec_vec.end()) {
                                if ((*iter).size() < road_split_end_link_index_ + 1) {
                                     // not this lane
                                } else {
                                    uint8_t end_lane_num = (*iter)[road_split_end_link_index_];
                                    if(link_id_index_lane_info_map_->find(road_split_end_link_id_)== link_id_index_lane_info_map_->end()){
                                       return false;
                                    }
                                    int link_index = link_id_index_lane_info_map_->at(road_split_end_link_id_);
                                    for (auto lane : map_static_info_->LinkInfos.LinkInfos[link_index].LaneInfos.LaneInfos) {
                                        if (lane.LaneNum.LaneNum == end_lane_num) {
            //       std::cout << __FILE__ << "," << __LINE__ << ","
            //   << "end_lane_num: " << (int)end_lane_num <<" ,road_split_end_link_id_: "<<road_split_end_link_id_ <<" ,lane.LaneType.data: " <<(int)lane.LaneType.data<<std::endl;                                 
                                            if (lane.LaneType.data == 0) {
                                        // continue to check whether the next is on route, main road may not in route on
                                        // road split
                                                if ((*iter).size() == road_split_end_link_index_ + 1) {
                                                    to_main_road_continue = true;
                                                    road_split_lane_num_continue_ = end_lane_num;
                                                }else if((*iter).size() > road_split_end_link_index_ + 1){
                                                    to_main_road_split = true;
                                                    road_split_lane_num_split_ = end_lane_num;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                if(to_main_road_continue && to_main_road_split && main_split_two_main_ == false){
                    main_split_two_main_ = true;
                    if(link_id_index_lane_info_map_->find(link_id) != link_id_index_lane_info_map_->end()){
                        int link_index = link_id_index_lane_info_map_->at(link_id);
                        road_split_start_dist_ = static_cast<double>(map_static_info_->LinkInfos.LinkInfos[link_index].EndOffset.EndOffset)/100.0 - static_cast<double>(map_position_->PathOffset) /100.0; 
                        road_split_start_lane_num_ =  origin_lane_num;  
                        road_split_start_link_id_ = link_id;
                    }
             
                }
                if (main_split_two_main_) {
                    break;
                }
            }

#ifdef SCENARIOJUDGEMODEL_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "map_position_->PathOffset: " << map_position_->PathOffset << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "road_split_end_link_id_: " << road_split_end_link_id_ << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "road_split_end_link_index_: " << road_split_end_link_index_ << std::endl;
              
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "road_split_end_dist_: " << road_split_end_dist_ << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "main_split_two_main_: " <<(int) main_split_two_main_ << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "road_split_start_dist_: " << road_split_start_dist_ << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "road_split_start_lane_num_: " << (int)road_split_start_lane_num_ << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "road_split_start_link_id_: " << road_split_start_link_id_ << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "road_split_lane_num_continue_: " << (int)road_split_lane_num_continue_ << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << "road_split_lane_num_split_: " << (int)road_split_lane_num_split_ << std::endl;
#endif
        }
    }

    return true;
}

}  // namespace framework
}  // namespace shell
}  // namespace earth
