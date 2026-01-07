#include "efm_output.h"
#ifdef __QNX__
#include "gptp/gptp.h"
#endif
namespace efm {
EfmOutput* EfmOutput::GetInstance() {
    static EfmOutput instance;
    return &instance;
}

bool EfmOutput::MakeOutput(const MapMapMsg& map_msg, const MapRouteListMsg& map_route_list,
                           const MapSwitchInfoMsg& switch_info_msg, MapPositionTmp& map_position_tmp,
                           const MapRawData map_raw_data, const MapRawDataMap& map_raw_data_map,
                           const SubModuleOutput& sub_module_output, EfmInfoMsg& efm_info, MapLaneMsg& map_lane,
                           MapLppInfoMsg& map_lpp_info, RMFMapinfoDataclose& rmf_dc, uint64_t& errorcode) {
    OutputMapGlobalLine(map_msg, switch_info_msg, map_position_tmp, map_raw_data, map_lpp_info);

    if (false == OutputKerbLine(map_msg, map_route_list, map_position_tmp, sub_module_output, map_raw_data,
                                map_raw_data_map, map_lane)) {
        return false;
    }

    if (false == MakeOutputMapLppInfo(map_msg, switch_info_msg, map_raw_data_map, sub_module_output, map_position_tmp,
                                      map_lpp_info, map_lane)) {
        return false;
    }

    if (false == MakeOutputMapEfmInfo(sub_module_output, efm_info)) {
        return false;
    }

    if (false == MakeOutputMapLane(map_position_tmp, sub_module_output, efm_info, map_raw_data, map_lane)) {
        return false;
    }

    map_lpp_info.NodeInfo = sub_module_output.node_info;

#ifdef __QNX__
    auto now = bsw::gptp_clock::now();

    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
    auto nanos_after_seconds = nanos - seconds;

    uint64_t sec = static_cast<uint64_t>(seconds.count());
    uint64_t nsec = static_cast<uint64_t>(nanos_after_seconds.count());
    map_lpp_info.header.timestamp = sec;
    map_lane.header.timestamp = sec;
    efm_info.header.timestamp = sec;
#endif

    MakeOutputZTEXT(efm_info, map_lane, map_lpp_info, rmf_dc);

    if (map_lpp_info.NodeInfo.EndPointOffset < 0) {
        errorcode = errorcode | EFM_CODE_EFM_NARROW_LANE_CLOSE_ERROR;
    }

    return true;
}

bool EfmOutput::OutputMapGlobalLine(const MapMapMsg& map_msg, const MapSwitchInfoMsg& switch_info_msg,
                                    MapPositionTmp& map_position_tmp, const MapRawData map_raw_data,
                                    MapLppInfoMsg& map_lpp_info) {
    std::vector<double> plot_x{}, plot_y{};  // for plot
    std::vector<std::pair<double, double>> global_lines_points{};

    if (-1 == map_raw_data.ego_linkid_routelist_idx) {
        return false;
    }
    for (int link_idx = map_raw_data.ego_linkid_routelist_idx;
         link_idx < switch_info_msg.NOAInfo.PairOfIds.PairOfIds.size(); link_idx++) {
        uint32_t cur_search_linear_id = switch_info_msg.NOAInfo.PairOfIds.PairOfIds[link_idx].LinearObjectId;
        std::vector<message::map_map::s_GeometryPoint_t> global_line_point_wgs{};
        EFMRefLinePoints global_line_point_utm{}, global_line_point_ego{};
        if (false ==
            MapCommonTool::GetInstance()->GetLineGeometry(map_msg, cur_search_linear_id, global_line_point_wgs)) {
            return false;
        }
        // CommonTool::CoordinateTool::GetInstance()->LineWGS2UTM2(global_line_point_wgs, global_line_point_utm,
        // map_position_); CommonTool::CoordinateTool::GetInstance()->LineUTM2EgoVehicle(global_line_point_utm,
        // ego_utm_x_, ego_utm_y_, map_position_tmp.Heading.Heading, global_line_point_ego);
        auto map_position_tmp_ptr = std::make_shared<const efm::MapPositionTmp>(map_position_tmp);
        CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBody(
            global_line_point_wgs, global_line_point_ego, map_position_tmp_ptr,
            static_cast<double>(map_position_tmp.Heading.Heading));
        if (global_lines_points.size() > 200) {
            break;
        }
        for (const auto& point : global_line_point_ego) {
            global_lines_points.push_back({point.x, point.y});
        }
    }
    uint8_t point_num = 0;
    bool plot_done = false;       // for plot
    bool find_ego_point = false;  // for plot
    double last_point_x = 10000;  // for plot
    double last_point_y = 10000;  // for plot
    double dist = 0;              // for plot
    for (const auto& point : global_lines_points) {
        if (point_num < 120) {
            map_lpp_info.MapRefLine.PriorRefLine[point_num] = {static_cast<float>(point.first),
                                                               static_cast<float>(point.second), 0.0f};
            map_lpp_info.MapRefLine.pntSize = point_num;
            //##plot#####
            if (find_ego_point == false && (last_point_x < 0 && point.first >= 0)) {
                // plot_x.push_back(point.first);
                // plot_y.push_back(point.second);
                find_ego_point = true;
                dist += sqrt(pow(point.first, 2) + pow(point.second, 2));
            } else {
                if (plot_done == true) {
                    // do nothing
                } else {
                    if (dist < 150) {
                        dist += sqrt(pow(point.first - last_point_x, 2) + pow(point.second - last_point_y, 2));
                        // plot_x.push_back(point.first);
                        // plot_y.push_back(point.second);
                    } else {
                        plot_done = true;
                    }
                }
            }
            //#########
            point_num++;
            last_point_x = point.first;   // plot
            last_point_y = point.second;  // plot
        } else {
            break;
        }
    }

    // std::cout << "plot_x.size(): " << int(plot_x.size()) << std::endl;
    // ZPLOTXYF("EFM_INFO", ".green6", plot_x, plot_y);
    return true;
}

bool EfmOutput::OutputKerbLine(const MapMapMsg& map_msg, const MapRouteListMsg& map_route_list,
                               MapPositionTmp& map_position_tmp, const SubModuleOutput& sub_module_output,
                               const MapRawData map_raw_data, const MapRawDataMap& map_raw_data_map,
                               MapLaneMsg& map_lane) {
    EFMRefLinePoints center_line_points_ego{};
    std::vector<uint32_t> link_id_vec = {};
    EFMRefLinePoints kerb_line_left_points_ego{};
    EFMRefLinePoints kerb_line_right_points_ego{};
    kerb_line_left_points_ego.clear();
    kerb_line_right_points_ego.clear();

    center_line_points_ego = sub_module_output.ego_path.linePoints;

    // TODO link_id_vec做成共有变量
    for (int map_route_list_linkid_index = map_raw_data.ego_linkid_routelist_idx;
         map_route_list_linkid_index < map_route_list.LinkIds.LinkIds.size(); map_route_list_linkid_index++) {
        message::map_map::s_LinkInfo_t cur_search_link_infos;
        // TODO GetLinkInfos写成公有函数
        efm::MapCommonTool::GetInstance()->GetLinkInfos(
            map_msg, map_raw_data_map.link_id_index_lane_info_map,
            map_route_list.LinkIds.LinkIds[map_route_list_linkid_index].LinkId, cur_search_link_infos);
        if (cur_search_link_infos.PathOffset.PathOffset > map_position_tmp.PathOffset + 150 * 100) {
            break;
        }

        for (auto& linearobject : map_msg.LinearObjects.LinearObjects) {
            EFMRefLinePoints kerb_line_left_points_utm{};
            EFMRefLinePoints kerb_line_right_points_utm{};
            EFMPoint tmp_point{0.0, 0.0};
            kerb_line_left_points_utm.clear();
            kerb_line_right_points_utm.clear();
            if ((linearobject.InstanceId.InstanceId ==
                 map_route_list.LinkIds.LinkIds[map_route_list_linkid_index].LinkId) &&
                (linearobject.LinearObjectType.data == 3 || linearobject.LinearObjectType.data == 2 ||
                 linearobject.LinearObjectType.data == 4 || linearobject.LinearObjectType.data == 6)) {
                if (cur_search_link_infos.TotalNumLane.TotalNumLane > 1) {
                    if (linearobject.LaneNum.LaneNum == 1) {
                        CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBodyV2(
                            linearobject.GeometryPoints.GeometryPoints, kerb_line_right_points_ego, map_position_tmp,
                            static_cast<double>(map_position_tmp.Heading.Heading));
                    } else {
                        CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBodyV2(
                            linearobject.GeometryPoints.GeometryPoints, kerb_line_left_points_ego, map_position_tmp,
                            static_cast<double>(map_position_tmp.Heading.Heading));
                    }
                } else {
                    if (linearobject.LaneNum.LaneNum == 1 &&
                        false == linearobject.GeometryPoints.GeometryPoints.empty()) {
                        auto first_point = linearobject.GeometryPoints.GeometryPoints[0];
                        CommonTool::CoordinateTool::GetInstance()->WGS84ToBody(
                            first_point.Longitude.Longitude, first_point.Latitude.Latitude, map_position_tmp.Lon.Lon,
                            map_position_tmp.Lat.Lat, map_position_tmp.Heading.Heading, tmp_point.x, tmp_point.y);

                        // Find the nearest point on center_line_points_ego to tmp_point
                        double min_distance = std::numeric_limits<double>::max();
                        int nearest_point_index = -1;
                        for (int i = 0; i < center_line_points_ego.size(); i++) {
                            double distance = std::hypot(center_line_points_ego[i].x - tmp_point.x,
                                                         center_line_points_ego[i].y - tmp_point.y);
                            if (distance < min_distance) {
                                min_distance = distance;
                                nearest_point_index = i;
                            }
                        }

                        // Determine if tmp_point is on the left or right side of center_line_points_ego
                        if (nearest_point_index != -1) {
                            // double x_diff = center_line_points_ego[nearest_point_index].x - tmp_point.x;
                            double y_diff = tmp_point.y - center_line_points_ego[nearest_point_index].y;
                            if (y_diff < 0) {
                                CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBodyV2(
                                    linearobject.GeometryPoints.GeometryPoints, kerb_line_right_points_ego,
                                    map_position_tmp, static_cast<double>(map_position_tmp.Heading.Heading));
                            } else {
                                CommonTool::CoordinateTool::GetInstance()->LineWGS84ToBodyV2(
                                    linearobject.GeometryPoints.GeometryPoints, kerb_line_left_points_ego,
                                    map_position_tmp, static_cast<double>(map_position_tmp.Heading.Heading));
                            }
                        }
                    }
                }
            }
        }
    }
    std::sort(kerb_line_left_points_ego.begin(), kerb_line_left_points_ego.end(),
              [](const EFMPoint& a, const EFMPoint& b) { return a.x < b.x; });
    std::sort(kerb_line_right_points_ego.begin(), kerb_line_right_points_ego.end(),
              [](const EFMPoint& a, const EFMPoint& b) { return a.x < b.x; });

    kerb_line_left_points_ego.erase(
        std::remove_if(
            kerb_line_left_points_ego.begin(), kerb_line_left_points_ego.end(),
            [](const EFMPoint& point) { return point.x < p_kerb_back_distance || point.x > p_kerb_forward_distance; }),
        kerb_line_left_points_ego.end());
    kerb_line_right_points_ego.erase(
        std::remove_if(
            kerb_line_right_points_ego.begin(), kerb_line_right_points_ego.end(),
            [](const EFMPoint& point) { return point.x < p_kerb_back_distance || point.x > p_kerb_forward_distance; }),
        kerb_line_right_points_ego.end());

    // std::cout << "kerb_line_left_points_egox.size(): [" << kerb_line_left_points_ego.size() << std::endl;
    // for (int i = 0; i < kerb_line_left_points_ego.size(); i++) {
    //     std::cout << kerb_line_left_points_ego[i].x << ", ";
    // }
    // std::cout << "]" << std::endl;
    // std::cout << "kerb_line_left_points_egoy.size(): [" << kerb_line_left_points_ego.size() << std::endl;
    // for (int i = 0; i < kerb_line_left_points_ego.size(); i++) {
    //     std::cout << kerb_line_left_points_ego[i].y << ", ";
    // }
    // std::cout << "]" << std::endl;

    for (int i = 0; i < kerb_line_left_points_ego.size(); i++) {
        map_lane.CurbInfo[1].linePoints.push_back({kerb_line_left_points_ego[i].x, kerb_line_left_points_ego[i].y});
    }

    // std::cout << "kerb_line_right_points_ego.sizex(): [" << kerb_line_right_points_ego.size() << std::endl;
    // for (int i = 0; i < kerb_line_right_points_ego.size(); i++) {
    //     std::cout << kerb_line_right_points_ego[i].x << ", ";
    // }
    // std::cout << "]" << std::endl;
    // std::cout << "kerb_line_right_points_ego.sizey(): [" << kerb_line_right_points_ego.size() << std::endl;
    // for (int i = 0; i < kerb_line_right_points_ego.size(); i++) {
    //     std::cout << kerb_line_right_points_ego[i].y << ", ";
    // }
    // std::cout << "]" << std::endl;

    for (int i = 0; i < kerb_line_right_points_ego.size(); i++) {
        map_lane.CurbInfo[0].linePoints.push_back({kerb_line_right_points_ego[i].x, kerb_line_right_points_ego[i].y});
    }

    return true;
}

bool EfmOutput::MakeOutputMapLppInfo(const MapMapMsg& map_msg, const MapSwitchInfoMsg& switch_info_msg,
                                     const MapRawDataMap& map_raw_data_map, const SubModuleOutput& sub_module_output,
                                     MapPositionTmp& map_position_tmp, MapLppInfoMsg& map_lpp_info,
                                     MapLaneMsg& map_lane) {
    MakeOutputMapLppInfoSwitch(switch_info_msg, sub_module_output, map_lpp_info);
    MakeOutputMapLppInfoOdd(sub_module_output, map_position_tmp, map_lpp_info);
    MakeOutputMapLppInfoRefPath(map_msg, sub_module_output, map_raw_data_map, map_lane, map_lpp_info);
    MakeOutputMapLppInfoOther(map_msg, map_position_tmp, sub_module_output, map_lpp_info);
    // std::cout << __FILE__ << __LINE__ << "odd: " << int(map_lpp_info.mMapOdd.eInOdd.data_) << std::endl;
    return true;
}

bool EfmOutput::MakeOutputMapLppInfoSwitch(const MapSwitchInfoMsg& switch_info_msg,
                                           const SubModuleOutput& sub_module_output, MapLppInfoMsg& map_lpp_info) {
    bool is_y_shape_link = sub_module_output.is_y_shape_link;
    // std::cout << "is_y_shape_link: " << int(is_y_shape_link) << std::endl;
    if (is_y_shape_link) {
        map_lpp_info.SwitchLaneReason.data_ = 6;
        // std::cout << "map_lpp_info.SwitchLaneReason.data_: " << int(map_lpp_info.SwitchLaneReason.data_) <<
        // std::endl;
    }
    // std::cout << "dDistanceByPassMerge: "  << int(scenario_judge_model_->dDistanceByPassMerge()) << std::endl;
    // std::cout << "sub_module_output.fixed_lane_id: " << int(sub_module_output.fixed_lane_id) <<
    // std::endl;
    map_lpp_info.mRefPaths[1].dNodeOffset = sub_module_output.bypass_merge_distance;
    map_lpp_info.dDistanceByPassMerge = 5000.0;
    // std::cout << "sub_module_output.fixed_lane_id: " << int(sub_module_output.fixed_lane_id) <<
    // std::endl;
    if (sub_module_output.fixed_lane_id == 1) {
        map_lpp_info.dDistanceByPassMerge = sub_module_output.bypass_merge_distance;
    }

    if (switch_info_msg.NOAInfo.NavigationStatus.data == 2) {
        map_lpp_info.mNavInfo.bIsSetNav = true;
    } else if (switch_info_msg.NOAInfo.NavigationStatus.data == 4 ||
               switch_info_msg.NOAInfo.NavigationStatus.data == 5) {
        map_lpp_info.mNavInfo.bNavIsYaw = true;
    }

    return true;
}

bool EfmOutput::MakeOutputMapLppInfoOdd(const SubModuleOutput& sub_module_output, MapPositionTmp& map_position_tmp,
                                        MapLppInfoMsg& map_lpp_info) {
    // odd
    // odd.tyoe 0: None; 1:TollBooth; 2:ServiceArea_ 3: bigcurv
    std::vector<OddDis> Odds{};
    OddDis odd{};
    odd.odddis = sub_module_output.odd_end_dist;
    odd.oddtype = sub_module_output.in_odd_type;
    Odds.push_back(odd);
    bool is_has_big_curvature = sub_module_output.is_has_big_curvature;
    double exit_end_dist = sub_module_output.exit_end_dist;
    ZTEXT("EFM_INFO", "has_big_curvature: ", -28, -30, "has_big_curvature: {}", is_has_big_curvature);

    std::vector<GeoFenceSegment> geofence_segments;
    std::vector<GeoFenceSegment> valid_geofence_segments;
    geofence_segments = sub_module_output.geofence_segments;
    valid_geofence_segments = sub_module_output.valid_geofence_segments;

    if (valid_geofence_segments.empty()) {
        map_lpp_info.mSpecialSit.dStartDistance = 5000;
        map_lpp_info.mSpecialSit.eSpecSitType.data_ = 0;
        map_lpp_info.mSpecialSit.dEndDistance = 5000;

    } else {
        bool isLinkIDPresent = false;
        for (const auto& linkid : valid_geofence_segments[0].linkids) {
            if (linkid == map_position_tmp.LinkId) {
                isLinkIDPresent = true;
                break;
            }
        }
        if (isLinkIDPresent) {
            if (valid_geofence_segments[0].geo_fence_type != 9) {
                map_lpp_info.mSpecialSit.dStartDistance = 0;
                map_lpp_info.mSpecialSit.dEndDistance =
                    (valid_geofence_segments[0].last_endoffset - map_position_tmp.PathOffset) / 100;
            }
            if (valid_geofence_segments[0].geo_fence_type == 3) {
                map_lpp_info.mSpecialSit.eSpecSitType.data_ = 3;
            } else if (valid_geofence_segments[0].geo_fence_type == 4) {
                map_lpp_info.mSpecialSit.eSpecSitType.data_ = 1;
            } else if (valid_geofence_segments[0].geo_fence_type == 22) {
                map_lpp_info.mSpecialSit.eSpecSitType.data_ = 2;
            } else {
                map_lpp_info.mSpecialSit.eSpecSitType.data_ = 0;
            }
        } else {
            if (valid_geofence_segments[0].geo_fence_type != 9) {
                map_lpp_info.mSpecialSit.dStartDistance =
                    (valid_geofence_segments[0].first_pathoffset - map_position_tmp.PathOffset) / 100;
                map_lpp_info.mSpecialSit.dEndDistance =
                    (valid_geofence_segments[0].last_endoffset - map_position_tmp.PathOffset) / 100;
            }
            if (valid_geofence_segments[0].geo_fence_type == 3) {
                map_lpp_info.mSpecialSit.eSpecSitType.data_ = 3;
            } else if (valid_geofence_segments[0].geo_fence_type == 4) {
                map_lpp_info.mSpecialSit.eSpecSitType.data_ = 1;
            } else if (valid_geofence_segments[0].geo_fence_type == 22) {
                map_lpp_info.mSpecialSit.eSpecSitType.data_ = 2;
            } else {
                map_lpp_info.mSpecialSit.eSpecSitType.data_ = 0;
            }
        }
    }
    // TollBooth_ = 1; ServiceArea_ = 2;
    if (map_lpp_info.mSpecialSit.eSpecSitType.data_ == 1) {
        OddDis oddtoll{};
        oddtoll.odddis = (map_lpp_info.mSpecialSit.dStartDistance - p_toll_odd_to0_distance) >= 0
                             ? static_cast<uint32_t>(map_lpp_info.mSpecialSit.dStartDistance) -
                                   static_cast<uint32_t>(p_toll_odd_to0_distance)
                             : 0;
        oddtoll.oddtype = 1;
        Odds.push_back(oddtoll);
    } else if (map_lpp_info.mSpecialSit.eSpecSitType.data_ == 2) {
        OddDis oddserv{};
        oddserv.odddis = static_cast<uint32_t>(map_lpp_info.mSpecialSit.dStartDistance);
        oddserv.oddtype = 1;
        Odds.push_back(oddserv);
    }
    // std::cout << "is_has_big_curvature: " << int(is_has_big_curvature) << std::endl;
    // std::cout << "exit_end_dist: " << int(exit_end_dist) << std::endl;
    if (true == is_has_big_curvature && exit_end_dist > p_bigcurve_oddto0_distance &&
        exit_end_dist < p_bigcurve_oddto2_distance) {
        OddDis oddbigcurv{};
        oddbigcurv.odddis =
            (exit_end_dist - p_bigcurve_oddto0_distance) > 0
                ? static_cast<uint32_t>(exit_end_dist) - static_cast<uint32_t>(p_bigcurve_oddto0_distance)
                : 0;
        oddbigcurv.oddtype = 3;
        Odds.push_back(oddbigcurv);
        // map_lpp_info.mMapOdd.eInOdd.data_ = 2;
        // map_lpp_info.mMapOdd.dDisToOdd =
        //     (exit_end_dist - p_bigcurve_oddto0_distance) > 0
        //         ? static_cast<uint32_t>(exit_end_dist) - static_cast<uint32_t>(p_bigcurve_oddto0_distance)
        //         : 0;
        // map_lpp_info.mMapOdd.eOddReason.data_ = 1;

    } else if (true == is_has_big_curvature) {
        OddDis oddbigcurv2{};
        oddbigcurv2.odddis = 0;
        oddbigcurv2.oddtype = 3;
        Odds.push_back(oddbigcurv2);
    }
    if (false == Odds.empty()) {
        std::sort(Odds.begin(), Odds.end(), compareOddDis);
        map_lpp_info.mMapOdd.dDisToOdd = Odds[0].odddis;

        if (Odds[0].odddis > p_odd_to2_distance - 100) {
            map_lpp_info.mMapOdd.eInOdd.data_ = 1;
            map_lpp_info.mMapOdd.eOddReason.data_ = 0;
        } else if (Odds[0].odddis > 0) {
            if (Odds[0].oddtype == 1) {
                map_lpp_info.mMapOdd.eInOdd.data_ = 2;
                map_lpp_info.mMapOdd.eOddReason.data_ = 2;
            } else {
                map_lpp_info.mMapOdd.eInOdd.data_ = 2;
                map_lpp_info.mMapOdd.eOddReason.data_ = 0;
            }

        } else {
            if (Odds[0].oddtype == 1) {
                map_lpp_info.mMapOdd.eInOdd.data_ = 0;
                map_lpp_info.mMapOdd.eOddReason.data_ = 2;
            } else {
                map_lpp_info.mMapOdd.eInOdd.data_ = 0;
                map_lpp_info.mMapOdd.eOddReason.data_ = 1;
            }
        }
    }

    return true;
}

bool EfmOutput::MakeOutputMapLppInfoRefPath(const MapMapMsg& map_msg, const SubModuleOutput& sub_module_output,
                                            const MapRawDataMap& map_raw_data_map, MapLaneMsg& map_lane,
                                            MapLppInfoMsg& map_lpp_info) {
    {  // entry exit
        auto node = sub_module_output.node_info;
        if (node.LaneChgType & (1 << 3)) {
            map_lpp_info.mRampInfo.OnRampStartPoint.dX = node.StartPointOffset;
            map_lpp_info.mRampInfo.OnRampEndPoint.dX = node.EndPointOffset;
        }
        // map_lpp_info.mRampInfo.OnRampStartPoint.dX = sub_module_output.entry_start_dist;
        // map_lpp_info.mRampInfo.OnRampEndPoint.dX = -255;
        if (sub_module_output.entry_end_dist > 0 &&
            ((sub_module_output.ego_path.merge_type == 1 && sub_module_output.ego_path.merge_dist > 0) ||
             (sub_module_output.left_path.merge_type == 4 && sub_module_output.left_path.merge_dist > 0))) {
            map_lpp_info.mRefPaths[5].dNodeOffset = sub_module_output.entry_end_dist;
            map_lpp_info.mRefPaths[5].bProbabilityLeft = true;

        } else if (sub_module_output.entry_start_dist > 0 && sub_module_output.entry_start_dist < 2000) {
            map_lpp_info.mRefPaths[5].dNodeOffset = sub_module_output.entry_start_dist;
            map_lpp_info.mRefPaths[5].bProbabilityLeft = true;
        }
    }

    {  // exit
        bool is_continue = false;
        bool is_split = false;
        JudgeExitDirection(map_msg, sub_module_output, map_raw_data_map, is_continue, is_split);

        int prior_index = sub_module_output.prior_path_index;  // 0-ego;1-left;2-right
        if ((sub_module_output.exit_end_dist > 0 && sub_module_output.exit_end_dist < 2000)) {
            map_lpp_info.mRefPaths[4].dNodeOffset = sub_module_output.exit_end_dist;
            map_lpp_info.mRefPaths[4].bProbabilityRight = true;
        }
        if (sub_module_output.exit_start_dist < 0 && sub_module_output.exit_end_dist > 0 && prior_index == 0) {
            map_lpp_info.mRefPaths[4].dNodeOffset = sub_module_output.exit_end_dist;
            map_lpp_info.mRefPaths[4].bProbabilityRight = false;
        }
        if (prior_index == 0 && is_continue == true) {
            map_lpp_info.mRefPaths[4].dNodeOffset = sub_module_output.exit_end_dist;
            map_lpp_info.mRefPaths[4].bProbabilityRight = false;
        }
        // if (prior_index == 0 && is_split == true) {
        //     prior_index = 2;
        // }
        double end = sub_module_output.entry_end_dist;
        double start = sub_module_output.entry_start_dist;
        double start_out = map_lpp_info.mRampInfo.OnRampStartPoint.dX;
        double end_out = map_lpp_info.mRampInfo.OnRampEndPoint.dX;

        // ZTEXT("EFM_INFO", "raw_entry_dist_: ", 80, 27, "r_entry_start_dist_: {}", start);
        // ZTEXT("EFM_INFO", "r_entry_end_dist_: ", 80, 24, "r_entry_end_dist_: {}", end);
        ZTEXT("EFM_INFO", "entry_dist_: ", 80, 54, "entry_start_dist_: {}", start_out);
        ZTEXT("EFM_INFO", "entry_end_dist_: ", 80, 57, "entry_end_dist_: {}", end_out);
#ifdef EM_COUT
        std::cout << __FILE__ << "," << __LINE__ << ","
                  << "is_continue: " << (int)is_continue << " ,is_split: " << (int)is_split << std::endl;
#endif
        map_lpp_info.priorIndex = prior_index;
    }

    int prior_index = sub_module_output.prior_path_index;  // 0-ego;1-left;2-right
    {                                                      // //  out put global refpath
        // EFMRefLine ref_path;
        message::efm::s_PriorRefLine_t& global_line_points = map_lpp_info.priorRefLine;
        int prior_index = sub_module_output.prior_path_index;  // 0-ego;1-left;2-right
        int point_num = 0;
        switch (prior_index) {
            case 1:
                if (map_lane.lanes[1].enable_flag == true) {
                    for (int i = 0;
                         i < map_lane.lanes[1].center_line.pntSize && i < global_line_points.PriorRefLine.size(); i++) {
                        global_line_points.PriorRefLine[i] = map_lane.lanes[1].center_line.linePnt[i];
                        point_num++;
                    }
                }

                break;
            case 2:
                if (map_lane.lanes[3].enable_flag == true) {
                    for (int i = 0;
                         i < map_lane.lanes[3].center_line.pntSize && i < global_line_points.PriorRefLine.size(); i++) {
                        global_line_points.PriorRefLine[i] = map_lane.lanes[3].center_line.linePnt[i];
                        point_num++;
                    }
                }
                break;
            case 3:
                if (map_lane.lanes[0].enable_flag == true) {
                    for (int i = 0;
                         i < map_lane.lanes[0].center_line.pntSize && i < global_line_points.PriorRefLine.size(); i++) {
                        global_line_points.PriorRefLine[i] = map_lane.lanes[0].center_line.linePnt[i];
                        point_num++;
                    }
                }
                break;
            case 4:
            // std::cout << __FILE__ << "," << __LINE__ << " prior_index: " << prior_index << std::endl;
                if (map_lane.lanes[4].enable_flag == true) {
                    for (int i = 0;
                         i < map_lane.lanes[4].center_line.pntSize && i < global_line_points.PriorRefLine.size(); i++) {
                        global_line_points.PriorRefLine[i] = map_lane.lanes[4].center_line.linePnt[i];
                        point_num++;
                    }
                }
                break;
            default:
                if (map_lane.lanes[2].enable_flag == true) {
                    for (int i = 0;
                         i < map_lane.lanes[2].center_line.pntSize && i < global_line_points.PriorRefLine.size(); i++) {
                        global_line_points.PriorRefLine[i] = map_lane.lanes[2].center_line.linePnt[i];
                        point_num++;
                    }
                }
                break;
        }
        global_line_points.pntSize = point_num;
    }

    {  // deal node [0]
        if (sub_module_output.prior_path_change_type == LaneChangeType_e::MERGE ||
            sub_module_output.prior_path_change_type == LaneChangeType_e::LANE_END) {
            // 0-ego;1-left;2-right
            if (prior_index == 1) {
                map_lpp_info.mRefPaths[0].dNodeOffset = sub_module_output.rest_dist;
                map_lpp_info.mRefPaths[0].bProbabilityLeft = true;
                map_lpp_info.mRefPaths[0].bProbabilityRight = false;
            } else if (prior_index == 2) {
                map_lpp_info.mRefPaths[0].dNodeOffset = sub_module_output.rest_dist;
                map_lpp_info.mRefPaths[0].bProbabilityLeft = false;
                map_lpp_info.mRefPaths[0].bProbabilityRight = true;
            }
        }
    }

    {
        bool is_has_virtual_lane = sub_module_output.is_has_virtual_lane;
        if (is_has_virtual_lane) {
            map_lpp_info.mRefPaths[2].bProbabilityLeft = 1;
        }
        // std::cout << "bProbabilityLeft: " << map_lpp_info.mRefPaths[2].bProbabilityLeft << std::endl;
    }
    return true;
}

bool EfmOutput::JudgeExitDirection(const MapMapMsg& map_msg, const SubModuleOutput& sub_module_output,
                                   const MapRawDataMap& map_raw_data_map, bool& is_continue, bool& is_split) {
    is_continue = false;
    is_split = false;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << " sub_module_output.exit_start_link_index: " << sub_module_output.exit_start_link_index
    //           << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << " sub_module_output.prior_path_index: " << sub_module_output.prior_path_index << std::endl;
    if (sub_module_output.exit_start_link_index >= 0 && sub_module_output.prior_path_index == 0) {
        // just judge ego car before offramp split
        std::vector<uint32_t> link_id_vec = sub_module_output.link_id_vec;
        std::vector<uint8_t> ego_path_lane_num = sub_module_output.ego_path_lane_num;
        std::map<uint32_t, SplitNode> candidate_lanes_split_nodes = sub_module_output.candidate_lanes_split_nodes;
        std::vector<bool> is_contain_split_vec = sub_module_output.is_contain_split_vec;

        int exit_start_link_index = sub_module_output.exit_start_link_index;
        // exit_start_link_index's next link is split , exit_start_link_index's link is split origin ,
        if (exit_start_link_index > 0 && (exit_start_link_index + 1) < is_contain_split_vec.size() &&
            (exit_start_link_index + 1) < link_id_vec.size() &&
            (exit_start_link_index + 1) < ego_path_lane_num.size()) {
            uint32_t link_id = link_id_vec[exit_start_link_index + 1];
            uint8_t lane_num = ego_path_lane_num[exit_start_link_index + 1];

            // std::cout << __FILE__ << "," << __LINE__ << ","
            //           << "link_id: " << link_id << " lane_num: " << (int)lane_num << std::endl;
            message::map_map::s_LinkInfo_t link_infos{};
            if (false == MapCommonTool::GetInstance()->GetLinkInfos(
                             map_msg, map_raw_data_map.link_id_index_lane_info_map, link_id, link_infos)) {
                return false;
            }
            auto iter =
                std::find_if(link_infos.LaneInfos.LaneInfos.begin(), link_infos.LaneInfos.LaneInfos.end(),
                             [&](const message::map_map::s_LaneInfo_t& it) { return it.LaneNum.LaneNum == lane_num; });
            if (iter != link_infos.LaneInfos.LaneInfos.end()) {
                if ((*iter).Transit.data == 1) {
                    // transit continue
                    is_continue = true;
                } else if ((*iter).Transit.data == 3) {
                    is_split = true;
                }
            }
        }
    }

    return true;
}

bool EfmOutput::MakeOutputMapLppInfoOther(const MapMapMsg& map_msg, MapPositionTmp& map_position_tmp,
                                          const SubModuleOutput& sub_module_output, MapLppInfoMsg& map_lpp_info) {
    map_lpp_info.PositionTimeStamp = map_position_tmp.PositionTimeStamp;
    {
        map_lpp_info.mRampInfo.startPoint.dX = sub_module_output.exit_start_dist;
        map_lpp_info.mRampInfo.endPoint.dX = sub_module_output.exit_end_dist;
    }
    // std::cout << "map_lpp_info.mRampInfo.endPoint.dX: " << map_lpp_info.mRampInfo.endPoint.dX << std::endl;
    {
        map_lpp_info.mRefLineSpeeds.nSpeedLimitationLocalLane = int(sub_module_output.cur_speed);
        map_lpp_info.mRefLineSpeeds.nSpeedLimitationTargetLane = int(sub_module_output.next_speed);
        map_lpp_info.mRefLineSpeeds.nPntIdx = sub_module_output.pntIdx;
    }

    {
        map_lpp_info.laneDecision[0].isDriveable = sub_module_output.ego_path.bIsAvailable;
        map_lpp_info.laneDecision[1].isDriveable = sub_module_output.left_path.bIsAvailable;
        map_lpp_info.laneDecision[2].isDriveable = sub_module_output.right_path.bIsAvailable;
#ifdef EM_COUT
        std::cout << __FILE__ << "," << __LINE__ << ","
                  << "map_lpp_info.laneDecision[0].isDriveable: " << map_lpp_info.laneDecision[0].isDriveable
                  << std::endl;
        std::cout << __FILE__ << "," << __LINE__ << ","
                  << "map_lpp_info.laneDecision[1].isDriveable: " << map_lpp_info.laneDecision[1].isDriveable
                  << std::endl;
        std::cout << __FILE__ << "," << __LINE__ << ","
                  << "map_lpp_info.laneDecision[2].isDriveable: " << map_lpp_info.laneDecision[2].isDriveable
                  << std::endl;

        // lane type
        std::cout << __FILE__ << "," << __LINE__ << ","
                  << "sub_module_output.ego_path.LaneType: " << static_cast<int>(sub_module_output.ego_path.LaneType)
                  << std::endl;

#endif
    }

    {  // cur road type
        uint32_t ego_link_id = map_position_tmp.LinkId;
        auto iter = std::find_if(
            map_msg.FormOfWays.FormOfWays.begin(), map_msg.FormOfWays.FormOfWays.end(),
            [&](const message::map_map::s_FormOfWay_t& it) { return it.InstanceId.InstanceId == ego_link_id; });

        if (iter != map_msg.FormOfWays.FormOfWays.end()) {
            uint8_t road_class = (*iter).FunctionRoadClass.data;
#ifdef EM_COUT
            std::cout << __FILE__ << "," << __LINE__ << ","
                      << "road_class: " << static_cast<int>(road_class) << " ,ego_link_id: " << ego_link_id
                      << " ,formway_link_id: " << (*iter).InstanceId.InstanceId << std::endl;
#endif
            switch (road_class) {
                case 1:
                    map_lpp_info.RoadType.data_ = 5;
                    break;
                case 2:
                    map_lpp_info.RoadType.data_ = 1;
                    break;
                default:
                    map_lpp_info.RoadType.data_ = 0;
                    break;
            }
        }
    }

    {  // lane id, lane number
        map_lpp_info.laneIndex = sub_module_output.fixed_lane_id;
        map_lpp_info.laneNumber = sub_module_output.driveable_lane_size;
        map_lpp_info.egoLaneIndex = sub_module_output.fixed_lane_id;
        // }

// #ifdef EM_COUT
//         std::cout << __FILE__ << "," << __LINE__ << ","
//                   << "candidate_lanes_model_->min_driveable_lane_num() : "
//                   << (int)candidate_lanes_model_->min_driveable_lane_num() << std::endl;
//         std::cout << __FILE__ << "," << __LINE__ << ","
//                   << "candidate_lanes_model_->max_driveable_lane_num() : "
//                   << (int)candidate_lanes_model_->max_driveable_lane_num() << std::endl;
//         std::cout << __FILE__ << "," << __LINE__ << ","
//                   << "map_lpp_info.nLaneIndex: " << (int)map_lpp_info.laneIndex << std::endl;
//         std::cout << __FILE__ << "," << __LINE__ << ","
//                   << "map_lpp_info.nLaneNumber: " << (int)map_lpp_info.laneNumber << std::endl;
//         std::cout << __FILE__ << "," << __LINE__ << ","
//                   << "map_lpp_info.nEgoLaneIndex: " << (int)map_lpp_info.egoLaneIndex << std::endl;
// #endif
    }
    //1029 pre cut in 
    if(sub_module_output.pre_cutin_merge_offset.size()>0){
        map_lpp_info.mRefPaths[2].dNodeOffset = (static_cast<double>(sub_module_output.pre_cutin_merge_offset[0])- static_cast<double>(map_position_tmp.PathOffset))/100.0;
        if(map_lpp_info.mRefPaths[2].dNodeOffset<0){
            map_lpp_info.mRefPaths[2].dNodeOffset = -1;
            if(sub_module_output.pre_cutin_merge_offset.size()>1){
                map_lpp_info.mRefPaths[2].dNodeOffset = 
                    (static_cast<double>(sub_module_output.pre_cutin_merge_offset[1])- static_cast<double>(map_position_tmp.PathOffset))/100.0;
            }
        }
    }else{
        map_lpp_info.mRefPaths[2].dNodeOffset = -1;
    }
    ZTEXT("EFM_INFO", "pre_cut_dist: ", 50, -15, "pre_cut_dist: {}", map_lpp_info.mRefPaths[2].dNodeOffset);
    return true;
}

bool EfmOutput::MakeOutputMapEfmInfo(const SubModuleOutput& sub_module_output, EfmInfoMsg& efm_info) {
    int curvature_idx = 0;
    int curvature_s_idx = 0;

    for (int i = 0; i < sub_module_output.opt_kappas.size(); i++) {
        if (i % 4 == 0 && curvature_idx < 21) {
            efm_info.curvature_infos[0].curvature_vec[curvature_idx].curvature_value = sub_module_output.opt_kappas[i];
            efm_info.curvature_infos[0].curvature_vec[curvature_idx].offset = sub_module_output.opt_accumulated_s[i];
            curvature_idx++;
        }
    }

    // std::cout << __FILE__ << "," << __LINE__ << ","<< "curvature_value: " ;
    // for(int i = 0; i < efm_info.curvature_infos[0].curvature_vec.size(); i++){
    //         std::cout << std::fixed << std::setprecision(12) <<
    //         efm_info.curvature_infos[0].curvature_vec[i].curvature_value << " ";
    // }
    // std::cout << "}"<<std::endl;

    // std::cout << __FILE__ << "," << __LINE__ << ","<< "opt_accumulated_s: " ;
    // for(int i = 0; i < efm_info.curvature_infos[0].curvature_vec.size(); i++){
    //         std::cout << efm_info.curvature_infos[0].curvature_vec[i].offset << " ";
    // }
    // std::cout << "}"<<std::endl;

    double max_curvature = 0.0;
    double max_offset = 0.0;
    int max_index = 0;
    for (int i = 0; i < efm_info.curvature_infos[0].curvature_vec.size(); i++) {
        double curvature_value = efm_info.curvature_infos[0].curvature_vec[i].curvature_value;
        double offset = efm_info.curvature_infos[0].curvature_vec[i].offset;
        if (std::abs(curvature_value) > std::abs(max_curvature)) {
            max_curvature = std::abs(curvature_value);
            max_offset = offset;
            max_index = i;
        }
    }

    ZTEXT("EFM_INFO", "max_curvature_radius: ", -30, -56, "max_curvature_radius: {}",
          int(1 / (max_curvature + 0.0000001)));
    ZTEXT("EFM_INFO", "max_curvature_offset: ", -30, -59, "max_curvature_offset: {}", max_offset);
    // ZTEXT("EFM_INFO", "max_index: ", -30, -62, "max_index: {}", max_index);
    // std::cout << "Max Curvature: " << 1/max_curvature << std::endl;
    // std::cout << "Offset: " << max_offset << std::endl;
    // std::cout << "Index: " << max_index << std::endl;

    // #endif
    // temp add speed limit

    efm_info.SpdLmtInfo.MapSpdLimFirst = static_cast<float>(sub_module_output.exit_scene_speed_limit[0].first);
    efm_info.SpdLmtInfo.DstToMapSpdLimFirst = static_cast<float>(sub_module_output.exit_scene_speed_limit[0].second);
    efm_info.SpdLmtInfo.MapSpdLimSec = static_cast<float>(sub_module_output.exit_scene_speed_limit[1].first);
    efm_info.SpdLmtInfo.DstToMapSpdLimSec = static_cast<float>(sub_module_output.exit_scene_speed_limit[1].second);
    if (efm_info.SpdLmtInfo.DstToMapSpdLimFirst > 300) {
        efm_info.SpdLmtInfo.DstToMapSpdLimFirst = 1;
        efm_info.SpdLmtInfo.MapSpdLimFirst = 99;
    }
    if (efm_info.SpdLmtInfo.DstToMapSpdLimSec > 300) {
        efm_info.SpdLmtInfo.DstToMapSpdLimSec = 1;
        efm_info.SpdLmtInfo.MapSpdLimSec = 99;
    }

    // direct off ramp
    if (sub_module_output.exit_type == 1) {
        efm_info.SpdLmtInfo.DstToMapSpdLimFirst = 1;
        efm_info.SpdLmtInfo.MapSpdLimFirst = 99;
        efm_info.SpdLmtInfo.DstToMapSpdLimSec = 1;
        efm_info.SpdLmtInfo.MapSpdLimSec = 99;
    }

// #undef EM_COUT
#ifdef EM_COUT
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " efm_info.curvature_infos[0].curvature_vec[19].curvature_value: "
              << efm_info.curvature_infos[0].curvature_vec[19].curvature_value << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " efm_info.curvature_infos[0].curvature_vec[19].offset: "
              << efm_info.curvature_infos[0].curvature_vec[19].offset << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " efm_info.curvature_infos[0].curvature_vec[20].curvature_value: "
              << efm_info.curvature_infos[0].curvature_vec[20].curvature_value << std::endl;
    std::cout << __FILE__ << "," << __LINE__ << ","
              << " efm_info.curvature_infos[0].curvature_vec[20].offset: "
              << efm_info.curvature_infos[0].curvature_vec[20].offset << std::endl;
#endif

    return true;
}

bool EfmOutput::MakeOutputMapLane(MapPositionTmp& map_position_tmp, const SubModuleOutput& sub_module_output,
                                  const EfmInfoMsg& efm_info, const MapRawData& map_raw_data, MapLaneMsg& map_lane) {
    // map_lane.LaneModel[0].lane_type.data_ = extract_ref_line_->ego_path().LaneType;
    // map_lane.LaneModel[1].lane_type.data_ = extract_ref_line_->left_path().LaneType;
    // map_lane.LaneModel[2].lane_type.data_ = extract_ref_line_->right_path().LaneType;

    // {  // curvature
    //     map_lane.LaneModel[0].curvatures = efm_info.curvature_infos[0].curvature_vec;
    //     map_lane.LaneModel[1].curvatures = efm_info.curvature_infos[0].curvature_vec;
    //     map_lane.LaneModel[2].curvatures = efm_info.curvature_infos[0].curvature_vec;
    // }

    {  // map_position
        map_lane.Position.timeStamp = map_position_tmp.PositionTimeStamp;
        map_lane.Position.positionage = map_position_tmp.PositionAge;
        map_lane.Position.dLon = map_raw_data.veh_point_utm_x;
        map_lane.Position.dLat = map_raw_data.veh_point_utm_y;
        map_lane.Position.dSpeed = map_position_tmp.Speed;
        map_lane.Position.dAcc_x = map_position_tmp.xAcc;
        map_lane.Position.dAcc_y = map_position_tmp.yAcc;
        map_lane.Position.dAcc_z = map_position_tmp.zAcc;
        map_lane.Position.dRollRate = map_position_tmp.AngularVelocityX.AngularVelocityX;
        map_lane.Position.dPitchRate = map_position_tmp.AngularVelocityY.AngularVelocityY;
        map_lane.Position.dYawRate = map_position_tmp.AngularVelocityZ.AngularVelocityZ;
        map_lane.Position.locStatus.data_ = map_position_tmp.FailSafeLocStatus.data;
        map_lane.Position.mapIsValid = 1;
        // uint8_t isNearRamp;
        map_lane.Position.heading = map_position_tmp.Heading.Heading;
    }

    // extra
    SetOneElementExtra(map_position_tmp, sub_module_output.left_left_extra_info, sub_module_output.left_left_path_lane_num,sub_module_output.link_id_vec, 
                           sub_module_output.split_info_map,sub_module_output.split_info_deque, map_lane.lanes[0]);
    SetOneElementExtra(map_position_tmp, sub_module_output.left_extra_info, sub_module_output.left_path_lane_num,sub_module_output.link_id_vec, 
                           sub_module_output.split_info_map,sub_module_output.split_info_deque, map_lane.lanes[1]);
    SetOneElementExtra(map_position_tmp, sub_module_output.ego_extra_info, sub_module_output.ego_path_lane_num,sub_module_output.link_id_vec, 
                           sub_module_output.split_info_map,sub_module_output.split_info_deque,map_lane.lanes[2]);
    SetOneElementExtra(map_position_tmp, sub_module_output.right_extra_info, sub_module_output.right_path_lane_num,sub_module_output.link_id_vec, 
                           sub_module_output.split_info_map,sub_module_output.split_info_deque,map_lane.lanes[3]);
    SetOneElementExtra(map_position_tmp, sub_module_output.right_right_extra_info, sub_module_output.right_right_path_lane_num,sub_module_output.link_id_vec, 
                           sub_module_output.split_info_map,sub_module_output.split_info_deque,map_lane.lanes[4]);    

    return true;
}

bool EfmOutput::SetOneElementExtra(MapPositionTmp& map_position_tmp,std::vector<LaneExtraInfo_s> lane_extra_info, std::vector<uint8_t> lane_num_vec, 
                                   std::vector<uint32_t> link_id_vec, 
                                   const std::map<uint64_t,SplitInfo_S>& split_info_map, const std::deque<uint64_t>& split_info_deque, message::efm::s_Lane_t& efm_lane_info) {
    if (0 == lane_extra_info.size()) {
        return true;
    }

    {
        // lanetype
        efm_lane_info.types[0].valid = false;
        efm_lane_info.types[1].valid = false;
        efm_lane_info.types[2].valid = false;

        int32_t type_idx = 0;
        for (auto& lane_type : lane_extra_info) {
            if (type_idx >= 3) {
                break;
            }

            float temp_s_offset = static_cast<float>(lane_type.s_offset / 100.0);
            temp_s_offset = temp_s_offset > 0 ? temp_s_offset : 0;
            if (0 == type_idx) {
                if (lane_type.e_offset > 0) {
                    efm_lane_info.types[type_idx].valid = true;
                    efm_lane_info.types[type_idx].s = temp_s_offset;
                    efm_lane_info.types[type_idx].type.data_ = lane_type.lane_type;
                    type_idx++;
                }
            } else {
                if (lane_type.lane_type == efm_lane_info.types[type_idx - 1].type.data_) {
                    continue;
                }

                if (temp_s_offset > efm_lane_info.types[type_idx - 1].s) {
                    efm_lane_info.types[type_idx].valid = true;
                    efm_lane_info.types[type_idx].s = temp_s_offset;
                    efm_lane_info.types[type_idx].type.data_ = lane_type.lane_type;
                    type_idx++;
                } else {
                    efm_lane_info.types[type_idx - 1].s = temp_s_offset;
                    efm_lane_info.types[type_idx - 1].type.data_ = lane_type.lane_type;
                }
            }
        }
    }

    {
        // curvature
        efm_lane_info.lane_attrs.curveSize = 0;
        efm_lane_info.lane_attrs.curves = {};
        for (auto& lane_info : lane_extra_info) {
            for (auto& curvinfo : lane_info.curvpoints) {
                double curvpoint_offset = (curvinfo.CurvPointPathOffset - map_position_tmp.PathOffset) / 100.0;
                if (curvpoint_offset > 0 && curvpoint_offset < 300 && efm_lane_info.lane_attrs.curveSize < 199) {
                    efm_lane_info.lane_attrs.curves[efm_lane_info.lane_attrs.curveSize].offset = curvpoint_offset;
                    efm_lane_info.lane_attrs.curves[efm_lane_info.lane_attrs.curveSize].curvature_value =
                        curvinfo.CurvPointValue;
                    efm_lane_info.lane_attrs.curveSize++;
                }
            }
        }
    }

    {
        // merge
        efm_lane_info.lane_merge[0].valid = false;
        efm_lane_info.lane_merge[1].valid = false;
        efm_lane_info.lane_merge[2].valid = false;
        int32_t merge_idx = 0;
        for (auto& merge_info : lane_extra_info) {
            if (merge_idx >= 3) {
                break;
            }

            if (EFM_MergeType_NONE == merge_info.merge_value || merge_info.e_offset <= 0) {
                continue;
            }

            float temp_s_offset = static_cast<float>(merge_info.s_offset / 100.0);
            if(merge_info.is_road_merge == true){
                temp_s_offset = static_cast<float>(merge_info.road_merge_s_e_offset.first/100);
            }
            temp_s_offset = temp_s_offset > 0 ? temp_s_offset : 0;
            float temp_e_offset = static_cast<float>(merge_info.e_offset / 100.0);

            efm_lane_info.lane_merge[merge_idx].valid = true;
            efm_lane_info.lane_merge[merge_idx].s_start = temp_s_offset;
            efm_lane_info.lane_merge[merge_idx].s_end = temp_e_offset;
            switch (merge_info.merge_value) {
                case EFM_MergeType_NONE:
                case EFM_MergeType_TO_LEFT:
                case EFM_MergeType_FROM_LEFT:
                case EFM_MergeType_LEFT_TO_MIDDLE:
                case EFM_MergeType_TO_RIGHT:
                case EFM_MergeType_FROM_RIGHT:
                    efm_lane_info.lane_merge[merge_idx].dir.data_ = merge_info.merge_value;
                    break;
                case EFM_MergeType_RIGHT_TO_MIDDLE:
                    efm_lane_info.lane_merge[merge_idx].dir.data_ = 3;
                    break;
                default:
                    efm_lane_info.lane_merge[merge_idx].dir.data_ = 0;
                    break;
            }
            merge_idx++;
        }
    }

    {
        //split
        earth::shell::framework::CandidateLanesModel model_tmp;
        efm_lane_info.lane_split[0].valid = false;
        efm_lane_info.lane_split[1].valid = false;
        efm_lane_info.lane_split[2].valid = false;
        int32_t split_idx = 0;
        //行驶方向反向的split先找
        uint8_t pos_lane_id = lane_num_vec.front();
        uint32_t pos_link_id = link_id_vec.front();
        // uint64_t key_temp = ((static_cast<uint64_t>(lane_id)) <<32) | (static_cast<uint64_t>(link_id));
        // std::cout << __FILE__ << "," << __LINE__ << "," << "lane_id: " << (int)pos_lane_id<<" ,link_id: "<<pos_link_id<< std::endl;      
        SplitInfo_S store_split_info;
        if(model_tmp.GetStoredSplitInfo(split_info_map, split_info_deque, 
                                                       pos_lane_id, pos_link_id, store_split_info) == true){
            float temp_s_offset = (static_cast<float>(store_split_info.s_offset) - static_cast<float>( map_position_tmp.PathOffset))/100;
            // temp_s_offset = temp_s_offset > 0 ? temp_s_offset : 0;
            float temp_e_offset = (static_cast<float>(store_split_info.e_offset) - static_cast<float>( map_position_tmp.PathOffset))/100;
            if(temp_s_offset > -200){
                efm_lane_info.lane_split[split_idx].valid = true;
                efm_lane_info.lane_split[split_idx].s_start = temp_s_offset;
                efm_lane_info.lane_split[split_idx].s_end = temp_e_offset;
                // std::cout << __FILE__ << "," << __LINE__ << "," << "lane_id: " << (int)pos_lane_id<<" ,link_id: "<<pos_link_id<< " ,store_split_info.e_offset: "<<store_split_info.e_offset<<" ,store_split_info.s_offset: "<<store_split_info.s_offset<< std::endl;  
                switch (store_split_info.split_dir)
                {
                case EFM_SplitType_NONE:
                case EFM_SplitType_TO_LEFT:
                case EFM_SplitType_FROM_LEFT:
                case EFM_SplitType_TO_RIGHT:
                case EFM_SplitType_FROM_RIGHT:
                    efm_lane_info.lane_split[split_idx].dir.data_ = store_split_info.split_dir;
                    break;
                case EFM_SplitType_CONTIUE_FROM_LEFT:
                case EFM_SplitType_SPLIT_FROM_LEFT:
                    efm_lane_info.lane_split[split_idx].dir.data_ = 2;
                    break; 
                case EFM_SplitType_CONTIUE_FROM_RIGHT:
                case EFM_SplitType_SPLIT_FROM_RIGHT:
                    efm_lane_info.lane_split[split_idx].dir.data_ = 5;
                    break;     
                default:
                    efm_lane_info.lane_split[split_idx].dir.data_ = 0;
                    break;
                }
                split_idx++;                
            }

        }

        //行驶方向前方的split       
        for (int common_index = 0;common_index<lane_extra_info.size() && common_index<lane_num_vec.size() && common_index<link_id_vec.size();common_index++){
            auto split_info = lane_extra_info[common_index];
            uint8_t lane_id = lane_num_vec[common_index];
            uint32_t link_id = link_id_vec[common_index];
            // std::cout << __FILE__ << "," << __LINE__ << "," <<"common_index: "<<common_index <<"lane_id: " << (int)lane_id<<" ,link_id: "<<link_id<< std::endl;  
            if (split_idx >= 3){
                break;
            }

            if (EFM_SplitType_NONE == split_info.split_value || split_info.e_offset <= 0 ||(split_idx>0 && lane_id == pos_lane_id && link_id == pos_link_id)){
                continue;
            }
            SplitInfo_S store_split_info;
            if(model_tmp.GetStoredSplitInfo(split_info_map, split_info_deque, 
                                                       lane_id, link_id, store_split_info) == true){
            float temp_s_offset = (static_cast<float>(store_split_info.s_offset) - static_cast<float>( map_position_tmp.PathOffset))/100;
            temp_s_offset = temp_s_offset > 0 ? temp_s_offset : 0;
            float temp_e_offset = (static_cast<float>(store_split_info.e_offset) - static_cast<float>( map_position_tmp.PathOffset))/100;
            efm_lane_info.lane_split[split_idx].valid = true;
            efm_lane_info.lane_split[split_idx].s_start = temp_s_offset;
            efm_lane_info.lane_split[split_idx].s_end = temp_e_offset;
            switch (store_split_info.split_dir)
            {
            case EFM_SplitType_NONE:
            case EFM_SplitType_TO_LEFT:
            case EFM_SplitType_FROM_LEFT:
            case EFM_SplitType_TO_RIGHT:
            case EFM_SplitType_FROM_RIGHT:
                efm_lane_info.lane_split[split_idx].dir.data_ = store_split_info.split_dir;
                break;
            case EFM_SplitType_CONTIUE_FROM_LEFT:
            case EFM_SplitType_SPLIT_FROM_LEFT:
                efm_lane_info.lane_split[split_idx].dir.data_ = 2;
                break; 
            case EFM_SplitType_CONTIUE_FROM_RIGHT:
            case EFM_SplitType_SPLIT_FROM_RIGHT:
                efm_lane_info.lane_split[split_idx].dir.data_ = 5;
                break;     
            default:
                efm_lane_info.lane_split[split_idx].dir.data_ = 0;
                break;
            }
            split_idx++;
            // std::cout << __FILE__ << "," << __LINE__ << ",get!!!: " << "lane_id: " << (int)lane_id<<" ,link_id: "<<link_id<< " ,store_split_info.e_offset: "<<store_split_info.e_offset<<" ,store_split_info.s_offset: "<<store_split_info.s_offset<< std::endl;  
            // std::cout << __FILE__ << "," << __LINE__ << "," << "temp_s_offset: " << temp_s_offset<<" ,temp_e_offset: "<<temp_e_offset<< std::endl;  
            }

        }
    }

    return true;
}

bool EfmOutput::MakeOutputZTEXT(const EfmInfoMsg& efm_info, const MapLaneMsg& map_lane,
                                const MapLppInfoMsg& map_lpp_info, const RMFMapinfoDataclose& rmf_dc) {
    uint64_t efm_globalline_use_time_ns = 0;
    uint64_t efm_outputztext_use_time_ns = 0;

#ifdef __QNX__
    auto efm_outputztext_start_now = bsw::gptp_clock::now();
    auto efm_outputztext_start_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(efm_outputztext_start_now.time_since_epoch());
#endif
    // global line
    std::vector<double> global_ref_plot_x{};  // for plot
    std::vector<double> global_ref_plot_y{};  // for plot
    int ref_pnt_num = 0;
    for (auto& point : map_lpp_info.priorRefLine.PriorRefLine) {
        if (ref_pnt_num % 3 == 0) {
            global_ref_plot_x.push_back(point.dX);
            global_ref_plot_y.push_back(point.dY);
        }
        ref_pnt_num++;
    }
#ifdef __QNX__
    auto efm_globalline_start_now = bsw::gptp_clock::now();
    auto efm_globalline_start_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(efm_globalline_start_now.time_since_epoch());
#endif
    ZPLOTXYF("EFM_INFO", ".black8", global_ref_plot_x, global_ref_plot_y);
#ifdef __QNX__
    auto efm_globalline_end_now = bsw::gptp_clock::now();
    auto efm_globalline_end_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(efm_globalline_end_now.time_since_epoch());
    efm_globalline_use_time_ns =
        static_cast<uint64_t>(efm_globalline_end_time.count() - efm_globalline_start_time.count());
#endif
    ZTEXT("EFM_INFO", "efm_globalline_use_time_ns: ", 80, 24, "efm_globalline_use_time_ns: {}",
          efm_globalline_use_time_ns);

// output
#ifdef EM_COUT
    std::cout << "EFMdebugforEnvironmentModel-"
              << "mRefPaths[0].dNodeOffset-" << map_lpp_info.mRefPaths[0].dNodeOffset << std::endl;
    std::cout << "EFMdebugforEnvironmentModel-"
              << "mRefPaths[0].bProbabilityLeft-" << map_lpp_info.mRefPaths[0].bProbabilityLeft << std::endl;
    std::cout << "EFMdebugforEnvironmentModel-"
              << "mRefPaths[0].bProbabilityRight-" << map_lpp_info.mRefPaths[0].bProbabilityRight << std::endl;
    std::cout << "EFMdebugforEnvironmentModel-"
              << "mRefPaths[4].dNodeOffset-" << map_lpp_info.mRefPaths[4].dNodeOffset << std::endl;
    std::cout << "EFMdebugforEnvironmentModel-"
              << "mRefPaths[4].bProbabilityLeft-" << map_lpp_info.mRefPaths[4].bProbabilityLeft << std::endl;
    std::cout << "EFMdebugforEnvironmentModel-"
              << "mRefPaths[4].bProbabilityRight-" << map_lpp_info.mRefPaths[4].bProbabilityRight << std::endl;
    std::cout << "EFMdebugforEnvironmentModel-"
              << "mRefPaths[5].dNodeOffset-" << map_lpp_info.mRefPaths[5].dNodeOffset << std::endl;
    std::cout << "EFMdebugforEnvironmentModel-"
              << "mRefPaths[5].bProbabilityLeft-" << map_lpp_info.mRefPaths[5].bProbabilityLeft << std::endl;
    std::cout << "EFMdebugforEnvironmentModel-"
              << "mRefPaths[5].bProbabilityRight-" << map_lpp_info.mRefPaths[5].bProbabilityRight << std::endl;
    std::cout << "EFM数据重组完成 " << std::endl;
    std::cout << "start point:" << map_lpp_info.mRampInfo.startPoint.dX << std::endl;
    std::cout << "end point:" << map_lpp_info.mRampInfo.endPoint.dX << std::endl;
    std::cout << __FILE__ << __LINE__ << "nSpeedLimitationLocalLane"
              << (int)map_lpp_info.mRefLineSpeeds.nSpeedLimitationLocalLane << std::endl;

    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "candidate_lanes_model->ego_lane_prior_dist(): " << candidate_lanes_model_->ego_lane_prior_dist()
    //           << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "candidate_lanes_model->left_lane_prior_dist():: " << candidate_lanes_model_->left_lane_prior_dist()
    //           << std::endl;
    // std::cout << __FILE__ << "," << __LINE__ << ","
    //           << "candidate_lanes_model->right_lane_prior_dist(): " << candidate_lanes_model_->right_lane_prior_dist()
    //           << std::endl;

    std::cout << "EFM end " << std::endl;

#endif

#ifdef EFM_PLOT2D
    ZTEXT("EFM_INFO", "exit_start_dist_: ", 80, 48, "exit_start_dist_: {}", map_lpp_info.mRampInfo.startPoint.dX);
    ZTEXT("EFM_INFO", "exit_end_dist_: ", 80, 51, "exit_end_dist_: {}", map_lpp_info.mRampInfo.endPoint.dX);
    ZTEXT("EFM_INFO", "speed_limit[0].value: ", 19, 36, "speed_limit[0].value: {}", efm_info.SpdLmtInfo.MapSpdLimFirst);
    ZTEXT("EFM_INFO", "speed_limit[0].dist: ", 18, 39, "speed_limit[0].dist: {}",
          efm_info.SpdLmtInfo.DstToMapSpdLimFirst);
    ZTEXT("EFM_INFO", "speed_limit[1].value: ", 19, 42, "speed_limit[1].value: {}", efm_info.SpdLmtInfo.MapSpdLimSec);
    ZTEXT("EFM_INFO", "speed_limit[1].dist: ", 18, 45, "speed_limit[1].dist: {}",
          efm_info.SpdLmtInfo.DstToMapSpdLimSec);
    // ZTEXT("EFM_INFO", "cur_lane_speed: ", 10, 18, "cur_lane_speed: {}",
    //       int(map_lpp_info.mRefLineSpeeds.nSpeedLimitationLocalLane));
    // ZTEXT("EFM_INFO", "cur_lane_type: ", 10, 15, "cur_lane_type: {}", int(map_lane.LaneModel[0].lane_type.data_));
    ZTEXT("EFM_INFO", "special_startdist_: ", -25, -15, "special_startdist_: {}",
          int(map_lpp_info.mSpecialSit.dStartDistance));
    ZTEXT("EFM_INFO", "special_enddist_: ", -25, -18, "special_enddist_: {}",
          int(map_lpp_info.mSpecialSit.dEndDistance));
    ZTEXT("EFM_INFO", "special_type_: ", -35, -21, "special_type_: {}", map_lpp_info.mSpecialSit.eSpecSitType.data_);
    ZTEXT("EFM_INFO", "ByPassMergeDis: ", -25, -24, "ByPassMergeDis: {}", int(map_lpp_info.dDistanceByPassMerge));
    ZTEXT("EFM_INFO", "ByPassMergeDis1: ", -25, -27, "ByPassMergeDis1: {}", int(map_lpp_info.mRefPaths[1].dNodeOffset));

    ZTEXT("EFM_INFO", "bIsSetNav: ", -30, 53, "bIsSetNav: {}", map_lpp_info.mNavInfo.bIsSetNav);
    ZTEXT("EFM_INFO", "bNavIsYaw: ", -30, 56, "bNavIsYaw: {}", map_lpp_info.mNavInfo.bNavIsYaw);
    ZTEXT("EFM_INFO", "SwtLaneReason: ", -30, 59, "SwtLaneReason: {}", map_lpp_info.SwitchLaneReason.data_);
    ZTEXT("EFM_INFO", "lanetype1: ", 14, -20, "lanetype1: {}", map_lane.lanes[2].types[0].type.data_);
    ZTEXT("EFM_INFO", "lanetype1dis: ", 14, -23, "lanetype1dis: {}", int(map_lane.lanes[2].types[0].s));
    ZTEXT("EFM_INFO", "lanetype2: ", 14, -26, "lanetype2: {}", map_lane.lanes[2].types[1].type.data_);
    ZTEXT("EFM_INFO", "lanetype2dis: ", 14, -29, "lanetype2dis: {}", int(map_lane.lanes[2].types[1].s));
    // ZTEXT("EFM_INFO", "lanetype3: ", 14, -32, "lanetype3: {}", map_lane.lanes[2].types[2].type.data_);
    // ZTEXT("EFM_INFO", "lanetype3dis: ", 14, -35, "lanetype3dis: {}", int(map_lane.lanes[2].types[2].s));

    ZTEXT("EFM_INFO", "lanemerge1: ", 55, -20, "lanemerge1: {}", map_lane.lanes[2].lane_merge[0].dir.data_);
    ZTEXT("EFM_INFO", "lanemerge1start: ", 55, -23, "lanemerge1start: {}", map_lane.lanes[2].lane_merge[0].s_start);
    ZTEXT("EFM_INFO", "lanemerge1end: ", 55, -26, "lanemerge1end: {}", map_lane.lanes[2].lane_merge[0].s_end);
    // ZTEXT("EFM_INFO", "egolanemerge2: ", 45, -29, "egolanemerge2: {}", map_lane.lanes[2].lane_merge[1].dir.data_);
    // ZTEXT("EFM_INFO", "egolanemerge2start: ", 45, -32, "egolanemerge2start: {}",
    // map_lane.lanes[2].lane_merge[1].s_start); ZTEXT("EFM_INFO", "egolanemerge2end: ", 45, -35, "egolanemerge2end:
    // {}", map_lane.lanes[2].lane_merge[1].s_end); ZTEXT("EFM_INFO", "egolanemerge3: ", 45, -38, "egolanemerge3: {}",
    // map_lane.lanes[2].lane_merge[2].dir.data_); ZTEXT("EFM_INFO", "egolanemerge3start: ", 45, -41,
    // "egolanemerge3start: {}", map_lane.lanes[2].lane_merge[2].s_start); ZTEXT("EFM_INFO", "egolanemerge3end: ", 45,
    // -44, "egolanemerge3end: {}", map_lane.lanes[2].lane_merge[2].s_end);

    ZTEXT("EFM_INFO", "lanesplit1: ", 55, -29, "lanesplit1: {}", map_lane.lanes[2].lane_split[0].dir.data_);
    ZTEXT("EFM_INFO", "lanesplit1start: ", 55, -32, "lanesplit1start: {}", map_lane.lanes[2].lane_split[0].s_start);
    ZTEXT("EFM_INFO", "lanesplit1end: ", 55, -35, "lanesplit1end: {}", map_lane.lanes[2].lane_split[0].s_end);
    // ZTEXT("EFM_INFO", "egolanesplit2: ", 45, -56, "egolanesplit2: {}", map_lane.lanes[2].lane_split[1].dir.data_);
    // ZTEXT("EFM_INFO", "egolanesplit2start: ", 45, -59, "egolanesplit2start: {}",
    // map_lane.lanes[2].lane_split[1].s_start); ZTEXT("EFM_INFO", "egolanesplit2end: ", 45, -62, "egolanesplit2end:
    // {}", map_lane.lanes[2].lane_split[1].s_end); ZTEXT("EFM_INFO", "egolanesplit3: ", 45, -65, "egolanesplit3: {}",
    // map_lane.lanes[2].lane_split[2].dir.data_); ZTEXT("EFM_INFO", "egolanesplit3start: ", 45, -68,
    // "egolanesplit3start: {}", map_lane.lanes[2].lane_split[2].s_start); ZTEXT("EFM_INFO", "egolanesplit3end: ", 45,
    // -71, "egolanesplit3end: {}", map_lane.lanes[2].lane_split[2].s_end);

    ZTEXT("EFM_INFO", "leLaneIsAvl: ", 14, -38, "leLineIsAvl: {}", map_lpp_info.laneDecision[1].isDriveable);
    ZTEXT("EFM_INFO", "egoLaneIsAvl: ", 15, -41, "egoLaneIsAvl: {}", map_lpp_info.laneDecision[0].isDriveable);
    ZTEXT("EFM_INFO", "riLaneIsAvl: ", 14, -44, "riLaneIsAvl: {}", map_lpp_info.laneDecision[2].isDriveable);
    ZTEXT("EFM_INFO", "egoLaneId: ", 14, -47, "egoLaneId: {}", map_lpp_info.egoLaneIndex);
    ZTEXT("EFM_INFO", "drvLaneSize: ", 14, -50, "drvLaneSize: {}", map_lpp_info.laneNumber);
    // ZTEXT("EFM_INFO", "bIsActive: ", -30, 38, "bIsActive: {}", map_lpp_info.bIsActive);

    ZTEXT("EFM_INFO", "in_odd: ", -38, -36, "in_odd: {}", map_lpp_info.mMapOdd.eInOdd.data_);
    ZTEXT("EFM_INFO", "odd_end_dist: ", -30, -39, "odd_end_dist: {}", map_lpp_info.mMapOdd.dDisToOdd);
    ZTEXT("EFM_INFO", "out_odd_reason: ", -30, -42, "out_odd_reason: {}", map_lpp_info.mMapOdd.eOddReason.data_);

    ZTEXT("EFM_INFO", "node.StartPointOffset: ", 120, -34, "StartPointOffset: {}",
          map_lpp_info.NodeInfo.StartPointOffset);
    ZTEXT("EFM_INFO", "node.EndPointOffset: ", 120, -37, "EndPointOffset: {}", map_lpp_info.NodeInfo.EndPointOffset);
    ZTEXT("EFM_INFO", "node.DirectionType: ", 120, -40, "DirectionType: {}",
          int(map_lpp_info.NodeInfo.DirectionType.data_));
    ZTEXT("EFM_INFO", "node.LaneChgType: ", 120, -43, "LaneChgType: {}", int(map_lpp_info.NodeInfo.LaneChgType));
    ZTEXT("EFM_INFO", "node.LaneChgTimes: ", 120, -46, "LaneChgTimes: {}", int(map_lpp_info.NodeInfo.LaneChgTimes));

    ZTEXT("EFM_INFO", "ego_laneDecision: ", 125, 12, "ego_laneDecision: {}",
          int(map_lpp_info.laneDecision[0].isDriveable));
    ZTEXT("EFM_INFO", "left_laneDecision: ", 125, 15, "left_laneDecision: {}",
          int(map_lpp_info.laneDecision[1].isDriveable));
    ZTEXT("EFM_INFO", "right_laneDecision: ", 125, 18, "right_laneDecision: {}",
          int(map_lpp_info.laneDecision[2].isDriveable));

    ZTEXT("EFM_INFO", "roadClass: ", 14, -53, "roadClass: {}", map_lpp_info.RoadType.data_);
    ZTEXT("EFM_INFO", "priorindex: ", 14, -56, "priorindex: {}", map_lpp_info.priorIndex);
    ZTEXT("EFM_INFO", "errorcode_tolpp: ", -30, 50, "errorcode_tolpp: {}", map_lpp_info.header.err_code);
    ZTEXT("EFM_INFO", "cur_speed_final: ", 140, 55, "cur_speed_final: {}",
          int(map_lpp_info.mRefLineSpeeds.nSpeedLimitationLocalLane));
    ZTEXT("EFM_INFO", "next_speed_final: ", 140, 58, "next_speed_final: {}",
          int(map_lpp_info.mRefLineSpeeds.nSpeedLimitationTargetLane));
    ZTEXT("EFM_INFO", "next_speed_dist_final: ", 140, 61, "next_speed_dist_final: {}",
          map_lpp_info.mRefLineSpeeds.nPntIdx);
    ZTEXT("EFM_INFO", "has_wide_lane: ", -30, -33, "has_wide_lane: {}", map_lpp_info.mRefPaths[2].bProbabilityLeft);
#endif

    ZTEXT("EFM_INFO", "efm_outputztext_use_time_ns: ", 80, 21, "efm_outputztext_use_time_ns: {}",
          efm_outputztext_use_time_ns);

    return true;
}

bool EfmOutput::compareOddDis(const OddDis& a, const OddDis& b) { return a.odddis < b.odddis; }

}  // namespace efm
