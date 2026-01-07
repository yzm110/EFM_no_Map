
#include "data_to_geojson.h"
#include <iostream>
#include <string>
#include <typeinfo>

namespace earth {
namespace shell {
namespace framework {

CDataToJson::CDataToJson() : write_times_(0), write_times_kerb_(0), write_times_connect_(0), write_times_global_(0) {
    const char* file_name_char_section_c_;
    const char* file_name_char_section_l_;

    std::string geojson_file_c;
    geojson_file_c = std::to_string(0) + "_center.geojson";
    file_name_char_section_c_ = geojson_file_c.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_section_c_, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"center_json\",\n\"features\":[\n");

    std::string geojson_file_l;
    geojson_file_l = std::to_string(0) + "_left.geojson";
    file_name_char_section_l_ = geojson_file_l.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_section_l_, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"side_json\",\n\"features\":[\n");

    std::string geojson_file_kerb;
    geojson_file_kerb = std::to_string(0) + "_kerb.geojson";
    file_name_char_section_kerb_ = geojson_file_kerb.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_section_kerb_, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"kerb_json\",\n\"features\":[\n");

    std::string geojson_file_geofence;
    geojson_file_geofence = std::to_string(0) + "_geofence.geojson";
    file_name_char_geofence_ = geojson_file_geofence.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_geofence_, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"geofence_json\",\n\"features\":[\n");

    std::string geojson_file_position;
    geojson_file_position = std::to_string(0) + "_position.geojson";
    file_name_char_position_ = geojson_file_position.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_position_, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"position_json\",\n\"features\":[\n");

    std::string geojson_file_connect;
    geojson_file_connect = std::to_string(0) + "_connect.geojson";
    file_name_char_connect_ = geojson_file_connect.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_connect_, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"connect_json\",\n\"features\":[\n");

    std::string geojson_file_curvature;
    geojson_file_curvature = std::to_string(0) + "_curvature.geojson";
    file_name_char_curvature_ = geojson_file_curvature.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_curvature_, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"curvature_json\",\n\"features\":[\n");

    std::string geojson_file_slop;
    geojson_file_slop = std::to_string(0) + "_slop.geojson";
    file_name_char_slop_ = geojson_file_slop.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_slop_, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"slop_json\",\n\"features\":[\n");

    std::string geojson_file_switch;
    geojson_file_switch = std::to_string(0) + "_switch.geojson";
    file_name_char_switch_ = geojson_file_switch.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_switch_, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"curvature_json\",\n\"features\":[\n");

    std::string geojson_file_switch_global;
    geojson_file_switch_global = std::to_string(0) + "_switch_global.geojson";
    file_name_char_switch_global_ = geojson_file_switch_global.c_str();
    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_switch_global_, true,
        "{\n\"type\":\"FeatureCollection\",\n\"name\":\"switch_global_json\",\n\"features\":[\n");
}

CDataToJson::~CDataToJson() {}

bool CDataToJson::DownMapToGeoJson(const TopicTrait::MapMapMsg& map_msg) {
    map_static_info_ = std::make_shared<const TopicTrait::MapMapMsg>(map_msg);

    // std::string geojson_file_r;
    // geojson_file_r = std::to_string(write_times_) + "_right.geojson";
    // const char* file_name_char_section_r = geojson_file_r.c_str();

    // if (false == write_flag_)
    // {
    //     hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_c, true,
    //     "{\n\"type\":\"FeatureCollection\",\n\"name\":\"center_json\",\n\"features\":[\n" );
    //     hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_l, true,
    //     "{\n\"type\":\"FeatureCollection\",\n\"name\":\"center_json\",\n\"features\":[\n" );
    //     hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_r, true,
    //     "{\n\"type\":\"FeatureCollection\",\n\"name\":\"center_json\",\n\"features\":[\n" ); write_flag_ = true;
    // }

    std::string geojson_file_c;
    geojson_file_c = std::to_string(0) + "_center.geojson";
    file_name_char_section_c_ = geojson_file_c.c_str();

    std::string geojson_file_l;
    geojson_file_l = std::to_string(0) + "_left.geojson";
    file_name_char_section_l_ = geojson_file_l.c_str();

    std::string geojson_file_kerb;
    geojson_file_kerb = std::to_string(0) + "_kerb.geojson";
    file_name_char_section_kerb_ = geojson_file_kerb.c_str();

    // bool first_write = true;
    for (size_t i = 0; i < map_static_info_->LinkInfos.LinkInfos.size(); i++) {
        auto link_info = map_static_info_->LinkInfos.LinkInfos[i];
        uint32_t instance_id_ = link_info.InstanceId.InstanceId;
        uint32_t path_id_ = link_info.PathId.PathId;
        uint32_t s_offset_ = link_info.PathOffset.PathOffset;
        uint32_t e_offset_ = link_info.EndOffset.EndOffset;

        uint64_t path_link_key = static_cast<uint64_t>(path_id_) << 32 | instance_id_;
        if (path_id_link_id_map_.find(path_link_key) != path_id_link_id_map_.end()) {
            path_id_link_id_map_[path_link_key] += 1;
            continue;
        } else {
            path_id_link_id_map_[path_link_key] = 1;
        }

        for (auto& lane_info : link_info.LaneInfos.LaneInfos) {
            if (0 == write_times_) {
                // hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_c_, true,
                // "{\n\"type\":\"FeatureCollection\",\n\"name\":\"center_json\",\n\"features\":[\n" );
                // hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_l, true,
                // "{\n\"type\":\"FeatureCollection\",\n\"name\":\"side_json\",\n\"features\":[\n" );
                // hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_r, true,
                // "{\n\"type\":\"FeatureCollection\",\n\"name\":\"center_json\",\n\"features\":[\n" );
                write_times_ += 1;
                // first_write = false;
            } else {
                hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_c_, true, ",\n");
                hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_l_, true, ",\n");
                // hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_r, true, ",\n" );
            }

            uint8_t lane_num_ = lane_info.LaneNum.LaneNum;
            uint8_t dir_ = lane_info.Direction.data;
            uint8_t tran_ = lane_info.Transit.data;
            uint8_t lane_type_ = lane_info.LaneType.data;
            uint32_t cente_line_ = lane_info.Centeline.Centeline;
            uint32_t l_line_ = lane_info.LBound.LBound;
            uint32_t r_line_ = lane_info.RBound.RBound;
            int16_t max_speed = -1;
            int16_t min_speed = -1;
            int16_t max_width = 0;
            int16_t min_width = 0;
            // get speed
            for (auto lanespeed : map_static_info_->LaneSpeedLimitis.LaneSpeedLimits) {
                if (lanespeed.InstanceId.InstanceId == instance_id_ && lanespeed.PathId.PathId == path_id_ &&
                    lane_num_ == lanespeed.LaneNum.LaneNum) {
                    max_speed = lanespeed.ValueH.ValueH;
                    min_speed = lanespeed.ValueL.ValueL;
                }
            }
            // get width
            for (auto lanewidth : map_static_info_->LaneWidths.LaneWidths) {
                if (lanewidth.InstanceId.InstanceId == instance_id_ && lanewidth.PathId.PathId == path_id_ &&
                    lane_num_ == lanewidth.LaneNum.LaneNum) {
                    max_width = lanewidth.MaxWidth.MaxWidth;
                    min_width = lanewidth.MinWidth.MinWidth;
                }
            }

            // center
            hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_c_, false,
                                                          "{\"type\":\"Feature\", \"properties\":{\
\"path_id\":\"%u\",\"link_id\":\"%u\",\"s_offset\":\"%u\",\"e_offset\":\"%u\",\"lane_number\":\"%u\",\
\"dir\":\"%u\",\"tran\":\"%u\",\"type\":\"%u\",\"line_id\":\"%u\",\"maxspeed\":\"%d\",\"minspeed\":\"%d\",\
\"lane_in_line\":\"%u\",  \"max_width\":\"%d\",  \"min_width\":\"%d\",  \"efm_counter\":\"%llu\", ",
                                                          path_id_, instance_id_, s_offset_, e_offset_, lane_num_, dir_,
                                                          tran_, lane_type_, cente_line_, max_speed, min_speed, 0,
                                                          max_width, min_width, efm_counter_);

            for (auto lineobj : map_static_info_->LinearObjects.LinearObjects) {
                if (lineobj.IDLinearObject.IDLinearObject == cente_line_) {
                    hdm_utility::CHdmDbgLog::GetInstance()->Write(
                        file_name_char_section_c_, false,
                        "\"ObjectType\":\"%u\",\"ObjectMarking.\":\"%u\",\"ObjectColour\":\"%u\",\"ObjectIsBold\":\"%"
                        "u\",\"ObjectCurveType\":\"%u\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[",
                        lineobj.LinearObjectType.data, lineobj.LinearObjectMarking.data,
                        lineobj.LinearObjectColour.data, lineobj.LinearObjectIsBold.data,
                        lineobj.LinearObjectCurveType.data);

                    for (size_t point_idx = 0; point_idx < lineobj.PointCount.PointCount; point_idx++) {
                        if (0 == point_idx) {
                            hdm_utility::CHdmDbgLog::GetInstance()->Write(
                                file_name_char_section_c_, false, " [ %lf,  %lf]",
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Longitude.Longitude) *
                                    360.0 / (pow(2, 32)),
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Latitude.Latitude) *
                                    360.0 / (pow(2, 32)));
                        } else {
                            hdm_utility::CHdmDbgLog::GetInstance()->Write(
                                file_name_char_section_c_, false, " ,[ %lf,  %lf]",
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Longitude.Longitude) *
                                    360.0 / (pow(2, 32)),
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Latitude.Latitude) *
                                    360.0 / (pow(2, 32)));
                        }
                    }
                    hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_c_, true, "]}}");
                    break;
                }
            }

            // left
            hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_l_, false,
                                                          "{\"type\":\"Feature\", \"properties\":{\
\"path_id\":\"%u\",\"link_id\":\"%u\",\"s_offset\":\"%u\",\"e_offset\":\"%u\",\"lane_number\":\"%u\",\
\"dir\":\"%u\",\"tran\":\"%u\",\"type\":\"%u\",\"line_id\":\"%u\",\"lane_in_line\":\"%u\",  \"efm_counter\":\"%llu\", ",
                                                          path_id_, instance_id_, s_offset_, e_offset_, lane_num_, dir_,
                                                          tran_, lane_type_, l_line_, 1, efm_counter_);

            for (auto lineobj : map_static_info_->LinearObjects.LinearObjects) {
                if (lineobj.IDLinearObject.IDLinearObject == l_line_) {
                    hdm_utility::CHdmDbgLog::GetInstance()->Write(
                        file_name_char_section_l_, false,
                        "\"ObjectType\":\"%u\",\"ObjectMarking.\":\"%u\",\"ObjectColour\":\"%u\",\"ObjectIsBold\":\"%"
                        "u\",\"ObjectCurveType\":\"%u\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[",
                        lineobj.LinearObjectType.data, lineobj.LinearObjectMarking.data,
                        lineobj.LinearObjectColour.data, lineobj.LinearObjectIsBold.data,
                        lineobj.LinearObjectCurveType.data);
                    for (size_t point_idx = 0; point_idx < lineobj.PointCount.PointCount; point_idx++) {
                        if (0 == point_idx) {
                            hdm_utility::CHdmDbgLog::GetInstance()->Write(
                                file_name_char_section_l_, false, " [ %lf,  %lf]",
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Longitude.Longitude) *
                                    360.0 / (pow(2, 32)),
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Latitude.Latitude) *
                                    360.0 / (pow(2, 32)));
                        } else {
                            hdm_utility::CHdmDbgLog::GetInstance()->Write(
                                file_name_char_section_l_, false, " ,[ %lf,  %lf]",
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Longitude.Longitude) *
                                    360.0 / (pow(2, 32)),
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Latitude.Latitude) *
                                    360.0 / (pow(2, 32)));
                        }
                    }
                    hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_l_, true, "]}}");
                    break;
                }
            }

            hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_l_, true, ",\n");
            // right
            hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_l_, false,
                                                          "{\"type\":\"Feature\", \"properties\":{\
\"path_id\":\"%u\",\"link_id\":\"%u\",\"s_offset\":\"%u\",\"e_offset\":\"%u\",\"lane_number\":\"%u\",\
\"dir\":\"%u\",\"tran\":\"%u\",\"type\":\"%u\",\"line_id\":\"%u\",\"lane_in_line\":\"%u\", \"efm_counter\":\"%llu\",",
                                                          path_id_, instance_id_, s_offset_, e_offset_, lane_num_, dir_,
                                                          tran_, lane_type_, r_line_, 2, efm_counter_);

            for (auto lineobj : map_static_info_->LinearObjects.LinearObjects) {
                if (lineobj.IDLinearObject.IDLinearObject == r_line_) {
                    hdm_utility::CHdmDbgLog::GetInstance()->Write(
                        file_name_char_section_l_, false,
                        "\"ObjectType\":\"%u\",\"ObjectMarking.\":\"%u\",\"ObjectColour\":\"%u\",\"ObjectIsBold\":\"%"
                        "u\",\"ObjectCurveType\":\"%u\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[",
                        lineobj.LinearObjectType.data, lineobj.LinearObjectMarking.data,
                        lineobj.LinearObjectColour.data, lineobj.LinearObjectIsBold.data,
                        lineobj.LinearObjectCurveType.data);

                    for (size_t point_idx = 0; point_idx < lineobj.PointCount.PointCount; point_idx++) {
                        if (0 == point_idx) {
                            hdm_utility::CHdmDbgLog::GetInstance()->Write(
                                file_name_char_section_l_, false, " [ %lf,  %lf]",
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Longitude.Longitude) *
                                    360.0 / (pow(2, 32)),
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Latitude.Latitude) *
                                    360.0 / (pow(2, 32)));
                        } else {
                            hdm_utility::CHdmDbgLog::GetInstance()->Write(
                                file_name_char_section_l_, false, " ,[ %lf,  %lf]",
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Longitude.Longitude) *
                                    360.0 / (pow(2, 32)),
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Latitude.Latitude) *
                                    360.0 / (pow(2, 32)));
                        }
                    }
                    hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_l_, true, "]}}");
                    break;
                }
            }
        }
    }

    // kerb
    for (size_t i = 0; i < map_static_info_->LinkInfos.LinkInfos.size(); i++) {
        auto link_info = map_static_info_->LinkInfos.LinkInfos[i];
        uint32_t instance_id_ = link_info.InstanceId.InstanceId;
        uint32_t path_id_ = link_info.PathId.PathId;
        uint64_t path_link_key = static_cast<uint64_t>(path_id_) << 32 | instance_id_;
        if (path_id_link_idkerb_map_.find(path_link_key) != path_id_link_idkerb_map_.end()) {
            path_id_link_idkerb_map_[path_link_key] += 1;
            continue;
        } else {
            path_id_link_idkerb_map_[path_link_key] = 1;
        }
        for (auto lineobj : map_static_info_->LinearObjects.LinearObjects) {
            uint32_t instance_kerb_id_ = lineobj.InstanceId.InstanceId;
            uint32_t path_kerb_id_ = lineobj.PathId.PathId;
            uint32_t s_kerb_offset_ = lineobj.PathOffset.PathOffset;
            uint32_t e_kerb_offset_ = lineobj.EndOffset.EndOffset;
            uint8_t instance_LaneNum = lineobj.LaneNum.LaneNum;
            uint32_t lineatobject_id = lineobj.IDLinearObject.IDLinearObject;
            // uint64_t path_link_key = static_cast<uint64_t>(path_kerb_id_) << 32 | instance_kerb_id_;
            // if (path_id_link_idkerb_map_.find(path_link_key) != path_id_link_idkerb_map_.end()) {
            //     path_id_link_idkerb_map_[path_link_key] += 1;
            //     continue;
            // } else {
            //     path_id_link_idkerb_map_[path_link_key] = 1;
            // }
            if ((lineobj.LinearObjectType.data == 2 || lineobj.LinearObjectType.data == 3 ||
                 lineobj.LinearObjectType.data == 4 || lineobj.LinearObjectType.data == 6) &&
                lineobj.InstanceId.InstanceId == instance_id_) {
                if (0 == write_times_kerb_) {
                    write_times_kerb_ += 1;
                } else {
                    hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_kerb_, true, ",\n");
                }

                hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_kerb_, false,
                                                              "{\"type\":\"Feature\", \"properties\":{\
\"path_id\":\"%u\",\"link_id\":\"%u\",\"s_offset\":\"%u\",  \"lane_num\":\"%u\",  \"linear_id\":\"%u\",   \"e_offset\":\"%u\",",
                                                              path_kerb_id_, instance_kerb_id_, s_kerb_offset_,
                                                              instance_LaneNum, lineatobject_id, e_kerb_offset_);
                hdm_utility::CHdmDbgLog::GetInstance()->Write(
                    file_name_char_section_kerb_, false,
                    "\"ObjectType\":\"%u\",\"ObjectMarking.\":\"%u\",\"ObjectColour\":\"%u\",\"ObjectIsBold\":\"%"
                    "u\",\"ObjectCurveType\":\"%u\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[",
                    lineobj.LinearObjectType.data, lineobj.LinearObjectMarking.data, lineobj.LinearObjectColour.data,
                    lineobj.LinearObjectIsBold.data, lineobj.LinearObjectCurveType.data);

                for (size_t point_idx = 0; point_idx < lineobj.PointCount.PointCount; point_idx++) {
                    if (0 == point_idx) {
                        hdm_utility::CHdmDbgLog::GetInstance()->Write(
                            file_name_char_section_kerb_, false, " [ %lf,  %lf]",
                            static_cast<double>(lineobj.GeometryPoints.GeometryPoints[point_idx].Longitude.Longitude) *
                                360.0 / (pow(2, 32)),
                            static_cast<double>(lineobj.GeometryPoints.GeometryPoints[point_idx].Latitude.Latitude) *
                                360.0 / (pow(2, 32)));
                    } else {
                        hdm_utility::CHdmDbgLog::GetInstance()->Write(
                            file_name_char_section_kerb_, false, " ,[ %lf,  %lf]",
                            static_cast<double>(lineobj.GeometryPoints.GeometryPoints[point_idx].Longitude.Longitude) *
                                360.0 / (pow(2, 32)),
                            static_cast<double>(lineobj.GeometryPoints.GeometryPoints[point_idx].Latitude.Latitude) *
                                360.0 / (pow(2, 32)));
                    }
                }
                hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_kerb_, true, "]}}");
            }
        }
    }
    // if(false == first_write)
    // {
    //     hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_c, true, "]\n}" );
    //     hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_l, true, "]\n}" );
    //     //hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_section_r, true, "]\n}" );
    // }

    //_genfence
    std::string geojson_file_geofence;
    geojson_file_geofence = std::to_string(0) + "_geofence.geojson";
    file_name_char_geofence_ = geojson_file_geofence.c_str();
    for (auto geofence_info : map_static_info_->GeoFences.GeoFences) {
        uint64_t path_link_key =
            static_cast<uint64_t>(geofence_info.PathId_.PathId) << 32 | geofence_info.InstanceId_.InstanceId;
        if (path_id_link_id_cur_geofence_.find(path_link_key) != path_id_link_id_cur_geofence_.end()) {
            path_id_link_id_cur_geofence_[path_link_key] += 1;
            continue;
        }

        // uint32_t offset = geofence_info.PathOffset_.PathOffset;
        uint8_t genfencetype = geofence_info.GeoFenceType.data;
        // std::string geofence_list_char;
        // geofence_list_char += std::to_string(offset);
        // geofence_list_char += ",";
        // geofence_list_char += std::to_string(genfencetype);
        // std::cout << __FILE__ << " " << __LINE__ << " " << geofence_info.InstanceId_.InstanceId << std::endl;
        // std::cout << __FILE__ << " " << __LINE__ << " " << int(geofence_info.LaneNum_.LaneNum) << std::endl;
        // std::cout << __FILE__ << " " << __LINE__ << " " << int(geofence_info.GeoFenceType.data) << std::endl;
        // std::cout << __FILE__ << " " << __LINE__ << " " << geofence_list_char << std::endl;

        // for (int32_t i = 0; i < geofence_info.GeoFenceCount.GeoFenceCount; i++) {
        //     uint32_t offset = geofence_info.CurvPoints.CurvPoints[i].PathOffset.PathOffset;
        //     float value = curvature_info.CurvPoints.CurvPoints[i].CurvPointValue.CurvPointValue;
        //     curvature_list_char += std::to_string(offset);
        //     curvature_list_char += ",";
        //     curvature_list_char += std::to_string(value);
        //     if (i < (curvature_info.CurvCount.CurvCount - 1)) {
        //         curvature_list_char += "##";
        //     }
        // }

        hdm_utility::CHdmDbgLog::GetInstance()->Write(
            file_name_char_geofence_, true,
            "{\"type\":\"Feature\", \"properties\":{\
\"path_id\":\"%u\",\"link_id\":\"%u\",  \"GeoFenceType\":\"%u\",\"lanenumber\":\"%u\",\"offset\":\"%u\",\"end_offset\":\"%u\",\
\"GeoFenceCount\":\"%u\" , \"position_counter\":\"%llu\" },\"geometry\":{\"type\":\"Point\",\"coordinates\":[0,  0]}},\n",
            geofence_info.PathId_.PathId, geofence_info.InstanceId_.InstanceId, genfencetype,
            geofence_info.LaneNum_.LaneNum, geofence_info.PathOffset_.PathOffset, geofence_info.EndOffset_.EndOffset,
            geofence_info.GeoFenceCount.GeoFenceCount, position_counter_);
    }

    for (auto geofence_info : map_static_info_->GeoFences.GeoFences) {
        uint64_t path_link_key =
            static_cast<uint64_t>(geofence_info.PathId_.PathId) << 32 | geofence_info.InstanceId_.InstanceId;
        if (path_id_link_id_cur_geofence_.find(path_link_key) == path_id_link_id_cur_geofence_.end()) {
            path_id_link_id_cur_geofence_[path_link_key] = 1;
        }
    }

    // connect
    std::string geojson_file_connect;
    geojson_file_connect = std::to_string(0) + "_connect.geojson";
    file_name_char_connect_ = geojson_file_connect.c_str();
    for (auto connect_info : map_static_info_->LaneConnectivitys.PairConnectivity) {
        uint64_t path_link_key =
            static_cast<uint64_t>(connect_info.PathId.PathId) << 32 | connect_info.InstanceId.InstanceId;
        if (path_id_link_id_connect_map_.find(path_link_key) != path_id_link_id_connect_map_.end()) {
            path_id_link_id_connect_map_[path_link_key] += 1;
            continue;
        }

        hdm_utility::CHdmDbgLog::GetInstance()->Write(
            file_name_char_connect_, true,
            "{\"type\":\"Feature\", \"properties\":{\
\"path_id\":\"%u\",\"link_id\":\"%u\",\"lanenumber\":\"%u\",\"offset\":\"%u\",\"fromlinkid\":\"%u\",\
\"initpath\":\"%u\",\"initlanenum\":\"%u\",\"tolinkid\":\"%u\",\"newpath\":\"%u\",\"newlanenum\":\"%u\", \"efm_counter\":\"%llu\" },\"geometry\":{\"type\":\"Point\",\"coordinates\":[0,  0]}},\n",
            connect_info.PathId.PathId, connect_info.InstanceId.InstanceId, connect_info.LaneNumber.LaneNumber,
            connect_info.PathOffset.PathOffset, connect_info.FromLinkId.FromLinkId, connect_info.InitPath.InitPath,
            connect_info.InitLaneNum.InitLaneNum, connect_info.ToLinkId.ToLink, connect_info.NewPath.NewPath,
            connect_info.NewLaneNum.NewLaneNum, efm_counter_);
    }

    for (auto connect_info : map_static_info_->LaneConnectivitys.PairConnectivity) {
        uint64_t path_link_key =
            static_cast<uint64_t>(connect_info.PathId.PathId) << 32 | connect_info.InstanceId.InstanceId;
        if (path_id_link_id_connect_map_.find(path_link_key) != path_id_link_id_connect_map_.end()) {
            path_id_link_id_connect_map_[path_link_key] += 1;
        } else {
            path_id_link_id_connect_map_[path_link_key] = 1;
        }
    }

    //_curvature
    std::string geojson_file_curvature;
    geojson_file_curvature = std::to_string(0) + "_curvature.geojson";
    file_name_char_curvature_ = geojson_file_curvature.c_str();
    for (auto curvature_info : map_static_info_->LinkCurvatures.LinkCurvatures) {
        uint64_t path_link_key =
            static_cast<uint64_t>(curvature_info.PathId.PathId) << 32 | curvature_info.InstanceId.InstanceId;
        if (path_id_link_id_cur_map_.find(path_link_key) != path_id_link_id_cur_map_.end()) {
            path_id_link_id_cur_map_[path_link_key] += 1;
            continue;
        }

        std::string curvature_list_char;
        for (int32_t i = 0; i < curvature_info.CurvCount.CurvCount; i++) {
            uint32_t offset = curvature_info.CurvPoints.CurvPoints[i].PathOffset.PathOffset;
            float value = curvature_info.CurvPoints.CurvPoints[i].CurvPointValue.CurvPointValue;
            curvature_list_char += std::to_string(offset);
            curvature_list_char += ",";
            curvature_list_char += std::to_string(value);
            if (i < (curvature_info.CurvCount.CurvCount - 1)) {
                curvature_list_char += "##";
            }
        }

        hdm_utility::CHdmDbgLog::GetInstance()->Write(
            file_name_char_curvature_, true,
            "{\"type\":\"Feature\", \"properties\":{\
\"path_id\":\"%u\",\"link_id\":\"%u\",\"lanenumber\":\"%u\",\"offset\":\"%u\",\"end_offset\":\"%u\",\
\"CurvCount\":\"%u\",\"curvature_list_char\":\"%s\", \"efm_counter\":\"%llu\" },\"geometry\":{\"type\":\"Point\",\"coordinates\":[0,  0]}},\n",
            curvature_info.PathId.PathId, curvature_info.InstanceId.InstanceId, curvature_info.LaneNum.LaneNum,
            curvature_info.PathOffset.PathOffset, curvature_info.EndOffset.EndOffset,
            curvature_info.CurvCount.CurvCount, curvature_list_char.c_str(), efm_counter_);
    }

    for (auto curvature_info : map_static_info_->LinkCurvatures.LinkCurvatures) {
        uint64_t path_link_key =
            static_cast<uint64_t>(curvature_info.PathId.PathId) << 32 | curvature_info.InstanceId.InstanceId;
        if (path_id_link_id_cur_map_.find(path_link_key) == path_id_link_id_cur_map_.end()) {
            path_id_link_id_cur_map_[path_link_key] = 1;
        }
    }


    //slop
    std::string geojson_file_slop;
    geojson_file_slop = std::to_string(0) + "_slop.geojson";
    file_name_char_slop_ = geojson_file_slop.c_str();
    for (auto slop_info : map_static_info_->LinkSlopes.LinkSlope) {
        uint64_t path_link_key =
            static_cast<uint64_t>(slop_info.PathId.PathId) << 32 | slop_info.InstanceId.InstanceId;
        if (path_id_link_id_slop_map_.find(path_link_key) != path_id_link_id_slop_map_.end()) {
            path_id_link_id_slop_map_[path_link_key] += 1;
            continue;
        }

        std::string slop_list_char;
        for (int32_t i = 0; i < slop_info.SlopeCount.SlopeCount; i++) {
            uint32_t offset = slop_info.SlopePoints.SlopePoints[i].PathOffset.PathOffset;
            float value = slop_info.SlopePoints.SlopePoints[i].SlopeValue.SlopeValue;
            float cvalue = slop_info.SlopePoints.SlopePoints[i].CrossValue.CrossValue;
            slop_list_char += std::to_string(offset);
            slop_list_char += ",";
            slop_list_char += std::to_string(value);
            slop_list_char += ",";
            slop_list_char += std::to_string(cvalue);
            if (i < (slop_info.SlopeCount.SlopeCount - 1)) {
                slop_list_char += "##";
            }
        }

        hdm_utility::CHdmDbgLog::GetInstance()->Write(
            file_name_char_slop_, true,
            "{\"type\":\"Feature\", \"properties\":{\
\"path_id\":\"%u\",\"link_id\":\"%u\",\"lanenumber\":\"%u\",\"offset\":\"%u\",\"end_offset\":\"%u\",\
\"CurvCount\":\"%u\",\"slop_list_char\":\"%s\", \"efm_counter\":\"%llu\" },\"geometry\":{\"type\":\"Point\",\"coordinates\":[0,  0]}},\n",
            slop_info.PathId.PathId, slop_info.InstanceId.InstanceId, slop_info.NumOfLane.NumOfLane,
            slop_info.PathOffset.PathOffset, slop_info.EndOffset.EndOffset,
            slop_info.SlopeCount.SlopeCount, slop_list_char.c_str(), efm_counter_);
    }

    for (auto slop_info : map_static_info_->LinkSlopes.LinkSlope) {
        uint64_t path_link_key =
            static_cast<uint64_t>(slop_info.PathId.PathId) << 32 | slop_info.InstanceId.InstanceId;
        if (path_id_link_id_slop_map_.find(path_link_key) == path_id_link_id_slop_map_.end()) {
            path_id_link_id_slop_map_[path_link_key] = 1;
        }
    }

    return true;
}

bool CDataToJson::DownPositionToGeoJson(const TopicTrait::MapPositionMsg& position_msg,
                                        const TopicTrait::MapRouteListMsg& map_route_list) {
    std::string geojson_file_position;
    geojson_file_position = std::to_string(0) + "_position.geojson";
    file_name_char_position_ = geojson_file_position.c_str();

    position_msg_ = std::make_shared<const TopicTrait::MapPositionMsg>(position_msg);
    map_route_list_ = std::make_shared<const TopicTrait::MapRouteListMsg>(map_route_list);

    std::string map_route_list_char;
    for (int32_t i = 0; i < map_route_list_->LinkCount.LinkCount; i++) {
        uint32_t linkid = map_route_list_->LinkIds.LinkIds[i].LinkId;
        map_route_list_char += std::to_string(linkid);
        if (i < (map_route_list_->LinkCount.LinkCount - 1)) {
            map_route_list_char += ",";
        }
    }

    hdm_utility::CHdmDbgLog::GetInstance()->Write(
        file_name_char_position_, true,
        "{\"type\":\"Feature\", \"properties\":{\
\"path_id\":\"%u\",\"link_id\":\"%u\",\"offset\":\"%u\",\"lane_id\":\"%u\",\"timestamp\":\"%llu\",\
\"Heading\":\"%f\",\"Speed\":\"%f\",\"LocStatus\":\"%u\",\"GeoFenceJudgeStatus\":\"%u\",\"Lon\":\"%u\",\"Lat\":\"%u\",\
\"GeoFenceJudgeType\":\"%u\", \"map_route_list\":\"%s\", \"efm_counter\":\"%llu\", \"position_counter\":\"%llu\" },\"geometry\":{\"type\":\"Point\",\"coordinates\":[%lf,  %lf]}},\n",
        position_msg_->Position.PathId, position_msg_->Position.LinkId, position_msg_->Position.PathOffset,
        position_msg_->Position.LaneId, position_msg_->header.timestamp,
        static_cast<double>(position_msg_->Position.Heading.Heading),
        static_cast<double>(position_msg_->Position.Speed), position_msg_->Position.FailSafeLocStatus.data,
        position_msg_->Position.GeoFenceJudgeStatus.data, 
        position_msg_->Position.Lon.Lon,
        position_msg_->Position.Lat.Lat,
        position_msg_->Position.GeoFenceJudgeType.data,
        map_route_list_char.c_str(), efm_counter_, position_counter_,
        static_cast<double>(position_msg_->Position.Lon.Lon) * 360.0 / (pow(2, 32)),
        static_cast<double>(position_msg_->Position.Lat.Lat) * 360.0 / (pow(2, 32)));

    return true;
}

bool CDataToJson::DownSwitchToGeoJson(const TopicTrait::MapSwitchInfoMsg& switch_info_msg,
                                      const TopicTrait::MapMapMsg& map_msg) {
    switch_info_msg_ = std::make_shared<const TopicTrait::MapSwitchInfoMsg>(switch_info_msg);
    map_static_info_ = std::make_shared<const TopicTrait::MapMapMsg>(map_msg);
    std::string geojson_file_switch;
    geojson_file_switch = std::to_string(0) + "_switch.geojson";
    file_name_char_switch_ = geojson_file_switch.c_str();

    std::string geojson_file_switch_global;
    geojson_file_switch_global = std::to_string(0) + "_switch_global.geojson";
    file_name_char_switch_global_ = geojson_file_switch_global.c_str();

    std::string path_ss;
    std::string link_ss;
    std::string lanenumber_ss;
    std::string liner_ss;
    for (auto pair : switch_info_msg_->NOAInfo.PairOfIds.PairOfIds) {
        path_ss += std::to_string(pair.PathId) + ", ";
        link_ss += std::to_string(pair.InstanceId) + ", ";
        lanenumber_ss += std::to_string(pair.LaneNumber) + ", ";
        liner_ss += std::to_string(pair.LinearObjectId) + ", ";
    }

    hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_switch_, true,
                                                  "{\"type\":\"Feature\", \"properties\":{\
\"path_ids\":\"%s\",\"link_ids\":\"%s\",\"lanenumbers\":\"%s\",\"liners\":\"%s\",\"efm_counter\":\"%llu\" },\"geometry\":{\"type\":\"Point\",\"coordinates\":[0,  0]}},\n",
                                                  path_ss.c_str(), link_ss.c_str(), lanenumber_ss.c_str(),
                                                  liner_ss.c_str(), efm_counter_);
    if (!map_static_info_->LinearObjects.LinearObjects.empty()) {
        for (auto pair : switch_info_msg_->NOAInfo.PairOfIds.PairOfIds) {
            uint32_t lineatobjectid = pair.LinearObjectId;
            if (lineatobject_id_cur_map_.find(lineatobjectid) != lineatobject_id_cur_map_.end()) {
                lineatobject_id_cur_map_[lineatobjectid] += 1;
                continue;
            } else {
                lineatobject_id_cur_map_[lineatobjectid] = 1;
            }

            if (0 == write_times_global_) {
                write_times_global_ += 1;
            } else {
                hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_switch_global_, true, ",\n");
            }
            for (auto lineobj : map_static_info_->LinearObjects.LinearObjects) {
                uint32_t instance_global_id_ = lineobj.InstanceId.InstanceId;
                uint32_t path_global_id_ = lineobj.PathId.PathId;
                uint32_t s_global_offset_ = lineobj.PathOffset.PathOffset;
                uint32_t e_global_offset_ = lineobj.EndOffset.EndOffset;

                if (lineobj.IDLinearObject.IDLinearObject == pair.LinearObjectId) {
                    hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_switch_global_, false,
                                                                  "{\"type\":\"Feature\", \"properties\":{\
\"path_id\":\"%u\",\"link_id\":\"%u\",\"s_offset\":\"%u\",\"e_offset\":\"%u\",",
                                                                  path_global_id_, instance_global_id_,
                                                                  s_global_offset_, e_global_offset_);

                    hdm_utility::CHdmDbgLog::GetInstance()->Write(
                        file_name_char_switch_global_, false,
                        "\"ObjectType\":\"%u\",\"ObjectMarking.\":\"%u\",\"ObjectColour\":\"%u\",\"ObjectIsBold\":\"%"
                        "u\",\"ObjectCurveType\":\"%u\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[",
                        lineobj.LinearObjectType.data, lineobj.LinearObjectMarking.data,
                        lineobj.LinearObjectColour.data, lineobj.LinearObjectIsBold.data,
                        lineobj.LinearObjectCurveType.data);
                    for (size_t point_idx = 0; point_idx < lineobj.PointCount.PointCount; point_idx++) {
                        if (0 == point_idx) {
                            hdm_utility::CHdmDbgLog::GetInstance()->Write(
                                file_name_char_switch_global_, false, " [ %lf,  %lf]",
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Longitude.Longitude) *
                                    360.0 / (pow(2, 32)),
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Latitude.Latitude) *
                                    360.0 / (pow(2, 32)));
                        } else {
                            hdm_utility::CHdmDbgLog::GetInstance()->Write(
                                file_name_char_switch_global_, false, " ,[ %lf,  %lf]",
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Longitude.Longitude) *
                                    360.0 / (pow(2, 32)),
                                static_cast<double>(
                                    lineobj.GeometryPoints.GeometryPoints[point_idx].Latitude.Latitude) *
                                    360.0 / (pow(2, 32)));
                        }
                    }
                    hdm_utility::CHdmDbgLog::GetInstance()->Write(file_name_char_switch_global_, true, "]}}");
                }
            }
        }
    }

    return true;
}

bool CDataToJson::DownRouteToGeoJson(const TopicTrait::MapRouteListMsg& map_route_list) {
    map_route_list_ = std::make_shared<const TopicTrait::MapRouteListMsg>(map_route_list);
    return true;
}

bool CDataToJson::DownGlobalToGeoJson(const TopicTrait::MapGlobalDataMsg& global_data_msg) {
    global_data_msg_ = std::make_shared<const TopicTrait::MapGlobalDataMsg>(global_data_msg);
    return true;
}

bool CDataToJson::DownDynamicToGeoJson(const TopicTrait::MapDynamicMsg& map_dynamic_msg) {
    map_dynamic_msg_ = std::make_shared<const TopicTrait::MapDynamicMsg>(map_dynamic_msg);
    return true;
}

bool CDataToJson::GenerateData(const TopicTrait::MapMapMsg& map_msg, const TopicTrait::MapPositionMsg& position_msg,
                               const TopicTrait::MapSwitchInfoMsg& switch_info_msg,
                               const TopicTrait::MapRouteListMsg& map_route_list,
                               const TopicTrait::MapGlobalDataMsg& global_data_msg,
                               const TopicTrait::MapDynamicMsg& map_dynamic_msg, uint64_t efm_counter) {
    efm_counter_ = efm_counter;
    position_counter_ = position_msg.header.cntr;

    DownMapToGeoJson(map_msg);
    DownPositionToGeoJson(position_msg, map_route_list);
    DownSwitchToGeoJson(switch_info_msg, map_msg);
    DownRouteToGeoJson(map_route_list);
    DownGlobalToGeoJson(global_data_msg);
    DownDynamicToGeoJson(map_dynamic_msg);

    return true;
}

}  // namespace framework
}  // namespace shell
}  // namespace earth
