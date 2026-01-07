/*
 * @Author: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @Date: 2023-12-19 20:29:06
 * @LastEditors: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @LastEditTime: 2024-04-28 17:00:05
 * @FilePath: /environmentmodelfunction/include/CommonDataType.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <stdint.h>
#include <vector>
#include <map>
#include <array>
#include <algorithm>
#include <unordered_map>

#include "message.h"


extern std::array<uint8_t,8> hist_camera_spd_limit_;

typedef enum { RoadChangeType_NONE = 0, RoadChangeType_MERGE = 1, RoadChangeType_SPLIT = 2 } RoadChangeType;
typedef enum { DIRECTIONTYPE_UNKNOW = 0, DIRECTIONTYPE_LEFT = 1, DIRECTIONTYPE_RIGHT = 2 } DirectionType;

typedef enum {
    MergeType_NONE = 0,
    MergeType_MERGE_TO_LEFT = 1,
    MergeType_MERGE_TO_RIGHT = 2,
    MergeType_MERGE_FROM_LEFT = 3,
    MergeType_MERGE_FROM_RIGHT = 4
} MergeType_e;

typedef enum {
    EFM_MergeType_NONE = 0,
    EFM_MergeType_TO_LEFT = 1,
    EFM_MergeType_FROM_LEFT = 2,
    EFM_MergeType_LEFT_TO_MIDDLE = 3,
    EFM_MergeType_TO_RIGHT = 4,
    EFM_MergeType_FROM_RIGHT = 5,
    EFM_MergeType_RIGHT_TO_MIDDLE = 6
} EfmMergeType;

typedef enum {
    EFM_SplitType_NONE = 0,
    EFM_SplitType_TO_LEFT = 1,
    EFM_SplitType_FROM_LEFT = 2,
    EFM_SplitType_CONTIUE_FROM_LEFT = 3,
    EFM_SplitType_TO_RIGHT = 4,
    EFM_SplitType_FROM_RIGHT = 5,
    EFM_SplitType_SPLIT_FROM_LEFT = 6,
    EFM_SplitType_CONTIUE_FROM_RIGHT = 7,
    EFM_SplitType_SPLIT_FROM_RIGHT = 8
} EfmSplitType;

typedef enum {
    EnumLaneType_NORMAL_ = 0,
    EnumLaneType_ENTRY_ = 1,
    EnumLaneType_EXIT_ = 2,
    EnumLaneType_EMERGENCY_ = 3,
    EnumLaneType_ON_RAMP_ = 4,
    EnumLaneType_OFF_RAMP_ = 5,
    EnumLaneType_CONNECT_RAMP_ = 6,
    EnumLaneType_ACCELERATE_ = 7,
    EnumLaneType_DECELERATE_ = 8,
    EnumLaneType_EMERGENCY_PARKING_STRIP_ = 9,
    EnumLaneType_RESERVE0_ = 10,
    EnumLaneType_RESERVE1_ = 11,
    EnumLaneType_RESERVE2_ = 12,
    EnumLaneType_RESERVE3_ = 13,
    EnumLaneType_RESERVE4_ = 14,
    EnumLaneType_DIVERSION_ = 15,
    EnumLaneType_RESERVE5_ = 16,
    EnumLaneType_RESERVE6_ = 17,
    EnumLaneType_RESERVE7_ = 18,
    EnumLaneType_RESERVE8_ = 19,
    EnumLaneType_RESERVE9_ = 20,
    EnumLaneType_RESERVE10_ = 21
} LaneType_e;

typedef struct {
    double distance;
    RoadChangeType type;
} RaodChange;

typedef struct EFMPoint2D {
    EFMPoint2D(double x_in, double y_in) : x(x_in), y(y_in){};
    EFMPoint2D() : x(0.0), y(0.0){};
    ~EFMPoint2D() = default;
    double x;
    double y;
    //点+
    EFMPoint2D operator +(const EFMPoint2D &b)const{
        return EFMPoint2D(x+ b.x, y+b.y);
    }
    //点-
    EFMPoint2D operator -(const EFMPoint2D &b)const{
        return EFMPoint2D(x- b.x, y-b.y);
    }
    //点积
    double operator *(const EFMPoint2D &b)const{
        return x* b.x + y*b.y;
    }
    //叉乘
    double operator ^(const EFMPoint2D &b)const{
        return x* b.y - y*b.x;
    }
} EFMPoint;

typedef struct {
    double x;
    double y;
} GCJ02Point;

typedef enum {
    Unknown = 0,
    None = 1,
    SolidLine = 2,
    DashedLine = 3,
    DoubleSolidLine = 4,
    DoubleDashedLine = 5,
    LeftSolidRightDashed = 6,
    RightSolidLeftDashed = 7,
    Virtual = 8
} EFMMarkrType_e;  //== map map

typedef enum {
    None_ = 0,
    Other_ = 1,
    White_ = 2,
    Yellow_ = 3,
    Orange_ = 4,
    Red_ = 5,
    Blue_ = 6
} EFMMarkrColor_e;  // ==map map

struct CurvPoint{
    CurvPoint()
    : CurvPointValue (0.0f)
    , CurvPointPathOffset (0)
    , linkid (0){};
    ~CurvPoint() = default;

    float CurvPointValue;
    uint32_t CurvPointPathOffset;
    uint32_t linkid;
};

struct MapRawDataMap{
    MapRawDataMap()
    : link_id_index_lane_info_map{}
    , link_id_index_lane_connect_map{}
    , to_link_id_index_lane_connect_map{}
    , route_list_link_ids{}
    , linear_obj_id_map{}
    , curve_index_map{}
    , lane_widths_map{}{};
    ~MapRawDataMap() = default;

    std::unordered_map<uint32_t, int> link_id_index_lane_info_map;
    std::unordered_map<uint32_t, std::vector<int>> link_id_index_lane_connect_map;
    std::unordered_map<uint32_t, std::vector<int>> to_link_id_index_lane_connect_map;
    std::vector<uint32_t>  route_list_link_ids;
    std::unordered_map<uint32_t, int> linear_obj_id_map;//key,IDLinearObject; val在LinearObjects里的index
    std::unordered_map<uint64_t, int> curve_index_map;//key 高32lane_id,低32 link id; val在LinkCurvatures里的index
    std::unordered_map<uint64_t, int> lane_widths_map;//key 高32lane_id,低32 link id; val在s_LaneWidths_t里的index
};

struct PreCycleData{
    PreCycleData()
    : last_merge_back_links_id{}
    , last_merge_ego_back_lanes_id{}
    , last_merge_side_back_lanes_id{}
    , last_split_line_id_lane_id{}
    , last_map_map_counter(0){};
    ~PreCycleData() = default;

    std::vector<uint32_t> last_merge_back_links_id;
    std::vector<uint8_t> last_merge_ego_back_lanes_id;
    std::vector<uint8_t> last_merge_side_back_lanes_id;
    std::map<uint64_t,uint8_t> last_split_line_id_lane_id;
    uint64_t last_map_map_counter;
};

struct MapRawData{
    MapRawData()
    : ego_linkid_routelist_idx(0)
    , veh_point_wgs84_x(0.0)
    , veh_point_wgs84_y(0.0)
    , veh_point_utm_x(0.0)
    , veh_point_utm_y(0.0)
    , re_navigate_map_map_cntr(0){};
    ~MapRawData() = default;

    int ego_linkid_routelist_idx;
    double veh_point_wgs84_x;
    double veh_point_wgs84_y;
    double veh_point_utm_x;
    double veh_point_utm_y;
    uint64_t re_navigate_map_map_cntr;
};

typedef struct SplitInfoS{
    SplitInfoS()
    : is_road_split(false)
    , s_offset(0)
    , e_offset(0)
    , split_dir(EFM_SplitType_NONE){};
    SplitInfoS(bool x,uint32_t y, uint32_t z, EfmSplitType dir)
    : is_road_split(x)
    , s_offset(y)
    , e_offset(z)
    , split_dir(dir){};
    ~SplitInfoS() = default;
    bool is_road_split;
    uint32_t s_offset;
    uint32_t e_offset;
    EfmSplitType split_dir;
}SplitInfo_S;

struct MainFuncStaticVar{
    MainFuncStaticVar()
    : counter_position_lost(0)
    , loge_counter(0)
    , map_position_counter(0)
    , map_map_counter(0)
    , map_switch_counter(0)
    , map_route_counter(0)
    , map_global_counter(0)
    , map_dynamic_counter(0)
    , virtual_line_split_link_id(0)
    , last_path_id(0)
    , errorcode(0) {};
    ~MainFuncStaticVar() = default;

    uint32_t counter_position_lost;
    uint32_t loge_counter;
    uint64_t map_position_counter;
    uint64_t map_map_counter;
    uint64_t map_switch_counter;
    uint64_t map_route_counter;
    uint64_t map_global_counter;
    uint64_t map_dynamic_counter;
    uint32_t virtual_line_split_link_id;
    uint32_t last_path_id;
    uint64_t errorcode;
};

struct LaneExtraInfo_s {
    LaneExtraInfo_s()
    : merge_value(EFM_MergeType_NONE)
    , split_value(EFM_SplitType_NONE)
    , curvpoints()
    , s_offset(0)
    , e_offset(0)
    , s_offset_raw(0)
    , e_offset_raw(0)
    , lane_type(0)
    , transit(1)
    , is_road_merge(false)
    , road_merge_s_e_offset(0,0) {};
    ~LaneExtraInfo_s() = default;

    EfmMergeType merge_value;
    EfmSplitType split_value;
    std::vector<CurvPoint> curvpoints;
    int32_t s_offset;
    int32_t e_offset;
    uint32_t s_offset_raw;//没有减去自车的offset
    uint32_t e_offset_raw;//没有减去自车的offset
    uint8_t lane_type;
    uint8_t transit;
    bool is_road_merge;
    std::pair<int32_t,int32_t> road_merge_s_e_offset;//first-start;second-end, 减去了自车的offset
};

typedef enum {
    NONE = 0,
    MERGE = 1,
    SPLIT = 2,
    LANE_END = 3,
    ENTRY = 4,
    EXIT = 5
} LaneChangeType_e;
typedef struct EFMMarkr2D {
    EFMMarkr2D(double x_in, double y_in, uint8_t type_in, uint8_t color_in)
        : x(x_in), y(y_in), type(type_in), color(color_in){};
    EFMMarkr2D() : x(0.0), y(0.0), type(0), color(0){};
    ~EFMMarkr2D() = default;
    double x;
    double y;
    uint8_t type;  //EFMMarkrType_e
    uint8_t color; //EFMMarkrColor_e
} EFMMarkr;

typedef struct LineMkrInfo {
    LineMkrInfo(uint32_t x_in, uint32_t y_in, uint8_t type_in, uint8_t color_in)
        : path_offset(x_in), end_offset(y_in), type(type_in), color(color_in){};    
    LineMkrInfo() : path_offset(0), end_offset(0),type(0),color(0){};
    ~LineMkrInfo() = default;
    uint32_t path_offset;
    uint32_t end_offset;
    uint8_t type;
    uint8_t color;
} LineMkrInfo_s;

using EFMRefLinePoints = std::vector<EFMPoint>;
using EFMRefLineMarkr = std::vector<EFMMarkr>;
using EFMRefLineMarkrSection = std::array<std::vector<EFMMarkr>, 2>;
using EFMRefLinePointsSection = std::array<std::vector<EFMPoint>, 2>;

typedef struct SmoothSplitInfo_s {   
    SmoothSplitInfo_s() : dist(0)
    ,road_class(0)
    ,link_id{}
    ,lane_id{}
    ,side_link_id{}
    ,side_lane_id{}
    ,link_length{}
    ,side_link_length{}
    ,lane_curvpoints{}
    ,side_lane_curvpoints{}
    ,left_line{}
    ,right_line{}
    ,side_left_line{}
    ,side_right_line{}
    ,max_width{}
    ,side_max_width{}
    ,min_width{}
    ,side_min_width{}
    ,before_split_max_width{}
    ,before_split_min_width{}
    ,before_split_left_line{}
    ,before_split_right_line{}
    ,before_link_length{}
    ,before_lane_curvpoints{}{};
    ~SmoothSplitInfo_s() = default;
    double dist;//距离自车位置m
    uint8_t road_class;//1- highway
    //split后方的信息
    // std::vector<double> link_lengths;//向后2段link长度m
    std::vector<uint32_t> link_id;//向后2段link id
    std::vector<uint8_t> lane_id;//向后2段的自身道路的lane id
    std::vector<uint32_t> side_link_id;//向后2段的split临车道的lane id
    std::vector<uint8_t> side_lane_id;//向后2段的split临车道的lane id
    std::vector<double> link_length;//向后2段link 长度
    std::vector<double> side_link_length;//向后2段link 长度
    std::vector<std::vector<std::pair<double,double>>> lane_curvpoints;//first-offset to ego; second- curvature value
    std::vector<std::vector<std::pair<double,double>>> side_lane_curvpoints;
    std::vector<uint8_t> left_line;//向后2段自身车道的左边线类型
    std::vector<uint8_t> right_line;//向后2段自身车道右边线类型
    std::vector<uint8_t> side_left_line;//向后2段自身车道的左边线类型
    std::vector<uint8_t> side_right_line;//向后2段自身车道右边线类型
    std::vector<uint16_t> max_width;
    std::vector<uint16_t> side_max_width;
    std::vector<uint16_t> min_width;
    std::vector<uint16_t> side_min_width;
    std::vector<uint8_t> speed_limit;
    std::vector<uint8_t> side_speed_limit;
    std::vector<uint8_t> lane_type;
    std::vector<uint8_t> side_lane_type;    
    //split前方的信息
    std::vector<uint16_t> before_split_max_width;//[0]靠近split
    std::vector<uint16_t> before_split_min_width;//[0]靠近split
    std::vector<EFMRefLineMarkr> before_split_left_line;//[0]靠近split
    std::vector<EFMRefLineMarkr> before_split_right_line;//[0]靠近split
    std::vector<double> before_link_length;//向后2段link 长度
    std::vector<std::vector<std::pair<double,double>>> before_lane_curvpoints;
    std::vector<uint8_t> before_speed_limit;
    std::vector<uint8_t> before_lane_type;
} SmoothSplitInfo;

typedef struct LaneLineOffsetInfo{
    uint8_t left_line_type;
    uint8_t right_line_type;
    uint32_t start_offset;
    uint32_t end_offset;
}LaneLineOffsetInfo_s;

typedef struct {
    EFMRefLinePoints linePoints;//原始点
    std::vector<uint32_t> linePointsOffset;//原始点offset
    EFMRefLineMarkr leftMarkr;
    std::vector<uint32_t> leftMarkrOffset;
    EFMRefLineMarkr rightMarkr; 
    std::vector<uint32_t> rightMarkrOffset;   
    EFMRefLinePointsSection linePointsSection;//分成两段的，并且插值了的
    EFMRefLineMarkrSection leftMarkrSection;  // 0-back,1-front, first point is proj point
    EFMRefLineMarkrSection rightMarkrSection;
    uint8_t nXMin;
    uint8_t nXMax;
    bool bIsAvailable;
    uint8_t nLaneNumber;    // 当前所在道路的车道数
    uint8_t nLaneIndex;     // 三条车道位于当前道路的索引值
    uint8_t nEgoLaneIndex;  // 自车所在当前道路车道的索引值
    RaodChange roadChange;  // 车道变化
    // s_Construction_t construction;
    double dLaneWidthMin;
    double dLaneWidthMax;
    double dSpeedMax;
    double dSpeedMin;
    uint8_t LaneType;
    //::std::array<s_Curvatures_t, 21> curvatures;
    // s_BusMergePoint_t MergePoint;
    // s_SpecialSit_t mSpecialSit;
    // uint8_t nTunnelCount;
    // s_RampInfo_t mRampInfo;
    // e_SwitchLaneReason_t SwitchLaneReason;
    MergeType_e merge_type;
    double merge_dist; //unit m
    bool is_ref_line;
    int  split_position_index;
    int  split_in_point_index;
    uint8_t split_from;  // 0:none;1:left;2:right
	std::vector<double> laneMarkSectionLength;  // only front
    std::vector<std::pair<double, int>>
        sum_curvature_vec;  // first - sum of curvature, second - curvature points number, // only front
    std::vector<LineMkrInfo_s> leftLineMkrInfos;  //自车当前以及前方的link
    std::vector<LineMkrInfo_s> rightLineMkrInfos;  //自车当前以及前方的link
    int  next_position_index; //100m last split
    int  next_in_point_index; //100m last split
    std::vector<double> back_link_offsets; //自车尾部的link
    std::vector<LineMkrInfo_s> back_link_left_line_infos;//自车尾部的link
    std::vector<LineMkrInfo_s> back_link_right_line_infos;//自车尾部的link
    int back_lane_split_to; //0 none; 1:left; 2:right;
    bool back_lane_is_split;
    int back_lane_split_point_index;
    std::vector<std::pair<double,double>> split_start_s_vec; //ego_pos is ori point, unit m;自车后方是负值; second- split link length, first + second 是split end的距离
    std::vector<std::pair<double,double>> merge_end_s_vec; //ego_pos is ori point, unit m； second- second- split link length, first - second 是merge start的距离
    std::vector<SmoothSplitInfo> smooth_split_info;
    std::vector<LaneLineOffsetInfo_s> line_type_offset;//按link分，[0]是最尾部的点所在，<<left_type,right_type>,<start_offset,end_offset>>
} EFMRefLine;

typedef enum{
    LaneChangeDirection_NONE = 0,
    LaneChangeDirection_LEFT = 1,
    LaneChangeDirection_RIGHT =2
}LaneChangeDirection_e;

typedef struct{
    uint32_t link_id;
    uint8_t lane_num;
}LanesNode;

typedef struct{
    LanesNode origin_node;
    std::vector<LanesNode> split_nodes;
}SplitNode;

typedef struct {
    std::vector<uint32_t> linkids;
    uint32_t first_pathoffset;
    uint32_t last_endoffset;
    uint8_t geo_fence_type;
}GeoFenceSegment;

enum EEFMLaneChgType : uint8_t
{
    WITH_CONTROL = 1,
    WITH_LIGHT   = 1 << 1,
    OFF_RAMP     = 1 << 2,
    ON_RAMP      = 1 << 3,
};

struct LaneElement{
    LaneElement()
    : lane_num_vec{}
    , transit_type{}
    , lane_extra_infos{}
    , rest_length(0)
    , org_rest_length(0.0)
    , candidate_index(-1)
    , close_times(0)
    , lane_change_types{} 
    , is_dest(false)
    , min_step(0)
    , is_close_dest(false)
    , is_group_dest(true)
    , split_position(100)
    , is_virtually_split(false)
    , split_from(0)
    , split_offset(0)
    , next_split_position(100)
    , close_position(100)
    , is_virtually_close(false)
    , close_to(0)
    , dir_type(DIRECTIONTYPE_UNKNOW)
    , change_times(0)
    , lane_change_position(0.0)
    , first_lane_is_no_route(false) {};
    ~LaneElement() = default;

    std::vector<uint8_t> lane_num_vec;
	std::map<int, uint8_t> transit_type;//key- index in lane_num_vec; value- transit type
    std::vector<LaneExtraInfo_s> lane_extra_infos;//lane_extra_infos.size = lane_num_vec.size
    // std::vector<EfmSplitType> transit_split_type;//transit_split_type.size = lane_num_vec.size
    // std::vector<int32_t> lane_s_offset;//lane_s_offset.size = lane_num_vec.size
    // std::vector<int32_t> lane_e_offset;//lane_e_offset.size = lane_num_vec.size
    //std::map<int, uint8_t> transit_type;//key- index in lane_num_vec; value- transit type
    double  rest_length;      //经过判断是否是实线推算的长度
    double  org_rest_length;  //未经过推算的原始可行驶长度
    int32_t candidate_index;//index in all_lanes_vec_vec_
    uint32_t close_times;
    uint8_t  lane_change_types; // EEFMLaneChgType
    bool is_dest;
    uint8_t min_step;
    bool is_close_dest;
    bool is_group_dest;
    int32_t split_position;
    bool    is_virtually_split;
    uint8_t split_from;  // 0:none;1:left;2:right
    uint32_t split_offset;  // 
    int32_t next_split_position; //from split_position 100m
    int32_t close_position;
    bool    is_virtually_close;
    uint8_t close_to;  // 0:none;1:left;2:right
    DirectionType dir_type;
    int32_t change_times;
    double lane_change_position;
    bool   first_lane_is_no_route;
};

struct BigWidthLane{
    BigWidthLane()
    : link_id (0)
    , link_pathoffset (0)
    , link_endoffset(0){};
    ~BigWidthLane() = default;

    uint32_t link_id;
    uint32_t link_pathoffset;
    uint32_t link_endoffset;
};

struct OddDis{
    OddDis()
    : oddtype (0)
    , odddis (0){};
    ~OddDis() = default;

    uint8_t oddtype;
    uint32_t odddis;
};

struct SubModuleOutput{
    SubModuleOutput()
    : node_info{}
    , linePoints{}
    , ego_path{}
    , left_path{}
    , right_path{}
    , prior_path_change_type{NONE}
    , left_extra_info{}
    , ego_extra_info{}
    , right_extra_info{}
    , left_left_extra_info{}
    , right_right_extra_info{}
    , geofence_segments{}
    , valid_geofence_segments{}
    , link_id_vec{}
    , ego_path_lane_num{}
    , left_path_lane_num{}
    , right_path_lane_num{}
    , left_left_path_lane_num{}
    , right_right_path_lane_num{}
    , candidate_lanes_split_nodes{}
    , is_contain_split_vec{}
    , opt_kappas{}
    , opt_accumulated_s{}
    , exit_scene_speed_limit{}
    , bypass_merge_distance(0.0)
    , entry_dist(0.0)
    , entry_start_dist(0.0)
    , entry_end_dist(0.0)
    , exit_start_dist(0.0)
    , exit_end_dist(0.0)
    , rest_dist(0.0)
    , is_has_big_curvature(false)
    , is_y_shape_link(false)
    , is_has_virtual_lane(false)
    , exit_start_link_index(0)
    , prior_path_index(0)
    , fixed_lane_id(0)
    , in_odd_type(0)
    , driveable_lane_size(0)
    , cur_speed(0)
    , next_speed(0)
    , exit_type(0)
    , pntIdx(0)
    , odd_end_dist(0)
    , split_info_map{}
    , split_info_deque{}
    , pre_cutin_merge_offset{} {};
    // , pGetStoredSplitInfo(NULL)
    ~SubModuleOutput() = default;

    //  void operator()() const {}

    message::efm::s_NodeInfo_t node_info;
    EFMRefLinePoints linePoints;
    EFMRefLine ego_path;
    EFMRefLine left_path;
    EFMRefLine right_path;
    LaneChangeType_e prior_path_change_type;
    std::vector<LaneExtraInfo_s> left_extra_info;
    std::vector<LaneExtraInfo_s> ego_extra_info;
    std::vector<LaneExtraInfo_s> right_extra_info;
    std::vector<LaneExtraInfo_s> left_left_extra_info;
    std::vector<LaneExtraInfo_s> right_right_extra_info;
    std::vector<GeoFenceSegment> geofence_segments;
    std::vector<GeoFenceSegment> valid_geofence_segments;
    std::vector<uint32_t>  link_id_vec;
    std::vector<uint8_t> ego_path_lane_num;
    std::vector<uint8_t> left_path_lane_num;
    std::vector<uint8_t> right_path_lane_num;
    std::vector<uint8_t> left_left_path_lane_num;
    std::vector<uint8_t> right_right_path_lane_num;
    std::map<uint32_t, SplitNode> candidate_lanes_split_nodes;
    std::vector<bool> is_contain_split_vec;
    std::vector<double> opt_kappas;
    std::vector<double> opt_accumulated_s;
    std::vector<std::pair<double, double>> exit_scene_speed_limit;
    double bypass_merge_distance;
    double entry_dist;
    double entry_start_dist;
    double entry_end_dist;
    double exit_start_dist;
    double exit_end_dist;
    double rest_dist;
    bool is_has_big_curvature;
    bool is_y_shape_link;
    bool is_has_virtual_lane;
    int exit_start_link_index;
    int prior_path_index;
    uint8_t fixed_lane_id;
    uint8_t in_odd_type;
    uint8_t driveable_lane_size;
    uint8_t cur_speed;
    uint8_t next_speed;
    uint8_t exit_type;
    uint16_t pntIdx;
    uint32_t odd_end_dist;
    std::map<uint64_t,SplitInfo_S> split_info_map;
    std::deque<uint64_t> split_info_deque;
    std::vector<uint32_t> pre_cutin_merge_offset;
    // bool (*pGetStoredSplitInfo)(const std::map<uint64_t,SplitInfo_S>&,const std::deque<uint64_t>& , uint8_t , uint32_t , SplitInfo_S& );
};

enum EEFMReturnCode : uint64_t
{
    EFM_CODE_EFM_NO_DATA                    = 1,        // 0 自车link不在静态图和动态图中
    EFM_CODE_EHP_NO_NAVIGATION_STATUS       = 1 << 1,   // 1 NavigationStatus状态值不为2
    EFM_CODE_EHP_NO_MATCHING_STATUS         = 1 << 2,   // 2 MatchingStatus状态值不为1、2、3
    EFM_CODE_EHP_NO_LOC_STATUS              = 1 << 3,   // 3 FailSafeLocStatus状态值不为0、1、2或者map_position_offset为0
    EFM_CODE_EHP_NO_STATIC_MAP              = 1 << 4,   // 4 NO MAP
    EFM_CODE_EHP_NO_POSITION_ERROR          = 1 << 5,   // 5 NO POSITION
    EFM_CODE_EHP_NO_SWITCH_ERROR            = 1 << 6,   // 6 NO SWITCH
    EFM_CODE_EHP_NO_ROUTE_ERROR             = 1 << 7,   // 7 NO ROUTE
    EFM_CODE_EHP_NO_GLOBAL_ERROR            = 1 << 8,   // 8 NO GLOBAL
    EFM_CODE_EHP_NO_DYNAMIC_ERROR           = 1 << 9,   // 9 NO DYNAMIC
    EFM_CODE_EHP_POSITION_ERROR             = 1 << 10,  // 10 POSITION INFO ERROR
    EFM_CODE_EHP_NO_LINK_INFOS              = 1 << 11,  // 11 NO LINK INFOS
    EFM_CODE_EHP_NO_CONNECT_INFOS           = 1 << 12,  // 12 NO_CONNECT_INFOS
    EFM_CODE_EHP_PATH_ID_NOT_MATCH          = 1 << 13,  // 13 自车的pathid和routelist里的link的pathid不匹配
    EFM_CODE_EFM_LANE_TREE_ERROR            = 1 << 14,  // 14 MakeLaneTree return false 
    EFM_CODE_EFM_MAKE_ALGO_ERROR            = 1 << 15,  // 15 MakeAlgo return false  
    EFM_CODE_EFM_MAKE_OUTPUT_ERROR          = 1 << 16,  // 16 MakeCenterLane, MakeSideLane return false 
    EFM_CODE_EFM_SPEED_LIMIT_ERROR          = 1 << 17,   // 17 MakeGuide return false
    EFM_CODE_EFM_NARROW_LANE_CLOSE_ERROR    = 1 << 18,   // narrow lane near close point
    EFM_CODE_EHP_POSITON_COUNTER_REPEAT_ERROR    = 1 << 19,   // position counter repeat more than 6 times
    // EFM_CODE_INIT_FAILED = 1 << 4,             // 4 EHR未完成初始化 无
    // EFM_CODE_INDEXEHP_ERROR = 1 << 5,             // 5 EHR创建索引错误 8
    // EHR_CODE0_FIND_ERROR = 1 << 6,              // 6 EHR查找车道数据失败 9
    // EHR_CODE0_LOC_STATUS_ERROR = 1 << 7,        // 7 EHP吐出地图定位失效 21
    // EHR_CODE0_NO_SECTION_DATA = 1 << 8,         // 8 EHR section数据查找失败 11
    // EHR_CODE0_NULLPTR = 1 << 9,                 // 9 EHR 返回空指针错误 12
    // EHR_CODE0_LOOP_TO_MAX = 1 << 10,            // 10 EHR 查找循环达到最大值 13
    // EHR_CODE0_DATA_EXIST = 1 << 11,             // 11 EHR EHP的数据重复 14
    // EHR_CODE0_INVALID_INDEX = 1 << 12,          // 12 EHR 索引错误 16
    // EHR_CODE0_LOCARION_ERROR = 1 << 13,         // 13 EHR 定位错误 10
    // EHR_CODE0_LOCATION_EXPIRED = 1 << 14,       // 14 EHR 定位过期 19
    // EHR_CODE0_NO_CENTER_LINE = 1 << 15,         // 15 EHR 车道中心线缺失 无
    // EHR_CODE0_LON_LAT_ERROR = 1 << 16,          // 16 EHR 地图经纬度不在有效范围内 无
    // EHR_CODE0_PROC_TIME_OUT = 1 << 17,          // 17 EHR 地图处理超时 无
    // EHR_CODE0_COORD_TRANS_FAIL = 1 << 18,       // 18 EHR 坐标系转换失败 - 在自车坐标系下的位置检查 无
    // EHR_CODE0_LOCATION_GNSS_ERROR = 1 << 19,    // 19 EHP吐出GNSS失效 22
    // EHR_CODE0_LOCATION_CAMERA_ERROR = 1 << 20,  // 20 EHP吐出Camera失效 23
    // EHR_CODE0_LOCATION_VEHICLE_ERROR = 1 << 21, // 21 EHP吐出车身相关失效 24
    // EHR_CODE0_LOCATION_IMU_ERROR = 1 << 22,     // 22 EHP吐出IMU故障类失效 25
    // EHR_CODE0_LOCATION_HDMAP_ERROR = 1 << 23,   // 23 EHP吐出HDMAP相关失效 26
    // EHR_CODE0_LOCATION_NOT_MATCH = 1 << 24,     // 24 EHP吐出自车的PathId为0 
    // EHR_CODE0_EHR_NO_SECTION = 1 << 25,         // 25 EHR没有自车所在位置的地图
    // EHR_CODE0_EHR_FILE_READ_WRITE_ERROR = 1 << 26, // 26 EHR文件读写异常
    // EHR_CODE0_HDMAP_LICENSE_ERROR = 1 << 27,    // 27 EHP license未激活或过期
    // // 28~29 Reserve
    // EHR_CODE0_POSITION_LONGITUDINAL_JUMP = 1 << 30,  // 30 EHR收到EHP输出的定位纵向跳变 
    // EHR_CODE0_POSITION_JAMP = 1U << 31,         // 31 EHR收到EHP输出的定位跳动 100
};


using LaneElementGroup = std::vector<LaneElement>; //put same first lane_num together
using LaneElementGroupSets = std::vector<LaneElementGroup>;

typedef struct SLPoint2D {
    SLPoint2D(double s_in, double l_in) : s(s_in), l(l_in){};
    SLPoint2D() : s(0.0), l(0.0){};
    ~SLPoint2D() = default;
    double s;
    double l;
} SLPoint;

typedef struct RefLineSmoothConfig_s {
    double weight_fem_pos_deviation;
    double weight_path_length;
    double weight_ref_deviation;
    double curvature_constraint;
    double ego_bounds_val;
    double bounds_val;
    double ego_s_front;
    double ego_s_back;
    double no_boundary_bounds_val;
    double no_boundary_s_front;
    double no_boundary_s_back;
    double severe_bend_kappa_up_limt;
    double severe_bend_kappa_down_limt;
    double severe_bend_bounds_val;
    double severe_bend_s_front;
    double severe_bend_s_back;
    double s_shape_kappa_up_limt;
    double s_shape_kappa_down_limt;
    double s_shape_bounds_val;
    double s_shape_s_front;
    double s_shape_s_back;
    double z_shape_heading_up_limt;
    double z_shape_heading_down_limt;
    double z_shape_bounds_val;
    double z_shape_s_front;
    double z_shape_s_back;
    double p_merge_front_s;
    double p_merge_back_s;
    double p_split_front_s;
    double p_split_back_s; 
    double p_split_merge_bounds_val_factor;  
    double p_continuous_split_length_s; 
    double p_continuous_split_bounds_val_factor;  
    double p_curve_take_line_thred;
    double p_short_continuous_split_length_s;
    double p_short_continuous_split_bounds_val_factor;
    double p_short_continuous_split_front_s;
    double p_merge_gap_to_lane_marker;
    double p_straight_split_bounds_val_factor;
}RefLineSmoothConfig;
