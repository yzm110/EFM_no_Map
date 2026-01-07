/******************************************************************************
/                              Copyright
/------------------------------------------------------------------------------
/    Date: 2025-4-17
/    Author: zj
/------------------------------------------------------------------------------
*******************************************************************************/

#pragma once
#include <stdint.h>
#include <vector>
#include "CommonDataType.h"

namespace NoMapEFM{

/**
 * @brief lane tyep（车道种别）
 *
 * @details
 */
enum LANE_TYPE : uint8_t {
    LANE_TYPE_NONE                    = 0,  // 无
    LANE_TYPE_REGULAR                 = 1,  // 普通车道
    LANE_TYPE_COMPOUND                = 2,  // 加减速复合车道
    LANE_TYPE_LEFT_ACCELERATION       = 3,  // 左侧加速车道
    LANE_TYPE_LEFT_DECELERATION       = 4,  // 左侧减速车道
    LANE_TYPE_HOV                     = 5,  // HOV
    LANE_TYPE_SLOW                    = 6,  // 慢车道
    LANE_TYPE_SHOULDER                = 7,  // 路肩
    LANE_TYPE_DRIVABLE_SHOULDER       = 8,  // 可行驶路肩
    LANE_TYPE_CONTROL                 = 9,  // 管制车道
    LANE_TYPE_EMERGENCY_PARKING_STRIP = 10, // 紧急停车带
    LANE_TYPE_BUS                     = 11, // 公交车道
    LANE_TYPE_BICYCLE                 = 12, // 非机动车道
    LANE_TYPE_TIDE                    = 13, // 潮汐车道
    LANE_TYPE_DIRECTION_CHANGE        = 14, // 可变车道
    LANE_TYPE_PARKING_ROAD            = 15, // 停车车道
    LANE_TYPE_DRIVABLE_PARKING_ROAD   = 16, // 可行驶停车道
    LANE_TYPE_TRUCK                   = 17, // 货车专用道
    LANE_TYPE_TIME_LIMIT_BUS          = 18, // 限时公交车道
    LANE_TYPE_PASSING_BAY             = 19, // 错车道
    LANE_TYPE_REVERSAL_LEFT_TURN      = 20, // 借道左转车道
    LANE_TYPE_TAXI                    = 21, // 出租车车道
    LANE_TYPE_TURN_LEFT_WAITING       = 22, // 转弯待转区车道
    LANE_TYPE_DIRECTION_LOCKED        = 23, // 定向车道
                                    //
                                    //
                                    //
    LANE_TYPE_STRAIGHT_WAITING        = 27, //
    LANE_TYPE_VEHICLE_BICYCLE_MIX     = 28, // 机非混合车道
    LANE_TYPE_MOTOR                   = 29, // 摩托车道
                                    //
    LANE_TYPE_TOLL                    = 31, // 收费站车道
    LANE_TYPE_CHECK_POINT             = 32, // 检查站车道
    LANE_TYPE_DANGEROUS_ARTICLE       = 33, // 危险品专用车道
    LANE_TYPE_FORBIDDEN_DRIVE         = 34, // 非行驶区域
    LANE_TYPE_THOUGH_LANE_ZONE        = 35, // 借道区
    LANE_TYPE_STREET_RAILWAY          = 36, // 有轨电车车道
    LANE_TYPE_BUS_BAY                 = 37, // 公交港湾车道
    LANE_TYPE_SPECIAL_CAR             = 38, // 特殊车辆专用车道
    LANE_TYPE_PEDESTRIANS             = 39, // 人行道
    LANE_TYPE_EMERGENCY               = 40, // 应急车道
    LANE_TYPE_HEDGING                 = 41, // 避险车道
    LANE_TYPE_BUS_LANE_WITH_TIME      = 42, // 时间段公交车道
    LANE_TYPE_EMPTY                   = 43, // 空车道
    LANE_TYPE_RIGHT_ACCELERATION      = 44, // 右侧加速车道
    LANE_TYPE_RIGHT_DECELERATION      = 45, // 右侧减速车道
    LANE_TYPE_SLOPE                   = 46, // 爬跑车道
    LANE_TYPE_REVERSE_NON_VEHICLE     = 47, // 逆向非机动车道
    LANE_TYPE_EDGE                    = 48, // 边缘车道
    LANE_TYPE_OTHER                   = 99, // 其他车道
};

/**
 * @brief 车道边类型定义
 *
 * @details
 */
enum LANE_BORDERTYPE : uint8_t {
  LANE_BORDERTYPE_Unknown = 0,                                  // 未调查
  LANE_BORDERTYPE_DashLine,                                     // 普通虚线
  LANE_BORDERTYPE_ShortThickDashLine,                           // 短粗虚线
  LANE_BORDERTYPE_TurnWaitingLine,                              // 待转区标线
  LANE_BORDERTYPE_NormalSolidLine,                              // 普通实线
  LANE_BORDERTYPE_DiversionZoneLine,                            // 导流区线
  LANE_BORDERTYPE_ParkingZoneLine,                              // 停车位标线
  LANE_BORDERTYPE_ThickSolidLine,                               // 粗实线
  LANE_BORDERTYPE_LeftDashRightSolidLine,                       // 左虚右实(通过4K数组进行还原，问题：是否0左1右)
  LANE_BORDERTYPE_LeftSolidRightDashLine,                       // 左实右虚(通过4K数组进行还原)
  LANE_BORDERTYPE_DoubleSolidLine,                              // 双实线(通过4K数组进行还原：双普通实线)
  LANE_BORDERTYPE_DoubleDashLine,                               // 双虚线(通过4K数组进行还原：双普通虚线)
  LANE_BORDERTYPE_OtherLine,                                    // 其它线
  LANE_BORDERTYPE_RoadEdgeLine,                                 // 道路边缘（来源4K border type的道路边缘）
  LANE_BORDERTYPE_PhysicalBarrier,                              // 物理隔离 (来源4K border type，如下：道路设施边界、路缘石、隔离桩、护栏、墙、防护网、施工围挡、移动护栏)
  LANE_BORDERTYPE_VirtualLine,                                  // 虚拟线
  LANE_BORDERTYPE_None                                          // 无
};
typedef std::vector<LANE_BORDERTYPE> LANE_BORDERTYPE_LIST;

/**
 * @brief 车道长实线类型定义
 *
 * @details
 */
enum LANE_LSLTYPE : uint8_t {
  LANE_LSLTYPE_NONE    = 0,  // 无长实线
  LANE_LSLTYPE_LEFT    = 1,  // 左侧长实线
  LANE_LSLTYPE_RIGHT   = 2,  // 右侧长实线
  LANE_LSLTYPE_BOTH    = 3,  // 两侧长实线
  LANE_LSLTYPE_UNKNOWN = 4   // 未调查
};

/**
 * 车道拓扑变化类型
 */
enum LaneChangeType : uint8_t {
  LeftTurnExpandingLane       = 1,                        // 左向扩展车道
  RightTurnExpandingLane      = 2,                        // 右向扩展车道
  LeftTurnMergingLane         = 3,                        // 左向汇入车道
  RightTurnMergingLane        = 4,                        // 右向汇入车道
  BothDirectionExpandingLane  = 5,                        // 双向扩展车道
  BothDirectionMergingLane    = 6,                        // 双向汇入车道
  OtherLane                   = 7,                        // 其他
  NotApplicable               = 99,                       // 不适用
};

/**
 * @brief 定义特征点变化类型
 *
 * @details
 */
enum FEATUREPOINT_TYPE : uint8_t {
  FEATUREPOINT_TYPE_LaneTypeChange = 0,                                    // 车道类型变化
  FEATUREPOINT_TYPE_LaneCountChange = 1,                                   // 车道数变化
  FEATUREPOINT_TYPE_DataBoundaryStart = 2,                                 // 数据边界开始
  FEATUREPOINT_TYPE_DataBoundaryEnd = 3,                                   // 数据边界结束
  FEATUREPOINT_TYPE_LaneContinuityPoint = 4,                               // 车道连续点
  FEATUREPOINT_TYPE_ExchangeAreaStart = 5,                                 // 交换区起点
  FEATUREPOINT_TYPE_ExchangeAreaEnd = 6,                                   // 交换区终点
  FEATUREPOINT_TYPE_RegularIntersectionEntrance = 7,                       // 普通路口进入点
  FEATUREPOINT_TYPE_RegularIntersectionExit = 8,                           // 普通路口退出点
  FEATUREPOINT_TYPE_UTurnIntersectionEntrance = 9,                         // 掉头路口进入点
  FEATUREPOINT_TYPE_UTurnIntersectionExit = 10,                            // 掉头路口退出点
  FEATUREPOINT_TYPE_WideLaneEntrance = 11,                                 // 宽车道进入点
  FEATUREPOINT_TYPE_WideLaneExit = 12,                                     // 宽车道退出点
  FEATUREPOINT_TYPE_TollBoothEntrance = 13,                                // 收费站进入点
  FEATUREPOINT_TYPE_TollBoothExit = 14,                                    // 收费站退出点
  FEATUREPOINT_TYPE_ExchangeAreaInteriorPoint = 15,                        // 交换区内点
  FEATUREPOINT_TYPE_WideLaneInteriorPoint = 16,                            // 宽车道内点
  FEATUREPOINT_TYPE_TollBoothInteriorPoint = 17,                           // 收费站内点
  FEATUREPOINT_TYPE_MarkingLineChangePoint = 18,                           // 标线变化点
  FEATUREPOINT_TYPE_TIntersectionEntrancePoint = 19                        // T型路口进入点
};

/**
 * @brief Functional Road Class（道路种别）
 *
 * @details
 */
enum LINK_FRC {
    LINK_FRC_UNKNOWN,         // 未调查
    LINK_FRC_MOTORWAY,        // 都市间高速
    LINK_FRC_URBAN_MOTORWAY,  // 都市内封闭路
    LINK_FRC_LOCAL_ROAD,      // 国道
    LINK_FRC_NORMAL_ROAD,     // 一般道路
    LINK_FRC_NO_DRIVE_ROAD,   // 非机动车道，内部路也收录在此范围
};

/**
 * @brief Form Of Way（Link种别）
 *
 * @details
 */
enum LINK_FOW {
    LINK_FOW_UNKNOWN,        // 未知
    LINK_FOW_NO_SPECIAL,     // 一般道
    LINK_FOW_ENTRANCE_RAMP,  // 匝道入口
    LINK_FOW_EXIT_RAMP,      // 匝道出口
    LINK_FOW_RAMP,           // 匝道
    LINK_FOW_JCT,            // JCT
    LINK_FOW_REST_AREA,      // 休息区
    LINK_FOW_TOLL_AREA,      // 收费站
    LINK_FOW_ROUNDABOUT,     // 环岛
};

/**
 * @brief struct（Link构造属性）
 *
 * @details
 */
enum LINK_STRUCT {
    LINK_STRUCT_UNKNOWN,        // 未知
    LINK_STRUCT_NO_SPECIAL,     // 一般道
    LINK_STRUCT_TUNNEL,         // 隧道
    LINK_STRUCT_BRIDGE,         // 桥
    LINK_STRUCT_REST_AREA,      // 休息区
    LINK_STRUCT_TOLL_AREA,      // 收费站
    LINK_STRUCT_ROUNDABOUT,     // 环岛
    LINK_STRUCT_CROSS_LINK,     // 交叉点内LINK
};

/**
 * @brief link方向
 */
enum LINK_DIRECTION {
    LINK_DIRECTION_UNKNOWN = 0,  //未知，可认为不可通行
    LINK_DIRECTION_BOTH = 1,     //双向
    LINK_DIRECTION_START_TO_END, //顺向
    LINK_DIRECTION_END_TO_START  //逆向
};

struct SLaneChange {
public:
    SLaneChange() : num_(0)
                 , offset_(0)
                 , split_dir_(0)
                 , meger_dir_(0)
                 , is_new_lane_(false)
                 , is_end_lane_(false){}
    ~SLaneChange() {}

    uint32_t num_;            //车道id，从右向左，从1开始
    uint32_t offset_;         //单位cm，相对于link的起点
    uint8_t  split_dir_;      //0:无分歧;1:向左split一根车道;2:向右split一根车道;3:中间车道向两边分岔
    uint8_t  meger_dir_;      //0:无合流;1:左边车道merge进来;2:右边车道merge进来;3:中间车道向中间汇流
    uint8_t  is_new_lane_;    //true：对比上一个link，是否为新增车道
    uint8_t  is_end_lane_;    //true：对比下一个link，是否为结束车道
};

struct SOffsetValue {
    SOffsetValue() : offset_(0), value_(0.0){};
    ~SOffsetValue() = default;

    uint32_t offset_;   //单位cm，对应value_的起点
    float    value_;    //根据不同数据进行定义
};

enum PATH_SPECIAL_TYPE {
    // TODO(lxf)  这里面是道路级别的数据
    PATH_SPECIAL_TYPE_UNKNOWN = 0,     //   未知，
    // TODO(lxf)  后续自己判断
    PATH_SPECIAL_TYPE_TO_RAMP = 1,     //   上匝道
    PATH_SPECIAL_TYPE_TO_MAIN,         //   上主路
    // 可以透传
    PATH_SPECIAL_TYPE_TOLL_BOOTH,      //   收费站
    // TODO(lxf)  需要拼接下
    PATH_SPECIAL_TYPE_TUNNEL,          //   隧道
    // TODO(lxf) 后续自己判断
    PATH_SPECIAL_TYPE_RAMP_ROADMERGE,  //   匝道道路merge
    PATH_SPECIAL_TYPE_RAMP_ROADSPLIT,  //   匝道道路split
    PATH_SPECIAL_TYPE_MAIN_ROADMERGE,  //   主路道路merge
    PATH_SPECIAL_TYPE_MAIN_ROADSPLIT,  //   主路道路split
};

struct SEhpOutputLoc{
public:
    SEhpOutputLoc() : id_(0)
                    , lat_(0.0)
                    , lon_(0.0)
                    , heading_(0.0)
                    , speed_(0.0)
                    , path_offset_id_(0)
                    , offset_(0)
                    , link_id_(0)
                    , path_id_(0)
                    , time_stmp_(0)
                    , lane_id_(0)
                    , lane_ids_{}
                    , lane_offset_(0){}
    ~SEhpOutputLoc() {}

    uint32_t id_;                     // 从第一帧开始，每帧加一，标识定位传输顺序
    double  lat_;                     // 维度
    double  lon_;                     // 经度
    double  heading_;                 // 航向角、单位弧度
    double  speed_;                   // 自车速度，单位m/s
    uint64_t path_offset_id_;         //自车所在的link，link唯一ID
    uint32_t offset_;                 //相对于path的起点的偏移，单位cm
    uint64_t link_id_;                //自车所在的link，原始ID
    uint32_t path_id_;                //自车所在的path，该path必须是主path
    uint64_t time_stmp_;              //时间戳
    uint64_t lane_id_;                //自车所在的lane，lane唯一ID
    std::vector<uint64_t> lane_ids_;  //自车所在的lanes，因为未进行车道定位，所以为一个数组，从右向左
    uint32_t lane_offset_;            //相对于lane的起点的偏移，单位cm
};

struct SSpecialData{
public:
    SSpecialData() : type_(PATH_SPECIAL_TYPE())
                   , dir_(0)
                   , s_offset_(0)
                   , e_offset_(0)
                   , child_path_(0){}
    ~SSpecialData() {}

    PATH_SPECIAL_TYPE type_;       //类型
    uint8_t           dir_;        //方向，0：中间；1：左侧；2：右侧；
    uint32_t          s_offset_;   //相对于path起点的offset，该对象的起点，单位：cm
    uint32_t          e_offset_;   //相对于path起点的offset，该对象的终点，单位：cm
    uint32_t          child_path_; //对象的对一个的子pathID
};

struct SLinkOffset{
public:
    SLinkOffset()  : path_offset_id_(0)
                   , s_offset_(0)
                   , e_offset_(0){}
    ~SLinkOffset() {}

    uint64_t path_offset_id_; //link全局唯一ID
    uint32_t s_offset_;       //link的起点 相对于path起点的offset，单位：cm
    uint32_t e_offset_;       //link的终点 相对于path起点的offset，单位：cm
};

struct SEhpOutputPath{
public:
    SEhpOutputPath() : path_id_(0)
                     , parent_path_id_(0)
                     , InParentOffset_(0)
                     , special_datas_{}
                     , link_offsets_{}
                     , is_end_(false){}
    ~SEhpOutputPath() {}

    uint32_t                  path_id_;         //path id,全局唯一
    uint32_t                  parent_path_id_;  //父path_id， 主path该值为0，其他path不为0
    uint32_t                  InParentOffset_;   // 在父path中的偏移，单位cm
    std::vector<SSpecialData> special_datas_;   //路径上特殊数据，从起点开始，由近及远
    std::vector<SLinkOffset>  link_offsets_;    //路径上的link对应的数据， 从起点开始，由近及远
    bool                      is_end_;          //当前path的最后一个link，是否是导航路径的终点
};

struct SEhpOutputPathList {
    SEhpOutputPathList() = default;
    std::vector<SEhpOutputPath> ehp_output_path_list{};  // 当前所有的path信息
};

struct LinkLaneInfo{
    LinkLaneInfo() = default;
    uint64_t lane_id{0};
    uint8_t lane_num{0};    // 车道编号,从右到左，从1开始，非机动车道和应急车道这些不可行驶车道也需要添加
    std::vector<uint64_t> next_lane_ids{};   // 后继车道id列表
    std::vector<uint64_t> pre_lane_ids{};   // 后继车道id列表
    std::vector<LANE_TYPE> lane_types{};
    LANE_BORDERTYPE_LIST left_line_border_types{};  // 左侧车道边界类型列表
    LANE_BORDERTYPE_LIST right_line_border_types{};   // 右侧车道边界类型列表
    LANE_LSLTYPE lane_lsl_type{LANE_LSLTYPE_NONE};  // 车道长实线类型
    bool is_lane_num_change{false};   // 该车道的起点位置是否存在分歧、合流的变化
    FEATUREPOINT_TYPE feature_point_type{FEATUREPOINT_TYPE_LaneContinuityPoint};  // 特征点类型
    uint32_t start_point_to_link_start_dis{0};  // 起点到link起点的距离，单位cm
    bool is_route_lane{false};  // 是否是导航路径上的车道
    LaneChangeType lane_change_type{LaneChangeType::NotApplicable};  // 车道拓扑变化类型
    uint32_t length{0};  // 车道长度，单位cm, 需要在EFM实现

    // TODO(lxf)  由于在车道变化的地方sd的link可能不会打断，导致一个link中有两个特征点，并且两个特征点的车道数量不一样，
    // 所以一个link中可能存在多个特征点信息对应多条lane。
    // std::vector<LaneFeaturePointInfo> feature_point_info_list{};  // 车道上所有的特征点信息
};


struct SEhpOutputLink{
public:
    SEhpOutputLink() : id_(0)
            , path_id_(0)
            , path_offset_id_(0)
            , s_offset_(0)
            , e_offset_(0)
            , frc_(LINK_FRC())
            , fow_(LINK_FOW())
            , struct_(LINK_STRUCT())
            , dir_(LINK_DIRECTION())
            , speeds_{}
            , speed_unit_(1)
            , s_lane_num_(0)
            , e_lane_num_(0)
            , lane_changes_{}
            , link_lane_info_list_{}
            , curvs_{}
            , slopes_{}
            , all_in_links_{}
            , all_out_links_{}
            , last_in_link_(0)
            , next_out_link_(0)
            , geoms_{}
            , is_highway_city(0) {}
    ~SEhpOutputLink() {}

    uint64_t id_;                           // 全局ID，可以采用原数据的linkid
    uint32_t path_id_;                      // 所在的path，同一个link只能存在一个path，
    uint64_t path_offset_id_;               // path_id_ << 32 + s_offset_
    uint32_t s_offset_;                     // 所在path的偏移，单位cm；如果回环路径，则可能在一个path中出现多次，保存多个对象，每个对象的s_offset不一样
    uint32_t e_offset_;                     // 所在path的偏移，单位cm；如果回环路径，则可能在一个path中出现多次，保存多个对象，每个对象的e_offset不一样
    LINK_FRC frc_;                          // FunctionalRoadClass
    LINK_FOW fow_;                          // FormOfWay（JC、IC等）
    LINK_STRUCT struct_;                    // LINK_STRUCT
    LINK_DIRECTION dir_;                    // 该link的通行方向
    std::vector<SOffsetValue> speeds_;      // 该link在path方向上的通行速度，etc：60、70,可能一段路存在多段限速
    uint8_t speed_unit_;                    // speed_对应的单位：1：km/h
    uint8_t s_lane_num_;                    // 根据path通行方向，link起点的车道数
    uint8_t e_lane_num_;                    // 根据path通行方向，link终点的车道数
    std::vector<SLaneChange> lane_changes_; // 车道变化信息
    std::vector<LinkLaneInfo> link_lane_info_list_;  // link上所有的车道信息
    std::vector<SOffsetValue> curvs_;       // 根据path方向，从起点到终点的curv
    std::vector<SOffsetValue> slopes_;      // 根据path方向，从起点到终点的slop
    std::vector<uint64_t> all_in_links_;    // 进入links， path_offset_id_
    std::vector<uint64_t> all_out_links_;   // 脱出links， path_offset_id_
    uint64_t last_in_link_;                 // 根据path路径，进入link
    uint64_t next_out_link_;               // 根据path路径，脱出link
    std::vector<EFMPoint> geoms_;            // 根据path方向，从起点到终点的坐标

    uint8_t is_highway_city;  // 0:未知; 1:高速/高架道路; 2：城市道路
};

struct SEhpOutputLinkList {
    SEhpOutputLinkList() = default;
    std::vector<SEhpOutputLink> ehp_output_link_list{};  // 只需要发送自车后方100米，前方1公里的数据，每帧都需要发送
};



}