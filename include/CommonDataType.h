/*
 * @Author: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @Date: 2023-12-13 20:11:41
 * @LastEditors: xiaofeng.liu xiaofeng.liu@jicaai.com
 * @LastEditTime: 2023-12-15 11:35:59
 * @FilePath: /repo/code/dx11_noa/application/environmentmodelfunction/include/CommonMathMethod.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <array>
#include <vector>
#include <set>
#include <unordered_map>
#include <map>
#include "CommonMathMethod.h"
#include "CompileConfig.h"

template <class T>
    struct Point2D {
    T x, y;

    static_assert(std::is_same<T, int>::value || std::is_same<T, float>::value
                        || std::is_same<T, double>::value,
                    "Type not iis nt or float or double!");
    Point2D() : Point2D(0, 0) {}
    Point2D(const T& x, const T& y) : x(x), y(y) {}
    Point2D(const Point2D<T>& point) : Point2D(point.x, point.y) {}
    ~Point2D() = default;
    void Reset(const Point2D<T>& point) {
        x = point.x;
        y = point.y;
    }
    Point2D<T>& operator=(const Point2D<T>& point) {
        Reset(point);
        return *this;
    }
    Point2D<T>& operator+=(const Point2D<T>& point) {
        this->x += point.x;
        this->y += point.y;
        return *this;
    }
    Point2D<T>& operator-=(const Point2D<T>& point) {
        this->x -= point.x;
        this->y -= point.y;
        return *this;
    }
    const Point2D<T> operator-() const {
        return Point2D<T>(-this->x, -this->y);
    }
    const Point2D<T> operator+(const Point2D<T>& point) const {
        return Point2D<T>(this->x + point.x, this->y + point.y);
    }
    const Point2D<T> operator-(const Point2D<T>& point) const {
        return Point2D<T>(this->x - point.x, this->y - point.y);
    }
    const Point2D<T> operator/(const Point2D<T>& point) const {
        return Point2D<T>(this->x / point.x, this->y / point.y);
    }
    const Point2D<T> operator/(const T& value) const {
        return Point2D<T>(this->x / value, this->y / value);
    }
    const Point2D<T> operator*(const Point2D<T>& point) const {
        return Point2D<T>(this->x * point.x, this->y * point.y);
    }
    const Point2D<T> operator*(const T& value) const {
        return Point2D<T>(this->x * value, this->y * value);
    }
    friend const Point2D<T> operator*(const T& value, const Point2D<T>& point) {
        return Point2D<T>(value * point.x, value * point.y);
    }
    //叉乘
    T CrossProduct(const Point2D<T>& point) const {
        return (this->x * point.y - this->y * point.x);
    }
    //点积
    T InnerProduct(const Point2D<T>& point) const{
        return (this->x * point.x + this->y * point.y);
    }      
    // bool IsIn(const Rect<T> &rect) const {
    //     return x >= rect.l && x <= rect.r && y >= rect.t && y <= rect.b;
    // }
};
typedef Point2D<int> Point2Di;
typedef Point2D<float> Point2Df;
typedef Point2D<double> Point2Dd;
using EFMPoint = Point2D<double>;
using EFMRefLinePoints = std::vector<EFMPoint>;
using EFMRefLinePointsSection = std::array<std::vector<EFMPoint>, 2>;

template <class T>
struct Point2DEx : public Point2D<T> {
 public:
  Point2DEx() {
    this->x = 0;
    this->y = 0;
    this->source_ = 0;
  }
  Point2DEx(T x, T y) {
    this->x = x; this->y = y;
    this->source_ = 0;
  }

  Point2DEx<T>& operator = (const Point2DEx<T> &pt) {
    this->x = pt.x;
    this->y = pt.y;
    this->source_ = pt.source_;
    return *this;
  }
  // bool IsIn(const Rect<T> &rect) const {
  //   return this->x >= rect.l && this->x <= rect.r
  //     && this->y >= rect.t && this->y <= rect.b;
  // }
  int source_;
};

template <typename T>
struct PointSL {
  static_assert(std::is_same<T, int>::value || std::is_same<T, float>::value|| std::is_same<T, double>::value,
      "Type is not int or float!");
  PointSL() : PointSL(0, 0) {}
  PointSL(const T& s, const T& l) : s(s), l(l) {}
  PointSL(const PointSL<T>& point) : PointSL(point.s, point.l) {}
  ~PointSL() = default;
  void Reset(const PointSL<T>& point) {
    s = point.s;
    l = point.l;
  }
  PointSL<T>& operator=(const PointSL<T>& point) {
    Reset(point);
    return *this;
  }
  T s, l;
};  
typedef PointSL<int> PointSLi;
typedef PointSL<float> PointSLf;
typedef PointSL<double> PointSLd;

template <typename T>
struct Point3D {
  static_assert(std::is_same<T, int>::value || std::is_same<T, float>::value ||
      std::is_same<T, double>::value, "Type is not int or float or double!");
  Point3D() : Point3D(0, 0, 0) {}
  Point3D(const T& x, const T& y, const T& z) : x(x), y(y), z(z) {}
  Point3D(const Point3D<T>& point) {Reset(point);}
  ~Point3D() = default;
  float norm2d() const {return sqrt(x*x+y*y);}
  void Reset(const Point3D<T>& point) {
    x = point.x;
    y = point.y;
    z = point.z;
  }
  Point3D<T>& operator=(const Point3D<T>& point) {
    Reset(point);
    return *this;
  }
  T x, y, z;
}; 

template <typename T>
struct EulerAngle {
  static_assert(std::is_same<T, int>::value || std::is_same<T, float>::value ||
      std::is_same<T, double>::value, "Type is not int or float or double!");
  EulerAngle() : EulerAngle(0, 0, 0) {}
  EulerAngle(const T& yaw, const T& pitch, const T& roll) : yaw(yaw),
      pitch(pitch), roll(roll) {}
  EulerAngle(const T& w, const T& x, const T& y, const T& z) : EulerAngle(
      atan2f(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z)),
      asinf(2.0 * (w*y - z*x)),
      atan2f(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y))) {}
  EulerAngle(const EulerAngle<T>& euler_angle) : EulerAngle(euler_angle.yaw,
                                                            euler_angle.pitch,
                                                            euler_angle.roll) {}
  ~EulerAngle() = default;
  void Reset(const EulerAngle<T>& euler_angle) {
    yaw = euler_angle.yaw;
    pitch = euler_angle.pitch;
    roll = euler_angle.roll;
  }
  EulerAngle<T>& operator=(const EulerAngle<T>& euler_angle) {
    Reset(euler_angle);
    return *this;
  }
  T yaw, pitch, roll;
};  
typedef EulerAngle<int> EulerAnglei;
typedef EulerAngle<float> EulerAnglef;
typedef EulerAngle<double> DoubleEulerAngle;

template <typename T>
struct PosePoint {
  static_assert(std::is_same<T, int>::value || std::is_same<T, float>::value ||
      std::is_same<T, double>::value, "Type is not int or float or double!");
  PosePoint() : PosePoint(0, 0, 0, 0, 0, 0) {}
  PosePoint(const T& x, const T& y, const T& z, const T& yaw, const T& pitch,
            const T& roll) : x(x), y(y), z(z),
                yaw(yaw), pitch(pitch), roll(roll) {}
  PosePoint(const Point3D<T>& point, const EulerAngle<T>& euler_angle)
  : PosePoint(point.x, point.y, point.z, euler_angle.yaw, euler_angle.pitch,
              euler_angle.roll) {}
  ~PosePoint() = default;
  float norm2d() const {return sqrt(x*x+y*y);}
  PosePoint<T>& operator=(const PosePoint<T>& point) {
    x = point.x;
    y = point.y;
    z = point.z;
    yaw = point.yaw;
    pitch = point.pitch;
    roll = point.roll;
    return *this;
  }
  PosePoint<T>& operator=(const Point3D<T>& point) {
    x = point.x;
    y = point.y;
    z = point.z;
    return *this;
  }
  PosePoint<T>& operator=(const EulerAngle<T>& point) {
    yaw = point.yaw;
    pitch = point.pitch;
    roll = point.roll;
    return *this;
  }
  PosePoint<T>& operator+=(const PosePoint<T>& point) {
    x += point.x;
    y += point.y;
    z += point.z;
    yaw = UnifyTheta(yaw+point.yaw);
    pitch = UnifyTheta(pitch+point.pitch);
    roll = UnifyTheta(roll+point.roll);
    return *this;
  }
  PosePoint<T>& operator-=(const PosePoint<T>& point) {
    x -= point.x;
    y -= point.y;
    z -= point.z;
    yaw = UnifyTheta(yaw-point.yaw);
    pitch = UnifyTheta(pitch-point.pitch);
    roll = UnifyTheta(roll-point.roll);
    return *this;
  }
  T x, y, z, yaw, pitch, roll;
};  
typedef PosePoint<int> PosePointi;
typedef PosePoint<float> PosePointf;
typedef PosePoint<double> DoublePosePoint;


enum EfmMergeType : uint8_t {
    EFM_MergeType_NONE = 0,
    EFM_MergeType_TO_LEFT = 1,
    EFM_MergeType_FROM_LEFT = 2,
    EFM_MergeType_LEFT_TO_MIDDLE = 3,
    EFM_MergeType_TO_RIGHT = 4,
    EFM_MergeType_FROM_RIGHT = 5,
    EFM_MergeType_RIGHT_TO_MIDDLE = 6,
    EFM_MergeType_TO_MIDDLE = 7
};

enum EfmSplitType : uint8_t {
    EFM_SplitType_NONE = 0,
    EFM_SplitType_TO_LEFT = 1,
    EFM_SplitType_FROM_LEFT = 2,
    EFM_SplitType_CONTIUE_FROM_LEFT = 3,
    EFM_SplitType_TO_RIGHT = 4,
    EFM_SplitType_FROM_RIGHT = 5,
    EFM_SplitType_SPLIT_FROM_LEFT = 6,
    EFM_SplitType_CONTIUE_FROM_RIGHT = 7,
    EFM_SplitType_SPLIT_FROM_RIGHT = 8
} ;

enum LaneType : uint8_t {
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
} ;

enum LineLocType_e : uint8_t
{
    EGO_LEFT = 1,
    EGO_RIGHT = 2,
    ADJACENT_LEFT = 3,
    ADJACENT_RIGHT = 4,
    NEXT_2_LEFT   = 5,
    NEXT_2_RIGHT   = 6,
    LEFT_LANE_RIGHT = 98,
    RIGHT_LANE_LEFT = 99
};

//Type of the lane (车道线类型)
enum BstdsLineType :uint8_t{
  LINE_UNKNOWN = 0,
  LINE_SOLID = 1,
  LINE_DASHED = 2,
  LINE_DOUBLESOLID = 3,
  LINE_DOUBLEDASHED = 4,
  LINE_SOLIDDASHED = 5, /**< zuo实you虚 */
  LINE_DASHEDSOLID = 6, /**< zuo虚you实 */
  LINE_FISHBONE = 7,
  LINE_TEMPORARY = 8,
  LINE_CURB = 9,
  LINE_CONE = 10
};

//node dir
enum NodeDir :uint8_t{
  NODE_DIR_UNKNOWN = 0,
  NODE_DIR_LEFT = 1,
  NODE_DIR_RIGHT = 2
};

//cyj↓
//c1236
typedef struct GPSLinePoint_s {
    double Longitude;
    double Latitude;
}GPSLinePoint;

using GPSLinePoints = std::vector<GPSLinePoint>;
typedef struct PointsTangent_s {
    double tangent;
}PointTangent;

using PointsTangent = std::vector<PointTangent>;

typedef struct SCALAR_STATE_STRUCT_TAG   /*  scalar state struct */
{
    /*  state vector */
    float            mean_f32;
    /*  state variance */
    float            variance_f32;
} SCALAR_STATE_STRUCT;
typedef struct KALMAN_MODEL_ONEDIM_STRUCT_TAG
{
    /*current Kalman state */
    SCALAR_STATE_STRUCT state_s;
    bool             has_been_initialised_b;
} KALMAN_MODEL_ONEDIM_STRUCT;

typedef struct LaneFusionPoint_TAG {
    bool valid; // 当前这个点是否有效，即是否有数据
    float x;    // 横坐标
    float y;    // 纵坐标
    float dydx; // 导数
    KALMAN_MODEL_ONEDIM_STRUCT kalman_model_onedim_struct;
} LaneFusionPoint;
using LaneFusionPointsType = std::vector<LaneFusionPoint>;

typedef struct {
    float bev_line_age; // BEV线的存活时间;
    float zombie_line_age;//纯僵尸线的存活时间;
    int zombie_line_origin_id; // 僵尸线的原始id;
    float zombie_line_total_length; // 僵尸线的总长度;
    GPSLinePoints total_line_gps_points;//用GPS表示的点的经纬度（为了避免发现是僵尸轨迹的时候，会晚一个周期）
    LaneFusionPointsType LaneFusionPoints; 
} LaneFusionProcessType;

typedef struct {
    int line_id; // 僵尸线的原始id;
    float line_fused_age;//僵尸线参与融合的时间，用来退出融合
} EFMFusedLineStsType;
using EFMFusedLineSts = std::vector<EFMFusedLineStsType>;

typedef struct {
    int index; // 在total_line中的index
    float x; // 僵尸线的原始id;
    float y;//僵尸线参与融合的时间，用来退出融合
} OverlapPointType;

typedef struct {
    OverlapPointType start_overlap_point; // 僵尸线的原始id;
    OverlapPointType end_overlap_point;//僵尸线参与融合的时间，用来退出融合
    bool is_valid;//当前这个起始点是否可用。当前只有实时更新的线才可用
} EFMOverlapSts;
//cyj ↑

typedef struct BevLineInnerS {
    BevLineInnerS()
    : minus_line{}
    , C0Minus(0)
    , C1Minus(0)
    , C2Minus(0)
    , C3Minus(0)
    , MinusStartPoint(0)
    , MinusEndPoint(0)
    , minus_valid(false)
    , first_line{}
    , C0First(0)
    , C1First(0)
    , C2First(0)
    , C3First(0)
    , FirstStartPoint(0)
    , FirstEndPoint(0)
    , first_valid(false)
    , sec_line{}
    , C0Sec(0)
    , C1Sec(0)
    , C2Sec(0)
    , C3Sec(0)
    , SecStartPoint(0)
    , SecEndPoint(0)
    , sec_valid(false)
    , id(0)
    , lane_location_type(0)
    , line_type(0)
    , typ_chg_point(0)
    , typ_aft_chg_point(0)
    , md_qly(0)
    , line_is_available(false)
    , line_location(0)
    , total_line{}
    , is_connected_f(false)
    , is_inner_line_f(false)
    , inner_point_index_pair(-1,-1)
    , between_line_id_pair(0,0)
    , sl_to_ego()
    , ego_is_inside(false)
    , line_length(0)
    , ego_side(0)
    , attached_curb_side(0)
    //c1236
    , line_fusion_type(0)
    , fused_line_sts{}
    , line_overlap_points{}
    , lane_fusion_process_info{}{}; 
    EFMRefLinePoints minus_line;
    float C0Minus;
    float C1Minus;
    float C2Minus;
    float C3Minus;
    float MinusStartPoint;
    float MinusEndPoint;
    bool minus_valid;
    EFMRefLinePoints first_line;
    float C0First;
    float C1First;
    float C2First;
    float C3First;
    float FirstStartPoint;
    float FirstEndPoint;
    bool first_valid;
    EFMRefLinePoints sec_line;
    float C0Sec;
    float C1Sec;
    float C2Sec;
    float C3Sec;
    float SecStartPoint;
    float SecEndPoint;
    bool sec_valid;
    int id;
    int lane_location_type;//原始的BEV的分配结果
    int line_type; //车道线类型 BstdsLineType
    float typ_chg_point; //车道线类型变化点距离自车的纵向距离
    int typ_aft_chg_point; //车道线类型发生变化后，车道线的类型 BstdsLineType
    float md_qly;
    //**********分界线，以上为bev原始的属性*****以下为后处理后的属性***************//
    bool line_is_available;//点是否大于1
    uint8_t line_location;//最右边排1，依次增大， 未排的为0,内部排的
    EFMRefLinePoints total_line;//三段连一起后的线
    bool is_connected_f; //线是否被连接
    bool is_inner_line_f; //是否是介在中间的线
    std::pair<int, int> inner_point_index_pair;//total_line里，从[first]到[second]的点介于线between_line_id_pair之间
    std::pair<int, int> between_line_id_pair;//介于那两条线之间，两条线的右，左 location
    PointSLd sl_to_ego;//和自车的sl
    bool ego_is_inside;//自车是否落在线的区间内
    double line_length;
    int ego_side; //位于自车的左或者右，1左，2右
    int attached_curb_side; //有没有横向距离很近的路沿，sec-路沿在线的哪一侧0-无，1-左，2-右， 3-both
    //c1236
    int line_fusion_type;//当前车道线的类型：0:纯实时感知滤波；1:僵尸线+感知融合滤波；2:纯僵尸线; 3:路口拓扑线
    EFMFusedLineSts fused_line_sts;//被融合上的僵尸线原始id
    EFMOverlapSts line_overlap_points;//前后帧，有重叠的，起点和终止点的位置，只对0纯实时感知滤波有效
    LaneFusionProcessType lane_fusion_process_info;//车道线融合处理信息
    //cyj ↑
}BevLineInner;

typedef struct BevLineInnerConnectS {
    BevLineInnerConnectS()
    :line_point{}
    ,base_line_ptr(nullptr)
    ,back_connect_line_set_ptr{}
    ,front_connect_line_set_ptr{}
    // ,sl_to_ego(0.0, 0.0)
    // ,ego_is_inside(false)
    ,line_length(0)
    ,split_merge_type(0)
    ,split_side(0)
    ,split_point(0,0)
    ,merge_side(0)
    ,merge_point(0,0){};

    BevLineInnerConnectS(BevLineInnerS* base_line)
    :line_point{}
    ,base_line_ptr(base_line)
    ,back_connect_line_set_ptr{}
    ,front_connect_line_set_ptr{}
    // ,sl_to_ego(0.0, 0.0)
    // ,ego_is_inside(false)
    ,line_length(0)
    ,split_merge_type(0)
    ,split_side(0)
    ,split_point(0,0)
    ,split_point_index(-1)
    ,split_erea_is_drivable(false)
    ,merge_side(0)
    ,merge_point(0,0)
    ,merge_point_index(-1)
    ,merge_erea_is_drivable(false){
      line_point = base_line->total_line;
    };

    // BevLineInnerConnectS(uint8_t location_type, EFMRefLinePoints points, int id);
    // void reset();

    EFMRefLinePoints line_point;
    BevLineInnerS* base_line_ptr;
    std::map<BevLineInnerS*,int> back_connect_line_set_ptr;//base line的back()连接的线,key是那条bev线，val是连接的类型，是截断还是直连
    std::map<BevLineInnerS*,int> front_connect_line_set_ptr;//base line 的front()连接的线
    // PointSLd sl_to_ego;//和自车的sl
    // bool ego_is_inside;//自车是否落在线的区间内  
    double line_length;
    int split_merge_type;//1-split, 2-merge, 3-both, 0 -无
    int split_side;//1左线，2右线，0无
    EFMPoint split_point;//split点在自车坐标下的x,y
    int split_point_index;//在line_point中的下标
    bool split_erea_is_drivable;//split的区域是否可行驶
    int merge_side;  //1左线，2右线，0无
    EFMPoint merge_point;//merge点在自车坐标下的x,y
    int merge_point_index;
    bool merge_erea_is_drivable;
}BevLineInnerConnect;

typedef struct LineTypeInfoS{
    bool is_valid;
    float start_point;//车道线类型起点
    int line_type; //车道线类型 BstdsLineType
    float typ_chg_point; //车道线类型变化点距离自车的纵向距离
    int typ_aft_chg_point; //车道线类型发生变化后，车道线的类型   BstdsLineType
    LineTypeInfoS()
    :is_valid(false){}
      
}LineTypeInfo;

typedef struct SingleLineTypeInfoS{
    bool is_valid;
    float start_point;//车道线类型起点
    float end_point;//车道线类型终点
    int line_type; //车道线类型 BstdsLineType
    SingleLineTypeInfoS()
    :is_valid(false){}      
}SingleLineTypeInfo;

typedef struct BevLaneElementS {
    BevLaneElementS()
    : left_line_points{}
    , left_types{}
    , right_line_points{}
    , right_types{}
    , left_base_line_ptr(nullptr)
    , right_base_line_ptr(nullptr)
    , left_line_base_id(0)
    , right_line_base_id(0)
    , left_front_connect_id(0)
    , left_back_connect_id(0)
    , right_front_connect_id(0)
    , right_back_connect_id(0) 
    , left_line_split_index(-1)
    , left_line_merge_index(-1)
    , right_line_split_index(-1)
    , right_line_merge_index(-1)
    // , left_line_split_index_back(-1)
    // , left_line_merge_index_back(-1)
    // , right_line_split_index_back(-1)
    // , right_line_merge_index_back(-1)
    // , left_line_split_index_front(-1)
    // , left_line_merge_index_front(-1)
    // , right_line_split_index_front(-1)
    // , right_line_merge_index_front(-1)
    , left_line_split_x(0)
    , left_line_merge_x(0)
    , right_line_split_x(0)
    , right_line_merge_x(0)
    , center_line_points{}
    , left_overlap_right_start_index(-1)
    , left_overlap_right_end_index(-1)
    , right_overlap_left_start_index(-1)
    , right_overlap_left_end_index(-1)
    , merge_dir(0)
    , left_merge_point(-1)
    , right_merge_point(-1)
    , left_merge_narrow_pos(-1)
    , right_merge_narrow_pos(-1)
    , s_merge_point(-1)
    , e_merge_point(-1)
    , is_main_merge(true)
    , split_dir(0)
    , s_split_point(-1)
    , e_split_point(-1)
    , is_main_ele(false)
    , to_main_lane_times(0)
    , lane_width_last_cycle(-1) {}; 
    //是否需要车道线的起始和终止距离？？
    EFMRefLinePoints left_line_points;
    std::vector<LineTypeInfo> left_types; //从起点开始
    //LineTypeInfo left_type_sec;//第二段
    EFMRefLinePoints right_line_points;
    std::vector<LineTypeInfo> right_types;//从起点开始

    // LineTypeInfo right_type_first;//第一段
    // LineTypeInfo right_type_sec;//第二段
    //debug
    BevLineInner* left_base_line_ptr;
    BevLineInner* right_base_line_ptr;
    int left_line_base_id;
    int right_line_base_id;
    int left_front_connect_id;//线本身的vector的front()和back()
    int left_back_connect_id;
    int right_front_connect_id;
    int right_back_connect_id;
    int left_line_split_index;
    int left_line_merge_index;
    int right_line_split_index;
    int right_line_merge_index;
    //
    // int left_line_split_index_back;   //自车后方，距离自车最近的split点在left的index
    // int left_line_merge_index_back;   //自车后方，距离自车最近的merge点在left的index
    // int right_line_split_index_back;  //自车后方，距离自车最近的split点在right的index
    // int right_line_merge_index_back;  //自车后方，距离自车最近的merge点在right的index
    
    // int left_line_split_index_front;   //自车前方，距离自车最近的split点在left的index
    // int left_line_merge_index_front;   //自车前方，距离自车最近的merge点在left的index
    // int right_line_split_index_front;  //自车前方，距离自车最近的split点在right的index
    // int right_line_merge_index_front;  //自车前方，距离自车最近的merge点在right的index
    double left_line_split_x;
    double left_line_merge_x;
    double right_line_split_x;
    double right_line_merge_x;

    //std::vector<std::pair<int,int>> left_new_point_section;//表示哪些点是新的点
    //std::vector<std::pair<int,int>> right_new_point_section;//表示哪些点是新的点
    EFMRefLinePoints center_line_points; //中心线
    int32_t left_overlap_right_start_index; //左侧重叠的右侧线的起始点index
    int32_t left_overlap_right_end_index; //左侧重叠的右侧线的终止点index
    int32_t right_overlap_left_start_index; //右侧重叠的左侧线的起始点index
    int32_t right_overlap_left_end_index; //右侧重叠的左侧线的终止点index

    //融合后计算的到的属性
    uint8_t merge_dir; //合并方向，1-左侧合并，2-右侧合并，0-无
    int32_t left_merge_point; //左侧合并点在left_line_points中的index
    int32_t right_merge_point; //右侧合并点在right_line_points中的index
    int32_t left_merge_narrow_pos; //左侧合并点在left_line_points中的index, narrow_pos表示合并后变窄的点
    int32_t right_merge_narrow_pos; //右侧合并点在right_line_points中的index, narrow_pos表示合并后变窄的点
    int32_t s_merge_point; //右侧合并点在right_line_points中的index, narrow_pos表示合并后变窄的点
    int32_t e_merge_point; //右侧合并点在right_line_points中的index, narrow_pos表示合并后变窄的点
    bool    is_main_merge; //多合流道路中，是否为主路

    uint8_t split_dir; //分离方向，1-左侧分离，2-右侧分离，0-无
    int32_t s_split_point; //分离点的起点在center_line_points中的index
    int32_t e_split_point; //分离点的终点在center_line_points中的index
    bool is_main_ele; //多分歧道路中，是否为主路
    uint8_t to_main_lane_times; //到主车道的次数，0表示已经是主车道了，1表示需要一次变道到主车道，2表示需要两次变道到主车道
    double lane_width_last_cycle; //上个周期的车道宽度，滤波用

}BevLaneElement;

using BevLaneElementGroup = std::vector<BevLaneElement>;//一分多的会放在一起, 右侧的index小, 
using BevLaneElementGroupSet = std::vector<std::pair<int,BevLaneElementGroup>>; //右侧的index小 //first-0自车，1左，2右，255未分配, 99-左路沿，100-右路沿； second-车道group

typedef struct SDLaneElement {
    SDLaneElement()
    : lane_id(0)
    , ele_id(0)
    , link_path_ids{}
    , lane_in_link{}
    , lane_ids{}
    , lane_nums{}
    , lane_merges{}
    , lane_splits{}
    , lane_types{}
    , lane_routes{}
    , lane_s_offsets{}
    , lane_e_offsets{}
    , remain_dist(0)
    , all_length(0)
    , is_dest(false)
    , is_group_dest(false)
    , is_end(false)
    , first_merge_index(-1)
    , first_split_index(-1)
    , merge_count(0)
    , step_to_dest(UINT8_MAX) {}; 

    uint8_t lane_id;                             //自车所在link的lane_id，从右向左，全部车道需要排序，从1开始
    uint32_t ele_id;                             //该element的id，唯一标识一个element，和lane_id无关
    std::vector<uint64_t> link_path_ids;         //该element经过的link
    std::vector<int> lane_in_link;               //该element中，lane_ids与link_path_ids的对应关系，数量和lane_ids一致，内容为link在link_path_ids的index，-1表示该lane不在link_path_ids中
    std::vector<uint64_t>  lane_ids;             //该element经过的lane id
    std::vector<uint8_t>  lane_nums;             //该element经过的lane的num,与lane_ids数量一致
    std::vector<EfmMergeType>  lane_merges;      //该element经过的lane的merge属性,与lane_ids一一对应
    std::vector<EfmSplitType>  lane_splits;      //该element经过的lane的split属性,与lane_ids一一对应
    std::vector<LaneType>  lane_types;           //该element经过的lane的type属性,与lane_ids一一对应
    std::vector<bool>  lane_routes;              //该element经过的lane的route属性,与lane_ids一一对应，如果该车道需要经过无法经过虚线变到目标车道，则该属性为false
    std::vector<int> lane_s_offsets;             //该element中车道的起点相对于自车的距离，单位cm，与lane_ids一一对应
    std::vector<int> lane_e_offsets;             //该element中车道的终点相对于自车的距离，单位cm，与lane_ids一一对应
    uint32_t  remain_dist;                       //该element可行驶距离，单位cm
    uint32_t  all_length;                        //该element总长，单位cm
    bool is_dest;                                //是否为目标车道，所有的车道里面必须有目标车道
    bool is_group_dest;                          //是否为该group下的目标，每个group下必须最少一个
    bool is_end;                                 //当前检索已经到达最后一个车道了(path的最后一个link，或者子路径上)，后续不再继续检索
    int  first_merge_index;                      //该车道组内第一个merge的index，-1表示没有merge
    int  first_split_index;                      //该车道组内第一个split的index，-1表示没有split
    uint8_t merge_count;                         //该车道组内merge的数量, 0表示没有merge
    uint8_t step_to_dest;                        //该车道组到目标车道的步数，0表示已经是目标车道了
}SDLaneElement;
using SDLaneElementGroup = std::vector<SDLaneElement>;//一分多的会放在一起,根据自车前进方向，由近及远，从右向左排序，
using SDLaneElementGroupSet = std::vector<std::pair<uint8_t,SDLaneElementGroup>>; //first-lane id，second-车道group

struct MapRawDataMap{
    MapRawDataMap()
    : path_id_index_map{}
    , link_path_offset_id_index_map{}
    , lane_id_index_map{} {};
    ~MapRawDataMap() = default;

    std::unordered_map<uint32_t, int> path_id_index_map;
    std::unordered_map<uint64_t, int> link_path_offset_id_index_map;
    std::unordered_map<uint64_t, std::vector<int>> lane_id_index_map;
};

struct NodeInfo {
    NodeInfo()
    : StartPointOffset(0.0)
    , EndPointOffset(0.0)
    , dir(NODE_DIR_UNKNOWN)
    , LaneChgType(0)
    , LaneChgTimes(0) {}; 

    double StartPointOffset; //变道起始位置
    double EndPointOffset;   //变道终止位置
    NodeDir dir;             //变道方向
    uint8_t LaneChgType;     //bit位表示：bit0- with control, bit1- with light, bit2- off ramp, bit3- on ramp
    uint8_t LaneChgTimes;    //变道次数
};

struct OddDis{
    OddDis()
    : oddtype (0)
    , odddis (0){};
    OddDis(uint8_t type, uint32_t dis)
    : oddtype (type)
    , odddis (dis){};
    ~OddDis() = default;

    uint8_t oddtype;
    uint32_t odddis;
};

typedef struct StoredInfoS{
    StoredInfoS()
    :lane_id(0)
    ,lane_num(0){};
    uint64_t lane_id; //车道id
    uint8_t lane_num; //车道数
    std::vector<int> ego_left_line_ids; //自车左侧车道线id,包含拼接的id
    std::vector<int> ego_right_line_ids; //自车右侧车道线id,包含拼接的id
}StoredInfo;

struct LocJudgeInfo {
    LocJudgeInfo()
    : loc_lane_num_(0)
    , loc_judge_dir_(0){};
    uint8_t loc_lane_num_;
    uint8_t loc_judge_dir_;//1-左侧路沿；2-右侧路沿；0-历史；
};
