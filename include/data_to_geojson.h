#ifndef	_DATA_TO_GEOJSON_H_
#define	_DATA_TO_GEOJSON_H_

// #include "topic/topic_trait.h"
#include "common/framework.h"
#include "hdm_dbg_log.h"
// #include "CommonDataType.h"

namespace earth {
namespace shell {
namespace framework {

class CDataToJson
{
public:
    typedef struct EFMPoint2D {
    EFMPoint2D(double x_in, double y_in) : x(x_in), y(y_in){};
    EFMPoint2D() : x(0.0), y(0.0){};
    ~EFMPoint2D() = default;
    double x;
    double y;
    } EFMPoint1;

    CDataToJson();
	virtual ~CDataToJson();
 
	bool GenerateData(const TopicTrait::MapMapMsg& map_msg, const TopicTrait::MapPositionMsg& position_msg,
                      const TopicTrait::MapSwitchInfoMsg& switch_info_msg, const TopicTrait::MapRouteListMsg& map_route_list,
                      const TopicTrait::MapGlobalDataMsg& global_data_msg, const TopicTrait::MapDynamicMsg& map_dynamic_msg,
                      uint64_t efm_counter);

private:
    bool DownMapToGeoJson(const TopicTrait::MapMapMsg& map_msg);
    bool DownPositionToGeoJson(const TopicTrait::MapPositionMsg& position_msg, const TopicTrait::MapRouteListMsg& map_route_list);
    bool DownSwitchToGeoJson(const TopicTrait::MapSwitchInfoMsg& switch_info_msg, const TopicTrait::MapMapMsg& map_msg);
    bool DownRouteToGeoJson(const TopicTrait::MapRouteListMsg& map_route_list);
    bool DownGlobalToGeoJson(const TopicTrait::MapGlobalDataMsg& global_data_msg);
    bool DownDynamicToGeoJson(const TopicTrait::MapDynamicMsg& map_dynamic_msg);

    // bool GetSectionAttribute(std::shared_ptr<zone::common::EHRSection> section_i, const char* file_name_char_section);
    // bool GetLaneLineAttribute(std::shared_ptr<zone::common::EHRSection> section_i, const char* file_name_char_line);

    std::unordered_map<uint64_t, uint8_t>  path_id_link_id_map_;
    std::unordered_map<uint64_t, uint8_t>  path_id_link_idkerb_map_;
    std::unordered_map<uint64_t, uint8_t>  path_id_link_id_connect_map_;
    std::unordered_map<uint64_t, uint8_t>  path_id_link_id_cur_map_;
    std::unordered_map<uint32_t, uint8_t>  lineatobject_id_cur_map_;
    std::unordered_map<uint64_t, uint8_t>  path_id_link_id_cur_geofence_;
    std::unordered_map<uint64_t, uint8_t>  path_id_link_id_slop_map_;
    std::shared_ptr<const TopicTrait::MapMapMsg> map_static_info_;
    std::shared_ptr<const TopicTrait::MapPositionMsg> position_msg_;
    std::shared_ptr<const TopicTrait::MapSwitchInfoMsg> switch_info_msg_;
    std::shared_ptr<const TopicTrait::MapRouteListMsg> map_route_list_;
    std::shared_ptr<const TopicTrait::MapGlobalDataMsg> global_data_msg_;
    std::shared_ptr<const TopicTrait::MapDynamicMsg> map_dynamic_msg_;

    uint32_t write_times_;
    uint32_t write_times_kerb_;
    uint32_t write_times_connect_;
    uint32_t write_times_global_;
    uint64_t efm_counter_;
    uint64_t position_counter_;
    const char* file_name_char_section_c_;
    const char* file_name_char_section_l_;
    const char* file_name_char_section_kerb_;
    const char* file_name_char_position_;
    const char* file_name_char_connect_;
    const char* file_name_char_curvature_;
    const char* file_name_char_slop_;
    const char* file_name_char_switch_;
    const char* file_name_char_switch_global_;
    const char* file_name_char_geofence_;


};

}  // namespace framework
}  // namespace shell
}  // namespace earth

#endif	//	_DATA_TO_GEOJSON_H_
