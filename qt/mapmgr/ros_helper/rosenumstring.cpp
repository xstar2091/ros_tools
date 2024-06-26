#include "rosenumstring.h"

#include <unordered_map>

RosEnumString *RosEnumString::instance()
{
    static RosEnumString inst;
    return &inst;
}

namespace
{

std::unordered_map<int16_t, QString> multimap_status_table = {
    {0, "成功"},
    {1, "计算中"},
    {2, "失败"},
};

std::unordered_map<uint8_t, QString> map_switch_type_table = {
    {1, "电梯"},
    {2, "电梯"},
};

QString default_string("unknown");

}

const QString &MultimapStatus::toString(int16_t val) const
{
    auto it = multimap_status_table.find(val);
    if (it != multimap_status_table.end())
    {
        return it->second;
    }
    return default_string;
}

const QString &MapSwitchType::toString(uint8_t val) const
{
    auto it = map_switch_type_table.find(val);
    if (it != map_switch_type_table.end())
    {
        return it->second;
    }
    return default_string;
}
