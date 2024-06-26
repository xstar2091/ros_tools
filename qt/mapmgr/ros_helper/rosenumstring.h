#ifndef ROSENUMSTRING_H
#define ROSENUMSTRING_H

#include <QString>

struct MultimapStatus
{
    const QString& toString(int16_t val) const;
};

struct MapSwitchType
{
    const QString& toString(uint8_t val) const;
};

class RosEnumString
{
public:
    static RosEnumString* instance();
    const MapSwitchType& map_switch_type() const { return map_switch_type_; }
    const MultimapStatus& multimap_status() const { return multimap_status_; }

private:
    MapSwitchType map_switch_type_;
    MultimapStatus multimap_status_;
};

#endif // ROSENUMSTRING_H
