#ifndef MAPCOLLECTION_H
#define MAPCOLLECTION_H

#include <string>
#include <unordered_map>
#include <vector>
#include "mapgroup.h"

class MapCollection
{
public:
    void clear();
    void init(const std::string& collection_info_file);
    const MapGroup* findGroup(const std::string& group_id) const;

public:
    const std::string& id() const { return id_; }
    const std::string& name() const { return name_; }
    const MapGroup* default_group() const { return default_group_; }
    const std::vector<MapGroup>& group_list() const { return group_list_; }

private:
    std::string id_;
    std::string name_;
    MapGroup* default_group_;
    std::vector<MapGroup> group_list_;
    std::unordered_map<std::string, MapGroup*> group_table_;
};

#endif // MAPCOLLECTION_H
