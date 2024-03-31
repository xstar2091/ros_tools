#include "mapcollection.h"

#include <boost/filesystem.hpp>
#include <boost/property_tree/xml_parser.hpp>

namespace
{

std::string default_map_info_name("mapInfo.xml");
std::string default_collection_info_name("collectionInfo.xml");

}

void MapCollection::clear()
{
    id_ = "";
    name_ = "";
    default_group_ = nullptr;
    group_list_.clear();
    group_table_.clear();
}

void MapCollection::init(const std::string &collection_info_file)
{
    clear();
    boost::property_tree::ptree ptree;
    boost::property_tree::xml_parser::read_xml(collection_info_file, ptree);
    const boost::property_tree::ptree& root = ptree.get_child("xml");

    id_ = root.get_child("id").get_value<std::string>();
    name_ = root.get_child("name").get_value<std::string>();
    for (auto& group_node : root)
    {
        if (group_node.first != "group") continue;
        for (auto& child : group_node.second)
        {
            if (child.first != "path") continue;
            group_list_.emplace_back(MapGroup());
            boost::filesystem::path map_info_file_path(child.second.get_value<std::string>());
            map_info_file_path /= default_map_info_name;
            if (!group_list_.back().init(this, map_info_file_path.string()))
            {
                group_list_.pop_back();
            }
        }
    }
    for (MapGroup& group : group_list_)
    {
        group_table_.insert(std::make_pair(group.id(), &group));
    }

    const boost::property_tree::ptree& default_group_id_node = root.get_child("defgroup.id");
    std::string default_group_id = default_group_id_node.get_value<std::string>();
    auto it = group_table_.find(default_group_id);
    if (it != group_table_.end())
    {
        default_group_ = it->second;
    }
}

const MapGroup *MapCollection::findGroup(const std::string &group_id) const
{
    const MapGroup* group = nullptr;
    auto it = group_table_.find(group_id);
    if (it != group_table_.end())
    {
        group = it->second;
    }
    return group;
}
