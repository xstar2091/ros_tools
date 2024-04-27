#include "mapgroup.h"
#include <boost/property_tree/xml_parser.hpp>

void MapGroup::clear()
{
    collection_ = nullptr;
    map2d_.pgm.clear();
    map2d_.yaml.clear();
    vectormap_.osm.clear();
    id_ = "";
    name_ = "";
    path_ = "";
}

bool MapGroup::init(MapCollection *collection, const std::string &map_info_file)
{
    clear();
    collection_ = collection;
    {
        FileInfo map_info;
        map_info.reset(map_info_file);
        path_ = map_info.parent();
    }

    boost::property_tree::ptree ptree;
    boost::property_tree::xml_parser::read_xml(map_info_file, ptree);
    boost::property_tree::ptree& root = ptree.get_child("xml");

    std::string folder, suffix, path;
    id_ = root.get_child("id").get_value<std::string>();
    name_ = root.get_child("name").get_value<std::string>();
    for (auto& file_node : root)
    {
        if (file_node.first != "file") continue;
        folder = file_node.second.get_child("folder").get_value<std::string>();
        suffix = file_node.second.get_child("suffix").get_value<std::string>();
        path = file_node.second.get_child("path").get_value<std::string>();
        if (folder == "2dmap")
        {
            if (suffix == "pgm")
            {
                map2d_.pgm.reset(path);
            }
            else if (suffix == "yaml")
            {
                map2d_.yaml.reset(path);
            }
        }
        else if (folder == "vectormap")
        {
            if (suffix == "osm")
            {
                vectormap_.osm.reset(path);
            }
        }
    }
    if (name_.empty())
    {
        name_ = "未命名地图";
    }

    return !map2d_.pgm.file_name().empty() && !map2d_.yaml.file_name().empty() &&
            !vectormap_.osm.file_name().empty() && !id_.empty();
}
