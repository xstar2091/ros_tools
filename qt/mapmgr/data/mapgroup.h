#ifndef MAPGROUP_H
#define MAPGROUP_H

#include "fileinfo.h"

class MapCollection;

struct Map2D
{
    FileInfo pgm;
    FileInfo yaml;
};

struct VectorMap
{
    FileInfo osm;
};

class MapGroup
{
public:
    void clear();
    bool init(MapCollection* collection, const std::string& map_info_file);

public:
    const MapCollection* collection() const { return collection_; }
    const Map2D& map2d() const { return map2d_; }
    const VectorMap& vectormap() const { return vectormap_; }
    const std::string& id() const { return id_; }
    const std::string& name() const { return name_; }
    const std::string& path() const { return path_; }

private:
    MapCollection* collection_;
    Map2D map2d_;
    VectorMap vectormap_;
    std::string id_;
    std::string name_;
    std::string path_;
};

#endif // MAPGROUP_H
