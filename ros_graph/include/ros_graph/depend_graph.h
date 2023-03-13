#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>
#include "ros_graph/graph.h"

class PackageInfo
{
public:
    // package目录，该目录下存在CMakeLists.txt和package.xml两个文件
    std::string path;
    std::string cmake_lists_txt_file_path;
    std::string package_xml_file_path;
    std::string package_name;

public:
    bool init();
    void findDependPackage(const std::function<void(const std::string&, const std::string&)>& on_package_found);
};

class DependGraph
{
public:
    bool init(const std::string& workspace_dir);

    Graph<std::string>& depend_graph() { return depend_graph_; }
    std::unordered_map<std::string, PackageInfo>& depend_map() { return depend_map_; }

private:
    void readAllPackage(const std::string& workspace_dir);

private:
    Graph<std::string> depend_graph_;
    std::unordered_map<std::string, PackageInfo> depend_map_;
};
