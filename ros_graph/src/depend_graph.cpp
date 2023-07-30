#include "ros_graph/depend_graph.h"

#include <fstream>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

bool doFindDependPackage(std::ifstream& ifs, std::vector<std::string>& vec)
{
    vec.clear();
    std::string line;
    bool is_find_package_start = false;
    bool is_find_package_end = false;
    while (std::getline(ifs, line))
    {
        boost::trim(line);
        if (boost::starts_with(line, "find_package"))
        {
            is_find_package_start = true;
            break;
        }
    }

    if (!is_find_package_start) return false;
    size_t i = 12;
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) i++;
    if (i >= line.size()) return false;
    if (line[i++] != '(') return false;
    line = line.substr(i);

    std::vector<std::string> depend_package_vec;
    while (!line.empty() && !boost::ends_with(line, ")"))
    {
        depend_package_vec.clear();
        boost::split(depend_package_vec, line, boost::is_any_of(" \t"));
        for (auto& package : depend_package_vec) vec.emplace_back(package);

        line = "";
        if (!std::getline(ifs, line)) break;
        boost::trim(line);
    }

    if (line.back() == ')') line.pop_back();
    boost::split(depend_package_vec, line, boost::is_any_of(" \t"));
    for (const auto& depend_package : depend_package_vec) vec.emplace_back(depend_package);
    return true;
}

bool PackageInfo::init()
{
    std::ifstream ifs(package_xml_file_path);
    if (!ifs)
    {
        std::cerr<<"open package.xml failed: "<<package_xml_file_path<<std::endl;
        return false;
    }
    std::string line;
    while (std::getline(ifs, line))
    {
        boost::trim(line);
        if (!boost::starts_with(line, "<name>") || !boost::ends_with(line, "</name>")) continue;
        package_name = line.substr(6, line.size() - 13);
        break;
    }
    return !package_name.empty();
}

void PackageInfo::findDependPackage(const std::function<void(const std::string&, const std::string&)>& on_package_found)
{
    std::ifstream ifs(cmake_lists_txt_file_path);
    if (!ifs)
    {
        std::cerr<<"open CMakeLists.txt failed: "<<cmake_lists_txt_file_path<<std::endl;
        return;
    }

    std::vector<std::string> vec;
    while (doFindDependPackage(ifs, vec))
    {
        if (vec.empty()) continue;
        if (vec[0] != "catkin") continue;
        size_t i = 1;
        if (vec.size() > i && vec[i] == "REQUIRED") i++;
        if (vec.size() > i && vec[i] == "COMPONENTS") i++;
        for (; i < vec.size(); i++)
        {
            on_package_found(package_name, vec[i]);
        }
        break;
    }
}

bool DependGraph::init(const std::string& workspace_dir)
{
    readAllPackage(workspace_dir);
    if (depend_map_.empty())
    {
        std::cerr<<"init depend graph failed, no package found"<<std::endl;
        return false;
    }
    for (auto& pair : depend_map_)
    {
        depend_graph_.createNode(pair.first);
    }
    for (auto& pair : depend_map_)
    {
        pair.second.findDependPackage([this] (const std::string& package, const std::string& depend_package) {
            auto it = this->depend_map_.find(depend_package);
            if (it == this->depend_map_.end()) return;
            this->depend_graph_.createEdge(package, depend_package);
        });
    }
    return true;
}

void DependGraph::readAllPackage(const std::string& workspace_dir)
{
    boost::filesystem::path package_xml_path;
    boost::filesystem::path cmake_lists_txt_path;
    boost::filesystem::recursive_directory_iterator end_it;
    boost::filesystem::recursive_directory_iterator it(workspace_dir);
    for (; it != end_it; ++it)
    {
        if (!boost::filesystem::is_directory(it->path())) continue;
        package_xml_path = it->path() / "package.xml";
        cmake_lists_txt_path = it->path() / "CMakeLists.txt";
        if (!boost::filesystem::is_regular_file(package_xml_path) ||
            !boost::filesystem::is_regular_file(cmake_lists_txt_path)) continue;
        PackageInfo info;
        info.path = it->path().string();
        info.package_xml_file_path = package_xml_path.string();
        info.cmake_lists_txt_file_path = cmake_lists_txt_path.string();
        if (!info.init()) continue;
        depend_map_.insert(std::make_pair(info.package_name, info));
    }
}
