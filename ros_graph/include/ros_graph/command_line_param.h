#pragma once

#include <string>

class CommandLineParam
{
public:
    CommandLineParam();
    ~CommandLineParam();
    bool init(int& argc, char**& argv);

public:
    std::string module_name;
    std::string ros_tool_dir;
    std::string command;

public:
    bool is_level_set_by_cmd;
    bool is_package_set_by_cmd;
    bool is_workspace_dir_set_by_cmd;
    bool is_separator_set_by_cmd;

    int level;

    std::string package;
    std::string workspace_dir;
    std::string separator;

private:
    bool findRosToolDir();
    // 从.ros_tool/ros_graph/${module_name}.ini配置文件中读取参数
    void readParamFromRosToolDir();
    void readParamFromCommandLine(int& argc, char**& argv);
};
