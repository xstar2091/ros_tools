#include "ros_graph/command_line_param.h"

#include <limits>
#include <boost/filesystem.hpp>
#include "ros_common/cmdkey/cmdkey.h"
#include "ros_common/cmdkey/cmdkey_info.h"

#define ROS_GRAPH_ROS_TOOL_DIR_NAME ".ros_tool"
#define ROS_GRAPH_DEFAULT_PARAM_PACKAGE ""
#define ROS_GRAPH_DEFAULT_PARAM_WORKSPACE_DIR ""
#define ROS_GRAPH_DEFAULT_PARAM_SEPARATOR ";"

define_cmdkey_int32(level, 0, "");
define_cmdkey_int32(indent, 4, "");
define_cmdkey_string(package, ROS_GRAPH_DEFAULT_PARAM_PACKAGE, "");
define_cmdkey_string(workspace_dir, ROS_GRAPH_DEFAULT_PARAM_WORKSPACE_DIR, "");
define_cmdkey_string(separator, ROS_GRAPH_DEFAULT_PARAM_SEPARATOR, "");

CommandLineParam::CommandLineParam()
    : module_name("ros_graph")
    , is_level_set_by_cmd(false)
    , is_indent_set_by_cmd(false)
    , is_package_set_by_cmd(false)
    , is_workspace_dir_set_by_cmd(false)
    , is_separator_set_by_cmd(false)
    , level(0)
    , indent(4)
{}

bool CommandLineParam::init(int& argc, char**& argv)
{
    command = argv[1];
    readParamFromCommandLine(argc, argv);
    if (findRosToolDir())
    {
        readParamFromRosToolDir();
    }
    return true;
}

bool CommandLineParam::findRosToolDir()
{
    boost::filesystem::path path = boost::filesystem::current_path();
    boost::filesystem::path ros_tool_path;
    std::string tmp = path.string();
    bool ret = false;
    do
    {
        ros_tool_path = path / ROS_GRAPH_ROS_TOOL_DIR_NAME;
        if (boost::filesystem::is_directory(ros_tool_path))
        {
            ros_tool_dir = ros_tool_path.string();
            tmp = path.string();
            ret = true;
            break;
        }
        path = path.parent_path();
    } while (!path.empty());

    if (!is_workspace_dir_set_by_cmd)
    {
        workspace_dir = tmp;
    }
    return ret;
}

void CommandLineParam::readParamFromRosToolDir()
{
    boost::filesystem::path path(ros_tool_dir);
    path = path / module_name / command;
    std::string file_path = path.string() + ".ini";
    if (boost::filesystem::is_regular_file(file_path))
    {
        Cmdkey::parseFile(file_path.c_str());
        if (!is_level_set_by_cmd) level = cmdkey_level > 0 ? cmdkey_level : std::numeric_limits<int>::max();
        if (!is_indent_set_by_cmd) indent = cmdkey_indent > 0 ? cmdkey_indent : 4;
        if (!is_package_set_by_cmd) package = cmdkey_package;
        if (!is_separator_set_by_cmd) separator = cmdkey_separator;
    }
}

void CommandLineParam::readParamFromCommandLine(int& argc, char**& argv)
{
    Cmdkey::parseCommandLine(argc, argv, true);
    CmdkeyInfo info;

    info.find("level");
    if (!info.is_default) is_level_set_by_cmd = true;
    level = cmdkey_level > 0 ? cmdkey_level : std::numeric_limits<int>::max();

    info.find("indent");
    if (!info.is_default) is_indent_set_by_cmd = true;
    indent = cmdkey_indent > 0 ? cmdkey_indent : 4;

    info.find("package");
    if (!info.is_default) is_package_set_by_cmd = true;
    package = cmdkey_package;

    info.find("workspace_dir");
    if (!info.is_default) is_workspace_dir_set_by_cmd = true;
    workspace_dir = cmdkey_workspace_dir;

    info.find("separator");
    if (!info.is_default) is_separator_set_by_cmd = true;
    separator = cmdkey_separator;
    if (separator == "\\n") separator = "\n";
    if (separator == "\\r\\n") separator = "\r\n";
    if (separator == ROS_GRAPH_DEFAULT_PARAM_SEPARATOR)
    {
        if (command == "dependtree" || command == "dependtreeby")
        {
            separator = " ";
        }
    }
}
