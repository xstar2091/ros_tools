#include "ros_graph/command_line_param.h"

//#define STRIP_FLAG_HELP 1

#include <boost/filesystem.hpp>
#include <fmt/format.h>
#include <gflags/gflags.h>

#define ROS_GRAPH_ROS_TOOL_DIR_NAME ".ros_tool"
#define ROS_GRAPH_DEFAULT_PARAM_PACKAGE ""
#define ROS_GRAPH_DEFAULT_PARAM_WORKSPACE_DIR ""
#define ROS_GRAPH_DEFAULT_PARAM_SEPARATOR ";"

DEFINE_string(package, ROS_GRAPH_DEFAULT_PARAM_PACKAGE, "");
DEFINE_string(workspace_dir, ROS_GRAPH_DEFAULT_PARAM_WORKSPACE_DIR, "");
DEFINE_string(separator, ROS_GRAPH_DEFAULT_PARAM_SEPARATOR, "");

CommandLineParam::CommandLineParam()
    : module_name("ros_graph")
    , is_debug_mode_set_by_cmd(false)
    , is_package_set_by_cmd(false)
    , is_workspace_dir_set_by_cmd(false)
    , is_separator_set_by_cmd(false)
    , is_debug_mode(true)
{}

CommandLineParam::~CommandLineParam()
{
    gflags::ShutDownCommandLineFlags();
}

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
    if (is_debug_mode)
    {
        if (is_workspace_dir_set_by_cmd)
        {
            fmt::print("workspace_dir is set from command line:{}\n", workspace_dir);
        }
        else if (ret)
        {
            fmt::print("ros_tool path is found:{}, set workspace_dir to:{}\n", ros_tool_path.string(), workspace_dir);
        }
        else
        {
            fmt::print("ros_tool path is not found, set workspace_dir to:{}\n", workspace_dir);
        }
    }
    return false;
}

void CommandLineParam::readParamFromRosToolDir()
{
    boost::filesystem::path path(ros_tool_dir);
    path = path / module_name / command;
    std::string file_path = path.string() + ".ini";
    gflags::SetCommandLineOption("flagfile", file_path.c_str());

    if (!is_package_set_by_cmd) package = FLAGS_package;
    if (!is_separator_set_by_cmd) separator = FLAGS_separator;
}

void CommandLineParam::readParamFromCommandLine(int& argc, char**& argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    gflags::CommandLineFlagInfo info;

    gflags::GetCommandLineFlagInfo("package", &info);
    if (!info.is_default) is_package_set_by_cmd = true;
    package = FLAGS_package;

    gflags::GetCommandLineFlagInfo("workspace_dir", &info);
    if (!info.is_default) is_workspace_dir_set_by_cmd = true;
    workspace_dir = FLAGS_workspace_dir;

    gflags::GetCommandLineFlagInfo("separator", &info);
    if (!info.is_default) is_separator_set_by_cmd = true;
    separator = FLAGS_separator;
    if (separator == "\\n") separator = "\n";
    if (separator == "\\r\\n") separator = "\r\n";
}
