#include "ros_graph/command_line_param.h"

#define STRIP_FLAG_HELP 1

#include <boost/filesystem.hpp>
#include <gflags/gflags.h>

#define ROS_GRAPH_ROS_TOOL_DIR_NAME ".ros_tool"
#define ROS_GRAPH_DEFAULT_PARAM_PACKAGE ""
#define ROS_GRAPH_DEFAULT_PARAM_WORKSPACE_DIR ""

DEFINE_string(package, ROS_GRAPH_DEFAULT_PARAM_PACKAGE, "");
DEFINE_string(workspace_dir, ROS_GRAPH_DEFAULT_PARAM_WORKSPACE_DIR, "");

CommandLineParam::CommandLineParam()
    : module_name("ros_graph")
    , is_package_set_by_cmd(false)
    , is_workspace_dir_set_by_cmd(false)
{}

CommandLineParam::~CommandLineParam()
{
    gflags::ShutDownCommandLineFlags();
}

bool CommandLineParam::init(int& argc, char** argv)
{
    std::string command(argv[1]);
    readParamFromCommandLine(argc, argv);
    if (findRosToolDir())
    {
        readParamFromRosToolDir(command);
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
    return false;
}

void CommandLineParam::readParamFromRosToolDir(const std::string& command)
{
    boost::filesystem::path path(ros_tool_dir);
    path = path / module_name / command;
    std::string file_path = path.string() + ".ini";
    gflags::SetCommandLineOption("flagfile", file_path.c_str());

    package = FLAGS_package;
}

void CommandLineParam::readParamFromCommandLine(int& argc, char** argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    gflags::CommandLineFlagInfo info;
    gflags::GetCommandLineFlagInfo("package", &info);
    if (!info.is_default)
    {
        package = FLAGS_package;
        is_package_set_by_cmd = true;
    }
    gflags::GetCommandLineFlagInfo("workspace_dir", &info);
    if (!info.is_default)
    {
        workspace_dir = FLAGS_workspace_dir;
        is_workspace_dir_set_by_cmd = true;
    }
}
