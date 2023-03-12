#include "ros_graph/worker/worker.h"
#include "ros_graph/worker/depend/depend_worker.h"
#include <fmt/format.h>

Worker::~Worker()
{}

Worker* Worker::create(const char *command)
{
    Worker* worker = nullptr;
    if (strcmp(command, "depend") == 0)
    {
        worker = new DependWorker;
    }

    if (worker == nullptr)
    {
        fmt::print(stderr, "invalid command: {}\n", command);
    }
    return worker;
}

void Worker::printDebugInfo()
{
    fmt::print("-----------------command line params-----------------\n");
    fmt::print("  module_name: {}\n", command_line_param_.module_name);
    fmt::print(" ros_tool_dir: {}\n", command_line_param_.ros_tool_dir);
    fmt::print("      command: {}\n", command_line_param_.command);

    fmt::print("      package, from_cmd: {}, value: {}\n",
        command_line_param_.is_package_set_by_cmd, command_line_param_.package);
    fmt::print("workspace_dir, from_cmd: {}, value: {}\n",
        command_line_param_.is_workspace_dir_set_by_cmd, command_line_param_.workspace_dir);
}
