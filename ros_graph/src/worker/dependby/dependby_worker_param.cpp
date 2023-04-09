#include "ros_graph/worker/dependby/dependby_worker_param.h"

#include "ros_graph/command_line_param.h"

bool DependByWorkerParam::reset(const CommandLineParam& param)
{
    level = param.level;
    package = param.package;
    workspace_dir = param.workspace_dir;
    separator = param.separator;
    return check();
}

bool DependByWorkerParam::check()
{
    return true;
}
