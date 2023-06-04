#include "ros_graph/worker/depend/depend_worker_param.h"

#include "ros_graph/command_line_param.h"

bool DependWorkerParam::reset(const CommandLineParam& param)
{
    level = param.level;
    package = param.package;
    workspace_dir = param.workspace_dir;
    separator = param.separator;
    return check();
}

bool DependWorkerParam::check()
{
    return true;
}
