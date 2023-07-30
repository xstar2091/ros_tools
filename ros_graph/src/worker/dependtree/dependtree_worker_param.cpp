#include "ros_graph/worker/dependtree/dependtree_worker_param.h"

#include "ros_graph/command_line_param.h"

bool DependTreeWorkerParam::reset(const CommandLineParam& param)
{
    package = param.package;
    workspace_dir = param.workspace_dir;
    separator = param.separator;
    indent = param.indent;
    level = param.level;

    if (separator == "\n" || separator.size() > 1 || separator.empty())
    {
        separator = " ";
    }
    return check();
}

bool DependTreeWorkerParam::check()
{
    return true;
}
