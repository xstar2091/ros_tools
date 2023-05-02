#include "ros_graph/worker/dependtree/dependtree_worker_param.h"

#include <fmt/format.h>
#include "ros_graph/command_line_param.h"

bool DependTreeWorkerParam::reset(const CommandLineParam& param)
{
    package = param.package;
    workspace_dir = param.workspace_dir;
    separator = param.separator;
    indent = param.indent;

    if (separator == "\n" || separator.size() > 1 || separator.empty())
    {
        separator = " ";
        separator_format_string = "{: <{}}";
    }
    else if (separator == "\t")
    {
        separator_format_string = "{:\t<{}}";
    }
    else
    {
        separator_format_string = fmt::format("{{:{}<{{}}}}", separator[0]);
    }
    return check();
}

bool DependTreeWorkerParam::check()
{
    return true;
}
