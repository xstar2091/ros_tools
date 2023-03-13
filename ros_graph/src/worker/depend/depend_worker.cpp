#include "ros_graph/worker/depend/depend_worker.h"

#include <queue>
#include <unordered_set>
#include <fmt/format.h>

DependWorker::~DependWorker()
{}

bool DependWorker::init(int& argc, char**& argv)
{
    if (!command_line_param_.init(argc, argv)) return false;
    if (!param_.reset(command_line_param_)) return false;
    if (!depend_graph_.init(param_.workspace_dir)) return false;
    return true;
}

bool DependWorker::check()
{
    if (param_.package.empty())
    {
        fmt::print(stderr, "not implemented for all packages depend analysis\n");
        return false;
    }
    if (!depend_graph_.depend_graph().isNodeExist(param_.package))
    {
        fmt::print(stderr, "unknown package: {}\n", param_.package);
        return false;
    }
    return true;
}

bool DependWorker::run()
{
    std::unordered_set<std::string> table{param_.package};
    std::queue<GraphNode<std::string>*> queue;
    queue.push(depend_graph_.depend_graph().nodes()[param_.package]);
    while (!queue.empty())
    {
        GraphNode<std::string>* package = queue.front();
        queue.pop();
        for (auto& pair : package->next_nodes())
        {
            queue.push(pair.second);
            table.insert(pair.second->value());
        }
    }

    if (table.empty()) return true;

    auto it = table.begin();
    fmt::print("{}", *it);
    for (++it; it != table.end(); ++it)
    {
        fmt::print("{}{}", param_.separator, *it);
    }
    fmt::print("\n");
    return true;
}
