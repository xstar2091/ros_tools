#include "ros_graph/worker/depend/depend_worker.h"

#include <algorithm>
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
    std::vector<std::string> package_list;
    findPackages(package_list);
    if (package_list.empty()) return true;
    sortPackages(package_list);
    printPackages(package_list);
    
    return true;
}

void DependWorker::findPackages(std::vector<std::string>& package_list)
{
    package_list.emplace_back(param_.package);
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
            const std::string& package = pair.second->value();
            if (table.count(package) == 0)
            {
                table.insert(package);
                package_list.emplace_back(package);
            }
        }
    }
}

void DependWorker::sortPackages(std::vector<std::string>& package_list)
{
    for (size_t i = 0; i < package_list.size(); i++)
    {
        for (size_t j = i + 1; j < package_list.size(); j++)
        {
            if (depend_graph_.depend_graph().isEdgeExist(package_list[i], package_list[j]))
            {
                std::swap(package_list[i], package_list[j]);
            }
        }
    }
}

void DependWorker::printPackages(std::vector<std::string>& package_list)
{
    auto it = package_list.begin();
    fmt::print("{}", *it);
    for (++it; it != package_list.end(); ++it)
    {
        fmt::print("{}{}", param_.separator, *it);
    }
    fmt::print("\n");
}
