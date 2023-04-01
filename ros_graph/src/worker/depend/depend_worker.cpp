#include "ros_graph/worker/depend/depend_worker.h"

#include <algorithm>
#include <queue>
#include <boost/algorithm/string.hpp>
#include <fmt/format.h>

DependWorker::~DependWorker()
{}

bool DependWorker::init(int& argc, char**& argv)
{
    if (!command_line_param_.init(argc, argv)) return false;
    if (!param_.reset(command_line_param_)) return false;
    if (!depend_graph_.init(param_.workspace_dir)) return false;

    if (!param_.package.empty())
    {
        boost::split(package_list_, param_.package, boost::is_any_of(";"));
    }
    else
    {
        for (const auto& pair : depend_graph_.depend_map())
        {
            package_list_.emplace_back(pair.first);
        }
    }
    package_list_.erase(std::remove(package_list_.begin(), package_list_.end(), std::string()), package_list_.end());
    return true;
}

bool DependWorker::check()
{
    for (const auto& package : package_list_)
    {
        if (!depend_graph_.depend_graph().isNodeExist(package))
        {
            fmt::print(stderr, "unknown package: {}\n", package);
            return false;
        }
    }
    // if (param_.package.empty())
    // {
    //     fmt::print(stderr, "not implemented for all packages depend analysis\n");
    //     return false;
    // }
    return true;
}

bool DependWorker::run()
{
    std::vector<std::string> package_list;
    std::unordered_set<std::string> table;
    for (const auto& package : package_list_)
    {
        findPackages(package, table, package_list);
    }
    if (package_list.empty()) return true;
    sortPackages(package_list);
    printPackages(package_list);
    
    return true;
}

void DependWorker::findPackages(const std::string& package_name,
                                std::unordered_set<std::string>& table,
                                std::vector<std::string>& package_list)
{
    if (table.count(package_name) == 0)
    {
        package_list.emplace_back(package_name);
    }
    std::queue<GraphNode<std::string>*> queue;
    queue.push(depend_graph_.depend_graph().nodes()[package_name]);
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
