#pragma once

#include <unordered_set>
#include "ros_graph/worker/worker.h"
#include "ros_graph/worker/dependtreeby/dependtreeby_worker_param.h"

class DependTreeByWorker : public Worker
{
public:
    virtual ~DependTreeByWorker() override;
    virtual bool init(int& argc, char**& argv) override;
    virtual bool check() override;
    virtual bool run() override;

private:
    void findPackageDependTree(const std::string& package_name);
    void findPackageDependTree(int indent, int level, GraphNode<std::string>* package);
    void printPackage(int indent, const std::string& package_name);

private:
    DependTreeByWorkerParam param_;
    std::vector<std::string> package_list_;
};
