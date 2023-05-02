#pragma once

#include <unordered_set>
#include "ros_graph/worker/worker.h"
#include "ros_graph/worker/dependtree/dependtree_worker_param.h"

class DependTreeWorker : public Worker
{
public:
    virtual ~DependTreeWorker() override;
    virtual bool init(int& argc, char**& argv) override;
    virtual bool check() override;
    virtual bool run() override;

private:
    void findPackageDependTree(const std::string& package_name);
    void findPackageDependTree(int indent, GraphNode<std::string>* package);
    void printPackage(int indent, const std::string& package_name);

private:
    DependTreeWorkerParam param_;
    std::vector<std::string> package_list_;
};
