#pragma once

#include <unordered_set>
#include "ros_graph/worker/worker.h"
#include "ros_graph/worker/dependby/dependby_worker_param.h"

class DependByWorker : public Worker
{
public:
    virtual ~DependByWorker() override;
    virtual bool init(int& argc, char**& argv) override;
    virtual bool check() override;
    virtual bool run() override;

private:
    void findPackages(const std::string& package_name,
                      std::unordered_set<std::string>& table,
                      std::vector<std::string>& package_list);
    void sortPackages(std::vector<std::string>& package_list);
    void printPackages(std::vector<std::string>& package_list);

private:
    DependByWorkerParam param_;
    std::vector<std::string> package_list_;
};
