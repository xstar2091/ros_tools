#pragma once

#include "ros_graph/worker/worker.h"
#include "ros_graph/worker/depend/depend_worker_param.h"

class DependWorker : public Worker
{
public:
    virtual ~DependWorker() override;
    virtual bool init(int& argc, char**& argv) override;
    virtual bool check() override;
    virtual bool run() override;

private:
    void findPackages(std::vector<std::string>& package_list);
    void sortPackages(std::vector<std::string>& package_list);
    void printPackages(std::vector<std::string>& package_list);

private:
    DependWorkerParam param_;
};
