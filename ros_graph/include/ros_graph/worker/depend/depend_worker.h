#pragma once

#include "ros_graph/worker/worker.h"

class DependWorker : public Worker
{
public:
    virtual ~DependWorker() override;
    virtual bool init(int& argc, char**& argv) override;
    virtual bool run() override;
    virtual void printDebugInfo() override;
};
