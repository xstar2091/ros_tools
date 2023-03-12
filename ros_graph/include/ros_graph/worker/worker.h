#pragma once

#include "ros_graph/command_line_param.h"

class Worker
{
public:
    virtual ~Worker();
    virtual bool init(int& argc, char**& argv) = 0;
    virtual bool run() = 0;
    virtual void printDebugInfo();

public:
    static Worker* create(const char* command);

protected:
    CommandLineParam command_line_param_;
};
