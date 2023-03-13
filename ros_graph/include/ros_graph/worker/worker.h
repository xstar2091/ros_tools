#pragma once

#include "ros_graph/command_line_param.h"
#include "ros_graph/depend_graph.h"

class Worker
{
public:
    virtual ~Worker();
    virtual bool init(int& argc, char**& argv) = 0;
    virtual bool check() = 0;
    virtual bool run() = 0;

public:
    static Worker* create(const char* command);

protected:
    CommandLineParam command_line_param_;
    DependGraph depend_graph_;
};
