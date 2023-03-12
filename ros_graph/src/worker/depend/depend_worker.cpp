#include "ros_graph/worker/depend/depend_worker.h"

DependWorker::~DependWorker()
{}

bool DependWorker::init(int& argc, char** argv)
{
    if (!command_line_param_.init(argc, argv)) return false;
    return true;
}

bool DependWorker::run()
{
    return false;
}
