#include "ros_graph/worker/worker.h"
#include <cstring>
#include <iostream>
#include "ros_graph/worker/depend/depend_worker.h"
#include "ros_graph/worker/dependby/dependby_worker.h"
#include "ros_graph/worker/dependtree/dependtree_worker.h"
#include "ros_graph/worker/dependtreeby/dependtreeby_worker.h"

Worker::~Worker()
{}

Worker* Worker::create(const char *command)
{
    Worker* worker = nullptr;
    if (strcmp(command, "depend") == 0)
    {
        worker = new DependWorker;
    }
    else if (strcmp(command, "dependby") == 0)
    {
        worker = new DependByWorker;
    }
    else if (strcmp(command, "dependtree") == 0)
    {
        worker = new DependTreeWorker;
    }
    else if (strcmp(command, "dependtreeby") == 0)
    {
        worker = new DependTreeByWorker;
    }

    if (worker == nullptr)
    {
        std::cerr<<"invalid command: "<<command<<std::endl;
    }
    return worker;
}
