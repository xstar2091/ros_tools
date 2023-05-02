#include "ros_graph/worker/worker.h"
#include "ros_graph/worker/depend/depend_worker.h"
#include "ros_graph/worker/dependby/dependby_worker.h"
#include "ros_graph/worker/dependtree/dependtree_worker.h"
#include <fmt/format.h>

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

    if (worker == nullptr)
    {
        fmt::print(stderr, "invalid command: {}\n", command);
    }
    return worker;
}
