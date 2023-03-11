#include "ros_graph/worker/worker.h"
#include <fmt/format.h>

Worker::~Worker()
{}

Worker* Worker::create(const char *command)
{
    fmt::print("aabbcc");
    return nullptr;
}
