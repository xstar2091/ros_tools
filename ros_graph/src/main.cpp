#include <cstring>
#include <memory>
#include <fmt/format.h>
#include "ros_graph/worker/worker.h"
bool showHelpInfo(int argc, char** argv);

int main(int argc, char* argv[])
{
    int exit_code = 1;
    do
    {
        if (showHelpInfo(argc, argv))
        {
            exit_code = 0;
            break;
        }

        std::unique_ptr<Worker> worker(Worker::create(argv[1]));
        if (worker.get() == nullptr) break;
        if (!worker->init(argc, argv)) break;
        if (!worker->check()) break;
        if (!worker->run()) break;
        exit_code = 0;
    } while (false);
    return exit_code;
}

const char* help_info = R"startstring(usage:
{} <command> [options]
command list:
depend: 显示指定包的依赖
)startstring";

bool showHelpInfo(int argc, char** argv)
{
    if (argc == 1)
    {
        fmt::print(help_info, argv[0]);
        return true;
    }
    if (argc == 2)
    {
        if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)
        {
            fmt::print(help_info, argv[0]);
            return true;
        }
    }
    return false;
}
