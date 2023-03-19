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

version: 0.1.0

command list:
depend:   显示指定包的依赖
dependby: 显示指定包被哪些包依赖

公共选项
    --workspace_dir catkin workspace 目录

命令选项
depend
    --package 指定的包(不可指定空字符串，不可指定多个包)
    --separator 输出包的分隔符
dependby
    --level   分析深度
    --package 指定的包(不可指定空字符串，不可指定多个包)
    --separator 输出包的分隔符
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
