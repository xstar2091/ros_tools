#include <cstring>
#include <fmt/format.h>
#include "worker/worker.h"

bool showHelpInfo(int argc, char* argv[]);

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
        exit_code = 0;
    } while (false);
    return exit_code;
}

const char* help_info = R"startstring(usage:
{} <command> [options]
command list:
depend: 显示指定包的依赖
)startstring";

bool showHelpInfo(int argc, char* argv[])
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
