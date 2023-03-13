#pragma once

#include <string>

class CommandLineParam;

class DependWorkerParam
{
public:
    bool reset(const CommandLineParam& param);

public:
    std::string package;
    std::string workspace_dir;
    std::string separator;

private:
    bool check();
};
