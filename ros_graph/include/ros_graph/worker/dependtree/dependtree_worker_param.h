#pragma once

#include <string>

class CommandLineParam;

class DependTreeWorkerParam
{
public:
    bool reset(const CommandLineParam& param);

public:
    // 每行的缩进数，缩进字符为separator
    int indent;
    int level;
    std::string package;
    std::string workspace_dir;
    std::string separator;

private:
    bool check();
};
