#pragma once

#include <string>
#include <unordered_map>
#include "ros_common/cmdkey/cmdkey_info.h"

class CmdkeyTable
{
public:
    static CmdkeyTable* inst();

public:
    std::unordered_map<std::string, CmdkeyInfo> table;
};
