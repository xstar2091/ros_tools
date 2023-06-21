#pragma once

#include <string>

class CmdkeyInfo
{
public:
    CmdkeyInfo();
    CmdkeyInfo(const void* ptr, const std::string& name, const std::string& type,
               const std::string& description, const std::string& default_value);
    CmdkeyInfo(const CmdkeyInfo& other);
    CmdkeyInfo& operator=(const CmdkeyInfo& other);

public:
    void copy(const CmdkeyInfo& other);
    bool exist(const std::string& name);
    bool find(const std::string& name);
    void reset();

public:
    bool valid;
    bool is_default;
    const void* ptr;
    std::string name;
    std::string type;
    std::string description;
    std::string current_value;
    std::string default_value;
};
