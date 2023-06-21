#include "ros_common/cmdkey/cmdkey_info.h"

#include <unordered_map>

extern std::unordered_map<std::string, CmdkeyInfo> ros_common_internal_cmdkey_info_map;

CmdkeyInfo::CmdkeyInfo()
{
    valid = false;
    is_default = true;
    ptr = nullptr;
    name = "";
    type = "";
    description = "";
    current_value = "";
    default_value = "";
}

CmdkeyInfo::CmdkeyInfo(const void* ptr, const std::string& name, const std::string& type,
                       const std::string& description, const std::string& default_value)
{
    valid = true;
    is_default = true;
    this->ptr = ptr;
    this->name = name;
    this->type = type;
    this->description = description;
    this->current_value = default_value;
    this->default_value = default_value;
}

CmdkeyInfo::CmdkeyInfo(const CmdkeyInfo& other)
{
    copy(other);
}

CmdkeyInfo& CmdkeyInfo::operator=(const CmdkeyInfo& other)
{
    copy(other);
    return *this;
}

void CmdkeyInfo::copy(const CmdkeyInfo& other)
{
    valid = other.valid;
    is_default = other.is_default;
    this->ptr = other.ptr;
    this->name = other.name;
    this->type = other.type;
    this->description = other.description;
    this->current_value = other.default_value;
    this->default_value = other.default_value;
}

bool CmdkeyInfo::exist(const std::string& name)
{
    auto it = ros_common_internal_cmdkey_info_map.find(name);
    return it != ros_common_internal_cmdkey_info_map.end();
}

bool CmdkeyInfo::find(const std::string& name)
{
    valid = false;
    auto it = ros_common_internal_cmdkey_info_map.find(name);
    if (it != ros_common_internal_cmdkey_info_map.end())
    {
        copy(it->second);
    }
    return valid;
}

void CmdkeyInfo::reset()
{
    is_default = true;
    current_value = default_value;
}
