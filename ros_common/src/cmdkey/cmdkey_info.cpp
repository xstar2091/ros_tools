#include "ros_common/cmdkey/cmdkey_info.h"

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include "ros_common/cmdkey/cmdkey_table.h"

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
    this->current_value = other.current_value;
    this->default_value = other.default_value;
}

bool CmdkeyInfo::exist(const std::string& name)
{
    auto it = CmdkeyTable::inst()->table.find(name);
    return it != CmdkeyTable::inst()->table.end();
}

bool CmdkeyInfo::find(const std::string& name)
{
    valid = false;
    auto it = CmdkeyTable::inst()->table.find(name);
    if (it != CmdkeyTable::inst()->table.end())
    {
        copy(it->second);
    }
    return valid;
}

void CmdkeyInfo::reset()
{
    size_t pos{};
    is_default = true;
    current_value = default_value;
    if (type == "bool")
    {
        if (default_value == "true" || default_value == "1")
        {
            *(bool*)ptr = true;
        }
        else
        {
            *(bool*)ptr = false;
        }
    }
    else if (type == "int32_t")
    {
        long long int num = std::stoll(default_value, &pos, 10);
        if (num < std::numeric_limits<int32_t>::min() || num > std::numeric_limits<int32_t>::max())
        {
            throw std::out_of_range("parse number out of range");
        }
        *(int32_t*)ptr = static_cast<int32_t>(num);
    }
    else if (type == "uint32_t")
    {
        long long int num = std::stoll(default_value, &pos, 10);
        if (num < std::numeric_limits<uint32_t>::min() || num > std::numeric_limits<uint32_t>::max())
        {
            throw std::out_of_range("parse number out of range");
        }
        *(uint32_t*)ptr = static_cast<uint32_t>(num);
    }
    else if (type == "int64_t")
    {
        long long int num = std::stoll(default_value, &pos, 10);
        *(int64_t*)ptr = static_cast<int64_t>(num);
    }
    else if (type == "uint64_t")
    {
        long long int num = std::stoll(default_value, &pos, 10);
        *(uint64_t*)ptr = static_cast<uint64_t>(num);
    }
    else if (type == "float")
    {
        *(float*)ptr = std::stof(default_value, &pos);
    }
    else if (type == "double")
    {
        *(double*)ptr = std::stod(default_value, &pos);
    }
    else if (type == "string")
    {
        *(std::string*)ptr = default_value;
    }
}
