#include "ros_common/cmdkey/cmdkey.h"

#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include "ros_common/cmdkey/cmdkey_info.h"
#include "ros_common/cmdkey/cmdkey_table.h"

#define CMDKEY_MIN_LEN 5

void addCommandLineKey(const void* ptr, const std::string& name, const std::string& type,
                       const std::string& default_value, const std::string& desc)
{
    auto it = CmdkeyTable::inst()->table.find(name);
    if (it != CmdkeyTable::inst()->table.end())
    {
        std::string exception_message("invalid command line key, already exist, name:");
        exception_message.append(name).append(", current default value:")
                         .append(default_value).append(", current description:")
                         .append(desc).append(", previous default value:")
                         .append(it->second.default_value).append(", previous description:")
                         .append(it->second.description);
        throw std::invalid_argument(exception_message);
    }
    CmdkeyTable::inst()->table.insert(std::make_pair(name, CmdkeyInfo(ptr, name, type, desc, default_value)));
}

Cmdkey::Cmdkey(bool* ptr, bool default_value, const char* name, const char* desc)
{
    addCommandLineKey(ptr, std::string(name), std::string("bool"), std::to_string(default_value), std::string(desc));
}

Cmdkey::Cmdkey(int32_t* ptr, int32_t default_value, const char* name, const char* desc)
{
    addCommandLineKey(ptr, std::string(name), std::string("int32_t"), std::to_string(default_value), std::string(desc));
}

Cmdkey::Cmdkey(uint32_t* ptr, uint32_t default_value, const char* name, const char* desc)
{
    addCommandLineKey(ptr, std::string(name), std::string("uint32_t"), std::to_string(default_value), std::string(desc));
}

Cmdkey::Cmdkey(int64_t* ptr, int64_t default_value, const char* name, const char* desc)
{
    addCommandLineKey(ptr, std::string(name), std::string("int64_t"), std::to_string(default_value), std::string(desc));
}

Cmdkey::Cmdkey(uint64_t* ptr, uint64_t default_value, const char* name, const char* desc)
{
    addCommandLineKey(ptr, std::string(name), std::string("uint64_t"), std::to_string(default_value), std::string(desc));
}

Cmdkey::Cmdkey(float* ptr, float default_value, const char* name, const char* desc)
{
    addCommandLineKey(ptr, std::string(name), std::string("float"), std::to_string(default_value), std::string(desc));
}

Cmdkey::Cmdkey(double* ptr, double default_value, const char* name, const char* desc)
{
    addCommandLineKey(ptr, std::string(name), std::string("double"), std::to_string(default_value), std::string(desc));
}

Cmdkey::Cmdkey(std::string* ptr, std::string default_value, const char* name, const char* desc)
{
    addCommandLineKey(ptr, std::string(name), std::string("string"), default_value, std::string(desc));
}

void Cmdkey::parseCommandLine(int& argc, char** argv, bool remove_key)
{
    std::string key;
    std::string val;
    char* kv = nullptr;
    std::vector<int> index_list;
    for (int ind = 1; ind < argc; ind++)
    {
        kv = argv[ind];
        if (!check(kv, ind, &index_list))
        {
            continue;
        }
        parseKV(kv, key, val);
        setKV(key, val);
    }
    if (!remove_key) return;
    for (int i = 1, j = 0; j < static_cast<int>(index_list.size()); i++, j++)
    {
        if (i == index_list[j]) continue;
        std::swap(argv[i], argv[index_list[j]]);
    }
}

void Cmdkey::parseFile(const char* file_name)
{
    std::ifstream ifs(file_name);
    if (!ifs)
    {
        std::string exception_message("invalid command line key, open file failed:");
        exception_message.append(file_name);
        throw std::invalid_argument(exception_message);
    }
    std::string key;
    std::string val;
    std::string line;
    int index = 0;
    std::vector<int> index_list;
    while (std::getline(ifs, line))
    {
        if (!check(line.c_str(), index, &index_list))
        {
            continue;
        }
        parseKV(line.c_str(), key, val);
        setKV(key, val);
    }
}

void Cmdkey::reset()
{
    for (auto& pair : CmdkeyTable::inst()->table)
    {
        pair.second.reset();
    }
}

bool Cmdkey::check(const char* kv, int index, void* index_list_ptr)
{
    std::vector<int>* index_list = static_cast<std::vector<int>*>(index_list_ptr);
    size_t len = strlen(kv);
    if (kv == nullptr)
    {
        return false;
    }
    {
        size_t count = 0;
        for (size_t i = 0; i < len; i++)
        {
            if (kv[i] == '-') count++;
        }
        if (count == len)
        {
            return false;
        }
    }
    if (len < CMDKEY_MIN_LEN)
    {
        index_list->push_back(index);
        return false;
    }
    if (kv[0] != '-' || kv[1] != '-')
    {
        index_list->push_back(index);
        return false;
    }
    if (kv[2] == '-')
    {
        index_list->push_back(index);
        return false;
    }
    {
        for (size_t i = 2; i < len - 1; i++)
        {
            if (kv[i] == '=')
            {
                return true;
            }
        }
    }
    index_list->push_back(index);
    return false;
}

void Cmdkey::parseKV(const char* head, std::string& k, std::string& v)
{
    const char* h = head;
    const char* k_ptr_start = nullptr;
    const char* k_ptr_end = nullptr;
    const char* v_ptr_start = nullptr;
    const char* v_ptr_end = nullptr;
    while (*h == '-') ++h;
    k_ptr_start = h;
    while (*h != '=') ++h;
    k_ptr_end = h;
    ++h;
    v_ptr_start = h;
    v_ptr_end = head + strlen(head);
    k.assign(k_ptr_start, k_ptr_end);
    v.assign(v_ptr_start, v_ptr_end);
}

void Cmdkey::setKV(const std::string& key, const std::string& val)
{
    auto it = CmdkeyTable::inst()->table.find(key);
    if (it == CmdkeyTable::inst()->table.end())
    {
        std::string exception_message("unknown command line key:");
        exception_message.append(key);
        throw std::invalid_argument(exception_message);
    }
    CmdkeyInfo& info = it->second;
    info.is_default = false;
    size_t pos{};
    try
    {
        if (info.type == "bool")
        {
            bool* ptr = (bool*)info.ptr;
            if (val == "true" || val == "1")
            {
                *ptr = true;
            }
            else
            {
                *ptr = false;
            }
            info.current_value = std::to_string(*ptr);
        }
        else if (info.type == "int32_t")
        {
            long long int num = std::stoll(val, &pos, 10);
            if (num < std::numeric_limits<int32_t>::min() || num > std::numeric_limits<int32_t>::max())
            {
                throw std::out_of_range("parse number out of range");
            }
            int32_t* ptr = (int32_t*)info.ptr;
            *ptr = static_cast<int32_t>(num);
            info.current_value = std::to_string(*ptr);
        }
        else if (info.type == "uint32_t")
        {
            long long int num = std::stoll(val, &pos, 10);
            if (num < std::numeric_limits<uint32_t>::min() || num > std::numeric_limits<uint32_t>::max())
            {
                throw std::out_of_range("parse number out of range");
            }
            uint32_t* ptr = (uint32_t*)info.ptr;
            *ptr = static_cast<uint32_t>(num);
            info.current_value = std::to_string(*ptr);
        }
        else if (info.type == "int64_t")
        {
            long long int num = std::stoll(val, &pos, 10);
            int64_t* ptr = (int64_t*)info.ptr;
            *ptr = static_cast<int64_t>(num);
            info.current_value = std::to_string(*ptr);
        }
        else if (info.type == "uint64_t")
        {
            long long int num = std::stoll(val, &pos, 10);
            uint64_t* ptr = (uint64_t*)info.ptr;
            *ptr = static_cast<uint64_t>(num);
            info.current_value = std::to_string(*ptr);
        }
        else if (info.type == "float")
        {
            float* ptr = (float*)info.ptr;
            *ptr = std::stof(val, &pos);
            info.current_value = std::to_string(*ptr);
        }
        else if (info.type == "double")
        {
            double* ptr = (double*)info.ptr;
            *ptr = std::stod(val, &pos);
            info.current_value = std::to_string(*ptr);
        }
        else if (info.type == "string")
        {
            std::string* ptr = (std::string*)info.ptr;
            *ptr = val;
            info.current_value = *ptr;
        }
    }
    catch (const std::invalid_argument& err)
    {
        std::string exception_message("invalid argument, key:");
        exception_message.append(key).append(", val:").append(val).append(", parse to ")
                         .append(info.type).append(" failed, error message:").append(err.what());
        throw std::invalid_argument(exception_message);
    }
    catch (const std::out_of_range& err)
    {
        std::string exception_message("invalid argument, key:");
        exception_message.append(key).append(", val:").append(val).append(", parse to ")
                         .append(info.type).append(" out of range, error message:").append(err.what());
        throw std::invalid_argument(exception_message);
    }
}
