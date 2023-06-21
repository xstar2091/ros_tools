#include "ros_common/cmdkey/cmdkey.h"

#include <cstring>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include "ros_common/cmdkey/cmdkey_info.h"

#define CMDKEY_MIN_LEN 5

std::unordered_map<std::string, CmdkeyInfo> ros_common_internal_cmdkey_info_map;

void addCommandLineKey(const void* ptr, const std::string& name, const std::string& type,
                       const std::string& default_value, const std::string& desc)
{
    auto it = ros_common_internal_cmdkey_info_map.find(name);
    if (it != ros_common_internal_cmdkey_info_map.end())
    {
        std::string exception_message("invalid command line key, already exist, name:");
        exception_message.append(name).append(", current default value:")
                         .append(default_value).append(", current description:")
                         .append(desc).append(", previous default value:")
                         .append(it->second.default_value).append(", previous description:")
                         .append(it->second.description);
        throw std::invalid_argument(exception_message);
    }
    ros_common_internal_cmdkey_info_map.insert(std::make_pair(name, CmdkeyInfo(ptr, name, type, desc, default_value)));
}

void Cmdkey::addKey(bool& ptr, bool default_value, const char* name, const char* desc)
{
    addCommandLineKey(&ptr, std::string(name), std::string("bool"), std::to_string(default_value), std::string(desc));
}

void Cmdkey::addKey(int8_t& ptr, int8_t default_value, const char* name, const char* desc)
{
    addCommandLineKey(&ptr, std::string(name), std::string("int8_t"), std::to_string(default_value), std::string(desc));
}

void Cmdkey::addKey(uint8_t& ptr, uint8_t default_value, const char* name, const char* desc)
{
    addCommandLineKey(&ptr, std::string(name), std::string("uint8_t"), std::to_string(default_value), std::string(desc));
}

void Cmdkey::addKey(int16_t& ptr, int16_t default_value, const char* name, const char* desc)
{
    addCommandLineKey(&ptr, std::string(name), std::string("int16_t"), std::to_string(default_value), std::string(desc));
}

void Cmdkey::addKey(uint16_t& ptr, uint16_t default_value, const char* name, const char* desc)
{
    addCommandLineKey(&ptr, std::string(name), std::string("uint16_t"), std::to_string(default_value), std::string(desc));
}

void Cmdkey::addKey(int32_t& ptr, int32_t default_value, const char* name, const char* desc)
{
    addCommandLineKey(&ptr, std::string(name), std::string("int32_t"), std::to_string(default_value), std::string(desc));
}

void Cmdkey::addKey(uint32_t& ptr, uint32_t default_value, const char* name, const char* desc)
{
    addCommandLineKey(&ptr, std::string(name), std::string("uint32_t"), std::to_string(default_value), std::string(desc));
}

void Cmdkey::addKey(int64_t& ptr, int64_t default_value, const char* name, const char* desc)
{
    addCommandLineKey(&ptr, std::string(name), std::string("int64_t"), std::to_string(default_value), std::string(desc));
}

void Cmdkey::addKey(uint64_t& ptr, uint64_t default_value, const char* name, const char* desc)
{
    addCommandLineKey(&ptr, std::string(name), std::string("uint64_t"), std::to_string(default_value), std::string(desc));
}

void Cmdkey::addKey(float& ptr, float default_value, const char* name, const char* desc)
{
    addCommandLineKey(&ptr, std::string(name), std::string("float"), std::to_string(default_value), std::string(desc));
}

void Cmdkey::addKey(double& ptr, double default_value, const char* name, const char* desc)
{
    addCommandLineKey(&ptr, std::string(name), std::string("double"), std::to_string(default_value), std::string(desc));
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
        if (!check(kv, &index_list, ind))
        {
            continue;
        }
        parseKV(kv, key, val);
        setKV(key, val);
    }
}

void Cmdkey::parseFile(const char* file_name)
{}

void Cmdkey::reset()
{
    for (auto& pair : ros_common_internal_cmdkey_info_map)
    {
        pair.second.reset();
    }
}

bool Cmdkey::check(char* kv, void* index_list_ptr, int index)
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
        if (count != len)
        {
            index_list->push_back(index);
        }
        return false;
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
        for (size_t i = 2; i < len; i++)
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

void Cmdkey::parseKV(char* head, std::string& k, std::string& v)
{
    int left = strlen(head);
    char* h = head;
    char* k_ptr_start = nullptr;
    char* k_ptr_end = nullptr;
    char* v_ptr_start = nullptr;
    char* v_ptr_end = nullptr;
    while (*h == '-') ++h;
    k_ptr_start = h;
    while (*h != '=') ++h;
}

void Cmdkey::setKV(const std::string& key, const std::string& val)
{}
