#pragma once

#include <cstdint>
#include <string>

class Cmdkey
{
public:
    void addKey(bool& ptr, bool default_value, const char* name, const char* desc);
    void addKey(int8_t& ptr, int8_t default_value, const char* name, const char* desc);
    void addKey(uint8_t& ptr, uint8_t default_value, const char* name, const char* desc);
    void addKey(int16_t& ptr, int16_t default_value, const char* name, const char* desc);
    void addKey(uint16_t& ptr, uint16_t default_value, const char* name, const char* desc);
    void addKey(int32_t& ptr, int32_t default_value, const char* name, const char* desc);
    void addKey(uint32_t& ptr, uint32_t default_value, const char* name, const char* desc);
    void addKey(int64_t& ptr, int64_t default_value, const char* name, const char* desc);
    void addKey(uint64_t& ptr, uint64_t default_value, const char* name, const char* desc);
    void addKey(float& ptr, float default_value, const char* name, const char* desc);
    void addKey(double& ptr, double default_value, const char* name, const char* desc);

public:
    void parseCommandLine(int& argc, char** argv, bool remove_key = true);
    void parseFile(const char* file_name);
    void reset();

private:
    bool check(char* kv, void* index_list_ptr, int index);
    void parseKV(char* head, std::string& k, std::string& v);
    void setKV(const std::string& key, const std::string& val);
};
