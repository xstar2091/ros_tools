#pragma once

#include <cstdint>
#include <string>

class Cmdkey
{
public:
    Cmdkey(bool* ptr, bool default_value, const char* name, const char* desc);
    Cmdkey(int32_t* ptr, int32_t default_value, const char* name, const char* desc);
    Cmdkey(uint32_t* ptr, uint32_t default_value, const char* name, const char* desc);
    Cmdkey(int64_t* ptr, int64_t default_value, const char* name, const char* desc);
    Cmdkey(uint64_t* ptr, uint64_t default_value, const char* name, const char* desc);
    Cmdkey(float* ptr, float default_value, const char* name, const char* desc);
    Cmdkey(double* ptr, double default_value, const char* name, const char* desc);
    Cmdkey(std::string* ptr, std::string default_value, const char* name, const char* desc);

public:
    static void parseCommandLine(int& argc, char** argv, bool remove_key = true);
    static void parseFile(const char* file_name);
    static void reset();

private:
    static bool check(const char* kv, int index, void* index_list_ptr);
    static void parseKV(const char* head, std::string& k, std::string& v);
    static void setKV(const std::string& key, const std::string& val);
};

#define define_cmdkey_bool(name, val, desc) \
    bool cmdkey_##name = val; \
    static Cmdkey o_my_cmdkey_##name(&cmdkey_##name, val, #name, desc);

#define define_cmdkey_int32(name, val, desc) \
    int32_t cmdkey_##name = val; \
    static Cmdkey o_my_cmdkey_##name(&cmdkey_##name, val, #name, desc);

#define define_cmdkey_uint32(name, val, desc) \
    uint32_t cmdkey_##name = val; \
    static Cmdkey o_my_cmdkey_##name(&cmdkey_##name, val, #name, desc);

#define define_cmdkey_int64(name, val, desc) \
    int64_t cmdkey_##name = val; \
    static Cmdkey o_my_cmdkey_##name(&cmdkey_##name, val, #name, desc);

#define define_cmdkey_uint64(name, val, desc) \
    uint64_t cmdkey_##name = val; \
    static Cmdkey o_my_cmdkey_##name(&cmdkey_##name, val, #name, desc);

#define define_cmdkey_float(name, val, desc) \
    float cmdkey_##name = val; \
    static Cmdkey o_my_cmdkey_##name(&cmdkey_##name, val, #name, desc);

#define define_cmdkey_double(name, val, desc) \
    double cmdkey_##name = val; \
    static Cmdkey o_my_cmdkey_##name(&cmdkey_##name, val, #name, desc);

#define define_cmdkey_string(name, val, desc) \
    std::string cmdkey_##name = val; \
    static Cmdkey o_my_cmdkey_##name(&cmdkey_##name, val, #name, desc);
