#include <vector>
#include <gtest/gtest.h>

#include "ros_common/cmdkey/cmdkey_info.h"
#include "ros_common/cmdkey/cmdkey.h"

void makeArgv(const std::vector<std::string>& vec, int& argc, char**& argv)
{
    argc = static_cast<int>(vec.size());
    argv = new char*[argc + 1];
    argv[argc] = nullptr;
    for (int i = 0; i < argc; i++)
    {
        char* str = new char[vec[i].size() + 1];
        str[vec[i].size()] = '\0';
        memcpy(str, vec[i].c_str(), vec[i].size());
        argv[i] = str;
    }
}

void destroyArgv(int& argc, char**& argv)
{
    for (int i = 0; i < argc; i++)
    {
        delete[] argv[i];
    }
    delete[] argv;
}

#define VAR_BOOL_DEFAULT_VAL true
#define VAR_INT32_DEFAULT_VAL 7
#define VAR_UINT32_DEFAULT_VAL 107
#define VAL_INT64_DEFAULT_VAL 9
#define VAL_UINT64_DEFAULT_VAL 109
#define VAL_FLOAT_DEFAULT_VAL 2.3
#define VAL_DOUBLE_DEFAULT_VAL 5.7
#define VAL_STRING_DEFAULT_VAL std::string("string var default val")

define_cmdkey_bool(var_bool, VAR_BOOL_DEFAULT_VAL, "");
define_cmdkey_int32(var_int32, VAR_INT32_DEFAULT_VAL, "");
define_cmdkey_uint32(var_uint32, VAR_UINT32_DEFAULT_VAL, "");
define_cmdkey_int64(var_int64, VAL_INT64_DEFAULT_VAL, "");
define_cmdkey_uint64(var_uint64, VAL_UINT64_DEFAULT_VAL, "");
define_cmdkey_float(var_float, VAL_FLOAT_DEFAULT_VAL, "");
define_cmdkey_double(var_double, VAL_DOUBLE_DEFAULT_VAL, "");
define_cmdkey_string(var_string, VAL_STRING_DEFAULT_VAL, "");

TEST(Cmdkey, Bool)
{
    int argc = 0;
    char** argv = nullptr;

    CmdkeyInfo info;
    info.find("var_bool");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(cmdkey_var_bool, VAR_BOOL_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_BOOL_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_BOOL_DEFAULT_VAL));

    makeArgv({"unittest", "--var_bool=true"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_bool");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_bool, true);
    EXPECT_EQ(info.current_value, std::string("1"));
    EXPECT_EQ(info.default_value, std::to_string(VAR_BOOL_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    info.find("var_bool");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(cmdkey_var_bool, VAR_BOOL_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_BOOL_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_BOOL_DEFAULT_VAL));

    makeArgv({"unittest", "--var_bool=false"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_bool");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_bool, false);
    EXPECT_EQ(info.current_value, std::string("0"));
    EXPECT_EQ(info.default_value, std::to_string(VAR_BOOL_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    makeArgv({"unittest", "--var_bool=1"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_bool");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_bool, true);
    EXPECT_EQ(info.current_value, std::string("1"));
    EXPECT_EQ(info.default_value, std::to_string(VAR_BOOL_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    makeArgv({"unittest", "--var_bool=10"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_bool");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_bool, false);
    EXPECT_EQ(info.current_value, std::string("0"));
    EXPECT_EQ(info.default_value, std::to_string(VAR_BOOL_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    makeArgv({"unittest", "--var_bool=-10"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_bool");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_bool, false);
    EXPECT_EQ(info.current_value, std::string("0"));
    EXPECT_EQ(info.default_value, std::to_string(VAR_BOOL_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    makeArgv({"unittest", "--var_bool=0"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_bool");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_bool, false);
    EXPECT_EQ(info.current_value, std::string("0"));
    EXPECT_EQ(info.default_value, std::to_string(VAR_BOOL_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    makeArgv({"unittest", "--var_bool=abc"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_bool");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_bool, false);
    EXPECT_EQ(info.current_value, std::string("0"));
    EXPECT_EQ(info.default_value, std::to_string(VAR_BOOL_DEFAULT_VAL));
    destroyArgv(argc, argv);
}

TEST(Cmdkey, Int32)
{
    int argc = 0;
    char** argv = nullptr;

    CmdkeyInfo info;
    info.find("var_int32");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(cmdkey_var_int32, VAR_INT32_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_INT32_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_INT32_DEFAULT_VAL));

    makeArgv({"unittest", "--var_int32=32"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_int32");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_int32, 32);
    EXPECT_EQ(info.current_value, std::string("32"));
    EXPECT_EQ(info.default_value, std::to_string(VAR_INT32_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    info.find("var_int32");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(cmdkey_var_int32, VAR_INT32_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_INT32_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_INT32_DEFAULT_VAL));

    makeArgv({"unittest", "--var_int32=-32"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_int32");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_int32, -32);
    EXPECT_EQ(info.current_value, std::string("-32"));
    EXPECT_EQ(info.default_value, std::to_string(VAR_INT32_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    makeArgv({"unittest", "--var_int32=12345678912"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
}

TEST(Cmdkey, Uint32)
{
    int argc = 0;
    char** argv = nullptr;

    CmdkeyInfo info;
    info.find("var_uint32");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(cmdkey_var_uint32, VAR_UINT32_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_UINT32_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_UINT32_DEFAULT_VAL));

    makeArgv({"unittest", "--var_uint32=32"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_uint32");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_uint32, 32);
    EXPECT_EQ(info.current_value, std::string("32"));
    EXPECT_EQ(info.default_value, std::to_string(VAR_UINT32_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    info.find("var_uint32");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(cmdkey_var_uint32, VAR_UINT32_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_UINT32_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_UINT32_DEFAULT_VAL));

    Cmdkey::reset();
    makeArgv({"unittest", "--var_uint32=-32"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);

    Cmdkey::reset();
    makeArgv({"unittest", "--var_uint32=12345678912"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
}
