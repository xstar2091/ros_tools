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
#define VAR_INT64_DEFAULT_VAL 9
#define VAR_UINT64_DEFAULT_VAL 109
#define VAR_FLOAT_DEFAULT_VAL 2.3
#define VAR_DOUBLE_DEFAULT_VAL 5.7
#define VAR_STRING_DEFAULT_VAL std::string("string var default val")

define_cmdkey_bool(var_bool, VAR_BOOL_DEFAULT_VAL, "");
define_cmdkey_int32(var_int32, VAR_INT32_DEFAULT_VAL, "");
define_cmdkey_uint32(var_uint32, VAR_UINT32_DEFAULT_VAL, "");
define_cmdkey_int64(var_int64, VAR_INT64_DEFAULT_VAL, "");
define_cmdkey_uint64(var_uint64, VAR_UINT64_DEFAULT_VAL, "");
define_cmdkey_float(var_float, VAR_FLOAT_DEFAULT_VAL, "");
define_cmdkey_double(var_double, VAR_DOUBLE_DEFAULT_VAL, "");
define_cmdkey_string(var_string, VAR_STRING_DEFAULT_VAL, "");

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
    Cmdkey::reset();
    makeArgv({"unittest", "--var_int32=abc"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_int32=abc32"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_int32=32abc"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    EXPECT_EQ(cmdkey_var_int32, 32);
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
    Cmdkey::reset();
    makeArgv({"unittest", "--var_uint32=abc"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_uint32=abc32"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_uint32=32abc"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    EXPECT_EQ(cmdkey_var_uint32, 32);
    destroyArgv(argc, argv);
}

TEST(Cmdkey, Int64)
{
    int argc = 0;
    char** argv = nullptr;

    CmdkeyInfo info;
    info.find("var_int64");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(cmdkey_var_int64, VAR_INT64_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_INT64_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_INT64_DEFAULT_VAL));

    makeArgv({"unittest", "--var_int64=64"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_int64");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_int64, 64);
    EXPECT_EQ(info.current_value, std::string("64"));
    EXPECT_EQ(info.default_value, std::to_string(VAR_INT64_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    info.find("var_int64");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(cmdkey_var_int64, VAR_INT64_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_INT64_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_INT64_DEFAULT_VAL));

    makeArgv({"unittest", "--var_int64=-64"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_int64");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_int64, -64);
    EXPECT_EQ(info.current_value, std::string("-64"));
    EXPECT_EQ(info.default_value, std::to_string(VAR_INT64_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    makeArgv({"unittest", "--var_int64=12345678912123456789012345678901234567890"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_int64=abc"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_int64=abc64"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_int64=64abc"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    EXPECT_EQ(cmdkey_var_int64, 64);
    destroyArgv(argc, argv);
}

TEST(Cmdkey, Uint64)
{
    int argc = 0;
    char** argv = nullptr;

    CmdkeyInfo info;
    info.find("var_uint64");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(cmdkey_var_uint64, VAR_UINT64_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_UINT64_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_UINT64_DEFAULT_VAL));

    makeArgv({"unittest", "--var_uint64=64"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_uint64");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_uint64, 64);
    EXPECT_EQ(info.current_value, std::string("64"));
    EXPECT_EQ(info.default_value, std::to_string(VAR_UINT64_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    info.find("var_uint64");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(cmdkey_var_uint64, VAR_UINT64_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_UINT64_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_UINT64_DEFAULT_VAL));

    Cmdkey::reset();
    makeArgv({"unittest", "--var_uint64=-64"}, argc, argv);
    EXPECT_NO_THROW(Cmdkey::parseCommandLine(argc, argv));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    makeArgv({"unittest", "--var_uint64=1234567891212345678901234567890"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_uint64=abc"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_uint64=abc64"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_uint64=64abc"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    EXPECT_EQ(cmdkey_var_uint64, 64);
    destroyArgv(argc, argv);
}

TEST(Cmdkey, Float)
{
    int argc = 0;
    char** argv = nullptr;

    CmdkeyInfo info;
    info.find("var_float");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_FLOAT_EQ(cmdkey_var_float, VAR_FLOAT_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_FLOAT_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_FLOAT_DEFAULT_VAL));

    makeArgv({"unittest", "--var_float=32.32"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_float");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_FLOAT_EQ(cmdkey_var_float, 32.32);
    EXPECT_EQ(info.current_value.find(std::string("32.32")), 0);
    EXPECT_EQ(info.default_value, std::to_string(VAR_FLOAT_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    info.find("var_float");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_FLOAT_EQ(cmdkey_var_float, VAR_FLOAT_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_FLOAT_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_FLOAT_DEFAULT_VAL));

    makeArgv({"unittest", "--var_float=-32.32"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_float");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_FLOAT_EQ(cmdkey_var_float, -32.32);
    EXPECT_EQ(info.current_value.find(std::string("-32.32")), 0);
    EXPECT_EQ(info.default_value.find(std::to_string(VAR_FLOAT_DEFAULT_VAL)), 0);
    destroyArgv(argc, argv);

    Cmdkey::reset();
    makeArgv({"unittest", "--var_float=12345678912"}, argc, argv);
    EXPECT_NO_THROW(Cmdkey::parseCommandLine(argc, argv));
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_float=abc"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_float=abc32.32"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_float=32.32abc"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    EXPECT_FLOAT_EQ(cmdkey_var_float, 32.32);
    destroyArgv(argc, argv);
}

TEST(Cmdkey, Double)
{
    int argc = 0;
    char** argv = nullptr;

    CmdkeyInfo info;
    info.find("var_double");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_DOUBLE_EQ(cmdkey_var_double, VAR_DOUBLE_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_DOUBLE_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_DOUBLE_DEFAULT_VAL));

    makeArgv({"unittest", "--var_double=64.64"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_double");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_DOUBLE_EQ(cmdkey_var_double, 64.64);
    EXPECT_EQ(info.current_value.find(std::string("64.64")), 0);
    EXPECT_EQ(info.default_value, std::to_string(VAR_DOUBLE_DEFAULT_VAL));
    destroyArgv(argc, argv);

    Cmdkey::reset();
    info.find("var_double");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_DOUBLE_EQ(cmdkey_var_double, VAR_DOUBLE_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, std::to_string(VAR_DOUBLE_DEFAULT_VAL));
    EXPECT_EQ(info.default_value, std::to_string(VAR_DOUBLE_DEFAULT_VAL));

    makeArgv({"unittest", "--var_double=-64.64"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_double");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_DOUBLE_EQ(cmdkey_var_double, -64.64);
    EXPECT_EQ(info.current_value.find(std::string("-64.64")), 0);
    EXPECT_EQ(info.default_value.find(std::to_string(VAR_DOUBLE_DEFAULT_VAL)), 0);
    destroyArgv(argc, argv);

    Cmdkey::reset();
    makeArgv({"unittest", "--var_double=12345678912"}, argc, argv);
    EXPECT_NO_THROW(Cmdkey::parseCommandLine(argc, argv));
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_double=abc"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_double=abc64.64"}, argc, argv);
    EXPECT_THROW(Cmdkey::parseCommandLine(argc, argv), std::invalid_argument);
    destroyArgv(argc, argv);
    Cmdkey::reset();
    makeArgv({"unittest", "--var_double=64.64abc"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    EXPECT_DOUBLE_EQ(cmdkey_var_double, 64.64);
    destroyArgv(argc, argv);
}

TEST(Cmdkey, String)
{
    int argc = 0;
    char** argv = nullptr;

    CmdkeyInfo info;
    info.find("var_string");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(cmdkey_var_string, VAR_STRING_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, VAR_STRING_DEFAULT_VAL);
    EXPECT_EQ(info.default_value, VAR_STRING_DEFAULT_VAL);

    makeArgv({"unittest", "--var_string=64.64"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_string");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_string, std::string("64.64"));
    EXPECT_EQ(info.current_value, std::string("64.64"));
    EXPECT_EQ(info.default_value, VAR_STRING_DEFAULT_VAL);
    destroyArgv(argc, argv);

    Cmdkey::reset();
    info.find("var_string");
    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(cmdkey_var_string, VAR_STRING_DEFAULT_VAL);
    EXPECT_EQ(info.current_value, VAR_STRING_DEFAULT_VAL);
    EXPECT_EQ(info.default_value, VAR_STRING_DEFAULT_VAL);

    makeArgv({"unittest", "--var_string=-64.64"}, argc, argv);
    Cmdkey::parseCommandLine(argc, argv);
    info.find("var_string");
    EXPECT_TRUE(info.valid);
    EXPECT_FALSE(info.is_default);
    EXPECT_EQ(cmdkey_var_string, std::string("-64.64"));
    EXPECT_EQ(info.current_value, std::string("-64.64"));
    EXPECT_EQ(info.default_value, VAR_STRING_DEFAULT_VAL);
    destroyArgv(argc, argv);

    Cmdkey::reset();
    makeArgv({"unittest", "--var_string=12345678912"}, argc, argv);
    EXPECT_NO_THROW(Cmdkey::parseCommandLine(argc, argv));
    destroyArgv(argc, argv);
}

TEST(Cmdkey, TotalKey)
{
    int argc = 0;
    char** argv = nullptr;

    Cmdkey::reset();
    makeArgv({"unittest", "--var_bool=false", "--var_int32=-32", "--var_uint32=32", "--var_int64=-64",
              "--var_uint64=64", "--var_float=32.32", "--var_double=64.64", "--var_string=abc def"}, argc, argv);
    int destroy_argc = argc;
    Cmdkey::parseCommandLine(argc, argv);
    EXPECT_EQ(cmdkey_var_bool, false);
    EXPECT_EQ(cmdkey_var_int32, -32);
    EXPECT_EQ(cmdkey_var_uint32, 32);
    EXPECT_EQ(cmdkey_var_int64, -64);
    EXPECT_EQ(cmdkey_var_uint64, 64);
    EXPECT_FLOAT_EQ(cmdkey_var_float, 32.32);
    EXPECT_DOUBLE_EQ(cmdkey_var_double, 64.64);
    EXPECT_EQ(cmdkey_var_string, std::string("abc def"));
    destroyArgv(destroy_argc, argv);
}

TEST(Cmdkey, RemoveKey)
{
    int argc = 0;
    char** argv = nullptr;

    Cmdkey::reset();
    makeArgv({"unittest", "depend", "--var_bool=false", "--var_int32=-32", "--var_uint32=32", "--var_int64=-64",
              "--var_uint64=64", "--var_float=32.32", "--var_double=64.64", "--var_string=abc def", "abc", "def gh"}, argc, argv);
    int destroy_argc = argc;
    EXPECT_EQ(argc, 12);
    Cmdkey::parseCommandLine(argc, argv, true);
    EXPECT_EQ(cmdkey_var_bool, false);
    EXPECT_EQ(cmdkey_var_int32, -32);
    EXPECT_EQ(cmdkey_var_uint32, 32);
    EXPECT_EQ(cmdkey_var_int64, -64);
    EXPECT_EQ(cmdkey_var_uint64, 64);
    EXPECT_FLOAT_EQ(cmdkey_var_float, 32.32);
    EXPECT_DOUBLE_EQ(cmdkey_var_double, 64.64);
    EXPECT_EQ(cmdkey_var_string, std::string("abc def"));

    EXPECT_EQ(argc, 4);
    EXPECT_STREQ(argv[1], "depend");
    EXPECT_STREQ(argv[2], "abc");
    EXPECT_STREQ(argv[3], "def gh");
    destroyArgv(destroy_argc, argv);
}
