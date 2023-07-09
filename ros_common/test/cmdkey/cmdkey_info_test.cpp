#include <gtest/gtest.h>

#include "ros_common/cmdkey/cmdkey_info.h"

TEST(CmdkeyInfo, DefaultConstructor)
{
    CmdkeyInfo info;

    EXPECT_FALSE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_TRUE(info.ptr == nullptr);
    EXPECT_TRUE(info.name.empty());
    EXPECT_TRUE(info.type.empty());
    EXPECT_TRUE(info.description.empty());
    EXPECT_TRUE(info.current_value.empty());
    EXPECT_TRUE(info.default_value.empty());
}

TEST(CmdkeyInfo, Constructor)
{
    bool key = false;
    std::string name("key");
    std::string type("bool");
    std::string desc("this is a description");
    std::string val("false");
    CmdkeyInfo info(&key, name, type, desc, val);

    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(info.ptr, (&key));
    EXPECT_EQ(info.name, name);
    EXPECT_EQ(info.type, type);
    EXPECT_EQ(info.description, desc);
    EXPECT_EQ(info.current_value, val);
    EXPECT_EQ(info.default_value, val);
}

TEST(CmdkeyInfo, CopyConstructor)
{
    bool key = false;
    std::string name("key");
    std::string type("bool");
    std::string desc("this is a description");
    std::string val("false");
    CmdkeyInfo info(&key, name, type, desc, val);
    CmdkeyInfo other(info);

    EXPECT_EQ(info.valid, other.valid);
    EXPECT_EQ(info.is_default, other.is_default);
    EXPECT_EQ(info.ptr, other.ptr);
    EXPECT_EQ(info.name, other.name);
    EXPECT_EQ(info.type, other.type);
    EXPECT_EQ(info.description, other.description);
    EXPECT_EQ(info.current_value, other.current_value);
    EXPECT_EQ(info.default_value, other.default_value);
}

TEST(CmdkeyInfo, OperatorAssign)
{
    bool key = false;
    std::string name("key");
    std::string type("bool");
    std::string desc("this is a description");
    std::string val("false");
    CmdkeyInfo info(&key, name, type, desc, val);
    CmdkeyInfo other;
    other = info;

    EXPECT_EQ(info.valid, other.valid);
    EXPECT_EQ(info.is_default, other.is_default);
    EXPECT_EQ(info.ptr, other.ptr);
    EXPECT_EQ(info.name, other.name);
    EXPECT_EQ(info.type, other.type);
    EXPECT_EQ(info.description, other.description);
    EXPECT_EQ(info.current_value, other.current_value);
    EXPECT_EQ(info.default_value, other.default_value);
}

TEST(CmdkeyInfo, copy)
{
    bool key = false;
    std::string name("key");
    std::string type("bool");
    std::string desc("this is a description");
    std::string val("false");
    CmdkeyInfo info(&key, name, type, desc, val);
    CmdkeyInfo other;
    other.copy(info);

    EXPECT_EQ(info.valid, other.valid);
    EXPECT_EQ(info.is_default, other.is_default);
    EXPECT_EQ(info.ptr, other.ptr);
    EXPECT_EQ(info.name, other.name);
    EXPECT_EQ(info.type, other.type);
    EXPECT_EQ(info.description, other.description);
    EXPECT_EQ(info.current_value, other.current_value);
    EXPECT_EQ(info.default_value, other.default_value);
}

TEST(CmdkeyInfo, reset)
{
    bool key = false;
    std::string name("key");
    std::string type("bool");
    std::string desc("this is a description");
    std::string val("false");
    CmdkeyInfo info(&key, name, type, desc, val);
    info.is_default = false;
    info.current_value = "other value";
    info.reset();

    EXPECT_TRUE(info.valid);
    EXPECT_TRUE(info.is_default);
    EXPECT_EQ(info.ptr, (&key));
    EXPECT_EQ(info.name, name);
    EXPECT_EQ(info.type, type);
    EXPECT_EQ(info.description, desc);
    EXPECT_EQ(info.current_value, val);
    EXPECT_EQ(info.default_value, val);
}
