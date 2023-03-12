#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include "ros_graph/command_line_param.h"

void createArgv(const std::vector<std::string>& params, int& argc, char**& argv)
{
    argc = static_cast<int>(params.size());
    argv = new char*[argc + 1];
    for (size_t i = 0; i < params.size(); i++)
    {
        argv[i] = new char[params[i].size() + 1];
        memcpy(argv[i], params[i].data(), params[i].size());
        argv[i][params[i].size()] = '\0';
    }
    argv[argc] = nullptr;
}

void destroyArgv(int argc, char**& argv)
{
    for (int i = 0; i < argc; i++)
    {
        delete argv[i];
    }
    delete argv;
}

TEST(CommandLineParam, DefaultValue)
{
    int argc = 0;
    char** argv = nullptr;
    createArgv(std::vector<std::string>{"ros_graph", "depend"}, argc, argv);

    CommandLineParam* param;
    param = new CommandLineParam();
    EXPECT_TRUE(param->init(argc, argv));
    EXPECT_FALSE(param->is_package_set_by_cmd);
    EXPECT_TRUE(param->package.empty());
    EXPECT_FALSE(param->is_workspace_dir_set_by_cmd);
    EXPECT_EQ(param->workspace_dir, boost::filesystem::current_path().string());
    delete param;
    destroyArgv(argc, argv);
}

TEST(CommandLineParam, ReadFromCommandLine)
{
    int argc = 0;
    char** argv = nullptr;
    createArgv(std::vector<std::string>{"ros_graph", "depend", "--package=cmd_package", "--workspace_dir=cmd_workspace_dir"}, argc, argv);

    CommandLineParam* param;
    param = new CommandLineParam();
    EXPECT_TRUE(param->init(argc, argv));
    EXPECT_TRUE(param->is_package_set_by_cmd);
    EXPECT_EQ(param->package, "cmd_package");
    EXPECT_TRUE(param->is_workspace_dir_set_by_cmd);
    EXPECT_EQ(param->workspace_dir, "cmd_workspace_dir");
    delete param;
    destroyArgv(argc, argv);
}
