#include "fileinfo.h"

#include <boost/filesystem.hpp>

void FileInfo::clear()
{
    extention_ = "";
    file_name_ = "";
    full_path_ = "";
    parent_ = "";
}

void FileInfo::reset(const std::string &full_path)
{
    boost::filesystem::path path(full_path);
    extention_ = path.extension().string();
    file_name_ = path.filename().string();
    full_path_ = full_path;
    parent_ = path.parent_path().string();

    file_name_ = file_name_.substr(0, file_name_.size() - extention_.size());
}
