#ifndef FILEINFO_H
#define FILEINFO_H

#include <string>

class FileInfo
{
public:
    void clear();
    void reset(const std::string& full_path);

public:
    const std::string& extention() const { return extention_; }
    const std::string& file_name() const { return file_name_; }
    const std::string& full_path() const { return full_path_; }
    const std::string& parent() const { return parent_; }

private:
    // full_path_ = parent_ + "/" + file_name_ + extention_
    std::string extention_;     ///< 扩展名，包含小数点
    std::string file_name_;     ///< 文件名，不包含扩展名
    std::string full_path_;     ///< 全路径
    std::string parent_;        ///< 路径
};

#endif // FILEINFO_H
