#ifndef SEMANTICINFO_H
#define SEMANTICINFO_H

#include <string>

struct Map2D;

class SemanticInfo
{
public:
    void init(const Map2D& map2d, const std::string& osm_file);
    void mapToWorld(int mx, int my, double& wx, double& wy);
    void worldToMap(double wx, double wy, int& mx, int& my);

public:
    int widht() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }
    double origin_x() const { return origin_x_; }
    double origin_y() const { return origin_y_; }

private:
    void initFromYaml(const std::string& yaml_file);
    void initFromPgm(const std::string& pgm_file);
    void initFromOsm(const std::string& osm_file);

private:
    int width_;
    int height_;
    double resolution_;
    double origin_x_;
    double origin_y_;
};

#endif // SEMANTICINFO_H
