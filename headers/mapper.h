#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include <limits>

struct Point2D {
    double x; // в метрах
    double y; // в метрах
};

struct AStarNode {
    cv::Point pos;
    double g; // Стоимость пути от старта до текущей клетки
    double h; // Эвристика (примерное расстояние до цели)
    
    // Для очереди с приоритетом (минимальный f = g + h всплывает наверх)
    bool operator>(const AStarNode& other) const {
        return (g + h) > (other.g + other.h);
    }
};

class LidarMapper {
private:
    int width, height;      // Размер карты в пикселях
    double resolution;      // Метров на пиксель (0.05 m = 5 cm)
    cv::Mat map;  

public:
    LidarMapper(double real_width_m, double real_height_m, double res);
    cv::Point worldToPixel(double x, double y) const;
    bool isInside(const cv::Point& pt) const;
    void addScan(const Point2D& sensor_pos, const std::vector<Point2D>& scan_points);
    Point2D pixelToWorld(double u, double v) const;
    cv::Point findClosestFrontier(cv::Point robot_pixel);
    std::vector<Point2D> findPathAStar(cv::Point start, cv::Point goal) const;
};