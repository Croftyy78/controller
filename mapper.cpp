#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <headers/mapper.h>



LidarMapper::LidarMapper(double real_width_m, double real_height_m, double res) 
    : resolution(res) {

    width = static_cast<int>(std::ceil(real_width_m / resolution));
    height = static_cast<int>(std::ceil(real_height_m / resolution));
    // Создаем серую карту (127 - неизвестное пространство)
    map = cv::Mat(height, width, CV_8UC1, cv::Scalar(127));
}

    // Перевод из метров в пиксели: (0,0) метров -> в правом нижнем углу (width-1, height-1)
cv::Point LidarMapper::worldToPixel(double x, double y) const {
    int u = static_cast<int>(width - 1 - (x / resolution));
    int v = static_cast<int>(height - 1 - (y / resolution));
    return cv::Point(u, v);
}

    // Проверка, находится ли пиксель внутри границ изображения
bool LidarMapper::isInside(const cv::Point& pt) const {
    return (pt.x >= 0 && pt.x < width && pt.y >= 0 && pt.y < height);
}

    // Обработка одного скана лидара
void LidarMapper::addScan(const Point2D& sensor_pos, const std::vector<Point2D>& scan_points) {
    cv::Point sensor_pt = worldToPixel(sensor_pos.x, sensor_pos.y); // позиция робота на картинке карты
    if (!isInside(sensor_pt)) return; // если он выходит из границ то выходим из функции 

    cv::circle(map, sensor_pt, 4, cv::Scalar(255), cv::FILLED); // закрашиваем потому что под роботом стен нету

    for (const auto& pt : scan_points) { // проходимся циклом по точкам, которые имеют абсолютную координату 
        cv::Point obstacle_pt = worldToPixel(pt.x, pt.y); // превращаем коориданты точки в координаты на карте 
        if (isInside(obstacle_pt)) { // проверяем что точка не вышла за пределы карты
            // Проводим луч "пустого (белого) пространства" 
            // Используем алгоритм Брезенхема (cv::LineIterator), чтобы луч не затирал само препятствие
            cv::LineIterator it(map, sensor_pt, obstacle_pt, 8);
            for (int i = 0; i < it.count - 1; ++i, ++it) { 
                **it = 255; // Закрашиваем линию свободным пространством
            }
            // 2. Саму точку попадания отмечаем ЧЕРНЫМ (препятствие - 0)
            map.at<uchar>(obstacle_pt) = 0;
        }
    }
    cv::imwrite("map.png", map);
}

Point2D LidarMapper::pixelToWorld(double u, double v) const{
    // Математически точная инверсия функции worldToPixel
    double x = (static_cast<double>(width - 1) - u) * resolution;
    double y = (static_cast<double>(height - 1) - v) * resolution;
    return Point2D {x, y}; 
}

// Концептуальный пример поиска фронтиров с помощью OpenCV
cv::Point LidarMapper::findClosestFrontier(cv::Point robot_pixel) {
    cv::Mat free_space, unknown_space, frontiers;
    
    // Выделяем свободное пространство (255)
    cv::threshold(map, free_space, 254, 255, cv::THRESH_BINARY);
    
    // Выделяем неизвестное пространство (127)
    cv::inRange(map, 127, 127, unknown_space);
    
    // Находим границы свободного пространства (расширяем его на 1 пиксель)
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat free_dilated;
    cv::dilate(free_space, free_dilated, kernel);
    
    // Пересечение расширенного свободного пространства с неизвестным даст нам линию фронтира
    cv::bitwise_and(free_dilated, unknown_space, frontiers);
    
    // Ищем все ненулевые пиксели во frontiers
    std::vector<cv::Point> frontier_points;
    cv::findNonZero(frontiers, frontier_points);
    
    if (frontier_points.empty()) {
        return cv::Point(-1, -1); // Фронтиров нет, карта построена
    }
    
    // Ищем ближайший к роботу фронтир (для простоты)
    cv::Point best_target = frontier_points[0];
    double min_dist = std::numeric_limits<double>::max();
    
    for (const auto& pt : frontier_points) {
        double dist = std::sqrt(std::pow(pt.x - robot_pixel.x, 2) + std::pow(pt.y - robot_pixel.y, 2));
        if (dist < min_dist && dist > 4.0) {
            min_dist = dist;
            best_target = pt;
        }
    }
    return best_target;
}

std::vector<Point2D> LidarMapper::findPathAStar(cv::Point start, cv::Point goal) const {
    std::vector<cv::Point> path;
    std::vector<Point2D> path_m; 
    
    // Проверка: находятся ли старт и финиш в пределах карты
    if (!isInside(start) || !isInside(goal)) return path_m;

    // Массив стоимостей (g) и таблица родителей для восстановления пути
    // Заполняем бесконечностью
    std::vector<std::vector<double>> g_cost(height, std::vector<double>(width, std::numeric_limits<double>::infinity()));
    std::vector<std::vector<cv::Point>> came_from(height, std::vector<cv::Point>(width, cv::Point(-1, -1)));
    
    // Очередь с приоритетом для обработки узлов (выдает узел с наименьшим f)
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    
    // Инициализация стартовой точки
    g_cost[start.y][start.x] = 0.0;
    open_set.push({start, 0.0, 0.0});
    
    // Массивы смещений для 8 соседей (4 прямых, 4 диагональных)
    int dx[] = {0, 1, 0, -1, 1, 1, -1, -1};
    int dy[] = {-1, 0, 1, 0, -1, 1, 1, -1};
    double cost[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414}; 
    
    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();
        
        // Если достигли цели — восстанавливаем путь с конца
        if (current.pos == goal) {
            cv::Point curr = goal;
            while (curr != start) {
                path.push_back(curr);
                curr = came_from[curr.y][curr.x];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end()); // Разворачиваем путь от старта к цели

            path_m.reserve(path.size());
            for (const auto& pt : path) {
                Point2D world_pt = pixelToWorld(pt.x, pt.y);
                path_m.push_back(world_pt);
            }

            return path_m;
        }
        
        // Если мы достали дубликат узла с худшей стоимостью — пропускаем
        if (current.g > g_cost[current.pos.y][current.pos.x]) continue;
        
        // Проверяем всех 8 соседей
        for (int i = 0; i < 8; ++i) {
            cv::Point neighbor(current.pos.x + dx[i], current.pos.y + dy[i]);
            
            // Если сосед за пределами карты — пропускаем
            if (!isInside(neighbor)) continue;
            
            // Считываем значение карты
            // Если это препятствие (0), туда ехать нельзя
            if (map.at<uchar>(neighbor) == 0) continue; 
            
            // Рассчитываем новую стоимость пути до соседа
            double tentative_g = current.g + cost[i];
            
            // Если новый путь короче, чем ранее найденный
            if (tentative_g < g_cost[neighbor.y][neighbor.x]) {
                g_cost[neighbor.y][neighbor.x] = tentative_g;
                came_from[neighbor.y][neighbor.x] = current.pos;
                
                // Рассчитываем эвристику (Евклидово расстояние до цели)
                double h = std::sqrt(std::pow(neighbor.x - goal.x, 2) + std::pow(neighbor.y - goal.y, 2));
                
                open_set.push({neighbor, tentative_g, h});
            }
        }
    }
    
    // Если очередь пуста, а цель не найдена, возвращаем пустой вектор
    return path_m; 
}

