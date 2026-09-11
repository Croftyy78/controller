#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Lidar.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include <cmath>
#include <headers/mapper.h>
#include <headers/generate.h>

using namespace std;
using namespace webots;

void count_position(double sensorPosleft, double sensorPosright, double &leftSensorOld, double &rightSensorOld, Point2D &pos, double &ugol, double &old_ugol) {
    const float R = 0.095;
    double delta_S = (R * (sensorPosright - rightSensorOld + sensorPosleft - leftSensorOld))/ 2.0; // смещение центра

    pos.x += delta_S * cos((ugol + old_ugol) / 2);
    pos.y += delta_S * sin((ugol + old_ugol) / 2);

    old_ugol = ugol;
    leftSensorOld = sensorPosleft;
    rightSensorOld = sensorPosright;
}

vector <Point2D> getAbsolutCoordinatesOfPoint(const float* points, Point2D pos, int sizeRes, double robot_orientation) {
    vector <Point2D> res;
    res.reserve(sizeRes);
    const float TWO_PI = 2.0 * M_PI;

    for (int i = 0; i < sizeRes; i++) {
        float dist = points[i];
        if (isinf(dist) || isnan(dist)) 
            continue;
        
        float angle = robot_orientation - (static_cast<double>(i) / sizeRes) * TWO_PI;
        
        // Переводим полярные координаты в декартовы локальные и прибавляем позицию робота
        float abs_x = pos.x + dist * cos(angle);
        float abs_y = pos.y + dist * sin(angle);
        res.push_back({abs_x, abs_y});
    }

    return res;
}


void runRobot(Robot *robot) {
    Point2D robotPosition = {0.0, 0.0};
    int timeStep = static_cast<int>(robot -> getBasicTimeStep()); // получаем базовую частоту обновления сцены

    // Получаем моторы через объект supervisor
    Motor *left_wheel = robot -> getMotor("left wheel");
    Motor *right_wheel = robot -> getMotor("right wheel");

    // Переводим моторы в режим работы по скорости (position = INFINITY)
    left_wheel -> setPosition(INFINITY); 
    right_wheel -> setPosition(INFINITY);
    // устаналиваем начальную скорость 0 
    left_wheel -> setVelocity(0.0); 
    right_wheel -> setVelocity(0.0);
    
    PositionSensor *leftPosSensor = robot -> getPositionSensor("left wheel sensor");  // берем указатель на левый энкодер
    PositionSensor *rightPosSensor = robot -> getPositionSensor("right wheel sensor"); // берем указатель на правый энкодер

    if (leftPosSensor == nullptr) { // проверяем что левый энкодер нашелся
        cout << "Левый энкодер не найден" << endl;
        return;
    }

    if (rightPosSensor == nullptr) { // проверяем что правый энкодер нашелся
        cout << "Правый энкодер не найден" << endl;
        return;
    }
    // включаем энкодеры
    rightPosSensor -> enable(timeStep); 
    leftPosSensor -> enable(timeStep);

    Lidar *lidar = robot -> getLidar("lidar"); // инициализируем лидар
    if (lidar == nullptr) { // проверяем что он успешно инициализировался
        cout << "лидар не найден" << endl;
        return;
    }
    lidar -> enable(timeStep); // включаем лидар

    InertialUnit *imu = robot -> getInertialUnit("imu"); // инициализируем imu + акселерометр 
    if (imu == nullptr) { // проверяем что он успешно инициалзировался
        cout << "imu не найден" << endl;
        return;
    }
    imu -> enable(timeStep); // включаем imu 

    LidarMapper map(IMAGE_WIDTH_m, IMAGE_HEIGHT_m, 0.05); // создаем объект карты в масштбом на пиксель в 25 см в квадрате
    int count_point = lidar -> getHorizontalResolution();
    vector <Point2D> path;
    // инициализация перменных для хранения предыдущего значения
    double oldEncoderLeft = 0, oldEncoderRight = 0; 
    double oldYaw = 0;
    cout << "Начинаю основной цикл" << endl;
    while (robot -> step(timeStep) != -1) {
        const double *rpy = imu -> getRollPitchYaw(); // считываем значение углов
        double yaw = rpy[2]; // берем угол куда смотрит робот 

        // считываем значения с энкодеров
        double encoderLeft = leftPosSensor -> getValue();
        double encoderRight = rightPosSensor -> getValue();
        
        count_position(encoderLeft, encoderRight, oldEncoderLeft, oldEncoderRight, robotPosition, yaw, oldYaw); // считаем текущую позицию
        cout << "x " << robotPosition.x << " y " << robotPosition.y << endl;
        const float *rangeImage = lidar -> getRangeImage(); // берем данные с лидара 
        map.addScan(robotPosition, getAbsolutCoordinatesOfPoint(rangeImage, robotPosition, count_point, yaw)); // обновляем карту

        cout << "Карта обновлена" << endl;

        // планирование движения
        if (path.empty()) {
            cout << "Путь закончился вычисляю новый" << endl;
            cv::Point robot_pixel = map.worldToPixel(robotPosition.x, robotPosition.y);
            cv::Point frontier_pixel = map.findClosestFrontier(robot_pixel); // берем ближающую точку пересечения не иследованной зоны с доступной по т. Пифагора 

            if (frontier_pixel != cv::Point(-1, -1)) {
                path = map.findPathAStar(robot_pixel, frontier_pixel); // строим путь до границы
                if (!path.empty())
                    path.erase(path.begin());
            } else {
                cout << "Лабиринт полностью исследован" << endl;
                break;
            }
        }

        double leftSpeed = 0.0;
        double rightSpeed = 0.0;

        if (!path.empty()) {
            Point2D target = path.front();
            cout << "Я еду в x: " << target.x << " y " << target.y << endl;
            double dx = target.x - robotPosition.x;
            double dy = target.y - robotPosition.y;
            double distance = sqrt(dx*dx + dy*dy);

            // Если финальная точка маршрута достигнута (осталась 1 точка)
            if (path.size() == 1 && distance < 0.05) {
                path.clear(); 
                cout << "осталась одна точка до нее 5 см" << endl;
            } else {
                // Look-ahead: удаляем из начала пути все точки, которые ближе 15 см к роботу
                // Это заставит робота всегда смотреть "немного вперед" по маршруту
                while (path.size() > 1 && distance < 0.15) {
                    path.erase(path.begin());
                    target = path.front();
                    dx = target.x - robotPosition.x;
                    dy = target.y - robotPosition.y;
                    distance = sqrt(dx*dx + dy*dy);
                }

                double targetAngle = atan2(dy, dx);
                double angleError = targetAngle - yaw;
                
                while (angleError > M_PI) angleError -= 2.0 * M_PI;
                while (angleError < -M_PI) angleError += 2.0 * M_PI;

                double baseSpeed = 3.0; 
                double Kp = 4.0;        

                if (abs(angleError) > 0.5) {
                    baseSpeed = 0.0;
                }

                leftSpeed = baseSpeed - Kp * angleError;
                rightSpeed = baseSpeed + Kp * angleError;

                leftSpeed = max(-5.0, min(5.0, leftSpeed));
                rightSpeed = max(-5.0, min(5.0, rightSpeed));
                cout << "Скорость левого колеса " << leftSpeed << " Скорость правого колеса " << rightSpeed << endl;
            }
        } 
        
        left_wheel -> setVelocity(leftSpeed);
        right_wheel -> setVelocity(rightSpeed);
    
    }

    left_wheel -> setVelocity(0.0);
    right_wheel -> setVelocity(0.0);

    cout << "Завершил" << endl;
}