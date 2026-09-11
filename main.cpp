#include <iostream>
#include <headers/generate.h>
#include <headers/robot_controller.h>
#include <webots/Supervisor.hpp>

using namespace std;
using namespace webots;

int main() {
    Supervisor *supervisor = new Supervisor();

    // Генерация лабиринта
    mazeGenerate(supervisor);
    
    // Запуск контроллера робота
    runRobot(supervisor);

    // Освобождаем память в самом конце
    delete supervisor;
    return 0;
}