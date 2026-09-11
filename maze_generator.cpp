#include <webots/Supervisor.hpp>
#include <vector>
#include <stack>
#include <random>
#include <chrono>
#include <string>
#include <cstdio>
#include <iostream>
#include <headers/generate.h>

using namespace webots;

// Настройки лабиринта
const int MAZE_WIDTH = 10;   // Количество клеток по X
const int MAZE_HEIGHT = 10;  // Количество клеток по Y (на плоскости)

// Размеры под вашу задачу
const double CELL_SIZE = 0.5;      // Ширина и длина клетки (25 см)
const double WALL_THICKNESS = 0.05; // Толщина стены (2 см)
const double WALL_HEIGHT = 0.5;    // Высота стены (15 см)

const double IMAGE_WIDTH_m = MAZE_WIDTH * CELL_SIZE + WALL_THICKNESS;
const double IMAGE_HEIGHT_m = MAZE_HEIGHT * CELL_SIZE + WALL_THICKNESS;

struct Cell {
    int x;
    int y;
    bool visited = false;
    // [0: Верх(Y-), 1: Право(X+), 2: Низ(Y+), 3: Лево(X-)]
    bool walls[4] = {true, true, true, true}; 
};

// Функция для добавления стены (Адаптирована под Z-Up координаты)
void spawnWall(Supervisor *supervisor, Field *childrenField, double x, double y, double z, double sizeX, double sizeY, double sizeZ) {
    char nodeString[1024];
    
    // Формируем текстовое описание узла Solid
    // Теперь sizeX и sizeY - это размеры по полу, а sizeZ - высота стены
    std::snprintf(nodeString, sizeof(nodeString),
        "Solid { "
        "  translation %f %f %f "
        "  children [ "
        "    Shape { "
        "      appearance PBRAppearance { baseColor 0.4 0.4 0.4 roughness 1.0 } "
        "      geometry Box { size %f %f %f } "
        "    } "
        "  ] "
        "  boundingObject Box { size %f %f %f } "
        "}",
        x, y, z, 
        sizeX, sizeY, sizeZ,
        sizeX, sizeY, sizeZ
    );

    // Добавляем созданную стену в конец списка объектов мира
    childrenField->importMFNodeFromString(-1, nodeString);
}

void mazeGenerate(Supervisor *supervisor) {
    
    int timeStep = (int)supervisor->getBasicTimeStep();
    
    Node *rootNode = supervisor->getRoot();
    Field *childrenField = rootNode->getField("children");

    std::cout << "Starting maze generation..." << std::endl;

    std::vector<std::vector<Cell>> grid(MAZE_WIDTH, std::vector<Cell>(MAZE_HEIGHT));
    for (int x = 0; x < MAZE_WIDTH; ++x) {
        for (int y = 0; y < MAZE_HEIGHT; ++y) {
            grid[x][y].x = x;
            grid[x][y].y = y;
        }
    }

    unsigned seed = std::chrono::steady_clock::now().time_since_epoch().count();
    std::mt19937 rng(seed);

    std::stack<Cell*> stack;
    Cell* start_cell = &grid[0][0];
    start_cell->visited = true;
    stack.push(start_cell);

    while (!stack.empty()) {
        Cell* current = stack.top();
        stack.pop();

        int cx = current->x;
        int cy = current->y;
        std::vector<Cell*> neighbors;

        if (cy > 0 && !grid[cx][cy - 1].visited) neighbors.push_back(&grid[cx][cy - 1]);
        if (cx < MAZE_WIDTH - 1 && !grid[cx + 1][cy].visited) neighbors.push_back(&grid[cx + 1][cy]);
        if (cy < MAZE_HEIGHT - 1 && !grid[cx][cy + 1].visited) neighbors.push_back(&grid[cx][cy + 1]);
        if (cx > 0 && !grid[cx - 1][cy].visited) neighbors.push_back(&grid[cx - 1][cy]);

        if (!neighbors.empty()) {
            stack.push(current);

            std::uniform_int_distribution<size_t> dist(0, neighbors.size() - 1);
            Cell* next = neighbors[dist(rng)];

            if (next->x > current->x) {
                current->walls[1] = false; next->walls[3] = false;
            } else if (next->x < current->x) {
                current->walls[3] = false; next->walls[1] = false;
            } else if (next->y > current->y) {
                current->walls[2] = false; next->walls[0] = false;
            } else if (next->y < current->y) {
                current->walls[0] = false; next->walls[2] = false;
            }

            next->visited = true;
            stack.push(next);
        }
    }

    std::cout << "Maze generated logically. Spawning walls on XY plane..." << std::endl;

    double wallLength = CELL_SIZE + WALL_THICKNESS; 
    // Координата Z для всех стен одинакова (чтобы стены стояли на полу)
    double worldZ = WALL_HEIGHT / 2.0;

    for (int x = 0; x < MAZE_WIDTH; ++x) {
        for (int y = 0; y < MAZE_HEIGHT; ++y) {
            // Центр клетки на плоскости X/Y
            double worldX = x * CELL_SIZE;
            double worldY = y * CELL_SIZE; 

            // ВЕРХНЯЯ стена (горизонтальная, ориентирована вдоль оси X)
            if (grid[x][y].walls[0]) {
                spawnWall(supervisor, childrenField, 
                          worldX, worldY - CELL_SIZE / 2.0, worldZ, 
                          wallLength, WALL_THICKNESS, WALL_HEIGHT);
            }

            // ЛЕВАЯ стена (вертикальная, ориентирована вдоль оси Y)
            if (grid[x][y].walls[3]) {
                spawnWall(supervisor, childrenField, 
                          worldX - CELL_SIZE / 2.0, worldY, worldZ, 
                          WALL_THICKNESS, wallLength, WALL_HEIGHT);
            }
            
            // ПРАВАЯ граница лабиринта (крайний столбец)
            if (x == MAZE_WIDTH - 1 && grid[x][y].walls[1]) {
                spawnWall(supervisor, childrenField, 
                          worldX + CELL_SIZE / 2.0, worldY, worldZ, 
                          WALL_THICKNESS, wallLength, WALL_HEIGHT);
            }

            // НИЖНЯЯ граница лабиринта (крайняя строка)
            if (y == MAZE_HEIGHT - 1 && grid[x][y].walls[2]) {
                spawnWall(supervisor, childrenField, 
                          worldX, worldY + CELL_SIZE / 2.0, worldZ, 
                          wallLength, WALL_THICKNESS, WALL_HEIGHT);
            }
        }
    }

    std::cout << "Maze built completely!" << std::endl;

    supervisor->step(timeStep);
}



