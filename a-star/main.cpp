#include <iostream>
#include "source/AStar.hpp"

int main()
{
    AStar::Generator generator; 
    generator.setWorldSize({25, 25}); //初始化地图数据
    generator.addCollision({ 0,13 });
    generator.addCollision({1,13});
    generator.addCollision({ 2,13 });
    generator.addCollision({ 3,13 });
    generator.addCollision({ 4,13 });
    generator.addCollision({ 5,13 });
    generator.addCollision({ 6,13 });
    generator.addCollision({ 7,13 });
    generator.addCollision({ 8,13 });
    generator.addCollision({ 9,13 });
    generator.addCollision({ 10,13 });
    generator.addCollision({ 11,13 });
    generator.addCollision({ 12,13 });
    generator.addCollision({ 13,13 });
    generator.addCollision({ 15,13 });
    generator.addCollision({ 16,13 });
    generator.addCollision({ 17,13 });
    generator.addCollision({ 18,13 });
    generator.addCollision({ 19,13 });
    generator.addCollision({ 20,13 });
    generator.addCollision({ 21,13 });
    generator.addCollision({ 22,13 });
    generator.addCollision({ 23,13 });
    generator.addCollision({ 24,13 });
    generator.addCollision({ 25,13 });

    generator.addCollision({ 12,11 });
    generator.addCollision({ 13,11 });
    generator.addCollision({ 14,11 });
    generator.addCollision({ 15,11 });
    generator.addCollision({ 16,11 });

	generator.addCollision({ 12,15 });
	generator.addCollision({ 13,15 });
	generator.addCollision({ 14,15 });
	generator.addCollision({ 15,15 });
	generator.addCollision({ 16,15 });

    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    std::cout << "Generate path ... \n";
    auto path = generator.findPath({15, 25}, {1, 3}); //寻找 0 点到20 点的路径

    //打印路径
    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }

    while (1);
}