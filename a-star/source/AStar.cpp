#include "AStar.hpp"
#include <algorithm>
#include <math.h>

using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic (HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

// ( 起始点 , 目标点 )
AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    Node *current = nullptr; //初始化当前点
    NodeSet openSet, closedSet; //初始化 开放列表 和 关闭列表
    openSet.reserve(100); 
    closedSet.reserve(100);
    openSet.push_back(new Node(source_)); //将起始点置入开放列表

    //开放列表不为空时循环其中节点
    while (!openSet.empty()) 
    {
        auto current_it = openSet.begin(); //遍历列表,从头部开始取出节点当做当前节点
        current = *current_it;

        //遍历开放列表中所有节点,对比代价,取出代价最小的节点作为当前节点
        for (auto it = openSet.begin(); it != openSet.end(); it++) 
        {
            auto node = *it;
            if (node->getScore() <= current->getScore()) 
            {
                current = node;
                current_it = it;
            }
        }

        //对比当前节点的坐标是否是终点
        if (current->coordinates == target_) 
        {
            break; //如果找到了终点就退出循环
        }

        //没有找到终点,将当前节点放入关闭列表不在关心
        closedSet.push_back(current);
        openSet.erase(current_it); //将当前节点从开发列表中移除,不在关心

        //遍历方向
        for (uint i = 0; i < directions; ++i) 
        {
            //根据方向获得下一个坐标
            Vec2i newCoordinates(current->coordinates + direction[i]);
            // 查询该坐标是否是墙 || 该坐标是否在关闭列表
            if ( detectCollision(newCoordinates) || findNodeOnList(closedSet, newCoordinates) ) 
            {
                continue; 
            }

            //根据方向得出走一步的代价消耗
            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            //查询新节点是否在开放列表
            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) 
            {
                //不在开放列表则初始化该节点,将上一个节点设为父节点
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_); //计算出当前点到终点的代价消耗
                openSet.push_back(successor); //加入开放列表
            }
            else if (totalCost < successor->G) 
            {
                //下一个坐标在开放列表中,则对比两个节点的消耗,上一个节点的消耗小于新节点,则将新节点的父节点设置为上一个节点
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    //将找到的所有节点根据其父节点倒推,得到路径
    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
