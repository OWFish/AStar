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

// ( ��ʼ�� , Ŀ��� )
AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    Node *current = nullptr; //��ʼ����ǰ��
    NodeSet openSet, closedSet; //��ʼ�� �����б� �� �ر��б�
    openSet.reserve(100); 
    closedSet.reserve(100);
    openSet.push_back(new Node(source_)); //����ʼ�����뿪���б�

    //�����б�Ϊ��ʱѭ�����нڵ�
    while (!openSet.empty()) 
    {
        auto current_it = openSet.begin(); //�����б�,��ͷ����ʼȡ���ڵ㵱����ǰ�ڵ�
        current = *current_it;

        //���������б������нڵ�,�Աȴ���,ȡ��������С�Ľڵ���Ϊ��ǰ�ڵ�
        for (auto it = openSet.begin(); it != openSet.end(); it++) 
        {
            auto node = *it;
            if (node->getScore() <= current->getScore()) 
            {
                current = node;
                current_it = it;
            }
        }

        //�Աȵ�ǰ�ڵ�������Ƿ����յ�
        if (current->coordinates == target_) 
        {
            break; //����ҵ����յ���˳�ѭ��
        }

        //û���ҵ��յ�,����ǰ�ڵ����ر��б��ڹ���
        closedSet.push_back(current);
        openSet.erase(current_it); //����ǰ�ڵ�ӿ����б����Ƴ�,���ڹ���

        //��������
        for (uint i = 0; i < directions; ++i) 
        {
            //���ݷ�������һ������
            Vec2i newCoordinates(current->coordinates + direction[i]);
            // ��ѯ�������Ƿ���ǽ || �������Ƿ��ڹر��б�
            if ( detectCollision(newCoordinates) || findNodeOnList(closedSet, newCoordinates) ) 
            {
                continue; 
            }

            //���ݷ���ó���һ���Ĵ�������
            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            //��ѯ�½ڵ��Ƿ��ڿ����б�
            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) 
            {
                //���ڿ����б����ʼ���ýڵ�,����һ���ڵ���Ϊ���ڵ�
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_); //�������ǰ�㵽�յ�Ĵ�������
                openSet.push_back(successor); //���뿪���б�
            }
            else if (totalCost < successor->G) 
            {
                //��һ�������ڿ����б���,��Ա������ڵ������,��һ���ڵ������С���½ڵ�,���½ڵ�ĸ��ڵ�����Ϊ��һ���ڵ�
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    //���ҵ������нڵ�����丸�ڵ㵹��,�õ�·��
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
