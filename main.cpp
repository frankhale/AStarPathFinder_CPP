
#include <boost/numeric/ublas/matrix.hpp>
#include <cmath>
#include <iostream>
#include <vector> 

struct Point
{
public:
		Point(int x, int y)
		{
				this->x = x;
				this->y = y;
		}

		int x;
		int y;
};

struct AStarNode
{
public:
		AStarNode() {}
		AStarNode(const AStarNode& p, const Point& pos)
		{
				if (&parent != nullptr)
						parent = std::make_shared<AStarNode>(p);

				if (&position != nullptr)
						position = std::make_shared<Point>(pos);
		}

		bool eq(const AStarNode& x)
		{
				if (&x == nullptr) return false;

				return (position->x == x.position->x &&
						position->y == x.position->y);
		}

		std::shared_ptr<AStarNode> parent{};
		std::shared_ptr<Point> position{};
		int f = 0;
		int g = 0;
		int h = 0;
};

class AStarPathFinder
{
public:
		AStarPathFinder(boost::numeric::ublas::matrix<int> map)
		{
				this->map = std::make_shared<boost::numeric::ublas::matrix<int>>(map);
		}

		std::shared_ptr<std::vector<Point>> FindPath(Point start, Point end);

private:
		std::shared_ptr<boost::numeric::ublas::matrix<int>> map;
};

std::shared_ptr<std::vector<Point>> AStarPathFinder::FindPath(Point start, Point end)
{
		auto path = std::make_shared<std::vector<Point>>();

		AStarNode none{};

		auto start_node = std::make_shared<AStarNode>(none, start);
		auto end_node = std::make_shared<AStarNode>(none, end);
		auto open_list = std::make_shared<std::vector<std::shared_ptr<AStarNode>>>();
		auto closed_list = std::make_shared<std::vector<std::shared_ptr<AStarNode>>>();

		Point pos_array[4] = {
				{ 0, -1 },
				{ 0, 1 },
				{ -1, 0 },
				{ 1, 0 },
		};

		open_list->emplace_back(start_node);

		while (open_list->size() > 0)
		{
				auto current_node = open_list->front();
				int current_index = 0;

				int _index = 0;
				for (const auto& item : *open_list)
				{
						if (item->f < current_node->f)
						{
								current_node = item;
								current_index = _index;
						}
						_index++;
				}

				open_list->erase(open_list->begin() + current_index);
				closed_list->emplace_back(current_node);

				if (current_node->eq(*end_node))
				{
						auto current = current_node;
						while (current != nullptr && current->position != nullptr)
						{
								Point path_point = { current->position->x, current->position->y };
								path->emplace_back(path_point);
								current = current->parent;
						}

						std::reverse(path->begin(), path->end());
						return path;
				}

				auto children = std::make_shared<std::vector<std::shared_ptr<AStarNode>>>();

				for (const auto& new_position : pos_array)
				{
						auto node_position = std::make_shared<Point>(current_node->position->x + new_position.x, current_node->position->y + new_position.y);

						if (node_position->x > (map->size2() - 1) || node_position->x < 0 ||
								node_position->y >(map->size1() - 1) || node_position->y < 0) continue;

						if ((*map)(node_position->y, node_position->x) != 0) continue;

						auto child = std::make_shared<AStarNode>(*current_node, *node_position);
						children->emplace_back(child);
				}

				for (const auto& child : *children)
				{
						auto closed_list_result = std::find_if(closed_list->begin(), closed_list->end(),
								[&](const std::shared_ptr<AStarNode>& c) {
										return c->eq(*child);
								});

						if (closed_list_result != closed_list->end() && *closed_list_result != nullptr) 
								continue;

						child->g = current_node->g + 1;
						child->h = (int)pow(child->position->x - end_node->position->x, 2) + (int)pow(child->position->y - end_node->position->y, 2);
						child->f = child->g + child->h;

						auto open_node_result = std::find_if(open_list->begin(), open_list->end(),
								[&](const std::shared_ptr<AStarNode>& o) {
										return child->eq(*o) && child->g > o->g;
								});

						if (open_node_result != open_list->end() && *open_node_result != nullptr) 
								continue;

						open_list->emplace_back(child);
				}
		}

		return nullptr;
}

int main()
{
		int map[10][10] = {
				{ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
				{ 0, 0, 0, 0, 1, 0, 0, 1, 0, 0 },
				{ 0, 1, 1, 1, 1, 0, 0, 1, 0, 0 },
				{ 0, 1, 0, 0, 1, 1, 1, 1, 0, 0 },
				{ 0, 1, 0, 0, 0, 0, 1, 0, 0, 0 },
				{ 0, 1, 0, 1, 1, 0, 1, 0, 0, 0 },
				{ 0, 1, 0, 0, 1, 0, 1, 0, 0, 0 },
				{ 0, 1, 1, 0, 1, 0, 1, 1, 1, 0 },
				{ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
				{ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 }
		};

		auto _map = std::make_shared<boost::numeric::ublas::matrix<int>>(10,10);

		// convert map to a boost matrix
		for (int r = 0; r < 10; ++r)
		{
				for (int c = 0; c < 10; ++c)
				{
						(*_map)(r, c) = map[r][c];
				}
		}

		auto astar = std::make_shared<AStarPathFinder>(*_map);
		auto path = astar->FindPath({ 1, 0 }, { 6, 2 });

		for (int r = 0; r < 10; ++r)
		{
				for (int c = 0; c < 10; ++c)
				{
						auto p = std::find_if(path->begin(), path->end(),
								[&](const Point& s) {
										return s.x == c && s.y == r;
								});

						if (p != path->end()) std::cout << "+";
						else if ((*_map)(r, c) == 0) std::cout << ".";
						else if ((*_map)(r, c) == 1) std::cout << "#";
				}

				std::cout << std::endl;
		}

		//std::cout << "path steps: " << path->size() << std::endl;

		//for (const auto& step : *path)
		//{
		//		std::cout << "(" << step.x << ", " << step.y << ")" << std::endl;
		//}

		return 0;
}