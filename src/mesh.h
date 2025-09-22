#pragma once

#ifndef _MESH_H_
#define _MESH_H_

#include <SDL.h>
#include <queue>
#include <vector>
#include <unordered_set>
#include <unordered_map>

#include "vector2.h"

namespace std 
{
	template<> struct hash<Vector2>
	{
		size_t operator()(const Vector2& point) const
		{
			size_t h1 = hash<double>()(point.x);
			size_t h2 = hash<double>()(point.y);
			return h1 ^ (h2 << 1);
		}
	};
}

struct Edge
{
	Vector2 p1, p2;

	bool operator==(const Edge& other) const
	{
		return (p1 == other.p1 && p2 == other.p2) ||
			(p1 == other.p2 && p2 == other.p1);
	}

	Edge() = default;
	Edge(Vector2 _p1, Vector2 _p2) : p1(_p1), p2(_p2) {}
};

struct Triangle
{
	Vector2 p1, p2, p3;

	const bool is_in_circumcircle(const Vector2& point) const
	{
		double ax = p1.x - point.x, ay = p1.y - point.y;
		double bx = p2.x - point.x, by = p2.y - point.y;
		double cx = p3.x - point.x, cy = p3.y - point.y;

		double det = (ax * ax + ay * ay) * (bx * cy - cx * by) -
			(bx * bx + by * by) * (ax * cy - cx * ay) +
			(cx * cx + cy * cy) * (ax * by - bx * ay);

		double area = (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);

		if (area < 0)
			det = -det;

		return det > 0;
	}

	const Vector2 get_centroid() const
	{
		return Vector2((p1.x + p2.x + p3.x) / 3, (p1.y + p2.y + p3.y) / 3);
	}

	const Vector2 get_circumcenter() const
	{
		double ax = p1.x, ay = p1.y,
			bx = p2.x, by = p2.y,
			cx = p3.x, cy = p3.y;

		double d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));

		double ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
		double uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;

		return Vector2(ux, uy);
	}

	const bool contains_edge(const Edge& edge) const
	{
		return (p1 == edge.p1 && p2 == edge.p2) ||
			(p2 == edge.p1 && p3 == edge.p2) ||
			(p3 == edge.p1 && p1 == edge.p2) ||
			(p1 == edge.p2 && p2 == edge.p1) ||
			(p2 == edge.p2 && p3 == edge.p1) ||
			(p3 == edge.p2 && p1 == edge.p1);
	}

	const void render(SDL_Renderer* renderer) const
	{
		SDL_RenderDrawLine(renderer, p1.x, p1.y, p2.x, p2.y);
		SDL_RenderDrawLine(renderer, p2.x, p2.y, p3.x, p3.y);
		SDL_RenderDrawLine(renderer, p3.x, p3.y, p1.x, p1.y);
	}

	bool operator==(const Triangle& other) const
	{
		return (p1 == other.p1 || p1 == other.p2 || p1 == other.p3) &&
			(p2 == other.p1 || p2 == other.p2 || p2 == other.p3) &&
			(p3 == other.p1 || p3 == other.p2 || p3 == other.p3);
	}

	Triangle(Vector2 _p1, Vector2 _p2, Vector2 _p3) : p1(_p1), p2(_p2), p3(_p3) {}
};

struct Polygon
{
	std::vector<Vector2> vertices;
	bool done;

	void render(SDL_Renderer* renderer)
	{
		for (int i = 0; i < vertices.size(); i++)
		{
			SDL_RenderDrawLine(renderer, vertices[i].x, vertices[i].y,
				vertices[(i + 1) % vertices.size()].x, vertices[(i + 1) % vertices.size()].y);
		}
	}

	Polygon() = default;
	Polygon(std::vector<Vector2> _vertices, bool _done) :vertices(_vertices), done(_done) {}
};

const double heuristic(Vector2& p1, Vector2& p2)
{
	return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

struct Node
{
	double g_cost, h_cost;

	Vector2 position;
	
	Node* parent;

	const double get_f_cost() const
	{
		return g_cost + h_cost;
	}

	//bool operator<(const Node& n) const { return n.get_f_cost() < get_f_cost(); }

	Node() = default;
	Node(Vector2& _position, Node* _parent) : position(_position), parent(_parent) {}
};

struct NodeCompare {
	bool operator()(const Node* node1, const Node* node2) const {
		return node1->get_f_cost() > node2->get_f_cost();
	}
};

class Mesh : public Singleton<Mesh>
{
	friend class Singleton<Mesh>;

public:
	void instialize()
	{
		Vector2 left_up(0, 0);
		Vector2 right_up(1279, 0);
		Vector2 left_down(0, 719);
		Vector2 right_down(1279, 719);

		triangles.push_back(Triangle(left_up, right_up, left_down));
		triangles.push_back(Triangle(left_down, right_down, right_up));
	}

	void ganerate_mesh(std::vector<Polygon> polygons)
	{
		for (const Polygon& polygon : polygons)
		{
			for (int i = 0; i < polygon.vertices.size(); i++)
			{
				points.push_back(polygon.vertices[i]);
				Edge edge = Edge(polygon.vertices[i], polygon.vertices[(i + 1) % polygon.vertices.size()]);
				constraint_edges.push(edge);
			}
		}

		for (const Vector2& point : points)
		{
			add_point(point);
		}

		enforce_constraints();

		generate_navigation_mesh();
		generate_voronoi_diagram();
	}

	void find_path(Vector2& start, Vector2& end)
	{
		if (start == end) return;

		Vector2 start_vertex = find_closest_vertex(start);
		Vector2 end_vertex = find_closest_vertex(end);

		Node* current;

		Node* start_node = new Node(start_vertex, nullptr);

		start_node->g_cost = 0;
		start_node->h_cost = heuristic(start_node->position, end_vertex);

		nodes[start_node->position] = start_node;

		node_queue.push(start_node);

		while (!node_queue.empty())
		{
			current = node_queue.top();
			node_queue.pop();

			if (current->position == end_vertex)
			{
				path.clear();

				while (current != nullptr)
				{
					path.push_back(current->position);
					current = current->parent;
				}

				std::reverse(path.begin(), path.end());

				for (auto& node : nodes) delete node.second;
				nodes.clear();

				path.insert(path.begin(), start);
				path.push_back(end);

				while (!node_queue.empty())
					node_queue.pop();
				closed_set.clear();

				return;
			}

			closed_set.insert(current->position);

			for (Vector2& neighbor : navigation_mesh[current->position])
			{
				if (closed_set.find(neighbor) != closed_set.end()) continue;

				double tentative_g = current->g_cost + heuristic(current->position, neighbor);

				Node* neighbor_node = new Node(neighbor, current);
				neighbor_node->g_cost = tentative_g;
				neighbor_node->h_cost = heuristic(neighbor_node->position, end_vertex);

				node_queue.push(neighbor_node);

				if (nodes.find(neighbor) != nodes.end()) delete nodes[neighbor];

				nodes[neighbor] = neighbor_node;
			}
		}

		for (auto& node : nodes) delete node.second;
		nodes.clear();
	}

	void render(SDL_Renderer* renderer)
	{
		render_trianglation(renderer);
		render_path(renderer);
	}

	void render_voronoi_diagram(SDL_Renderer* renderer)
	{
		SDL_SetRenderDrawColor(renderer, 100, 100, 255, 255);

		for (const Vector2& vertex : voronoi_vertices)
		{
			for (const Vector2& v : voronoi_diagram[vertex])
			{
				SDL_RenderDrawLine(renderer, vertex.x, vertex.y, v.x, v.y);
			}
		}
	}

	void reset()
	{
		points.clear();
		path.clear();
		triangles.clear();

		while (!constraint_edges.empty()) constraint_edges.pop();
		while (!node_queue.empty()) node_queue.pop();

		closed_set.clear();

		for (const Vector2& vertex : voronoi_vertices) voronoi_diagram[vertex].clear();
		for (const Vector2& centroid : centroids) navigation_mesh[centroid].clear();
		
		centroids.clear();
		voronoi_vertices.clear();

		for (auto& node : nodes) delete node.second;
		nodes.clear();
	}

private:
	std::vector<Vector2> points;
	std::vector<Vector2> centroids;
	std::vector<Vector2> path;
	std::vector<Vector2> voronoi_vertices;

	std::vector<Triangle> triangles;

	std::queue<Edge> constraint_edges;

	std::priority_queue<Node*, std::vector<Node*>, NodeCompare> node_queue;

	std::unordered_set<Vector2> closed_set;

	std::unordered_map<Vector2, std::vector<Vector2>> navigation_mesh;
	std::unordered_map<Vector2, std::vector<Vector2>> voronoi_diagram;

	std::unordered_map<Vector2, Node*> nodes;	//存储节点指针对象，用于内存管理

private:
	void add_point(const Vector2& point)
	{
		std::vector<Triangle> bad_triangles;

		std::vector<Edge> polygon_edges;

		for (const Triangle& triangle : triangles)
		{
			if (triangle.is_in_circumcircle(point))
				bad_triangles.push_back(triangle);
		}

		for (const Triangle& triangle : bad_triangles)
		{
			Edge edge1(triangle.p1, triangle.p2);
			Edge edge2(triangle.p2, triangle.p3);
			Edge edge3(triangle.p3, triangle.p1);

			bool found1 = false, found2 = false, found3 = false;

			for (const Triangle& bad_triangle : bad_triangles)
			{
				if (&triangle == &bad_triangle) continue;

				if (bad_triangle.contains_edge(edge1))
					found1 = true;
				if (bad_triangle.contains_edge(edge2))
					found2 = true;
				if (bad_triangle.contains_edge(edge3))
					found3 = true;
			}

			if (!found1) polygon_edges.push_back(edge1);
			if (!found2) polygon_edges.push_back(edge2);
			if (!found3) polygon_edges.push_back(edge3);
		}

		for (const Triangle& triangle : bad_triangles)
			triangles.erase(std::remove(triangles.begin(), triangles.end(), triangle), triangles.end());

		for (const Edge& edge : polygon_edges)
			triangles.push_back(Triangle(edge.p1, edge.p2, point));
	}

	void enforce_constraints()
	{
		while (!constraint_edges.empty())
		{ 
			bool is_contained_top = false;

			Edge& constraint_edge = constraint_edges.front();

			for (Triangle& triangle : triangles)
			{
				if (triangle.contains_edge(constraint_edge))
				{
					is_contained_top = true;
					break;
				}
			}

			if (!is_contained_top)
			{
				Vector2 mid_point = Vector2((constraint_edge.p1.x + constraint_edge.p2.x) / 2, (constraint_edge.p1.y + constraint_edge.p2.y) / 2);

				points.push_back(mid_point);

				constraint_edges.push(Edge(constraint_edge.p1, mid_point));
				constraint_edges.push(Edge(constraint_edge.p2, mid_point));

				add_point(mid_point);
			}

			constraint_edges.pop();
		}
	}

	Vector2 find_closest_vertex(Vector2& point)
	{
		Vector2 closest_vertex = centroids[0];

		for (Vector2 centroid : centroids)
			if (heuristic(point, closest_vertex) > heuristic(point, centroid)) closest_vertex = centroid;

		return closest_vertex;
	}

	void generate_navigation_mesh()
	{
		for (const Triangle& triangle : triangles)
		{
			Vector2 centroid = triangle.get_centroid();

			centroids.push_back(centroid);

			for (const Triangle& t : triangles)
			{
				if (&triangle == &t) continue;

				if (triangle.contains_edge(Edge(t.p1, t.p2)) ||
					triangle.contains_edge(Edge(t.p2, t.p3)) ||
					triangle.contains_edge(Edge(t.p3, t.p1)))
				{
					navigation_mesh[centroid].push_back(t.get_centroid());
				}

				if (navigation_mesh[centroid].size() == 3) break;
			}
		}
	}

	void generate_voronoi_diagram()
	{
		for (const Triangle& triangle : triangles)
		{
			Vector2 voronoi_vertex = triangle.get_circumcenter();

			voronoi_vertices.push_back(voronoi_vertex);

			for (const Triangle& t : triangles)
			{
				if (&triangle == &t) continue;

				if (triangle.contains_edge(Edge(t.p1, t.p2)) ||
					triangle.contains_edge(Edge(t.p2, t.p3)) ||
					triangle.contains_edge(Edge(t.p3, t.p1)))
				{
					voronoi_diagram[voronoi_vertex].push_back(t.get_circumcenter());
				}

				if (voronoi_diagram[voronoi_vertex].size() == 3) break;
			}
		}
	}

	void render_trianglation(SDL_Renderer* renderer)
	{
		SDL_SetRenderDrawColor(renderer, 150, 255, 150, 255);

		for (const Triangle& triangle : triangles)
			triangle.render(renderer);
	}

	void render_path(SDL_Renderer* renderer)
	{
		if (path.size() < 2) return;

		SDL_SetRenderDrawColor(renderer, 100, 100, 255, 255);

		for (int i = 0; i < path.size() - 1; i++)
		{
			SDL_RenderDrawLine(renderer, path[i].x, path[i].y, path[i + 1].x, path[i + 1].y);
		}
	}

private:
	Mesh() = default;
	~Mesh() = default;
};

#endif //!_MESH_H_