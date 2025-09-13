#pragma once

#ifndef _MESH_H_
#define _MESH_H_

#include <iostream>
#include <SDL.h>
#include <vector>
#include <unordered_map>

#include "vector2.h"

struct Edge
{
	Vector2 p1, p2;
	Edge(Vector2 _p1, Vector2 _p2) : p1(_p1), p2(_p2) {}
};

struct Triangle
{
	Vector2 p1, p2, p3;

	Triangle(Vector2 _p1, Vector2 _p2, Vector2 _p3) : p1(_p1), p2(_p2), p3(_p3) {}

	bool is_in_circumcircle(const Vector2& D) const
	{
		double ax = p1.x - D.x, ay = p1.y - D.y;
		double bx = p2.x - D.x, by = p2.y - D.y;
		double cx = p3.x - D.x, cy = p3.y - D.y;

		double det = (ax * ax + ay * ay) * (bx * cy - cx * by) -
			(bx * bx + by * by) * (ax * cy - cx * ay) +
			(cx * cx + cy * cy) * (ax * by - bx * ay);

		double area = (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);

		if (area < 0)
			det = -det;

		return det > 0;
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

	void render(SDL_Renderer* renderer)
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

namespace std {
	template<> struct hash<Vector2> {
		size_t operator()(const Vector2& point) const {
			return hash<double>()(point.x) ^ (hash<double>()(point.y) << 1);
		}
	};
}

class Mesh: public Singleton<Mesh>
{
	friend class Singleton<Mesh>;

public:
	void instialize()
	{
		Vector2 left_up(0, 0);
		Vector2 right_up(1280, 0);
		Vector2 left_down(0, 720);
		Vector2 right_down(1280, 720);

		triangles.push_back(Triangle(left_up, right_up, left_down));
		triangles.push_back(Triangle(left_down, right_down, right_up));
	}

	void ganerate_mesh(std::vector<Polygon> polygons)
	{
		for (Polygon polygon : polygons)
		{
			for (int i = 0; i < polygon.vertices.size(); i++)
			{
				points.push_back(polygon.vertices[i]);
				Edge edge = Edge(polygon.vertices[i], polygon.vertices[(i + 1) % polygon.vertices.size()]);
				constraint_edges.push_back(edge);
			}
		}

		for (Vector2& point : points)
		{
			for (Triangle& triangle : triangles)
			{
				if (triangle.is_in_circumcircle(point))
					bad_triangles.push_back(triangle);
			}

			for (Triangle& triangle : bad_triangles)
			{
				Edge e1(triangle.p1, triangle.p2);
				Edge e2(triangle.p2, triangle.p3);
				Edge e3(triangle.p3, triangle.p1);

				bool found1 = false, found2 = false, found3 = false;

				for (const Triangle& t : bad_triangles) 
				{
					if (&triangle == &t) continue;

					if ((t.p1 == e1.p1 && t.p2 == e1.p2) ||
						(t.p2 == e1.p1 && t.p3 == e1.p2) ||
						(t.p3 == e1.p1 && t.p1 == e1.p2) ||
						(t.p1 == e1.p2 && t.p2 == e1.p1) ||
						(t.p2 == e1.p2 && t.p3 == e1.p1) ||
						(t.p3 == e1.p2 && t.p1 == e1.p1))
						found1 = true;

					if ((t.p1 == e2.p1 && t.p2 == e2.p2) ||
						(t.p2 == e2.p1 && t.p3 == e2.p2) ||
						(t.p3 == e2.p1 && t.p1 == e2.p2) ||
						(t.p1 == e2.p2 && t.p2 == e2.p1) ||
						(t.p2 == e2.p2 && t.p3 == e2.p1) ||
						(t.p3 == e2.p2 && t.p1 == e2.p1))
						found2 = true;

					if ((t.p1 == e3.p1 && t.p2 == e3.p2) ||
						(t.p2 == e3.p1 && t.p3 == e3.p2) ||
						(t.p3 == e3.p1 && t.p1 == e3.p2) ||
						(t.p1 == e3.p2 && t.p2 == e3.p1) ||
						(t.p2 == e3.p2 && t.p3 == e3.p1) ||
						(t.p3 == e3.p2 && t.p1 == e3.p1))
						found3 = true;
				}

				if (!found1) edges.push_back(e1);
				if (!found2) edges.push_back(e2);
				if (!found3) edges.push_back(e3);
			}

			for (const auto& triangle : bad_triangles) {
				triangles.erase(std::remove(triangles.begin(), triangles.end(), triangle), triangles.end());
			}

			for (const auto& edge : edges) {
				triangles.push_back(Triangle(edge.p1, edge.p2, point));
			}

			bad_triangles.clear();
			edges.clear();
		}

		for (Triangle triangle : triangles)
		{
			Vector2 voronoi_vertex = triangle.get_circumcenter();

			voronoi_vertices.push_back(voronoi_vertex);

			for (const Triangle& t : triangles)
			{
				if (&triangle == &t) continue;
				
				if ((triangle.p1 == t.p1 && triangle.p2 == t.p2) ||
					(triangle.p2 == t.p1 && triangle.p3 == t.p2) ||
					(triangle.p3 == t.p1 && triangle.p1 == t.p2) ||
					(triangle.p1 == t.p2 && triangle.p2 == t.p1) ||
					(triangle.p2 == t.p2 && triangle.p3 == t.p1) ||
					(triangle.p3 == t.p2 && triangle.p1 == t.p1) ||
					(triangle.p1 == t.p2 && triangle.p2 == t.p3) ||
					(triangle.p2 == t.p2 && triangle.p3 == t.p3) ||
					(triangle.p3 == t.p2 && triangle.p1 == t.p3) ||
					(triangle.p1 == t.p3 && triangle.p2 == t.p2) ||
					(triangle.p2 == t.p3 && triangle.p3 == t.p2) ||
					(triangle.p3 == t.p3 && triangle.p1 == t.p2) ||
					(triangle.p1 == t.p3 && triangle.p2 == t.p1) ||
					(triangle.p2 == t.p3 && triangle.p3 == t.p1) ||
					(triangle.p3 == t.p3 && triangle.p1 == t.p1) ||
					(triangle.p1 == t.p1 && triangle.p2 == t.p3) ||
					(triangle.p2 == t.p1 && triangle.p3 == t.p3) ||
					(triangle.p3 == t.p1 && triangle.p1 == t.p3))
				{
					voronoi_diagram[voronoi_vertex].push_back(t.get_circumcenter());
				}

				if (voronoi_diagram[voronoi_vertex].size() == 3) break;
			}
		}
	}

	void render(SDL_Renderer* renderer)
	{
		SDL_SetRenderDrawColor(renderer, 100, 255, 100, 255);

		for (Triangle triangle : triangles)
			triangle.render(renderer);

		SDL_SetRenderDrawColor(renderer, 100, 100, 255, 255);

		for (Vector2 vertex : voronoi_vertices)
		{
			for (Vector2 v : voronoi_diagram[vertex])
			{
				SDL_RenderDrawLine(renderer, vertex.x, vertex.y, v.x, v.y);
			}
		}
	}

private:
	std::vector<Vector2> points;
	std::vector<Vector2> voronoi_vertices;
	std::vector<Edge> edges;
	std::vector<Edge> constraint_edges;
	std::vector<Triangle> triangles;
	std::vector<Triangle> bad_triangles;

	std::unordered_map<Vector2, std::vector<Vector2>> voronoi_diagram;

private:
	Mesh() = default;
	~Mesh() = default;
};

#endif //!_MESH_H_