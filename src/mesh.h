#pragma once

#ifndef _MESH_H_
#define _MESH_H_

#include <iostream>
#include <SDL.h>
#include <vector>

#include "vector2.h"

struct Edge
{
	Vector2 p1, p2;
	Edge(Vector2 _p1, Vector2 _p2) : p1(_p1), p2(_p2) {}
};

struct Triangle
{
	Vector2 p1, p2, p3;
	Vector2 circumcenter;

	double circumradius;

	Triangle(Vector2 _p1, Vector2 _p2, Vector2 _p3) : p1(_p1), p2(_p2), p3(_p3) {}

	 bool is_in_circumcircle(const Vector2& D) const {
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

	 bool operator==(const Triangle& other) const
	 {
		 return (p1 == other.p1 || p1 == other.p2 || p1 == other.p3) &&
			 (p2 == other.p1 || p2 == other.p2 || p2 == other.p3) &&
			 (p3 == other.p1 || p3 == other.p2 || p3 == other.p3);
	 }

	void render(SDL_Renderer* renderer)
	{
		SDL_SetRenderDrawColor(renderer, 100, 255, 100, 255);
		SDL_RenderDrawLine(renderer, p1.x, p1.y, p2.x, p2.y);
		SDL_RenderDrawLine(renderer, p2.x, p2.y, p3.x, p3.y);
		SDL_RenderDrawLine(renderer, p3.x, p3.y, p1.x, p1.y);
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
			SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
			SDL_RenderDrawLine(renderer, vertices[i].x, vertices[i].y,
				vertices[(i + 1) % vertices.size()].x, vertices[(i + 1) % vertices.size()].y);
		}
	}

	Polygon() = default;
	Polygon(std::vector<Vector2> _vertices, bool _done) :vertices(_vertices), done(_done) {}
};

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

		//points.push_back(left_up);
		//points.push_back(right_up);
		//points.push_back(left_down);
		//points.push_back(right_down);

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

				for (const Triangle& t : bad_triangles) {
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
	}

	void render(SDL_Renderer* renderer)
	{
		for (Triangle triangle : triangles)
			triangle.render(renderer);
	}

private:
	std::vector<Vector2> points;
	std::vector<Edge> edges;
	std::vector<Edge> constraint_edges;
	std::vector<Triangle> triangles;
	std::vector<Triangle> bad_triangles;
	//std::vector<Polygon> polygons;

private:
	Mesh() = default;
	~Mesh() = default;
};

#endif //!_MESH_H_