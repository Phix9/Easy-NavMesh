#pragma once

#ifndef _SCENE_H_
#define _SCENE_H_

#include <vector>
#include <SDL.h>
#include <SDL_ttf.h>

#include "vector2.h"
#include "resources_manager.h"
#include "button.h"
#include "mesh.h"

class Scene
{
public:
	void handle_event(SDL_Event& e)
	{
		int x, y;
		SDL_GetMouseState(&x, &y);

		for (auto& button : button_list)
		{
			is_mouse_hovering_button = (x >= button.rect.x && x <= button.rect.x + button.rect.w &&
				y >= button.rect.y && y <= button.rect.y + button.rect.h);
			if (is_mouse_hovering_button)
				break;
		}
		
		if (e.type == SDL_MOUSEBUTTONDOWN)
		{
			if (is_mouse_hovering_button)
			{
				for (auto& button : button_list)
				{
					if (x >= button.rect.x && x <= button.rect.x + button.rect.w &&
						y >= button.rect.y && y <= button.rect.y + button.rect.h)
					{
						button.on_click();
					}
				}
			}

			else
			{
				if (!is_mesh_generated) 
				{
					current_polygon.vertices.push_back(Vector2((double)x, (double)y));
				}

				else if (!is_start_set)
				{
					start = Vector2((double)x, (double)y);
					is_start_set = true;
				}

				else
				{
					end = Vector2((double)x, (double)y);
					Mesh::instance()->find_path(start, end);
				}
			}
		}
	}

	void update()
	{
		render();
	}

public:
	Scene(SDL_Renderer* _renderer, SDL_Window* _window) :renderer(_renderer), window(_window)
	{
		button_list.push_back(Button(renderer, window, { 20, 20, 140, 50 }, ResourcesManager::instance()->get_font(), "New Polygon", std::bind(&Scene::on_New_Polygon_click, this)));
		button_list.push_back(Button(renderer, window, { 20, 90, 140, 50 }, ResourcesManager::instance()->get_font(), "Generate Mesh", std::bind(&Scene::on_Generate_Mesh_click, this)));
		button_list.push_back(Button(renderer, window, { 20, 160, 140, 50 }, ResourcesManager::instance()->get_font(), "Voronoi Diagram", std::bind(&Scene::on_Voronoi_Diagram_click, this)));
		button_list.push_back(Button(renderer, window, { 20, 230, 140, 50 }, ResourcesManager::instance()->get_font(), "Reset", std::bind(&Scene::on_Reset_click, this)));

		current_polygon = Polygon({}, false);
	}
	~Scene() = default;

private:
	SDL_Renderer* renderer;
	SDL_Window* window;

	std::vector<Button> button_list;

	Vector2 start;
	Vector2 end;

	Polygon current_polygon;
	std::vector<Polygon> polygons;

	bool is_mouse_hovering_button;
	bool is_mesh_generated = false;
	bool is_voronoi_diagram_generated = false;
	bool is_start_set = false;

private:

	void on_New_Polygon_click()
	{
		if (current_polygon.vertices.size() >= 3)
		{
			current_polygon.done = true;
			polygons.push_back(current_polygon);
			current_polygon = Polygon({}, false);
		}
	}

	void on_Generate_Mesh_click()
	{
		if (current_polygon.vertices.size() >= 3)
		{
			current_polygon.done = true;
			polygons.push_back(current_polygon);
			current_polygon = Polygon({}, false);
		}
		else
		{
			current_polygon = Polygon({}, false);
		}

		Mesh::instance()->instialize();
		Mesh::instance()->ganerate_mesh(polygons);

		is_mesh_generated = true;
	}

	void on_Voronoi_Diagram_click()
	{
		if (is_mesh_generated)
		{
			is_voronoi_diagram_generated = true;
		}
	}

	void on_Reset_click()
	{
		polygons.clear();
		is_mesh_generated = false;
		is_voronoi_diagram_generated = false;
		is_start_set = false;

		Mesh::instance()->reset();
	}

	void render()
	{
		for (Button button : button_list)
		{
			button.render();
		}

		Mesh::instance()->render(renderer);

		if(is_voronoi_diagram_generated)
			Mesh::instance()->render_voronoi_diagram(renderer);

		SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);

		for (Polygon polygon : polygons) polygon.render(renderer);

		for (int i = 0; current_polygon.vertices.size() > 1 && i < current_polygon.vertices.size() - 1; i++)
		{
			SDL_RenderDrawLine(renderer, current_polygon.vertices[i].x, current_polygon.vertices[i].y,
				current_polygon.vertices[i + 1].x, current_polygon.vertices[i + 1].y);
		}
	}
};

#endif // !_SCENE_H_
