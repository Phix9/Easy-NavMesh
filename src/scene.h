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
				current_polygon.vertices.push_back(Vector2{ (double)x,(double)y });
			}
		}
	}

	void render()
	{
		for (Button button : button_list)
		{
			button.render();
		}

		SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

		Mesh::instance()->render(renderer);

		for (Polygon polygon : polygons) polygon.render(renderer);
		
		for (int i = 0; current_polygon.vertices.size() > 1 && i < current_polygon.vertices.size() - 1; i++)
		{
			SDL_RenderDrawLine(renderer, current_polygon.vertices[i].x, current_polygon.vertices[i].y,
				current_polygon.vertices[i + 1].x, current_polygon.vertices[i + 1].y);
		}
	}

public:
	Scene(SDL_Renderer* _renderer, SDL_Window* _window) :renderer(_renderer), window(_window)
	{
		button_list.push_back(Button(renderer, window, { 20, 20, 100, 50 }, ResourcesManager::instance()->get_font(), "New", std::bind(&Scene::on_New_click, this)));
		button_list.push_back(Button(renderer, window, { 20, 90, 100, 50 }, ResourcesManager::instance()->get_font(), "Finish", std::bind(&Scene::on_Finish_click, this)));

		current_polygon = Polygon({}, false);
	}
	~Scene() = default;

private:
	SDL_Renderer* renderer;
	SDL_Window* window;

	std::vector<Button> button_list;

	Polygon current_polygon;
	std::vector<Polygon> polygons;

	bool is_mouse_hovering_button;

private:

	void on_New_click()
	{
		std::cout << "New" << std::endl;

		if (current_polygon.vertices.size() >= 3)
		{
			current_polygon.done = true;
			polygons.push_back(current_polygon);
			current_polygon = Polygon({}, false);
		}
	}

	void on_Finish_click()
	{
		std::cout << "Finish" << std::endl;

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
		//polygons.clear();
	}
};

#endif // !_SCENE_H_
