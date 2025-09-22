#define SDL_MAIN_HANDLED

#include <iostream>
#include <SDL.h>
#include <SDL_ttf.h>

#include "vector2.h"
#include "scene.h"

bool running = true;

int main()
{
	if (SDL_Init(SDL_INIT_EVERYTHING)) {
		SDL_Log("SDL_2 Initialization failed: %s", SDL_GetError());
		return -1;
	}
	TTF_Init();

	SDL_Window* window = SDL_CreateWindow(u8"NavMesh", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, SDL_WINDOW_SHOWN);
	SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_TARGETTEXTURE);

	if (!window) {
		SDL_Log("Window creation failed: %s", SDL_GetError());
		SDL_Quit();
		return -1;
	}
	if (!renderer) {
		SDL_Log("Renderer creation failed: %s", SDL_GetError());
		SDL_Quit();
		return -1;
	}

	ResourcesManager::instance()->load_from_file(renderer);

	Uint32 frame_start;
	int frame_time;

	Scene* scene = new Scene(renderer, window);

	SDL_Rect rect = { 20, 20, 100, 50 };

	while (running)
	{
		frame_start = SDL_GetTicks();

		SDL_Event e;
		while (SDL_PollEvent(&e))
		{
			if (e.type == SDL_QUIT) {
				running = false;
			}
			scene->handle_event(e);
		}

		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
		SDL_RenderClear(renderer);

		scene->update();

		SDL_RenderPresent(renderer);

		frame_time = SDL_GetTicks() - frame_start;

		if (frame_time < 1000 / 60)
		{
			SDL_Delay(1000 / 60 - frame_time);
		}
	}
}