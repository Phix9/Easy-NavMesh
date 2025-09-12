#pragma once

#ifndef _RESOURCES_MANAGER_H_
#define _RESOURCES_MANAGER_H_

#include <SDL.h>
#include <SDL_ttf.h>

#include "singleton.h"

class ResourcesManager :public Singleton<ResourcesManager>
{
	friend class Singleton<ResourcesManager>;

public:

public:
	bool load_from_file(SDL_Renderer* renderer)
	{
		font = TTF_OpenFont("../res/font/Roboto-Regular.ttf", 17);

		if (font)
			return true;
		else
			return false;
	}

	TTF_Font* get_font() const
	{
		return font;
	}

private:
	TTF_Font* font;

private:
	ResourcesManager() = default;
	~ResourcesManager() = default;
};

#endif // !_RESOURCES_MANAGER_H_
