#pragma once

#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <string>
#include <functional>
#include <SDL.h>
#include <SDL_ttf.h>

#include "vector2.h"

class Scene;

class Button
{
	friend class Scene;

public:
	void render()
	{
		SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
		SDL_RenderDrawRect(renderer, &rect);
		render_text_in_center(renderer, font, text, rect, text_color);
	}

private:
	SDL_Window* window;
	SDL_Renderer* renderer;

	SDL_Rect rect;
	std::string text; 
	TTF_Font* font;
	SDL_Color text_color;

	std::function<void()> on_click;

private:
	void render_text_in_center(SDL_Renderer* renderer, TTF_Font* font, std::string text, SDL_Rect rect, SDL_Color color)
	{
		SDL_Surface* surface = TTF_RenderText_Blended(font, text.c_str(), color);
		SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);

		int textWidth = surface->w;
		int textHeight = surface->h;

		SDL_Rect textRect;
		textRect.x = rect.x + (rect.w - textWidth) / 2;
		textRect.y = rect.y + (rect.h - textHeight) / 2;
		textRect.w = textWidth;
		textRect.h = textHeight;

		SDL_RenderCopy(renderer, texture, NULL, &textRect);

		SDL_DestroyTexture(texture);
		SDL_FreeSurface(surface);
	}

public:
	Button(SDL_Renderer* _renderer, SDL_Window* _window, SDL_Rect _rect, TTF_Font* _font, std::string _text, std::function<void()> _on_click) :
		renderer(_renderer), window(_window), rect(_rect), font(_font), text(_text), on_click(_on_click)
	{
		text_color = SDL_Color{ 255, 255, 255, 255 };
	}
	~Button() = default;
};

#endif // !_BUTTON_H_
