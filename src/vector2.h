#pragma once

#ifndef _VECTOR2_H_
#define _VECTOR2_H_

#include <cmath>

class Vector2
{
public:
	double x, y;

public:
	Vector2() = default;
	~Vector2() = default;

	Vector2(double x, double y)	: x(x), y(y) {}

	Vector2 operator+(const Vector2& vec) const
	{
		return Vector2(x + vec.x, y + vec.y);
	}

	void operator+=(const Vector2& vec)
	{
		x += vec.x;
		y += vec.y;
	}

	Vector2 operator-(const Vector2& vec) const
	{
		return Vector2(x - vec.x, y - vec.y);
	}

	void operator-=(const Vector2& vec)
	{
		x -= vec.x, y -= vec.y;
	}

	double operator*(const Vector2& vec) const
	{
		return x * vec.x + y * vec.y;
	}

	Vector2 operator*(const double val) const
	{
		return Vector2(x * val, y * val);
	}

	void operator*=(const double val)
	{
		x *= val, y *= val;
	}

	Vector2 operator/(const double val) const
	{
		return Vector2(x / val, y / val);
	}

	void operator/=(const double val)
	{
		x /= val, y /= val;
	}

	bool operator==(const Vector2& vec) const
	{
		return x == vec.x && y == vec.y;
	}

	double length() const
	{
		return sqrt(x * x + y * y);
	}

	Vector2 normalize() const
	{
		double len = this->length();
		return Vector2(x / len, y / len);
	}

	Vector2 perpendicular() const 
	{
		return Vector2(-y, x);
	}
};

#endif // !_VECTOR2_H_
