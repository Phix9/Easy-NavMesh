#pragma once

#ifndef _SINGLETON_H_
#define _SINGLETON_H_

template <typename T>
class Singleton
{
public:
	static T* instance() 
	{
		if (!_instance)
			_instance = new T();
		return _instance;
	}

private:
	static T* _instance;

protected:
	Singleton() = default;
	~Singleton() = default;
	Singleton(const Singleton&) = delete;
	Singleton& operator = (const Singleton&) = delete;
};

template <typename T>
T* Singleton<T>::_instance = nullptr;

#endif // !_SINGLETON_H_
