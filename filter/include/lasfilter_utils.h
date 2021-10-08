#ifndef LASFILTER_UTILS
#define LASFILTER_UTILS
#include <iostream>
#include <string.h>
#ifdef  _WIN32
#include<io.h>
#include<direct.h>
#else defined linux
#include <sys/io.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

inline void splitpath(const std::string path, std::string& drive, std::string& dir, std::string& fname, std::string& ext);

inline void split_whole_path(std::string path, std::string& drive, std::string& dir, std::string& fname, std::string& ext);

inline void splitpath(const std::string path, std::string& drive, std::string& dir, std::string& fname, std::string& ext)
{
	std::string str;

	char pre_char = path[0];
	str.push_back(path[0]);

	int len = path.size();

	for (int i = 1; i < len; ++i)
	{

		if (path[i] == '/')
		{
			if (pre_char == '/' || pre_char == '\\')
				continue;
			else
				str.push_back(path[i]);
		}
		else if (path[i] == '\\')
		{
			if (pre_char == '/' || pre_char == '\\')
				continue;
			else
				str.push_back('/');
		}
		else
		{
			str.push_back(path[i]);
		}

		pre_char = str.back();
	}
	split_whole_path(str, drive, dir, fname, ext);
}

inline void split_whole_path(std::string path, std::string& drive, std::string& dir, std::string& fname, std::string& ext)
{
	const size_t whole_length = path.length();

	if (whole_length == 0)
		return;

#ifdef _WIN32
	size_t colon_pos = path.find_first_of(":");
	if (colon_pos != std::string::npos)
		drive = path.substr(0, colon_pos);
#endif

	size_t slant_pos = path.find_last_of("/");

	if (slant_pos == std::string::npos)
		return;

	if (slant_pos == whole_length - 1)
	{
		dir = path;
		return;
	}
	else
	{
		dir = path.substr(0, slant_pos + 1);

		std::string whole_name = path.substr(slant_pos + 1);

		size_t dot_pos = whole_name.find_last_of(".");

		if (dot_pos == std::string::npos)
			return;

		fname = whole_name.substr(0, dot_pos);
		ext = whole_name.substr(dot_pos);
	}

	return;
}


#endif 


