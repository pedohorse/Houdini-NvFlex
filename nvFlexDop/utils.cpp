#include <cstdarg>
#include <iostream>

#include "utils.h"



void messageLog(unsigned short level, const char* fmt, ...) {
	if (level > 999)return;
	va_list args;
	va_start(args, fmt);
	std::vfprintf(stderr, fmt, args);
	std::fflush(stderr);
	va_end(args);
}