#include <cstdarg>
#include <iostream>

#include "utils.h"

static unsigned short msglevel = 2;

void messageLog(unsigned short level, const char* fmt, ...) {
	if (level > msglevel)return;
	va_list args;
	va_start(args, fmt);
	std::vfprintf(stderr, fmt, args);
	std::fflush(stderr);
	va_end(args);
}

void setMessageLogLevel(unsigned short level) {
	msglevel = level;
}

short getMessageLogLevel() {
	return msglevel;
}