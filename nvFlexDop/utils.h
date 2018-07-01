#pragma once

void messageLog(unsigned short level, const char* fmt, ...);

void setMessageLogLevel(unsigned short level);
short getMessageLogLevel();