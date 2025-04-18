#pragma once

#include "SD.h"
#include "FS.h"

#include <string>
#include <vector>

using std::string;
using std::vector;

void listdir(const char *dirname, uint8_t levels);
bool readdir(const char *dirname, bool incdirs, bool incfiles, vector<string> &out);