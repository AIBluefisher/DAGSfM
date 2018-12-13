/*
Copyright (c) 2015, Tianwei Shen
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of libvot nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
/*! \file io_utils.cpp
 * \brief I/O utilities implementations
 *
 * This file contains some utility classes and functions to facilitate file read/write.
 */

#include <cstdlib>
#include <fstream>
#include <vector>
#include <cassert>
#include "io_utils.h"

// defines for file IO manipulation
#if defined(__WIN32__) || defined(_MSC_VER)
#include <io.h>
#include <windows.h>
#include <sys/stat.h>
#if defined(_MSC_VER)
#include <direct.h>
#endif
#else
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>
#endif
// define for GetAvailMem()
#if defined(__WIN32__) || defined(_MSC_VER)
#include <excpt.h>
#else
#include <sys/sysctl.h>
#endif

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace tw
{
template<typename T> T* allocate_2d(T ***array, int row, int col)
{
	*array = (T**) malloc(row * sizeof(T*));
	T *data = (T*) malloc(row * col * sizeof(T));
	for (int i = 0; i < row; i++) {
		(*array)[i] = data + i * col;
	}
	return data;
}

template<typename T> void free_2d(T ***array, T *data)
{
	free(data);
	free(*array);
}

template int* allocate_2d<int>(int ***array, int row, int col);
template void free_2d<int>(int ***array, int *data);

template float* allocate_2d<float>(float***array, int row, int col);
template void free_2d<float>(float ***array, float *data);

template double* allocate_2d<double>(double ***array, int row, int col);
template void free_2d<double>(double ***array, double *data);

template <class charType>
void IO::TrimLeft(std::basic_string<charType> & str, const char* chars2remove)
{
	if (!str.empty()) {  	//trim the characters in chars2remove from the left
		std::string::size_type pos = 0;
		if (chars2remove != NULL) {
			pos = str.find_first_not_of(chars2remove);

			if (pos != std::string::npos)
				str.erase(0,pos);
			else
				str.erase( str.begin() , str.end() ); // make empty
		}
		else        //trim space
		{
			pos = std::string::npos;        //pos = -1
			for (size_t i = 0; i < str.size(); ++i) {
				if (!isspace(str[i])) {
					pos = i;
					break;
				}
			}
			if (pos != std::string::npos) {
				if (pos > 0) {
					size_t length = str.size() - pos;
					for (size_t i = 0; i < length; ++i)
						str[i] = str[i+pos];
					str.resize(length);
				}
			}
			else
			{
				str.clear();
			}
		}
	}
}

template <class charType>
void IO::TrimRight(std::basic_string<charType> & str, const char* chars2remove)
{
	if (!str.empty()) {  	//trim the characters in chars2remove from the right
		std::string::size_type pos = 0;
		if (chars2remove != NULL) {
			pos = str.find_last_not_of(chars2remove);

			if (pos != std::string::npos)
				str.erase(pos+1);
			else
				str.erase( str.begin() , str.end() ); // make empty
		}
		else {       		//trim space
			pos = std::string::npos;
			for (int i = str.size()-1; i >= 0; --i) {
				if (!isspace(str[i])) {
					pos = i;
					break;
				}
			}
			if (pos != std::string::npos) {
				if (pos+1 != str.size())
					str.resize(pos+1);
			}
			else {
				str.clear();
			}
		}
	}
}

template <class charType>
void IO::Trim(std::basic_string<charType> & str, const char* chars2remove)
{
	TrimLeft(str, chars2remove);
	TrimRight(str, chars2remove);
}

bool IO::IsEmptyString(const std::string & str)
{
	if (str.empty()) {       //this is true when the length of str is 0
		return true;
	}
	for (size_t i = 0; i < str.length(); ++i) {
		if (!isspace(str[i]))
			return false;
	}
	return true;
}

int IO::ExtractLines(const char *input_file_path, std::vector<std::string> &lines)
{
	int lineNum = -1;
	std::ifstream fin(input_file_path);
	if (!fin.is_open()) {
		std::cerr << "Can't open the file " << input_file_path << "\n";
		return lineNum;
	}
	while (!fin.eof()) {
		std::string line;
		getline(fin, line);
		Trim(line);
		if (!IsEmptyString(line)) {
			lines.push_back(line);
			lineNum++;
		}
	}

	return lineNum;
}

bool IO::IsFileExist(const char *filename)      //POSIX
{
	struct stat buffer;
	return (stat(filename, &buffer) == 0);
}

bool IO::IsFileExist(const std::string& filename)
{
	std::ifstream infile(filename);
	return infile.good();
}

std::string IO::JoinPath(std::string folder, std::string filename)
{
#ifdef WIN32
	std::string sep = "\\";
#else
	std::string sep = "/";
#endif
	if (folder.length() > 0) {
		if (folder[folder.size()-1] != sep[0])
			return folder+sep+filename;
		else
			return folder+filename;
	}
	else
		return filename;
}

std::pair<std::string, std::string> IO::SplitPath(std::string path)
{
	std::pair<std::string, std::string> res;
	size_t found;
	found = path.find_last_of("/\\");
	res.first = path.substr(0, found);
	res.second = path.substr(found+1);
	return res;
}

std::pair<std::string, std::string> IO::SplitPathExt(std::string path)
{
	std::pair<std::string, std::string> res;
	size_t found;
	found = path.find_last_of(".");
	res.first = path.substr(0, found);
	res.second = path.substr(found+1);
	return res;
}

std::string IO::GetFilename(std::string path)
{
	std::string res = SplitPath(path).second;
	res = SplitPathExt(res).first;
	return res;
}

bool IO::Mkdir(const std::string path)
{
#ifdef _MSC_VER
	int mkDirRes = _mkdir(path.c_str());
#elif __WIN32__
	int mkDirRes = mkdir(path.c_str());
#else
	int mkDirRes = mkdir(path.c_str(), S_IRWXU | S_IRGRP | S_IROTH);
#endif

	if (0 == mkDirRes || (mkDirRes !=0 && EEXIST == errno)  ) {
		std::cout << "Create folder \"" << path<< "\"" << std::endl;
	}
	else {
		std::cout << "The folder may exist \"" << path<< "\"" << std::endl;
	};

	return mkDirRes == 0;
}

size_t IO::GetAvailMem()
{
	// get the physical memory
	size_t total_mem;
#if defined(__linux__)                  // linux
	long pages = sysconf(_SC_PHYS_PAGES);
	long page_size = sysconf(_SC_PAGE_SIZE);
	total_mem = pages * page_size;

#elif defined(__APPLE__)                // mac
	int mib[2] = { CTL_HW, HW_MEMSIZE };
	u_int namelen = sizeof(mib) / sizeof(mib[0]);
	size_t len = sizeof(total_mem);
	if (sysctl(mib, namelen, &total_mem, &len, NULL, 0) < 0) {
		assert(false && "IO::GetAvailMem() failed on MacOS");
		return 0;
	}

#else                                   // windows
	MEMORYSTATUSEX status;
	status.dwLength = sizeof(status);
	GlobalMemoryStatusEx(&status);
	total_mem = status.ullTotalPhys;
#endif
	// reserve some memory for system purpose
	return total_mem * 3/4;
}
}   // end of namespace tw
