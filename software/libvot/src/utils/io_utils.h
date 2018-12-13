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

/*! \file io_utils.h
 * \brief I/O utilities
 *
 * This file contains some utility classes and functions to facilitate file read/write.
 */

#ifndef VOT_IO_UTILS_H
#define VOT_IO_UTILS_H

#include <iostream>
#include <vector>

namespace tw 
{
    /**
     *  allocate a continuous 2d array
     */
    template<typename T> T* allocate_2d(T ***arr, int row, int col);

    /**
     *  free a continuous 2d array, both the raw pointer to the data
     *  and the 2d array pointers are needed.
     */
    template<typename T> void free_2d(T ***array, T *data);

    /** @brief To safely read and write files
     *
     * IO class contains a set of utility function for dealing with file read/write. It also has some utility function
     * related to operating system, such as getting the maximum available memory.
     */
    class IO
    {
    private:
        IO() {}
    public:
        template <class charType>
        static void TrimLeft(std::basic_string<charType> & str, const char *chars2remove = NULL);   //!< trim the left of a string
        template <class charType>
        static void TrimRight(std::basic_string<charType> & str, const char *chars2remove = NULL);  //!< trim the right of a string
        template <class charType>
        static void Trim(std::basic_string<charType> & str, const char *chars2remove = NULL);       // it consists TrimLeft and TrimRight
        static bool IsEmptyString(const std::string & str);                                     // judge whether the string is empty
        static int ExtractLines(const char *input_file_path, std::vector<std::string> & lines); //!< extract lines from a text file
        static bool IsFileExist(const char *filename);				//!< judge whether the file exists
		static bool IsFileExist(const std::string& filename);		//!< judge whether the file exists
        static std::string JoinPath(std::string folder, std::string filename);  //!< join a filename with a directory name

        static std::pair<std::string, std::string> SplitPath(std::string path);		//!< separate the folder from the filename
        static std::pair<std::string, std::string> SplitPathExt(std::string path); 	//!< separate the basename from the file type (extension)
		static std::string GetFilename(std::string path);		//!< get filename without folder path and suffix file format

		static bool Mkdir(const std::string path);      //!< make a directory
        static size_t GetAvailMem();                    //!< return the total availbale memory that can be used 
    };

	/**
	 * @brief PrintStringVector: a handy debugging function to print first 'num' element of a vector.
	 * @param strings: the vector to be printed.
	 * @param num: the number of elements to be printed.
	 */
	template <class TYPE>
	void PrintVector(std::vector<TYPE> &vec, int num)
	{
		int print_lines = vec.size() > num ? num : vec.size();
		for (int i = 0; i < print_lines; i++) {
			std::cout << vec[i] << std::endl;
		}
		return;
	}

}   // end of namespace tw

#endif  // VOT_IO_UTILS_H
