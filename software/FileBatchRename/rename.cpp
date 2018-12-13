#include <iostream>
#include <string>

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

using namespace std;

int main(int argc, char* argv[])
{
    string input_dir = argv[1];
    string new_dir = input_dir + "_merged";
    cout << new_dir << endl;
    if (!stlplus::folder_create(new_dir)) {
        cerr << "folder " << new_dir << " cannot be created!\n";
        return false;
    }

    // extracts subfolder names of all the subdirectories found in input_dir
    vector<string> subdirectories = stlplus::folder_subdirectories(input_dir);
    size_t k = 0;
    cout << subdirectories.size() << endl;
    for (auto subdirectory : subdirectories) {
        auto sd_path = input_dir + "/" + subdirectory;
        cout << "sub directory: " << sd_path << endl;
        vector<string> files = stlplus::folder_files(sd_path);
        for (auto file : files) {
            // file is image
            string extension = stlplus::extension_part(file);
            if (extension == "png" || extension == "jpg" || extension == "JPG" ||
                extension == "jpeg" || extension == "JPEG") {
                string filename = stlplus::basename_part(file);
                string new_filename = to_string(k) + "." + extension;
                if (!stlplus::file_copy(sd_path + "/" + file, new_dir + "/" + new_filename)) {
                    cout << "cannot copy "
                         << sd_path + "/" + file << " to "
                         << new_dir + "/" + new_filename << "!\n";
                }
                k++;
            }
        }
    }
}
