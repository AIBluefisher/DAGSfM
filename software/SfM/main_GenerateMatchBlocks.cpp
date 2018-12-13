// Created by Bluefish
// Date: 2018-09-01

// This algorithm is used to generate match blocks,
// where each match block is a subset of the exhaustive matches.
// We split the orignal matches into some small blocks, and then,
// matching is performed inner each block, each matched block
// then merged into a new match block until one match block is preserved. 

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "i23dSFM/sfm/sfm.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

using namespace std;
using namespace i23dSFM;
using namespace i23dSFM::sfm;

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Usage: i23dSFM_main_GenerateMatchBlocks sfm_data_filename max_image_num\n";
        return false;
    }

    string sfmdata_filename = argv[1];
    size_t max_image_num = atoi(argv[2]);

    ofstream out_blocks(stlplus::folder_part(sfmdata_filename) + "/blocks_path.txt");
    if (!out_blocks.is_open()) {
        cerr << "The file of out_blocks cannot be read\n";
        return false;
    }

    // Generate matches file list
    int part_num = 0;
    ofstream stream(stlplus::folder_part(sfmdata_filename) + "/matches_list.txt");
    if (!stream.is_open()) {
        cerr << "matches list file cannot be opened!\n";
        return false;
    }

    // Load input scene
    SfM_Data sfm_data;
    if (!Load(sfm_data, sfmdata_filename, ESfM_Data(VIEWS|INTRINSICS))) {
        cerr << "\nThe input file \"" << sfmdata_filename << "\" cannot be read\n";
        return false;
    }

    vector<size_t> vec_viewid;
    Views views = sfm_data.views;

    // Get all the view's id
    for (auto ite = views.begin(); ite != views.end(); ite++) {
        vec_viewid.push_back(ite->second->id_view);
    }

    size_t blocks_num = vec_viewid.size() / max_image_num + 1;
    vector<vector<size_t>> matches_blocks;
    int k = 0;
    for (int i = 0; i < blocks_num; i++) {
        vector<size_t> matches_block;
        for (int j = 0; j < max_image_num && k < vec_viewid.size(); j++) {
            matches_block.push_back(k++);
        }
        matches_blocks.push_back(matches_block);
    }

    // full block matches for testing
    // string filename = stlplus::folder_part(sfmdata_filename) + "/full_matches_pair.txt";
    // ofstream full_block(filename.c_str());
    // out_blocks << filename << endl;
    // stream << stlplus::folder_part(sfmdata_filename) + "/" + 
    //           to_string(part_num++) +
    //           "/matches.f.txt"; 
    // for (int i = 0; i < vec_viewid.size() - 1; i++) {
    //     full_block << i << " ";
    //     for (int j = i + 1; j < vec_viewid.size(); j++) {
    //         full_block << j << " ";
    //     }
    //     full_block << "\n";
    // }
    // full_block.close();

    // inner block match pairs
    k = 0;
    for (; k < matches_blocks.size(); k++) {
        auto matches_block = matches_blocks[k];
        string block_filename = stlplus::folder_part(sfmdata_filename) + 
                                "/block" + to_string(k) + "_matches_pair.txt";
        ofstream out(block_filename.c_str());
        out_blocks << block_filename << endl;
        stream << stlplus::folder_part(sfmdata_filename) + "/" + 
                  to_string(part_num++) + "/matches.f.txt"
               << endl;

        if(!out.is_open()) {
            cerr << "The file \"" << block_filename << "\" cannot be read\n";
            return false;
        }
        for (int i = 0; i < matches_block.size() - 1; i++) {
            out << matches_block[i] << " ";
            for (int j = i + 1; j < matches_block.size(); j++) {
                out << matches_block[j] << " ";
            }
            out << endl;
        }
        out.close();
    }

    // inter block match pairs
    for (int i = 0; i < matches_blocks.size(); i++) {
        for (int j = i + 1; j < matches_blocks.size(); j++) {
            string block_filename = stlplus::folder_part(sfmdata_filename) + 
                                    "/block" + to_string(i) + "_" + 
                                    to_string(j) + "_matches_pair.txt";
            ofstream out(block_filename.c_str());
            out_blocks << block_filename << endl;
            stream << stlplus::folder_part(sfmdata_filename) + "/" + 
                      to_string(part_num++) + "/matches.f.txt"
                   << endl;
            if(!out.is_open()) {
                cerr << "The file \"" << block_filename << "\" cannot be read\n";
                return false;
            }

            for (int l = 0; l < matches_blocks[i].size(); l++) {
                out << matches_blocks[i][l] << " ";
                for (int r = 0; r < matches_blocks[j].size(); r++) {
                    out << matches_blocks[j][r] << " ";
                }
                out << endl;
            }
            out.close();
        } 
    }

    out_blocks.close();
    stream.close();

}