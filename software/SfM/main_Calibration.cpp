//
// Created by haibao637 on 17-11-22.
//
#include "third_party/cmdLine/cmdLine.h"
#include <i23dSFM/sfm/sfm.hpp>
#include <i23dSFM/types.hpp>
#include<fstream>
#include<iterator>
#include"i23dSFM/sfm/pipelines/localization/SfM_Localizer.hpp"
using namespace std;
using namespace i23dSFM;
class Observation{
public:

    Vec3 vec3;
    Vec2 vec2;
    friend istream& operator>>(istream& in,Observation& ob){
        in>>ob.vec3[0]>>ob.vec3[1]>>ob.vec3[2];
        in>>ob.vec2[0]>>ob.vec2[1];
        return in;
    }
};
int main(int argc,char** argv){
    CmdLine cmd;
    string filename;
    string sOutDir;
    cmd.add( make_option('i', filename, "input_file") );
    cmd.add( make_option('o', sOutDir, "out_dir") );
    cmd.process(argc,argv);
    Pair img_size;
    string img_name;

    ifstream fin(filename);
    fin>>img_name;
    fin>>img_size.first>>img_size.second;
    vector<Observation> obs;
    copy(istream_iterator<Observation>(fin),istream_iterator<Observation>(),back_inserter(obs));
    sfm::Image_Localizer_Match_Data matching_data;
    matching_data.pt3D.resize(3,obs.size());
    matching_data.pt2D.resize(2,obs.size());

    for(IndexT cpt=0;cpt<obs.size();++cpt){
        matching_data.pt3D.col(cpt)=obs[cpt].vec3;
        matching_data.pt2D.col(cpt)=obs[cpt].vec2;
    }

    geometry::Pose3 pose;
    std::shared_ptr<cameras::IntrinsicBase> optional_intrinsic (nullptr);
    const bool bResection =  sfm::SfM_Localizer::Localize(img_size, optional_intrinsic.get(), matching_data, pose);
    if(bResection){
        const std::string out_file_name = stlplus::create_filespec(sOutDir,  stlplus::basename_part(img_name), "txt");
        ofstream fout(out_file_name);
        Mat3 K,R;
        Vec3 t;
        KRt_From_P(matching_data.projection_matrix, &K, &R, &t);

        fout<<setiosflags(ios::fixed);
        fout<<img_name<<"\n"<<K<<endl<<pose.rotation()<<endl<<pose.center().transpose()<<endl;
        fout.close();
//        fout<<img_name<<"\r\n"<<img_size.first<<" "<<img_size.second<<"\r\n" <<K.row(0)<<"\r\n"<<K.row(1)<<"\r\n"<<K.row(2)<<"\r\n" <<R.col(0).transpose()<<"\r\n"<<R.col(1).transpose()<<"\r\n"<<R.col(2).transpose()<<"\r\n" <<t.transpose()<<"\r\n";
    }



}