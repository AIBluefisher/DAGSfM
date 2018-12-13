/*
Copyright (c) 2018, Yu Chen
All rights reserved.
*/

#include <string>
#include <iostream>
#include <fstream>

#include "SfMAligner.hpp"

using namespace std;
using namespace i23dSFM;

#define USE_CAMERA
// #define USE_OBSERVATION

Vec3 c01(-0.58681815686003524, -1.4471659310155991, 2.7532201276039983);
Vec3 c02(-0.7564618423548185, -1.4008725163451317, 2.6573793692051422);
Vec3 c03(-0.92405029401987493, -1.3573365275942129, 2.5707425909775838);
Vec3 c04(-1.0450224651002122, -1.3089118380572615, 2.4759842635980536);
Vec3 c05(-1.1800414610569661, -1.2495757428974741, 2.3519360820169322);
Vec3 c06(-0.0074998390385831726, 0.01211374998305666, 0.016789060913904159);
Vec3 c08(0.51598435646719443, -0.0071268454137640002, 0.078756167469592631);
Vec3 c09(0.72690754008407943, -0.050583158520495553, 0.17497293792935878);
Vec3 c010(0.93050818837981475, -0.11506933491867609, 0.28764001853900611);

Vec3 c10(0.1433035828710533, -1.1495230926133868, 2.1632258707002534);
Vec3 c11(-0.0091111323884611045,  -1.1436124666589014, 2.1525736633742318);
Vec3 c12(-0.15685828114387518, -1.1391640988679768, 2.1475891553947548);
Vec3 c13(-0.27633899031905301, -1.124344075174553, 2.1222242928783261);
Vec3 c14(-0.41650543727345168, -1.1044861512669479, 2.0814348284434727);
Vec3 c111(-0.55822201073503075, -0.063296217282066219, 0.13227225012117569);
Vec3 c112(-0.18110109338123861, 0.0022517995770048871, 0.0065894207273441913);
Vec3 c113(-0.0010216440486818363, 0.0042722565109568865, 0.0041160969073334901);
Vec3 c114(0.18291841466735262, -0.0099613929308572345, 0.014606842091388067);

void TEST()
{
    // Simulate two point set, apply a known transformation and estimate it back:
    const int nbPoints = 10;
    Mat x1 = Mat::Random(3,nbPoints);
    Mat x2 = x1;

    x1.col(0) = c01; x1.col(1) = c02;  x1.col(2) = c03; x1.col(3) = c04; x1.col(4) = c05;
    x1.col(5) = c06; x1.col(6) = c08; x1.col(7) = c09; x1.col(8) = c010; 

    x2.col(0) = c10; x2.col(1) = c11;  x2.col(2) = c12; x2.col(3) = c13; x2.col(4) = c14;
    x2.col(5) = c111; x2.col(6) = c112; x2.col(7) = c113; x2.col(8) = c114; 

    // const double scale = 2.0;
    // const Mat3 rot = (Eigen::AngleAxis<double>(.2, Vec3::UnitX())
    // * Eigen::AngleAxis<double>(.3, Vec3::UnitY())
    // * Eigen::AngleAxis<double>(.6, Vec3::UnitZ())).toRotationMatrix();
    // const Vec3 t(0.5,-0.3,.38);

    // for (int i=0; i < nbPoints; ++i) {
    //     const Vec3 pt = x1.col(i);
    //     x2.col(i) = (scale * rot * pt + t);
    // }

    // Compute the Similarity transform
    double Sc;
    Mat3 Rc;
    Vec3 tc;
    i23dSFM::geometry::FindRTS(x1, x2, &Sc, &tc, &Rc);
    // Optional non linear refinement of the found parameters
     i23dSFM::geometry::Refine_RTS(x1,x2,&Sc,&tc,&Rc);

    std::cout << "\n"
    << "Scale " << Sc << "\n"
    << "Rot \n" << Rc << "\n"
    << "t " << tc.transpose();

    // std::cout << "\nGT\n"
    // << "Scale " << scale << "\n"
    // << "Rot \n" << rot << "\n"
    // << "t " << t.transpose();
}

int main(int argc, char* argv[])
{
    if(argc < 3) {
        cout << "Please input the file path of sfm_data and image path\n";
        return 0;
    }

    string sfm_data_path = argv[1];
    string img_path = argv[2];

    cout << "---------------- Structure from Motion Aligner --------------\n";
    SfMAligner sfm_aligner;
    if(!sfm_aligner.InitializeGraph(sfm_data_path)) {
        cout << "cluster graph cannot be initialized!\n";
        return 0;
    }

    sfm_aligner.GetClusterGraph().ShowInfo();

    sfm_aligner.UpdateGraph();

    // Merge clusters
    string dir = stlplus::folder_part(sfm_data_path);
    sfm_aligner.MergeClusters(dir, img_path);
}