//
// Created by puze on 02.12.20.
//

#include <iostream>
#include "iiwas_kinematics.h"
#include <chrono>

using namespace Eigen;
using namespace iiwas_kinematics;
using namespace std;

int main(int argc, char* argv[]){
    Kinematics kinematics = Kinematics();
    Kinematics::JointArrayType q;
    std::srand((unsigned int) time(0));

    Vector3d ee_pos;
    Quaterniond ee_quad;

    q.setRandom();
    auto start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        kinematics.forward_kinematics(q, ee_pos, ee_quad);
    }
    auto finish = chrono::high_resolution_clock::now();
    cout << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";
    cout << "Position: " << endl << ee_pos << endl;
    cout << "Rotation: " << endl << ee_quad.coeffs() << endl;
    cout << "Rotation Matrix: " << endl << ee_quad.matrix() << endl;
    return 0;
}