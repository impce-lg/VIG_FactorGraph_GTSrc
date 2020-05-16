/**
                   _ooOoo_
                  o8888888o
                  88" . "88
                  (| -_- |)
                  O\  =  /O
               ____/`---'\____
             .'  \\|     |//  `.
            /  \\|||  :  |||//  \
           /  _||||| -:- |||||-  \
           |   | \\\  -  /// |   |
           | \_|  ''\---/''  |   |
           \  .-\__  `-`  ___/-. /
         ___`. .'  /--.--\  `. . __
      ."" '<  `.___\_<|>_/___.'  >'"".
     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
     \  \ `-.   \_ __\ /__ _/   .-` /  /
======`-.____`-.___\_____/___.-`____.-'======
                   `=---='
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
             佛祖保佑       永无BUG
*/
//
// Created by steve on 4/1/19.
//

/**
 * @file imuFactorsExample
 * @brief Test example for using GTSAM ImuFactor and ImuCombinedFactor
 * navigation code.
 * @author Garrett (ghemann@gmail.com), Luca Carlone
 */

/**
 * Example of use of the imuFactors (imuFactor and combinedImuFactor) in
 * conjunction with GPS
 *  - you can test imuFactor (resp. combinedImuFactor) by commenting (resp.
 * uncommenting) the line #define USE_COMBINED (few lines below)
 *  - we read IMU and GPS data from a CSV file, with the following format:
 *  A row starting with "i" is the first initial position formatted with
 *  N, E, D, qx, qY, qZ, qW, velN, velE, velD
 *  A row starting with "0" is an imu measurement
 *  linAccN, linAccE, linAccD, angVelN, angVelE, angVelD
 *  A row starting with "1" is a gps correction formatted with
 *  N, E, D, qX, qY, qZ, qW
 *  Note that for GPS correction, we're only using the position not the
 * rotation. The rotation is provided in the file for ground truth comparison.
 */

// GTSAM related includes.

#include <fstream>
#include <iostream>
#include <random>

#include "VIGSystem.h"

using namespace std;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << endl << "Usage: ./VIG_FactorGraph path_to_config_yaml" << endl;
    return 1;
  }
  string strConfigFilename = string(argv[1]);
  VIG::VIGSystem VIG(strConfigFilename);

  cout << "Processing..." << endl << endl;

  while (VIG.IsAvailable()) {
    VIG.PropagateIMU();
    VIG.CoupleOtherSensors();
    VIG.StepIMU();
  }

  VIG.OutputNavStates();

  VIG.Exit();

  return 0;
}
