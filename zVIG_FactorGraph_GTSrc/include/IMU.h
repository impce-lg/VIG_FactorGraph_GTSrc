#pragma once

#include "Configuration.h"
#include "VIGCommon.h"

namespace VIG {

// void ModifyIMUObs(const IMUData &ImuData, double dt, const int IMUType,
//                   Vector3d &VelInc, Vector3d AngleInc, IMUAccumlation
//                   &IMUAccum);
class Configuration;

class IMU {
 public:
  IMU(const Configuration *pConfig, ifstream &fIMU);
  void ExtrIMULine(const string &strIMULine, IMUData &sIMUData);
  void GrabNextIMUData(ifstream &fIMU);
  void ModifyObs();
  void MechanizeECEF(gtsam::NavState &sIMUPVQs2e);
  void TransferDynamicsECEF(Vector3d &XYZ, Matrix3d &Cse, IMU *pIMU,
                            MatrixXd &Fai, MatrixXd &Qd);
  void TransferDynamicsLocal(gtsam::Pose3 &sPoses02e,
                             gtsam::NavState &sSensorPVQs2s0, IMU *pIMU,
                             MatrixXd &Fai, MatrixXd &Qd);
  void ModelingIMUErrorStates(const double dt, Vector3d &Acc, Vector3d &Gyro,
                              const int IMUType, const int IMUErrorModelID,
                              Matrix<double, 12, 12> &Fd,
                              Matrix<double, 12, 12> &Qd,
                              Matrix<double, 6, 12> &C,
                              Matrix<double, 6, 6> &R);

 public:
  int mIMUType;  // imu type name
  // double mDt;           // sampling interval
  int mIMUMisalignMode;  // 0:e-formulation,1:phi,2:psi,3:s0 local frame
  int mIMUFileType;      // imu file type, 0:rate,1:increment
  double mStartTime, mCovUpdtTime;

  IMUData msPreIMUData, msCurIMUData, msNextIMUData;
  IMUData msInterpIMUData;

  Vector3d mv3VelInc;    // velocity increment in a sampling interval
  Vector3d mv3AngleInc;  // angle increment in a sampling interval
  IMUAccumlation mIMUAccum;
};
}  // namespace VIG
