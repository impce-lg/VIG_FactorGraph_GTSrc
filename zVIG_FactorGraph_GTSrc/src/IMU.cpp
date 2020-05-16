#include "IMU.h"

namespace VIG {

IMU::IMU(const Configuration *pConfig, ifstream &fIMU) {
  // Paras
  mIMUType = pConfig->mIMUType;
  mIMUMisalignMode = pConfig->mIMUMisalignMode;
  mIMUFileType = pConfig->mIMUFileType;
  mStartTime = pConfig->mIMUStartTime;

  // Grab intial IMU data
  if (!fIMU.eof()) {
    string strIMUHeader;
    string strPreIMULine, strCurIMULine, strNextIMULine;
    double t;

    getline(fIMU, strIMUHeader);  // skip Header

    if (!fIMU.eof()) getline(fIMU, strCurIMULine);
    if (!strCurIMULine.empty()) {
      strPreIMULine = strCurIMULine;

      {
        stringstream ss;
        ss << strCurIMULine;
        ss >> t;
      }

      while (t < mStartTime && !fIMU.eof()) {
        if (!strCurIMULine.empty()) {
          stringstream ss;
          ss << strCurIMULine;
          ss >> t;
        }

        strPreIMULine = strCurIMULine;
        getline(fIMU, strCurIMULine);
      }

      if (!strCurIMULine.empty()) {
        ExtrIMULine(strPreIMULine, msPreIMUData);
        ExtrIMULine(strCurIMULine, msCurIMUData);

        return;
      }
    }
  }
  msCurIMUData.T = DBL_MAX;
}

void IMU::ExtrIMULine(const string &strIMULine, IMUData &sIMUData) {
  stringstream ss;
  double t, ax, ay, az, wx, wy, wz;

  ss << strIMULine;
  ss >> t;
  ss >> ax;
  ss >> ay;
  ss >> az;
  ss >> wx;
  ss >> wy;
  ss >> wz;

  sIMUData.T = t;
  sIMUData.Acc << ax, ay, az;
  sIMUData.Gyro << wx, wy, wz;
}

void IMU::GrabNextIMUData(ifstream &fIMU) {
  msPreIMUData = msCurIMUData;

  if (msPreIMUData.T >= msCurIMUData.T && !fIMU.eof()) {
    string strCurIMULine;

    getline(fIMU, strCurIMULine);
    if (!strCurIMULine.empty()) {
      ExtrIMULine(strCurIMULine, msCurIMUData);
      return;
    }
  }
  msCurIMUData.T = DBL_MAX;
}

// void IMU::ModifyObs()
// {
//     double dt = msCurIMUData.T - msPreIMUData.T;
//     Vector3d Acc = msCurIMUData.Acc;
//     Vector3d Gyro = msCurIMUData.Gyro;

//     IMUErrorModel sIMUErrorModel = GetIMUErrorModel(mIMUType);
//     if (mbInvalidateIMUErrorStates)
//     {
//         for (int i = 0; i < 3; i++)
//         {
//             if (fabs(msIMUErrorStates.AccBias(i)) > 2 *
//             sIMUErrorModel.initacc_bias_err ||
//                 fabs(msIMUErrorStates.GyroBias(i)) > 2 *
//                 sIMUErrorModel.initgyro_bias_err)
//             {
//                 msIMUErrorStates.GyroBias.setZero();
//                 msIMUErrorStates.AccBias.setZero();
//                 msIMUErrorStates.GyroScale.setZero();
//                 msIMUErrorStates.AccScale.setZero();
//                 break;
//             }
//         }
//     }

//     Matrix3d Ascl = 1.0 / 1000 * msIMUErrorStates.AccScale.asDiagonal();
//     Matrix3d Gscl = 1.0 / 1000 * msIMUErrorStates.GyroScale.asDiagonal();

//     // rate
//     if (mIMUFileType == 0)
//     {
//         Acc = (Matrix3d::Identity() + Ascl).inverse() * (Acc -
//         msIMUErrorStates.AccBias);    // Modified Acceleration Gyro =
//         (Matrix3d::Identity() + Gscl).inverse() * (Gyro -
//         msIMUErrorStates.GyroBias); // Modified Angular Velocity mv3VelInc =
//         Acc * dt; mv3AngleInc = Gyro * dt;
//         //imu data accumulator for the covariance update
//         mIMUAccum.Vel += mv3VelInc;
//         mIMUAccum.Angle += mv3AngleInc;
//     }
//     // increment
//     else if (mIMUFileType == 1)
//     {
//         Acc = (Matrix3d::Identity() + Ascl).inverse() * (Acc -
//         msIMUErrorStates.AccBias * dt);    // Modified Velocity Gyro =
//         (Matrix3d::Identity() + Gscl).inverse() * (Gyro -
//         msIMUErrorStates.GyroBias * dt); // Modified Angle mv3VelInc = Acc;
//         mv3AngleInc = Gyro;
//         //imu data accumulator for the covariance update
//         mIMUAccum.Vel += mv3VelInc;
//         mIMUAccum.Angle += mv3AngleInc;
//     }
// }

// void IMU::MechanizeECEF(gtsam::NavState &sIMUPVQs2e)
// {
//     Vector3d XYZ = sIMUPVQs2e.P;
//     Vector3d Ve = sIMUPVQs2e.V;
//     Quaterniond Qbe = sIMUPVQs2e.Q;

//     double dt = msCurIMUData.T - msPreIMUData.T;
//     // Gravity (most time consuming part of ecef implementations)
//     Vector3d BLH = ECEF2Geo(XYZ);

//     Vector3d gn = CalgravityN(BLH);
//     Matrix3d Cen = BLH2DCM(BLH.head(2)); // Cne denotes Ce2n
//     // ge=Cn2e*g,g=[0,0,g],so only the last column of Cn2e is needed in
//     computing Vector3d ge = Cen.transpose() * gn - Vector3d(XYZ(0), XYZ(1),
//     0.0) * SQR(kOmega_WGS);

//     // Update attitude:Qbe_new=Qee*Qbe*Qbb;
//     // EulerAngles->RotationVector->RotationMatrix->Quaternion
//     Quaterniond Qbb = cfRotationVector2Quaterniond(mv3AngleInc);
//     Quaterniond Qee = cfRotationVector2Quaterniond(-kWie_e * dt);

//     Quaterniond tmpQVar, Qbe_new;
//     tmpQVar = Qbe * Qbb;
//     Qbe_new = Qee * tmpQVar;

//     // Update velocity
//     Vector3d VelInc1 = Qbe.toRotationMatrix() * mv3VelInc;
//     Vector3d VelInc2 = (ge + 2 * Ve.cross(kWie_e) + Vector3d(XYZ(0), XYZ(1),
//     0.0) * SQR(kOmega_WGS)) * dt; Vector3d Ve_new = Ve + VelInc1 + VelInc2;

//     // Update position
//     Vector3d XYZ_new = XYZ + Ve * dt;

//     sIMUPVQs2e.Q = Qbe_new;
//     sIMUPVQs2e.P = XYZ_new;
//     sIMUPVQs2e.V = Ve_new;
// }

// void IMU::ModelingIMUErrorStates(const double dt, Vector3d &Acc, Vector3d
// &Gyro, const int IMUType, const int IMUErrorModelID,
//                             Matrix<double, 12, 12> &Fd, Matrix<double, 12,
//                             12> &Qd, Matrix<double, 6, 12> &C, Matrix<double,
//                             6, 6> &R)
// {
//     switch (IMUErrorModelID)
//     {
//         case 1:
//         {
//             //Note that the following lines are model definition dependent.
//             Even if you use another imu model (for instance a model
//             //withtout any scale factor error), do not modify these lines.
//             Instead, just add a new "elseif" statement.
//             //I know this looks wierd. However, a completely modular
//             structure turns out to be impractible.
//             //Imu error states
//             //1-3:Acc bias, first order GM
//             //4-6:gyro bias, first order GM
//             //7-9:Acc scale (ppt=part per thousand not million). ppm def can
//             //sometimes cause numerical problems, random walk
//             //10-12:Gyro scale, random walk

//             //load continuous time parameters
//             IMUErrorModel sIMUErrorModel = GetIMUErrorModel(IMUType);

//             //Discrete time model of imu error states
//             double acc_bias_a = exp(-dt / sIMUErrorModel.acc_bias_Tc);
//             double acc_bias_w = sIMUErrorModel.acc_bias_Tc *
//             (sIMUErrorModel.acc_bias_Q) / 2 * (1 - SQR(acc_bias_a));
//             //(m/s^2)^2  (*s^2) double gyro_bias_a = exp(-dt /
//             sIMUErrorModel.gyro_bias_Tc); double gyro_bias_w =
//             sIMUErrorModel.gyro_bias_Tc * (sIMUErrorModel.gyro_bias_Q) / 2 *
//             (1 - SQR(gyro_bias_a)); //(rad/sec)^2 (*s^2) double acc_scale_a =
//             1; double acc_scale_w = sIMUErrorModel.acc_scale_Q * dt; //ppt^2
//             (*s^2) double gyro_scale_a = 1; double gyro_scale_w =
//             sIMUErrorModel.acc_scale_Q * dt; //(ppt)^2  (*s^2)

//             Matrix<double, 12, 1> vFd;
//             vFd << acc_bias_a, acc_bias_a, acc_bias_a, gyro_bias_a,
//             gyro_bias_a, gyro_bias_a, acc_scale_a, acc_scale_a, acc_scale_a,
//             gyro_scale_a, gyro_scale_a, gyro_scale_a; Fd = vFd.asDiagonal();
//             Matrix<double, 12, 1> vQd;
//             vQd << acc_bias_w, acc_bias_w, acc_bias_w, gyro_bias_w,
//             gyro_bias_w, gyro_bias_w, acc_scale_w, acc_scale_w, acc_scale_w,
//             gyro_scale_w, gyro_scale_w, gyro_scale_w; Qd = vQd.asDiagonal();

//             //Continuous time output model for the imu [acc;gyro]
//             Matrix<double, 6, 1> vIMUObs;
//             vIMUObs << Acc / 1000, Gyro / 1000;
//             Matrix<double, 6, 6> IMUObs = vIMUObs.asDiagonal();
//             C << Matrix<double, 6, 6>::Identity(), IMUObs;

//             Matrix<double, 6, 1> vR;
//             vR << sIMUErrorModel.acc_vrw, sIMUErrorModel.acc_vrw,
//             sIMUErrorModel.acc_vrw, sIMUErrorModel.gyro_arw,
//             sIMUErrorModel.gyro_arw, sIMUErrorModel.gyro_arw; R =
//             vR.asDiagonal();

//             break;
//         }
//         case 2: // turn on Acc/Gyro bias, random constant
//         {       // //Imu error states
//             // //1-3:Acc bias drift, first order GM process
//             // //4-6:gyro bias drift, first order GM process
//             // //7-9:Acc scale (ppt=part per thousand not million). ppm def
//             can
//             // //sometimes cause numerical problems, random walk
//             // //10-12:Gyro scale factor, random walk
//             // //13-15: Acc turn on bias, random constant
//             // //16-18:gyro turn on bias, random constant
//             // //load continuous time parameters
//             // sIMUErrorModel = GetIMUErrorModel(IMUType);

//             // //Discrete time model of imu error states
//             // double acc_bias_a = exp(-dt / sIMUErrorModel.acc_bias_Tc);
//             // double acc_bias_w = sIMUErrorModel.acc_bias_Tc *
//             (sIMUErrorModel.acc_bias_Q) / 2 * (1 -  SQR(acc_bias_a));
//             //(m/s^2)^2  (*s^2)
//             // double gyro_bias_a = exp(-dt / sIMUErrorModel.gyro_bias_Tc);
//             // double gyro_bias_w = sIMUErrorModel.gyro_bias_Tc *
//             (sIMUErrorModel.gyro_bias_Q) / 2 * (1 -  SQR(gyro_bias_a));
//             //(rad/sec)^2 (*s^2)
//             // double acc_scale_a = 1;
//             // double acc_scale_w = sIMUErrorModel.acc_scale_Q * dt; //ppt^2
//             (*s^2)
//             // double gyro_scale_a = 1;
//             // double gyro_scale_w = sIMUErrorModel.acc_scale_Q * dt;
//             //(ppt)^2  (*s^2)

//             // Matrix<double, 18, 1> vFd;
//             // vFd << acc_bias_a, acc_bias_a, acc_bias_a, gyro_bias_a,
//             gyro_bias_a, gyro_bias_a, acc_scale_a, acc_scale_a, acc_scale_a,
//             gyro_scale_a, gyro_scale_a, gyro_scale_a,
//             // MatrixXd::Ones(6,1);
//             // Fd = vFd.asDiagonal();
//             // Matrix<double, 18, 1> vQd;
//             // vQd << acc_bias_w, acc_bias_w, acc_bias_w, gyro_bias_w,
//             gyro_bias_w, gyro_bias_w, acc_scale_w, acc_scale_w, acc_scale_w,
//             gyro_scale_w, gyro_scale_w, gyro_scale_w,
//             // MatrixXd::Ones(6,1);
//             // Qd = vQd.asDiagonal();

//             // //Continuous time output model for the imu [acc;gyro]
//             // Matrix<double, 6, 1> vIMUObs;
//             // vIMUObs << Acc / 1000, Gyro / 1000;
//             // Matrix<double, 6, 6> IMUObs = vIMUObs.asDiagonal();
//             // Matrix<double, 6, 18> tmpC;
//             // tmpC << Matrix<double, 6, 6>::Identity(),
//             vIMUObs.asDiagonal(),Matrix<double, 6, 6>::Identity();
//             // C = tmpC;

//             // Matrix<double, 6, 1> vR;
//             // vR << sIMUErrorModel.acc_vrw, sIMUErrorModel.acc_vrw,
//             sIMUErrorModel.acc_vrw, sIMUErrorModel.gyro_arw,
//             sIMUErrorModel.gyro_arw, sIMUErrorModel.gyro_arw;
//             // R = vR.asDiagonal();

//             break;
//         }
//         case 3:
//         {
//             //  Imu error states
//             //1-3:Acc bias drift, random walk
//             //4-6:gyro bias drift, random walk
//             //7-9:Acc scale (ppt=part per thousand not million). ppm def can
//             //sometimes cause numerical problems, random walk
//             //10-12:Gyro scale factor, random walk
//             //load continuous time parameters
//             IMUErrorModel sIMUErrorModel = GetIMUErrorModel(IMUType);

//             //Discrete time model of imu error states
//             double acc_bias_a = 1;
//             double acc_bias_w = sIMUErrorModel.acc_bias_Q * dt; //(m/s^2)^2
//             (*s^2) double gyro_bias_a = 1; double gyro_bias_w =
//             sIMUErrorModel.gyro_bias_Q * dt; //(rad/sec)^2 (*s^2) double
//             acc_scale_a = 1; double acc_scale_w = sIMUErrorModel.acc_scale_Q
//             * dt; //ppt^2  (*s^2) double gyro_scale_a = 1; double
//             gyro_scale_w = sIMUErrorModel.acc_scale_Q * dt; //(ppt)^2  (*s^2)

//             Matrix<double, 12, 1> vFd;
//             vFd << acc_bias_a, acc_bias_a, acc_bias_a, gyro_bias_a,
//             gyro_bias_a, gyro_bias_a, acc_scale_a, acc_scale_a, acc_scale_a,
//             gyro_scale_a, gyro_scale_a, gyro_scale_a; Fd = vFd.asDiagonal();
//             Matrix<double, 12, 1> vQd;
//             vQd << acc_bias_w, acc_bias_w, acc_bias_w, gyro_bias_w,
//             gyro_bias_w, gyro_bias_w, acc_scale_w, acc_scale_w, acc_scale_w,
//             gyro_scale_w, gyro_scale_w, gyro_scale_w; Qd = vQd.asDiagonal();

//             //Continuous time output model for the imu [acc;gyro]
//             Matrix<double, 6, 1> vIMUObs;
//             vIMUObs << Acc / 1000, Gyro / 1000;
//             Matrix<double, 6, 6> IMUObs = vIMUObs.asDiagonal();
//             C << Matrix<double, 6, 6>::Identity(), IMUObs;

//             Matrix<double, 6, 1> vR;
//             vR << sIMUErrorModel.acc_vrw, sIMUErrorModel.acc_vrw,
//             sIMUErrorModel.acc_vrw, sIMUErrorModel.gyro_arw,
//             sIMUErrorModel.gyro_arw, sIMUErrorModel.gyro_arw; R =
//             vR.asDiagonal();

//             break;
//         }
//         case 4: // constant bias and constant scale factor for high end IMUs
//         {
//             //Imu error states
//             //1-3:Acc bias drift, random constant
//             //4-6:gyro bias drift, random constant
//             //7-9:Acc scale (ppt=part per thousand not million). ppm def can
//             //sometimes cause numerical problems, random constant
//             //10-12:Gyro scale factor, random constant

//             IMUErrorModel sIMUErrorModel = GetIMUErrorModel(IMUType);
//             //Discrete time model of imu error states
//             Fd.setIdentity();
//             Qd.setZero();
//             //Continuous time output model for the imu [acc;gyro]
//             Matrix<double, 6, 1> vIMUObs;
//             vIMUObs << Acc / 1000, Gyro / 1000;
//             Matrix<double, 6, 6> IMUObs = vIMUObs.asDiagonal();
//             C << Matrix<double, 6, 6>::Identity(), IMUObs;

//             Matrix<double, 6, 1> vR;
//             vR << sIMUErrorModel.acc_vrw, sIMUErrorModel.acc_vrw,
//             sIMUErrorModel.acc_vrw, sIMUErrorModel.gyro_arw,
//             sIMUErrorModel.gyro_arw, sIMUErrorModel.gyro_arw; R =
//             vR.asDiagonal();

//             break;
//         }
//         case 5:
//         {
//             //  Imu error states
//             //1-3:Acc bias drift, random walk
//             //4-6:gyro bias drift, random constant
//             //7-9:Acc scale (ppt=part per thousand not million). ppm def can
//             //sometimes cause numerical problems, random constant
//             //10-12:Gyro scale factor, random constant
//             //load continuous time parameters
//             IMUErrorModel sIMUErrorModel = GetIMUErrorModel(IMUType);

//             //Discrete time model of imu error states
//             double acc_bias_a = 1;
//             double acc_bias_w = sIMUErrorModel.acc_bias_Q * dt; //(m/s^2)^2
//             (*s^2) double gyro_bias_a = 1; double gyro_bias_w = 0;
//             //(rad/sec)^2 (*s^2) double acc_scale_a = 1; double acc_scale_w =
//             0; //ppt^2  (*s^2) double gyro_scale_a = 1; double gyro_scale_w =
//             0; //(ppt)^2  (*s^2)

//             Matrix<double, 12, 1> vFd;
//             vFd << acc_bias_a, acc_bias_a, acc_bias_a, gyro_bias_a,
//             gyro_bias_a, gyro_bias_a, acc_scale_a, acc_scale_a, acc_scale_a,
//             gyro_scale_a, gyro_scale_a, gyro_scale_a; Fd = vFd.asDiagonal();
//             Matrix<double, 12, 1> vQd;
//             vQd << acc_bias_w, acc_bias_w, acc_bias_w, gyro_bias_w,
//             gyro_bias_w, gyro_bias_w, acc_scale_w, acc_scale_w, acc_scale_w,
//             gyro_scale_w, gyro_scale_w, gyro_scale_w; Qd = vQd.asDiagonal();

//             //Continuous time output model for the imu [acc;gyro]
//             Matrix<double, 6, 1> vIMUObs;
//             vIMUObs << Acc / 1000, Gyro / 1000;
//             Matrix<double, 6, 6> IMUObs = vIMUObs.asDiagonal();
//             C << Matrix<double, 6, 6>::Identity(), IMUObs;

//             Matrix<double, 6, 1> vR;
//             vR << sIMUErrorModel.acc_vrw, sIMUErrorModel.acc_vrw,
//             sIMUErrorModel.acc_vrw, sIMUErrorModel.gyro_arw,
//             sIMUErrorModel.gyro_arw, sIMUErrorModel.gyro_arw; R =
//             vR.asDiagonal();

//             break;
//         }
//         default:
//         {
//             cout << "This is an undefined error model.";
//         }
//     }
// }

// void IMU::TransferDynamicsECEF(Vector3d &XYZ, Matrix3d &Cse, IMU *pIMU,
// MatrixXd &Fai, MatrixXd &Qd)
// {
//     // for modelNo=5 and imutype=5
//     // position   (1-3)
//     // velocity   (4-6)
//     // attitude   (7-9)
//     // acc bias drift
//     // gyro bias drift
//     // acc scale factor
//     // gyro scale factor
//     // acc turn on bias
//     // gyro turn on bias
//     // for modelNo=1 and imutype=5 the acc and gyro turn on bias are removed
//     //system disturbance coefs
//     double dt = msCurIMUData.T - mCovUpdtTime;
//     Vector3d AccAvg = mIMUAccum.Vel / dt;
//     Vector3d GyroAvg = mIMUAccum.Angle / dt;

//     Matrix<double, 9, 6> G, Gnav;
//     G.setZero();
//     G.bottomRightCorner<3, 3>() = -Cse; //attitude
//     G.block<3, 3>(3, 0) = Cse;          //velocity

//     //system matrix
//     Matrix<double, 9, 9> F, Fnav;
//     F.setZero();
//     Vector3d Acc_e = Cse * AccAvg;
//     //Position
//     F.block<3, 3>(0, 3).setIdentity();
//     //Velocity
//     Vector3d UnitVec = XYZ / XYZ.norm();
//     F.block<3, 3>(3, 0) = -kGe * SQR(kRavg) / pow(XYZ.norm(),3) *
//     (Matrix3d::Identity() - 3 * (UnitVec * UnitVec.transpose())) -
//     Skew(kWie_e) * Skew(kWie_e); F.block<3, 3>(3, 3) = -2 * Skew(kWie_e);
//     F.block<3, 3>(3, 6) = Skew(Acc_e);
//     //Attitude
//     F.bottomRightCorner<3, 3>() = -Skew(kWie_e);
//     Gnav = G;
//     Fnav = F;
//     // cout << "Gnav:" << endl
//     //      << Gnav << endl;
//     // cout << "Fnav:" << endl
//     //      << Fnav << endl;

//     // Imu error model parameters
//     Matrix<double, 12, 12> Fimu_d, Qimu_d;
//     Matrix<double, 6, 12> Cimu;
//     Matrix<double, 6, 6> Rimu;
//     ModelingIMUErrorStates(dt, AccAvg, GyroAvg, mIMUType, mIMUErrorModelID,
//     Fimu_d, Qimu_d, Cimu, Rimu);

//     // Combine and discretize nav and imu models
//     // this discretization can also be accomplished by Loans matrix
//     exponential
//     // method, see sys_metric_phipsi_v000.m
//     Matrix<double, 9, 9> Fnav_d = Matrix<double, 9, 9>::Identity() + dt *
//     Fnav; //Use 1st order taylor series to discretize Fnav Matrix<double, 9,
//     9> Qnav = Gnav * Rimu * Gnav.transpose(); Matrix<double, 9, 9> Qnav_d =
//     dt / 2.0 * (Fnav_d * Qnav + Qnav * Fnav_d.transpose()); // Use
//     trapezoidal rule to discretize Rimu

//     Fai = Matrix<double, 21, 21>::Zero();
//     Fai.topLeftCorner<9, 9>() = Fnav_d;
//     Fai.topRightCorner<9, 12>() = Gnav * Cimu * dt;
//     Fai.bottomRightCorner<12, 12>() = Fimu_d;

//     Qd = Matrix<double, 21, 21>::Zero();
//     Qd.topLeftCorner<9, 9>() = Qnav_d;
//     Qd.topRightCorner<9, 12>() = Gnav * Cimu * Qimu_d * dt / 2;
//     Qd.bottomLeftCorner<12, 9>() = Qd.topRightCorner<9, 12>().transpose();
//     Qd.bottomRightCorner<12, 12>() = Qimu_d;

//     // cout << "Fai:" << endl
//     //      << Fai << endl;
//     // cout << "Qd:" << endl
//     //      << Qd << endl;
// }

// void IMU::TransferDynamicsLocal(gtsam::Pose3 &sPoses02e, gtsam::NavState
// &sSensorPVQs2s0, IMU *pIMU, MatrixXd &Fai, MatrixXd &Qd)
// {
//     double dt = msCurIMUData.T - mCovUpdtTime;
//     Vector3d AccAvg = mIMUAccum.Vel / dt;
//     Vector3d GyroAvg = mIMUAccum.Angle / dt;

//     Matrix<double, 15, 6> G, Gnav;
//     G.setZero();
//     G.bottomRightCorner<3, 3>().setIdentity(); //attitude
//     Matrix3d Cs02s = sSensorPVQs2s0.Q.toRotationMatrix();
//     G.block<3, 3>(9, 0) = Cs02s.transpose(); //velocity

//     //system matrix
//     Matrix<double, 15, 15> F, Fnav;
//     F.setZero();
//     // 0s for r s0 in e and q s0 to e
//     F.block<3, 3>(6, 9).setIdentity(); // rs in s0
//     //Velocity
//     Vector3d XYZ = sPoses02e.T +
//     sPoses02e.Q.toRotationMatrix()*sSensorPVQs2s0.P; Vector3d UnitVec = XYZ /
//     XYZ.norm(); Vector3d BLH = ECEF2Geo(XYZ); Vector3d gn = CalgravityN(BLH);
//     Matrix3d geCoeff = -gn(2) * SQR(kRavg) / pow(XYZ.norm(),3) *
//     (Matrix3d::Identity() - 3 * (UnitVec * UnitVec.transpose())) -
//     Skew(kWie_e) * Skew(kWie_e); Matrix3d Cs02e =
//     sPoses02e.Q.toRotationMatrix(); F.block<3, 3>(9, 0) = Cs02e.transpose() *
//     geCoeff; F.block<3, 3>(9, 6) = Cs02e.transpose() * geCoeff * Cs02e;

//     Matrix3d Cen = BLH2DCM(BLH.head(2)); // Cne denotes Ce2n
//     // ge=Cn2e*g,g=[0,0,g],so only the last column of Cn2e is needed in
//     computing Vector3d ge = Cen.transpose() * gn - Vector3d(XYZ(0), XYZ(1),
//     0.0) * SQR(kOmega_WGS); Vector3d Vs2e = sPoses02e.Q.toRotationMatrix() *
//     sSensorPVQs2s0.V; F.block<3, 3>(9, 3) = Cs02e.transpose() * (-Skew(ge) +
//     geCoeff * Skew(sPoses02e.Q.toRotationMatrix() * sSensorPVQs2s0.P) +
//     Skew(Skew(kWie_e) * Skew(kWie_e) * XYZ) - 2 * Skew(Vs2e) * Skew(kWie_e));
//     F.block<3, 3>(9, 9) = -2 * Skew(sPoses02e.Q.toRotationMatrix() * kWie_e);
//     F.block<3, 3>(9, 12) = -Cs02s.transpose() * Skew(AccAvg);
//     //Attitude
//     F.block<3, 3>(12, 3) = Cs02s * Cs02e.transpose() * Skew(kWie_e);
//     F.bottomRightCorner<3, 3>() = Skew(-GyroAvg);
//     Fnav = F;
//     Gnav = G;
//     // X(k+1) = ffun[X(k),U(k),V(k)]
//     // X(k+1) = Fk*X(k)+Gk*Vk

//     // Imu error model parameters
//     Matrix<double, 12, 12> Fimu_d, Qimu_d;
//     Matrix<double, 6, 12> Cimu;
//     Matrix<double, 6, 6> Rimu;
//     ModelingIMUErrorStates(dt, AccAvg, GyroAvg, mIMUType, mIMUErrorModelID,
//     Fimu_d, Qimu_d, Cimu, Rimu);

//     // Combine and discretize nav and imu models
//     // this discretization can also be accomplished by Loan.transpose()s
//     matrix exponential method, see sys_metric_phipsi_v000.m
//     // int navStates = 15,
//     sPoses02e.Q,sPoses02e.T,sSensorPVQs2s0.P,sSensorPVQs2s0.V,sSensorPVQs2s0.Q
//     Matrix<double, 15, 15> Fnav_d = Matrix<double, 15, 15>::Identity() + dt *
//     Fnav; //Use 1st order taylor series to discretize Fnav Matrix<double, 15,
//     15> Qnav = Gnav * Rimu * Gnav.transpose(); Matrix<double, 15, 15> Qnav_d
//     = dt / 2.0 * (Fnav_d * Qnav + Qnav * Fnav_d.transpose()); // Use
//     trapezoidal rule to discretize Rimu

//     Fai = Matrix<double, 27, 27>::Zero();
//     Fai.topLeftCorner<15, 15>() = Fnav_d;
//     Fai.topRightCorner<15, 12>() = Gnav * Cimu * dt;
//     Fai.bottomRightCorner<12, 12>() = Fimu_d;

//     Qd = Matrix<double, 27, 27>::Zero();
//     Qd.topLeftCorner<15, 15>() = Qnav_d;
//     Qd.topRightCorner<15, 12>() = Gnav * Cimu * Qimu_d * dt / 2;
//     Qd.bottomLeftCorner<12, 15>() = Qd.topRightCorner<15, 12>().transpose();
//     Qd.bottomRightCorner<12, 12>() = Qimu_d;

//     // cout << "Fai:" << endl
//     //      << Fai << endl;
//     // cout << "Qd:" << endl
//     //      << Qd << endl;
// }

}  // namespace VIG
