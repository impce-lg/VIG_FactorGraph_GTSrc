#include "VIGCommon.h"

namespace VIG {
string gstrNavFrame = "NED";            // default NED
Vector3d gG = Vector3d(0., 0., -9.81);  // default gravity value
bool gbUseGNSS = false;                 // default false
bool gbUseCam = false;                  // default false

double gFocalLength = 0.;
Matrix3d gCamK;
Eigen::Matrix<double, 5, 1> gCamDistCoef;
int gNumUnDisIter;
int gNRow;
int gNCol;
double gInitDepth = 5.0;
int gKeyFramesQueueSize, gMaxFrameIdInKFQue, gCoVisualFramesThreshold;

Vector3d gIMUBaseBLH;
gtsam::Pose3 gExtrPose_i2g;
gtsam::Pose3 gExtrPose_c2i;

double gFirstPubFrameEpoch;
bool gbFirstFrame = true;  // the frame id w.r.t the video

eVIGEstimatorState geVIGEstimatorState;

// Test params
timespec gTestTime1, gTestTime2;
// int gTestFramesThreshold;

IMUErrorModel4PreIntegration GetIMUErrorModel4PreIntegration(
    const int IMUType) {
  double AccSigma;
  double GyroSigma;
  double IntegCov;
  double BiasAccRWSigma;
  double BiasOmegaRWSigma;
  double BiasAccOmegaInt;

  IMUErrorModel4PreIntegration sIMUErrorModel4PreIntegration;

  switch (IMUType) {
    case 5: {                            // OXTS RT 3003
      AccSigma = 0.04 * 1.0e-2;          // 2 ug
      GyroSigma = 18 * kDeg2Rad / 3600;  // 2 °/hr
      IntegCov = 1e-8;
      BiasAccRWSigma = 80 * 1.0e-5;        // 80 ug with 1 Hz
      BiasOmegaRWSigma = 0.03 * kDeg2Rad;  // 0.03 deg/sec/sqrt(Hz)
      BiasAccOmegaInt = 1e-5;
      break;
    }
    case 8: {                           // OXTS RT 3003
      AccSigma = 2 * 9.8 * 1e-6;        // 2 ug
      GyroSigma = 2 * kDeg2Rad / 3600;  // 2 °/hr
      IntegCov = 1e-8;
      BiasAccRWSigma = 0.005 / 60;             // 0.005 m/s/sqrt(hr)
      BiasOmegaRWSigma = 0.2 * kDeg2Rad / 60;  // 0.2 °/sqrt(hr)
      BiasAccOmegaInt = 1e-5;
      break;
    }
    case 9: {  // gtsam example
               // double accel_noise_sigma = 0.03924;
               // double gyro_noise_sigma = 0.0205689024915;
               // double accel_bias_rw_sigma = 0.04905;
               // double gyro_bias_rw_sigma = 0.0001454441043;
    }

    default: {
      cout << "Error IMU type!" << endl;
    }
  }

  sIMUErrorModel4PreIntegration.accelerometerCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(AccSigma, 2);
  sIMUErrorModel4PreIntegration.gyroscopeCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(GyroSigma, 2);
  sIMUErrorModel4PreIntegration.integrationCovariance =
      gtsam::Matrix33::Identity(3, 3) *
      IntegCov;  // error committed in integrating position from
                 // velocities
  sIMUErrorModel4PreIntegration.biasAccCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(BiasAccRWSigma, 2);
  sIMUErrorModel4PreIntegration.biasOmegaCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(BiasOmegaRWSigma, 2);
  sIMUErrorModel4PreIntegration.biasAccOmegaInt =
      gtsam::Matrix::Identity(6, 6) *
      BiasAccOmegaInt;  // error in the bias used for preintegration

  return sIMUErrorModel4PreIntegration;
}

Matrix3d Skew(const Vector3d &vec3d) {
  Matrix3d SkewMat;
  SkewMat << 0.0, -vec3d(2), vec3d(1), vec3d(2), 0.0, -vec3d(0), -vec3d(1),
      vec3d(0), 0.0;

  return SkewMat;
}

Matrix3d R1(double XrotRad) {
  Matrix3d R1;
  R1 << 1, 0, 0, 0, cos(XrotRad), sin(XrotRad), 0, -sin(XrotRad), cos(XrotRad);

  return R1;
}

Matrix3d R2(double YrotRad) {
  Matrix3d R2;
  R2 << cos(YrotRad), 0, -sin(YrotRad), 0, 1, 0, sin(YrotRad), 0, cos(YrotRad);

  return R2;
}

Matrix3d R3(double ZrotRad) {
  Matrix3d R3;
  R3 << cos(ZrotRad), sin(ZrotRad), 0, -sin(ZrotRad), cos(ZrotRad), 0, 0, 0, 1;

  return R3;
}

void RemoveAttAnglesRedundancy(Vector3d &AttRad) {
  // There is some reduncancy in euler angle values((i) is not supported yet):
  // (i)  If m(1)==m(3) then e=[a b c] and e=[a+-pi -b c+-pi] are equivalent.
  //   The output of this routine will always have b>=0;
  // (ii) If m(1)~=m(3) then e=[a b c] and e=[a+-pi pi-b c+-pi] are equivalent.
  //   The output of this routine will always have |b|<=pi/2
  if (fabs(AttRad(1)) > M_PI / 2) {
    AttRad(0) -= (2 * (AttRad(0) > 0) - 1) * M_PI;
    AttRad(1) = (2 * (AttRad(1) > 0) - 1) * M_PI - AttRad(1);
    AttRad(2) -= (2 * (AttRad(2) > 0) - 1) * M_PI;
  }
}

Quaterniond cfRotationVector2Quaterniond(const Vector3d &RotationVector) {
  double S = 0.0;
  S = sqrt(pow(RotationVector[0], 2) + pow(RotationVector[1], 2) +
           pow(RotationVector[2], 2));
  if (S == 0) {
    return Quaterniond::Identity();
  }

  Quaterniond Q;
  Q.w() = cos(0.5 * S);
  Q.x() = sin(0.5 * S) / (0.5 * S) * 0.5 * RotationVector[0];
  Q.y() = sin(0.5 * S) / (0.5 * S) * 0.5 * RotationVector[1];
  Q.z() = sin(0.5 * S) / (0.5 * S) * 0.5 * RotationVector[2];
  Q.normalize();

  return Q;
}

Vector3d CalgravityN(const Vector3d &BLH) {
  Vector3d gn;
  gn.setZero();

  double m = SQR(kOmega_WGS) * kA_84 * kA_84 * kB_84 / kGM_Earth;
  double g0 = (kA_84 * kTa * cos(BLH(0)) * cos(BLH(0)) +
               kB_84 * kTb * sin(BLH(0)) * sin(BLH(0))) /
              sqrt(kA_84 * kA_84 * cos(BLH(0)) * cos(BLH(0)) +
                   kB_84 * kB_84 * sin(BLH(0)) * sin(BLH(0)));
  gn(2) =
      g0 *
      (1 -
       2 * (1 + kFlattening + m - 2 * kFlattening * sin(BLH(0)) * sin(BLH(0))) *
           BLH(2) / kA_84 +
       3 * BLH(2) * BLH(2) / kA_84 / kA_84);

  return gn;
}

Matrix3d LeverArmRot(const Vector3d &BLH) {
  double slat = sin(BLH(0));
  Matrix3d D_1;
  D_1.setIdentity();

  double M = kA_84 * (1.0 - k_e1_2) * pow((1.0 - k_e1_2 * slat * slat), -1.5);
  double N = kA_84 / sqrt(1.0 - k_e1_2 * slat * slat);

  D_1(0, 0) = 1 / (M + BLH(2));
  D_1(1, 1) = 1 / ((N + BLH(2)) * cos(BLH(0)));
  D_1(2, 2) = -1;

  return D_1;
}

Vector3d GNSSPoint2Body(const Vector3d &BLH, const Matrix3d &Cb2n,
                        const Vector3d &Timu2ant) {
  Matrix3d D_1;
  D_1 = LeverArmRot(BLH);
  Vector3d BLH_cor = BLH - D_1 * Cb2n * Timu2ant;

  return BLH_cor;
}

Matrix3d BLH2DCM(const Vector2d &LatLon) {
  Matrix3d Cgn, Ceg, Cen;
  Cgn.setIdentity();

  double sL = sin(LatLon(0));
  double cL = cos(LatLon(0));
  double sl = sin(LatLon(1));
  double cl = cos(LatLon(1));
  Ceg << -sL * cl, -sL * sl, cL, -sl, cl, 0.0, -cL * cl, -cL * sl, -sL;

  Cen = Cgn * Ceg;
  return Cen;
}

Matrix3d BLH2DCM(const Vector2d &LatLon, const Vector2d &Wander) {
  Matrix3d Cgn, Ceg, Cen;

  Cgn << Wander(1), Wander(0), 0.0, -Wander(0), Wander(1), 0.0, 0.0, 0.0, 1.0;

  double sL = sin(LatLon(0));
  double cL = cos(LatLon(0));
  double sl = sin(LatLon(1));
  double cl = cos(LatLon(1));
  Ceg << -sL * cl, -sL * sl, cL, -sl, cl, 0.0, -cL * cl, -cL * sl, -sL;

  Cen = Cgn * Ceg;
  return Cen;
}

// ecef2geo & geo2ecef are refered to gpstk
Vector3d ECEF2Geo(const Vector3d &XYZ) {
  double p, slat, N, htold, latold;
  Vector3d BLH;

  p = XYZ.head(2).norm();
  // if (p < POSITION_TOLERANCE / 5)
  // { // pole or origin
  //     BLH(0) = (XYZ(2) > 0 ? 90.0 : -90.0);
  //     BLH(1) = 0; // lon undefined, really
  //     BLH(2) = fabs(XYZ(2)) - kA_84 * SQRT(1.0 - k_e1_2);
  //     return;
  // }
  BLH(0) = atan2(XYZ(2), p * (1.0 - k_e1_2));
  BLH(2) = 0;
  for (int i = 0; i < 5; i++) {
    slat = sin(BLH(0));
    N = kA_84 / sqrt(1.0 - k_e1_2 * slat * slat);
    htold = BLH(2);
    BLH(2) = p / cos(BLH(0)) - N;
    latold = BLH(0);
    BLH(0) = atan2(XYZ(2), p * (1.0 - k_e1_2 * (N / (N + BLH(2)))));
    if (fabs(BLH(0) - latold) < 1.0e-9 && fabs(BLH(2) - htold) < 1.0e-9 * kA_84)
      break;
  }
  BLH(1) = atan2(XYZ(1), XYZ(0));
  // if (BLH(1) < 0.0)
  //     BLH(1) += TWO_PI;

  return BLH;
}

Vector3d Geo2ECEF(const Vector3d &BLH) {
  Vector3d XYZ;
  double clat = cos(BLH(0));
  double slat = sin(BLH(0));
  double clon = cos(BLH(1));
  double slon = sin(BLH(1));

  double N = kA_84 / sqrt(1.0 - k_e1_2 * slat * slat);
  XYZ(0) = (N + BLH(2)) * clat * clon;
  XYZ(1) = (N + BLH(2)) * clat * slon;
  XYZ(2) = (N * (1.0 - k_e1_2) + BLH(2)) * slat;

  return XYZ;
}

Vector3d NEDInLocalFrame(const Vector3d &CurBLH, const Vector3d &baseBLH) {
  Vector3d rovXYZ = Geo2ECEF(CurBLH);
  Vector3d baseXYZ = Geo2ECEF(baseBLH);

  Matrix3d baseCen = BLH2DCM(baseBLH.head(2), Vector2d(0, 1));
  Vector3d dXYZ = rovXYZ - baseXYZ;
  Vector3d dNED = baseCen * dXYZ;

  return dNED;
}

Vector3d ENUInLocalFrame(const Vector3d &CurBLH, const Vector3d &baseBLH) {
  Vector3d rovXYZ = Geo2ECEF(CurBLH);
  Vector3d baseXYZ = Geo2ECEF(baseBLH);

  Matrix3d baseCen = BLH2DCM(baseBLH.head(2), Vector2d(0, 1));
  Vector3d dXYZ = rovXYZ - baseXYZ;
  Vector3d dNED = baseCen * dXYZ;

  Vector3d dENU = Vector3d(dNED(1), dNED(0), -dNED(2));
  return dENU;
}

Vector3d CalBLHInNavFrame(const Vector3d &CurBLH) {
  Vector3d Point_NavFrame;
  if (gstrNavFrame == "NED")
    Point_NavFrame = NEDInLocalFrame(CurBLH, gIMUBaseBLH);
  else if (gstrNavFrame == "ENU")
    Point_NavFrame = ENUInLocalFrame(CurBLH, gIMUBaseBLH);
  else {
    cerr << "Unknown NavFrame when running CalBLHInNavFrame() in VIGCommon.cpp!"
         << endl;
    exit(1);
  }

  return Point_NavFrame;
}

Matrix3d RTKlibStd2CovMatrix(Eigen::Matrix<double, 6, 1> &RTKliblStd) {
  Matrix3d CovMatrix;
  Vector3d atVar;  // autocorelative var
  double maxCov = RTKliblStd.maxCoeff();

  atVar = RTKliblStd.head(3);
  atVar = atVar.cwiseAbs2();
  CovMatrix = atVar.asDiagonal();

  for (int i = 0; i < 6; i++) {
    if (RTKliblStd(i) == 0.0)  // very rarerly, but sometimes var is 0
      RTKliblStd(i) = 2 * maxCov;
  }

  CovMatrix(0, 1) = fabs(RTKliblStd(3)) * RTKliblStd(3);
  CovMatrix(1, 0) = CovMatrix(0, 1);
  CovMatrix(1, 2) = fabs(RTKliblStd(4)) * RTKliblStd(4);
  CovMatrix(2, 1) = CovMatrix(1, 2);
  CovMatrix(0, 2) = fabs(RTKliblStd(5)) * RTKliblStd(5);
  CovMatrix(2, 0) = CovMatrix(0, 2);

  return CovMatrix;
}

Eigen::Matrix<double, 6, 1> CovMatrix2RTKlibStd(const Matrix3d &CovMatrix) {
  Eigen::Matrix<double, 6, 1> RTKliblStd;
  Vector3d atStd;  // autocorelative std

  atStd = CovMatrix.diagonal();
  atStd = atStd.cwiseSqrt();

  // sg=sign(CovMatrix);
  RTKliblStd.head(3) = atStd;
  RTKliblStd(3) = SGN(CovMatrix(0, 1)) * sqrt(fabs(CovMatrix(0, 1)));
  RTKliblStd(4) = SGN(CovMatrix(1, 2)) * sqrt(fabs(CovMatrix(1, 2)));
  RTKliblStd(5) = SGN(CovMatrix(0, 2)) * sqrt(fabs(CovMatrix(0, 2)));

  return RTKliblStd;
}

Matrix3d GetCamK() {
  Matrix3d CamK;
  CamK = gCamK;

  return CamK;
}

void GetRoi(int MaskWidth, cv::Rect &Roi) {
  Roi = cv::Rect(MaskWidth, MaskWidth, gNCol - MaskWidth, gNRow - MaskWidth);
}

gtsam::Pose3 GetCameraPose2Ref(const gtsam::Pose3 &Pose_imu2ref,
                               const gtsam::Pose3 &CamPosec2i) {
  // gtsam::Rot3 Rotc2r;
  // gtsam::Point3 Trsc2r;

  // Rotc2r = Pose_imu2ref.rotation() * CamPosec2i.rotation();
  // Trsc2r = Pose_imu2ref.translation() + Pose_imu2ref.rotation() *
  // CamPosec2i.translation();

  gtsam::Pose3 Pose_cam2ref =
      Pose_imu2ref * CamPosec2i;  // gtsam::Pose3(Rotc2r, Trsc2r)

  return Pose_cam2ref;
}

double CalCVDistance(cv::Point2f &pt1, cv::Point2f &pt2) {
  // printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

void GetDistortion(const Vector2d &mxy_d, Vector2d &d_u) {
  double k1 = gCamDistCoef(0);  // k1
  double k2 = gCamDistCoef(1);  // k2
  double p1 = gCamDistCoef(2);  // p1
  double p2 = gCamDistCoef(3);  // p2

  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = mxy_d(0) * mxy_d(0);
  my2_u = mxy_d(1) * mxy_d(1);
  mxy_u = mxy_d(0) * mxy_d(1);
  rho2_u = mx2_u + my2_u;
  rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
  d_u << mxy_d(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
      mxy_d(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

void LiftProjective(const Vector2d &uv, const int NumUnDisIter, Vector3d &xyz) {
  double mx_d, my_d, mx_u, my_u;

  double m_inv_K11 = 1.0 / gCamK(0, 0);
  double m_inv_K13 = -gCamK(0, 2) / gCamK(0, 0);
  double m_inv_K22 = 1.0 / gCamK(1, 1);
  double m_inv_K23 = -gCamK(1, 2) / gCamK(1, 1);
  mx_d = m_inv_K11 * uv(0) + m_inv_K13;
  my_d = m_inv_K22 * uv(1) + m_inv_K23;

  // Lift points to normalised plane
  mx_d = 1 / gCamK(0, 0) * uv(0) - gCamK(0, 2) / gCamK(0, 0);
  my_d = 1 / gCamK(1, 1) * uv(1) - gCamK(1, 2) / gCamK(1, 1);

  // Recursive distortion model
  Vector2d d_u;
  GetDistortion(Vector2d(mx_d, my_d), d_u);
  // Approximate value
  mx_u = mx_d - d_u(0);
  my_u = my_d - d_u(1);

  for (int i = 1; i < NumUnDisIter; ++i) {
    GetDistortion(Vector2d(mx_u, my_u), d_u);
    mx_u = mx_d - d_u(0);
    my_u = my_d - d_u(1);
  }
  // Obtain a projective ray
  xyz << mx_u, my_u, 1.0;
}

void ProjectUnDisXYZ2DisUV(const Vector3d &UnDisXYZ, Vector2d &DisUV) {
  double inv_Z = 1 / UnDisXYZ(2);
  Vector2d x = UnDisXYZ.head(2) * inv_Z;

  // Add distortion:
  double r2 = SQR(x(0)) + SQR(x(1));
  double r4 = SQR(r2), r6 = CUB(r2);

  // Radial distortion:
  double cdist =
      1 + gCamDistCoef(0) * r2 + gCamDistCoef(1) * r4 + gCamDistCoef(4) * r6;
  Vector2d xd1 = x * cdist;

  // tangential distortion:
  double a1 = 2. * x(0) * x(1);
  double a2 = r2 + 2 * SQR(x(0));
  double a3 = r2 + 2 * SQR(x(1));

  Vector2d delta_x;
  delta_x(0) = gCamDistCoef(2) * a1 + gCamDistCoef(3) * a2;
  delta_x(1) = gCamDistCoef(2) * a3 + gCamDistCoef(3) * a1;

  // Vector3d aa = (2 * gCamDistCoef(2) * x(1) + 6 * gCamDistCoef(3) * x(0)) *
  // Vector3d(1, 1, 1); Vector3d bb = (2 * gCamDistCoef(2) * x(0) + 2 *
  // gCamDistCoef(3) * x(1)) * Vector3d(1, 1, 1); Vector3d cc = (6 *
  // gCamDistCoef(2) * x(1) + 2 * gCamDistCoef(3) * x(0)) * Vector3d(1, 1, 1);

  Vector2d xd2 = xd1 + delta_x;

  // Add Skew:
  Vector2d xd3;
  xd3(0) = xd2(0) + gCamK(0, 1) / gCamK(0, 0) * xd2(1);
  xd3(1) = xd2(1);

  // Pixel coordinates:
  DisUV(0) = xd3(0) * gCamK(0, 0) + gCamK(0, 2);
  DisUV(1) = xd3(1) * gCamK(1, 1) + gCamK(1, 2);
}

void ProjectUnDisXYZ2UnDisUV(const Vector3d &UnDisXYZ, Vector2d &UnDisUV) {
  Vector3d UnDisXY1 = UnDisXYZ / UnDisXYZ(2);
  // Vector3d FeatureUnDisUVrepj = gCamK * UnDisXY1;
  // UnDisUV = FeatureUnDisUVrepj.head(2);

  UnDisUV(0) = UnDisXY1(0) * gCamK(0, 0) + gCamK(0, 2);
  UnDisUV(1) = UnDisXY1(1) * gCamK(1, 1) + gCamK(1, 2);
}

// UnDis_Dis_Flag, 0： UnDis; 1: Dis
bool ProjectUnDisXYZ2UV(const Vector3d &UnDisXYZ,
                        const eUnDis_Dis_Flag UnDis_Dis_Flag, Vector2d &UV) {
  // is in front of the camera
  if (UnDisXYZ(2) <= 0) {
    // UV = Vector2d(0, 0);
    return 0;
  }

  // Is in front of the camera?
  if ((atan2(UnDisXYZ(0), UnDisXYZ(2)) * kRad2Deg < -70) ||
      (atan2(UnDisXYZ(0), UnDisXYZ(2)) * kRad2Deg > 70) ||
      (atan2(UnDisXYZ(1), UnDisXYZ(2)) * kRad2Deg < -70) ||
      (atan2(UnDisXYZ(1), UnDisXYZ(2)) * kRad2Deg > 70)) {
    // UV = Vector2d(0, 0);
    return 0;
  }

  if (UnDis_Dis_Flag == UNDIS) {
    Vector2d UnDisUV;
    ProjectUnDisXYZ2UnDisUV(UnDisXYZ, UnDisUV);
    UV = UnDisUV;
  } else {
    Vector2d DisUV;
    ProjectUnDisXYZ2DisUV(UnDisXYZ, DisUV);
    UV = DisUV;
  }

  // Is visible in the image?
  if ((UV(0) <= 0) || (UV(0) >= gNCol) || (UV(1) <= 0) || (UV(1) >= gNRow)) {
    // UV = Vector2d(0, 0);
    return 0;
  }

  return 1;
}

void GetXYdistortbyXYnormalized(const Vector2d &xyn, Matrix2d &dxyd_dxyn) {
  double k1 = gCamDistCoef(0);
  double k2 = gCamDistCoef(1);
  double p1 = gCamDistCoef(2);
  double p2 = gCamDistCoef(3);
  double k3 = gCamDistCoef(4);

  double xn = xyn(0);
  double yn = xyn(1);

  dxyd_dxyn.setZero();
  // xyd to xyn
  dxyd_dxyn(0, 0) = k2 * SQR(SQR(xn) + SQR(yn)) + k3 * CUB(SQR(xn) + SQR(yn)) +
                    6 * p2 * xn + 2 * p1 * yn +
                    xn * (2 * k1 * xn + 4 * k2 * xn * (SQR(xn) + SQR(yn)) +
                          6 * k3 * xn * SQR(SQR(xn) + SQR(yn))) +
                    k1 * (SQR(xn) + SQR(yn)) + 1;
  // xd to yn
  dxyd_dxyn(0, 1) = 2 * p1 * xn + 2 * p2 * yn +
                    xn * (2 * k1 * yn + 4 * k2 * yn * (SQR(xn) + SQR(yn)) +
                          6 * k3 * yn * SQR(SQR(xn) + SQR(yn)));
  // yd to (xn, yn)
  dxyd_dxyn(1, 0) = 2 * p1 * xn + 2 * p2 * yn +
                    yn * (2 * k1 * xn + 4 * k2 * xn * (SQR(xn) + SQR(yn)) +
                          6 * k3 * xn * SQR(SQR(xn) + SQR(yn)));
  dxyd_dxyn(1, 1) = k2 * SQR(SQR(xn) + SQR(yn)) + k3 * CUB(SQR(xn) + SQR(yn)) +
                    2 * p2 * xn + 6 * p1 * yn +
                    yn * (2 * k1 * yn + 4 * k2 * yn * (SQR(xn) + SQR(yn)) +
                          6 * k3 * yn * SQR(SQR(xn) + SQR(yn))) +
                    k1 * (SQR(xn) + SQR(yn)) + 1;
}


Vector3d GTSAMRot2Att(const gtsam::Rot3 &gtRot3) {
  Vector3d AttDeg;
  AttDeg = Vector3d(gtRot3.roll(), gtRot3.pitch(), gtRot3.yaw());
  AttDeg *= kRad2Deg;

  return AttDeg;
}

void TriangulateOneFt_DualFrame(const Vector2d &FeatureUnDisXYl,
                                const gtsam::Pose3 &Poself2w,
                                const Vector2d &FeatureUnDisXYr,
                                const gtsam::Pose3 &Poserf2w,
                                Vector3d &Point3w) {
  assert(FeatureUnDisXYl != FeatureUnDisXYr && !(Poself2w.equals(Poserf2w)));
  // gtsam::Pose3 Pose_l2r = Poserf2w.inverse() * Poself2w;

  Eigen::Matrix<double, 4, 4> Pose_w2l_mt, Pose_w2r_mt;
  Pose_w2l_mt = Poself2w.inverse().matrix();  // Matrix4d::Identity();
  Pose_w2r_mt = Poserf2w.inverse().matrix();  // Pose_l2r.matrix();

  Matrix4d DesignMatrix = Matrix4d::Zero();
  DesignMatrix.row(0) =
      FeatureUnDisXYl[0] * Pose_w2l_mt.row(2) - Pose_w2l_mt.row(0);
  DesignMatrix.row(1) =
      FeatureUnDisXYl[1] * Pose_w2l_mt.row(2) - Pose_w2l_mt.row(1);
  DesignMatrix.row(2) =
      FeatureUnDisXYr[0] * Pose_w2r_mt.row(2) - Pose_w2r_mt.row(0);
  DesignMatrix.row(3) =
      FeatureUnDisXYr[1] * Pose_w2r_mt.row(2) - Pose_w2r_mt.row(1);
  Vector4d TriangulatedPoint3w;
  TriangulatedPoint3w =
      DesignMatrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();

  //! 转为齐次坐标
  Point3w(0) = TriangulatedPoint3w(0) / TriangulatedPoint3w(3);
  Point3w(1) = TriangulatedPoint3w(1) / TriangulatedPoint3w(3);
  Point3w(2) = TriangulatedPoint3w(2) / TriangulatedPoint3w(3);
}

bool SolveFrameByPnP(const vector<cv::Point3f> &vCVPoint3ws,
                     const vector<cv::Point2f> &vCVUnDisXYs,
                     gtsam::Pose3 &Pose_fx2w_inital) {
  //! PnP的初始值为上一帧的变换矩阵
  gtsam::Pose3 Pose_w2cx_inital = Pose_fx2w_inital.inverse();

  cv::Mat r, rvec, t, D, tmp_r;
  cv::eigen2cv(Pose_w2cx_inital.rotation().matrix(), tmp_r);
  cv::Rodrigues(tmp_r, rvec);
  cv::eigen2cv(Vector3d(Pose_w2cx_inital.translation().matrix()), t);
  cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  bool bPnPSucc;
  bPnPSucc = cv::solvePnP(vCVPoint3ws, vCVUnDisXYs, K, D, rvec, t, 1);
  if (!bPnPSucc) return false;

  //! 这里要注意的是PnP求解的结果是将3D点从世界坐标系转到相机坐标系的变换，
  //! 而我们要求解的相机位姿是相机坐标系到世界坐标系的变换
  cv::Rodrigues(rvec, r);
  // cout << "r " << endl << r << endl;
  MatrixXd Rpnp_w2cx;
  cv::cv2eigen(r, Rpnp_w2cx);
  MatrixXd Tpnp_w2cx;
  cv::cv2eigen(t, Tpnp_w2cx);

  gtsam::Rot3 Rot_w2cx = gtsam::Rot3(Rpnp_w2cx);
  gtsam::Point3 Trans_w2cx = gtsam::Point3(Tpnp_w2cx);
  Pose_w2cx_inital = gtsam::Pose3(Rot_w2cx, Trans_w2cx);

  Pose_fx2w_inital = Pose_w2cx_inital.inverse();

  return true;
}

// // Unused functions
// double InvTPR2Depth(const Vector3d &InvTPR) {
//   double Depth = -1.;
//   double theta = InvTPR(0), phi = InvTPR(1), rho = InvTPR(2);
//   Depth = cos(phi) * cos(theta) / rho;

//   return Depth;
// }

// void InvTPR2Point3w(const gtsam::Pose3 &CamPose, const Vector3d &InvTPR,
//                     gtsam::Vector3 &Point3w) {
//   // Calculate the 3D coordinates of the landmark in the Pose1 frame
//   double theta = InvTPR(0), phi = InvTPR(1), rho = InvTPR(2);
//   gtsam::Point3 pose1_P_landmark(cos(phi) * sin(theta) / rho, sin(phi) / rho,
//                                  cos(phi) * cos(theta) / rho);
//   // Convert the landmark to world coordinates
//   Point3w = CamPose.transform_from(pose1_P_landmark);
// }

// void Point3w2InvTPR(const gtsam::Pose3 &CamPose,
//                     const gtsam::Vector3 &Point3w, gtsam::Vector3 &InvTPR,
//                     double &Depth_StartFrame) {
//   Vector3d UnDisXYZ = CamPose.inverse() * Point3w;
//   Depth_StartFrame = UnDisXYZ(2);

//   double Theta = atan2(UnDisXYZ.x(), UnDisXYZ.z());
//   double Phi = atan2(UnDisXYZ.y(), sqrt(UnDisXYZ.x() * UnDisXYZ.x() +
//                                         UnDisXYZ.z() * UnDisXYZ.z()));
//   double Rho = 1. / (Depth_StartFrame * UnDisXYZ.norm());

//   InvTPR = gtsam::Vector3(Theta, Phi, Rho);
// }

// void UnDisXYZ_Point3w2InvDepth(const Vector3d UnDisXYZ,
//                                const gtsam::Pose3 &CamPose,
//                                const gtsam::Vector3 &Point3w,
//                                gtsam::Vector3 &InvTPR,
//                                double &Depth_StartFrame) {
//   Vector3d Point3_StartFrm = CamPose.inverse() * Point3w;
//   Depth_StartFrame = Point3_StartFrm(2);

//   double Theta = atan2(UnDisXYZ.x(), UnDisXYZ.z());
//   double Phi = atan2(UnDisXYZ.y(), sqrt(UnDisXYZ.x() * UnDisXYZ.x() +
//                                         UnDisXYZ.z() * UnDisXYZ.z()));
//   double Rho = 1. / (Depth_StartFrame * UnDisXYZ.norm());

//   InvTPR = gtsam::Vector3(Theta, Phi, Rho);
// }

// gtsam::Vector3 SetInvDepth(const Vector3d UnDisXYZ, const double Depth) {
//   double Theta = atan2(UnDisXYZ.x(), UnDisXYZ.z());
//   double Phi = atan2(UnDisXYZ.y(), sqrt(UnDisXYZ.x() * UnDisXYZ.x() +
//                                         UnDisXYZ.z() * UnDisXYZ.z()));
//   double Rho = 1. / (Depth * UnDisXYZ.norm());

//   gtsam::Vector3 TPR = gtsam::Vector3(Theta, Phi, Rho);
//   return TPR;
// }

void TestSetTime1() {
  // if (CurImageID - InitiImageId >= IDRange_Int2Cur)
  clock_gettime(CLOCK_REALTIME, &gTestTime1);
}

void TestSetTime2AndShowTimeRange() {
  // if (CurImageID - InitiImageId >= IDRange_Int2Cur)
  // {
  clock_gettime(CLOCK_REALTIME, &gTestTime2);
  cout /*<< endl*/
      << (gTestTime2.tv_sec - gTestTime1.tv_sec) * 1000 +
             (gTestTime2.tv_nsec - gTestTime1.tv_nsec) / 1000000
      << "  ";  // "ms" << endl;
                // }
}

// void TestSetTime1(double InitiTime, double CurTime, double TimeRange_Int2Cur)
// {
//     if (CurTime - InitiTime >= TimeRange_Int2Cur)
//         clock_gettime(CLOCK_REALTIME, &gTestTime1);
// }

// void TestSetTime2AndShowTimeRange(double InitiTime, double CurTime,
//                                   double TimeRange_Int2Cur)
// {
//     if (CurTime - InitiTime >= TimeRange_Int2Cur)
//     {
//         clock_gettime(CLOCK_REALTIME, &gTestTime2);
//         cout << endl
//              << "Prepare Time: " << (gTestTime2.tv_sec - gTestTime1.tv_sec) *
//              1000 + (gTestTime2.tv_nsec - gTestTime1.tv_nsec) / 1000000 <<
//              "ms" << endl;
//     }
// }

// void TestTxt_CopyAndModifyWhenNeed() {
  // //// 1. InitializingStructure of FactorGraphEstimator.cpp Test
  // {
  //     for (int i = 0; i < mFrontFrameIDInSW; i++)
  //     {
  //         cout << endl
  //              << i << "th:" << endl;

  //         gtsam::Pose3 PoseXth_0th_SW = mdsFramesInSW[i + 1].Pose.inverse() *
  //         mdsFramesInSW[0].Pose; gtsam::Pose3 PoseXth_0th_CV = arCVPose_x2w[i
  //         + 1].inverse() * arCVPose_x2w[0];

  //         // PoseXth_0th_SW.print("PoseXth_0th_SW:");
  //         cout << endl
  //              << "PoseXth_0th_SW Att: "
  //              << Vector3d(PoseXth_0th_SW.rotation().roll(),
  //                          PoseXth_0th_SW.rotation().pitch(),
  //                          PoseXth_0th_SW.rotation().yaw())
  //                         .transpose() *
  //                     kRad2Deg
  //              << endl;
  //         // PoseXth_0th_CV.print("PoseXth_0th_CV:");
  //         cout << endl
  //              << "PoseXth_0th_CV Att: "
  //              << Vector3d(PoseXth_0th_CV.rotation().roll(),
  //                          PoseXth_0th_CV.rotation().pitch(),
  //                          PoseXth_0th_CV.rotation().yaw())
  //                         .transpose() *
  //                     kRad2Deg
  //              << endl;

  //         cout << endl
  //              << "Diff_SW_CV Att: "
  //              << Vector3d((PoseXth_0th_SW.rotation().inverse() *
  //              PoseXth_0th_CV.rotation()).roll(),
  //                          (PoseXth_0th_SW.rotation().inverse() *
  //                          PoseXth_0th_CV.rotation()).pitch(),
  //                          (PoseXth_0th_SW.rotation().inverse() *
  //                          PoseXth_0th_CV.rotation()).yaw())
  //                         .transpose() *
  //                     kRad2Deg
  //              << endl;

  //         Vector3d Scl = Vector3d(PoseXth_0th_SW.translation().matrix()(0) /
  //         PoseXth_0th_CV.translation().matrix()(0),
  //                                 PoseXth_0th_SW.translation().matrix()(1) /
  //                                 PoseXth_0th_CV.translation().matrix()(1),
  //                                 PoseXth_0th_SW.translation().matrix()(2) /
  //                                 PoseXth_0th_CV.translation().matrix()(2));
  //         // double
  //         Scl=mdsFramesInSW_ithPoint3.norm()/arCVPose_x2w_ithPoint3.norm();
  //         cout << endl
  //              << "Scale:" << Scl << endl;
  //     }

  //     gtsam::Pose3 Pose_l2latest_SW = mdsFramesInSW.back().Pose.inverse() *
  //                                     mdsFramesInSW.front().Pose;
  //     cout << endl
  //          << "Delt_SW_l2latest Att: "
  //          << Vector3d(Pose_l2latest_SW.rotation().roll(),
  //                      Pose_l2latest_SW.rotation().pitch(),
  //                      Pose_l2latest_SW.rotation().yaw())
  //                     .transpose() *
  //                 kRad2Deg
  //          << endl;
  //     cout << endl
  //          << "Delt_CV_ep_l2latest Att: "
  //          << Vector3d(Pose_l2latest.rotation().roll(),
  //                      Pose_l2latest.rotation().pitch(),
  //                      Pose_l2latest.rotation().yaw())
  //                     .transpose() *
  //                 kRad2Deg
  //          << endl;

  //     Vector3d TotalScale =
  //     Vector3d(Pose_l2latest_SW.translation().matrix()(0) /
  //     Pose_l2latest.translation().matrix()(0),
  //                                    Pose_l2latest_SW.translation().matrix()(1)
  //                                    /
  //                                    Pose_l2latest.translation().matrix()(1),
  //                                    Pose_l2latest_SW.translation().matrix()(2)
  //                                    /
  //                                    Pose_l2latest.translation().matrix()(2));
  //     cout << endl
  //          << "Delt_SW_l2latest Translation: " <<
  //          Pose_l2latest_SW.translation().matrix().transpose() << endl
  //          << "Delt_CV_l2latest Translation: " <<
  //          Pose_l2latest.translation().matrix().transpose() << endl
  //          << "Total Scale: " << TotalScale.transpose()
  //          << endl;

  //     cout << endl
  //          << "Test Finished!" << endl;
  // }

  // // // 2. Test: mpImu_preintegrated_ output of PreIntegrating of
  // FactorGraphEstimator.cpp
  // {
  //     cout << "J_theta_Bg" << endl
  //      << mCamPosec2i.rotation().matrix() *
  //             mpImu_preintegrated_->preintegrated_H_biasOmega().block<3,
  //             3>(0, 0)
  //      << endl;
  //      mpImu_preintegrated_->print(" ");
  //      gtsam::Pose3 CurPredPose = GetCurPredPose();
  //      CurPredPose.print();
  // }

  // // 3. pose inference comparison between PreIntegration/EpiGeo/PnP, of
  // VIGSystem.cpp
  // {
  //     if (mpFactorGraphEstimator->meVIGEstimatorState == VIG_OK)
  //     {
  //         mtstCurPose_c2w = mpFactorGraphEstimator->GetCurPredPose(); //
  //         body_P_sensor has been setted
  //         // PreIntegration
  //         // mtstCurPose_c2w.print("mtstCurPose_c2w ");

  //         // gtsam::Pose3 DeltPose_prein = mtstCurPose_c2w.inverse() *
  //         mpFactorGraphEstimator->mOptimizedCurNavState.pose();
  //         // DeltPose_prein.print("DeltPose_prein: ");
  //         // Vector3d Att_prein = Vector3d(DeltPose_prein.rotation().roll(),
  //         DeltPose_prein.rotation().pitch(),
  //         DeltPose_prein.rotation().yaw());
  //         // cout << "CurFrameId: " <<
  //         mpFeatureTracker->mprsCurFrameWisedFeatures.first.FrameId << endl
  //         //      << "Att_prein: " << Att_prein.transpose() * kRad2Deg <<
  //         endl;
  //         // cout << "Trs_prein: " <<
  //         DeltPose_prein.translation().matrix().transpose() << endl;

  //         // EpoGeo
  //         gtsam::Pose3 DeltPose_epigeo;
  //         if (mpFeatureTracker->mprsCurFrameWisedFeatures.first.FrameId ==
  //         58)
  //             cout << "Wait! EpiGeo Error!" << endl;
  //         if
  //         (mpFeatureManager->CalRelativePose_L2R(mpFactorGraphEstimator->mMaxFrameIdInSW
  //         - 1,
  //                                                   mpFactorGraphEstimator->mMaxFrameIdInSW,
  //                                                   DeltPose_epigeo))
  //         {
  //             gtsam::Pose3 CurPose_cam2ref_epigeo =
  //             mpFactorGraphEstimator->mOptimizedCurNavState.pose() *
  //             DeltPose_epigeo.inverse();
  //             // CurPose_cam2ref_epigeo.print("CurPose_cam2ref_epigeo ");
  //             cout << "Att_epigeo: " <<
  //             GTSAMRot2Att(CurPose_cam2ref_epigeo.rotation()).transpose() <<
  //             endl;
  //             // cout << "Trs_epigeo: " <<
  //             CurPose_cam2ref_epigeo.translation().matrix().transpose() <<
  //             endl;

  //             // DeltPose_epigeo.print("DeltPose_epigeo: ");
  //             // Vector3d Att_epigeo =
  //             Vector3d(DeltPose_epigeo.rotation().roll(),
  //             DeltPose_epigeo.rotation().pitch(),
  //             DeltPose_epigeo.rotation().yaw());
  //             // cout << "Att_epigeo: " << Att_epigeo.transpose() * kRad2Deg
  //             << endl;
  //             // cout << "Trs_epigeo: " <<
  //             DeltPose_epigeo.translation().matrix().transpose() << endl;
  //         }
  //     }

  //     if (mpFactorGraphEstimator->meVIGEstimatorState == VIG_OK &&
  //     mbIsVisualAvailable)
  //     {
  //         // PnP
  //         gtsam::Pose3 CurPose_cam2ref_PnP = mtstCurPose_c2w, DeltPose_pnp;
  //         bool bPnP = false;
  //         if
  //         (mpFeatureManager->SolvePnP4CurFrameInKFQue(mpFeatureTracker->mprsCurFrameWisedFeatures,
  //                                                        CurPose_cam2ref_PnP))
  //         {
  //             bPnP = true;
  //             // CurPose_cam2ref_PnP.print("CurPose_cam2ref_PnP ");

  //             // DeltPose_pnp = CurPose_cam2ref_PnP.inverse() *
  //             mpFactorGraphEstimator->mOptimizedCurNavState.pose();
  //             // DeltPose_pnp.print("DeltPose_pnp: ");
  //             // Vector3d Att_pnp = Vector3d(DeltPose_pnp.rotation().roll(),
  //             DeltPose_pnp.rotation().pitch(),
  //             DeltPose_pnp.rotation().yaw());
  //             // cout << "Att_pnp: " << Att_pnp.transpose() * kRad2Deg <<
  //             endl;
  //             // cout << "Trs_pnp: " <<
  //             DeltPose_pnp.translation().matrix().transpose() << endl;
  //         }
  //         cout << "CurFrameId: " <<
  //         mpFeatureTracker->mprsCurFrameWisedFeatures.first.FrameId << endl
  //              << "Att_prein: " <<
  //              GTSAMRot2Att(mtstCurPose_c2w.rotation()).transpose() << endl;
  //         if (bPnP)
  //             cout << "Att_pnp: " <<
  //             GTSAMRot2Att(CurPose_cam2ref_PnP.rotation()).transpose() <<
  //             endl;

  //         cout << "Trs_prein: " <<
  //         mtstCurPose_c2w.translation().matrix().transpose() << endl; if
  //         (bPnP)
  //             cout << "Trs_pnp: " <<
  //             CurPose_cam2ref_PnP.translation().matrix().transpose() << endl;
  //         cout << endl;
  //     }
  // }

  // // 4. TriangulateRestFts_MultFrames test of fun
  // TriangulateRestFts_MultFrames of Featuremanager.cpp
  // // Test1
  // int LastFrameIdInKFQue = XthFrameIDInKFQue;
  // for (int itFtOnXthFrameInKFQue = StartFrameIDInKFQue, itFtOnXthFrame = 0;
  //      itFtOnXthFrameInKFQue <= LastFrameIdInKFQue;
  //      itFtOnXthFrameInKFQue++, itFtOnXthFrame++)
  // {
  //     gtsam::Point3 Point3_XthFrameIDInKFQue =
  //     dKeyFrames[itFtOnXthFrameInKFQue].Pose.inverse() *
  //                                              litIdWisedFeature.Point3w;
  //     Point3_XthFrameIDInKFQue /= Point3_XthFrameIDInKFQue(2);
  //     gtsam::Point3 UnDisUVrep_homo = mCamK * Point3_XthFrameIDInKFQue;

  //     gtsam::Point2 ErrorRepj = UnDisUVrep_homo.head(2) -
  //     litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames[itFtOnXthFrame].UnDisUV;
  //     ErrorRepj.print("ErrorRepj ");
  //     cout << endl;
  // }
  //
  // // Test2
  // gtsam::Pose3 Posec02cx_prein = dKeyFrames[XthFrameIDInKFQue].Pose.inverse()
  // *
  //                                dKeyFrames[StartFrameIDInKFQue].Pose;
  // gtsam::Pose3 Posec02cx_epgeo_cal, Posec02cx_epgeo;
  // if (CalRelativePose_L2R(StartFrameIDInKFQue, XthFrameIDInKFQue,
  // Posec02cx_epgeo_cal))
  //     Posec02cx_epgeo = Posec02cx_epgeo_cal;

  // Vector3d Point3c0;
  // TriangulateOneFt_DualFrame(gtsam::Pose3::identity(), Posec02cx_epgeo,
  //                            litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames[StartFrameIDInKFQue].UnDisXYZ.head(2),
  //                            litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames[XthFrameIDInKFQue].UnDisXYZ.head(2),
  //                            Point3c0);

  // Vector3d Altc02cx_prein = Vector3d(Posec02cx_prein.rotation().roll(),
  // Posec02cx_prein.rotation().pitch(), Posec02cx_prein.rotation().yaw());
  // Altc02cx_prein *= kRad2Deg;
  // Vector3d Altc02cx_epgeo = Vector3d(Posec02cx_epgeo.rotation().roll(),
  // Posec02cx_epgeo.rotation().pitch(), Posec02cx_epgeo.rotation().yaw());
  // Altc02cx_epgeo *= kRad2Deg;
  // gtsam::Rot3 DeltRot = Posec02cx_prein.rotation() *
  //                       Posec02cx_epgeo.rotation().inverse();
  // Vector3d DeltAlt = Vector3d(DeltRot.roll(), DeltRot.pitch(),
  // DeltRot.yaw()); DeltAlt *= kRad2Deg; Vector3d Scl; Vector3d Point3w =
  // dKeyFrames[XthFrameIDInKFQue].Pose * Point3c0; for (int i = 0; i < 3; i++)
  //     Scl(i) = Point3_StartFrameIDInKFQue(i) / Point3c0(i);

  // double svd_method = svd_V[2] / svd_V[3];

  // if (litIdWisedFeature.EstimatedDepth_StartFrameIDInKFQue < 0.1)
  // {
  //     litIdWisedFeature.EstimatedDepth_StartFrameIDInKFQue = gInitiDepth;
  // }

  // // 5. remove unused factors_
  // if (gbUseCam)
  // {
  //     for (int i = 0; i < MAX_ITERATION_ISAM_UPD_CAL; i++)
  //     {
  //         gtsam::VectorValues CurDeltas = mFLSmootherISAM2.getDelta();
  //         // Test
  //         CurDeltas.print();
  //         cout << endl;
  //         GetFactorsWtLandMarkToRemove(CurDeltas);

  //         if (mFactorsWtLandMarkToRemove.empty())
  //             break;

  //         gtsam::NonlinearFactorGraph nullFactorGraph;
  //         gtsam::Values nullValues;
  //         gtsam::FixedLagSmootherKeyTimestampMap nullKey_TimeStamps;
  //         mFLSmootherISAM2.update(nullFactorGraph, nullValues,
  //         nullKey_TimeStamps, mFactorsWtLandMarkToRemove); FLResultValues =
  //         mFLSmootherISAM2.calculateEstimate();
  //     }
  // }

  /// remove unused keys
  // for (auto itUnusedLandMarkKey :
  // mFLSmootherISAM2.getISAM2Result().unusedKeys)
  // {
  //     if (mResultValues.exists(itUnusedLandMarkKey))
  //         mResultValues.erase(itUnusedLandMarkKey);
  // }

  // // 2*. Sovle scale,gravity,Acc bias
  // Eigen::Matrix<double, Dynamic, 4> A;
  // Eigen::Matrix<double, Dynamic, 3> A3;
  // Eigen::Matrix<double, Dynamic, 1> B;
  // Eigen::Matrix<double, Dynamic, 1> B3;
  // Matrix3d I3 = Matrix3d::Identity();
  // A.setZero(3 * (NTrackedFrms - 2), 4);
  // A3.setZero(3 * (NTrackedFrms - 2), 3);
  // B.setZero(3 * (NTrackedFrms - 2), 1);
  // B3.setZero(3 * (NTrackedFrms - 2), 1);
  // // double s_star0 = 0.20803365278239949;
  // // Vector3d g_w_star0 =
  // //     Vector3d(-0.77205850928756448, 1.1962859828478352,
  // -9.715290726491423); double s_star0 = 1.; Vector3d g_w_star0 = Vector3d(0,
  // 0, -9.81);
  // // 2.1 Approx Scale and Gravity vector in world frame (nav frame)
  // for (FID4IMUPre = 1; FID4IMUPre < NTrackedFrms - 1; FID4IMUPre++) {
  //   pair<Frame, FrameWisedFeatures> TrackedFrm1 =
  //       mdprTrackedFrames[FID4IMUPre - 1];
  //   pair<Frame, FrameWisedFeatures> TrackedFrm2 =
  //   mdprTrackedFrames[FID4IMUPre]; pair<Frame, FrameWisedFeatures>
  //   TrackedFrm3 =
  //       mdprTrackedFrames[FID4IMUPre + 1];
  //   // Delta time between frames
  //   double dt12 = TrackedFrm2.first.FrameEpoch -
  //   TrackedFrm1.first.FrameEpoch; double dt23 = TrackedFrm3.first.FrameEpoch
  //   - TrackedFrm2.first.FrameEpoch;
  //   // Pre-integrated measurements
  //   Vector3d dp12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].deltaPij();
  //   Vector3d dv12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].deltaVij();
  //   Vector3d dp23 = mmpIMUPreIntBetwDualFrms[FID4IMUPre + 1].deltaPij();
  //   Matrix3d Jpba12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].delPdelBiasAcc();
  //   Matrix3d Jvba12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].delVdelBiasAcc();
  //   Matrix3d Jpba23 = mmpIMUPreIntBetwDualFrms[FID4IMUPre +
  //   1].delPdelBiasAcc();
  //   // Pose of camera in world frame
  //   Vector3d pc1 = TrackedFrm1.first.Pose_c2w.translation().matrix();
  //   Vector3d pc2 = TrackedFrm2.first.Pose_c2w.translation().matrix();
  //   Vector3d pc3 = TrackedFrm3.first.Pose_c2w.translation().matrix();
  //   Matrix3d Rc1 = TrackedFrm1.first.Pose_c2w.rotation().matrix();
  //   Matrix3d Rc2 = TrackedFrm2.first.Pose_c2w.rotation().matrix();
  //   Matrix3d Rc3 = TrackedFrm3.first.Pose_c2w.rotation().matrix();

  //   Vector3d vb1 = TrackedFrm1.first.Vel_i2w;
  //   Vector3d vb2 = TrackedFrm2.first.Vel_i2w;
  //   Vector3d vb2_pre_gG = vb1 + gG * dt12 + Rc1 * Rcb * dv12;
  //   // cout << "vb2" << vb2.transpose() << endl;
  //   // cout << "vb2_pre_gstar" << vb2_pre_gstar.transpose() << endl;
  //   // cout << "vb2_pre_gG" << vb2_pre_gG.transpose() << endl;

  //   // lambda*s + beta*g = gamma
  //   A.block<3, 1>(3 * (FID4IMUPre - 1), 0) =
  //       (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;
  //   A.block<3, 3>(3 * (FID4IMUPre - 1), 1) =
  //       // Rc1 * Jpba12 * dt23 - Rc1 * Jvba12 * dt12 * dt23 - Rc2 * Jpba23 *
  //       // dt12;
  //       0.5 * I3 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23);
  //   B.middleRows<3>(3 * (FID4IMUPre - 1)) =
  //       (Rc3 - Rc2) * pcb * dt12 + (Rc1 - Rc2) * pcb * dt23 +
  //       Rc1 * Rcb * dp12 * dt23 - Rc2 * Rcb * dp23 * dt12 -
  //       Rc1 * Rcb * dv12 * dt12 * dt23;
  //   // - 0.5 * gG * dt12 * dt23 * (dt12 + dt23);

  //   Vector3d L1_0 = (pc2 - pc1) * s_star0 - 0.5 * g_w_star0 * dt12 * dt12;
  //   // Vector3d R1_0 = vb1 * dt12 + Rc1 * Rcb * dp12 + (Rc1 - Rc2) * pcb;
  //   Vector3d R1_0 = Rc1 * Rcb * dp12 + (Rc1 - Rc2) * pcb;
  //   Vector3d v1cal_1 = (L1_0 - R1_0) / dt12;
  //   Vector3d L2_0 = (pc3 - pc2) * s_star0 - 0.5 * g_w_star0 * dt23 * dt23;
  //   // Vector3d R2_0 = (vb1 + g_w_star0 * dt12 + Rc1 * Rcb * dv12) * dt23 +
  //   //                 Rc2 * Rcb * dp23 + (Rc2 - Rc3) * pcb;
  //   Vector3d R2_0 = (g_w_star0 * dt12 + Rc1 * Rcb * dv12) * dt23 +
  //                   Rc2 * Rcb * dp23 + (Rc2 - Rc3) * pcb;
  //   Vector3d v1cal_2 = (L2_0 - R2_0) / dt23;
  //   // cout << "L1_0 " << L1_0.transpose() << endl;
  //   // cout << "R1_0 " << R1_0.transpose() << endl;
  //   // cout << "L2_0 " << L2_0.transpose() << endl;
  //   // cout << "R2_0 " << R2_0.transpose() << endl;
  //   cout << "v1cal_1 " << v1cal_1.transpose() << endl;
  //   cout << "v1cal_2 " << v1cal_2.transpose() << endl;
  //   cout << endl;

  //   // Vector3d L1_1 =
  //   //     ((pc2 - pc1) * s_star0 - 0.5 * g_w_star0 * dt12 * dt12) * dt23;
  //   // Vector3d R1_1 = (vb1 * dt12 + Rc1 * Rcb * dp12 + (Rc1 - Rc2) * pcb) *
  //   // dt23; Vector3d L2_1 = ((pc3 - pc2) * s_star0 - 0.5 * g_w_star0 * dt23
  //   *
  //   // dt23 -
  //   //                  g_w_star0 * dt12 * dt23) *
  //   //                 dt12;
  //   // Vector3d R2_1 = ((vb1 + Rc1 * Rcb * dv12) * dt23 + Rc2 * Rcb * dp23 +
  //   //                  (Rc2 - Rc3) * pcb) *
  //   //                 dt12;
  //   // cout << "L1_1 " << L1_0.transpose() << endl;
  //   // cout << "R1_1 " << R1_0.transpose() << endl;
  //   // cout << "L2_1 " << L2_0.transpose() << endl;
  //   // cout << "R2_1 " << R2_0.transpose() << endl;
  //   // cout << endl;

  //   // A3.block<3, 3>(3 * (FID4IMUPre - 1), 0) =
  //   //     Rc1 * Jpba12 * dt23 - Rc1 * Jvba12 * dt12 * dt23 - Rc2 * Jpba23 *
  //   //     dt12;
  //   // B3.middleRows<3>(3 * (FID4IMUPre - 1)) =
  //   //     (Rc3 - Rc2) * pcb * dt12 + (Rc1 - Rc2) * pcb * dt23 +
  //   //     Rc1 * Rcb * dp12 * dt23 - Rc2 * Rcb * dp23 * dt12 -
  //   //     Rc1 * Rcb * dv12 * dt12 * dt23 -
  //   //     0.5 * gG * dt12 * dt23 * (dt12 + dt23) - (pc2 - pc1) * dt23 -
  //   //     (pc2 - pc3) * dt12;

  //   // Matrix3d A3_i = A3.block<3, 3>(3 * (FID4IMUPre - 1), 0);
  //   // Vector3d B3_i = B3.middleRows<3>(3 * (FID4IMUPre - 1));
  //   // Vector3d ba_i = A3_i.ldlt().solve(B3_i);
  //   // cout << "ba_i" << ba_i.transpose() << endl;
  //   // cout << endl;
  // }

  // // Solve Approx scale and gravity
  // cout << A << endl << endl;
  // cout << B << endl << endl;
  // Matrix4d ATA = A.transpose() * A;
  // Vector4d ATB = A.transpose() * B;
  // Vector4d s_g_w_star = ATA.ldlt().solve(ATB);
  // double s_star = s_g_w_star(0);
  // Vector3d g_w_star = s_g_w_star.tail(3);
  // if (s_star < 0) {
  //   cerr << "Wrong Scale!" << endl;
  //   exit(1);
  // }
  // Matrix3d ATA3 = A3.transpose() * A3;
  // Vector3d ATB3 = A3.transpose() * B3;
  // Vector3d b3 = ATA3.ldlt().solve(ATB3);

  // // Test	2.1, scale and gravity star
  // Eigen::Matrix<double, Dynamic, 4> A_;
  // Eigen::Matrix<double, Dynamic, 1> B_;
  // A_.setZero(3 * (NTrackedFrms - 1), 4);
  // B_.setZero(3 * (NTrackedFrms - 1), 1);
  // Vector3d g_w_sol =
  //     Vector3d(-18.01086718356709, 9.4237966080173443, -10.488114265581567);
  // for (FID4IMUPre = 1; FID4IMUPre < NTrackedFrms; FID4IMUPre++) {
  //   pair<Frame, FrameWisedFeatures> TrackedFrm1 =
  //       mdprTrackedFrames[FID4IMUPre - 1];
  //   pair<Frame, FrameWisedFeatures> TrackedFrm2 =
  //   mdprTrackedFrames[FID4IMUPre];
  //   // Delta time between frames
  //   double dt12 = TrackedFrm2.first.FrameEpoch -
  //   TrackedFrm1.first.FrameEpoch;
  //   // Pre-integrated measurements
  //   Vector3d dp12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].deltaPij();
  //   Vector3d dv12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].deltaVij();
  //   // Matrix3d Jpba12 =
  //   // mmpIMUPreIntBetwDualFrms[FID4IMUPre].delPdelBiasAcc(); Pose of camera
  //   // in world frame
  //   Vector3d pc1 = TrackedFrm1.first.Pose_c2w.translation().matrix();
  //   Vector3d pc2 = TrackedFrm2.first.Pose_c2w.translation().matrix();
  //   Matrix3d Rc1 = TrackedFrm1.first.Pose_c2w.rotation().matrix();
  //   Matrix3d Rc2 = TrackedFrm2.first.Pose_c2w.rotation().matrix();
  //   Vector3d vb1 = TrackedFrm1.first.Vel_i2w;
  //   Vector3d vb2 = TrackedFrm2.first.Vel_i2w;
  //   Vector3d vb2_pre_gstar = vb1 + g_w_star * dt12 + Rc1 * Rcb * dv12;
  //   Vector3d vb2_pre_gG = vb1 + gG * dt12 + Rc1 * Rcb * dv12;

  //   cout << "vb2" << vb2.transpose() << endl;
  //   cout << "vb2_pre_gstar" << vb2_pre_gstar.transpose() << endl;
  //   cout << "vb2_pre_gG" << vb2_pre_gG.transpose() << endl;
  //   // Vector3d deltv = vb2_pre - vb2;
  //   // cout << "deltv" << deltv.transpose() << endl;

  //   // lambda*s + beta*g = gamma
  //   gtsam::Pose3 Pose_b1 =
  //       mdInitKeyFrames[FID4IMUPre - 1].Pose_c2w * ExtrPose_i2c;
  //   gtsam::Pose3 Pose_b2 = mdInitKeyFrames[FID4IMUPre].Pose_c2w *
  //   ExtrPose_i2c; Vector3d pb1 = Pose_b1.translation().matrix(); Vector3d
  //   L_star = s_star * (pc2 - pc1) - 0.5 * g_w_star * dt12 * dt12; Vector3d
  //   L_s1_g_star = 1 * (pc2 - pc1) - 0.5 * g_w_star * dt12 * dt12; Vector3d
  //   L_true = 1 * (pc2 - pc1) - 0.5 * gG * dt12 * dt12; Vector3d R = (Rc1 -
  //   Rc2) * pcb + vb1 * dt12 + Rc1 * Rcb * dp12; cout << "L_star" <<
  //   L_star.transpose() << "      " << dt12 << endl; cout << "L_s1_g_star" <<
  //   L_s1_g_star.transpose() << endl; cout << "L_true" << L_true.transpose()
  //   << endl; cout << "R" << R.transpose() << endl << endl;

  //   A_.block<3, 1>(3 * (FID4IMUPre - 1), 0) = pc2 - pc1;
  //   A_.block<3, 3>(3 * (FID4IMUPre - 1), 1) = -0.5 * I3 * dt12 * dt12;
  //   B_.middleRows<3>(3 * (FID4IMUPre - 1)) =
  //       (Rc1 - Rc2) * pcb + vb1 * dt12 + Rc1 * Rcb * dp12;

  //   int a = 1;
  // }
  // cout << A_ << endl << endl;
  // cout << B_ << endl << endl;
  // // Solve Approx scale and gravity
  // Matrix4d ATA_ = A_.transpose() * A_;
  // Vector4d ATB_ = A_.transpose() * B_;
  // Vector4d s_g_w_star_ = ATA_.ldlt().solve(ATB_);

  // // 2.2 Solve Approx scale and gravity and bias
  // Vector3d g_I = Vector3d(0., 0., -1.);
  // Vector3d g_wn = g_w_star / g_w_star.norm();
  // Vector3d g_Ixg_wn = g_I.cross(g_wn);
  // double norm_g_Ixg_wn = g_Ixg_wn.norm();
  // Vector3d v_hat = g_Ixg_wn / norm_g_Ixg_wn;
  // double theta = atan2(norm_g_Ixg_wn, g_I.dot(g_wn));
  // Matrix3d Rwi = gtsam::Rot3::Expmap(gtsam::Vector3(v_hat * theta)).matrix();

  // Vector3d gw = Rwi * gG;

  // Eigen::Matrix<double, Dynamic, 6> C;
  // Eigen::Matrix<double, Dynamic, 1> D;
  // C.setZero(3 * (NTrackedFrms - 2), 6);
  // D.setZero(3 * (NTrackedFrms - 2), 1);
  // for (FID4IMUPre = 1; FID4IMUPre < NTrackedFrms - 1; FID4IMUPre++) {
  //   pair<Frame, FrameWisedFeatures> TrackedFrm1 =
  //       mdprTrackedFrames[FID4IMUPre - 1];
  //   pair<Frame, FrameWisedFeatures> TrackedFrm2 =
  //   mdprTrackedFrames[FID4IMUPre]; pair<Frame, FrameWisedFeatures>
  //   TrackedFrm3 =
  //       mdprTrackedFrames[FID4IMUPre + 1];
  //   // Delta time between frames
  //   double dt12 = TrackedFrm2.first.FrameEpoch -
  //   TrackedFrm1.first.FrameEpoch; double dt23 = TrackedFrm3.first.FrameEpoch
  //   - TrackedFrm2.first.FrameEpoch;
  //   // Pre-integrated measurements
  //   Vector3d dp12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].deltaPij();
  //   Vector3d dv12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].deltaVij();
  //   Vector3d dp23 = mmpIMUPreIntBetwDualFrms[FID4IMUPre + 1].deltaPij();
  //   Matrix3d Jpba12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].delPdelBiasAcc();
  //   Matrix3d Jvba12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].delVdelBiasAcc();
  //   Matrix3d Jpba23 = mmpIMUPreIntBetwDualFrms[FID4IMUPre +
  //   1].delPdelBiasAcc();

  //   // Pose of camera in world frame
  //   Vector3d pc1 = TrackedFrm1.first.Pose_c2w.translation().matrix();
  //   Vector3d pc2 = TrackedFrm2.first.Pose_c2w.translation().matrix();
  //   Vector3d pc3 = TrackedFrm3.first.Pose_c2w.translation().matrix();
  //   Matrix3d Rc1 = TrackedFrm1.first.Pose_c2w.rotation().matrix();
  //   Matrix3d Rc2 = TrackedFrm2.first.Pose_c2w.rotation().matrix();
  //   Matrix3d Rc3 = TrackedFrm3.first.Pose_c2w.rotation().matrix();

  //   C.block<3, 1>(3 * (FID4IMUPre - 1), 0) =
  //       (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;
  //   C.block<3, 3>(3 * (FID4IMUPre - 1), 1) =
  //       -0.5 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) * Rwi *
  //       Skew(gG);  // note: this has a '-', different to paper
  //   C.block<3, 3>(3 * (FID4IMUPre - 1), 3) = Rc2 * Rcb * Jpba23 * dt12 +
  //                                            Rc1 * Rcb * Jvba12 * dt12 * dt23
  //                                            - Rc1 * Rcb * Jpba12 * dt23;
  //   D.middleRows<3>(3 * (FID4IMUPre - 1)) =
  //       (Rc1 - Rc2) * pcb * dt23 + Rc1 * Rcb * dp12 * dt23 -
  //       (Rc2 - Rc3) * pcb * dt12 - Rc2 * Rcb * dp23 * dt12 -
  //       Rc1 * Rcb * dv12 * dt23 * dt12 -
  //       0.5 * Rwi * gG *
  //           (dt12 * dt12 * dt23 + dt12 * dt23 * dt23);  // note:  - paper
  // }
  // cout << C << endl << endl;
  // cout << D << endl << endl;
  // Eigen::Matrix<double, 6, 6> CTC = C.transpose() * C;
  // Eigen::Matrix<double, 6, 1> CTD = C.transpose() * D;
  // Eigen::Matrix<double, 6, 1> y = CTC.ldlt().solve(CTD);

  // double s = y(0);
  // Vector3d delta_theta = Vector3d(y(1), y(2), 0);
  // Vector3d ba = y.tail(3);

  // gw = Rwi * gG - Rwi * Skew(gG) * delta_theta;

  // for (FID4IMUPre = 1; FID4IMUPre < NTrackedFrms - 1; FID4IMUPre++) {
  //   pair<Frame, FrameWisedFeatures> TrackedFrm1 =
  //       mdprTrackedFrames[FID4IMUPre - 1];
  //   pair<Frame, FrameWisedFeatures> TrackedFrm2 =
  //   mdprTrackedFrames[FID4IMUPre];
  //   // Delta time between frames
  //   double dt12 = TrackedFrm2.first.FrameEpoch -
  //   TrackedFrm1.first.FrameEpoch;
  //   // Pre-integrated measurements
  //   Vector3d dp12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].deltaPij();
  //   Matrix3d Jpba12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].delPdelBiasAcc();

  //   // Pose of camera in world frame
  //   Vector3d pc1 = TrackedFrm1.first.Pose_c2w.translation().matrix();
  //   Vector3d pc2 = TrackedFrm2.first.Pose_c2w.translation().matrix();
  //   Vector3d v1 = TrackedFrm1.first.Vel_i2w;
  //   Matrix3d Rc1 = TrackedFrm1.first.Pose_c2w.rotation().matrix();
  //   Matrix3d Rc2 = TrackedFrm2.first.Pose_c2w.rotation().matrix();

  //   Vector3d L = s * pc2;
  //   Vector3d R = s * pc1 + v1 * dt12 -
  //                0.5 * Rwi * Skew(gG) * dt12 * dt12 * delta_theta +
  //                Rc1 * Rcb * (dp12 + Jpba12 * ba) + (Rc1 - Rc2) * pcb +
  //                0.5 * Rwi * gG * dt12 * dt12;
  //   cout << "L" << L.transpose() << endl;
  //   cout << "R" << R.transpose() << endl << endl;
  //   int a = 1;
  // }
// }

}  // namespace VIG
