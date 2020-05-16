#pragma once

#include <assert.h>
#include <float.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/slam/InvDepthFactorVariant4.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <istream>
#include <list>
#include <map>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <string>
#include <thread>
#include <vector>

using namespace std;
using namespace Eigen;

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::L;  // LandMark (X,Y,Z)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
typedef gtsam::InvDepthFactorVariant4c InvDepthFactor;

const double PI = 3.1415926535897932384626433832795;
const double TWO_PI = 2.0 * PI;      // 2PI
const double kDeg2Rad = PI / 180.0;  // Radians per degree
const double kRad2Deg = 180.0 / PI;  // Degrees per radian
const double kA_84 = 6378137.0;      // WGS84 long radius（m）
const double kB_84 = 6356752.3141;   // WGS84 short radius（m）
const double kFlattening = (kA_84 - kB_84) / kA_84;  // WGS84 falatenning
const double kRavg = 6317000.0;                      // Earth Average Radius
const double kOmega_WGS =
    7.292115e-5;  // [rad/s], earth autogiration angular rate
const double k_e1_2 =
    (kA_84 * kA_84 - kB_84 * kB_84) /
    (kA_84 * kA_84);  // First eccentricity's square of ellipsoid WGS84
const double k_e2_2 =
    (kA_84 * kA_84 - kB_84 * kB_84) /
    (kB_84 * kB_84);  // Second eccentricity's square of ellipsoid WGS84
const double kGM_Earth =
    3.986004418e+14;               // [m^3/s]; WGS84 Gravitational constant
const double kU0 = 62636860.8497;  // WGS84 Normal gravity potential
const double kGe = 9.7803267714;   // WGS84 nominal g on earth surface
const double kJ = -484.16685e-6;   // Second order harmonic coefficient of
                                   // gravitational potential
const double kTa = 9.7803267715;   // Gravity of the equator（m/s^2）
const double kTb = 9.8321863685;   // Gravity of the pole point（m/s^2）
// extern const double ARW = 0.0022;                     //
// 角度随机游走，deg/sqrt(h) extern const double VRW = 0.00075; // 速度随机游走,
// m/s/sqrt(h) extern const double T = 1000;                         //
// 零偏及比例因子高斯马尔科夫相关时间,s extern const double std_bg = 0.005; //
// 陀螺零偏标准差，deg/hr extern const double std_ba = 25; //
// 加表零偏标准差，mGal extern const double std_sg = 10;                      //
// 陀螺比例因子标准差,ppm extern const double std_sa = 10; //
// 加表比例因子标准差,ppm const int WINDOW_SIZE = 10;

const Vector3d kWie_e = Vector3d(0.0, 0.0, kOmega_WGS);
const Vector3d GNSSPosStdQ1 = Vector3d(0.05, 0.05, 0.15);
const Vector3d GNSSPosStdQ2 = Vector3d(1.0, 1.0, 2.0);
const Vector3d GNSSPosStdQ36 = Vector3d(8, 8, 15);

namespace VIG {

extern string gstrNavFrame;
extern Vector3d gG;
extern bool gbUseGNSS;
extern bool gbUseCam;

extern double gFocalLength;
extern Matrix3d gCamK;
extern Eigen::Matrix<double, 5, 1> gCamDistCoef;
extern int gNumUndisIter;
extern int gNRow;
extern int gNCol;
extern double gInitDepth;
extern int gKeyFramesQueueSize;
extern int gMaxFrameIdInKFQue;
extern int gCoVisualFramesThreshold;

extern Vector3d gIMUBaseBLH;
extern gtsam::Pose3 gExtrPose_i2g;
extern gtsam::Pose3 gExtrPose_c2i;

extern double gFirstPubFrameEpoch;
extern bool gbFirstFrame;  // the frame id w.r.t the video

// Test params
extern timespec gTestTime1, gTestTime2;
// static int gTestFramesThreshold;

enum eGNSSDataFormat { XYZ, BLHdeg, BLHdms };

enum eUnDis_Dis_Flag { UNDIS, DIS };

enum eIdWisedFeatureSolveFlag {
  NOT_SOLVED,
  OK_SOLVED,
  FAILED_SOLVED,
  WATING_FOR_SOLVE
};

enum eMarginFlag { MARGIN_OLD, MARGIN_SECOND_NEW };

enum eVIGEstimatorState { NOT_INITIALIZED, VIG_OK };

// IMU Error Definition
struct IMUErrorModel {
  double AccBiasTc;       // relatve time(sec)
  double AccBiasQ;        //
  double AccBiasVar;      // bias drift variability
  double InitAccBiasErr;  // mg  turn on bias

  double GyroBiasTc;       // sec
  double GyroBiasQ;        // deg/hr
  double GyroBiasVar;      // deg/hr
  double InitGyroBiasErr;  // deg turn on bias

  double AccScaleVar;   // ppm*1000
  double AccScaleQ;     // ppt, Tc~3600sec
  double GyroScaleVar;  // ppm*1000
  double GyroScaleQ;    // ppt, Tc~3600sec

  double AccVrw;   // m/s/sqrt(hr)
  double GyroArw;  // deg/sqrt(hr), but in tests I found it is much larger, so
                    // multiply it by 10
};

struct IMUErrorModel4PreIntegration {
  gtsam::Matrix33 accelerometerCovariance;
  gtsam::Matrix33 gyroscopeCovariance;
  gtsam::Matrix33 integrationCovariance;  // error committed in integrating
                                          // position from velocities
  gtsam::Matrix33 biasAccCovariance;
  gtsam::Matrix33 biasOmegaCovariance;
  gtsam::Matrix66 biasAccOmegaInt;  // error in the bias used for preintegration
};

struct IMUData {
  double T;
  Vector3d Acc;
  Vector3d Gyro;
};

struct IMUErrorStates {
  Vector3d AccBias;
  Vector3d GyroBias;
  Vector3d AccScale;
  Vector3d GyroScale;
};

struct IMUAccumlation {
  Vector3d Vel;
  Vector3d Angle;
};

struct GNSSSolData {
  double Tow;
  Vector3d BLH;
  Vector3d bodyPoint_NavFrame;
  int Qual;
  int ns;
  Eigen::Matrix<double, 6, 1> Std;
};

struct Frame {
  int FrameId;
  double FrameEpoch;
  gtsam::Pose3 Pose_c2w;
  gtsam::Vector3 Vel_i2w;
};

struct KeyFrame {
  int FrameId;
  double FrameEpoch;
  int NavStateIndex;
  gtsam::Pose3 Pose;
  // int FeaturesNum;
  KeyFrame(const int InFrameId, const double InFrameEpoch,
           const int InNavStateIndex, const gtsam::Pose3 &InPose)
      : FrameId(InFrameId),
        FrameEpoch(InFrameEpoch),
        NavStateIndex(InNavStateIndex),
        Pose(InPose) {}
  void SetPose(const gtsam::Pose3 &NewPose) { Pose = NewPose; }
};

struct FrameWisedFeatures {
  vector<int> vFeatureIds;
  vector<Vector2d> vUVs;
  vector<Vector2d> vUnDisUVs;
  vector<Vector3d> vUnDisXYZs;
  vector<bool> vbOptimized;
  // vector<double> InvDepths;
  vector<int> vTrackNums;
};

struct TrackMoreFeature {
  Vector2d UV;
  Vector2d UnDisUV;
  Vector3d UnDisXYZ;
  int TrackNum;
  // double InvDepth;
  // TrackMoreFeature(Vector2d UV_,
  //                  Vector2d UnDis_UV,
  //                  Vector3d UnDis_XYZ) : UV(UV_),
  //                                        UnDisUV(UnDis_UV),
  //                                        UnDisXYZ(UnDis_XYZ),
  //                                        bOptimized(false) {}
};

struct SFMFeature {
  bool bTriangulateState;
  int FeatureId;
  int MapPointIndex;
  int StartFrameIDInKFQue;
  map<int, Vector2d> mpKFID_UuvsOnAllFrms;
  map<int, Vector3d> mpKFID_UxyzsOnAllFrms;
  gtsam::Point3 Point3w;
  double InvDepth = -1;
  int SolvedFlag;  // 0 haven't solve yet; 1 solve succ;
                   // 2 solve fail; 3 waiting for solve
};

struct IdWisedFeatureOnCurFrame {
  // double CurTd;   // Used for optimization of delt between sensors
  int CurFrameId;
  Vector2d UnDisUV;
  Vector3d UnDisXYZ;
  // bool IsUsed;
};

struct IdWisedFeature {
  int FeatureId;
  int MapPointIndex;
  int StartFrameIDInKFQue;
  vector<IdWisedFeatureOnCurFrame> vIdWisedFeatureInfoOnAllFrames;

  int UsedFramesNum;
  bool bIsOutlier;
  bool bIsMargin;

  gtsam::Point3 Point3w;
  double InvDepth;
  int SolvedFlag;  // 0 haven't solve yet; 1 solve succ;
  // 2 solve fail; 3 waiting for solve
  // Vector3d gt_p;
  IdWisedFeature(int InFeatureId, int InStartFrameIDInKFQue)
      : FeatureId(InFeatureId),
        MapPointIndex(-1),
        StartFrameIDInKFQue(InStartFrameIDInKFQue),
        UsedFramesNum(0),
        InvDepth(-1.0),
        SolvedFlag(NOT_SOLVED) {}
};

template <typename T>
int SGN(T x) {
  return fabs(x) < 1e-10 ? 0 : (x < .0 ? -1 : 1);
};

template <typename T>
T SQR(T x) {
  return x * x;
};
template <typename T>
T CUB(T x) {
  return x * x * x;
};

IMUErrorModel4PreIntegration GetIMUErrorModel4PreIntegration(const int IMUType);
Matrix3d Skew(const Vector3d &vec3d);
Matrix3d R1(double XrotRad);
Matrix3d R2(double YrotRad);
Matrix3d R3(double ZrotRad);
void RemoveAttAnglesRedundancy(Vector3d &AttRad);
Quaterniond cfRotationVector2Quaterniond(const Vector3d &RotationVector);

Vector3d CalgravityN(const Vector3d &BLH);
Matrix3d LeverArmRot(const Vector3d &BLH);
Vector3d GNSSPoint2Body(const Vector3d &BLH, const Matrix3d &Cb2n,
                        const Vector3d &Timu2ant);

Matrix3d BLH2DCM(const Vector2d &LatLon);
Matrix3d BLH2DCM(const Vector2d &LatLon, const Vector2d &Wander);

Vector3d ECEF2Geo(const Vector3d &XYZ);
Vector3d Geo2ECEF(const Vector3d &BLH);

Vector3d NEDInLocalFrame(const Vector3d &CurBLH, const Vector3d &baseBLH);
Vector3d ENUInLocalFrame(const Vector3d &CurBLH, const Vector3d &baseBLH);
Vector3d CalBLHInNavFrame(const Vector3d &CurBLH);

Matrix3d RTKlibStd2CovMatrix(Eigen::Matrix<double, 6, 1> &RTKliblStd);
Eigen::Matrix<double, 6, 1> CovMatrix2RTKlibStd(const Matrix3d &CovMatrix);

Matrix3d GetCamK();
void GetRoi(int MaskWidth, cv::Rect &Roi);
gtsam::Pose3 GetCameraPose2Ref(const gtsam::Pose3 &Pose_imu2ref,
                               const gtsam::Pose3 &CamPosec2i);
double CalCVDistance(cv::Point2f &pt1, cv::Point2f &pt2);
void GetDistortion(const Vector2d &mxy_d, Vector2d &d_u);
void LiftProjective(const Vector2d &uv, const int NumUnDisIter, Vector3d &xyz);
void ProjectUnDisXYZ2DisUV(const Vector3d &UnDisXYZ, Vector2d &DisUV);
void ProjectUnDisXYZ2UnDisUV(const Vector3d &UnDisXYZ, Vector2d &UnDisUV);
// UnDis_Dis_Flag, 0： UnDis; 1: Dis
bool ProjectUnDisXYZ2UV(const Vector3d &UnDisXYZ,
                        const eUnDis_Dis_Flag UnDis_Dis_Flag, Vector2d &UV);
void GetXYdistortbyXYnormalized(const Vector2d &xyn, Matrix2d &dxyd_dxyn);


Vector3d GTSAMRot2Att(const gtsam::Rot3 &gtRot3);
void TriangulateOneFt_DualFrame(const Vector2d &FeatureUnDisXYl,
                                const gtsam::Pose3 &Poself2w,
                                const Vector2d &FeatureUnDisXYr,
                                const gtsam::Pose3 &Poserf2w,
                                Vector3d &Point3w);
bool SolveFrameByPnP(const vector<cv::Point3f> &vCVPoint3ws,
                     const vector<cv::Point2f> &vCVUnDisXYs,
                     gtsam::Pose3 &Pose_fx2w_inital);

// // Unused functions
// void Point3w2InvDepth(const gtsam::Pose3 &CamPose,
//                       const gtsam::Vector3 &Point3w, gtsam::Vector3 &InvTPR,
//                       double &InvDepth);
// void UnDisXYZ_Point3w2InvDepth(const Vector3d UnDisXYZ,
//                                const gtsam::Pose3 &CamPose,
//                                const gtsam::Vector3 &Point3w,
//                                gtsam::Vector3 &InvTPR,
//                                double &InvDepth);
// double InvTPR2Depth(const Vector3d &InvTPR);
// void InvTPR2Point3w(const gtsam::Pose3 &CamPose, const Vector3d &InvTPR,
//                     gtsam::Vector3 &Point3w);
// gtsam::Vector3 SetInvDepth(const Vector3d UnDisXYZ, const double Depth);

// Test func
void TestSetTime1();  // int InitiImageId, int CurImageID, int IDRange_Int2Cur
void TestSetTime2AndShowTimeRange();

// void TestSetTime1(double InitiTime, double CurTime, double
// TimeRange_Int2Cur); void TestSetTime2AndShowTimeRange(double InitiTime,
// double CurTime,
//                                   double TimeRange_Int2Cur);

void TestTxt_CopyAndModifyWhenNeed();
}  // namespace VIG
