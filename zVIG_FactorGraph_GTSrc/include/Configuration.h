#pragma once

#include "VIGCommon.h"

namespace VIG {

class Configuration {
 public:
  Configuration(const string &ConfigFilename);
  void SetGlobalParas();

 public:
  // input files
  string mstrInputDirectory;
  string mstrIMUFile;
  string mstrGNSSFile;
  string mstrCamPath;
  string mstrCamTimeFile;
  // output files
  string mstrOutputDirectory;
  string mstrNavfile;
  string mstrIMUBiasfile;
  string mstrKMLFilename;
  string mstrFeatureTableFile;
  string mstrCamPoseFile;
  string mstrVideoName;
  string mstrNavFrame;
  int mIMUMisalignMode;  // 0:e-formulation,1:phi,2:psi,3:s0 local frame

  // IMU Configuration
  bool mbUseIMU;
  int mIMUFileType;  // imu file type, 0:rate,1:increment
  int mIMUType;      // imu type name, mems 3dm-gx-3-35
  double mIMUStartTime;
  double mIMUEndTime;
  double mIMUDt;  // sampling interval
  Matrix3d mCamera2IMU;
  Vector3d mTIMU2Camera;
  gtsam::Pose3 mExtrPose_c2i;
  IMUErrorStates mInitiIMUErrorStates;  // imu errors(ba,bg,sa,sg)

  // initial navigation info
  Vector3d mIniBLH;
  Vector3d mIniXYZ;
  Vector3d mIniVn;
  Quaterniond mIniQi2n;

  double mInitAttSigma;
  double mInitPosSigma;
  double mInitVelSigma;
  double mInitAccBiasSigma;
  double mInitGyroBiasSigma;
  double mInitAccBetwBiasSigma;
  double mInitGyroBetwBiasSigma;
  double mAccSigma;
  double mGyroSigma;
  double mIntgSigma;

  Vector3d mG;
  Vector3d mw_coriolis;

  // GNSS Configuration
  bool mbUseGNSS;
  bool mbUseGNSSStd;
  Vector3d mTimu2ant;
  double mGNSSNum;
  int mGNSSPosType;
  double mGNSSAttSigma;
  double mGNSSPosSigma;
  Vector3d mGNSSResidulThreshold;
  vector<double> mvGNSSResidulThresholdFactor;

  // Camera Configuration
  bool mbUseCam;
  int mInitWindowSize, mKeyFramesQueueSize;
  bool mbEqualize;
  bool mbFishEye;
  string mstrFishEyeMask;
  int mPubFrequency;
  int mNRow;
  int mNCol;
  Matrix3d mCamK;
  // [ Cam.fc(1)   Cam.alpha*Cam.fc(1)   Cam.cc(1)
  //   0           Cam.fc(2)             Cam.cc(2)
  //   0           0                     1   ];
  Eigen::Matrix<double, 5, 1> mCamDistCoef;
  double mSigma_Pixel;
  Vector2d mPixelResidulThreshold;
  vector<double> mvPixelResidulThresholdFactor;
  int mImageSeqType;
  int mInitiImageID;
  int mLastImageID;

  // Feature Tracking Configuration
  int mMaxFeatureNumPerFrame;  // maximum points in the table, giving the table
                               // col number
  int mMinFeatureNumInStates;  // minimum number of tracked points in the
                               // features_, i.e.,in the states,
  // before creating a new key frame of features by triangulation from the
  // feature table
  double mFundamentalThreshold;  // CV Find Fundamental Matrix Threshold
  int mNumUnDisIter;

  // FeatureManager Configuration
  double mFocalLength;
  double mMinKeyFrameParallax;
  int mCoVisualFramesThreshold;

  // Optimization Configuration
  string mstrOptimizationParams;  // "GaussNewton", "Dogleg" supportted now
  string
      mstrOptimizer;  // "ISAM2", "IncrementalFixedLagSmoother" supportted now
  int mIsamParams_relinearizeSkip;
  bool mbIsamParams_findUnusedFactorSlots;
  bool mbIsamParams_evaluateNonlinearError;
  int mNonLinearISAMRelinearizeInterval;
  double mSmootherLag;
  int mNumFLSmootherUpdIters;

  // Viewer Configuration
  bool mbUseViewer;

  // Test params
  bool mbVSlam;
  double mTestPauseEpoch;
  int mTestFramesThreshold;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace VIG
