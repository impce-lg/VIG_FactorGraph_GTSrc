#pragma once

#include "Configuration.h"
#include "FeatureManager.h"
#include "GNSS.h"
#include "IMU.h"
#include "VIGCommon.h"

namespace VIG {
class Configuration;
class IMU;
class GNSS;
class FeatureManager;

class FactorGraphEstimator {
 public:
  FactorGraphEstimator(const Configuration *pConfiguration);
  void ClearVIGInitialization();
  gtsam::Pose3 GetCurPredPose();
  void GetGNSSSigmaFactor(const Vector3d &GNSSResidual,
                          Vector3d &GNSSSigmaFactor);
  void GetGNSSNewResThreshold(const Vector3d &GNSSResidual,
                              const Vector3d &GNSSSigmaFactor);
  bool CalcCurVisualNoiseModel(const gtsam::Pose3 &CurKFPose,
                               const gtsam::Point3 &Point3w,
                               const gtsam::Point2 &Measured_,
                               int CurMapPointIndex);
  void PreIntegrate(const IMU *pIMU);
  void AddNewVision(
      const pair<Frame, FrameWisedFeatures> &prsCurFrameWisedFeatures);
  void TriangulateCurFrameFeatures();

  void AddIMUFactor();
  void AddGNSSSolFactor(const GNSS *pGNSS);
  void AddVisualFactor();

  void UpdateNavStateTimeStamps_KeyTimeStampMap(double NavStateIMUTime);
  void InsertNewNavStateInitialValues();
  void GetFactorsWtLandMarkToRemove(const gtsam::VectorValues &CurDelta);
  void Optimize();
  void UpdateResults_FLSmoother(const gtsam::Values &FLResultValues);

  void UpdateKeyFramesAndFeatures();
  void SlideWindow();
  // void SaveNavStates(const double PreIMUTime, ofstream &fNavState);
  void SaveNavStates(ofstream &fNavState, ofstream &fIMUBias);

  // Test
  void PrintCurFactorGraph(gtsam::Values &CurLinearizationPoint);

 public:
  // struct FullNavInfo
  // {
  //   double T;
  //   gtsam::NavState NavSt;
  //   gtsam::imuBias::ConstantBias Bias;
  // };
  // Initial Conditions
  gtsam::Pose3 mPoseb2n;
  gtsam::Vector3 mVelb2n;

  gtsam::noiseModel::Diagonal::shared_ptr mNoiseModel_InitPose;
  gtsam::noiseModel::Isotropic::shared_ptr mNoiseModel_InitVel;
  gtsam::noiseModel::Diagonal::shared_ptr mNoiseModel_Initbias;

  // IMU Settings
  gtsam::noiseModel::Diagonal::shared_ptr mNoiseModel_BetwBias;
  // boost::shared_ptr<gtsam::PreintegrationParams> mIMU_params;
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
      mIMU_params;
  gtsam::PreintegratedImuMeasurements *mpIMUPreIntg;
  gtsam::Vector6 mInitSigma6_BetwBias;

  // GNSS Settings
  static int mGNSSCounter;
  gtsam::Vector3 mInitSigma3_GNSSpos;
  gtsam::noiseModel::Diagonal::shared_ptr mNoiseModel_GNSSpos;
  gtsam::Vector3 mGNSSResidulThreshold;
  double marGNSSResidulThresholdFactor[2];

  // Camera params
  gtsam::Pose3 mExtrPose_c2i;
  eMarginFlag meMarginFlag;
  int mInitWinSize;
  int mBackFrIDInKFQue;  //! Back frame ID in KeyFramesQueue，start from 0

  bool mbInitPub = 0, mbInitFeature = 0;
  pair<Frame, FrameWisedFeatures> mprsCurTrackedFrame;
  deque<pair<Frame, FrameWisedFeatures>> mdprTrackedFrames;
  deque<KeyFrame> mdKeyFrames;
  double mInitSigma_Pixel;
  gtsam::noiseModel::Isotropic::shared_ptr mNoiseModel_Pixel;
  gtsam::Vector2 mPixelResidulThreshold;
  double mvPixResiThreFactor[2];

  // Initialization
  // vector<double> mvprTimeStamp_SenorType;
  vector<IMU> mvInitialIMUs;
  deque<GNSS> mdInitialGNSSs;
  deque<pair<Frame, FrameWisedFeatures>> mdprInitialTrackedFrames;
  deque<KeyFrame> mdInitialKeyFrames;
  FeatureManager *mpFeatureManager;

  // Solver
  vector<double> mvNavStatesTimeStamps;
  static int mIMUCounterBetwDualObss;
  static int mNavStateIndex;
  static int mMapPointIndex;
  gtsam::imuBias::ConstantBias mOptimizedCurBias;
  gtsam::NavState mPredNextNavState4Init, mOptimizedCurNavState;
  gtsam::ISAM2Params mIsamParams;
  gtsam::Values mNewInitialValues;
  gtsam::NonlinearFactorGraph *mpNewGraph = new gtsam::NonlinearFactorGraph();
  gtsam::Values mResultValues;

  string mstrOptimizer;  // ISAM2, IncrementalFixedLagSmoother supportted now
  gtsam::ISAM2 mIsam2;
  double mSmootherLag;
  int mNumFLSmootherUpdIters;
  gtsam::IncrementalFixedLagSmoother mFLSmootherISAM2;
  gtsam::FixedLagSmoother::KeyTimestampMap mtdmpNewKey_TimeStamps;
  int mNewFactorIndex;
  gtsam::FactorIndices mFactorsWtLandMarkToRemove;
  // gtsam::BatchFixedLagSmoother mSmootherBatch;
  // int mRelinearizeInterval;
  // gtsam::Marginals mMarginals;

  // Test
};

}  // namespace VIG
