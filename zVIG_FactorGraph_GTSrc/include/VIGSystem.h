#pragma once

#include "Configuration.h"
#include "FactorGraphEstimator.h"
#include "FeatureManager.h"
#include "FeatureTracker.h"
#include "GNSS.h"
#include "IMU.h"
#include "TestVSlamSystem.h"
#include "VIGCommon.h"
#include "VIGInitializer.h"
#include "Viewer.h"

namespace VIG {
class Configuration;
class IMU;
class GNSS;
class FeatureTracker;
class FeatureManager;
class FactorGraphEstimator;
class VIGInitializer;
class Viewer;
class TestVSlamSystem;

class VIGSystem {
 public:
  VIGSystem(const string &ConfigFilename);
  bool IsAvailable();

  void PropagateIMU();
  void CoupleGNSS();
  void CoupleVision();
  void CoupleOtherSensors();
  void StepIMU();

  void OutputNavStates();
  void Exit();

 public:
  enum FilterRefFrame {
    EFrame = 0,
    NFrame = 1,
    S0Frame = 2
  } meOptimizerRefFrame;

  string mstrConfigFile;
  Configuration *mpConfiguration;
  ifstream mifIMU, mifGNSS, mifCamTime;
  string mifCam;

  // imu
  IMU *mpIMU;
  double mInitiTime;

  // GNSS
  GNSS *mpGNSS;
  int mUsedGNSSCounter;

  // Camera
  FeatureTracker *mpFeatureTracker;
  // bool mbLessThanInitialFrames = true;

  // Factor Graph Optimization
  bool mbHasIMUAdded, mbIsGNSSAvailable, mbIsVisualAvailable;
  eVIGEstimatorState meVIGEstimatorState;
  FactorGraphEstimator *mpFactorGraphEstimator;
  VIGInitializer *mpVIGInitializer;

  // visualizing
  // gtsam::Pose3 msCurCamPose_w2c;
  Viewer *mpViewer;

  // output
  vector<gtsam::Pose3> mvTrajectory;
  ofstream mofNavState, mofIMUBias, mofFeatureTable, mofCamPose;

  // Test
  bool mbVSlam;
  TestVSlamSystem *mpTestVSlamSystem;
};

}  // namespace VIG
