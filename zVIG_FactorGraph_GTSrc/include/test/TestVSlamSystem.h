#pragma once

#include "VIGCommon.h"
#include "Configuration.h"
#include "FeatureTracker.h"
#include "FeatureManager.h"
#include "Viewer.h"

namespace VIG
{
class Configuration;
// class IMU;
// class GNSS;
class FeatureTracker;
class FeatureManager;
class Viewer;

class TestVSlamSystem
{
public:
    TestVSlamSystem(const Configuration *pConfiguration,
                    const double FirstNavStateIMUTime,
                    const string &ConfigFilename);
    void InitializeEstimator(const Configuration *pConfiguration);
    bool CalcCurVisualNoiseModel(const gtsam::Pose3 &CurKFPose,
                                 const gtsam::Point3 &Point3w,
                                 const gtsam::Vector2 &Measured,
                                 int CurMapPointIndex);
    void UpdatePoseTimeStamps_KeyTimeStampMap(double CurPoseTime);
    void RunVSlam(const Configuration *pConfiguration,
                  const IMU *pIMU,
                  FactorGraphEstimator *pFactorGraphEstimator);
    void AddCurTrackedFrameAndKeyFrame(const pair<Frame, FrameWisedFeatures> &prsCurFrameWisedFeatures);
    void AddIdWisedFeatures(const gtsam::Pose3 &CurPredPose);
    bool InitializeSLAMStructure();
    void AddVisualFactor_Init();
    void AddVisualFactor(const gtsam::Pose3 &CurPredPose);
    void InsertNewPoseInitialValues(const gtsam::Pose3 &PredNextPose4Init);
    void Optimize();
    void UpdateKeyFramesAndFeatures();
    void SlideWindow();

public:
    ifstream mifCamTime_;
    string mifCam_;

    gtsam::Pose3 mPoseb2n_;
    gtsam::noiseModel::Diagonal::shared_ptr mNoiseModel_InitPose_;

    // Camera
    int mBackFrameIDInKFQue_;
    int mInitWindowSize_;
    eMarginFlag meMarginFlag_;
    double mInitSigma_Pixel_;
    gtsam::noiseModel::Isotropic::shared_ptr mNoiseModel_Pixel_;
    deque<pair<Frame, FrameWisedFeatures>> mdprTrackedFrames_;
    deque<KeyFrame> mdKeyFrames_;
    pair<Frame, FrameWisedFeatures> mprsCurTrackedFrame_;

    FeatureTracker *mpFeatureTracker_;
    FeatureManager *mpFeatureManager_;
    Viewer *mpViewer_;

    eVIGEstimatorState meVIGEstimatorState_;
    vector<double> mvPosesTimeStamps_;
    static int mPoseIndex_;
    static int mMapPointIndex_;
    gtsam::Pose3 mPredNextPose4Init_, mOptimizedCurPose_;
    // gtsam::FixedLagSmoother::KeyTimestampMap mtdmpNewKey_TimeStamps_;

    gtsam::ISAM2Params mIsamParams_;
    gtsam::ISAM2 mVSlamIsam2_;
    gtsam::Values mVSlamNewInitialValues_;
    gtsam::NonlinearFactorGraph *mpVSlamNewGraph_ = new gtsam::NonlinearFactorGraph();
    gtsam::Values mVSlamResultValues_;
    
};

} // namespace VIG