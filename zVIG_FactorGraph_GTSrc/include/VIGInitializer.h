#pragma once

#include "FactorGraphEstimator.h"
#include "GNSS.h"
#include "IMU.h"
#include "VIGCommon.h"
#include "drPreintegratedImuMeasurements.h"

namespace VIG {
class IMU;
class GNSS;
class drPreintegratedImuMeasurements;

class VIGInitializer : public FactorGraphEstimator {
 public:
  VIGInitializer(const Configuration *pConfig, IMU *pIMU);
  void AddIMU(const IMU &CurIMU);
  void AddGNSS(const GNSS &CurGNSS);

  // optimize initial VIG tightly, without scale
  void AddVisualFactor_TightInit();
  bool InitializeVIGStructure_Tightly();

  // optimize initial VIG loosely, including scale, bias
  bool OptmAndUpdSFM_InDep();
  bool OptmAndUpdSFM_Point3w();
  void TriangCurSFMFeat_DualFrm(const int LFrame, const int RFrame,
                                SFMFeature &CurSFMFeature);
  void TriangAllSFMFeats_DualFrm(const int LFrame, const int RFrame);
  bool SolveXthFrameByPnP(const int FrameId, gtsam::Pose3 &Pose_fx2w_inital);
  bool CalRelativePose_l2latest(int &l, gtsam::Pose3 &Pose_l2latest);
  bool ConstructSFM();
  bool VisualIMUAlignment();
  bool InitializeVIGStructure_Loosely();

 public:
  gtsam::Cal3_S2::shared_ptr mK;
  vector<IMU> mvInitialIMUs;
  deque<GNSS> mdInitialGNSSs;

  vector<SFMFeature> mvSFMFeatures;
  int mNumSFMFeatures;
  deque<Frame> mdInitKeyFrames;
  map<int, gtsam::Pose3> mmpKFCVPose_c2w, mmpKFCVPose_c2w_unOpt;
  map<int, gtsam::Pose3> mmpCVPose_i2w;
  map<int, drPreintegratedImuMeasurements> mmpIMUPreIntBetwDualFrms;
};

}  // namespace VIG
