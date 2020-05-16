#pragma once

#include "Configuration.h"
#include "VIGCommon.h"

namespace VIG {
class FeatureManager {
 public:
  FeatureManager(const Configuration *pConfig);

  void ClearState();
  int GetFeatureCount();
  int GetEndFrameId(const IdWisedFeature &sIdWisedFeature);

  bool AddIdWisedFeaturesAndCheckParallax(
      const int BackFrameIDInKFQue,
      const pair<Frame, FrameWisedFeatures> &prsCurFrameWisedFeatures);

  vector<SFMFeature> FormSFMFeatures();
  vector<pair<Vector3d, Vector3d>> GetCorresponding(
      const int LFrameCountIDInKFQue, const int RFrameCountIDInKFQue);
  bool CalRelativePose_L2R(int LFrameCountIDInKFQue, int RFrameCountIDInKFQue,
                           gtsam::Pose3 &Pose_L2R);
  bool SolveRelativePose_L2R(const vector<pair<Vector3d, Vector3d>> &corres,
                             gtsam::Pose3 &Pose_L2R);
  void TriangulateCurFrameFts_MultFrames(const deque<KeyFrame> &dKeyFrames);
  bool SolvePnP4AllKeyFrames(
      deque<pair<Frame, FrameWisedFeatures>> &dprTrackedFrames,
      const deque<KeyFrame> &dKeyFrames, const gtsam::Pose3 *arCVPose_fx2w,
      map<int, Vector3d> &mpSFMFeatIDs_Point3ws);
  bool SolvePnP4CurFrameInKFQue(
      const pair<Frame, FrameWisedFeatures> &prCurTrackedFrame,
      gtsam::Pose3 &CurPose_fx2w_inital);
  void SetFeaturesDepth(gtsam::Values &ResultValues,
                        deque<KeyFrame> &dKeyFrames);
  void RemoveFront(const int ErsFrmID);
  void RemoveBack(const int ErsFrmID, const int BackFrameIDInKFQue);
  void RemoveFailures();
  void RemoveOutlier();

 public:
  double mMinKeyFrameParallax;
  list<IdWisedFeature> mlsIdWisedFeatures;
  int mLastTrackNum;

 private:
  double compensatedParallax2(const IdWisedFeature &it_per_id,
                              int BackFrameIDInKFQue);
};

}  // namespace VIG
