#pragma once

#include "Configuration.h"
#include "VIGCommon.h"

namespace VIG {
class Configuration;

class FeatureTracker {
 public:
  FeatureTracker(const Configuration *pConfig, ifstream &ifCamTime,
                 const double PreIMUTime);
  void ReadImageTimeHeader(ifstream &ifCamTime, const double PreIMUTime);
  void GrabNextImageEpoch(ifstream &ifCamTime);
  void ReadCurrentImage(const string ifCam);

  bool IsInBorder(const cv::Point2f &pt);
  template <typename T>
  void ReduceVector(vector<T> &v, vector<uchar> status);
  void CVFeat2V2dFeat(const vector<cv::Point2f> &vCV, vector<Vector2d> &vV2d);
  void RejectWithFundamentalMat();
  void SetMask();
  void SetMask2(const int MaskWidth);
  void AddFeatures();

  void Distortion(const Vector2d &p_u, Vector2d &d_u);
  void UndistorteFeatures();
  bool UpdateIDs(const unsigned int i);
  void TrackFeatures(const gtsam::Pose3 &Pose_c2w);
  pair<Frame, FrameWisedFeatures> GetTrackMoreFeatures();
  gtsam::Pose3 GetRelativePoseInCamFrm(const gtsam::Pose3 &Pose_sj2ref,
                                       const gtsam::Pose3 &Pose_sj_12ref,
                                       const gtsam::Pose3 &CamPose);
  void TriangulatePoint(const gtsam::Pose3 &Pose0, const gtsam::Pose3 &Pose1,
                        const Vector2d &point0, const Vector2d &point1,
                        Vector3d &point_3d);

 public:
  bool mbEqualize, mbFishEye;
  int mMaskWidth;
  cv::Mat mFishEyeMask, mMask;
  int mPubFrequency;
  double mMindistBtTwoFeatures = 30.0;
  double mFundamentalThreshold;
  int mMinFrames4Initial;  // minimum number of frames before initialization
  int mNumUnDisIter;

  int mFrameCounterId = 0;   // the frm position w.r.t the feature table
  int mNextFeatureId = 1;    // the next feature id to be used
  int mNextKeyFrameNum = 1;  // the next key frame number/ID to be used
  // string mstrOutFilePath;   // the full path of the file to store the
  // tracking table fileHandle;               // the file pointer to the output
  // file
  double mInitiDepthRho = 0.1;  // unit 1/m
  // string mstrSeqPath;       // image sequence path
  int mImageSeqType;  // image sequence type, 0 for image sequence and 1 for
                      // video
  // downScale;                // down sample the input image, so that each edge
  // is
  // // reduced to 1/(1<<downScale) of its original length
  // myVid;
  int mInitiImageId, mImageID, mLastImageID;
  double mInitiFrameEpoch, mFrameEpoch;

  int mPubCounter = 0;
  bool mbPubThisFrame = false;
  pair<Frame, FrameWisedFeatures>
      mprsCurFrameWisedFeatures;  // row corresponds to frames, columns to
                                  // points

  cv::Mat mPreImg, mCurImg, mInitiCurImg;
  vector<cv::Point2f> mvPreCVFeatures, mvCurCVFeatures, mvNewTrackedCVFeatures;
  static int mCurNumIDs;

 private:
  int mMaxFrameNumInBuffer;    // giving the table row number
  int mMaxFeatureNumPerFrame;  // maximum points in the table, giving the table
                               // col number
  int mMinFeatureNumInStates;  // minimum number of tracked points in the
                               // features_, i.e.,in the states,
  // before creating a new key frame of features by triangulation from the
  // feature table

  // TO identify: the following paraeters maybe just like the sliding windows in
  // VINS
  int mHalf_patch_size_when_initialized;
  int mHalf_patch_size_when_matching;
  int mExcluded_band =
      mHalf_patch_size_when_initialized +
      1;  // the border area that is allowed to have a feature point
};

}  // namespace VIG
