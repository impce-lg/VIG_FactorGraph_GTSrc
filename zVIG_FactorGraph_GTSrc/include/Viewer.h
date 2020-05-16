#pragma once

#include <pangolin/pangolin.h>

#include <mutex>

#include "FeatureManager.h"
#include "VIGCommon.h"

namespace VIG {

class Viewer {
 public:
  Viewer(const string &ConfigFilename);
  void InitiViewer(const gtsam::Pose3 &InitFramePose_c2w);
  void RunViewer(const FeatureManager *pFeatureManager,
                 const deque<KeyFrame> &dKeyFrames,
                 const pair<Frame, FrameWisedFeatures> &prsCurTrackedFrame,
                 const cv::Mat &CurImg);
  void GetCurrentOpenGLCameraMatrix(const gtsam::Pose3 &sCurCamPose_c2w);
  void DrawCurrentCamera();
  void DrawKeyFrames(const bool bDrawKF, const deque<KeyFrame> dKeyFrames);
  void DrawMapPoints(const FeatureManager *pFeatureManager,
                     const FrameWisedFeatures &CurFrameWisedFeatures);
  cv::Mat DrawFrame(const FrameWisedFeatures &CurFrameWisedFeatures,
                    const cv::Mat &CurImg);
  void DrawTextInfo(const cv::Mat &im, const int nState, cv::Mat &imText);
  int NumOptimized() { return mNumOptimized; }

 private:
  // 1/fps in ms
  double mT;
  float mImageWidth, mImageHeight;
  float mCameraSize, mCameraLineWidth, mKeyFrameSize, mKeyFrameLineWidth,
      mPointSize;
  float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
  bool mbWriteVideo;
  cv::VideoWriter mVideoWriter;

  pangolin::Var<bool> *mmenuFollowCamera, *mmenuShowFeatures,
      *mmenuShowKeyFrames, *mmenuReset;

  pangolin::OpenGlRenderState mS_cam;
  pangolin::View mD_cam;
  pangolin::OpenGlMatrix mTc2w;

  int mNumMPs, mNumKFs, mNumOptimized, mNumTracked, mNumRejected;
  int mCurFrameId;

  bool mbFollow;
  bool mbFinishRequested;
  bool mbFinished;
  // std::mutex mMutexFinish;

  bool mbStopped;
  bool mbStopRequested;
  // std::mutex mMutexStop;
};
}  // namespace VIG
