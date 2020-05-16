#include "FeatureTracker.h"

namespace VIG {

int FeatureTracker::mCurNumIDs = 0;  // start from 0

FeatureTracker::FeatureTracker(const Configuration *pConfig,
                               ifstream &ifCamTime, const double PreIMUTime) {
  mbEqualize = pConfig->mbEqualize;
  mbFishEye = pConfig->mbFishEye;
  if (mbFishEye) mFishEyeMask = cv::imread(pConfig->mstrFishEyeMask, 0);

  mPubFrequency = pConfig->mPubFrequency;
  mFundamentalThreshold = pConfig->mFundamentalThreshold;
  mNumUnDisIter = pConfig->mNumUnDisIter;

  //  outFilePath = filename;
  //  fileHandle = fopen(filename, 'W'); // feature table
  // mstrSeqPath = pConfig-;
  // mMaxFrameNumInBuffer = pConfig->mMaxFrameNumInBuffer;
  mMaxFeatureNumPerFrame = pConfig->mMaxFeatureNumPerFrame;
  mMinFeatureNumInStates = pConfig->mMinFeatureNumInStates;
  // mMinFrames4Initial = pConfig->mMinFrames4Initial;

  mImageSeqType = pConfig->mImageSeqType;

  mImageID = pConfig->mInitiImageID;
  mLastImageID = pConfig->mLastImageID;

  if (!ifCamTime.eof()) {
    string strCamTimeheader, strCamTimeLine;

    int tmpInitiImageId;
    double time;

    getline(ifCamTime, strCamTimeheader);  // skip header

    if (!ifCamTime.eof()) getline(ifCamTime, strCamTimeLine);

    if (!strCamTimeLine.empty()) {
      stringstream ss;
      ss << strCamTimeLine;
      ss >> tmpInitiImageId;
      ss >> time;
    }

    while ((time < PreIMUTime || tmpInitiImageId < mImageID) &&
           !ifCamTime.eof()) {
      getline(ifCamTime, strCamTimeLine);
      if (!strCamTimeLine.empty()) {
        stringstream ss;
        ss << strCamTimeLine;
        ss >> tmpInitiImageId;
        ss >> time;
      }
    }

    mInitiImageId = tmpInitiImageId;
    mImageID = tmpInitiImageId;
    mInitiFrameEpoch = time;
    mFrameEpoch = time;

    return;
  }
  mFrameEpoch = DBL_MAX;
}

void FeatureTracker::GrabNextImageEpoch(ifstream &ifCamTime) {
  if (mImageID < mLastImageID) {
    string strCamTimeLine;
    int tmpImageId;
    double time;

    if (!ifCamTime.eof()) getline(ifCamTime, strCamTimeLine);

    if (!strCamTimeLine.empty()) {
      stringstream ss;
      ss << strCamTimeLine;
      ss >> tmpImageId;
      ss >> time;
    }

    mImageID = tmpImageId;
    mFrameEpoch = time;
  }
  // right now variance 'time' is the time of this mImageID image
  else {
    mFrameEpoch = DBL_MAX;  // stop using images
                            // close(outputVideo);
  }
}

void FeatureTracker::ReadCurrentImage(const string ifCam) {
  // Read Current Image
  switch (mImageSeqType) {
    case 0: {
      mInitiCurImg = cv::imread(ifCam + to_string(mImageID) + ".png", 0);
      break;
    }
    case 1: {
      // mInitiCurImg = takeimagefromvideo(tracker.myVid, frmId,
      // tracker.downScale);
      cout << "Video format is not supported yet!" << endl;
      break;
    }
    default: {
      cout << "Unknown Image path or format!" << endl;
      exit(1);
    }
  }

  if (mInitiCurImg.empty()) {
    cerr << "Current Image is empty!";
    exit(1);
  }
}

void FeatureTracker::TrackFeatures(const gtsam::Pose3 &Pose_c2w) {
  if (round(1.0 * mPubCounter / (mFrameEpoch - gFirstPubFrameEpoch)) <=
      mPubFrequency) {
    mbPubThisFrame = true;
    // reset the frequency control
    if (abs(1.0 * mPubCounter / (mFrameEpoch - gFirstPubFrameEpoch) -
            mPubFrequency) < 0.01 * mPubFrequency) {
      gFirstPubFrameEpoch = mFrameEpoch;
      mPubCounter = 0;
    }
  } else
    mbPubThisFrame = false;

  mprsCurFrameWisedFeatures.first.FrameId = mImageID;
  mprsCurFrameWisedFeatures.first.FrameEpoch = mFrameEpoch;
  mprsCurFrameWisedFeatures.first.Pose_c2w = Pose_c2w;

  cv::Mat CurImg;
  if (mbEqualize) {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(mInitiCurImg, CurImg);
  } else
    CurImg = mInitiCurImg;

  if (mCurImg.empty()) {
    mPreImg = mCurImg = CurImg;
  } else {
    mCurImg = CurImg;
  }

  mvCurCVFeatures.clear();
  // tracking all the points given their previous image coordinates and
  // predicted image coordinates
  if (mvPreCVFeatures.size() > 0) {
    vector<uchar> status;
    vector<float> err;
    // // uni-directional optical flow
    // cv::calcOpticalFlowPyrLK(mPreImg, mCurImg, mvPreCVFeatures,
    // mvCurCVFeatures, status,
    //                          err, cv::Size(21, 21), 3);
    /***** Start: bi-directional optical flow ****/
    cv::calcOpticalFlowPyrLK(mPreImg, mCurImg, mvPreCVFeatures, mvCurCVFeatures,
                             status, err, cv::Size(21, 21), 1);

    int succ_num = 0;
    for (size_t i = 0; i < status.size(); i++) {
      if (status[i]) succ_num++;
    }
    if (succ_num < 10)
      cv::calcOpticalFlowPyrLK(mPreImg, mCurImg, mvPreCVFeatures,
                               mvCurCVFeatures, status, err, cv::Size(21, 21),
                               3);

    vector<uchar> reverse_status;
    vector<cv::Point2f> reverse_pts = mvPreCVFeatures;
    cv::calcOpticalFlowPyrLK(
        mCurImg, mPreImg, mvCurCVFeatures, reverse_pts, reverse_status, err,
        cv::Size(21, 21), 1,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    for (size_t i = 0; i < status.size(); i++) {
      if (status[i] && reverse_status[i] &&
          CalCVDistance(mvPreCVFeatures[i], reverse_pts[i]) <= 0.5) {
        status[i] = 1;
      } else
        status[i] = 0;
    }
    /***** End: bi-directional optical flow ****/

    for (int i = 0; i < int(mvCurCVFeatures.size()); i++)
      if (status[i] && !IsInBorder(mvCurCVFeatures[i])) status[i] = 0;

    ReduceVector(mvPreCVFeatures, status);
    ReduceVector(mvCurCVFeatures, status);
    ReduceVector(mprsCurFrameWisedFeatures.second.vFeatureIds, status);
    ReduceVector(mprsCurFrameWisedFeatures.second.vTrackNums, status);
  }

  for (auto &TrackNum : mprsCurFrameWisedFeatures.second.vTrackNums) TrackNum++;

  if (mbPubThisFrame) {
    mPubCounter++;

    RejectWithFundamentalMat();
    SetMask();
    // mMaskWidth = 13;
    // SetMask2(mMaskWidth);

    int MaxCnt =
        mMaxFeatureNumPerFrame - static_cast<int>(mvCurCVFeatures.size());
    if (MaxCnt > 0) {
      if (mMask.empty()) cout << "Mask is empty " << endl;
      if (mMask.type() != CV_8UC1) cout << "Mask type wrong " << endl;
      if (mMask.size() != mCurImg.size()) cout << "wrong size " << endl;
      cv::goodFeaturesToTrack(mCurImg, mvNewTrackedCVFeatures,
                              mMaxFeatureNumPerFrame - mvCurCVFeatures.size(),
                              0.01, mMindistBtTwoFeatures, mMask);

      // cv::Size WinSize = cv::Size(5, 5);
      // cv::Size ZeroZone = cv::Size(-1, -1);
      // cv::TermCriteria Criteria = cv::TermCriteria(CV_TERMCRIT_EPS +
      // CV_TERMCRIT_ITER, 40, 0.001); cv::cornerSubPix(mCurImg,
      // mvNewTrackedCVFeatures, WinSize, ZeroZone, Criteria);
    } else
      mvNewTrackedCVFeatures.clear();

    AddFeatures();
  }
  mPreImg = mCurImg;
  mvPreCVFeatures = mvCurCVFeatures;
  UndistorteFeatures();

  // Update Feature Ids
  for (unsigned int i = 0;; i++) {
    bool completed = false;
    completed |= UpdateIDs(i);
    if (!completed) break;
  }
}

pair<Frame, FrameWisedFeatures> FeatureTracker::GetTrackMoreFeatures() {
  map<int, TrackMoreFeature> mpCurFrmTrackMoreFeatures;

  // Gather nice features( trackednum > 1 )
  if (mbPubThisFrame) {
    for (unsigned i = 0;
         i < mprsCurFrameWisedFeatures.second.vFeatureIds.size(); i++) {
      if (mprsCurFrameWisedFeatures.second.vTrackNums[i] > 1) {
        TrackMoreFeature sTrackMoreFeature;

        sTrackMoreFeature.UV = mprsCurFrameWisedFeatures.second.vUVs[i];
        sTrackMoreFeature.UnDisUV =
            mprsCurFrameWisedFeatures.second.vUnDisUVs[i];
        sTrackMoreFeature.UnDisXYZ =
            mprsCurFrameWisedFeatures.second.vUnDisXYZs[i];
        sTrackMoreFeature.TrackNum =
            mprsCurFrameWisedFeatures.second.vTrackNums[i];

        mpCurFrmTrackMoreFeatures[mprsCurFrameWisedFeatures.second
                                      .vFeatureIds[i]] = sTrackMoreFeature;
      }
    }
  }

  pair<Frame, FrameWisedFeatures> prsCurFrameWisedFeatures;
  prsCurFrameWisedFeatures.first = mprsCurFrameWisedFeatures.first;
  for (map<int, TrackMoreFeature>::iterator mpitTrkMFt =
           mpCurFrmTrackMoreFeatures.begin();
       mpitTrkMFt != mpCurFrmTrackMoreFeatures.end(); mpitTrkMFt++) {
    prsCurFrameWisedFeatures.second.vFeatureIds.push_back(mpitTrkMFt->first);
    prsCurFrameWisedFeatures.second.vbOptimized.push_back(0);
    prsCurFrameWisedFeatures.second.vTrackNums.push_back(
        mpitTrkMFt->second.TrackNum);
    prsCurFrameWisedFeatures.second.vUnDisUVs.push_back(
        mpitTrkMFt->second.UnDisUV);
    prsCurFrameWisedFeatures.second.vUnDisXYZs.push_back(
        mpitTrkMFt->second.UnDisXYZ);
    prsCurFrameWisedFeatures.second.vUVs.push_back(mpitTrkMFt->second.UV);
  }

  return prsCurFrameWisedFeatures;
}

bool FeatureTracker::IsInBorder(const cv::Point2f &pt) {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < gNCol - BORDER_SIZE &&
         BORDER_SIZE <= img_y && img_y < gNRow - BORDER_SIZE;
}

template <typename T>
void FeatureTracker::ReduceVector(vector<T> &v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i]) v[j++] = v[i];
  v.resize(j);
}

void FeatureTracker::CVFeat2V2dFeat(const vector<cv::Point2f> &vCV,
                                    vector<Vector2d> &vV2d) {
  vV2d.clear();
  for (int i = 0; i < static_cast<int>(vCV.size()); i++) {
    vV2d.push_back(Vector2d(vCV[i].x, vCV[i].y));
  }
  vV2d.resize(vCV.size());
}

void FeatureTracker::RejectWithFundamentalMat() {
  if (mvCurCVFeatures.size() >= 8) {
    vector<cv::Point2f> UnPreFts(mvPreCVFeatures.size()),
        UnCurFts(mvCurCVFeatures.size());
    for (unsigned int i = 0; i < mvPreCVFeatures.size(); i++) {
      Eigen::Vector3d tmp_p;

      LiftProjective(
          Eigen::Vector2d(mvPreCVFeatures[i].x, mvPreCVFeatures[i].y),
          mNumUnDisIter, tmp_p);
      tmp_p.x() = gFocalLength * tmp_p.x() / tmp_p.z() + gNCol / 2.0;
      tmp_p.y() = gFocalLength * tmp_p.y() / tmp_p.z() + gNRow / 2.0;
      UnPreFts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

      LiftProjective(
          Eigen::Vector2d(mvCurCVFeatures[i].x, mvCurCVFeatures[i].y),
          mNumUnDisIter, tmp_p);
      tmp_p.x() = gFocalLength * tmp_p.x() / tmp_p.z() + gNCol / 2.0;
      tmp_p.y() = gFocalLength * tmp_p.y() / tmp_p.z() + gNRow / 2.0;
      UnCurFts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
    }

    vector<uchar> status;
    cv::Mat F = cv::findFundamentalMat(UnPreFts, UnCurFts, cv::FM_RANSAC,
                                       mFundamentalThreshold, 0.99, status);

    ReduceVector(mvPreCVFeatures, status);
    ReduceVector(mvCurCVFeatures, status);
    ReduceVector(mprsCurFrameWisedFeatures.second.vFeatureIds, status);
    ReduceVector(mprsCurFrameWisedFeatures.second.vTrackNums, status);
  }
}

void FeatureTracker::SetMask() {
  if (mbFishEye)
    mMask = mFishEyeMask.clone();
  else
    mMask = cv::Mat(gNRow, gNCol, CV_8UC1, cv::Scalar(255));

  // prefer to keep features that are tracked for long time
  vector<pair<int, pair<cv::Point2f, int>>> vprTrackNums_prCurPt2fs_IDs;

  for (unsigned int i = 0; i < mvCurCVFeatures.size(); i++)
    vprTrackNums_prCurPt2fs_IDs.push_back(
        make_pair(mprsCurFrameWisedFeatures.second.vTrackNums[i],
                  make_pair(mvCurCVFeatures[i],
                            mprsCurFrameWisedFeatures.second.vFeatureIds[i])));

  sort(vprTrackNums_prCurPt2fs_IDs.begin(), vprTrackNums_prCurPt2fs_IDs.end(),
       [](const pair<int, pair<cv::Point2f, int>> &a,
          const pair<int, pair<cv::Point2f, int>> &b) {
         return a.first > b.first;
       });

  mvCurCVFeatures.clear();
  mprsCurFrameWisedFeatures.second.vFeatureIds.clear();
  mprsCurFrameWisedFeatures.second.vTrackNums.clear();

  for (auto &itprTrackNum_prCurPt2f_ID : vprTrackNums_prCurPt2fs_IDs) {
    if (mMask.at<uchar>(itprTrackNum_prCurPt2f_ID.second.first) == 255) {
      mvCurCVFeatures.push_back(itprTrackNum_prCurPt2f_ID.second.first);
      mprsCurFrameWisedFeatures.second.vFeatureIds.push_back(
          itprTrackNum_prCurPt2f_ID.second.second);
      mprsCurFrameWisedFeatures.second.vTrackNums.push_back(
          itprTrackNum_prCurPt2f_ID.first);
      cv::circle(mMask, itprTrackNum_prCurPt2f_ID.second.first,
                 mMindistBtTwoFeatures, 0, -1);
    }
  }
}

void FeatureTracker::SetMask2(const int MaskWidth) {
  cv::Rect Roi;
  GetRoi(MaskWidth, Roi);

  mMask = cv::Mat::zeros(gNRow, gNCol, CV_8UC1);
  mMask(Roi).setTo(255);
}

void FeatureTracker::AddFeatures() {
  for (auto &uv : mvNewTrackedCVFeatures) {
    mvCurCVFeatures.push_back(uv);
    mprsCurFrameWisedFeatures.second.vFeatureIds.push_back(-1);
    mprsCurFrameWisedFeatures.second.vTrackNums.push_back(1);
  }
}

void FeatureTracker::UndistorteFeatures() {
  mprsCurFrameWisedFeatures.second.vUVs.clear();
  mprsCurFrameWisedFeatures.second.vUnDisXYZs.clear();
  mprsCurFrameWisedFeatures.second.vUnDisUVs.clear();
  // cv::undistortPoints(mvCurCVFeatures, un_pts, K, cv::Mat());
  for (unsigned int i = 0; i < mvCurCVFeatures.size(); i++) {
    Vector2d UV(mvCurCVFeatures[i].x, mvCurCVFeatures[i].y);
    mprsCurFrameWisedFeatures.second.vUVs.push_back(UV);

    Vector3d UnDisXYZ;
    LiftProjective(UV, mNumUnDisIter, UnDisXYZ);
    UnDisXYZ /= UnDisXYZ(2);
    mprsCurFrameWisedFeatures.second.vUnDisXYZs.push_back(UnDisXYZ);

    Vector3d UnDisUV1 = gCamK * UnDisXYZ;
    mprsCurFrameWisedFeatures.second.vUnDisUVs.push_back(UnDisUV1.head(2));
  }
  mprsCurFrameWisedFeatures.second.vUVs.resize(mvCurCVFeatures.size());
  mprsCurFrameWisedFeatures.second.vUnDisXYZs.resize(mvCurCVFeatures.size());
  mprsCurFrameWisedFeatures.second.vUnDisUVs.resize(mvCurCVFeatures.size());
}

bool FeatureTracker::UpdateIDs(const unsigned int i) {
  if (i < mprsCurFrameWisedFeatures.second.vFeatureIds.size()) {
    if (mprsCurFrameWisedFeatures.second.vFeatureIds[i] == -1)
      mprsCurFrameWisedFeatures.second.vFeatureIds[i] = mCurNumIDs++;
    return true;
  } else
    return false;
}

gtsam::Pose3 FeatureTracker::GetRelativePoseInCamFrm(
    const gtsam::Pose3 &Pose_sj2ref, const gtsam::Pose3 &Pose_sj_12ref,
    const gtsam::Pose3 &CamPosec2i) {
  gtsam::Pose3 Pose_cj2cj_1;
  // the camera's pose c1 and c0 in ref frame
  gtsam::Rot3 R_cj2ref, R_cj_12ref, R_cj2cj_1;
  gtsam::Point3 T_cj2cj_1;

  R_cj2ref = Pose_sj2ref.rotation() * CamPosec2i.rotation();
  R_cj_12ref = Pose_sj_12ref.rotation() * CamPosec2i.rotation();

  // the relation between the frame c0 and the previous frame c1
  R_cj2cj_1 = R_cj_12ref.inverse() * R_cj2ref;
  T_cj2cj_1 =
      CamPosec2i.rotation().inverse() * (-CamPosec2i.translation()) -
      R_cj_12ref.inverse() *
          (Pose_sj_12ref.translation() - Pose_sj2ref.translation()) +
      R_cj2cj_1 * CamPosec2i.rotation().inverse() * CamPosec2i.translation();
  Pose_cj2cj_1 = gtsam::Pose3(R_cj2cj_1, T_cj2cj_1);

  return Pose_cj2cj_1;
}

void FeatureTracker::TriangulatePoint(const gtsam::Pose3 &Pose0,
                                      const gtsam::Pose3 &Pose1,
                                      const Vector2d &point0,
                                      const Vector2d &point1,
                                      Vector3d &point_3d) {
  Matrix<double, 4, 4> Pose0_T, Pose1_T;
  Pose0_T = Pose0.matrix();
  Pose1_T = Pose1.matrix();

  Matrix4d design_matrix = Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * Pose0_T.row(2) - Pose0_T.row(0);
  design_matrix.row(1) = point0[1] * Pose0_T.row(2) - Pose0_T.row(1);
  design_matrix.row(2) = point1[0] * Pose1_T.row(2) - Pose1_T.row(0);
  design_matrix.row(3) = point1[1] * Pose1_T.row(2) - Pose1_T.row(1);
  // cout<<"design_matrix:"<<endl<<design_matrix<<endl;
  Vector4d triangulated_point;
  triangulated_point =
      design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  // cout<<"triangulated_point:"<<endl<<triangulated_point<<endl;
  point_3d(0) = triangulated_point(0) / triangulated_point(3);
  point_3d(1) = triangulated_point(1) / triangulated_point(3);
  point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

}  // namespace VIG
