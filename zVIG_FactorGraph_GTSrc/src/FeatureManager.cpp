#include "FeatureManager.h"

namespace VIG {
FeatureManager::FeatureManager(const Configuration *pConfig) {
  mMinKeyFrameParallax = pConfig->mMinKeyFrameParallax / gFocalLength;
  ClearState();
}

void FeatureManager::ClearState() { mlsIdWisedFeatures.clear(); }

int FeatureManager::GetFeatureCount() {
  int cnt = 0;
  for (auto &lit : mlsIdWisedFeatures) {
    lit.UsedFramesNum = lit.vIdWisedFeatureInfoOnAllFrames.size();

    if (lit.UsedFramesNum >= gCoVisualFramesThreshold &&
        lit.StartFrameIDInKFQue < gMaxFrameIdInKFQue - 2) {
      cnt++;
    }
  }
  return cnt;
}

int FeatureManager::GetEndFrameId(const IdWisedFeature &sIdWisedFeature) {
  return sIdWisedFeature.StartFrameIDInKFQue +
         sIdWisedFeature.vIdWisedFeatureInfoOnAllFrames.size() - 1;
}

bool FeatureManager::AddIdWisedFeaturesAndCheckParallax(
    const int BackFrameIDInKFQue,
    const pair<Frame, FrameWisedFeatures> &prsCurFrameWisedFeatures) {
  // cout << "input mlsIdWisedFeatures: " <<
  // (int)prsCurFrameWisedFeatures.second.vFeatureIds.size(); cout << "num of
  // mlsIdWisedFeatures: " << getFeatureCount();
  mLastTrackNum = 0;

  auto &FrameId = prsCurFrameWisedFeatures.first.FrameId;
  auto &FeatureIds = prsCurFrameWisedFeatures.second.vFeatureIds;
  auto &UVs = prsCurFrameWisedFeatures.second.vUVs;
  auto &UnDisUVs = prsCurFrameWisedFeatures.second.vUnDisUVs;
  auto &XYZ = prsCurFrameWisedFeatures.second.vUnDisXYZs;
  // auto &InStatess = prsCurFrameWisedFeatures.second.InStatess;
  auto &TrackNums = prsCurFrameWisedFeatures.second.vTrackNums;

  //! 迭代单个特征点
  int NumCurFrameFeatures = FeatureIds.size();
  for (int itFt = 0; itFt < NumCurFrameFeatures; itFt++) {
    IdWisedFeatureOnCurFrame itIWFeatOnCurFrame;
    //! 特征点的归一化坐标
    itIWFeatOnCurFrame.CurFrameId = FrameId;
    itIWFeatOnCurFrame.UnDisUV = UnDisUVs[itFt];
    itIWFeatOnCurFrame.UnDisXYZ = Vector3d(XYZ[itFt](0), XYZ[itFt](1), 1.0);

    //! 在Features列表中寻找id为feature_id的Feature
    int FeatureId = FeatureIds[itFt];
    auto litIdWisedFeature =
        find_if(mlsIdWisedFeatures.begin(), mlsIdWisedFeatures.end(),
                [FeatureId](const IdWisedFeature &litIdWisedFeature) {
                  return litIdWisedFeature.FeatureId == FeatureId;
                });

    //! 如果该Feature不在Features列表中，则初始化相关属性,并将<FeatureID,Start_frame>存入到Feature列表中
    if (litIdWisedFeature == mlsIdWisedFeatures.end()) {
      mlsIdWisedFeatures.push_back(
          IdWisedFeature(FeatureId, BackFrameIDInKFQue));
      mlsIdWisedFeatures.back().vIdWisedFeatureInfoOnAllFrames.push_back(
          itIWFeatOnCurFrame);
    }
    //! 如果该Feature之前被观测到过
    else if (litIdWisedFeature->FeatureId == FeatureId) {
      litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames.push_back(
          itIWFeatOnCurFrame);
      mLastTrackNum++;
    }
  }

  double parallax_sum = 0;
  int parallax_num = 0;
  //! 如果滑窗内的关键帧的个数小于2或者总共被跟踪到的次数小于20
  if (BackFrameIDInKFQue < 2 || mLastTrackNum < 20) return true;

  for (auto &litIdWisedFeature : mlsIdWisedFeatures) {
    //! 至少有两帧观测到该特征点(起始帧要小于倒数第二帧，终止帧要大于倒数第二帧，这样至少有三帧观测到该Feature(包括当前帧))
    //! 然后比较观测到该faeature的倒数第二帧和倒数第三帧的视差
    if (litIdWisedFeature.StartFrameIDInKFQue <= BackFrameIDInKFQue - 2 &&
        litIdWisedFeature.StartFrameIDInKFQue +
                int(litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames.size()) -
                1 >=
            BackFrameIDInKFQue - 1) {
      parallax_sum +=
          compensatedParallax2(litIdWisedFeature, BackFrameIDInKFQue);
      parallax_num++;
    }
  }

  if (parallax_num == 0) {
    return true;
  } else {
    bool bParMoreThanMin =
        (parallax_sum / parallax_num >= mMinKeyFrameParallax);
    return bParMoreThanMin;
  }
}

vector<SFMFeature> FeatureManager::FormSFMFeatures() {
  vector<SFMFeature> vSFMFeatures;
  for (auto &litIdWisedFeature : mlsIdWisedFeatures) {
    if (litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames.size() <
        gCoVisualFramesThreshold)
      continue;

    int XthFrameIDInKFQue = litIdWisedFeature.StartFrameIDInKFQue - 1;
    SFMFeature TmpSFMFeature;
    TmpSFMFeature.bTriangulateState = false;
    TmpSFMFeature.FeatureId = litIdWisedFeature.FeatureId;
    TmpSFMFeature.StartFrameIDInKFQue = litIdWisedFeature.StartFrameIDInKFQue;

    for (auto &vitIdWisedFeaturePerFrame :
         litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames) {
      XthFrameIDInKFQue++;

      Vector2d IdWisedFeatureUnDisUVOnXthFr = vitIdWisedFeaturePerFrame.UnDisUV;
      TmpSFMFeature.mpKFID_UuvsOnAllFrms.insert(
          make_pair(XthFrameIDInKFQue, IdWisedFeatureUnDisUVOnXthFr));

      Vector3d IdWisedFeatureUnDisXYZOnXthFr =
          vitIdWisedFeaturePerFrame.UnDisXYZ;
      TmpSFMFeature.mpKFID_UxyzsOnAllFrms.insert(
          make_pair(XthFrameIDInKFQue, IdWisedFeatureUnDisXYZOnXthFr));
    }
    vSFMFeatures.push_back(TmpSFMFeature);
  }

  return vSFMFeatures;
}

vector<pair<Vector3d, Vector3d>> FeatureManager::GetCorresponding(
    const int LFrameCountIDInKFQue, const int RFrameCountIDInKFQue) {
  vector<pair<Vector3d, Vector3d>> vprCorresUnDisXYZPair;
  for (auto &litIdWisedFeature : mlsIdWisedFeatures) {
    if (litIdWisedFeature.StartFrameIDInKFQue <= LFrameCountIDInKFQue &&
        GetEndFrameId(litIdWisedFeature) >= RFrameCountIDInKFQue) {
      int EndFrameID = GetEndFrameId(litIdWisedFeature);
      Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
      int idx_l = LFrameCountIDInKFQue - litIdWisedFeature.StartFrameIDInKFQue;
      int idx_r = RFrameCountIDInKFQue - litIdWisedFeature.StartFrameIDInKFQue;

      a = litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames[idx_l].UnDisXYZ;
      b = litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames[idx_r].UnDisXYZ;

      vprCorresUnDisXYZPair.push_back(make_pair(a, b));
    }
  }
  return vprCorresUnDisXYZPair;
}

// Pose_L2R:  Point from Lfrm to Rfrm
bool FeatureManager::CalRelativePose_L2R(const int LFrameCountIDInKFQue,
                                         const int RFrameCountIDInKFQue,
                                         gtsam::Pose3 &Pose_L2R) {
  //！在滑窗内寻找与最新的关键帧共视点超过20(像素点)的关键帧
  // find previous frame which contians enough correspondance and parallex with
  // newest frame

  vector<pair<Vector3d, Vector3d>> vprCorresUnDisXYZPair;
  vprCorresUnDisXYZPair =
      GetCorresponding(LFrameCountIDInKFQue, RFrameCountIDInKFQue);
  //! 共视的Features应该大于20
  if (vprCorresUnDisXYZPair.size() > 20) {
    double sum_parallax = 0;
    double average_parallax;
    //! 求取匹配的特征点在图像上的视差和(归一化平面上)
    for (int j = 0; j < int(vprCorresUnDisXYZPair.size()); j++) {
      Vector2d pts_0(vprCorresUnDisXYZPair[j].first(0),
                     vprCorresUnDisXYZPair[j].first(1));
      Vector2d pts_1(vprCorresUnDisXYZPair[j].second(0),
                     vprCorresUnDisXYZPair[j].second(1));
      double parallax = (pts_0 - pts_1).norm();
      sum_parallax = sum_parallax + parallax;
    }
    //! 求取所有匹配的特征点的平均视差
    average_parallax = 1.0 * sum_parallax / int(vprCorresUnDisXYZPair.size());
    //! 视差大于一定阈值，并且能够有效地求解出变换矩阵
    if (SolveRelativePose_L2R(vprCorresUnDisXYZPair, Pose_L2R)) {
      // l = i;
      return true;
    }
  }

  return false;
}

// Rotation,Translation: Point from Lfrm to Rfrm
bool FeatureManager::SolveRelativePose_L2R(
    const vector<pair<Vector3d, Vector3d>> &vprCorresUnDisXYZPair,
    gtsam::Pose3 &Pose_L2R) {
  if (vprCorresUnDisXYZPair.size() >= 15) {
    vector<cv::Point2f> ll, rr;
    for (int i = 0; i < int(vprCorresUnDisXYZPair.size()); i++) {
      ll.push_back(cv::Point2f(vprCorresUnDisXYZPair[i].first(0),
                               vprCorresUnDisXYZPair[i].first(1)));
      rr.push_back(cv::Point2f(vprCorresUnDisXYZPair[i].second(0),
                               vprCorresUnDisXYZPair[i].second(1)));
    }
    cv::Mat mask;
    cv::Mat E = cv::findFundamentalMat(ll, rr, cv::FM_RANSAC,
                                       0.3 / gFocalLength, 0.99, mask);
    cv::Mat cameraMatrix =
        (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    cv::Mat rot, trans;
    int inlier_cnt = cv::recoverPose(E, ll, rr, cameraMatrix, rot, trans, mask);
    // cout << "inlier_cnt " << inlier_cnt << endl;

    Eigen::Matrix3d R_ll2rr;
    Eigen::Vector3d T_ll2rr;
    for (int i = 0; i < 3; i++) {
      T_ll2rr(i) = trans.at<double>(i, 0);
      for (int j = 0; j < 3; j++) R_ll2rr(i, j) = rot.at<double>(i, j);
    }

    gtsam::Rot3 Rot_ll2rr = gtsam::Rot3(R_ll2rr);
    gtsam::Point3 Trans_ll2rr = gtsam::Point3(T_ll2rr);
    Pose_L2R = gtsam::Pose3(Rot_ll2rr, Trans_ll2rr);

    if (inlier_cnt > 12)
      return true;
    else
      return false;
  }
  return false;
}

void FeatureManager::TriangulateCurFrameFts_MultFrames(
    const deque<KeyFrame> &dKeyFrames) {
  for (auto &litIdWisedFeature : mlsIdWisedFeatures) {
    litIdWisedFeature.UsedFramesNum =
        litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames.size();
    if (!(litIdWisedFeature.UsedFramesNum >= gCoVisualFramesThreshold &&
          litIdWisedFeature.StartFrameIDInKFQue < gMaxFrameIdInKFQue - 2))
      continue;
    if (litIdWisedFeature.InvDepth > 0) continue;

    int StartFrameIDInKFQue = litIdWisedFeature.StartFrameIDInKFQue,
        XthFrameIDInKFQue = StartFrameIDInKFQue - 1;

    MatrixXd svd_A(2 * litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames.size(),
                   4);
    int svd_idx = 0;
    for (auto &vitIdWisedFeaturePerFrame :
         litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames) {
      XthFrameIDInKFQue++;
      if (vitIdWisedFeaturePerFrame.CurFrameId !=
          dKeyFrames[XthFrameIDInKFQue].FrameId) {
        cerr << "Wrong frame id for current feature!";
        exit(1);
      }
      //! w ==> cam_x  ( 世界(n)到 cam_x )
      gtsam::Pose3 Pose_c02cx = dKeyFrames[XthFrameIDInKFQue].Pose.inverse() *
                                dKeyFrames[StartFrameIDInKFQue].Pose;
      Matrix<double, 4, 4> Pose_c02cx_mt = Pose_c02cx.matrix();

      Vector3d XthUnDisXYZ =
          vitIdWisedFeaturePerFrame.UnDisXYZ.normalized(); 
      svd_A.row(svd_idx++) = XthUnDisXYZ[0] * Pose_c02cx_mt.row(2) -
                             XthUnDisXYZ[2] * Pose_c02cx_mt.row(0);
      svd_A.row(svd_idx++) = XthUnDisXYZ[1] * Pose_c02cx_mt.row(2) -
                             XthUnDisXYZ[2] * Pose_c02cx_mt.row(1);
    }
    assert(svd_idx == svd_A.rows());
    Vector4d svd_V =
        JacobiSVD<MatrixXd>(svd_A, ComputeThinV).matrixV().rightCols<1>();

    double DepthOfStartFrame = svd_V[2] / svd_V[3];
    litIdWisedFeature.InvDepth =1./ DepthOfStartFrame;
    // error
    if (litIdWisedFeature.InvDepth < 0.01) {
      litIdWisedFeature.SolvedFlag = FAILED_SOLVED;
      continue;
    }

    litIdWisedFeature.Point3w =
        dKeyFrames[litIdWisedFeature.StartFrameIDInKFQue].Pose.transform_from(
            1. / litIdWisedFeature.InvDepth *
            litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames[0].UnDisXYZ);
    // Test
  }
}

bool FeatureManager::SolvePnP4AllKeyFrames(
    deque<pair<Frame, FrameWisedFeatures>> &dprTrackedFrames,
    const deque<KeyFrame> &dKeyFrames, const gtsam::Pose3 *arCVPose_fx2w,
    map<int, Vector3d> &mpSFMFeatIDs_Point3ws) {
  deque<pair<Frame, FrameWisedFeatures>>::iterator ditprsTrackedFrame;
  map<int, Vector3d>::iterator mpitSFMFeatIDs_Point3w;
  ditprsTrackedFrame = dprTrackedFrames.begin();
  for (int i = 0; ditprsTrackedFrame != dprTrackedFrames.end();
       ditprsTrackedFrame++) {
    //！如果帧头和滑窗内关键帧的帧头相同，则直接读取位姿即可
    // provide initial guess
    cv::Mat r_w2cx, rvec_w2cx, t_w2cx, D, tmp_r;
    if ((ditprsTrackedFrame->first.FrameId) == dKeyFrames[i].FrameId) {
      //！一次性转换到IMU坐标系下
      // ditprsTrackedFrame->second.bIsKeyFrame = true;
      ditprsTrackedFrame->first.Pose_c2w = arCVPose_fx2w[i];
      i++;
      continue;
    }
    if ((ditprsTrackedFrame->first.FrameId) > dKeyFrames[i].FrameId) {
      i++;
    }

    //! 将滑窗内第i帧的变换矩阵当做初始值
    gtsam::Pose3 Pose_w2x = arCVPose_fx2w[i].inverse();
    cv::eigen2cv(Pose_w2x.rotation().matrix(), tmp_r);
    cv::Rodrigues(tmp_r, rvec_w2cx);
    cv::eigen2cv(Vector3d(Pose_w2x.translation().matrix()), t_w2cx);

    //! 如果这部分位姿不作为关键帧的化，求解出来就没有意义啊
    // ditprsTrackedFrame->second.bIsKeyFrame = false;
    vector<cv::Point3f> vCVPoint3ws;
    vector<cv::Point2f> vCVUnDisXYs;

    //! 遍历该帧的所有Features
    int NumFeaturesOnXthFrame =
        (int)ditprsTrackedFrame->second.vFeatureIds.size();
    for (int i = 0; i < NumFeaturesOnXthFrame; i++) {
      int FeatureId = ditprsTrackedFrame->second.vFeatureIds[i];

      mpitSFMFeatIDs_Point3w = mpSFMFeatIDs_Point3ws.find(FeatureId);
      if (mpitSFMFeatIDs_Point3w != mpSFMFeatIDs_Point3ws.end()) {
        Vector3d world_pts = mpitSFMFeatIDs_Point3w->second;
        cv::Point3f CVPoint3w(world_pts(0), world_pts(1), world_pts(2));
        vCVPoint3ws.push_back(CVPoint3w);

        //！
        //注意这里的2D点img_pts是去畸变归一化平面上的点,而不是常规的像素平面的点
        // 因为下面的内参K已设置为单位阵，畸变矩阵D已设置为空
        Vector2d UnDisXY = ditprsTrackedFrame->second.vUnDisXYZs[i].head(2);
        cv::Point2f CVUnDisXY(UnDisXY(0), UnDisXY(1));
        vCVUnDisXYs.push_back(CVUnDisXY);
      }
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    if (vCVPoint3ws.size() < 6) {
      cout << "vCVPoint3ws size " << vCVPoint3ws.size() << endl;
      cout << "Not enough points for solve pnp !" << endl;
      return false;
    }
    if (!cv::solvePnP(vCVPoint3ws, vCVUnDisXYs, K, D, rvec_w2cx, t_w2cx, 1)) {
      cout << "solve pnp fail!" << endl;
      return false;
    }
    //! PnP求解出的位姿要取逆
    cv::Rodrigues(rvec_w2cx, r_w2cx);
    MatrixXd Rpnp_w2cx;
    cv::cv2eigen(r_w2cx, Rpnp_w2cx);
    MatrixXd Tpnp_w2cx;
    cv::cv2eigen(t_w2cx, Tpnp_w2cx);

    gtsam::Rot3 Rot_w2cx = gtsam::Rot3(Rpnp_w2cx);
    gtsam::Point3 Trans_w2cx = gtsam::Point3(Tpnp_w2cx);
    gtsam::Pose3 Pose_w2cx = gtsam::Pose3(Rot_w2cx, Trans_w2cx);

    ditprsTrackedFrame->first.Pose_c2w = Pose_w2cx.inverse();
  }
  return true;
}

bool FeatureManager::SolvePnP4CurFrameInKFQue(
    const pair<Frame, FrameWisedFeatures> &prCurTrackedFrame,
    gtsam::Pose3 &CurPose_fx2w_inital) {
  vector<cv::Point2f> vCVUnDisXYs;
  vector<cv::Point3f> vCVPoint3ws;

  int NumCurFrameFeatures = prCurTrackedFrame.second.vFeatureIds.size();
  for (int itFt = 0; itFt < NumCurFrameFeatures; itFt++) {
    int FeatureId = prCurTrackedFrame.second.vFeatureIds[itFt];
    auto litIdWisedFeature =
        find_if(mlsIdWisedFeatures.begin(), mlsIdWisedFeatures.end(),
                [FeatureId](const IdWisedFeature &litIdWisedFeature) {
                  return litIdWisedFeature.FeatureId == FeatureId;
                });

    if (litIdWisedFeature->InvDepth < 0.01)
      continue;

    if (litIdWisedFeature != mlsIdWisedFeatures.end()) {
      //！
      //注意这里的2D点img_pts是去畸变归一化平面上的点,而不是常规的像素平面的点
      // 因为下面的内参K已设置为单位阵，畸变矩阵D已设置为空
      Vector2d UnDisXY = prCurTrackedFrame.second.vUnDisXYZs[itFt].head(2);
      cv::Point2f CVUnDisXY(UnDisXY(0), UnDisXY(1));
      vCVUnDisXYs.push_back(CVUnDisXY);
      cv::Point3f CVPoint3w(litIdWisedFeature->Point3w[0],
                            litIdWisedFeature->Point3w[1],
                            litIdWisedFeature->Point3w[2]);
      vCVPoint3ws.push_back(CVPoint3w);
    }
  }

  //! 若第i帧中包含的sfm_f中的特征点不足10，则直接返回
  if (int(vCVUnDisXYs.size()) < 15) {
    printf("unstable features tracking, please slowly move you device!\n");
    if (int(vCVUnDisXYs.size()) < 10) return false;
  }

  if (!SolveFrameByPnP(vCVPoint3ws, vCVUnDisXYs, CurPose_fx2w_inital))
    return false;

  return true;
}

void FeatureManager::SetFeaturesDepth(gtsam::Values &ResultValues,
                                      deque<KeyFrame> &dKeyFrames) {
  // Update 3D Points in World frame
  for (auto &litIdWisedFeature : mlsIdWisedFeatures) {
    // Mark the outliers
    litIdWisedFeature.UsedFramesNum =
        litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames.size();

    // oldest orphan
    if (litIdWisedFeature.UsedFramesNum < 3 &&
        litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames.back().CurFrameId <=
            dKeyFrames.back().FrameId - 3) {
      litIdWisedFeature.SolvedFlag = FAILED_SOLVED;
      continue;
    }

    // If the features have been solved just now
    if (!(litIdWisedFeature.UsedFramesNum >= gCoVisualFramesThreshold &&
          litIdWisedFeature.StartFrameIDInKFQue < gMaxFrameIdInKFQue - 2 &&
          litIdWisedFeature.SolvedFlag == WATING_FOR_SOLVE) ||
        litIdWisedFeature.MapPointIndex < 0)
      continue;

    // 1. set the point3 in world frame
    // // Test
    // cout << "l" << litIdWisedFeature.MapPointIndex << ":" << endl;
    // Vector3d DeltaInvDepth = litIdWisedFeature.InvDepth3 -
    // ResultValues.at<gtsam::Vector3>(L(litIdWisedFeature.MapPointIndex)); cout
    // << "DeltaInvDepth: " << DeltaInvDepth.transpose() << endl;

    litIdWisedFeature.InvDepth =
        ResultValues.at<double>(L(litIdWisedFeature.MapPointIndex));

    // // Test
    // Vector3d DeltaPoint3w = litIdWisedFeature.Point3w -
    // InvTPR2Point3w(dKeyFrames[litIdWisedFeature.StartFrameIDInKFQue].Pose,
    //                                                                    litIdWisedFeature.InvDepth3);
    // cout << "DeltaPoint3w: " << DeltaPoint3w.transpose() << endl;

    litIdWisedFeature.Point3w =
        dKeyFrames[litIdWisedFeature.StartFrameIDInKFQue].Pose.transform_from(
            1. / litIdWisedFeature.InvDepth *
            litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames[0].UnDisXYZ);

    // 2. calculate the depth in the start frame in SW and set SolvedFlag
    if ((litIdWisedFeature.InvDepth < 0.01)) {
      litIdWisedFeature.SolvedFlag = FAILED_SOLVED;
    } else {
      litIdWisedFeature.SolvedFlag = OK_SOLVED;
    }
  }
}

void FeatureManager::RemoveFront(const int ErsFrmID) {
  for (auto litIdWisedFeature = mlsIdWisedFeatures.begin(),
            litIdWisedFeatureNext = mlsIdWisedFeatures.begin();
       litIdWisedFeature != mlsIdWisedFeatures.end();
       litIdWisedFeature = litIdWisedFeatureNext) {
    litIdWisedFeatureNext++;

    // // Test
    // if (litIdWisedFeature->MapPointIndex == 55)
    //         cout << "wait" << endl;

    if (litIdWisedFeature->StartFrameIDInKFQue != 0)
      litIdWisedFeature->StartFrameIDInKFQue--;
    else {
      assert(litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames[0].CurFrameId ==
             ErsFrmID);

      litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames.erase(
          litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames.begin());
      if (litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames.size() == 0)
        mlsIdWisedFeatures.erase(litIdWisedFeature);
    }
  }
}

void FeatureManager::RemoveBack(const int ErsFrmID,
                                const int BackFrameIDInKFQue) {
  for (auto litIdWisedFeature = mlsIdWisedFeatures.begin(),
            litIdWisedFeatureNext = mlsIdWisedFeatures.begin();
       litIdWisedFeature != mlsIdWisedFeatures.end();
       litIdWisedFeature = litIdWisedFeatureNext) {
    litIdWisedFeatureNext++;

    if (litIdWisedFeature->StartFrameIDInKFQue == BackFrameIDInKFQue) {
      litIdWisedFeature->StartFrameIDInKFQue--;
    } else {
      int j = gMaxFrameIdInKFQue - 1 - litIdWisedFeature->StartFrameIDInKFQue;
      if (GetEndFrameId(*litIdWisedFeature) < BackFrameIDInKFQue - 1) continue;

      auto jthFrm =
          litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames.begin() + j;
      assert(jthFrm->CurFrameId == ErsFrmID);
      litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames.erase(jthFrm);
      if (litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames.size() == 0)
        mlsIdWisedFeatures.erase(litIdWisedFeature);
    }
  }
}

void FeatureManager::RemoveFailures() {
  for (auto litIdWisedFeature = mlsIdWisedFeatures.begin(),
            litIdWisedFeatureNext = mlsIdWisedFeatures.begin();
       litIdWisedFeature != mlsIdWisedFeatures.end();
       litIdWisedFeature = litIdWisedFeatureNext) {
    litIdWisedFeatureNext++;
    if (litIdWisedFeature->SolvedFlag == FAILED_SOLVED)
      mlsIdWisedFeatures.erase(litIdWisedFeature);
  }
}

void FeatureManager::RemoveOutlier() {
  int i = -1;
  for (auto litIdWisedFeature = mlsIdWisedFeatures.begin(),
            litIdWisedFeatureNext = mlsIdWisedFeatures.begin();
       litIdWisedFeature != mlsIdWisedFeatures.end();
       litIdWisedFeature = litIdWisedFeatureNext) {
    litIdWisedFeatureNext++;
    i += litIdWisedFeature->UsedFramesNum != 0;
    if (litIdWisedFeature->UsedFramesNum != 0 &&
        litIdWisedFeature->bIsOutlier == true) {
      mlsIdWisedFeatures.erase(litIdWisedFeature);
    }
  }
}

double FeatureManager::compensatedParallax2(
    const IdWisedFeature &litIdWisedFeature, int BackFrameIDInKFQue) {
  // check the second last frame is keyframe or not
  // Parallax betwwen seconde last frame and third last frame
  const IdWisedFeatureOnCurFrame &FeatureOn3thLastFrame =
      litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames
          [BackFrameIDInKFQue - 2 - litIdWisedFeature.StartFrameIDInKFQue];
  const IdWisedFeatureOnCurFrame &FeatureOn2thLastFrame =
      litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames
          [BackFrameIDInKFQue - 1 - litIdWisedFeature.StartFrameIDInKFQue];

  double ans = 0;
  Vector3d p_2th = FeatureOn2thLastFrame.UnDisXYZ;

  double u_2th = p_2th(0);
  double v_2th = p_2th(1);

  Vector3d p_3th = FeatureOn3thLastFrame.UnDisXYZ;
  Vector3d p_3th_comp;

  p_3th_comp = p_3th;
  double dep_3th = p_3th(2);
  double u_3th = p_3th(0) / dep_3th;
  double v_3th = p_3th(1) / dep_3th;
  double du = u_3th - u_2th, dv = v_3th - v_2th;

  double dep_3th_comp = p_3th_comp(2);
  double u_3th_comp = p_3th_comp(0) / dep_3th_comp;
  double v_3th_comp = p_3th_comp(1) / dep_3th_comp;
  double du_comp = u_3th_comp - u_2th, dv_comp = v_3th_comp - v_2th;

  ans = max(
      ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

  return ans;
}

}  // namespace VIG
