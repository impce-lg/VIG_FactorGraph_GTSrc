#include "VIGInitializer.h"

namespace VIG {

VIGInitializer::VIGInitializer(const Configuration *pConfig, IMU *pIMU)
    : FactorGraphEstimator(pConfig) {
  mvInitialIMUs.push_back(*pIMU);
  // mdInitialGNSSs.push_back(*pGNSS);
  mK = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(gCamK(0, 0), gCamK(1, 1),
                                                     gCamK(0, 1) / gCamK(0, 0),
                                                     gCamK(0, 2), gCamK(1, 2)));
}

void VIGInitializer::AddIMU(const IMU &CurIMU) {
  mvInitialIMUs.push_back(CurIMU);
}

void VIGInitializer::AddGNSS(const GNSS &CurGNSS) {
  mdInitialGNSSs.push_back(CurGNSS);
}

bool VIGInitializer::OptmAndUpdSFM_Point3w() {
  // 1. Set isam2
  mtdmpNewKey_TimeStamps.clear();
  mpNewGraph->resize(0);
  mNewInitialValues.clear();
  mIsam2 = gtsam::ISAM2(mIsamParams);

  // 2. Add intial values
  int PoseIndex = -1;
  while (PoseIndex < (int)mdKeyFrames.size() - 1) {
    PoseIndex++;
    mNewInitialValues.insert(X(PoseIndex), mmpKFCVPose_c2w[PoseIndex]);
    mdKeyFrames[PoseIndex].NavStateIndex = PoseIndex;
    cout << PoseIndex << endl;
    mmpKFCVPose_c2w[PoseIndex].print("SFM");
    mdKeyFrames[PoseIndex].Pose.print("IMUPre");
    cout << endl;
  }

  // 3. Add factors
  // 3.1 Add prior factors
  gtsam::noiseModel::Diagonal::shared_ptr NoiseModel_InitPose =
      gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.01 * Vector3d(1, 1, 1),
           Vector3d(1000, 1000, 1000))
              .finished());
  gtsam::noiseModel::Diagonal::shared_ptr NoiseModel_LastPose =
      gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << Vector3d(1000, 1000, 1000),
           0.01 * Vector3d(1, 1, 1))
              .finished());
  mpNewGraph->add(gtsam::PriorFactor<gtsam::Pose3>(X(0), mmpKFCVPose_c2w[0],
                                                   NoiseModel_InitPose));
  mpNewGraph->add(gtsam::PriorFactor<gtsam::Pose3>(
      X(PoseIndex), mmpKFCVPose_c2w[PoseIndex], NoiseModel_LastPose));
  // 3.2 Add SFMFeature factors
  int MapPointIndex = -1;
  for (auto &vitSFMFeature : mvSFMFeatures) {
    if (vitSFMFeature.bTriangulateState == true) {
      MapPointIndex++;
      vitSFMFeature.MapPointIndex = MapPointIndex;

      int itFrameIDInKFQue = vitSFMFeature.StartFrameIDInKFQue - 1;
      vector<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3,
                                            gtsam::Cal3_S2>>
          vInvDepthFactors;
      for (auto mpitUndisUVOnCurFrame :
           vitSFMFeature.mpKFID_UuvsOnAllFrms) {
        itFrameIDInKFQue++;
        assert(mpitUndisUVOnCurFrame.first == itFrameIDInKFQue);

        Vector2d FeatureUnDisUV_meas = mpitUndisUVOnCurFrame.second;
        if (!CalcCurVisualNoiseModel(mmpKFCVPose_c2w[itFrameIDInKFQue],
                                     vitSFMFeature.Point3w, FeatureUnDisUV_meas,
                                     vitSFMFeature.MapPointIndex))
          continue;

        gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3,
                                       gtsam::Cal3_S2>
            InitPointPrjFactor(FeatureUnDisUV_meas, mNoiseModel_Pixel,
                               X(mdKeyFrames[itFrameIDInKFQue].NavStateIndex),
                               L(vitSFMFeature.MapPointIndex), mK);

        vInvDepthFactors.push_back(InitPointPrjFactor);
      }

      if ((int)vInvDepthFactors.size() >= gCoVisualFramesThreshold) {
        // cout << vitSFMFeature.FeatureId << " " <<
        // vitSFMFeature.Point3w.transpose() << endl;

        vitSFMFeature.SolvedFlag = WATING_FOR_SOLVE;
        mNewInitialValues.insert(L(vitSFMFeature.MapPointIndex),
                                 vitSFMFeature.Point3w);

        for (int i = 0; i < (int)vInvDepthFactors.size(); i++)
          mpNewGraph->add(vInvDepthFactors[i]);
        // mprsCurTrackedFrame_.second.vbOptimized[itFt] = 1;
      } else {
        MapPointIndex--;
        vitSFMFeature.MapPointIndex = -1;
        continue;
      }
    }
  }

  // 4. Optimize
  mIsam2.update(*mpNewGraph, mNewInitialValues);
  mResultValues = mIsam2.calculateEstimate();
  // mResultValues.print();
  // mIsam2.getDelta().print("Delta:");
  mpNewGraph->resize(0);
  mNewInitialValues.clear();

  // 5. Update
  // 5.1 Update SFMFeature Features
  map<int, gtsam::Point3> mpSFMID_Point3ws;
  for (auto vitSFMFeature : mvSFMFeatures) {
    if (vitSFMFeature.MapPointIndex < 0) continue;

    int FeatureId = vitSFMFeature.FeatureId;
    auto litIdWisedFeature =
        find_if(mpFeatureManager->mlsIdWisedFeatures.begin(),
                mpFeatureManager->mlsIdWisedFeatures.end(),
                [FeatureId](const IdWisedFeature &litIdWisedFeature) {
                  return litIdWisedFeature.FeatureId == FeatureId;
                });

    litIdWisedFeature->MapPointIndex = vitSFMFeature.MapPointIndex;
    litIdWisedFeature->Point3w =
        mResultValues.at<gtsam::Point3>(L(litIdWisedFeature->MapPointIndex));
    // Point3w2InvDepth(mdKeyFrames[litIdWisedFeature->StartFrameIDInKFQue].Pose,
    //                  litIdWisedFeature->Point3w, litIdWisedFeature->InvDepth,
    //                  litIdWisedFeature->InvDepth);
    litIdWisedFeature->SolvedFlag = OK_SOLVED;

    mpSFMID_Point3ws[litIdWisedFeature->FeatureId] = litIdWisedFeature->Point3w;

    cout << litIdWisedFeature->FeatureId << " "
         << litIdWisedFeature->Point3w.transpose() << endl;
  }
  // 5.2 Update mmpKFCVPose_c2w Poses
  mmpKFCVPose_c2w_unOpt = mmpKFCVPose_c2w;
  for (size_t i = 0, j = 0; i <= PoseIndex; i++, j++) {
    mmpKFCVPose_c2w[i] = mResultValues.at<gtsam::Pose3>(X(i));
    if (mdprTrackedFrames[j].first.FrameEpoch == mdKeyFrames[i].FrameEpoch) {
      mdInitKeyFrames.push_back(mdprTrackedFrames[j].first);
      mdprTrackedFrames[j].first.Pose_c2w = mmpKFCVPose_c2w[i];

      mmpKFCVPose_c2w[i].print("CV");
      mdKeyFrames[i].Pose.print("IMUPre");
      cout << endl;

      mmpCVPose_i2w[j] = mmpKFCVPose_c2w[i] * gExtrPose_c2i.inverse();
      continue;
    } else if (mdprTrackedFrames[j].first.FrameEpoch >
               mdKeyFrames[i].FrameEpoch) {
      cerr << "Error epoch!";
      exit(1);
    }

    while (mdprTrackedFrames[j].first.FrameEpoch < mdKeyFrames[i].FrameEpoch) {
      gtsam::Pose3 CurPose_fx2w_inital = mmpKFCVPose_c2w[i];

      vector<cv::Point2f> vCVUnDisXYs;
      vector<cv::Point3f> vCVPoint3ws;
      int NumCurFrameFeatures = mdprTrackedFrames[j].second.vFeatureIds.size();
      for (int itFt = 0; itFt < NumCurFrameFeatures; itFt++) {
        int FtId = mdprTrackedFrames[j].second.vFeatureIds[itFt];
        map<int, gtsam::Point3>::iterator itSFMFt = mpSFMID_Point3ws.find(FtId);

        //！若该特征点也被第i关键帧观测到，则将其加入序列
        if (itSFMFt != mpSFMID_Point3ws.end()) {
          //！
          //注意这里的2D点img_pts是去畸变归一化平面上的点,而不是常规的像素平面的点
          // 因为下面的内参K已设置为单位阵，畸变矩阵D已设置为空
          Vector3d UnDisXYZ = mdprTrackedFrames[j].second.vUnDisXYZs[itFt];
          cv::Point2f CVUnDisXY(UnDisXYZ(0), UnDisXYZ(1));
          vCVUnDisXYs.push_back(CVUnDisXY);

          cv::Point3f CVPoint3w(itSFMFt->second[0], itSFMFt->second[1],
                                itSFMFt->second[2]);
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

      mdInitKeyFrames.push_back(mdprTrackedFrames[j].first);
      mdprTrackedFrames[j].first.Pose_c2w = CurPose_fx2w_inital;
      cout << j << endl;
      mdprTrackedFrames[j].first.Pose_c2w.print();
      cout << endl;

      j++;
    }
    assert(mdprTrackedFrames[j].first.FrameEpoch == mdKeyFrames[i].FrameEpoch);
    mdInitKeyFrames.push_back(mdprTrackedFrames[j].first);
    mdprTrackedFrames[j].first.Pose_c2w = mmpKFCVPose_c2w[i];
  }

  return true;
}

bool VIGInitializer::OptmAndUpdSFM_InDep() {
  // 1. Set isam2
  mtdmpNewKey_TimeStamps.clear();
  mpNewGraph->resize(0);
  mNewInitialValues.clear();
  mIsam2 = gtsam::ISAM2(mIsamParams);

  // 2. Add intial values
  int PoseIndex = -1;
  while (PoseIndex < (int)mdKeyFrames.size() - 1) {
    PoseIndex++;
    mNewInitialValues.insert(X(PoseIndex), mmpKFCVPose_c2w[PoseIndex]);
    mdKeyFrames[PoseIndex].NavStateIndex = PoseIndex;
  }

  // 3. Add factors
  // 3.1 Add prior factors
  mpNewGraph->add(gtsam::PriorFactor<gtsam::Pose3>(X(0), mmpKFCVPose_c2w[0],
                                                   mNoiseModel_InitPose));
  mpNewGraph->add(gtsam::PriorFactor<gtsam::Pose3>(
      X(PoseIndex), mmpKFCVPose_c2w[PoseIndex], mNoiseModel_InitPose));
  // 3.2 Add SFMFeature factors
  int MapPointIndex = -1;
  for (auto &vitSFMFeature : mvSFMFeatures) {
    if (vitSFMFeature.bTriangulateState == true) {
      MapPointIndex++;
      vitSFMFeature.MapPointIndex = MapPointIndex;

      vector<InvDepthFactor> vInvDepthFactors;
      gtsam::Point2 Measured_ref;
      for (auto mpitUndisUVOnCurFrame :
           vitSFMFeature.mpKFID_UuvsOnAllFrms) {
        int itFrameIDInKFQue = mpitUndisUVOnCurFrame.first;
        if (itFrameIDInKFQue == vitSFMFeature.StartFrameIDInKFQue) {
          Measured_ref =
              vitSFMFeature.mpKFID_UxyzsOnAllFrms[itFrameIDInKFQue].head(2);
        } else {
          gtsam::Point2 Measured =
              vitSFMFeature.mpKFID_UxyzsOnAllFrms[itFrameIDInKFQue].head(2);
          if (!CalcCurVisualNoiseModel(
                  mmpKFCVPose_c2w[itFrameIDInKFQue], vitSFMFeature.Point3w,
                  vitSFMFeature.mpKFID_UuvsOnAllFrms[itFrameIDInKFQue],
                  vitSFMFeature.MapPointIndex))
            continue;
          vInvDepthFactors.push_back(InvDepthFactor(
              X(mdKeyFrames[vitSFMFeature.StartFrameIDInKFQue].NavStateIndex),
              X(mdKeyFrames[itFrameIDInKFQue].NavStateIndex),
              L(vitSFMFeature.MapPointIndex), Measured_ref, Measured,
              mNoiseModel_Pixel));
        }
      }

      if ((int)vInvDepthFactors.size() >= gCoVisualFramesThreshold - 1) {
        cout << vitSFMFeature.FeatureId << " "
             << vitSFMFeature.Point3w.transpose() << endl;

        vitSFMFeature.SolvedFlag = WATING_FOR_SOLVE;
        mNewInitialValues.insert(L(vitSFMFeature.MapPointIndex),
                                 vitSFMFeature.InvDepth);

        // mpNewGraph->add(InitPointPrjFactor);
        for (int i = 0; i < (int)vInvDepthFactors.size(); i++)
          mpNewGraph->add(vInvDepthFactors[i]);
        // mprsCurTrackedFrame_.second.vbOptimized[itFt] = 1;
      } else {
        MapPointIndex--;
        vitSFMFeature.MapPointIndex = -1;
        continue;
      }
    }
  }

  // 4. Optimize
  mIsam2.update(*mpNewGraph, mNewInitialValues);
  mResultValues = mIsam2.calculateEstimate();
  // mResultValues.print();
  // mIsam2.getDelta().print("Delta:");
  mpNewGraph->resize(0);
  mNewInitialValues.clear();

  // 5. Update
  // 5.1 Update SFMFeature Features
  map<int, gtsam::Point3> mpSFMID_Point3ws;
  for (auto vitSFMFeature : mvSFMFeatures) {
    if (vitSFMFeature.MapPointIndex < 0) continue;

    int FeatureId = vitSFMFeature.FeatureId;
    auto litIdWisedFeature =
        find_if(mpFeatureManager->mlsIdWisedFeatures.begin(),
                mpFeatureManager->mlsIdWisedFeatures.end(),
                [FeatureId](const IdWisedFeature &litIdWisedFeature) {
                  return litIdWisedFeature.FeatureId == FeatureId;
                });

    litIdWisedFeature->MapPointIndex = vitSFMFeature.MapPointIndex;
    litIdWisedFeature->InvDepth =
        mResultValues.at<double>(L(litIdWisedFeature->MapPointIndex));
    litIdWisedFeature->Point3w =
        mdKeyFrames[litIdWisedFeature->StartFrameIDInKFQue].Pose.transform_from(
            1. / litIdWisedFeature->InvDepth * litIdWisedFeature ->
            vIdWisedFeatureInfoOnAllFrames[0].UnDisXYZ);
    // litIdWisedFeature->InvDepth = InvTPR2Depth(litIdWisedFeature->InvDepth);
    litIdWisedFeature->SolvedFlag = OK_SOLVED;

    mpSFMID_Point3ws[litIdWisedFeature->FeatureId] = litIdWisedFeature->Point3w;

    cout << litIdWisedFeature->FeatureId << " "
         << litIdWisedFeature->Point3w.transpose() << endl;
  }
  // 5.2 Update mmpKFCVPose_c2w Poses
  for (size_t i = 0, j = 0; i <= PoseIndex; i++, j++) {
    mmpKFCVPose_c2w[i] = mResultValues.at<gtsam::Pose3>(X(i));
    if (mdprTrackedFrames[j].first.FrameEpoch == mdKeyFrames[i].FrameEpoch) {
      mdprTrackedFrames[j].first.Pose_c2w = mmpKFCVPose_c2w[i];
      mmpCVPose_i2w[j] = mmpKFCVPose_c2w[i] * gExtrPose_c2i.inverse();
      continue;
    } else if (mdprTrackedFrames[j].first.FrameEpoch <
               mdKeyFrames[i].FrameEpoch) {
      gtsam::Pose3 CurPose_fx2w_inital = mmpKFCVPose_c2w[i];

      vector<cv::Point2f> vCVUnDisXYs;
      vector<cv::Point3f> vCVPoint3ws;
      int NumCurFrameFeatures = mdprTrackedFrames[j].second.vFeatureIds.size();
      for (int itFt = 0; itFt < NumCurFrameFeatures; itFt++) {
        int FtId = mdprTrackedFrames[j].second.vFeatureIds[itFt];
        map<int, gtsam::Point3>::iterator itSFMFt = mpSFMID_Point3ws.find(FtId);

        //！若该特征点也被第i关键帧观测到，则将其加入序列
        if (itSFMFt != mpSFMID_Point3ws.end()) {
          //！
          //注意这里的2D点img_pts是去畸变归一化平面上的点,而不是常规的像素平面的点
          // 因为下面的内参K已设置为单位阵，畸变矩阵D已设置为空
          Vector3d UnDisXYZ = mdprTrackedFrames[j].second.vUnDisXYZs[itFt];
          cv::Point2f CVUnDisXY(UnDisXYZ(0), UnDisXYZ(1));
          vCVUnDisXYs.push_back(CVUnDisXY);

          cv::Point3f CVPoint3w(itSFMFt->second[0], itSFMFt->second[1],
                                itSFMFt->second[2]);
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

      mdprTrackedFrames[j].first.Pose_c2w = CurPose_fx2w_inital;
      j++;
    } else {
      cerr << "Error epoch!";
      exit(1);
    }
  }

  return true;
}

void VIGInitializer::TriangCurSFMFeat_DualFrm(const int LFrame,
                                              const int RFrame,
                                              SFMFeature &CurSFMFeature) {
  if (LFrame == RFrame) return;
  //! 选择两帧共有的Features
  Vector3d UnDisXY1_l = CurSFMFeature.mpKFID_UxyzsOnAllFrms[LFrame];
  Vector3d UnDisXY1_r = CurSFMFeature.mpKFID_UxyzsOnAllFrms[RFrame];

  TriangulateOneFt_DualFrame(UnDisXY1_l.head(2), mmpKFCVPose_c2w[LFrame],
                             UnDisXY1_r.head(2), mmpKFCVPose_c2w[RFrame],
                             CurSFMFeature.Point3w);

  // UnDisXYZ_Point3w2InvDepth(UnDisXY1_l, mmpKFCVPose_c2w[LFrame],
  //                           CurSFMFeature.Point3w, CurSFMFeature.InvDepth,
  //                           CurSFMFeature.InvDepth);

  if (CurSFMFeature.InvDepth > 0 && CurSFMFeature.InvDepth < 50)
    CurSFMFeature.bTriangulateState = true;
  else
    CurSFMFeature.bTriangulateState = false;
}

void VIGInitializer::TriangAllSFMFeats_DualFrm(const int LFrame,
                                               const int RFrame) {
  if (LFrame == RFrame) return;
  for (int j = 0; j < mNumSFMFeatures; j++) {
    if (mvSFMFeatures[j].bTriangulateState == true) continue;

    if (mvSFMFeatures[j].mpKFID_UxyzsOnAllFrms.find(LFrame) !=
            mvSFMFeatures[j].mpKFID_UxyzsOnAllFrms.end() &&
        mvSFMFeatures[j].mpKFID_UxyzsOnAllFrms.find(RFrame) !=
            mvSFMFeatures[j].mpKFID_UxyzsOnAllFrms.end())
      TriangCurSFMFeat_DualFrm(LFrame, RFrame, mvSFMFeatures[j]);
  }
}

bool VIGInitializer::SolveXthFrameByPnP(const int XthFrameId,
                                        gtsam::Pose3 &Pose_fx2w_inital) {
  vector<cv::Point2f> vCVUnDisXYs;
  vector<cv::Point3f> vCVPoint3ws;
  for (int j = 0; j < mNumSFMFeatures; j++) {
    if (mvSFMFeatures[j].bTriangulateState != true) continue;

    //！若该特征点也被第i关键帧观测到，则将其加入序列
    if (mvSFMFeatures[j].mpKFID_UxyzsOnAllFrms.find(XthFrameId) !=
        mvSFMFeatures[j].mpKFID_UxyzsOnAllFrms.end()) {
      //！
      //注意这里的2D点img_pts是去畸变归一化平面上的点,而不是常规的像素平面的点
      // 因为下面的内参K已设置为单位阵，畸变矩阵D已设置为空
      Vector3d UnDisXYZ =
          mvSFMFeatures[j].mpKFID_UxyzsOnAllFrms[XthFrameId];
      cv::Point2f CVUnDisXY(UnDisXYZ(0), UnDisXYZ(1));
      vCVUnDisXYs.push_back(CVUnDisXY);
      cv::Point3f CVPoint3w(mvSFMFeatures[j].Point3w[0],
                            mvSFMFeatures[j].Point3w[1],
                            mvSFMFeatures[j].Point3w[2]);
      vCVPoint3ws.push_back(CVPoint3w);
    }
  }
  //! 若第i帧中包含的sfm_f中的特征点不足10，则直接返回
  if (int(vCVUnDisXYs.size()) < 15) {
    printf("unstable features tracking, please slowly move you device!\n");
    if (int(vCVUnDisXYs.size()) < 10) return false;
  }

  if (!SolveFrameByPnP(vCVPoint3ws, vCVUnDisXYs, Pose_fx2w_inital))
    return false;

  return true;
}

bool VIGInitializer::CalRelativePose_l2latest(int &l,
                                              gtsam::Pose3 &Pose_l2latest) {
  //！在滑窗内寻找与最新的关键帧共视点超过20(像素点)的关键帧
  // find previous frame which contians enough correspondance and parallex with
  // newest frame
  for (int i = 0; i < mBackFrIDInKFQue; i++) {
    vector<pair<Vector3d, Vector3d>> vprCorresUnDisXYZPair;
    vprCorresUnDisXYZPair =
        mpFeatureManager->GetCorresponding(i, mBackFrIDInKFQue);
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
      if (average_parallax * gFocalLength > 30 &&
          mpFeatureManager->SolveRelativePose_L2R(vprCorresUnDisXYZPair,
                                                  Pose_l2latest)) {
        Pose_l2latest.rotation().print();
        cout << endl;
        l = i;
        cout << endl
             << "average_parallax: " << average_parallax * gFocalLength
             << ",  choose l " << l
             << " and newest frame to triangulate the whole structure" << endl;
        return true;
      }
    }
  }
  return false;
}

bool VIGInitializer::ConstructSFM() {
  // 1. Form SFMFeatures ready for SFM
  mvSFMFeatures = mpFeatureManager->FormSFMFeatures();
  mNumSFMFeatures = mvSFMFeatures.size();

  // 2. Assign poses to l and latest
  gtsam::Pose3 Pose_l2latest;
  int l;
  if (!CalRelativePose_l2latest(l, Pose_l2latest)) return false;
  // Assign the appropriate physical translation(IMU) to Epipolar-calculated
  // translation
  gtsam::Pose3 Pose_l2latest_IMUPre =
      mdKeyFrames[mBackFrIDInKFQue].Pose.inverse() * mdKeyFrames[l].Pose;

  // // Test
  // Pose_l2latest.translation().print("Pose_l2latest t:");
  // Pose_l2latest_IMUPre.translation().print("Pose_l2latest_IMUPre t:");

  Pose_l2latest = gtsam::Pose3(Pose_l2latest.rotation(),
                               Pose_l2latest_IMUPre.translation());

  assert((int)mdKeyFrames.size() == mBackFrIDInKFQue + 1);
  mmpKFCVPose_c2w[l] = mdKeyFrames[l].Pose;
  mmpKFCVPose_c2w[mBackFrIDInKFQue] =
      mmpKFCVPose_c2w[l] * Pose_l2latest.inverse();

  // 3: 三角化第l frame到第NumFrames - 1（最后一帧）之间的Features
  //  *: trangulate between l ----- mInitWindowSize - 1（最后一帧）
  //  *: solve pnp l + 1; trangulate l + 1 ------- mInitWindowSize -
  //  1（最后一帧）;
  for (int i = l; i < mBackFrIDInKFQue; i++) {
    // 3.1: 求解第l+1 frame到第NumFrames - 2的相机位姿逆
    // solve pnp
    if (i > l) {
      //！PnP求解第i帧相机位姿的逆
      mmpKFCVPose_c2w[i] =
          mmpKFCVPose_c2w[i - 1];  // initial pose from the previous
      if (!SolveXthFrameByPnP(i, mmpKFCVPose_c2w[i])) return false;
    }

    // 3.2: 三角化第l frame和第NumFrames - 1（最后一帧）共视的Features
    // mmpKFCVPose_c2w: 世界坐标系下的点=>now帧坐标系下点的变换矩阵
    //! 求取得点在世界坐标系
    // triangulate point based on the solve pnp result
    TriangAllSFMFeats_DualFrm(i, mBackFrIDInKFQue);
  }

  // 4：三角化第l frame和第NumFrames - 2共视的Features
  // 4: triangulate l-----l+1 l+2 ... mInitWindowSize -2
  for (int i = l + 1; i < mBackFrIDInKFQue; i++)
    TriangAllSFMFeats_DualFrm(l, i);

  // 6：三角化第l-1 frame到第1 frame之间的Features
  // 6: solve pnp l-1; triangulate l-1 ----- l
  //             l-2              l-2 ----- l
  for (int i = l - 1; i >= 0; i--) {
    // solve pnp
    mmpKFCVPose_c2w[i] = mmpKFCVPose_c2w[i + 1];  // initial pose from the next
    if (!SolveXthFrameByPnP(i, mmpKFCVPose_c2w[i])) return false;
    // triangulate
    TriangAllSFMFeats_DualFrm(i, l);
  }

  // 7：三角化其他没有恢复的满足共视点大于最小共视帧数的Features
  // 7: triangulate all other points
  for (int j = 0; j < mNumSFMFeatures; j++) {
    if (mvSFMFeatures[j].bTriangulateState == true ||
        mvSFMFeatures[j].mpKFID_UxyzsOnAllFrms.size() < 2)
      continue;

    int LID = mvSFMFeatures[j].mpKFID_UxyzsOnAllFrms.begin()->first;
    int RID = mvSFMFeatures[j].mpKFID_UxyzsOnAllFrms.rbegin()->first;
    TriangCurSFMFeat_DualFrm(LID, RID, mvSFMFeatures[j]);
  }

  return true;
}

bool VIGInitializer::VisualIMUAlignment() {
  gtsam::Pose3 ExtrPose_i2c = gExtrPose_c2i.inverse();
  Eigen::Matrix3d Rcb = ExtrPose_i2c.rotation().matrix();
  Eigen::Vector3d pcb = ExtrPose_i2c.translation().matrix();

  // 1. Sovle Initial Gyro Bias
  int NTrackedFrms = mdprTrackedFrames.size();
  Eigen::Matrix<double, Dynamic, 3> h;
  Eigen::Matrix<double, Dynamic, 1> b, b_u;
  h.setZero(3 * (NTrackedFrms - 1), 3);
  b.setZero(3 * (NTrackedFrms - 1), 1);
  b_u.setZero(3 * (NTrackedFrms - 1), 1);

  int FID4IMUPre = 1;
  drPreintegratedImuMeasurements *pdrPreintegratedImuMeasurements =
      new drPreintegratedImuMeasurements(mIMU_params, mOptimizedCurBias);
  // 1.1 Ascociate IMU Pre-Int to Tracked Frames, and Calc jacobians and
  // residuals
  assert(mvInitialIMUs.front().msCurIMUData.T <=
             mdprTrackedFrames.front().first.FrameEpoch &&
         mvInitialIMUs.back().msCurIMUData.T >=
             mdprTrackedFrames.back().first.FrameEpoch);
  for (auto &vitInitialIMU : mvInitialIMUs) {
    if (vitInitialIMU.msCurIMUData.T <= mdprTrackedFrames[0].first.FrameEpoch)
      continue;

    pdrPreintegratedImuMeasurements->integrateMeasurement(
        (vitInitialIMU.msPreIMUData.Acc + vitInitialIMU.msCurIMUData.Acc) * 0.5,
        (vitInitialIMU.msPreIMUData.Gyro + vitInitialIMU.msCurIMUData.Gyro) *
            0.5,
        vitInitialIMU.msCurIMUData.T - vitInitialIMU.msPreIMUData.T);

    if (vitInitialIMU.msCurIMUData.T >=
        mdprTrackedFrames[FID4IMUPre].first.FrameEpoch) {
      mmpIMUPreIntBetwDualFrms[FID4IMUPre] = *pdrPreintegratedImuMeasurements;
      // Jacobian
      h.middleRows<3>(3 * (FID4IMUPre - 1)) =
          mmpIMUPreIntBetwDualFrms[FID4IMUPre].delRdelBiasOmega();
      // Residual Error
      gtsam::Rot3 IMUPreDeltRij =
          mmpIMUPreIntBetwDualFrms[FID4IMUPre].deltaRij();
      gtsam::Rot3 BRot_Frm_i =
          mdprTrackedFrames[FID4IMUPre - 1].first.Pose_c2w.rotation() *
          gtsam::Rot3(Rcb);
      gtsam::Rot3 BRot_Frm_j =
          mdprTrackedFrames[FID4IMUPre].first.Pose_c2w.rotation() *
          gtsam::Rot3(Rcb);
      gtsam::Rot3 FrmDeltRij = BRot_Frm_i.inverse() * BRot_Frm_j;
      gtsam::Rot3 errR(IMUPreDeltRij.inverse() * FrmDeltRij);
      gtsam::Vector3 err = gtsam::Rot3::Logmap(errR);
      b.middleRows<3>(3 * (FID4IMUPre - 1)) = err;

      gtsam::Rot3 BRot_Frm_i_u =
          mmpKFCVPose_c2w_unOpt[FID4IMUPre - 1].rotation() * gtsam::Rot3(Rcb);
      gtsam::Rot3 BRot_Frm_j_u =
          mmpKFCVPose_c2w_unOpt[FID4IMUPre].rotation() * gtsam::Rot3(Rcb);
      gtsam::Rot3 FrmDeltRij_u = BRot_Frm_i_u.inverse() * BRot_Frm_j_u;
      gtsam::Rot3 errR_u(IMUPreDeltRij.inverse() * FrmDeltRij_u);
      gtsam::Vector3 err_u = gtsam::Rot3::Logmap(errR_u);
      b_u.middleRows<3>(3 * (FID4IMUPre - 1)) = err_u;

      // cout << "err" << err.transpose() << endl;
      // cout << "err_u" << err_u.transpose() << endl;
      // cout << endl;

      Matrix3d hi = h.middleRows<3>(3 * (FID4IMUPre - 1)).transpose() *
                    h.middleRows<3>(3 * (FID4IMUPre - 1));
      Vector3d bi = h.middleRows<3>(3 * (FID4IMUPre - 1)).transpose() * err;
      Vector3d delta_bgi = hi.ldlt().solve(bi);
      cout << delta_bgi.transpose() << endl;
      cout << endl;

      FID4IMUPre++;
      pdrPreintegratedImuMeasurements->resetIntegration();
    }
  }

  // 1.2 Solve delta GyroBias
  Matrix3d hTh = h.transpose() * h;
  Vector3d hTb = h.transpose() * b;
  gtsam::Vector3 dBg = hTh.ldlt().solve(hTb);
  mOptimizedCurBias =
      mOptimizedCurBias +
      gtsam::imuBias::ConstantBias(gtsam::Vector3(0.0, 0.0, 0.0), dBg);
  Vector3d hTb_u = h.transpose() * b_u;
  gtsam::Vector3 dBg_u = hTh.ldlt().solve(hTb_u);

  // 1.3 Update Pre-integration
  for (FID4IMUPre = 1; FID4IMUPre <= NTrackedFrms - 1; FID4IMUPre++) {
    mmpIMUPreIntBetwDualFrms[FID4IMUPre].BiasCorrectedDelta(mOptimizedCurBias);
  }

  // 2.Sovle accelerometer bais
  Eigen::Matrix<double, Dynamic, 4> A;
  Eigen::Matrix<double, Dynamic, 3> A3;
  Eigen::Matrix<double, Dynamic, 1> B;
  Eigen::Matrix<double, Dynamic, 1> B3;
  Matrix3d I3 = Matrix3d::Identity();
  A.setZero(3 * (NTrackedFrms - 2), 4);
  A3.setZero(3 * (NTrackedFrms - 2), 3);
  B.setZero(3 * (NTrackedFrms - 2), 1);
  B3.setZero(3 * (NTrackedFrms - 2), 1);
  // 2.1 Approx Scale and Gravity vector in world frame (nav frame)
  for (FID4IMUPre = 1; FID4IMUPre < NTrackedFrms - 1; FID4IMUPre++) {
    pair<Frame, FrameWisedFeatures> TrackedFrm1 =
        mdprTrackedFrames[FID4IMUPre - 1];
    pair<Frame, FrameWisedFeatures> TrackedFrm2 = mdprTrackedFrames[FID4IMUPre];
    pair<Frame, FrameWisedFeatures> TrackedFrm3 =
        mdprTrackedFrames[FID4IMUPre + 1];
    // Delta time between frames
    double dt12 = TrackedFrm2.first.FrameEpoch - TrackedFrm1.first.FrameEpoch;
    double dt23 = TrackedFrm3.first.FrameEpoch - TrackedFrm2.first.FrameEpoch;
    // Pre-integrated measurements
    Vector3d dp12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].deltaPij();
    Vector3d dv12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].deltaVij();
    Vector3d dp23 = mmpIMUPreIntBetwDualFrms[FID4IMUPre + 1].deltaPij();
    Matrix3d Jpba12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].delPdelBiasAcc();
    Matrix3d Jvba12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].delVdelBiasAcc();
    Matrix3d Jpba23 = mmpIMUPreIntBetwDualFrms[FID4IMUPre + 1].delPdelBiasAcc();
    // Pose of camera in world frame
    Vector3d pc1 = TrackedFrm1.first.Pose_c2w.translation().matrix();
    Vector3d pc2 = TrackedFrm2.first.Pose_c2w.translation().matrix();
    Vector3d pc3 = TrackedFrm3.first.Pose_c2w.translation().matrix();
    Matrix3d Rc1 = TrackedFrm1.first.Pose_c2w.rotation().matrix();
    Matrix3d Rc2 = TrackedFrm2.first.Pose_c2w.rotation().matrix();
    Matrix3d Rc3 = TrackedFrm3.first.Pose_c2w.rotation().matrix();

    // lambda*s + beta*g = gamma
    A.block<3, 1>(3 * (FID4IMUPre - 1), 0) =
        (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;
    A.block<3, 3>(3 * (FID4IMUPre - 1), 1) = -Rc1 * Rcb * Jpba12 * dt23 +
                                             Rc1 * Rcb * Jvba12 * dt12 * dt23 +
                                             Rc2 * Rcb * Jpba23 * dt12;
    // 0.5 * I3 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23);
    B.middleRows<3>(3 * (FID4IMUPre - 1)) =
        (Rc3 - Rc2) * pcb * dt12 + (Rc1 - Rc2) * pcb * dt23 +
        Rc1 * Rcb * dp12 * dt23 - Rc2 * Rcb * dp23 * dt12 -
        Rc1 * Rcb * dv12 * dt12 * dt23 - 0.5 * gG * dt12 * dt23 * (dt12 + dt23);

    A3.block<3, 3>(3 * (FID4IMUPre - 1), 0) = -Rc1 * Rcb * Jpba12 * dt23 +
                                              Rc1 * Rcb * Jvba12 * dt12 * dt23 +
                                              Rc2 * Rcb * Jpba23 * dt12;
    B3.middleRows<3>(3 * (FID4IMUPre - 1)) =
        (Rc3 - Rc2) * pcb * dt12 + (Rc1 - Rc2) * pcb * dt23 +
        Rc1 * Rcb * dp12 * dt23 - Rc2 * Rcb * dp23 * dt12 -
        Rc1 * Rcb * dv12 * dt12 * dt23 -
        0.5 * gG * dt12 * dt23 * (dt12 + dt23) - (pc2 - pc1) * dt23 -
        (pc2 - pc3) * dt12;

    Matrix3d A3_i = A3.block<3, 3>(3 * (FID4IMUPre - 1), 0);
    Vector3d B3_i = B3.middleRows<3>(3 * (FID4IMUPre - 1));
    Vector3d ba_i = A3_i.ldlt().solve(B3_i);
    cout << "ba_i" << ba_i.transpose() << endl;
    cout << endl;
  }

  // 2.2 Solve Accelerometer Bias
  cout << A3 << endl << endl;
  cout << B3 << endl << endl;
  Matrix4d ATA = A.transpose() * A;
  Vector4d ATB = A.transpose() * B;
  Vector4d s_b_star = ATA.ldlt().solve(ATB);
  double s_star = s_b_star(0);
  Vector3d b_star = s_b_star.tail(3);
  if (s_star < 0) {
    cerr << "Wrong Scale!" << endl;
    exit(1);
  }
  Matrix3d ATA3 = A3.transpose() * A3;
  Vector3d ATB3 = A3.transpose() * B3;
  Vector3d dBa = ATA3.ldlt().solve(ATB3);
  mOptimizedCurBias =
      mOptimizedCurBias +
      gtsam::imuBias::ConstantBias(dBa, gtsam::Vector3(0.0, 0.0, 0.0));

  // 2.3 Update Pre-integration
  for (FID4IMUPre = 1; FID4IMUPre <= NTrackedFrms - 1; FID4IMUPre++) {
    mmpIMUPreIntBetwDualFrms[FID4IMUPre].BiasCorrectedDelta(mOptimizedCurBias);
  }

  Vector3d v1, v1cal, v2, v2cal;
  v1 = mdprTrackedFrames.front().first.Vel_i2w;
  v1cal = v1;
  for (FID4IMUPre = 1; FID4IMUPre < NTrackedFrms; FID4IMUPre++) {
    pair<Frame, FrameWisedFeatures> TrackedFrm1 =
        mdprTrackedFrames[FID4IMUPre - 1];
    pair<Frame, FrameWisedFeatures> TrackedFrm2 = mdprTrackedFrames[FID4IMUPre];
    // Delta time between frames
    double dt12 = TrackedFrm2.first.FrameEpoch - TrackedFrm1.first.FrameEpoch;
    // Pre-integrated measurements
    Vector3d dp12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].deltaPij();
    Vector3d dv12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].deltaVij();
    Matrix3d Jpba12 = mmpIMUPreIntBetwDualFrms[FID4IMUPre].delPdelBiasAcc();

    // Pose of camera in world frame
    Vector3d pc1 = TrackedFrm1.first.Pose_c2w.translation().matrix();
    Vector3d pc2 = TrackedFrm2.first.Pose_c2w.translation().matrix();
    // v1 = TrackedFrm1.first.Vel_i2w;
    v2 = TrackedFrm2.first.Vel_i2w;
    Matrix3d Rc1 = TrackedFrm1.first.Pose_c2w.rotation().matrix();
    Matrix3d Rc2 = TrackedFrm2.first.Pose_c2w.rotation().matrix();

    Vector3d L = pc2;
    Vector3d R = pc1 + v1 * dt12 + Rc1 * Rcb * (dp12 + Jpba12 * dBa) +
                 (Rc1 - Rc2) * pcb + 0.5 * gG * dt12 * dt12;
    cout << "L" << L.transpose() << endl;
    cout << "R" << R.transpose() << endl << endl;

    // v1cal = (pc2 - (pc1 + Rc1 * Rcb * (dp12 + Jpba12 * dBa) +
    //                 (Rc1 - Rc2) * pcb + 0.5 * gG * dt12 * dt12)) /
    //         dt12;
    v1 = v1cal;
    v2cal = v1cal + gG * dt12 + Rc1 * Rcb * dv12;
    v1cal = v2cal;

    cout << TrackedFrm2.first.FrameEpoch << endl;
    cout << v2.transpose() << endl;
    cout << v2cal.transpose() << endl << endl;
    int a = 1;
  }

  return true;
}

bool VIGInitializer::InitializeVIGStructure_Loosely() {
  if (gbUseGNSS) assert(static_cast<int>(mdInitialGNSSs.size()) > 1);
  //    && mNavStateIndex > 0);

  // 1. Constuct SFM
  if (!ConstructSFM()) return false;

  // 2. Optimize and Update SFM
  // if (!OptmAndUpdSFM_InDep())
  //     return false;
  if (!OptmAndUpdSFM_Point3w()) return false;

  // 3. Align IMU and Visual
  return VisualIMUAlignment();
}

void VIGInitializer::AddVisualFactor_TightInit() {
  assert(mdKeyFrames.size() == mdprTrackedFrames.size());

  for (int itFrm = 0; itFrm < mdKeyFrames.size(); itFrm++) {
    mprsCurTrackedFrame = mdprTrackedFrames[itFrm];
    int NumCurFrameFeatures = mprsCurTrackedFrame.second.vFeatureIds.size();
    mprsCurTrackedFrame.second.vbOptimized.resize(NumCurFrameFeatures, 0);
    for (int itFt = 0; itFt < NumCurFrameFeatures; itFt++) {
      int FeatureId = mprsCurTrackedFrame.second.vFeatureIds[itFt];
      auto litIdWisedFeature =
          find_if(mpFeatureManager->mlsIdWisedFeatures.begin(),
                  mpFeatureManager->mlsIdWisedFeatures.end(),
                  [FeatureId](const IdWisedFeature &litIdWisedFeature) {
                    return litIdWisedFeature.FeatureId == FeatureId;
                  });
      assert(litIdWisedFeature != mpFeatureManager->mlsIdWisedFeatures.end());

      // if the feature isn't(or maybe not) triangulated successfully
      if (litIdWisedFeature->InvDepth < 0 ||
          litIdWisedFeature->InvDepth > 100)
        continue;

      if (litIdWisedFeature->SolvedFlag == NOT_SOLVED) {
        mMapPointIndex++;
        litIdWisedFeature->MapPointIndex = mMapPointIndex;

        int itFrameIDInKFQue = litIdWisedFeature->StartFrameIDInKFQue - 1;
        vector<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3,
                                              gtsam::Cal3_S2>>
            vInvDepthFactors;
        for (auto &vitIdWisedFeaturePerFrame :
             litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames) {
          itFrameIDInKFQue++;
          assert(vitIdWisedFeaturePerFrame.CurFrameId ==
                 mdKeyFrames[itFrameIDInKFQue].FrameId);

          Vector2d FeatureUnDisUV_meas = vitIdWisedFeaturePerFrame.UnDisUV;
          if (!CalcCurVisualNoiseModel(mdKeyFrames[itFrameIDInKFQue].Pose,
                                       litIdWisedFeature->Point3w,
                                       FeatureUnDisUV_meas,
                                       litIdWisedFeature->MapPointIndex))
            continue;

          gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3,
                                         gtsam::Cal3_S2>
              InitPointPrjFactor(FeatureUnDisUV_meas, mNoiseModel_Pixel,
                                 X(mdKeyFrames[itFrameIDInKFQue].NavStateIndex),
                                 L(litIdWisedFeature->MapPointIndex), mK,
                                 gExtrPose_c2i);
          vInvDepthFactors.push_back(InitPointPrjFactor);
        }

        if ((int)vInvDepthFactors.size() >= gCoVisualFramesThreshold) {
          litIdWisedFeature->SolvedFlag = WATING_FOR_SOLVE;
          mNewInitialValues.insert(L(litIdWisedFeature->MapPointIndex),
                                   litIdWisedFeature->Point3w);

          for (int i = 0; i < (int)vInvDepthFactors.size(); i++)
            mpNewGraph->add(vInvDepthFactors[i]);

          mprsCurTrackedFrame.second.vbOptimized[itFt] = 1;
        } else {
          mMapPointIndex--;
          litIdWisedFeature->MapPointIndex = -1;
          continue;
        }
      } else if (litIdWisedFeature->SolvedFlag == WATING_FOR_SOLVE) {
        mprsCurTrackedFrame.second.vbOptimized[itFt] = 1;
        continue;
      } else
        continue;
    }
  }

  mIMUCounterBetwDualObss = 0;
}

bool VIGInitializer::InitializeVIGStructure_Tightly() {
  /**** 1：Triangulate the Features in KeyFrames Queue ****/
  mpFeatureManager->TriangulateCurFrameFts_MultFrames(mdKeyFrames);
  assert(mdprTrackedFrames.size() == mdKeyFrames.size());
  deque<pair<Frame, FrameWisedFeatures>> dprInitialTrackedFrames =
      mdprTrackedFrames;
  deque<KeyFrame> dInitialKeyFrames = mdKeyFrames;
  mdprTrackedFrames.clear();
  mdKeyFrames.clear();

  /**** 2：Reinitialize FactorGraph and InitialValues ****/
  // mGNSSResidulThreshold = mpConfiguration->mGNSSResidulThreshold;
  mGNSSCounter = 0;
  mIMUCounterBetwDualObss = 0;
  mNavStateIndex = 0;
  mOptimizedCurNavState = gtsam::NavState(mPoseb2n, mVelb2n);
  mOptimizedCurBias = mResultValues.at<gtsam::imuBias::ConstantBias>(B(0));

  gtsam::PreintegratedImuMeasurements *pIMUPreIntBetwDualObss4AdIMUFactor,
      *pIMUPreIntBetwDualGNSSs4InitNavStPred;
  pIMUPreIntBetwDualObss4AdIMUFactor =
      new gtsam::PreintegratedImuMeasurements(mIMU_params, mOptimizedCurBias);
  pIMUPreIntBetwDualGNSSs4InitNavStPred =
      new gtsam::PreintegratedImuMeasurements(mIMU_params, mOptimizedCurBias);
  if (mstrOptimizer == "ISAM2")
    mIsam2 = gtsam::ISAM2(mIsamParams);
  else if (mstrOptimizer == "IncrementalFixedLagSmoother")
    mFLSmootherISAM2 =
        gtsam::IncrementalFixedLagSmoother(mSmootherLag, mIsamParams);

  mNewInitialValues.insert(X(mNavStateIndex), mOptimizedCurNavState.pose());
  mNewInitialValues.insert(V(mNavStateIndex), mOptimizedCurNavState.v());
  mNewInitialValues.insert(B(mNavStateIndex), mOptimizedCurBias);

  mpNewGraph->add(gtsam::PriorFactor<gtsam::Pose3>(
      X(mNavStateIndex), mOptimizedCurNavState.pose(), mNoiseModel_InitPose));
  mpNewGraph->add(gtsam::PriorFactor<gtsam::Vector3>(
      V(mNavStateIndex), mOptimizedCurNavState.v(), mNoiseModel_InitVel));
  mpNewGraph->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      B(mNavStateIndex), mOptimizedCurBias, mNoiseModel_Initbias));

  mvNavStatesTimeStamps.clear();
  mtdmpNewKey_TimeStamps.clear();
  UpdateNavStateTimeStamps_KeyTimeStampMap(
      mvInitialIMUs.front().msPreIMUData.T);

  /**** 3：Add Factors and Optimize ****/
  bool bHasIMUAdded /*, bIsGNSSAvailable, bIsVisualAvailable*/;
  assert(gbUseGNSS && gbUseCam &&
         mvInitialIMUs.front().msCurIMUData.T <=
             mdInitialGNSSs.front().msGNSSSolData.Tow &&
         mvInitialIMUs.front().msCurIMUData.T <=
             dInitialKeyFrames.front().FrameEpoch);

  for (auto &vitInitialIMU : mvInitialIMUs) {
    bHasIMUAdded = false;

    mIMUCounterBetwDualObss++;
    pIMUPreIntBetwDualObss4AdIMUFactor->integrateMeasurement(
        (vitInitialIMU.msPreIMUData.Acc + vitInitialIMU.msCurIMUData.Acc) * 0.5,
        (vitInitialIMU.msPreIMUData.Gyro + vitInitialIMU.msCurIMUData.Gyro) *
            0.5,
        // vitInitialIMU.msCurIMUData.Acc, vitInitialIMU.msCurIMUData.Gyro,
        vitInitialIMU.msCurIMUData.T - vitInitialIMU.msPreIMUData.T);
    pIMUPreIntBetwDualGNSSs4InitNavStPred->integrateMeasurement(
        (vitInitialIMU.msPreIMUData.Acc + vitInitialIMU.msCurIMUData.Acc) * 0.5,
        (vitInitialIMU.msPreIMUData.Gyro + vitInitialIMU.msCurIMUData.Gyro) *
            0.5,
        // vitInitialIMU.msCurIMUData.Acc, vitInitialIMU.msCurIMUData.Gyro,
        vitInitialIMU.msCurIMUData.T - vitInitialIMU.msPreIMUData.T);

    // bIsGNSSAvailable = false;
    if (vitInitialIMU.msCurIMUData.T >=
        mdInitialGNSSs.front().msGNSSSolData.Tow) {
      mOptimizedCurNavState = gtsam::NavState(
          mResultValues.at<gtsam::Pose3>(
              X(mResultValues.size() / 3 - mdInitialGNSSs.size() + 1)),
          mResultValues.at<gtsam::Vector3>(
              V(mResultValues.size() / 3 - mdInitialGNSSs.size() + 1)));
      mOptimizedCurBias = mResultValues.at<gtsam::imuBias::ConstantBias>(
          B(mResultValues.size() / 3 - mdInitialGNSSs.size() + 1));

      pIMUPreIntBetwDualGNSSs4InitNavStPred->resetIntegrationAndSetBias(
          mOptimizedCurBias);

      mpIMUPreIntg = pIMUPreIntBetwDualObss4AdIMUFactor;
      if (!bHasIMUAdded) {
        AddIMUFactor();
        bHasIMUAdded = true;
      }

      GNSS *pGNSS = &mdInitialGNSSs.front();
      pGNSS->CalGNSSBLH2BodyInNavFrame(GetCurPredPose().rotation());
      AddGNSSSolFactor(pGNSS);
      // bIsGNSSAvailable = true;

      pIMUPreIntBetwDualObss4AdIMUFactor->resetIntegrationAndSetBias(
          mOptimizedCurBias);
      mdInitialGNSSs.pop_front();
    }

    if (vitInitialIMU.msCurIMUData.T >= dInitialKeyFrames.front().FrameEpoch) {
      mpIMUPreIntg = pIMUPreIntBetwDualObss4AdIMUFactor;
      if (!bHasIMUAdded) {
        AddIMUFactor();
        bHasIMUAdded = true;
      }

      mdprTrackedFrames.push_back(dprInitialTrackedFrames.front());
      mdKeyFrames.push_back(dInitialKeyFrames.front());
      mdKeyFrames.back().NavStateIndex = mNavStateIndex;
      // mpIMUPreIntg = pIMUPreIntBetwDualGNSSs4InitNavStPred;
      // AddVisualFactor();

      pIMUPreIntBetwDualObss4AdIMUFactor->resetIntegrationAndSetBias(
          mOptimizedCurBias);
      dprInitialTrackedFrames.pop_front();
      dInitialKeyFrames.pop_front();
    }

    if (bHasIMUAdded) {
      UpdateNavStateTimeStamps_KeyTimeStampMap(vitInitialIMU.msCurIMUData.T);

      mpIMUPreIntg = pIMUPreIntBetwDualGNSSs4InitNavStPred;
      InsertNewNavStateInitialValues();
      // mNewInitialValues.print("Initial ");

      if (dInitialKeyFrames.empty()) {
        delete pIMUPreIntBetwDualObss4AdIMUFactor;
        pIMUPreIntBetwDualObss4AdIMUFactor = nullptr;
        delete pIMUPreIntBetwDualGNSSs4InitNavStPred;
        pIMUPreIntBetwDualGNSSs4InitNavStPred = nullptr;
      }
    }
  }

  AddVisualFactor_TightInit();

  mpIMUPreIntg =
      new gtsam::PreintegratedImuMeasurements(mIMU_params, mOptimizedCurBias);
  Optimize();

  meMarginFlag = MARGIN_OLD;
  ClearVIGInitialization();

  return true;
}
}  // namespace VIG
