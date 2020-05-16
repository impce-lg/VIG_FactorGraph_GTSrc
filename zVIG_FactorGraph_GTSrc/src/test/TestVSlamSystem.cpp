#include "VIGSystem.h"

namespace VIG {

int TestVSlamSystem::mPoseIndex_ = 0;       // start from 0
int TestVSlamSystem::mMapPointIndex_ = -1;  // start from 0

TestVSlamSystem::TestVSlamSystem(const Configuration *pConfiguration,
                                 const double FirstNavStateIMUTime,
                                 const string &ConfigFilename) {
  mifCam_ = pConfiguration->mstrCamPath;
  mifCamTime_.open(pConfiguration->mstrCamTimeFile, ios_base::in);

  mpFeatureTracker_ =
      new FeatureTracker(pConfiguration, mifCamTime_, FirstNavStateIMUTime);
  mpFeatureManager_ = new FeatureManager(pConfiguration);

  InitializeEstimator(pConfiguration);

  mpViewer_ = new Viewer(ConfigFilename);
  mpViewer_->InitiViewer(mPoseb2n_);
}

void TestVSlamSystem::InitializeEstimator(const Configuration *pConfiguration) {
  /********* I. Initial Conditions *******/
  gtsam::Rot3 InitialRotb2n =
      gtsam::Rot3(pConfiguration->mIniQi2n.toRotationMatrix());
  mPoseb2n_ = gtsam::Pose3(InitialRotb2n, gtsam::Point3(0.0, 0.0, 0.0));
  mNoiseModel_InitPose_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << pConfiguration->mInitAttSigma * Vector3d(1, 1, 1),
       pConfiguration->mInitPosSigma * Vector3d(1, 1, 1))
          .finished());

  /**** II. Camera settings ****/
  mBackFrameIDInKFQue_ = 0;
  mInitWindowSize_ = pConfiguration->mInitWindowSize;

  mInitSigma_Pixel_ = pConfiguration->mSigma_Pixel;
  mNoiseModel_Pixel_ =
      gtsam::noiseModel::Isotropic::Sigma(2, mInitSigma_Pixel_);
  meVIGEstimatorState_ = NOT_INITIALIZED;

  /**** III. ISAM2 settings ****/
  mIsamParams_.setOptimizationParams(gtsam::ISAM2GaussNewtonParams());
  mIsamParams_.setFactorization("CHOLESKY");
  mIsamParams_.setRelinearizeSkip(
      pConfiguration
          ->mIsamParams_relinearizeSkip);  // ISAM2Params set 10 as default
  mIsamParams_.findUnusedFactorSlots =
      pConfiguration->mbIsamParams_findUnusedFactorSlots;
  mIsamParams_.evaluateNonlinearError =
      pConfiguration->mbIsamParams_evaluateNonlinearError;
  mVSlamIsam2_ = gtsam::ISAM2(mIsamParams_);

  mOptimizedCurPose_ = mPoseb2n_;
  mPredNextPose4Init_ = mOptimizedCurPose_;
}

bool TestVSlamSystem::CalcCurVisualNoiseModel(const gtsam::Pose3 &CurKFPose,
                                              const gtsam::Point3 &Point3w,
                                              const gtsam::Vector2 &Measured,
                                              int CurMapPointIndex) {
  // Project landmark into Pose2
  gtsam::PinholeBase normalized_plane(CurKFPose);
  Vector2d ErrorRePrj = normalized_plane.project2(Point3w) - Measured;

  // Test

  if (fabs(ErrorRePrj(0)) < mInitSigma_Pixel_ &&
      fabs(ErrorRePrj(1)) < mInitSigma_Pixel_) {
    cout << CurMapPointIndex << "," << ErrorRePrj(0) << "," << ErrorRePrj(1)
         << endl;
    mNoiseModel_Pixel_ =
        gtsam::noiseModel::Isotropic::Sigma(2, mInitSigma_Pixel_);
    return true;
  } else if (fabs(ErrorRePrj(0)) < 1.5 * mInitSigma_Pixel_ &&
             fabs(ErrorRePrj(1)) < 1.5 * mInitSigma_Pixel_) {
    cout << CurMapPointIndex << "," << ErrorRePrj(0) << "," << ErrorRePrj(1)
         << endl;
    mNoiseModel_Pixel_ =
        gtsam::noiseModel::Isotropic::Sigma(2, 1.5 * mInitSigma_Pixel_);
    return true;
  } else if (fabs(ErrorRePrj(0)) < 2 * mInitSigma_Pixel_ &&
             fabs(ErrorRePrj(1)) < 2 * mInitSigma_Pixel_) {
    cout << CurMapPointIndex << "," << ErrorRePrj(0) << "," << ErrorRePrj(1)
         << endl;
    mNoiseModel_Pixel_ =
        gtsam::noiseModel::Isotropic::Sigma(2, 2 * mInitSigma_Pixel_);
    return true;
  } else
    return false;
}

void TestVSlamSystem::UpdatePoseTimeStamps_KeyTimeStampMap(double CurPoseTime) {
  mvPosesTimeStamps_.push_back(CurPoseTime);

  // mtdmpNewKey_TimeStamps[X(mPoseIndex_)] = CurPoseTime;
  // mtdmpNewKey_TimeStamps[V(mPoseIndex_)] = CurPoseTime;
  // mtdmpNewKey_TimeStamps[B(mPoseIndex_)] = CurPoseTime;
  // if (mMapPointIndex_ >= 0)
  // {
  //     for (auto &litIdWisedFeature : mpFeatureManager_->mlsIdWisedFeatures)
  //     {
  //         if (litIdWisedFeature.SolvedFlag == WATING_FOR_SOLVE)
  //             mtdmpNewKey_TimeStamps[L(litIdWisedFeature.MapPointIndex)] =
  //             CurPoseTime;
  //     }
  // }
}

void TestVSlamSystem::RunVSlam(const Configuration *pConfiguration,
                               const IMU *pIMU,
                               FactorGraphEstimator *pFactorGraphEstimator) {
  if (!gbUseCam && pIMU->msCurIMUData.T >= mpFeatureTracker_->mFrameEpoch) {
    // there are some gap in the imu recordings
    if (fabs(pIMU->msCurIMUData.T - mpFeatureTracker_->mFrameEpoch) >
        pConfiguration->mIMUDt)
      cout << "IMU gap at:" << pIMU->msCurIMUData.T
           << ",FramID:" << mpFeatureTracker_->mImageID << endl;

    // 1. get the image of index frmId 0 based
    mpFeatureTracker_->ReadCurrentImage(mifCam_);

    // 2. Get current frame pose
    gtsam::Pose3 CurPose_cam2ref =
        pFactorGraphEstimator->GetCurPredPose() * gExtrPose_c2i;

    // 3. Track the adjacency frame
    if (gbFirstFrame)  // the first frame
    {
      gFirstPubFrameEpoch = mpFeatureTracker_->mInitiFrameEpoch;
      gbFirstFrame = false;
      mpFeatureTracker_->GrabNextImageEpoch(mifCamTime_);
      return;
    }
    mpFeatureTracker_->TrackFeatures(CurPose_cam2ref);

    // 4. Add TrackedFrames, FramesId_PosesInKFQue
    AddCurTrackedFrameAndKeyFrame(mpFeatureTracker_->mprsCurFrameWisedFeatures);

    // 5. Add IdWised Features
    //    and Check the Parallax to decide which frame in the KeyFrames Queue to
    //    marginalize
    AddIdWisedFeatures(CurPose_cam2ref);

    // Initialize or Add Factors
    if (meVIGEstimatorState_ == NOT_INITIALIZED) {
      if (static_cast<int>(mdprTrackedFrames_.size()) == mInitWindowSize_) {
        // 6. Initialize
        bool bInitializationState = false;
        bInitializationState = InitializeSLAMStructure();

        if (bInitializationState) meVIGEstimatorState_ = VIG_OK;
      }
    } else {
      // 6. Triangulate the Features in navigation Frame which are not
      // triangulated yet
      mpFeatureManager_->TriangulateCurFrameFts_MultFrames(mdKeyFrames_);

      // 7. Optimize
      gtsam::Pose3 CurPredPose = pFactorGraphEstimator->GetCurPredPose();
      InsertNewPoseInitialValues(CurPredPose * gExtrPose_c2i);
      UpdatePoseTimeStamps_KeyTimeStampMap(pIMU->msCurIMUData.T);
      AddVisualFactor(CurPredPose);
      Optimize();
    }

    if (meVIGEstimatorState_ == VIG_OK) {
      // 8. Update Poses in KeyFrames Queue of Estimator
      UpdateKeyFramesAndFeatures();

      // 9. Viewer: TODO
      mpViewer_->RunViewer(mpFeatureManager_, mdKeyFrames_,
                           mprsCurTrackedFrame_, mpFeatureTracker_->mCurImg);
    }

    // 10. grab the next image time
    mpFeatureTracker_->GrabNextImageEpoch(mifCamTime_);
  }
  // // Test
  // cout << mpViewer->NumOptimized() << endl;
}

void TestVSlamSystem::AddCurTrackedFrameAndKeyFrame(
    const pair<Frame, FrameWisedFeatures> &prsCurFrameWisedFeatures) {
  mdprTrackedFrames_.push_back(prsCurFrameWisedFeatures);

  KeyFrame CurFrameInKFQue(prsCurFrameWisedFeatures.first.FrameId,
                           prsCurFrameWisedFeatures.first.FrameEpoch, -1,
                           prsCurFrameWisedFeatures.first.Pose_c2w);
  mdKeyFrames_.push_back(CurFrameInKFQue);
  mBackFrameIDInKFQue_ = (int)mdKeyFrames_.size() - 1;
  assert(mBackFrameIDInKFQue_ >= 0 &&
         mBackFrameIDInKFQue_ <= gMaxFrameIdInKFQue);
}

void TestVSlamSystem::AddIdWisedFeatures(const gtsam::Pose3 &CurPredPose) {
  if (mpFeatureManager_->AddIdWisedFeaturesAndCheckParallax(
          mBackFrameIDInKFQue_, mdprTrackedFrames_.back()))
    meMarginFlag_ = MARGIN_OLD;
  else
    meMarginFlag_ = MARGIN_SECOND_NEW;

  // Test
  {
    if (meVIGEstimatorState_ == VIG_OK) {
      gtsam::Pose3 tstCurPose_i2w =
          CurPredPose *
          gExtrPose_c2i.inverse();  // body_P_sensor has been setted
      // PreIntegration
      // tstCurPose_i2w.print("tstCurPose_i2w ");

      // gtsam::Pose3 DeltPose_prein = tstCurPose_i2w.inverse() *
      // mOptimizedCurNavState.pose(); DeltPose_prein.print("DeltPose_prein: ");
      // Vector3d Att_prein = Vector3d(DeltPose_prein.rotation().roll(),
      // DeltPose_prein.rotation().pitch(), DeltPose_prein.rotation().yaw());
      // cout << "CurFrameId: " << mdprTrackedFrames.back().first.FrameId <<
      // endl
      //      << "Att_prein: " << Att_prein.transpose() * kRad2Deg << endl;
      // cout << "Trs_prein: " <<
      // DeltPose_prein.translation().matrix().transpose() << endl;

      // EpoGeo
      bool bEpiGeo = false;
      gtsam::Pose3 DeltPose_epigeo, CurPose_i2ref_epigeo;
      if (mpFeatureManager_->CalRelativePose_L2R(mBackFrameIDInKFQue_ - 1,
                                                 mBackFrameIDInKFQue_,
                                                 DeltPose_epigeo)) {
        gtsam::Pose3 CurPose_cam2ref_epigeo =
            mOptimizedCurPose_ * DeltPose_epigeo.inverse();
        CurPose_i2ref_epigeo = CurPose_cam2ref_epigeo * gExtrPose_c2i.inverse();
        // CurPose_i2ref_epigeo.print("CurPose_i2ref_epigeo ");
        bEpiGeo = true;
        // cout << "Trs_epigeo: " <<
        // CurPose_i2ref_epigeo.translation().matrix().transpose() << endl;

        // DeltPose_epigeo.print("DeltPose_epigeo: ");
        // Vector3d Att_epigeo = Vector3d(DeltPose_epigeo.rotation().roll(),
        // DeltPose_epigeo.rotation().pitch(),
        // DeltPose_epigeo.rotation().yaw()); cout << "Att_epigeo: " <<
        // Att_epigeo.transpose() * kRad2Deg << endl; cout << "Trs_epigeo: " <<
        // DeltPose_epigeo.translation().matrix().transpose() << endl;
      }

      // PnP
      gtsam::Pose3 CurPose_cam2ref_PnP = tstCurPose_i2w * gExtrPose_c2i,
                   CurPose_i2ref_PnP, DeltPose_pnp;
      bool bPnP = false;
      if (mpFeatureManager_->SolvePnP4CurFrameInKFQue(mdprTrackedFrames_.back(),
                                                      CurPose_cam2ref_PnP)) {
        bPnP = true;
        CurPose_i2ref_PnP = CurPose_cam2ref_PnP * gExtrPose_c2i.inverse();
        // CurPose_i2ref_PnP.print("CurPose_i2ref_PnP ");

        // DeltPose_pnp = CurPose_i2ref_PnP.inverse() *
        // mOptimizedCurNavState.pose(); DeltPose_pnp.print("DeltPose_pnp: ");
        // Vector3d Att_pnp = Vector3d(DeltPose_pnp.rotation().roll(),
        // DeltPose_pnp.rotation().pitch(), DeltPose_pnp.rotation().yaw()); cout
        // << "Att_pnp: " << Att_pnp.transpose() * kRad2Deg << endl; cout <<
        // "Trs_pnp: " << DeltPose_pnp.translation().matrix().transpose() <<
        // endl;
      }
      cout << "CurFrameId: " << mdprTrackedFrames_.back().first.FrameId << endl
           << "Att_prein: "
           << GTSAMRot2Att(tstCurPose_i2w.rotation()).transpose() << endl;
      if (bEpiGeo)
        cout << "Att_epigeo: "
             << GTSAMRot2Att(CurPose_i2ref_epigeo.rotation()).transpose()
             << endl;
      if (bPnP)
        cout << "Att_pnp: "
             << GTSAMRot2Att(CurPose_i2ref_PnP.rotation()).transpose() << endl;

      if (bEpiGeo)
        cout << "Trs_prein: "
             << tstCurPose_i2w.translation().matrix().transpose() << endl;
      if (bPnP)
        cout << "Trs_pnp: "
             << CurPose_i2ref_PnP.translation().matrix().transpose() << endl;
      cout << endl;
    }
  }
}

bool TestVSlamSystem::InitializeSLAMStructure() {
  /**** 1：Triangulate the Features in KeyFrames Queue ****/
  mpFeatureManager_->TriangulateCurFrameFts_MultFrames(mdKeyFrames_);
  assert(mdprTrackedFrames_.size() == mdKeyFrames_.size());

  /**** 2：Reinitialize FactorGraph and InitialValues ****/
  mPoseIndex_ = 0;
  mdKeyFrames_.front().NavStateIndex = mPoseIndex_;
  mOptimizedCurPose_ = mdKeyFrames_.front().Pose;

  mVSlamIsam2_ = gtsam::ISAM2(mIsamParams_);
  mVSlamNewInitialValues_.insert(X(mPoseIndex_), mOptimizedCurPose_);
  mvPosesTimeStamps_.clear();
  UpdatePoseTimeStamps_KeyTimeStampMap(mdKeyFrames_.front().FrameEpoch);
  // mtdmpNewKey_TimeStamps_.clear();

  /**** 3：Add Factors and Optimize ****/
  for (int i = 1; i < mdKeyFrames_.size(); i++) {
    InsertNewPoseInitialValues(mdKeyFrames_[i].Pose);
    UpdatePoseTimeStamps_KeyTimeStampMap(mdKeyFrames_[i].FrameEpoch);
    mdKeyFrames_[i].NavStateIndex = mPoseIndex_;
    // mVSlamNewInitialValues_.print("Initial ");
  }

  mpVSlamNewGraph_->add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), mdKeyFrames_[0].Pose, mNoiseModel_InitPose_));
  mpVSlamNewGraph_->add(gtsam::PriorFactor<gtsam::Pose3>(
      X(mPoseIndex_), mdKeyFrames_[mPoseIndex_].Pose, mNoiseModel_InitPose_));
  AddVisualFactor_Init();
  Optimize();
  meMarginFlag_ = MARGIN_OLD;

  return true;
}

void TestVSlamSystem::AddVisualFactor_Init() {
  assert(mdKeyFrames_.size() == mdprTrackedFrames_.size());

  for (int itFrm = 0; itFrm < mdKeyFrames_.size(); itFrm++) {
    mprsCurTrackedFrame_ = mdprTrackedFrames_[itFrm];
    int NumCurFrameFeatures = mprsCurTrackedFrame_.second.vFeatureIds.size();
    mprsCurTrackedFrame_.second.vbOptimized.resize(NumCurFrameFeatures, 0);
    for (int itFt = 0; itFt < NumCurFrameFeatures; itFt++) {
      int FeatureId = mprsCurTrackedFrame_.second.vFeatureIds[itFt];
      auto litIdWisedFeature =
          find_if(mpFeatureManager_->mlsIdWisedFeatures.begin(),
                  mpFeatureManager_->mlsIdWisedFeatures.end(),
                  [FeatureId](const IdWisedFeature &litIdWisedFeature) {
                    return litIdWisedFeature.FeatureId == FeatureId;
                  });
      assert(litIdWisedFeature != mpFeatureManager_->mlsIdWisedFeatures.end());

      // if the feature isn't(or maybe not) triangulated successfully
      if ((litIdWisedFeature->InvDepth < 0.01))
        continue;

      if (litIdWisedFeature->SolvedFlag == NOT_SOLVED) {
        mMapPointIndex_++;
        litIdWisedFeature->MapPointIndex = mMapPointIndex_;

        int itFrameIDInKFQue = litIdWisedFeature->StartFrameIDInKFQue - 1;
        gtsam::Point2 Measured_ref;
        vector<InvDepthFactor> vInvDepthFactors;
        for (auto &vitIdWisedFeaturePerFrame :
             litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames) {
          itFrameIDInKFQue++;
          assert(vitIdWisedFeaturePerFrame.CurFrameId ==
                 mdKeyFrames_[itFrameIDInKFQue].FrameId);

          if (itFrameIDInKFQue == litIdWisedFeature->StartFrameIDInKFQue) {
            Measured_ref = vitIdWisedFeaturePerFrame.UnDisXYZ.head(2);
          } else {
            gtsam::Point2 Measured = vitIdWisedFeaturePerFrame.UnDisXYZ.head(2);
            if (!CalcCurVisualNoiseModel(mdKeyFrames_[itFrameIDInKFQue].Pose,
                                         litIdWisedFeature->Point3w, Measured,
                                         litIdWisedFeature->MapPointIndex))
              continue;

            vInvDepthFactors.push_back(InvDepthFactor(
                X(mdKeyFrames_[litIdWisedFeature->StartFrameIDInKFQue]
                      .NavStateIndex),
                X(mdKeyFrames_[itFrameIDInKFQue].NavStateIndex),
                L(litIdWisedFeature->MapPointIndex), Measured_ref, Measured,
                mNoiseModel_Pixel_));
          }
        }

        if ((int)vInvDepthFactors.size() >= gCoVisualFramesThreshold - 1) {
          litIdWisedFeature->SolvedFlag = WATING_FOR_SOLVE;
          mVSlamNewInitialValues_.insert(L(litIdWisedFeature->MapPointIndex),
                                         litIdWisedFeature->InvDepth);

          for (int i = 0; i < (int)vInvDepthFactors.size(); i++)
            mpVSlamNewGraph_->add(vInvDepthFactors[i]);

          mprsCurTrackedFrame_.second.vbOptimized[itFt] = 1;
        } else {
          mMapPointIndex_--;
          litIdWisedFeature->MapPointIndex = -1;
          continue;
        }
      } else if (litIdWisedFeature->SolvedFlag == WATING_FOR_SOLVE) {
        mprsCurTrackedFrame_.second.vbOptimized[itFt] = 1;
        continue;
      } else
        continue;
    }
  }
}

void TestVSlamSystem::AddVisualFactor(const gtsam::Pose3 &CurPredPose) {
  mPredNextPose4Init_ = CurPredPose * gExtrPose_c2i;

  mdKeyFrames_.back().NavStateIndex = mPoseIndex_;
  mdKeyFrames_.back().Pose = mPredNextPose4Init_;
  mprsCurTrackedFrame_ = mdprTrackedFrames_.back();

  int NumCurFrameFeatures = mprsCurTrackedFrame_.second.vFeatureIds.size();
  mprsCurTrackedFrame_.second.vbOptimized.resize(NumCurFrameFeatures, 0);
  for (int itFt = 0; itFt < NumCurFrameFeatures; itFt++) {
    int FeatureId = mprsCurTrackedFrame_.second.vFeatureIds[itFt];
    auto litIdWisedFeature =
        find_if(mpFeatureManager_->mlsIdWisedFeatures.begin(),
                mpFeatureManager_->mlsIdWisedFeatures.end(),
                [FeatureId](const IdWisedFeature &litIdWisedFeature) {
                  return litIdWisedFeature.FeatureId == FeatureId;
                });
    assert(litIdWisedFeature != mpFeatureManager_->mlsIdWisedFeatures.end());

    // if the feature isn't triangulated successfully
    if ((litIdWisedFeature->InvDepth < 0.01))
      continue;

    if (litIdWisedFeature->SolvedFlag == NOT_SOLVED) {
      mMapPointIndex_++;
      litIdWisedFeature->MapPointIndex = mMapPointIndex_;

      int itFrameIDInKFQue = litIdWisedFeature->StartFrameIDInKFQue - 1;
      gtsam::Point2 Measured_ref;
      vector<InvDepthFactor> vInvDepthFactors;
      for (auto &vitIdWisedFeaturePerFrame :
           litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames) {
        itFrameIDInKFQue++;
        assert(vitIdWisedFeaturePerFrame.CurFrameId ==
               mdKeyFrames_[itFrameIDInKFQue].FrameId);

        if (itFrameIDInKFQue == litIdWisedFeature->StartFrameIDInKFQue) {
          Measured_ref = vitIdWisedFeaturePerFrame.UnDisXYZ.head(2);
        } else {
          gtsam::Point2 Measured = vitIdWisedFeaturePerFrame.UnDisXYZ.head(2);
          if (!CalcCurVisualNoiseModel(mdKeyFrames_[itFrameIDInKFQue].Pose,
                                       litIdWisedFeature->Point3w, Measured,
                                       litIdWisedFeature->MapPointIndex))
            continue;
          // Notice that mNoiseModel_Pixel_ has been changed
          vInvDepthFactors.push_back(InvDepthFactor(
              X(mdKeyFrames_[litIdWisedFeature->StartFrameIDInKFQue]
                    .NavStateIndex),
              X(mdKeyFrames_[itFrameIDInKFQue].NavStateIndex),
              L(litIdWisedFeature->MapPointIndex), Measured_ref, Measured,
              mNoiseModel_Pixel_));
        }
      }

      if ((int)vInvDepthFactors.size() >= gCoVisualFramesThreshold - 1) {
        litIdWisedFeature->SolvedFlag = WATING_FOR_SOLVE;
        mVSlamNewInitialValues_.insert(L(litIdWisedFeature->MapPointIndex),
                                       litIdWisedFeature->InvDepth);

        for (int i = 0; i < (int)vInvDepthFactors.size(); i++)
          mpVSlamNewGraph_->add(vInvDepthFactors[i]);
      } else {
        mMapPointIndex_--;
        litIdWisedFeature->MapPointIndex = -1;
        continue;
      }

      mprsCurTrackedFrame_.second.vbOptimized[itFt] = 1;
      continue;
    }

    gtsam::Point2 Measured_ref =
        litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames[0].UnDisXYZ.head(2);
    gtsam::Point2 Measured =
        mprsCurTrackedFrame_.second.vUnDisXYZs[itFt].head(2);
    if (!CalcCurVisualNoiseModel(mdKeyFrames_.back().Pose,
                                 litIdWisedFeature->Point3w, Measured,
                                 litIdWisedFeature->MapPointIndex))
      continue;
    // Notice that mNoiseModel_Pixel_ has been changed
    mpVSlamNewGraph_->add(InvDepthFactor(
        X(mdKeyFrames_[litIdWisedFeature->StartFrameIDInKFQue].NavStateIndex),
        X(mPoseIndex_), L(litIdWisedFeature->MapPointIndex), Measured_ref,
        Measured, mNoiseModel_Pixel_));

    litIdWisedFeature->SolvedFlag = WATING_FOR_SOLVE;
    mprsCurTrackedFrame_.second.vbOptimized[itFt] = 1;
  }
}

void TestVSlamSystem::InsertNewPoseInitialValues(
    const gtsam::Pose3 &PredNextPose4Init) {
  mPoseIndex_++;
  mVSlamNewInitialValues_.insert(X(mPoseIndex_), PredNextPose4Init);
}

void TestVSlamSystem::Optimize() {
  // Update Solver
  mVSlamIsam2_.update(*mpVSlamNewGraph_, mVSlamNewInitialValues_);
  // mVSlamIsam2_.print();
  mVSlamResultValues_ = mVSlamIsam2_.calculateEstimate();

  // mVSlamIsam2_.getDelta().print();
  // mVSlamResultValues_.print("Current estimate: ");

  // for (int i = 0; i <= mPoseIndex_; i++)
  // {
  //     gtsam::Pose3 CurIMUPose3ResultValue =
  //     mVSlamResultValues_.at<gtsam::Pose3>(X(i)) * gExtrPose_c2i.inverse();

  //     // cout << "x" << i << endl;
  //     Vector3d CurCamAttDeg =
  //     GTSAMRot2Att(CurIMUPose3ResultValue.rotation());
  //     // cout << CurCamAttDeg.transpose() << endl;
  //     CurIMUPose3ResultValue.translation().print("");
  // }

  mOptimizedCurPose_ = mVSlamResultValues_.at<gtsam::Pose3>(X(mPoseIndex_));

  // // clear graph/values and Reset the preintegration object
  // mMarginals = gtsam::Marginals(*mpVSlamNewGraph_, mVSlamResultValues_);

  mpVSlamNewGraph_->resize(0);
  mVSlamNewInitialValues_.clear();
}

void TestVSlamSystem::UpdateKeyFramesAndFeatures() {
  // Update Poses in KeyFrames Queue of Estimator
  for (auto &ditFrameInSlidingWindow : mdKeyFrames_) {
    if (ditFrameInSlidingWindow.NavStateIndex < 0) continue;

    ditFrameInSlidingWindow.SetPose(mVSlamResultValues_.at<gtsam::Pose3>(
        X(ditFrameInSlidingWindow.NavStateIndex)));
  }

  mpFeatureManager_->SetFeaturesDepth(mVSlamResultValues_, mdKeyFrames_);

  if (mBackFrameIDInKFQue_ >= gMaxFrameIdInKFQue) SlideWindow();

  mpFeatureManager_->RemoveFailures();
}

void TestVSlamSystem::SlideWindow() {
  if (meMarginFlag_ == MARGIN_OLD) {
    mdKeyFrames_.pop_front();

    // for (deque<pair<Frame, FrameWisedFeatures>>::iterator ditprTrackedFrame =
    // mdprTrackedFrames_.begin();
    //      ditprTrackedFrame != mdprTrackedFrames_.end();
    //      ditprTrackedFrame++)
    // {
    //     if (ditprTrackedFrame->first.FrameId == mdKeyFrames_.front().FrameId)
    //     {
    //         mdprTrackedFrames_.erase(mdprTrackedFrames_.begin(),
    //         ditprTrackedFrame); break;
    //     }
    // }

    int ErsFrmID = mdKeyFrames_.front().FrameId;
    mdKeyFrames_.pop_front();
    mpFeatureManager_->RemoveFront(ErsFrmID);
  } else {
    // auto litTrackedFrame = mdprTrackedFrames_.end();
    // litTrackedFrame--;
    // mdprTrackedFrames_.erase(litTrackedFrame);

    auto ditFramesId_PoseInKFQue = mdKeyFrames_.end() - 1;
    ditFramesId_PoseInKFQue--;
    int ErsFrmID = ditFramesId_PoseInKFQue->FrameId;
    mdKeyFrames_.erase(ditFramesId_PoseInKFQue);

    mpFeatureManager_->RemoveBack(ErsFrmID, mBackFrameIDInKFQue_);
  }
}

}  // namespace VIG
