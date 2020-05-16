#include "FactorGraphEstimator.h"

namespace VIG {
#define MAX_ITERATION_ISAM_UPD_CAL 3

int FactorGraphEstimator::mGNSSCounter = 0;
int FactorGraphEstimator::mIMUCounterBetwDualObss = 0;
int FactorGraphEstimator::mNavStateIndex = 0;   // start from 0
int FactorGraphEstimator::mMapPointIndex = -1;  // start from 0

FactorGraphEstimator::FactorGraphEstimator(
    const Configuration *pConfiguration) {
  /**** I. Initial Conditions ****/
  // 1. Initial Position/Velocity/Attitude
  gtsam::Rot3 InitialRotb2n =
      gtsam::Rot3(pConfiguration->mIniQi2n.toRotationMatrix());
  mPoseb2n = gtsam::Pose3(InitialRotb2n, gtsam::Point3(0.0, 0.0, 0.0));
  mVelb2n = pConfiguration->mIniVn;
  // 2. Assemble prior noise model and add it the graph.
  mNoiseModel_InitPose = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << pConfiguration->mInitAttSigma * Vector3d(1, 1, 1),
       pConfiguration->mInitPosSigma * Vector3d(1, 1, 1))
          .finished());
  mNoiseModel_InitVel =
      gtsam::noiseModel::Isotropic::Sigma(3, pConfiguration->mInitVelSigma);
  mNoiseModel_Initbias = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << pConfiguration->mInitAccBiasSigma *
                               Vector3d(1, 1, 1),
       pConfiguration->mInitGyroBiasSigma * Vector3d(1, 1, 1))
          .finished());

  /**** II. IMU Settings ****/
  // We use the sensor specs to build the noise model for the IMU factor.
  IMUErrorModel4PreIntegration sIMUErrorModel4PreIntegration;
  sIMUErrorModel4PreIntegration =
      GetIMUErrorModel4PreIntegration(pConfiguration->mIMUType);
  if (pConfiguration->mstrNavFrame == "NED")
    mIMU_params =
        gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD();
  else if (pConfiguration->mstrNavFrame == "ENU")
    mIMU_params =
        gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
  else
    return;
  // PreintegrationBase params:
  mIMU_params->accelerometerCovariance =
      sIMUErrorModel4PreIntegration
          .accelerometerCovariance;  // acc white noise in continuous
  mIMU_params->integrationCovariance =
      sIMUErrorModel4PreIntegration
          .integrationCovariance;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  mIMU_params->gyroscopeCovariance =
      sIMUErrorModel4PreIntegration
          .gyroscopeCovariance;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  mIMU_params->biasAccCovariance =
      sIMUErrorModel4PreIntegration
          .biasAccCovariance;  // acc bias in continuous
  mIMU_params->biasOmegaCovariance =
      sIMUErrorModel4PreIntegration
          .biasOmegaCovariance;  // gyro bias in continuous
  mIMU_params->biasAccOmegaInt = sIMUErrorModel4PreIntegration.biasAccOmegaInt;
  mIMU_params->n_gravity = gtsam::Vector3(gG);
  mIMU_params->omegaCoriolis = gtsam::Vector3(pConfiguration->mw_coriolis);

  mInitSigma6_BetwBias << pConfiguration->mInitAccBetwBiasSigma *
                              Vector3d(1, 1, 1),
      pConfiguration->mInitGyroBetwBiasSigma * Vector3d(1, 1, 1);
  mNoiseModel_BetwBias =
      gtsam::noiseModel::Diagonal::Sigmas(mInitSigma6_BetwBias);

  /**** III. GNSS noise model ****/
  if (gbUseGNSS) {
    mInitSigma3_GNSSpos << pConfiguration->mGNSSPosSigma * Vector3d(1, 1, 2);

    mGNSSResidulThreshold = pConfiguration->mGNSSResidulThreshold;

    if ((int)pConfiguration->mvGNSSResidulThresholdFactor.size() != 2) {
      cerr << "Wrong GNSS std factor size ";
      exit(1);
    }
    marGNSSResidulThresholdFactor[0] =
        pConfiguration->mvGNSSResidulThresholdFactor[0];
    marGNSSResidulThresholdFactor[1] =
        pConfiguration->mvGNSSResidulThresholdFactor[1];
  }

  /**** IV. Camera settings ****/
  if (gbUseCam) {
    mBackFrIDInKFQue = 0;
    mInitWinSize = pConfiguration->mInitWindowSize;

    mInitSigma_Pixel = pConfiguration->mSigma_Pixel;
    mNoiseModel_Pixel =
        gtsam::noiseModel::Isotropic::Sigma(2, mInitSigma_Pixel);
    mPixelResidulThreshold = pConfiguration->mPixelResidulThreshold;
    if ((int)pConfiguration->mvPixelResidulThresholdFactor.size() != 2) {
      cerr << "Wrong GNSS std factor size ";
      exit(1);
    }
    mvPixResiThreFactor[0] = pConfiguration->mvPixelResidulThresholdFactor[0];
    mvPixResiThreFactor[1] = pConfiguration->mvPixelResidulThresholdFactor[1];

    /**** V. Initialization settings****/
    ClearVIGInitialization();
    // mvInitialIMUs.push_back(*pVIG->mpIMU);
    // mdInitialGNSSs.push_back(*pVIG->mpGNSS);

    mpFeatureManager = new FeatureManager(pConfiguration);
  }

  /**** VI. Solver settings ****/
  // 1. IMU Preintegration
  mOptimizedCurBias = gtsam::imuBias::ConstantBias(
      gtsam::Vector3(pConfiguration->mInitiIMUErrorStates.AccBias),
      gtsam::Vector3(pConfiguration->mInitiIMUErrorStates.GyroBias));
#ifdef USE_COMBINED
  // mOptimizedCurBias: ConstantBias set 0 by default
  mpIMUPreIntg = new gtsam::PreintegratedCombinedMeasurements(
      mIMU_params, mOptimizedCurBias);
#else
  mpIMUPreIntg =
      new gtsam::PreintegratedImuMeasurements(mIMU_params, mOptimizedCurBias);
#endif

  // 2. Factors Involving LandMark with big error To Remove
  mNewFactorIndex = 0;
  mFactorsWtLandMarkToRemove.clear();

  // 3. ISAM2 Parameters
  if (pConfiguration->mstrOptimizationParams == "GaussNewton")
    mIsamParams.setOptimizationParams(gtsam::ISAM2GaussNewtonParams());
  else if (pConfiguration->mstrOptimizationParams == "Dogleg")
    mIsamParams.setOptimizationParams(gtsam::ISAM2DoglegParams());
  else {
    cout << "OptimizationParams Error!" << endl;
    return;
  }
  mIsamParams.setFactorization("CHOLESKY");
  mIsamParams.setRelinearizeSkip(
      pConfiguration
          ->mIsamParams_relinearizeSkip);  // ISAM2Params set 10 as default
  mIsamParams.findUnusedFactorSlots =
      pConfiguration->mbIsamParams_findUnusedFactorSlots;
  mIsamParams.evaluateNonlinearError =
      pConfiguration->mbIsamParams_evaluateNonlinearError;
  // gtsam::FastMap<char, gtsam::Vector> thresholds;
  // thresholds['X'] = (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.5, 0.5,
  // 0.5).finished();
  // // 0.1 rad rotation threshold, 0.5 m translation threshold
  // thresholds['L'] = gtsam::Vector3(1.0, 1.0, 1.0); // 1.0 m landmark position
  // threshold mIsamParams.relinearizeThreshold = thresholds;
  // mIsamParams.setEnablePartialRelinearizationCheck(true);

  // 4. Optimizer
  mstrOptimizer = pConfiguration->mstrOptimizer;
  if (mstrOptimizer == "ISAM2")
    mIsam2 = gtsam::ISAM2(mIsamParams);
  else if (mstrOptimizer == "IncrementalFixedLagSmoother") {
    mSmootherLag = pConfiguration->mSmootherLag;
    mNumFLSmootherUpdIters = pConfiguration->mNumFLSmootherUpdIters;
    mFLSmootherISAM2 =
        gtsam::IncrementalFixedLagSmoother(mSmootherLag, mIsamParams);
  } else {
    cout << "Optimizer Error!" << endl;
    return;
  }

  /**** VII. Prepare factor graph ****/
  mNewInitialValues.insert(X(mNavStateIndex), mPoseb2n);
  mNewInitialValues.insert(V(mNavStateIndex), mVelb2n);
  mNewInitialValues.insert(B(mNavStateIndex), mOptimizedCurBias);

  mpNewGraph->add(gtsam::PriorFactor<gtsam::Pose3>(X(mNavStateIndex), mPoseb2n,
                                                   mNoiseModel_InitPose));
  mpNewGraph->add(gtsam::PriorFactor<gtsam::Vector3>(V(mNavStateIndex), mVelb2n,
                                                     mNoiseModel_InitVel));
  mpNewGraph->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      B(mNavStateIndex), mOptimizedCurBias, mNoiseModel_Initbias));

  mOptimizedCurNavState = gtsam::NavState(mPoseb2n, mVelb2n);
  mPredNextNavState4Init = mOptimizedCurNavState;
}

void FactorGraphEstimator::ClearVIGInitialization() {
  mdprInitialTrackedFrames.clear();
  mdInitialKeyFrames.clear();
  mvInitialIMUs.clear();
  mdInitialGNSSs.clear();
}

gtsam::Pose3 FactorGraphEstimator::GetCurPredPose() {
  mPredNextNavState4Init =
      mpIMUPreIntg->predict(mOptimizedCurNavState, mOptimizedCurBias);

  return mPredNextNavState4Init.pose();
}

void FactorGraphEstimator::GetGNSSSigmaFactor(const Vector3d &GNSSResidual,
                                              Vector3d &GNSSSigmaFactor) {
  for (int i = 0; i < 3; i++) {
    double absGNSSRes_i = fabs(GNSSResidual(i));
    if (absGNSSRes_i <=
        marGNSSResidulThresholdFactor[0] * mGNSSResidulThreshold(i))
      GNSSSigmaFactor(i) = 1;
    else if (absGNSSRes_i >
                 marGNSSResidulThresholdFactor[0] * mGNSSResidulThreshold(i) &&
             absGNSSRes_i <=
                 marGNSSResidulThresholdFactor[1] * mGNSSResidulThreshold(i))
      GNSSSigmaFactor(i) = (marGNSSResidulThresholdFactor[1] -
                            marGNSSResidulThresholdFactor[0]) /
                           (marGNSSResidulThresholdFactor[1] - 1);
    else if (absGNSSRes_i >
             marGNSSResidulThresholdFactor[1] * mGNSSResidulThreshold(i))
      GNSSSigmaFactor(i) = 10 * absGNSSRes_i / mGNSSResidulThreshold(i);
    else {
      cout << "Wrong GNSS Postion Residuals! " << endl;
      return;
    }
  }
}

void FactorGraphEstimator::GetGNSSNewResThreshold(
    const Vector3d &GNSSResidual, const Vector3d &GNSSSigmaFactor) {
  for (int i = 0; i < 3; i++) {
    mGNSSResidulThreshold(i) +=
        (fabs(GNSSResidual(i)) / GNSSSigmaFactor(i) -
         mGNSSResidulThreshold(i)) /
        (mGNSSCounter + 1);  // regard initial threshold as the one counter
  }
}

bool FactorGraphEstimator::CalcCurVisualNoiseModel(
    const gtsam::Pose3 &CurKFPose, const gtsam::Point3 &Point3w,
    const gtsam::Point2 &Measured, int CurMapPointIndex) {
  // Project landmark into Pose2
  gtsam::PinholeBase normalized_plane(CurKFPose);
  Vector2d ErrorRePrj = normalized_plane.project2(Point3w) - Measured;

  if (fabs(ErrorRePrj(0)) < mInitSigma_Pixel &&
      fabs(ErrorRePrj(1)) < mInitSigma_Pixel) {
    // cout << CurMapPointIndex << "," << ErrorRePrj(0) << "," << ErrorRePrj(1)
    // << endl;
    mNoiseModel_Pixel =
        gtsam::noiseModel::Isotropic::Sigma(2, mInitSigma_Pixel);
    return true;
  } else if (fabs(ErrorRePrj(0)) < 1.5 * mInitSigma_Pixel &&
             fabs(ErrorRePrj(1)) < 1.5 * mInitSigma_Pixel) {
    // cout << CurMapPointIndex << "," << ErrorRePrj(0) << "," << ErrorRePrj(1)
    // << endl;
    mNoiseModel_Pixel =
        gtsam::noiseModel::Isotropic::Sigma(2, 1.5 * mInitSigma_Pixel);
    return true;
  } else if (fabs(ErrorRePrj(0)) < 2 * mInitSigma_Pixel &&
             fabs(ErrorRePrj(1)) < 2 * mInitSigma_Pixel) {
    // cout << CurMapPointIndex << "," << ErrorRePrj(0) << "," << ErrorRePrj(1)
    // << endl;
    mNoiseModel_Pixel =
        gtsam::noiseModel::Isotropic::Sigma(2, 2 * mInitSigma_Pixel);
    return true;
  } else
    return false;
}

void FactorGraphEstimator::PreIntegrate(const IMU *pIMU) {
  mIMUCounterBetwDualObss++;
  double dt = pIMU->msCurIMUData.T - pIMU->msPreIMUData.T;
  mpIMUPreIntg->integrateMeasurement(
      (pIMU->msPreIMUData.Acc + pIMU->msCurIMUData.Acc) * 0.5,
      (pIMU->msPreIMUData.Gyro + pIMU->msCurIMUData.Gyro) * 0.5, dt);
}

void FactorGraphEstimator::AddNewVision(
    const pair<Frame, FrameWisedFeatures> &prsCurFrameWisedFeatures) {
  if (!mbInitPub) {
    mbInitPub = 1;
  } else if (!mbInitFeature) {
    mbInitFeature = 1;
  } else {
    // Add Current Tracked Frame
    mdprTrackedFrames.push_back(prsCurFrameWisedFeatures);
    mdprTrackedFrames.back().first.Vel_i2w = mPredNextNavState4Init.v();

    // Add Key Frame
    KeyFrame CurFrameInKFQue(prsCurFrameWisedFeatures.first.FrameId,
                             prsCurFrameWisedFeatures.first.FrameEpoch, -1,
                             prsCurFrameWisedFeatures.first.Pose_c2w);
    mdKeyFrames.push_back(CurFrameInKFQue);
    mBackFrIDInKFQue = (int)mdKeyFrames.size() - 1;
    if (!(mBackFrIDInKFQue >= 0 && mBackFrIDInKFQue <= gMaxFrameIdInKFQue)) {
      cerr << "Wrong Back Frame ID In KFQue!";
      exit(1);
    }

    // Add IdWisedFeatures
    if (mpFeatureManager->AddIdWisedFeaturesAndCheckParallax(
            mBackFrIDInKFQue, mdprTrackedFrames.back()))
      meMarginFlag = MARGIN_OLD;
    else
      meMarginFlag = MARGIN_SECOND_NEW;
  }
}

void FactorGraphEstimator::TriangulateCurFrameFeatures() {
  mpFeatureManager->TriangulateCurFrameFts_MultFrames(mdKeyFrames);
}

void FactorGraphEstimator::AddIMUFactor() {
  mNavStateIndex++;

  // Add IMU Factor
#ifdef USE_COMBINED
  gtsam::PreintegratedCombinedMeasurements *Preint_imu_combined =
      dynamic_cast<gtsam::PreintegratedCombinedMeasurements *>(mpIMUPreIntg);
  gtsam::CombinedImuFactor IMU_Factor(X(mNavStateIndex - 1),
                                      V(mNavStateIndex - 1), X(mNavStateIndex),
                                      V(mNavStateIndex), B(mNavStateIndex - 1),
                                      B(mNavStateIndex), *Preint_imu_combined);

  mpNewGraph->add(IMU_Factor);
#else
  gtsam::PreintegratedImuMeasurements *Preint_imu =
      dynamic_cast<gtsam::PreintegratedImuMeasurements *>(mpIMUPreIntg);
  gtsam::ImuFactor IMU_Factor(X(mNavStateIndex - 1), V(mNavStateIndex - 1),
                              X(mNavStateIndex), V(mNavStateIndex),
                              B(mNavStateIndex - 1), *Preint_imu);
  mpNewGraph->add(IMU_Factor);
  gtsam::imuBias::ConstantBias Zero_bias(Vector3d(0, 0, 0), Vector3d(0, 0, 0));
  // mNoiseModel_BetwBias =
  // gtsam::noiseModel::Diagonal::Sigmas(pow(mIMUCounterBetwDualObss, 0.5) *
  //                                                               mInitSigma6_BetwBias);
  // Add Bias(Between) Factor
  mpNewGraph->add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(mNavStateIndex - 1), B(mNavStateIndex), Zero_bias,
      mNoiseModel_BetwBias));
#endif
  // mpNewGraph->print();
}

void FactorGraphEstimator::AddGNSSSolFactor(const GNSS *pGNSS) {
  // Vector3d GNSSSigmaFactor;
  // GetGNSSSigmaFactor(GNSSResidual, GNSSSigmaFactor);
  // mNoiseModel_GNSSpos = gtsam::noiseModel::Diagonal::Sigmas(
  //     GNSSSigmaFactor.cwiseProduct(mInitSigma3_GNSSpos));
  mNoiseModel_GNSSpos =
      gtsam::noiseModel::Diagonal::Sigmas(2 * pGNSS->msGNSSSolData.Std.head(3));
  // Add GNSS Factor
  gtsam::GPSFactor GNSSFactor(
      X(mNavStateIndex),
      gtsam::Point3(pGNSS->msGNSSSolData.bodyPoint_NavFrame(0),
                    pGNSS->msGNSSSolData.bodyPoint_NavFrame(1),
                    pGNSS->msGNSSSolData.bodyPoint_NavFrame(2)),
      mNoiseModel_GNSSpos);
  mpNewGraph->add(GNSSFactor);

  // mGNSSCounter++;
  // GetGNSSNewResThreshold(GNSSResidual, GNSSSigmaFactor);
  mIMUCounterBetwDualObss = 0;
}

void FactorGraphEstimator::AddVisualFactor() {
  mPredNextNavState4Init =
      mpIMUPreIntg->predict(mOptimizedCurNavState, mOptimizedCurBias);

  mdKeyFrames.back().NavStateIndex = mNavStateIndex;
  mdKeyFrames.back().Pose = mPredNextNavState4Init.pose() * gExtrPose_c2i;
  mprsCurTrackedFrame = mdprTrackedFrames.back();
  // gtsam::Pose3 CurFramePose = mdKeyFrames.back().Pose;

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
    if (litIdWisedFeature == mpFeatureManager->mlsIdWisedFeatures.end()) {
      cerr << "No such feture founded!";
      exit(1);
    }

    // if the feature isn't(or maybe not) triangulated successfully
    if (litIdWisedFeature->InvDepth < 0.01)
      continue;

    if (litIdWisedFeature->SolvedFlag == NOT_SOLVED) {
      mMapPointIndex++;
      litIdWisedFeature->MapPointIndex = mMapPointIndex;

      int itFrameIDInKFQue = litIdWisedFeature->StartFrameIDInKFQue - 1;
      gtsam::Point2 Measured_ref;
      vector<InvDepthFactor> vInvDepthFactors;
      for (auto &vitIdWisedFeaturePerFrame :
           litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames) {
        itFrameIDInKFQue++;
        if (vitIdWisedFeaturePerFrame.CurFrameId !=
            mdKeyFrames[itFrameIDInKFQue].FrameId) {
          cerr << "Wrong frame id for current feature!";
          exit(1);
        }

        if (itFrameIDInKFQue == litIdWisedFeature->StartFrameIDInKFQue) {
          Measured_ref = vitIdWisedFeaturePerFrame.UnDisXYZ.head(2);
        } else {
          gtsam::Point2 Measured = vitIdWisedFeaturePerFrame.UnDisXYZ.head(2);
          if (!CalcCurVisualNoiseModel(mdKeyFrames[itFrameIDInKFQue].Pose,
                                       litIdWisedFeature->Point3w, Measured,
                                       litIdWisedFeature->MapPointIndex))
            continue;

          vInvDepthFactors.push_back(InvDepthFactor(
              X(mdKeyFrames[litIdWisedFeature->StartFrameIDInKFQue]
                    .NavStateIndex),
              X(mdKeyFrames[itFrameIDInKFQue].NavStateIndex),
              L(litIdWisedFeature->MapPointIndex), Measured_ref, Measured,
              mNoiseModel_Pixel, gExtrPose_c2i));
        }
      }

      if ((int)vInvDepthFactors.size() >= gCoVisualFramesThreshold - 1) {
        litIdWisedFeature->SolvedFlag = WATING_FOR_SOLVE;
        mNewInitialValues.insert(L(litIdWisedFeature->MapPointIndex),
                                 litIdWisedFeature->InvDepth);

        for (int i = 0; i < (int)vInvDepthFactors.size(); i++)
          mpNewGraph->add(vInvDepthFactors[i]);
      } else {
        mMapPointIndex--;
        litIdWisedFeature->MapPointIndex = -1;
        continue;
      }

      mprsCurTrackedFrame.second.vbOptimized[itFt] = 1;
      continue;
    }

    gtsam::Point2 Measured_ref =
        litIdWisedFeature->vIdWisedFeatureInfoOnAllFrames[0].UnDisXYZ.head(2);
    gtsam::Point2 Measured =
        mprsCurTrackedFrame.second.vUnDisXYZs[itFt].head(2);
    if (!CalcCurVisualNoiseModel(mdKeyFrames.back().Pose,
                                 litIdWisedFeature->Point3w, Measured,
                                 litIdWisedFeature->MapPointIndex))
      continue;

    mpNewGraph->add(InvDepthFactor(
        X(mdKeyFrames[litIdWisedFeature->StartFrameIDInKFQue].NavStateIndex),
        X(mNavStateIndex), L(litIdWisedFeature->MapPointIndex), Measured_ref,
        Measured, mNoiseModel_Pixel, gExtrPose_c2i));

    mprsCurTrackedFrame.second.vbOptimized[itFt] = 1;
  }

  mIMUCounterBetwDualObss = 0;
}

void FactorGraphEstimator::UpdateNavStateTimeStamps_KeyTimeStampMap(
    double NavStateIMUTime) {
  mvNavStatesTimeStamps.push_back(NavStateIMUTime);

  mtdmpNewKey_TimeStamps[X(mNavStateIndex)] = NavStateIMUTime;
  mtdmpNewKey_TimeStamps[V(mNavStateIndex)] = NavStateIMUTime;
  mtdmpNewKey_TimeStamps[B(mNavStateIndex)] = NavStateIMUTime;
  if (mMapPointIndex >= 0) {
    for (auto &litIdWisedFeature : mpFeatureManager->mlsIdWisedFeatures) {
      if (litIdWisedFeature.SolvedFlag == WATING_FOR_SOLVE)
        mtdmpNewKey_TimeStamps[L(litIdWisedFeature.MapPointIndex)] =
            NavStateIMUTime;
    }
  }
}

void FactorGraphEstimator::InsertNewNavStateInitialValues() {
  mPredNextNavState4Init =
      mpIMUPreIntg->predict(mOptimizedCurNavState, mOptimizedCurBias);
  mNewInitialValues.insert(X(mNavStateIndex), mPredNextNavState4Init.pose());
  mNewInitialValues.insert(V(mNavStateIndex), mPredNextNavState4Init.v());
  mNewInitialValues.insert(B(mNavStateIndex), mOptimizedCurBias);
}

void FactorGraphEstimator::GetFactorsWtLandMarkToRemove(
    const gtsam::VectorValues &CurDeltas) {
  mFactorsWtLandMarkToRemove.clear();
  gtsam::NonlinearFactorGraph CurISAMFactorGraph =
      mFLSmootherISAM2.getFactors();

  for (auto itDelta : CurDeltas) {
    // gtsam::Key key = itDelta.first;
    gtsam::Symbol itSymbol(itDelta.first);
    if (itSymbol.chr() == 'l') {
      gtsam::Vector3 CurLandMarkDelta = itDelta.second.cwiseAbs();

      if (CurLandMarkDelta(0) >= 3.0 || CurLandMarkDelta(1) >= 3.0 ||
          CurLandMarkDelta(2) >= 3.0) {
        if (mNewFactorIndex >= CurISAMFactorGraph.size()) {
          cerr << "Wrong New Factor Index!";
          exit(1);
        }

        // Mark it in mpFeatureManager
        int MapPointIndex = itSymbol.index();
        auto litIdWisedFeature =
            find_if(mpFeatureManager->mlsIdWisedFeatures.begin(),
                    mpFeatureManager->mlsIdWisedFeatures.end(),
                    [MapPointIndex](const IdWisedFeature &litIdWisedFeature) {
                      return litIdWisedFeature.MapPointIndex == MapPointIndex;
                    });
        litIdWisedFeature->SolvedFlag = FAILED_SOLVED;

        // Gather the involved factors
        for (int i = mNewFactorIndex; i < CurISAMFactorGraph.size(); i++) {
          if (CurISAMFactorGraph.at(i)->find(itDelta.first) !=
              CurISAMFactorGraph.at(i)->keys().end())
            mFactorsWtLandMarkToRemove.push_back(i);
        }
      }
    }
  }

  // Bug here
  if (!mFactorsWtLandMarkToRemove.empty())
    mNewFactorIndex = mFactorsWtLandMarkToRemove.back() + 1;
}

void FactorGraphEstimator::Optimize() {
  // Update Solver
  if (mstrOptimizer == "ISAM2") {
    mIsam2.update(*mpNewGraph, mNewInitialValues);
    // mIsam2.print();
    mResultValues = mIsam2.calculateEstimate();
  } else if (mstrOptimizer == "IncrementalFixedLagSmoother") {
    mFLSmootherISAM2.update(
        *mpNewGraph, mNewInitialValues,
        mtdmpNewKey_TimeStamps);  //, mFactorsWtLandMarkToRemove);
    // for (int j = 0; j < mNumFLSmootherUpdIters; ++j)
    // {
    //     // Optionally perform multiple iSAM2 iterations
    //     mFLSmootherISAM2.update();
    // }
    // // Test
    // cout << endl
    //      << "Nav" << mNavStateIndex << ":" << endl;
    // mFLSmootherISAM2.getLinearizationPoint().print("theta_: ");

    // // Test
    // gtsam::Values theta_out = mFLSmootherISAM2.getLinearizationPoint();
    // PrintCurFactorGraph(theta_out);

    gtsam::Values FLResultValues = mFLSmootherISAM2.calculateEstimate();

    // // Test
    // PrintCurFactorGraph(FLResultValues);

    UpdateResults_FLSmoother(FLResultValues);
  }

  // mFLSmootherISAM2.getDelta().print();
  mResultValues.print("Current estimate: ");

  mOptimizedCurNavState =
      gtsam::NavState(mResultValues.at<gtsam::Pose3>(X(mNavStateIndex)),
                      mResultValues.at<gtsam::Vector3>(V(mNavStateIndex)));
  mOptimizedCurBias =
      mResultValues.at<gtsam::imuBias::ConstantBias>(B(mNavStateIndex));

  // // clear graph/values and Reset the preintegration object
  // mMarginals = gtsam::Marginals(*mpNewGraph, mResultValues);

  mtdmpNewKey_TimeStamps.clear();
  mpNewGraph->resize(0);
  mNewInitialValues.clear();
  mpIMUPreIntg->resetIntegrationAndSetBias(mOptimizedCurBias);
}

void FactorGraphEstimator::UpdateResults_FLSmoother(
    const gtsam::Values &FLResultValues) {
  for (auto itValue : FLResultValues) {
    // update, if the key exsits in the old resultvalues
    if (mResultValues.exists(itValue.key))
      mResultValues.update(itValue.key, itValue.value);
    // insert, if the key is new
    else
      mResultValues.insert(itValue.key, itValue.value);
  }
}

void FactorGraphEstimator::UpdateKeyFramesAndFeatures() {
  // Update Poses in KeyFrames Queue of Estimator
  for (auto &ditKeyFrame : mdKeyFrames) {
    if (ditKeyFrame.NavStateIndex < 0) continue;

    ditKeyFrame.Pose =
        mResultValues.at<gtsam::Pose3>(X(ditKeyFrame.NavStateIndex)) *
        gExtrPose_c2i;
  }

  // Update 3D Points in World frame
  for (auto &litIdWisedFeature : mpFeatureManager->mlsIdWisedFeatures) {
    litIdWisedFeature.UsedFramesNum =
        litIdWisedFeature.vIdWisedFeatureInfoOnAllFrames.size();
    if (!(litIdWisedFeature.UsedFramesNum >= gCoVisualFramesThreshold &&
          litIdWisedFeature.StartFrameIDInKFQue < gMaxFrameIdInKFQue - 2) ||
        litIdWisedFeature.MapPointIndex < 0)
      continue;

    // 1. set the point3 in world frame
    litIdWisedFeature.Point3w =
        mResultValues.at<gtsam::Point3>(L(litIdWisedFeature.MapPointIndex));

    // 2. calculate the depth in the start frame in SW and set SolvedFlag
    int StartFrameIDInKFQue = litIdWisedFeature.StartFrameIDInKFQue;
    gtsam::Point3 Point3_StartFrameIDInKFQue =
        mdKeyFrames[StartFrameIDInKFQue].Pose.inverse() *
        litIdWisedFeature.Point3w;

    litIdWisedFeature.InvDepth = 1. / Point3_StartFrameIDInKFQue(2);

    if (litIdWisedFeature.InvDepth < 0.01) {
      litIdWisedFeature.SolvedFlag = FAILED_SOLVED;
    } else
      litIdWisedFeature.SolvedFlag = OK_SOLVED;
  }

  if (mBackFrIDInKFQue >= gMaxFrameIdInKFQue) SlideWindow();

  mpFeatureManager->RemoveFailures();
}

void FactorGraphEstimator::SlideWindow() {
  if (meMarginFlag == MARGIN_OLD) {
    // for (deque<pair<Frame, FrameWisedFeatures>>::iterator ditprTrackedFrame =
    // mdprTrackedFrames.begin();
    //      ditprTrackedFrame != mdprTrackedFrames.end();
    //      ditprTrackedFrame++)
    // {
    //     if (ditprTrackedFrame->first.FrameId == mdKeyFrames.front().FrameId)
    //     {
    //         mdprTrackedFrames.erase(mdprTrackedFrames.begin(),
    //         ditprTrackedFrame); break;
    //     }
    // }

    int ErsFrmID = mdKeyFrames.front().FrameId;
    mdKeyFrames.pop_front();

    mpFeatureManager->RemoveFront(ErsFrmID);
  } else {
    // auto litTrackedFrame = mdprTrackedFrames.end() - 1;
    // litTrackedFrame--;
    // mdprTrackedFrames.erase(litTrackedFrame);

    auto ditFramesId_PoseInKFQue = mdKeyFrames.end() - 1;
    ditFramesId_PoseInKFQue--;
    int ErsFrmID = ditFramesId_PoseInKFQue->FrameId;
    mdKeyFrames.erase(ditFramesId_PoseInKFQue);

    mpFeatureManager->RemoveBack(ErsFrmID, mBackFrIDInKFQue);
  }
}

void FactorGraphEstimator::SaveNavStates(ofstream &fNavState,
                                         ofstream &fIMUBias) {
  for (int i = 0; i < mNavStateIndex; i++) {
    mPoseb2n = mResultValues.at<gtsam::Pose3>(X(i));  // ???
    mVelb2n = mResultValues.at<gtsam::Vector3>(V(i));
    mOptimizedCurBias = mResultValues.at<gtsam::imuBias::ConstantBias>(B(i));
    // L,L,H
    Vector3d Pos = mPoseb2n.translation();
    // Vn,Ve,Vd
    Vector3d Vel = mVelb2n;
    // roll, pitch, yaw
    Vector3d AttDeg = GTSAMRot2Att(mPoseb2n.rotation());
    // // std
    // Matrix<double, 9, 1> Std = mPkk.block<9, 9>(mIMUPosSIP,
    // mIMUPosSIP).diagonal(); Std = Std.cwiseSqrt();
    fNavState << setprecision(8) << mvNavStatesTimeStamps[i] << setw(15)
              <<  // time
        Pos(0) << setw(15) << Pos(1) << setw(15) << Pos(2) << setw(15)
              <<  // pos
        Vel(0) << setw(15) << Vel(1) << setw(15) << Vel(2) << setw(15)
              <<  // vel
        AttDeg(0) << setw(15) << AttDeg(1) << setw(15) << AttDeg(2) << setw(15)
              <<  // att
        // Std(0) << setw(15) << Std(1) << setw(15) << Std(2) << setw(15) <<
        // Std(3) << setw(15) << Std(4) << // std setw(15) << Std(5) << setw(15)
        // << Std(6) << setw(15) << Std(7) << setw(15) << Std(8)
        endl;

    gtsam::Vector3 AccBias = mOptimizedCurBias.accelerometer();
    gtsam::Vector3 GyroBias = mOptimizedCurBias.gyroscope();
    fIMUBias << setprecision(8) << mvNavStatesTimeStamps[i] << setw(15)
             <<  // time
        AccBias(0) << setw(15) << AccBias(1) << setw(15) << AccBias(2)
             << setw(15) <<  // Acc Bias
        GyroBias(0) << setw(15) << GyroBias(1) << setw(15) << GyroBias(2)
             << setw(15) <<  // Gyro Bias
        endl;
  }
}

// Test
void FactorGraphEstimator::PrintCurFactorGraph(
    gtsam::Values &CurLinearizationPoint) {
  gtsam::NonlinearFactorGraph CurISAMFactorGraph =
      mFLSmootherISAM2.getFactors();
  for (int i = 0; i < CurISAMFactorGraph.size(); i++) {
    gtsam::KeyVector itKeyVector;
    if (CurISAMFactorGraph.at(i))
      itKeyVector = CurISAMFactorGraph.at(i)->keys();
    else
      continue;

    for (int j = 0; j < itKeyVector.size(); j++) {
      gtsam::Symbol itSymbol(itKeyVector[j]);
      if (itSymbol.chr() == 'l') {
        double CurPrjErr =
            CurISAMFactorGraph.at(i)->error(CurLinearizationPoint);
        // CurISAMFactorGraph.at(i)->print();
        cout << i << "  ";
        itSymbol.print();
        cout << CurPrjErr << endl;
      }
    }
  }
}

}  // namespace VIG
