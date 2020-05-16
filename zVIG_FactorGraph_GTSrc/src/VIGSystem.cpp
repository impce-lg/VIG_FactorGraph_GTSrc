#include "VIGSystem.h"

namespace VIG {

VIGSystem::VIGSystem(const string &ConfigFilename) {
  /**** Load Configuration config file ****/
  mstrConfigFile = ConfigFilename;
  mpConfiguration = new Configuration(mstrConfigFile);
  mpConfiguration->SetGlobalParas();

  mofNavState.open(mpConfiguration->mstrNavfile);
  mofNavState << fixed;
  mofIMUBias.open(mpConfiguration->mstrIMUBiasfile);
  mofIMUBias << fixed;

  /**** Initialize IMU ****/
  mifIMU.open(mpConfiguration->mstrIMUFile, ios_base::in);
  mpIMU = new IMU(mpConfiguration, mifIMU);
  mInitiTime = mpIMU->msPreIMUData.T;

  /**** Initialize GNSS files ****/
  if (gbUseGNSS) {
    mifGNSS.open(mpConfiguration->mstrGNSSFile, ios_base::in);
    mpGNSS = new GNSS(mpConfiguration, mifGNSS, mpIMU->msPreIMUData.T);
    mUsedGNSSCounter = 1;
  } else
    mpGNSS->msGNSSSolData.Tow = DBL_MAX;

  /**** Initialize mpFeatureTracker and mpFeatureManager ****/
  if (gbUseCam) {
    mifCam = mpConfiguration->mstrCamPath;
    mifCamTime.open(mpConfiguration->mstrCamTimeFile, ios_base::in);

    mofFeatureTable.open(mpConfiguration->mstrFeatureTableFile);
    mofFeatureTable << fixed;
    // mofCamPose.open(mpConfiguration->mstrCamPoseFile);
    // mofCamPose << fixed;

    mpFeatureTracker =
        new FeatureTracker(mpConfiguration, mifCamTime, mpIMU->msPreIMUData.T);

    meVIGEstimatorState = NOT_INITIALIZED;
    mpVIGInitializer = new VIGInitializer(mpConfiguration, mpIMU);
    mpFactorGraphEstimator = mpVIGInitializer;
  } else
    /**** Initialize FactorGraphEstimator ****/
    mpFactorGraphEstimator = new FactorGraphEstimator(mpConfiguration);
  mpFactorGraphEstimator->UpdateNavStateTimeStamps_KeyTimeStampMap(
      mpIMU->msPreIMUData.T);

  // Initialize Viewer
  if (gbUseCam && mpConfiguration->mbUseViewer) {
    mpViewer = new Viewer(mstrConfigFile);
    mpViewer->InitiViewer(mpVIGInitializer->mPoseb2n);
  }

  // Test
  // gTestFramesThreshold = mpConfiguration->mTestFramesThreshold;
  if (!gbUseCam && mpConfiguration->mbVSlam) {
    mpTestVSlamSystem = new TestVSlamSystem(
        mpConfiguration, mpIMU->msPreIMUData.T, mstrConfigFile);
  } else if (gbUseCam && mpConfiguration->mbVSlam) {
    cerr << "Don't use VIG and VSlam(testable) simultaneously,\
            please check your configuration ";
    exit(1);
  }
}

bool VIGSystem::IsAvailable() {
  if ((mpIMU->msCurIMUData.T - mInitiTime) > 60) {
    cout << "Process Epoch:" << mInitiTime << endl << endl;
    mInitiTime = mpIMU->msCurIMUData.T;
  }

  return !mifIMU.eof() && mpIMU->msCurIMUData.T < mpConfiguration->mIMUEndTime;
}

void VIGSystem::PropagateIMU() { mpFactorGraphEstimator->PreIntegrate(mpIMU); }

void VIGSystem::CoupleGNSS() {
  if (!mbHasIMUAdded) {
    mpFactorGraphEstimator->AddIMUFactor();
    mbHasIMUAdded = true;
  }

  mpGNSS->CalGNSSBLH2BodyInNavFrame(
      mpFactorGraphEstimator->GetCurPredPose().rotation());
  mpFactorGraphEstimator->AddGNSSSolFactor(mpGNSS);

  if (gbUseCam && meVIGEstimatorState == NOT_INITIALIZED) {
    mpVIGInitializer->AddGNSS(*mpGNSS);
  }
  mpGNSS->GrabNextGNSSData(mifGNSS);
}

void VIGSystem::CoupleVision() {
  // there are some gap in the imu recordings
  if (fabs(mpIMU->msCurIMUData.T - mpFeatureTracker->mFrameEpoch) >
      mpConfiguration->mIMUDt)
    cout << "IMU gap at:" << mpIMU->msCurIMUData.T
         << ",FramID:" << mpFeatureTracker->mImageID << endl;

  // 1. get the image of index frmId 0 based
  mpFeatureTracker->ReadCurrentImage(mifCam);

  // 2. Get current frame pose
  gtsam::Pose3 CurPose_cam2ref =
      mpFactorGraphEstimator->GetCurPredPose() * gExtrPose_c2i;

  // 3. Track the adjacency frame
  mpFeatureTracker->TrackFeatures(CurPose_cam2ref);

  // 4. Add TrackedFrames,FramesId_PosesInKFQue, IdWised Features
  mpFactorGraphEstimator->AddNewVision(
      mpFeatureTracker->GetTrackMoreFeatures());

  // Initialize or Add Factors
  if (meVIGEstimatorState == NOT_INITIALIZED) {
    if (static_cast<int>(mpFactorGraphEstimator->mdKeyFrames.size()) ==
        mpFactorGraphEstimator->mInitWinSize) {
      // 5. Initialize
      bool bInitializationState = false;
      bInitializationState = mpVIGInitializer->InitializeVIGStructure_Loosely();

      if (bInitializationState) {
        meVIGEstimatorState = VIG_OK;
        // mbIsVisualAvailable = true;

        // mpFactorGraphEstimator = mpVIGInitializer;
        // delete mpVIGInitializer;
        // mpVIGInitializer = NULL;
      } else
        mpFactorGraphEstimator->SlideWindow();
    }
    // else
    //     mpFactorGraphEstimator->mBackFrameIDInKFQue++;
  } else {
    // 6. Triangulate the Features in navigation Frame which are not
    // triangulated yet
    mpFactorGraphEstimator->TriangulateCurFrameFeatures();

    // 7. Add Factors
    if (!mbHasIMUAdded) {
      mpFactorGraphEstimator->AddIMUFactor();
      mbHasIMUAdded = true;
    }
    mpFactorGraphEstimator->AddVisualFactor();
    mbIsVisualAvailable = true;
  }

  // // Save File
  // mpFactorGraphEstimator->SaveCamPoseandPoses02e(mpFeatureTracker->mFrameEpoch,
  // mpFeatureTracker->mImageID, mofCamPose);

  // grab the next image time
  mpFeatureTracker->GrabNextImageEpoch(mifCamTime);
}

void VIGSystem::CoupleOtherSensors() {
  mbHasIMUAdded = false;

  // Add GNSS Factor
  if (gbUseGNSS && mpIMU->msCurIMUData.T >=
                       mpGNSS->msGNSSSolData.Tow)  // curimutime > gpstime
    CoupleGNSS();

  // Add Visual Factors
  mbIsVisualAvailable = false;
  if (gbUseCam && mpIMU->msCurIMUData.T >= mpFeatureTracker->mFrameEpoch) {
    if (gbFirstFrame) {  // the first frame
      gFirstPubFrameEpoch = mpFeatureTracker->mInitiFrameEpoch;
      gbFirstFrame = false;
      mpFeatureTracker->GrabNextImageEpoch(mifCamTime);
      return;
    }

    CoupleVision();
  }

  // Optimize
  if (mbHasIMUAdded) {
    mpFactorGraphEstimator->UpdateNavStateTimeStamps_KeyTimeStampMap(
        mpIMU->msCurIMUData.T);
    mpFactorGraphEstimator->InsertNewNavStateInitialValues();
    mpFactorGraphEstimator->Optimize();
  }

  if (mbIsVisualAvailable) {
    // 1. Update Poses in KeyFrames Queue of Estimator
    mpFactorGraphEstimator->UpdateKeyFramesAndFeatures();

    // 2. Viewer: TODO
    mpViewer->RunViewer(mpFactorGraphEstimator->mpFeatureManager,
                        mpFactorGraphEstimator->mdKeyFrames,
                        mpFactorGraphEstimator->mprsCurTrackedFrame,
                        mpFeatureTracker->mCurImg);

    // // Test
    // cout << mpViewer->NumOptimized() << endl;
  }

  // Test
  if (!gbUseCam && mpConfiguration->mbVSlam) {
    mpTestVSlamSystem->RunVSlam(mpConfiguration, mpIMU, mpFactorGraphEstimator);
  }
}

void VIGSystem::StepIMU() {
  mpIMU->GrabNextIMUData(mifIMU);

  if (gbUseCam && meVIGEstimatorState == NOT_INITIALIZED)
    mpVIGInitializer->AddIMU(*mpIMU);
}

void VIGSystem::OutputNavStates() {
  mpFactorGraphEstimator->SaveNavStates(mofNavState, mofIMUBias);

  // if (mpIMU->msCurIMUData.T >= 47005.34460718)
  // {
  //     cout << "error the next epoch:" << endl;
  // }
}

void VIGSystem::Exit() {
  mifIMU.close();
  mifGNSS.close();

  mofNavState.close();
  mofIMUBias.close();
  cout << endl << "Finihing..." << endl;
}

}  // namespace VIG
