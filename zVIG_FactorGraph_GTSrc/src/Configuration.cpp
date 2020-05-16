#include "Configuration.h"

namespace VIG {

Configuration::Configuration(const string &ConfigFilename) {
  string config_file = ConfigFilename;
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "ERROR: Wrong path to settings" << endl;
    exit(-1);
  }

  // input file names
  fsSettings["InputDirectory"] >> mstrInputDirectory;
  fsSettings["IMUfile"] >> mstrIMUFile;
  mstrIMUFile = mstrInputDirectory + mstrIMUFile;
  fsSettings["GNSSfile"] >> mstrGNSSFile;
  mstrGNSSFile = mstrInputDirectory + mstrGNSSFile;
  fsSettings["CamPath"] >> mstrCamPath;
  mstrCamPath = mstrInputDirectory + mstrCamPath;
  fsSettings["CamTimefile"] >> mstrCamTimeFile;
  mstrCamTimeFile = mstrInputDirectory + mstrCamTimeFile;
  // output file names
  fsSettings["OutputDirectory"] >> mstrOutputDirectory;
  fsSettings["Navfile"] >> mstrNavfile;
  mstrNavfile = mstrOutputDirectory + mstrNavfile;
  fsSettings["IMUBiasfile"] >> mstrIMUBiasfile;
  mstrIMUBiasfile = mstrOutputDirectory + mstrIMUBiasfile;
  fsSettings["KMLFilename"] >> mstrKMLFilename;
  mstrKMLFilename = mstrOutputDirectory + mstrKMLFilename;
  fsSettings["FeatureTableFile"] >> mstrFeatureTableFile;
  mstrFeatureTableFile = mstrOutputDirectory + mstrFeatureTableFile;
  fsSettings["CamPoseFile"] >> mstrCamPoseFile;
  mstrCamPoseFile = mstrOutputDirectory + mstrCamPoseFile;
  fsSettings["VideoName"] >> mstrVideoName;
  mstrVideoName = mstrOutputDirectory + mstrVideoName;
  fsSettings["NavFrame"] >> mstrNavFrame;
  assert(mstrNavFrame == "NED" || mstrNavFrame == "ENU");

  // IMU Configuration
  fsSettings["UseIMU"] >> mbUseIMU;
  mIMUFileType = fsSettings["IMUFileType"];
  mIMUMisalignMode = fsSettings["IMUMisalignMode"];
  mIMUType = fsSettings["IMUType"];
  mIMUStartTime = fsSettings["IMUStartTime"];
  mIMUEndTime = fsSettings["IMUEndTime"];
  mIMUDt = fsSettings["IMUDt"];

  // Extrinsic parameter between IMU and body.
  cv::Mat Camera2IMU, TIMU2Camera;
  Vector3d RotAngel;
  fsSettings["Camera2IMU"] >> Camera2IMU;
  cv::cv2eigen(Camera2IMU, RotAngel);
  mCamera2IMU = R3(RotAngel(2) * kDeg2Rad) * R2(RotAngel(1) * kDeg2Rad) *
                R1(RotAngel(0) * kDeg2Rad);
  // Vector3d Attc2i=GTSAMRot2Att(gtsam::Rot3(mCamera2IMU));
  // cout << "Rc2i" << endl
  //      << mCamera2IMU << endl;
  fsSettings["TIMU2Camera"] >> TIMU2Camera;
  cv::cv2eigen(TIMU2Camera, mTIMU2Camera);
  mExtrPose_c2i =
      gtsam::Pose3(gtsam::Rot3(mCamera2IMU), gtsam::Point3(mTIMU2Camera));

  cv::Mat IMUErrorStates;
  Eigen::Matrix<double, 4, 3> mtIMUErrorStates;
  fsSettings["IMUErrorStates"] >> IMUErrorStates;
  cv::cv2eigen(IMUErrorStates, mtIMUErrorStates);
  mInitiIMUErrorStates.GyroBias = mtIMUErrorStates.row(0);
  mInitiIMUErrorStates.AccBias = mtIMUErrorStates.row(1);
  mInitiIMUErrorStates.GyroScale = mtIMUErrorStates.row(2);
  mInitiIMUErrorStates.AccScale = mtIMUErrorStates.row(3);

  // initial navigation info
  cv::Mat IniBLH;
  fsSettings["IniBLH"] >> IniBLH;
  cv::cv2eigen(IniBLH, mIniBLH);
  mIniBLH.head(2) = mIniBLH.head(2) * kDeg2Rad;
  cv::Mat IniVn;
  fsSettings["IniVn"] >> IniVn;
  cv::cv2eigen(IniVn, mIniVn);

  cv::Mat IniAtti2n;
  fsSettings["IniAtti2n"] >> IniAtti2n;
  Eigen::Vector3d egIniRoti2n;
  cv::cv2eigen(IniAtti2n * kDeg2Rad, egIniRoti2n);
  Eigen::Quaterniond Qi2n = AngleAxisd(egIniRoti2n(2), Vector3d::UnitZ()) *
                            AngleAxisd(egIniRoti2n(1), Vector3d::UnitY()) *
                            AngleAxisd(egIniRoti2n(0), Vector3d::UnitX());
  mIniQi2n = Qi2n.normalized();
  // Quaterniond Q2(R3(-egIniRoti2n(2)) * R2(-egIniRoti2n(1)) *
  // R1(-egIniRoti2n(0))); Quaterniond Qmult=Q*Q2; cout << "Q1: " << Q << endl;
  // cout << "Q2: " << Q2 << endl;

  mInitAttSigma = fsSettings["InitAttSigma"];
  mInitAttSigma *= kDeg2Rad;
  mInitPosSigma = fsSettings["InitPosSigma"];
  mInitVelSigma = fsSettings["InitVelSigma"];

  mInitAccBiasSigma = fsSettings["InitAccBiasSigma"];
  mInitGyroBiasSigma = fsSettings["InitGyroBiasSigma"];
  mInitAccBetwBiasSigma = fsSettings["InitAccBetwBiasSigma"];
  mInitGyroBetwBiasSigma = fsSettings["InitGyroBetwBiasSigma"];

  mAccSigma = fsSettings["AccSigma"];
  mGyroSigma = fsSettings["GyroSigma"];
  mIntgSigma = fsSettings["IntgSigma"];

  cv::Mat g, w_coriolis;
  fsSettings["g"] >> g;
  cv::cv2eigen(g, mG);
  fsSettings["w_coriolis"] >> w_coriolis;
  cv::cv2eigen(w_coriolis, mw_coriolis);

  // GNSS Configuration
  fsSettings["UseGNSS"] >> mbUseGNSS;
  fsSettings["UseGNSSStd"] >> mbUseGNSSStd;
  cv::Mat Timu2ant;
  fsSettings["Timu2ant"] >> Timu2ant;
  cv::cv2eigen(Timu2ant, mTimu2ant);
  mGNSSNum = fsSettings["GNSSNum"];
  mGNSSPosType = fsSettings["GNSSPosType"];
  mGNSSAttSigma = fsSettings["GNSSAttSigma"];
  mGNSSPosSigma = fsSettings["GNSSPosSigma"];
  cv::Mat InitGNSSResidulThreshold;
  fsSettings["InitGNSSResidulThreshold"] >> InitGNSSResidulThreshold;
  cv::cv2eigen(InitGNSSResidulThreshold, mGNSSResidulThreshold);
  fsSettings["GNSSResidulThresholdFactor"] >> mvGNSSResidulThresholdFactor;

  // Camera Configuration
  fsSettings["UseCam"] >> mbUseCam;
  float fps = fsSettings["Cam.fps"];
  mInitWindowSize = fsSettings["InitWindowSize"];  // (int)fps + 1;
  mKeyFramesQueueSize = fsSettings["KeyFramesQueueSize"];
  fsSettings["Equalize"] >> mbEqualize;
  fsSettings["FishEye"] >> mbFishEye;
  if (mbFishEye) fsSettings["FishEyeMask"] >> mstrFishEyeMask;
  fsSettings["PubFrequency"] >> mPubFrequency;
  mNRow = fsSettings["Cam.nRows"];
  mNCol = fsSettings["Cam.nCols"];

  double fx = fsSettings["Cam.fcx"];
  double fy = fsSettings["Cam.fcy"];
  double cx = fsSettings["Cam.ccx"];
  double cy = fsSettings["Cam.ccy"];
  double alpha = fsSettings["Cam.alpha"];
  mCamK.setIdentity();
  mCamK(0, 0) = fx;
  mCamK(0, 1) = alpha * fx;
  mCamK(0, 2) = cx;
  mCamK(1, 1) = fy;
  mCamK(1, 2) = cy;

  cv::Mat CamDistCoef;
  fsSettings["Cam.CamDistCoef"] >> CamDistCoef;
  cv::cv2eigen(CamDistCoef, mCamDistCoef);

  mSigma_Pixel = fsSettings["Sigma_Pixel"];
  cv::Mat InitPixelResidulThreshold;
  fsSettings["InitPixelResidulThreshold"] >> InitPixelResidulThreshold;
  cv::cv2eigen(InitPixelResidulThreshold, mPixelResidulThreshold);
  fsSettings["PixelResidulThresholdFactor"] >> mvPixelResidulThresholdFactor;
  mImageSeqType = fsSettings["ImageSeqType"];
  mInitiImageID = fsSettings["InitiImageID"];
  mLastImageID = fsSettings["LastImageID"];

  // Feature Tracker Configuration
  mMaxFeatureNumPerFrame = fsSettings["MaxFeatureNumPerFrame"];
  mMinFeatureNumInStates = fsSettings["MinFeatureNumInStates"];
  mFundamentalThreshold = fsSettings["FundamentalThreshold"];
  mNumUnDisIter = fsSettings["NumUnDisIter"];

  // Feature Manager Configuration
  mFocalLength = fsSettings["FocalLength"];
  mMinKeyFrameParallax = fsSettings["MinKeyFrameParallax"];
  mCoVisualFramesThreshold = fsSettings["CoVisualFramesThreshold"];

  // Optimization Configuration
  fsSettings["OptimizationParams"] >> mstrOptimizationParams;
  fsSettings["Optimizer"] >> mstrOptimizer;
  assert(mstrOptimizer == "IncrementalFixedLagSmoother" ||
         mstrOptimizer == "ISAM2");
  mIsamParams_relinearizeSkip = fsSettings["IsamParams.relinearizeSkip"];
  fsSettings["IsamParams.findUnusedFactorSlots"] >>
      mbIsamParams_findUnusedFactorSlots;
  fsSettings["IsamParams.evaluateNonlinearError"] >>
      mbIsamParams_evaluateNonlinearError;
  mNonLinearISAMRelinearizeInterval =
      fsSettings["NonLinearISAMRelinearizeInterval"];
  mSmootherLag = fsSettings["SmootherLag"];
  mNumFLSmootherUpdIters = fsSettings["NumFLSmootherUpdIters"];

  // Viewer Configuration
  fsSettings["UseViewer"] >> mbUseViewer;

  // Test params
  fsSettings["VSlam"] >> mbVSlam;
  mTestPauseEpoch = fsSettings["TestPauseEpoch"];
  mTestFramesThreshold = fsSettings["TestFramesThreshold"];
}

void Configuration::SetGlobalParas() {
  gstrNavFrame = mstrNavFrame;
  gG = mG;

  // Base
  gIMUBaseBLH = mIniBLH;

  // GNSS
  gbUseGNSS = mbUseGNSS;
  // Note that this point should be IMU in GNSS body,
  // It's defined they share the same body, i.e., IMU frame,
  // which means the rotation between IMU and GNSS is always identity
  gExtrPose_i2g =
      gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(-mTimu2ant));

  // Camera
  gbUseCam = mbUseCam;
  gFocalLength = mFocalLength;
  gCamK = mCamK;
  gCamDistCoef = mCamDistCoef;
  gNRow = mNRow;
  gNCol = mNCol;

  gKeyFramesQueueSize = mKeyFramesQueueSize;
  gMaxFrameIdInKFQue = gKeyFramesQueueSize - 1;

  gCoVisualFramesThreshold = mCoVisualFramesThreshold;

  gExtrPose_c2i = mExtrPose_c2i;
}

}  // namespace VIG
