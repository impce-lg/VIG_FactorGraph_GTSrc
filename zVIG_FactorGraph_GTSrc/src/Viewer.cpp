#include "Viewer.h"

namespace VIG {

Viewer::Viewer(const string &ConfigFilename) {
  std::string config_file = ConfigFilename;
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);

  float fps = fsSettings["Cam.fps"];
  if (fps < 1) fps = 30;
  mT = 1e3 / fps;

  mImageWidth = fsSettings["Cam.nCols"];
  mImageHeight = fsSettings["Cam.nRows"];
  if (mImageWidth < 1 || mImageHeight < 1) {
    cout << "Unknown Camera Size!" << endl;
  }

  mCameraSize = fsSettings["Viewer.CameraSize"];
  mCameraLineWidth = fsSettings["Viewer.CameraLineWidth"];
  mKeyFrameSize = fsSettings["Viewer.KeyFrameSize"];
  mKeyFrameLineWidth = fsSettings["Viewer.KeyFrameLineWidth"];
  mPointSize = fsSettings["Viewer.PointSize"];

  mViewpointX = fsSettings["Viewer.ViewpointX"];
  mViewpointY = fsSettings["Viewer.ViewpointY"];
  mViewpointZ = fsSettings["Viewer.ViewpointZ"];
  mViewpointF = fsSettings["Viewer.ViewpointF"];

  fsSettings["WriteVideo"] >> mbWriteVideo;
  if (mbWriteVideo) {
    string OutPutDir = fsSettings["OutputDirectory"];
    string VideoName = fsSettings["VideoName"];
    string strOutPutFile = OutPutDir + VideoName;

    stringstream s;
    s << "LOCALIZATION | ";
    int baseline = 0;
    cv::Size textSize =
        cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

    mVideoWriter = cv::VideoWriter(
        strOutPutFile,                  //写入视频的目录
        CV_FOURCC('X', 'V', 'I', 'D'),  //-1时弹出选择编码对话框
        fps,                            //帧率
        cvSize(mImageWidth, mImageHeight + textSize.height + 10), 1);
  }
};

void Viewer::InitiViewer(const gtsam::Pose3 &InitFramePose_c2w) {
  mbFinished = false;
  mbStopped = false;
  mNumMPs = 0;

  cv::namedWindow("VIG-FactorGraph: Current Frame");
  pangolin::CreateWindowAndBind("VIG-FactorGraph: Map Viewer", 1024, 768);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(175));
  mmenuFollowCamera = new pangolin::Var<bool>("menu.Follow Camera", true, true);
  mmenuShowFeatures = new pangolin::Var<bool>("menu.Show Features", true, true);
  mmenuShowKeyFrames =
      new pangolin::Var<bool>("menu.Show KeyFrames", true, true);
  mmenuReset = new pangolin::Var<bool>("menu.Reset", false, false);

  // Define Camera Render Object (for view / scene browsing)
  // set a camera, pangolin::ProjectionMatrix: intrinsic parameters
  // pangolin::ProjectionMatrix(w,h,fu,fv,u0,v0,zNear,zFar);
  // pangolin::ModelViewLookAt()
  mS_cam = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389,
                                 0.1, 1000),
      pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0,
                                0.0, -1.0, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  // creat a Viewport
  mD_cam = pangolin::CreateDisplay()
               .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0,
                          -1024.0f / 768.0f)
               .SetHandler(new pangolin::Handler3D(mS_cam));

  GetCurrentOpenGLCameraMatrix(InitFramePose_c2w);
  // mTc2w.SetIdentity();
  mbFollow = true;
}

// Note: CurFramePose_c2w and CurKeyFramesPoses defines the transformation from
// CAMERA to WORLD coordinate frame WORLD Frame: c0 frame,i.e,the initial frame
// of camera frame
void Viewer::RunViewer(
    const FeatureManager *pFeatureManager, const deque<KeyFrame> &dKeyFrames,
    const pair<Frame, FrameWisedFeatures> &prsCurTrackedFrame,
    const cv::Mat &CurImg) {
  mNumKFs = dKeyFrames.size();
  mNumMPs += prsCurTrackedFrame.second.vFeatureIds.size();
  gtsam::Pose3 CurFramePose_c2w = dKeyFrames.back().Pose;

  /**** VIG-FactorGraph: Map Viewer ****/
  // clear color buffer, depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  GetCurrentOpenGLCameraMatrix(CurFramePose_c2w);

  // Follow Camera
  if (mmenuFollowCamera && mbFollow) {
    mS_cam.Follow(mTc2w);
  } else if (mmenuFollowCamera && !mbFollow) {
    mS_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
        mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
    mS_cam.Follow(mTc2w);
    mbFollow = true;
  } else if (!mmenuFollowCamera && mbFollow) {
    mbFollow = false;
  }

  mD_cam.Activate(mS_cam);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  // DrawCurrentCamera();
  // if (mmenuShowKeyFrames)
  //     DrawKeyFrames(mmenuShowKeyFrames, dKeyFrames);
  // if (mmenuShowFeatures)
  //     DrawMapPoints(pFeatureManager,
  //                   prsCurTrackedFrame.second);

  pangolin::FinishFrame();

  /**** VIG-FactorGraph: Frame Viewer ****/
  mCurFrameId = prsCurTrackedFrame.first.FrameId;  //+ to_string()
  cv::Mat im = DrawFrame(prsCurTrackedFrame.second, CurImg);
  cv::imshow("VIG-FactorGraph: Current Frame", im);
  if (mbWriteVideo) mVideoWriter.write(im);
  cv::waitKey(mT);
}

void Viewer::GetCurrentOpenGLCameraMatrix(
    const gtsam::Pose3 &CurFramePose_c2w) {
  Matrix4d eCurFramePose_c2w = CurFramePose_c2w.matrix();
  Matrix3d Rc2w = eCurFramePose_c2w.block<3, 3>(0, 0).transpose();
  Vector3d Tc2w = eCurFramePose_c2w.block<3, 1>(0, 3);

  mTc2w.m[0] = Rc2w(0, 0);
  mTc2w.m[1] = Rc2w(1, 0);
  mTc2w.m[2] = Rc2w(2, 0);
  mTc2w.m[3] = 0.0;

  mTc2w.m[4] = Rc2w(0, 1);
  mTc2w.m[5] = Rc2w(1, 1);
  mTc2w.m[6] = Rc2w(2, 1);
  mTc2w.m[7] = 0.0;

  mTc2w.m[8] = Rc2w(0, 2);
  mTc2w.m[9] = Rc2w(1, 2);
  mTc2w.m[10] = Rc2w(2, 2);
  mTc2w.m[11] = 0.0;

  mTc2w.m[12] = Tc2w(0);
  mTc2w.m[13] = Tc2w(1);
  mTc2w.m[14] = Tc2w(2);
  mTc2w.m[15] = 1.0;
}

void Viewer::DrawCurrentCamera() {
  const float &w = mCameraSize;
  const float h = w * 0.75;
  const float z = w * 0.6;

  glPushMatrix();
  // cout<<"mTc2w.m:"<<endl<<mTc2w<<endl;

#ifdef HAVE_GLES
  glMultMatrixf(mTc2w.m);
#else
  glMultMatrixd(mTc2w.m);
#endif

  glLineWidth(mCameraLineWidth);
  glColor3f(0.0f, 1.0f, 0.0f);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, z);

  glVertex3f(w, h, z);
  glVertex3f(w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(-w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);

  glVertex3f(-w, -h, z);
  glVertex3f(w, -h, z);
  glEnd();

  glPopMatrix();
}

void Viewer::DrawKeyFrames(const bool bDrawKF,
                           const deque<KeyFrame> dKeyFrames) {
  const float &w = mKeyFrameSize;
  const float h = w * 0.75;
  const float z = w * 0.6;

  if (bDrawKF) {
    for (auto &ditFrameInKFQue : dKeyFrames) {
      Matrix<double, 4, 4> Tc2wEgTr = ditFrameInKFQue.Pose.matrix().transpose();
      cv::Mat Tc2w;
      cv::eigen2cv(Tc2wEgTr, Tc2w);
      // cout<<"Tc2w:"<<endl<<Tc2wEg<<endl;

      glPushMatrix();

      glMultMatrixf(Tc2w.ptr<GLfloat>(0));

      glLineWidth(mKeyFrameLineWidth);
      glColor3f(0.0f, 0.0f, 1.0f);
      glBegin(GL_LINES);
      glVertex3f(0, 0, 0);
      glVertex3f(w, h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(w, -h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(-w, -h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(-w, h, z);

      glVertex3f(w, h, z);
      glVertex3f(w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(-w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(w, h, z);

      glVertex3f(-w, -h, z);
      glVertex3f(w, -h, z);
      glEnd();

      glPopMatrix();
    }
  }
}

void Viewer::DrawMapPoints(const FeatureManager *pFeatureManager,
                           const FrameWisedFeatures &CurFrameWisedFeatures) {
  if (CurFrameWisedFeatures.vFeatureIds.empty()) return;

  int NumAllCurFeats = CurFrameWisedFeatures.vFeatureIds.size();
  Vector3d Wpos;  // Undistorted Normalized postion, World position

  glPointSize(mPointSize);
  glBegin(GL_POINTS);

  for (int itFt = 0; itFt < NumAllCurFeats; itFt++) {
    // if (CurFrameWisedFeatures.LostNum[itFt] ||
    // CurFrameWisedFeatures.InvDepth[itFt] < 0)
    //     continue;
    //! 在Features列表中寻找id为feature_id的Feature
    int FeatureId = CurFrameWisedFeatures.vFeatureIds[itFt];
    auto litIdWisedFeature =
        find_if(pFeatureManager->mlsIdWisedFeatures.begin(),
                pFeatureManager->mlsIdWisedFeatures.end(),
                [FeatureId](const IdWisedFeature &litIdWisedFeature) {
                  return litIdWisedFeature.FeatureId == FeatureId;
                });

    //! 如果该Feature在mlsIdWisedFeatures列表中
    if (litIdWisedFeature->FeatureId == FeatureId) {
      Wpos = litIdWisedFeature->Point3w;
      // if (CurFrameWisedFeatures.InvDepth[itFt] < 3 *
      // sqrt(pFactorGraphEstimator->mPkk(FeatureSIP + itFt, FeatureSIP +
      // itFt)))
      //     glColor3f(0.0, 1.0, 0.0);
      // else
      glColor3f(1.0, 0.0, 0.0);
    } else
      continue;

    // cout<<"Wpos:"<<endl<<Wpos<<endl;
    glVertex3f(Wpos(0), Wpos(1), Wpos(2));
  }
  glEnd();
}

cv::Mat Viewer::DrawFrame(const FrameWisedFeatures &CurFrameWisedFeatures,
                          const cv::Mat &CurImg) {
  cv::Mat im;
  int state = 1;  // Tracking state

  CurImg.copyTo(im);
  if (im.channels() < 3)  // this should be always true
    cvtColor(im, im, CV_GRAY2BGR);

  // Draw
  mNumOptimized = 0;
  mNumTracked = 0;
  mNumRejected = 0;
  const float r = 5;
  const int NumAllCurFeats = CurFrameWisedFeatures.vFeatureIds.size();
  for (int itFt = 0; itFt < NumAllCurFeats; itFt++) {
    // if (!CurFrameWisedFeatures.vUVs[itFt].isZero() /*&&
    //     !CurFrameWisedFeatures.SPkk[itFt].isZero()*/)
    // {
    cv::Point2f FeatMeas, pt1, pt2;
    FeatMeas.x = CurFrameWisedFeatures.vUVs[itFt](0);
    FeatMeas.y = CurFrameWisedFeatures.vUVs[itFt](1);
    pt1.x = CurFrameWisedFeatures.vUVs[itFt](0) - r;
    pt1.y = CurFrameWisedFeatures.vUVs[itFt](1) - r;
    pt2.x = CurFrameWisedFeatures.vUVs[itFt](0) + r;
    pt2.y = CurFrameWisedFeatures.vUVs[itFt](1) + r;

    if (CurFrameWisedFeatures.vbOptimized[itFt]) {
      cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
      cv::circle(im, FeatMeas, 2, cv::Scalar(0, 255, 0), -1);
      mNumOptimized++;
    }
    // else if (CurFrameWisedFeatures.HighInnoInlier[itFt])
    // {
    //     cv::rectangle(im, pt1, pt2, cv::Scalar(255, 255, 0));
    //     cv::circle(im, FeatMeas, 2, cv::Scalar(255, 255, 0), -1);
    //     mNumOptimized++;
    // }
    // else if (!CurFrameWisedFeatures.MeasureHomoCoord[itFt].isZero())
    // {
    //     cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
    //     cv::circle(im, FeatMeas, 2, cv::Scalar(255, 0, 0), -1);
    //     mNumTracked++;
    // }
    else {
      cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 255));
      cv::circle(im, FeatMeas, 2, cv::Scalar(255, 0, 255), -1);
      mNumRejected++;
    }
    // }
  }

  cv::Mat imWithInfo;
  DrawTextInfo(im, state, imWithInfo);

  return imWithInfo;
}

void Viewer::DrawTextInfo(const cv::Mat &im, const int nState,
                          cv::Mat &imText) {
  stringstream s;
  if (nState == 1) {
    s << "LOCALIZATION | ";

    s << "CurFrameId: " << mCurFrameId << ", MPs: " << mNumMPs
      << ", Optimized: " << mNumOptimized;
    if (mNumTracked > 0) s << ", + Tracked: " << mNumTracked;
    if (mNumRejected > 0) s << ", + Rejected: " << mNumRejected;
  } else {
    s << " TRACK LOST. TRYING TO RELOCALIZE ";
  }

  int baseline = 0;
  cv::Size textSize =
      cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

  imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
  im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
  imText.rowRange(im.rows, imText.rows) =
      cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
  cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
}

};  // namespace VIG
