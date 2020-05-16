#pragma once

#include "Configuration.h"
#include "VIGCommon.h"

namespace VIG {

class Configuration;

class GNSS {
 public:
  GNSS(const Configuration *pConfig, ifstream &fGNSS, const double PreIMUTime);
  void GrabNextGNSSData(ifstream &fGNSS);
  void CalGNSSBLH2BodyInNavFrame(const gtsam::Rot3 &CurRotb2n);
  // void CalPoint_NavFrame2imu(const Quaterniond &Qb2n);

 public:
  // string mstrNavFrame;
  // int Q;
  // struct GNSSRawData
  // {
  //     TODO
  // };

  int mGNSSPosType;
  static eGNSSDataFormat meGNSSDataFormat;
  GNSSSolData msGNSSSolData;
};

}  // namespace VIG
