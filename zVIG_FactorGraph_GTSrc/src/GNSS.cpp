#include "GNSS.h"

namespace VIG {

eGNSSDataFormat GNSS::meGNSSDataFormat = BLHdeg;

GNSS::GNSS(const Configuration *pConfig, ifstream &fGNSS,
           const double PreIMUTime) {
  mGNSSPosType = pConfig->mGNSSPosType;

  // 0 for rinex raw,1 for RTKlib solution
  if (mGNSSPosType == 0 && !fGNSS.eof()) {
    // TODO
    return;
  } else if (mGNSSPosType == 1 && !fGNSS.eof()) {
    string strGNSSHeader0, strGNSSHeader, strGNSSLine;
    // int CoordType = 0;   // 0:ecef(m); 1:llh(deg); 2:llh(dd:mm:ss)
    double GNSSweek, GNSStow, B, L, H, Qual, ns, stdn, stde, stdu, stdne, stdeu,
        stdun;

    getline(fGNSS, strGNSSHeader);
    while (strGNSSHeader.find("%") != string::npos && !fGNSS.eof()) {
      strGNSSHeader0 = strGNSSHeader;
      getline(fGNSS, strGNSSHeader);
    }

    strGNSSLine = strGNSSHeader;
    if (!strGNSSLine.empty()) {
      stringstream ss;
      ss << strGNSSLine;
      ss >> GNSSweek;
      ss >> GNSStow;
    }
    while (GNSStow < PreIMUTime && !fGNSS.eof()) {
      getline(fGNSS, strGNSSLine);
      if (!strGNSSLine.empty()) {
        stringstream ss;
        ss << strGNSSLine;
        ss >> GNSSweek;
        ss >> GNSStow;
      }
    }

    stringstream ss;
    ss << strGNSSLine;
    ss >> GNSSweek;
    ss >> GNSStow;
    ss >> B;
    ss >> L;
    ss >> H;
    ss >> Qual;
    ss >> ns;
    ss >> stdn;
    ss >> stde;
    ss >> stdu;
    ss >> stdne;
    ss >> stdeu;
    ss >> stdun;

    msGNSSSolData.Tow = GNSStow;
    msGNSSSolData.BLH << B * kDeg2Rad, L * kDeg2Rad, H;
    msGNSSSolData.Qual = Qual;
    msGNSSSolData.ns = ns;
    msGNSSSolData.Std << stde, stdn, stdu, stdne, stdun, stdeu;

    return;
  }
  msGNSSSolData.Tow = DBL_MAX;
}

void GNSS::GrabNextGNSSData(ifstream &fGNSS) {
  // Read the next GNSS data, 0 for rinex raw,1 for RTKlib solution
  if (mGNSSPosType == 0) {
    // TODO
    return;
  } else if (mGNSSPosType == 1) {
    // output: gpsdata 9x1 GPS TOW, ECEF XYZ, Qual and num of satels and sdx sdy
    // sdz sdxy sdyz sdzx the gpspostype variable is determined based on the
    // first line of data (whether it is GPST or GPS TOW) and last line of the
    // Header to determine the coordinated type (this line contains unique key
    // words ecef or deg) more information about gpspostype in readgpsHeader
    if (!fGNSS.eof()) {
      string strGNSSLine;

      // int CoordType = 0;   // 0:ecef(m); 1:llh(deg); 2:llh(dd:mm:ss)
      double GNSSweek, GNSStow, B, L, H, Qual, ns, stdn, stde, stdu, stdne,
          stdeu, stdun;

      getline(fGNSS, strGNSSLine);
      if (!strGNSSLine.empty()) {
        stringstream ss;
        ss << strGNSSLine;
        ss >> GNSSweek;
        ss >> GNSStow;
        ss >> B;
        ss >> L;
        ss >> H;
        ss >> Qual;
        ss >> ns;
        ss >> stdn;
        ss >> stde;
        ss >> stdu;
        ss >> stdne;
        ss >> stdeu;
        ss >> stdun;

        msGNSSSolData.Tow = GNSStow;
        msGNSSSolData.BLH << B * kDeg2Rad, L * kDeg2Rad, H;
        msGNSSSolData.Qual = Qual;
        msGNSSSolData.ns = ns;
        msGNSSSolData.Std << stde, stdn, stdu, stdne, stdun, stdeu;
      }

      return;
    }
  }
  msGNSSSolData.Tow = DBL_MAX;
}

void GNSS::CalGNSSBLH2BodyInNavFrame(const gtsam::Rot3 &CurRotg2n) {
  Vector3d CurGNSSPoint_NavFrame = CalBLHInNavFrame(msGNSSSolData.BLH);
  gtsam::Pose3 CurGNSSPose_g2n =
      gtsam::Pose3(CurRotg2n, gtsam::Point3(CurGNSSPoint_NavFrame));

  gtsam::Pose3 CurGNSSPose_b2n = CurGNSSPose_g2n * gExtrPose_i2g;
  Vector3d GNSS2BodyPoint_NavFrame = CurGNSSPose_b2n.translation().matrix();

  msGNSSSolData.bodyPoint_NavFrame = GNSS2BodyPoint_NavFrame;
}

}  // namespace VIG
