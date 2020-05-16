/*
 * @file pppBayesTree.cpp
 * @brief Iterative GPS Range/Phase Estimator with collected vsRNXData
 * @author Ryan Watson & Jason Gross
 */

// GTSAM related includes.
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/gnssNavigation/GnssData.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/gnssNavigation/PhaseFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/configReader/ConfDataReader.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/gnssNavigation/PseudorangeFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// GPSTK
#include <gtsam/gpstk/MJD.hpp>
#include <gtsam/gpstk/PowerSum.hpp>
#include <gtsam/gpstk/Decimate.hpp>
#include <gtsam/gpstk/SolidTides.hpp>
#include <gtsam/gpstk/PoleTides.hpp>
#include <gtsam/gpstk/TropModel.hpp>
#include <gtsam/gpstk/BasicModel.hpp>
#include <gtsam/gpstk/CommonTime.hpp>
#include <gtsam/gpstk/PCSmoother.hpp>
#include <gtsam/gpstk/OceanLoading.hpp>
#include <gtsam/gpstk/CodeSmoother.hpp>
#include <gtsam/gpstk/SimpleFilter.hpp>
#include <gtsam/gpstk/MWCSDetector.hpp>
#include <gtsam/gpstk/SatArcMarker.hpp>
#include <gtsam/gpstk/DCBDataReader.hpp>
#include <gtsam/gpstk/ComputeWindUp.hpp>
#include <gtsam/gpstk/Rinex3NavData.hpp>
#include <gtsam/gpstk/GNSSconstants.hpp>
#include <gtsam/gpstk/ComputeLinear.hpp>
#include <gtsam/gpstk/GPSWeekSecond.hpp>
#include <gtsam/gpstk/LICSDetector2.hpp>
#include <gtsam/gpstk/DataStructures.hpp>
#include <gtsam/gpstk/RinexObsStream.hpp>
#include <gtsam/gpstk/Rinex3ObsStream.hpp>
#include <gtsam/gpstk/Rinex3NavStream.hpp>
#include <gtsam/gpstk/ComputeTropModel.hpp>
#include <gtsam/gpstk/SP3EphemerisStore.hpp>
#include <gtsam/gpstk/ComputeSatPCenter.hpp>
#include <gtsam/gpstk/EclipsedSatFilter.hpp>
#include <gtsam/gpstk/GPSEphemerisStore.hpp>
#include <gtsam/gpstk/CorrectCodeBiases.hpp>
#include <gtsam/gpstk/ComputeSatPCenter.hpp>
#include <gtsam/gpstk/RequireObservables.hpp>
#include <gtsam/gpstk/CorrectObservables.hpp>
#include <gtsam/gpstk/LinearCombinations.hpp>
#include <gtsam/gpstk/GravitationalDelay.hpp>
#include <gtsam/gpstk/PhaseCodeAlignment.hpp>

// BOOST
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/serialization/export.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

// STD
#include <chrono>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <algorithm>

using namespace std;
using namespace gtsam;
using namespace gpstk;
using namespace boost;
using namespace std::chrono;
namespace NM = gtsam::noiseModel;
namespace po = boost::program_options;
typedef noiseModel::Diagonal diagNoise;

// Intel Threading Building Block
#ifdef GTSAM_USE_TBB
#include <tbb/tbb.h>
#undef max // TBB seems to include windows.h and we don't want these macros
#undef min
#endif

using symbol_shorthand::G; // bias states ( Phase Biases )
using symbol_shorthand::X; // nonBiasStates ( dx, dy, dz, trop, cb )

int main(int argc, char *argv[])
{
        /*********** I. Configuration ***********/
        // define std out print color
        vector<int> vPRNVec;
        vector<rnxData> vsRNXData;
        const string strRed("\033[0;31m");
        const string strGreen("\033[0;32m");
        string strConfFile; 
        string strFilePath, strGNSSFile, strStation, strsp3File, strP1P2File, strP1C1File, strAntennaModel;
        string rnx_file, nav_file, sp3_file, strOutFile, antexFile;
        double Xn, Yn, Zn, xp, yp, Range, Phase, rho, MinElev, WeightFactor;
        double antennaOffSetH, antennaOffSetE, antennaOffSetN;
        int StartKey(0), CurrKey, startEpoch(0), SVN, Doy;
        int NThreads(-1), PhaseBreak, break_count(0), nextKey, dec_int, ItsBelowThree = 0, EpochCount = 0;
        bool bPrintECEF, bPrintENU, bPrintAmb, bPrintUpdateRate, bFirstOb(true), bUsingP1(false);

        cout.precision(12);

        po::options_description desc("Available options");
        desc.add_options()("help,h", "Print help message")("strConfFile,c", po::value<string>(&strConfFile)->default_value(""),
                                                           "Input config file")("out", po::value<string>(&strOutFile)->default_value(""),
                                                                                "output file.")("bUsingP1", "Are you using P1 instead of C1?");
        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        po::notify(vm);

        ConfDataReader clConfReader;
        clConfReader.open(strConfFile);

        if (strConfFile.empty())
        {
                cout << strRed << "\n\n Currently, you need to provide a conf file \n"
                     << "\n\n"
                     << strGreen << desc << endl;
        }

        while ((strStation = clConfReader.getEachSection()) != "")
        {
                // Fetch nominal strStation location [m]
                Xn = clConfReader.fetchListValueAsDouble("nominalECEF", strStation);
                Yn = clConfReader.fetchListValueAsDouble("nominalECEF", strStation);
                Zn = clConfReader.fetchListValueAsDouble("nominalECEF", strStation);
                // day of year ( used for Niell Trop model)
                Doy = clConfReader.getValueAsInt("DOY", strStation);
                // Elevation cut-off
                MinElev = clConfReader.getValueAsDouble("minElev", strStation);
                // Code/carrier ratio
                WeightFactor = clConfReader.getValueAsDouble("weightFactor", strStation);
                // Data file path
                strFilePath = clConfReader("FilePath", strStation);
                // Data file names
                strGNSSFile = clConfReader("rnxFile", strStation);
                strsp3File = clConfReader("sp3File", strStation);
                strP1P2File = clConfReader("p1p2", strStation);
                strP1C1File = clConfReader("p1c1", strStation);
                // Print statements
                bPrintENU = clConfReader.getValueAsBoolean("printENU", strStation);
                bPrintAmb = clConfReader.getValueAsBoolean("printAmb", strStation);
                bPrintECEF = clConfReader.getValueAsBoolean("printECEF", strStation);
                bPrintUpdateRate = clConfReader.getValueAsBoolean("printUpdateRate", strStation);
        }

        bUsingP1 = (vm.count("usingP1") > 0);

        Point3 NominalXYZ(Xn, Yn, Zn);
        Point3 PropXYZ = NominalXYZ;

        /*********** II. GTSAM Settings ***********/
#ifdef GTSAM_USE_TBB
        std::auto_ptr<tbb::task_scheduler_init> init;
        if (NThreads > 0)
        {
                init.reset(new tbb::task_scheduler_init(NThreads));
        }
        else
                cout << strGreen << " \n\n Using threads for all processors" << endl;
#else
        if (NThreads > 0)
        {
                cout << strRed << " \n\n GTSAM is not compiled with TBB, so threading is"
                     << " disabled and the --threads option cannot be used."
                     << endl;
                exit(1);
        }
#endif

        ISAM2DoglegParams doglegParams;
        ISAM2Params ISAM2Parameters;
        ISAM2Parameters.relinearizeThreshold = 0.1;
        ISAM2Parameters.relinearizeSkip = 100;
        ISAM2 Isam2(ISAM2Parameters);

        double OutputTime = 0.0;
        double RW = 2.5;
        double RangeWeight = pow(RW, 2);
        double PhaseWeight = pow(RW * 1 / WeightFactor, 2);

        string value;

        nonBiasStates clPriorNonBias = (gtsam::Vector(5) << 0.0, 0.0, 0.0, 0.0, 0.0).finished();
        phaseBias tdBiasState(Z_1x1);
        gnssStateVector tdPhaseArc(Z_34x1);
        gnssStateVector tdBiasCounter(Z_34x1);
        for (int i = 1; i < 34; i++)
        {
                tdBiasCounter(i) = tdBiasCounter(i - 1) + 10000;
        }

        nonBiasStates clInitEst(Z_5x1);
        nonBiasStates clBetweenNonBiasState(Z_5x1);

        Values NewInitialValues;
        Values Result;
        noiseModel::Diagonal::shared_ptr NonBias_InitNoise = noiseModel::Diagonal::Variances((gtsam::Vector(5) << 10.0, 10.0, 10.0, 3e8, 1e-1).finished());
        noiseModel::Diagonal::shared_ptr NonBias_ProcessNoise = noiseModel::Diagonal::Variances((gtsam::Vector(5) << 0.1, 0.1, 0.1, 3e6, 3e-5).finished());
        noiseModel::Diagonal::shared_ptr NnitNoise = noiseModel::Diagonal::Variances((gtsam::Vector(1) << 100).finished());

        /*********** III. Files Settings ***********/
        string strObsPath = strFilePath + strGNSSFile;
        string strP1P2Path = strFilePath + strP1P2File;
        string strP1C1Path = strFilePath + strP1C1File;
        if (strObsPath.empty())
        {
                cout << " Must pass in obs file !!! " << desc << endl;
                exit(1);
        }
        if (strsp3File.empty())
        {
                cout << " Must pass in ephemeris file !!! " << desc << endl;
                exit(1);
        }
        // Declare a "SP3EphemerisStore" object to handle precise ephemeris
        SP3EphemerisStore clSP3EphList;

        size_t pos = 0;
        string strsp3Path, strToken;
        string strDelimiter = " ";
        while ((pos = strsp3File.find(strDelimiter)) != string::npos)
        {
                strsp3Path = strFilePath + strsp3File.substr(0, pos);
                clSP3EphList.loadFile(strsp3Path);
                strsp3File.erase(0, pos + strDelimiter.length());
        }
        strsp3Path = strFilePath + strsp3File;
        clSP3EphList.loadFile(strsp3Path);

        // Set flags to reject satellites with bad or absent positional
        // values or clocks
        clSP3EphList.rejectBadPositions(true);
        clSP3EphList.rejectBadClocks(true);

        // Create the input observation file stream
        Rinex3ObsStream clRinex3ObsStream(strObsPath);

        // Station nominal position
        Position clNominalPos(Xn, Yn, Zn);

        CorrectCodeBiases clCorrCode;
        clCorrCode.setDCBFile(strP1P2Path, strP1C1Path);

        if (!bUsingP1)
        {
                clCorrCode.setUsingC1(true);
        }

        /*********** IV. RINEX Settings ***********/
        // This is the GNSS vsRNXData structure that will hold all the
        // GNSS-related information
        gnssRinex sGNSSRinex;

        RequireObservables clRequireObs;
        clRequireObs.addRequiredType(TypeID::L1);
        clRequireObs.addRequiredType(TypeID::L2);

        SimpleFilter clPObsFilter;
        clPObsFilter.setFilteredType(TypeID::C1);

        if (bUsingP1)
        {
                clRequireObs.addRequiredType(TypeID::P1);
                clPObsFilter.addFilteredType(TypeID::P1);
                clRequireObs.addRequiredType(TypeID::P2);
                clPObsFilter.addFilteredType(TypeID::P2);
        }
        else
        {
                clRequireObs.addRequiredType(TypeID::C1);
                clPObsFilter.addFilteredType(TypeID::C1);
                clRequireObs.addRequiredType(TypeID::P2);
                clPObsFilter.addFilteredType(TypeID::P2);
        }

        /*********** V. PPP Model Settings ***********/
        // Declare a couple of basic_ modelers
        BasicModel clBasicModel(clNominalPos, clSP3EphList);
        clBasicModel.setMinElev(MinElev);

        // Object to correct for SP3 Sat Phase-center offset
        ComputeSatPCenter clSVPcenter(clSP3EphList, clNominalPos);

        // Objects to mark cycle slips
        MWCSDetector markCSMW; // Checks Merbourne-Wubbena cycle slip

        // object def several linear combinations
        LinearCombinations clLinearCombinations;

        // Object to compute linear combinations for cycle slip detection
        ComputeLinear clLinear1;
        if (bUsingP1)
        {
                clLinear1.addLinear(clLinearCombinations.pdeltaCombination);
                clLinear1.addLinear(clLinearCombinations.mwubbenaCombination);
        }
        else
        {
                clLinear1.addLinear(clLinearCombinations.pdeltaCombWithC1);
                clLinear1.addLinear(clLinearCombinations.mwubbenaCombWithC1);
        }
        clLinear1.addLinear(clLinearCombinations.ldeltaCombination);
        clLinear1.addLinear(clLinearCombinations.liCombination);

        ComputeLinear clLinear2;

        // Read if we should use C1 instead of P1
        if (bUsingP1)
        {
                clLinear2.addLinear(clLinearCombinations.pcCombination);
        }
        else
        {
                clLinear2.addLinear(clLinearCombinations.pcCombWithC1);
        }
        clLinear2.addLinear(clLinearCombinations.lcCombination);

        LICSDetector2 clMarkCSLI2; // Checks LI cycle slips

        // Object to keep track of satellite arcs
        SatArcMarker clMarkArc;
        clMarkArc.setDeleteUnstableSats(true);

        // Objects to compute gravitational delay effects
        GravitationalDelay clGRDelay(clNominalPos);

        // Object to remove eclipsed satellites
        EclipsedSatFilter clEclipsedSV;

        //Object to compute wind-up effect
        ComputeWindUp clWindup(clSP3EphList, clNominalPos);

        // Object to compute prefit-residuals
        ComputeLinear clLinear3(clLinearCombinations.pcPrefit);
        clLinear3.addLinear(clLinearCombinations.lcPrefit);

        TypeIDSet tdTypeIDSet;
        tdTypeIDSet.insert(TypeID::prefitC);
        tdTypeIDSet.insert(TypeID::prefitL);

        // Declare a NeillTropModel object, setting the defaults
        NeillTropModel clNeillTropModel(clNominalPos.getAltitude(),
                                        clNominalPos.getGeodeticLatitude(),
                                        Doy);

        // Objects to compute the tropospheric vsRNXData
        ComputeTropModel clComputeTropModel(clNeillTropModel);

        // initialize factor NewGraph
        NonlinearFactorGraph *NewGraph = new NonlinearFactorGraph();

        /*********** VI. Main Loop ***********/
        auto start = std::chrono::steady_clock::now();
        auto end = std::chrono::steady_clock::now();
        // Loop over all vsRNXData epochs
        while (clRinex3ObsStream >> sGNSSRinex)
        {
                TimeSystem clTimeSys;
                clTimeSys.fromString("GPS");
                CommonTime clCommonTime(sGNSSRinex.header.epoch);
                clCommonTime.setTimeSystem(clTimeSys);
                GPSWeekSecond clGPSTime(clCommonTime);

                // update nominal ECEF with propogated pos.
                NeillTropModel clNeillTropModel(clNominalPos.getAltitude(),
                                                clNominalPos.getGeodeticLatitude(),
                                                Doy);
                try
                {
                        sGNSSRinex >> clRequireObs // Check if required observations are present
                            >> clPObsFilter        // Filter out spurious vsRNXData
                            >> clLinear1           // Compute linear combinations to detect CS
                            >> clMarkCSLI2         // Mark cycle slips
                            >> clMarkArc           // Keep track of satellite arcs
                            >> clBasicModel        // Compute the clBasicModel components of model
                            >> clEclipsedSV        // Remove satellites in eclipse
                            >> clGRDelay           // Compute gravitational delay
                            >> clSVPcenter         // Computer delta for sat. Phase center
                            >> clCorrCode          // Correct for differential code biases
                            >> clWindup            // Phase clWindup correction
                            >> clComputeTropModel  // neill trop function
                            >> clLinear2           // Compute ionosphere-free combinations
                            >> clLinear3;          // Compute prefit residuals
                }
                catch (Exception &e)
                {
                        continue;
                }
                catch (...)
                {
                        cerr << "Unknown exception at epoch: " << clCommonTime << endl;
                        continue;
                }

                // Iterate through the GNSS Data Structure
                satTypeValueMap::const_iterator tmplitGRinBody;
                typeValueMap::const_iterator itObs;
                if (ItsBelowThree > 0)
                {
                        ItsBelowThree = 0;
                        continue;
                }

                if (sGNSSRinex.body.size() == 0)
                {
                        continue;
                }

                // Loop over all observed sats at current epoch
                for (tmplitGRinBody = sGNSSRinex.body.begin(); tmplitGRinBody != sGNSSRinex.body.end(); tmplitGRinBody++)
                {

                        start = std::chrono::steady_clock::now();
                        SVN = ((*tmplitGRinBody).first).id;
                        double satX, satY, satZ;
                        satX = (*tmplitGRinBody).second.getValue(TypeID::satX);
                        satY = (*tmplitGRinBody).second.getValue(TypeID::satY);
                        satZ = (*tmplitGRinBody).second.getValue(TypeID::satZ);
                        Point3 SatXYZ = Point3(satX, satY, satZ);
                        double Range, RangeRes;
                        Range = (*tmplitGRinBody).second.getValue(TypeID::PC);
                        RangeRes = (*tmplitGRinBody).second.getValue(TypeID::prefitC);
                        double Phase, PhaseRes;
                        Phase = (*tmplitGRinBody).second.getValue(TypeID::LC);
                        PhaseRes = (*tmplitGRinBody).second.getValue(TypeID::prefitL);
                        int PhaseBreak;
                        PhaseBreak = (*tmplitGRinBody).second.getValue(TypeID::satArc);

                        if (bFirstOb)
                        {
                                StartKey = EpochCount;
                                bFirstOb = false;
                                NewGraph->add(PriorFactor<nonBiasStates>(X(EpochCount), clInitEst, NonBias_InitNoise));
                                NewInitialValues.insert(X(EpochCount), clInitEst);
                        }

                        if (tdPhaseArc[SVN] != PhaseBreak)
                        {
                                tdBiasState[0] = Phase - Range;
                                if (EpochCount > StartKey)
                                {
                                        tdBiasCounter[SVN] = tdBiasCounter[SVN] + 1;
                                }
                                NewGraph->add(PriorFactor<phaseBias>(G(tdBiasCounter[SVN]), tdBiasState, NnitNoise));
                                NewInitialValues.insert(G(tdBiasCounter[SVN]), tdBiasState);
                                tdPhaseArc[SVN] = PhaseBreak;
                        }
                        // Generate pseudorange factor
                        PseudorangeFactor cstGPSRangeFactor(X(EpochCount), RangeRes, SatXYZ, NominalXYZ, diagNoise::Variances((gtsam::Vector(1) << elDepWeight(SatXYZ, NominalXYZ, RangeWeight)).finished()));

                        NewGraph->add(cstGPSRangeFactor);

                        // Generate Phase factor
                        PhaseFactor cstGPSPhaseFactor(X(EpochCount), G(tdBiasCounter[SVN]), PhaseRes, SatXYZ, NominalXYZ, diagNoise::Variances((gtsam::Vector(1) << elDepWeight(SatXYZ, NominalXYZ, PhaseWeight)).finished()));

                        NewGraph->add(cstGPSPhaseFactor);
                        vPRNVec.push_back(SVN);
                }
                if (EpochCount > StartKey)
                {
                        NewGraph->add(BetweenFactor<nonBiasStates>(X(EpochCount), X(EpochCount - 1), clBetweenNonBiasState, NonBias_ProcessNoise));
                }
                Isam2.update(*NewGraph, NewInitialValues);
                Result = Isam2.calculateEstimate();

                end = std::chrono::steady_clock::now();

                clPriorNonBias = Result.at<nonBiasStates>(X(EpochCount));
                Point3 DeltaXYZ = (gtsam::Vector(3) << clPriorNonBias.x(), clPriorNonBias.y(), clPriorNonBias.z()).finished();
                Position clDeltaPos(clPriorNonBias.x(), clPriorNonBias.y(), clPriorNonBias.z());
                PropXYZ = NominalXYZ - DeltaXYZ;
                clNominalPos -= clDeltaPos;

                if (bPrintECEF)
                {
                        cout << "xyz " << clGPSTime.week << " " << clGPSTime.sow << " " << PropXYZ.x() << " " << PropXYZ.y() << " " << PropXYZ.z() << endl;
                }

                if (bPrintENU)
                {
                        Point3 enu = xyz2enu(PropXYZ, NominalXYZ);
                        cout << "enu " << clGPSTime.week << " " << clGPSTime.sow << " " << enu.x() << " " << enu.y() << " " << enu.z() << endl;
                }

                if (bPrintAmb)
                {
                        for (int k = 0; k < vPRNVec.size(); k++)
                        {
                                cout << "amb. " << clGPSTime.week << " " << clGPSTime.sow << " ";
                                cout << vPRNVec[k] << " ";
                                cout << Result.at<phaseBias>(G(tdBiasCounter[vPRNVec[k]])) << endl;
                        }
                }

                if (bPrintUpdateRate)
                {
                        cout << "Elapsed clCommonTime "
                             << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
                             << " µs" << endl;
                }

                OutputTime = OutputTime + 1;
                NewGraph->resize(0);
                NewInitialValues.clear();
                vPRNVec.clear();
                EpochCount++;
                NewInitialValues.insert(X(EpochCount), clPriorNonBias);
        }
        Isam2.saveGraph("gnss.tree");
        return 0;
}
