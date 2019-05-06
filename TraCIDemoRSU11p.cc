//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <algorithm>    // std::find


#include "veins/modules/application/traci/TraCIDemoRSU11p.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"

#define RECURRING_TIMER_EVT 2
#define CHECK_TRAFFIC_EVT 3
#define INITIAL_EVT 4

#define STATS_FILENAME "perfMetrics.txt"

//################# ADD UAQ ######################################

std::vector<int> carId_N_new;  //vettore id dei veicoli sulla lane N (sequenza di myId)
std::vector<int> carId_S_new;  //vettore id dei veicoli sulla lane S (sequenza di myId)
std::vector<int> carId_W_new;  //vettore id dei veicoli sulla lane W (sequenza di myId)
std::vector<int> carId_E_new;  //vettore id dei veicoli sulla lane E (sequenza di myId)

int count_N_new = 0; //contatore numero nuovi veicoli su lane N
int count_S_new = 0; //contatore numero nuovi veicoli su lane S
int count_W_new = 0; //contatore numero nuovi veicoli su lane W
int count_E_new = 0; //contatore numero nuovi veicoli su lane S


//Incrocio Crossing
//definizione variabili tabella temporanea (TEMP)
std::vector<int> carId_N;  //vettore veicoli sulla lane N (sequenza di myId)
std::vector<int> carId_S;  //vettore veicoli sulla lane S (sequenza di myId)
std::vector<int> carId_W;  //vettore veicoli sulla lane W (sequenza di myId)
std::vector<int> carId_E;  //vettore veicoli sulla lane E (sequenza di myId)
int count_N = 0; //contatore numero veicoli su lane N
int count_S = 0; //contatore numero veicoli su lane S
int count_W = 0; //contatore numero veicoli su lane W
int count_E = 0; //contatore numero veicoli su lane S

// ############END UAQ #################################################

// Use the names coming from aitek report
// Low scene complexity
static const constexpr double DetectionRate = 0.94;
static const constexpr double FalseAlertRate = 0.04;
// High scene complexity
// static const constexpr double DetectionRate = 0.81;
// static const constexpr double FalseAlertRate = 0.09;
// Extreme case for tests (not realistic)
// static const constexpr double DetectionRate = 0.5;
// static const constexpr double FalseAlertRate = 0.3;

// static std::random_device rd;
static std::mt19937_64 DetectionGenerator(0);
static std::mt19937_64 FalseAlertGenerator(0);
static std::uniform_real_distribution<double> DetectionProb(0.0L, 1.0L);
static std::uniform_real_distribution<double> FalseAlertProb(0.0L, 1.0L);

static bool isDetected() {
    return DetectionProb(DetectionGenerator) < DetectionRate;
}

static bool isFalseAlert() {
    return FalseAlertProb(FalseAlertGenerator) < FalseAlertRate;
}

static bool startsWith(const std::string &haystack, const std::string &needle) {
    const auto needle_len = needle.length();
    const auto haystack_len = haystack.length();
    if (needle_len > haystack_len)
        return false;
    return std::memcmp((void *)(needle.data()),
            (void *)(haystack.data()),
            needle_len) == 0;
}

static bool endsWith(const std::string &haystack, const std::string &needle) {
    const auto needle_len = needle.length();
    const auto haystack_len = haystack.length();
    if (needle_len > haystack_len)
        return false;
    const auto offset = haystack_len - needle_len;
    return std::memcmp((void *)(needle.data()),
            (void *)(haystack.data() + offset),
            needle_len) == 0;
}

static inline void setupStatFile(const std::string &TLName) {
    std::ofstream f_perf;
    f_perf.open(TLName + STATS_FILENAME);
    if (f_perf.is_open()) {
        f_perf << "Performance metric for traffic light optimal control\n\n"
                "simulation time\tnot optimized\toptimized\ttl program"
                << std::endl;
    }
    f_perf.close();
}

static inline void dumpCapacityMetric(const std::string &TLName,
        simtime_t SimTime,
        double balancedCapacity,
        double maxCapacity,
        const std::string &CurrProgram) {
    std::ofstream f_perf;
    std::string filename = TLName + STATS_FILENAME;
    f_perf.open(filename, std::ios::app);
    if (f_perf.is_open()) {
        f_perf << SimTime << "\t\t"
                << balancedCapacity << "\t\t"
                << maxCapacity << "\t\t"
                << CurrProgram << std::endl;
    }
    f_perf.close();
}

Define_Module(TraCIDemoRSU11p);

void TraCIDemoRSU11p::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);

    if (stage + 1 < numInitStages())
        return;

    // Initialize the pointer to the scenario manager
    manager = Veins::TraCIScenarioManagerAccess().get();
    assert(manager != nullptr);

    checkTrafficEvt = new cMessage("OptimalTraffic Event", CHECK_TRAFFIC_EVT);
   // scheduleAt(simTime() + 90, checkTrafficEvt);
    scheduleAt(simTime() + 4, checkTrafficEvt);

    initialEvt = new cMessage("RSU Startup Event", INITIAL_EVT);
    scheduleAt(simTime(), initialEvt);
}

 //ADD UAQ
void TraCIDemoRSU11p::onWSA(WaveServiceAdvertisment* wsa) {                     //add UAQ
    //if this RSU receives a WSA for service 42, it will tune to the chan
    if (wsa->getPsid() == 42) {
        mac->changeServiceChannel(wsa->getTargetChannel());
    }
}

void TraCIDemoRSU11p::onWSM(WaveShortMessage* wsm) {                            //add UAQ
    //this rsu repeats the received traffic update in 2 seconds plus some random delay
    wsm->setSenderAddress(myId);
    sendDelayedDown(wsm->dup(), 2 + uniform(0.01,0.2));
//    std::cout<<"RSU ha ricevuto un WSM da: " << wsm->getSenderAddress() << " campo dati: "<< wsm->getWsmData()<< " al t: " << simTime() << endl;  //
}


void TraCIDemoRSU11p::onBSM(BasicSafetyMessage* bsm) {                        //add UAQ
    // Your application has received a beacon message from another car or RSU
    // code for handling the message goes here.
    int Identificativo =  bsm ->getSenderAddress();
    std::string strada =  bsm->getSenderRoad();
    //std::cout<<"message received from "<<Identificativo<<" at time "<<simTime()<<" on the street "<<strada<<endl;

    //Discriminazione RoadID del veicolo associato al BSM ricevuto


     if(strada== "NC"){


         bool exists = std::find(std::begin(carId_N), std::end(carId_N), Identificativo) != std::end(carId_N);
         if(!exists)
         {
                carId_N.push_back(Identificativo);
                count_N++;
         }

     }

     else if(strada== "SC"){

         bool exists = std::find(std::begin(carId_S), std::end(carId_S), Identificativo) != std::end(carId_S);
            if(!exists){ //if the ID is not in the vector
                carId_S.push_back(Identificativo);
                count_S++;
            }
     }

     else if(strada==  "WC"){

         bool exists = std::find(std::begin(carId_W), std::end(carId_W), Identificativo) != std::end(carId_W);
            if(!exists){
                carId_W.push_back(Identificativo);
                count_W++;
            }
     }


     else if(strada==  "EC"){

         bool exists = std::find(std::begin(carId_E), std::end(carId_E), Identificativo) != std::end(carId_E);
            if(!exists){
                carId_E.push_back(Identificativo);
                count_E++;
            }
     }


     // CROSSING_NEW
     else if(strada==  "NC_new"){

         bool exists = std::find(std::begin(carId_N_new), std::end(carId_N_new), Identificativo) != std::end(carId_N_new);
            if(!exists){
                carId_N_new.push_back(Identificativo);
                count_N_new++;
//                std::cout<<"NEW VEHICLE "<<Identificativo<<" ON NC_new!!!!!!!!! The count now is: "<<count_N_new<<endl;
//                  for (std::vector<int>::iterator j=carId_N_new.begin(); j!=carId_N_new.end(); ++j)
//                         {
//                      std::cout << ' ' << *j;
//                         }
//                  std::cout << '\n';
            }
     }

     else if(strada==  "SC_new"){

         bool exists = std::find(std::begin(carId_S_new), std::end(carId_S_new), Identificativo) != std::end(carId_S_new);
                 if(!exists){
                     carId_S_new.push_back(Identificativo);
                     count_S_new++;

                 }
     }

     else if(strada==  "WC_new"){

         bool exists = std::find(std::begin(carId_W_new), std::end(carId_W_new), Identificativo) != std::end(carId_W_new);
                 if(!exists){
                     carId_W_new.push_back(Identificativo);
                     count_W_new++;

                 }
     }

     else if(strada==  "EC_new"){

         bool exists = std::find(std::begin(carId_E_new), std::end(carId_E_new), Identificativo) != std::end(carId_E_new);
                 if(!exists){
                     carId_E_new.push_back(Identificativo);
                     count_E_new++;

                 }
     }


 }

// END UAQ


// Traffic light cycle length
static const constexpr int t_C = 90;

// Fixed time for yellow in each direction
static const constexpr int YellowTime = 4;

// Total fixed time for yellow
static const constexpr int TotalYellowTime = 2 * YellowTime;

static double computeCapacity(const double VerGreenTime,
        const std::array<int, NumDirections> &InputFlow) {
    // Saturation flow of access i. In this case all the accesses are equal.
    // The exact reason why the value 120 was choosed it's not clear to me,
    // but I guess that given that it's a constant it does not really matter
    // for this specific example.
    static const constexpr double s_i = 120.0L;

    const int NorthInputFlow = InputFlow[getDirection('N')];
    const int SouthInputFlow = InputFlow[getDirection('S')];
    const int EastInputFlow  = InputFlow[getDirection('E')];
    const int WestInputFlow  = InputFlow[getDirection('W')];

    const double f_N = static_cast<double>(NorthInputFlow);
    const double f_S = static_cast<double>(SouthInputFlow);
    const double f_W = static_cast<double>(WestInputFlow);
    const double f_E = static_cast<double>(EastInputFlow);
    const double g_E_Ver = static_cast<double>(VerGreenTime);
    const double g_E_Hor = t_C - VerGreenTime - TotalYellowTime;

    // compute capacities for vertical and horizontal directions
    double WCapacity = (WestInputFlow != 0) ? ((s_i * g_E_Hor) / (f_W * t_C)) :
            std::numeric_limits<double>::max();
    double ECapacity = (EastInputFlow != 0) ? ((s_i * g_E_Hor) / (f_E * t_C)) :
            std::numeric_limits<double>::max();
    double NCapacity = (NorthInputFlow != 0) ? ((s_i * g_E_Ver) / (f_N * t_C)) :
            std::numeric_limits<double>::max();
    double SCapacity = (SouthInputFlow != 0) ? ((s_i * g_E_Ver) / (f_S * t_C)) :
            std::numeric_limits<double>::max();
    // compute minimal capacity for this program
    return std::min(std::min(ECapacity, WCapacity), std::min(NCapacity, SCapacity));
}

struct TLCapacity {
    double Capacity;
    int VerticalGreenSecs;
};

static TLCapacity getMaxCapacity(const std::array<int, NumDirections> &InputFlow) {
    double MaxCapacity = 0.0;
    int MaxCapacityVerGreenSecs = 0;
    // Maximization loop on p_k, where p_k is the duration of the next green
    // phase of the intersection (for vertical direction)
    for (int VerGreenSecs = 4; VerGreenSecs < 79; ++VerGreenSecs) {

        const double VerGreenTime = static_cast<double>(VerGreenSecs);
        const double ProgramCapacity = computeCapacity(VerGreenTime, InputFlow);

        // Store the id of the program with max capacity, along with the value of
        // the max capacity, used later for statistics
        if (ProgramCapacity > MaxCapacity) {
            MaxCapacity = ProgramCapacity;
            MaxCapacityVerGreenSecs = VerGreenSecs;
        }
    }
    return { MaxCapacity, MaxCapacityVerGreenSecs };
}

static constexpr const bool ErrorModeling = false;     // comm UAQ
bool V2IComm = true;    // add UAQ (abilitare l'ottimizzazione della logica semaforica con i V2Iflow )

void TraCIDemoRSU11p::setOptimalProgram(const std::string &TLName) {

    TraCICommandInterface::Trafficlight TL = traci->trafficlight(TLName);
    /*

     * This function tries to set the traffic light program to maximize the
     * intersection capacity, as defined in the paper "Global Sensitivity
     * Analysis of the Optimal Capacity of Intersections" - A. Di Febbraro,
     * N. Sacco Bauda` - IFAC Proceedings Volumes, Vol.47, Issue 3, 2014 -
     * https://doi.org/10.3182/20140824-6-ZA-1003.01975
     */

    int TLId = endsWith(TLName, "_new") ? 1 : 0;
    RedundantCrossingData &RCD = Scenario.TrafficLights[TLId];


    double BalancedCapacity = computeCapacity(41, RCD.RealFlow);
    TLCapacity MaxCapacity = getMaxCapacity(RCD.RealFlow);

//   std::cout<<"T: "<<simTime()<< "TLID"<<TLId<<" N:"<< RCD.RealFlow[0]<<" E:"<< RCD.RealFlow[1]<<" S:"<<RCD.RealFlow[2]<<" W:" << RCD.RealFlow[3]<< std::endl;

    std::string NewProgram = "VG" + std::to_string(MaxCapacity.VerticalGreenSecs) + "Secs";
    dumpCapacityMetric(TLName, simTime(), BalancedCapacity, MaxCapacity.Capacity, "Real" + NewProgram);

    // Assume that x is the real number of vehicles, and that we know the failure
    // rates of the detectors (DetectionRate, FalseAlertRate).
    // The number of detected vechicles from each detector R will be:
    //   x_R = x (DetectionRate + FalseAlertRate)
    // Hence, knowing a single x_R, our estimate for x is:
    //   x = x_R / (DetectionRate + FalseAlertRate)
    // As a consequence, knowing all the x_R for R in [0, Redundancy), we
    // estimate x as follows:
    //   x = mean(x_R) / (DetectionRate + FalseAlertRate)
      if (ErrorModeling) {
        std::array<int, NumDirections> DetectedFlow = { 0, 0, 0, 0};
        for (int DirectionId = 0; DirectionId < NumDirections; ++DirectionId) {
          auto &Detected = RCD.DetectedFlow[DirectionId];
          // CumulatedFlow = sum(x_R) over all the R
          int CumulatedFlow = std::accumulate(Detected.begin(), Detected.end(), 0);
          // Flow == mean(x_R) over all the R (with rounding)
          double Flow = (double) CumulatedFlow / (double) Redundancy;
          // Finally, we divide by (DetectionRate + FalseAlertRate)
          Flow = Flow / (DetectionRate + FalseAlertRate);
          DetectedFlow[DirectionId] = std::lround(std::ceil(Flow));
        }


        BalancedCapacity = computeCapacity(41, DetectedFlow);
        MaxCapacity = getMaxCapacity(DetectedFlow);

        NewProgram = "VG" + std::to_string(MaxCapacity.VerticalGreenSecs) + "Secs";
        dumpCapacityMetric(TLName, simTime(), BalancedCapacity, MaxCapacity.Capacity, "Detected" + NewProgram);
      }
 //######## ADD UAQ #####################################
      if (V2IComm){
          std::array<int, NumDirections> DetectedFlow = { 0, 0, 0, 0};
          if (TLId==0)
          {


               DetectedFlow[0] = count_N;
               DetectedFlow[1] = count_E;
               DetectedFlow[2] = count_S;
               DetectedFlow[3] = count_W;


         }
          else
         {
              DetectedFlow[0] = count_N_new;
              DetectedFlow[1] = count_E_new;
              DetectedFlow[2] = count_S_new;
              DetectedFlow[3] = count_W_new;

//              std::ofstream f_perf10;
//               std::string filename = "InputFlow.txt";
//               f_perf10.open(filename, std::ios::app);
//               if (f_perf10.is_open()) {
//                   f_perf10 << simTime() << "\t\t";
//                   f_perf10 << TLId << "\t\t";
//                   f_perf10 << DetectedFlow[0]<<"\t\t";
//                   f_perf10 << DetectedFlow[1]<<"\t\t";
//                   f_perf10 << DetectedFlow[2]<<"\t\t";
//                   f_perf10 << DetectedFlow[3]<< std::endl;
//               }
//               f_perf10.close();
        }


        BalancedCapacity = computeCapacity(41, DetectedFlow);
        MaxCapacity = getMaxCapacity(DetectedFlow);

        NewProgram = "VG" + std::to_string(MaxCapacity.VerticalGreenSecs) + "Secs";

//        if (TLId== 0) {
//            std::ofstream f_perf2;
//            std::string filename = "V2IComm_DetectedCrossing.txt";
//            f_perf2.open(filename, std::ios::app);
//            if (f_perf2.is_open()) {
//                f_perf2 << simTime() << "\t\t";
//                f_perf2 << BalancedCapacity << "\t\t";
//                f_perf2 << MaxCapacity.Capacity << "\t\t";
//                f_perf2 << "Detected" + NewProgram <<std::endl;
//            }
//            f_perf2.close();
//          }
//        if (TLId== 1) {
//            std::ofstream f_perf3;
//            std::string filename = "V2IComm_DetectedCrossing_new.txt";
//            f_perf3.open(filename, std::ios::app);
//            if (f_perf3.is_open()) {
//                f_perf3 << simTime() << "\t\t";
//                f_perf3 << BalancedCapacity << "\t\t";
//                f_perf3 << MaxCapacity.Capacity << "\t\t";
//                f_perf3 << "Detected" + NewProgram <<std::endl;
//            }
//
//            f_perf3.close();
//        }
    }


// ######################END UAQ#########################################


    const std::string &CurrProgram = TL.getCurrentProgram();
    if (CurrProgram == NewProgram){

        RCD.RealFlow = { 0 }; // ADD UAQ
        RCD.DetectedFlow = { { 0 } }; // ADD UAQ
        if (TLId==0)
                  {

        count_N=0;
        count_S=0;
        count_W=0;
        count_E=0;
                  }
        else{
        count_N_new=0;
        count_S_new=0;
        count_W_new=0;
        count_E_new=0;
        }

        return;
    }

    if (CurrProgram != NewProgram) {
        int NewVerGreenSecs = MaxCapacity.VerticalGreenSecs;
        int NewHorGreenSecs = t_C - TotalYellowTime - NewVerGreenSecs;
        using Phase = TraCICommandInterface::Trafficlight::Phase;
        std::vector<Phase> Phases = {};
        Phases.push_back({ NewVerGreenSecs, "ggrrggrr" });
        Phases.push_back({ YellowTime, "yyrryyrr" });
        Phases.push_back({ NewHorGreenSecs, "rrggrrgg" });
        Phases.push_back({ YellowTime, "rryyrryy" });

        assert(Phases.size() == 4);

        int PhaseID = TL.getPhaseIndex();
        assert(PhaseID < 4 and PhaseID > -1);

        int CurrPhaseDuration = TL.getCurrPhaseDuration();
        int NextSwitchTime = TL.getNextSwitchSecs();
        int SumoSimTime = traci->getCurrentTimeSecs();
        int CurrPhaseLeftDuration = NextSwitchTime - SumoSimTime;
        int CurrPhaseConsumedDuration = CurrPhaseDuration - CurrPhaseLeftDuration;
        int NewPhaseDuration = Phases.at(PhaseID).Duration;
        int NewPhaseLeftDuration = NewPhaseDuration - CurrPhaseConsumedDuration;

        // Set the tl program according to the maximization of the capacity
        TL.addCompleteProgram(NewProgram, Phases, 0);
        TL.setProgram(NewProgram);
        TL.setPhaseIndex(PhaseID);

        if (NewPhaseLeftDuration > 0)
            TL.setPhaseLeftDuration(NewPhaseLeftDuration);
        else
            TL.setPhaseLeftDuration(0);
    }

    RCD.RealFlow = { 0 };
    RCD.DetectedFlow = { { 0 } }; // ADD UAQ
    if (TLId==0)
              {
          count_N=0;
          count_S=0;
          count_W=0;
          count_E=0;
              }
    else
    {
          count_N_new=0;
          count_S_new=0;
          count_W_new=0;
          count_E_new=0;
    }
    return;
}


static void computeTrafficFlow(int TLId, int Dir,
                               VehicleSet &Vehicles,
                               int &InputFlow,
                               TraCICommandInterface *traci,
                               bool hasErrors = false
                               ) {
  VehicleSet NewVehicles;
  std::string laneDetectorId;   //add UAQ
  for (const auto &ID : traci->getLaneAreaDetectorIds()) {
    assert(ID.length() > 5);
    assert(startsWith(ID, "lad"));
//    std::cout<<"ID laneDetector: "<< ID << std::endl;
    // This is ad-hoc to tell the two traffic lights apart
    // It depends on the names given to the traffic light objects in the
    // configuration files and it may change in future.
    char TLCharID = ID.at(3);
    int LADTrafficLightID = charToInt(TLCharID);
    if (LADTrafficLightID != TLId)
      continue;
    // This is ad-hoc to tell lanes apart.
    // It depends on the names given to the lanes objects in the
    // configuration files and it may change in future.
    char DirChar = ID.at(5);
    enum Direction D = getDirection(DirChar);
    if (D != Dir)
      continue;

    auto LaneDetector = traci->laneAreaDetector(ID);
    VehicleSet Vehicles = LaneDetector.getLastStepVehicleIDs();
    NewVehicles.insert(Vehicles.begin(), Vehicles.end());
    }

  size_t NumNewVehicles = 0;
//  for (const auto &V : Vehicles)       //comm UAQ (verifica errata)
//    if (NewVehicles.count(V) != 0)     //comm UAQ
//      NumNewVehicles += 1;             // comm UAQ

//ADD UAQ ############################################
  for (auto num : NewVehicles)
  {
      // no deference operator

      if (Vehicles.count(num) != 0)
      {
          //              std::cout<<"T: "<<simTime() <<" Car già presente: "<<  num << "\n";
      }
      else{
          NumNewVehicles += 1;
          //              std::cout <<"T: "<< simTime()<< " New car: " << num << "\n";
      }
      //          std::cout<<"T: "<<simTime()<<" TLId: "<<TLId<<" D: "<< Dir<< " NumNewVehicles: " << NumNewVehicles<< std::endl;
  }

  //END UAQ ###############################################################
  if (hasErrors) {
      size_t NumDetectedVehicles = 0;
      for (size_t N = 0; N < NumNewVehicles; ++N)
          if (isDetected())
              NumDetectedVehicles += 1;
      for (size_t N = 0; N < NumNewVehicles; ++N)
          if (isFalseAlert())
              NumDetectedVehicles += 1;
      InputFlow += NumDetectedVehicles;
  } else {
      InputFlow += NumNewVehicles;
  }
//  std::cout<<"T: "<<simTime()<<" TLId: "<<TLId<<" D: "<< Dir<<" InputFlow: "<< InputFlow<< std::endl;
    Vehicles = std::move(NewVehicles);

//    if (TLId== 0  && Dir==0) {
//        std::ofstream f_perf0;
//        std::string filename = "InputFlow_TLId0.txt";
//        f_perf0.open(filename, std::ios::app);
//        if (f_perf0.is_open()) {
//            f_perf0 << simTime() << "\t\t";
//            f_perf0 << TLId << "\t\t";
//            f_perf0 << Dir << "\t\t";
//            f_perf0 << InputFlow << std::endl;
//        }
//        f_perf0.close();
//    }
//    if (TLId== 1&& Dir==0) {
//        std::ofstream f_perf1;
//        std::string filename = "InputFlow_TLId1.txt";
//        f_perf1.open(filename, std::ios::app);
//        if (f_perf1.is_open()) {
//            f_perf1 << simTime() << "\t\t";
//            f_perf1 << TLId<< "\t\t";
//            f_perf1 << Dir << "\t\t";
//            f_perf1 << InputFlow << std::endl;
//        }
//
//        f_perf1.close();
//    }

}

void TraCIDemoRSU11p::computeTrafficFlows(TraCICommandInterface *traci) {
  // Count the number of vehicles in horizontal and vertical directions
  for (int TLId = 0; TLId < NumIntersections; ++TLId) {
    RedundantCrossingData &RCD = Scenario.TrafficLights[TLId];
    for (int Dir = 0; Dir < NumDirections; ++Dir) {
      VehicleSet &RealVehicles = RCD.RealVehicles[Dir];
      int &RealInputFlow = RCD.RealFlow[Dir];
     computeTrafficFlow(TLId, Dir, RealVehicles, RealInputFlow, traci);
      if (not ErrorModeling)
        continue;
      for (int RedundantId = 0; RedundantId < Redundancy; ++RedundantId) {
        VehicleSet &DetectedVehicles = RCD.DetectedVehicles[Dir][RedundantId];
        int &DetectedInputFlow = RCD.DetectedFlow[Dir][RedundantId];
        computeTrafficFlow(TLId, Dir, DetectedVehicles, DetectedInputFlow, traci, true);
      }
    }
  }
}

static void setBalanced(TraCICommandInterface::Trafficlight &TL) {
    assert(TL.hasTraCI());
    assert(TL.hasConnection());
    TL.setProgram("Balanced");
    TL.setPhaseIndex(0);
}

void TraCIDemoRSU11p::handleSelfMsg(cMessage *msg) {

    switch (msg->getKind()) {

    case INITIAL_EVT: {
        // Initialize member pointers
        // We only do it here because this message comes at the beginning of the
        // simulation
        manager = Veins::TraCIScenarioManagerAccess().get();
        assert(manager != nullptr);
        traci = manager->getCommandInterface();
        assert(traci != nullptr);

        // This event is supposed to be received only once at the beginning of
        // simulation time and it is used for two things:
        // 1 - setup the traffic light with the default balanced policy
        // 2 - setup the recurring timer event that will wake up the RSU every
        //     second to collect statistics about the vehicles in the various
        //     lanes, and to decide the new policies to adopt according to the
        //     traffic

        // Set the default balanced program for the traffic lights
        auto OldTL = traci->trafficlight("Crossing");
        auto NewTL = traci->trafficlight("Crossing_new");
        setBalanced(OldTL);
        setBalanced(NewTL);
        setupStatFile("Crossing");
        setupStatFile("Crossing_new");

        // Cleanup the event
        cancelEvent(initialEvt);

        // Setup the recurring timer event
        sendTimerEvt = new cMessage("Timer Event", RECURRING_TIMER_EVT);
        scheduleAt(simTime() + 1, sendTimerEvt);

    } break;


      case RECURRING_TIMER_EVT: {
    // First, handle the recurring timer event and set it up for the new
    // occurence
    cancelEvent(sendTimerEvt);
    scheduleAt(simTime() + 1, sendTimerEvt);

    // Compute horizontal and vertical input traffic flows
    computeTrafficFlows(traci);
  } break;

    case CHECK_TRAFFIC_EVT: {
        setOptimalProgram("Crossing");
        setOptimalProgram("Crossing_new");
        cancelEvent(checkTrafficEvt);
        scheduleAt(simTime() + 4, checkTrafficEvt);
    } break;

    default:
        // do nothing
        break;
    }
}

static inline void cleanupMsg(TraCIDemoRSU11p *RSU, cMessage **MsgPtr) {
    assert(MsgPtr != nullptr);
    cMessage *Msg = *MsgPtr;
    if (Msg != nullptr) {
        if (Msg->isScheduled())
            RSU->cancelEvent(Msg);
    }
    delete Msg;
    Msg = nullptr;
}

void TraCIDemoRSU11p::finish() {
    Scenario = SafeCOPScenario();
    cleanupMsg(this, &sendTimerEvt);
    cleanupMsg(this, &checkTrafficEvt);
    cleanupMsg(this, &initialEvt);

    manager = nullptr;

    BaseWaveApplLayer::finish();
}

