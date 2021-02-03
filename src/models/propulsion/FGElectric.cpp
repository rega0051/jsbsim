/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Module:       FGElectric.cpp
 Author:       David Culp
 Date started: 04/07/2004
 Purpose:      This module models an electric motor

 --------- Copyright (C) 2004  David Culp (davidculp2@comcast.net) -------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free Software
 Foundation; either version 2 of the License, or (at your option) any later
 version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along with
 this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 Place - Suite 330, Boston, MA  02111-1307, USA.

 Further information about the GNU Lesser General Public License can also be found on
 the world wide web at http://www.gnu.org.

FUNCTIONAL DESCRIPTION
--------------------------------------------------------------------------------

This class descends from the FGEngine class and models an electric motor based on
parameters given in the engine config file for this class

HISTORY
--------------------------------------------------------------------------------
04/07/2004  DPC  Created
01/06/2005  DPC  Converted to new XML format
02/03/2021  CDR  Added option to command as percent RPM rather than percent PowerReq
                  if MaxRPM != 0 the input command is treated as a normalized RPM command
                  otherwise it will treat the input command as a normalized Power command
                Added option to use a first-order lag filter on the measurement feedback
                  if tau != 0, the first order filter will be used for the RPM
                  or PowerRequired from the Thruster model
                If "tau" and "maxrpm" are absent from the XML the algorithm functions as it did before.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
INCLUDES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <iostream>
#include <sstream>

#include "FGFDMExec.h"
#include "FGElectric.h"
#include "FGPropeller.h"
#include "input_output/FGXMLElement.h"

using namespace std;

namespace JSBSim {

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS IMPLEMENTATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

FGElectric::FGElectric(FGFDMExec* exec, Element *el, int engine_number, struct FGEngine::Inputs& input)
  : FGEngine(engine_number, input)
{
  Load(exec,el);

  Type = etElectric;

  if (el->FindElement("power"))
    PowerMax = el->FindElementValueAsNumberConvertTo("power","WATTS");

  MaxRPM = 0.0; // If MaxRPM is specified the thottle command will be wrt RPM
  if (el->FindElement("maxrpm"))
    MaxRPM = el->FindElementValueAsNumber("maxrpm");

  Tau = 0.0;
  if (el->FindElement("tau"))
    Tau = el->FindElementValueAsNumber("tau");

  FiltState = 0.0; // Initialize the filter state

  string base_property_name = CreateIndexedPropertyName("propulsion/engine", EngineNumber);
  exec->GetPropertyManager()->Tie(base_property_name + "/power-hp", &HP);

  Debug(0); // Call Debug() routine from constructor if needed
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FGElectric::~FGElectric()
{
  Debug(1); // Call Debug() routine from constructor if needed
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGElectric::Calculate(void)
{
  RunPreFunctions();

  if (Thruster->GetType() == FGThruster::ttPropeller) {
    ((FGPropeller*)Thruster)->SetAdvance(in.PropAdvance[EngineNumber]);
    ((FGPropeller*)Thruster)->SetFeather(in.PropFeather[EngineNumber]);
  }

  double PowerReq = Thruster->GetPowerRequired(); // [ft-lbs/sec]

  double PowerMax_ftlbssec = PowerMax * wattstohp * hptoftlbssec;
  double Cmd = in.ThrottlePos[EngineNumber];
  double CmdPower;
  double dt = max(0.0001, in.TotalDeltaT);
  double alpha;

  if (MaxRPM > 0.0) {
    double CmdRPM = MaxRPM * Cmd;
    CmdRPM = min(CmdRPM, MaxRPM);

    double GearRatio = Thruster->GetGearRatio();
    double RPM = Thruster->GetRPM() * GearRatio;
    RPM = min(RPM, MaxRPM);

    // First order lag on RPM measure
    if (Tau > 0.0) {
      dt = max(0.0001, in.TotalDeltaT);
      alpha = 1 / (1 + (Tau / dt));

      RPM = alpha*RPM + (1-alpha)*FiltState;
      FiltState = RPM;
    }

    // Change in RPM
    double DeltaRPM = CmdRPM - RPM;
    // Power Command: DeltaPower = DeltaRPM * TorqueReq
    double TorqueReq = abs(((FGPropeller*)Thruster)->GetTorque()) / GearRatio;
    CmdPower = (TorqueReq * (DeltaRPM * rpmtoradpsec) + PowerReq); // [ft-lbs/sec]

  } else {
    double PowerReqFilt = PowerReq;

    // First order lag on Power measure
    if (Tau > 0.0) {
      double PowerReqFilt;

      dt = max(0.0001, in.TotalDeltaT);
      alpha = 1 / (1 + (Tau / dt));

      PowerReqFilt = alpha*PowerReq + (1-alpha)*FiltState;
      FiltState = PowerReqFilt;

      CmdPower -= PowerReq;
    }

    CmdPower = (PowerMax_ftlbssec) * Cmd + PowerReq - PowerReqFilt;
  }

  CmdPower = min(CmdPower, PowerMax_ftlbssec); // Limit to PowerMax

  LoadThrusterInputs();
  Thruster->Calculate(CmdPower);
  HP = CmdPower / hptoftlbssec;

  RunPostFunctions();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double FGElectric::CalcFuelNeed(void)
{
  return 0;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string FGElectric::GetEngineLabels(const string& delimiter)
{
  std::ostringstream buf;

  buf << Name << " HP (engine " << EngineNumber << ")" << delimiter
      << Thruster->GetThrusterLabels(EngineNumber, delimiter);

  return buf.str();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string FGElectric::GetEngineValues(const string& delimiter)
{
  std::ostringstream buf;

  buf << HP << delimiter
     << Thruster->GetThrusterValues(EngineNumber, delimiter);

  return buf.str();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    The bitmasked value choices are as follows:
//    unset: In this case (the default) JSBSim would only print
//       out the normally expected messages, essentially echoing
//       the config files as they are read. If the environment
//       variable is not set, debug_lvl is set to 1 internally
//    0: This requests JSBSim not to output any messages
//       whatsoever.
//    1: This value explicity requests the normal JSBSim
//       startup messages
//    2: This value asks for a message to be printed out when
//       a class is instantiated
//    4: When this value is set, a message is displayed when a
//       FGModel object executes its Run() method
//    8: When this value is set, various runtime state variables
//       are printed out periodically
//    16: When set various parameters are sanity checked and
//       a message is printed out when they go out of bounds

void FGElectric::Debug(int from)
{
  if (debug_lvl <= 0) return;

  if (debug_lvl & 1) { // Standard console startup message output
    if (from == 0) { // Constructor

      cout << "\n    Engine Name: "         << Name << endl;
      cout << "      Power Max Watts: "         << PowerMax << endl;

    }
  }
  if (debug_lvl & 2 ) { // Instantiation/Destruction notification
    if (from == 0) cout << "Instantiated: FGElectric" << endl;
    if (from == 1) cout << "Destroyed:    FGElectric" << endl;
  }
  if (debug_lvl & 4 ) { // Run() method entry print for FGModel-derived objects
  }
  if (debug_lvl & 8 ) { // Runtime state variables
  }
  if (debug_lvl & 16) { // Sanity checking
  }
  if (debug_lvl & 64) {
    if (from == 0) { // Constructor
    }
  }
}

} // namespace JSBSim
