/* 
 * File:   main.cpp
 * Author: lbb
 *
 * Created on March 30, 2013, 2:20 PM
 */

#ifndef _OPENMP
#define _OPENMP
#endif

#include <cstdlib>
#include <stdio.h> /* needed this include just because Eclipse can't
                       track down namespace std from cstdlib */
#include <iostream>
#include <string>
#include <sstream>
#include "mbsim/integrators/integrators.h"
#include "inputTools.h"
#include "system.h"


using namespace std;
using namespace MBSim;

/* 
 * 
 */
int main(int argc, char** argv)
{
 // read input file
 string line;
 string inputFileName = "input.in";
  
  // settings
 double endTime = atof(searchParameter(inputFileName,"SIMULATION_TIME").c_str());

 // build single modules
 DynamicSystemSolver *sys = new System(searchParameter(inputFileName,"SYSTEM_NAME"),inputFileName);
 
 // add modules to overall dynamical system
 sys->setConstraintSolver(GaussSeidel);
 sys->setImpactSolver(GaussSeidel);
 sys->setFlushEvery(100);
 sys->initialize();
 sys->setStopIfNoConvergence(true,true);
 sys->setMaxIter(1e6);
 sys->setgTol(1e-6);
 sys->setgdTol(2e-3);
 sys->setLaTol(2e-2);
 sys->setgddTol(2e-5);
 
 TimeSteppingSSCIntegrator integrator;
 //integrator.setStepSize(5e-6);
 integrator.setAbsoluteTolerance(1e-8);
 integrator.setRelativeTolerance(1e-5);
 integrator.setEndTime(endTime);
 integrator.setPlotStepSize(1e-3);
 integrator.setGapControl(1);
 integrator.integrate(*sys);
 cout << "finished" << endl;
 
 return 0;
}
