/*
    Implements a truck with "constant" damping
    This model is based on AmstedMaxion SS Ride Control 7 in x 12 in.
    Part no. 3013-40754-00-1


    Copyright (C) 2016  Leonardo Baruffaldi leobart@fem.unicamp.br

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef BARBERTRUCK_H
#define BARBERTRUCK_H

#include "truck.h"
#include "wedge.h"
#include "RideControlWedge.h"
#include "bolster.h"
#include "sinusoidalmovement.h"
#include "wheelset.h"
#include "mbsim/rigid_body.h"
#include "mbsim/joint.h"
#include "mbsim/spring_damper.h"
#include "mbsim/environment.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/edge.h"
#include <mbsim/contours/compound_contour.h>
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"

/*
 * This object is not included in the mbsim 11.0 official distribution,
 * so it was added individually
 */
#include "isotropic_rotational_spring_damper.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/coilspring.h"
#include "openmbvcppinterface/cuboid.h"
#include "openmbvcppinterface/extrusion.h"
#include "openmbvcppinterface/sphere.h"
#endif

#include <iostream>

using namespace MBSim;
using namespace fmatvec;
using namespace std;



class RideControlTruck : public Truck
{
public:
  //RideControlTruck ( const std::string& projectName );
  RideControlTruck ( const std::string& projectName,
      double frictionCoeff_ = 0.2,
      double wheelBase = 2.0);

  RideControlWedge* wedgeFrontLeft;
  RideControlWedge* wedgeRearLeft;
  RideControlWedge* wedgeFrontRight;
  RideControlWedge* wedgeRearRight;
  Bolster* bolster;
  RigidBody* sideFrame;
  Wheelset* wheelL;
  Wheelset* wheelR;
  
  void setWheelBase(double wB_);
  double getWheelBase();
  
private:
  double frictionCoefficient;
  double wheelBase;

  // sets up the wedges by passing the basic parameters
  // the wedge DOFs are set inside this method
  void setupWedge ( RideControlWedge* wedge,
      double mass,
      double angleSideframe,
      double angleBolster,
      double height,
      double depth,
      Frame* refFrame);
};

#endif // BARBERTRUCK_H
