/*
 * truck.cpp
 *
 *  Created on: Oct 30, 2013
 *      Author: leonardo
 */

#include "truck.h"

Truck::Truck(const std::string& name) : MBSim::Group(name)
{
  // TODO Auto-generated constructor stub

}

void Truck::springOpenMBVgraphics(MBSim::SpringDamper *spring, int numOfCoils,
    double wireRadius, double spiralRadius){
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::CoilSpring *openMBVSpring = new OpenMBV::CoilSpring();
  openMBVSpring->setCrossSectionRadius ( wireRadius );
  openMBVSpring->setNumberOfCoils ( numOfCoils );
  openMBVSpring->setSpringRadius ( spiralRadius );
  spring->setOpenMBVSpring ( openMBVSpring );
#endif
}
