/*
 * wheelset.cpp
 *
 *  Created on: Nov 27, 2013
 *      Author: leonardo
 */

#include "wheelset.h"

Wheelset::Wheelset(const std::string& name) : MBSim::RigidBody(name), track(1.0)
{
  // TODO Auto-generated constructor stub

}

void Wheelset::enableOpenMBV()
{
  std::vector<OpenMBV::PolygonPoint*>* points = new std::vector<OpenMBV::PolygonPoint*>(7);
  points->at(0) = new OpenMBV::PolygonPoint(.1,-0.067,0.);
  points->at(1) = new OpenMBV::PolygonPoint(.4344,-0.067,0.);
  points->at(2) = new OpenMBV::PolygonPoint(.4572,0.0,0.);
  points->at(3) = new OpenMBV::PolygonPoint(.4593,0.043,0.);
  points->at(4) = new OpenMBV::PolygonPoint(.4824,0.043,0.);
  points->at(5) = new OpenMBV::PolygonPoint(.4824,0.077,0.);
  points->at(6) = new OpenMBV::PolygonPoint(.1,0.077,0.);

  OpenMBV::Rotation* openMBVwheel1 = new OpenMBV::Rotation();
  openMBVwheel1->setName("Roda 1");;
  openMBVwheel1->setContour(points);
  openMBVwheel1->setInitialRotation(M_PI/2,0.,0.);
  openMBVwheel1->setInitialTranslation(0.,0.,track/2);

  OpenMBV::Rotation* openMBVwheel2 = new OpenMBV::Rotation();
  openMBVwheel1->setName("Roda 2");
  openMBVwheel2->setContour(points);
  openMBVwheel2->setInitialRotation(3*M_PI/2,0.,0.);
  openMBVwheel2->setInitialTranslation(0.,0.,-track/2);

  OpenMBV::CompoundRigidBody* openMBVwheelset = new OpenMBV::CompoundRigidBody();
  openMBVwheelset->addRigidBody(openMBVwheel1);
  openMBVwheelset->addRigidBody(openMBVwheel2);

  this->setOpenMBVRigidBody(openMBVwheelset);
}
