/*
    Creates a wedged rigid body
    Copyright (C) 2013  Leonardo Baruffaldi leobart@fem.unicamp.br

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


#include "wedge.h"
#include <fmatvec/fmatvec.h>

Wedge::Wedge(const std::string &name) : RigidBody(name), height(1.0), depth(1.0), 
angle1Radians(0.5), angle2Radians(0.0) { 
  pointTable = new fmatvec::Mat();
  gRefFramePosition = new fmatvec::Vec(3,fmatvec::INIT,0.0);
  geometryReferenceFrame = new MBSim::FixedRelativeFrame("RG",*gRefFramePosition,
      fmatvec::SqrMat(3,fmatvec::EYE));
  this->addFrame(geometryReferenceFrame); /// TODO erro na modificação da posição
}

void Wedge::setHeight(double h_)
{
  height = h_;
  updateGrefFramePosition();
}

void Wedge::setAngles(fmatvec::Vec angs_)
{
  angle1Radians = angs_(0);
  angle2Radians = angs_(1);
  updateGrefFramePosition();
}

void Wedge::setDepth(double d_)
{
  depth = d_;
  updateGrefFramePosition();
}

void Wedge::buildContour(void )
{  
  int numberOfPoints = 8;

  pointTable->resize(numberOfPoints,3);

  double tip = 0.05;

  /// First point (upper left)
  (*pointTable)(0,0) = - tip * height * tan(angle2Radians);
  (*pointTable)(0,1) = - tip * height;
  (*pointTable)(0,2) = 0.0;

  /// Second point (lower left)
  (*pointTable)(1,0) = - height * tan(angle2Radians);
  (*pointTable)(1,1) = -height;
  (*pointTable)(1,2) = 0.0;

  /// Third point (lower right)
  (*pointTable)(2,0) = height * tan(angle1Radians);
  (*pointTable)(2,1) = -height;
  (*pointTable)(2,2) = 0.0;

  /// Fourth point (upper right)
  (*pointTable)(3,0) = tip * height * tan(angle1Radians);
  (*pointTable)(3,1) = - tip * height;
  (*pointTable)(3,2) = 0.0;


  /// Fifth, sixth, seventh, and eight points (depth)
  for (int i = 0; i < numberOfPoints/2; i++){
      (*pointTable)(i+numberOfPoints/2,0) = (*pointTable)(i,0);
      (*pointTable)(i+numberOfPoints/2,1) = (*pointTable)(i,1);
      (*pointTable)(i+numberOfPoints/2,2) = depth;
  }


  for(int i=0; i<numberOfPoints; i++) {
      std::stringstream s;
      s << i+1;
      MBSim::Point *point = new MBSim::Point(s.str());
      fmatvec::Vec *position = new fmatvec::Vec(3,fmatvec::INIT,0.0);
      position->operator()(0) = (*pointTable)(i,0);
      position->operator()(1) = (*pointTable)(i,1);
      position->operator()(2) = (*pointTable)(i,2);
      addContour(point,*position,fmatvec::SqrMat(3,fmatvec::EYE),geometryReferenceFrame);
  }

}

#ifdef HAVE_OPENMBVCPPINTERFACE
void Wedge::enableOpenMBV(bool enable)
{
  if(enable){
      std::vector<OpenMBV::PolygonPoint*> *vecPoint = new std::vector<OpenMBV::PolygonPoint*>(4);
      double x,y,z;
      for (unsigned i=0; i < 4; i++){
          x = this->getPointTable()->operator()(i,0);
          y = this->getPointTable()->operator()(i,1);
          z = this->getPointTable()->operator()(i,2);
          vecPoint->at(i) = new OpenMBV::PolygonPoint(x,y,z);
      }
      OpenMBV::Extrusion *openMBVWedge = new OpenMBV::Extrusion();
      openMBVWedge->setHeight(depth);
      openMBVWedge->setInitialTranslation(gRefFramePosition->operator()(0),
          gRefFramePosition->operator()(1),
          gRefFramePosition->operator()(2)
      );
      openMBVWedge->setStaticColor(OpenMBV::ScalarParameter("green",0.5));
      openMBVWedge->addContour(vecPoint);
      this->setOpenMBVRigidBody(openMBVWedge);
  }
}
#endif

void Wedge::updateGrefFramePosition()
{
  double cSecArea;
  cSecArea = pow(height,2)*(tan(angle1Radians)+tan(angle2Radians))/2;
  gRefFramePosition->operator()(0) = (-pow(height,3)*pow(tan(angle1Radians),2)/6 +
      pow(height,3)*pow(tan(angle2Radians),2)/6)/cSecArea;
  gRefFramePosition->operator()(1) = 2./3 * height;
  gRefFramePosition->operator()(2) = - depth / 2;
  geometryReferenceFrame->setRelativePosition(*gRefFramePosition);
}



std::vector<MBSim::Contact*>* Wedge::contactGeneral(MBSim::Contour* sfContactContour,
    double frictionCoefficient,
    double restitutionCoefficient,
    std::vector<int> idx,
    std::string contactName)
{
  std::vector<MBSim::Contact*>* contacts = new std::vector<MBSim::Contact*>;
  int numberOfContacts = this->getContours().size() / 2;
  contacts->resize ( numberOfContacts );

  for ( int i = 0; i < numberOfContacts; i++ )
    {
      std::stringstream s;
      std::string cNumber;
      s << i+1;
      cNumber = s.str();

      contacts->at ( i ) = new MBSim::Contact (
          std::string ( contactName + "-" + cNumber ) );
      contacts->at ( i )->connect (
          this->getContours() [idx[i]],sfContactContour );
      contacts->at ( i )->setContactForceLaw ( new MBSim::UnilateralConstraint() );
      contacts->at ( i )->setContactImpactLaw (
          new MBSim::UnilateralNewtonImpact ( restitutionCoefficient ) );
      contacts->at ( i )->setFrictionImpactLaw (
          new MBSim::PlanarCoulombImpact ( frictionCoefficient ) );
      contacts->at ( i )->setFrictionForceLaw (
          new MBSim::PlanarCoulombFriction ( frictionCoefficient ) );
    }
  return contacts;
}

std::vector<MBSim::Contact*>* Wedge::contactLeft(MBSim::Contour* sfContactContour,
    double frictionCoefficient,
    double restitutionCoefficient,
    std::string contactName)
{
  std::vector<int> idx;
  idx.resize(4);
  idx[0] = 0;
  idx[1] = 1;
  idx[2] = 4;
  idx[3] = 5;
  return this->contactGeneral(sfContactContour,
      frictionCoefficient,
      restitutionCoefficient,
      idx,contactName);
}

std::vector<MBSim::Contact*>* Wedge::contactRight(MBSim::Contour* boContactContour,
    double frictionCoefficient,
    double restitutionCoefficient,
    std::string contactName)
{
  std::vector<int> idx;
  idx.resize(4);
  idx[0] = 2;
  idx[1] = 3;
  idx[2] = 6;
  idx[3] = 7;
  return this->contactGeneral(boContactContour,
      frictionCoefficient,
      restitutionCoefficient,
      idx,contactName);
}
