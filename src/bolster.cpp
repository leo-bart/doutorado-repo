/*
    Bolster. Implements a three-piece-truck bolster
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


#include "bolster.h"

Bolster::Bolster(const std::string& name) : RigidBody(name), angle1Radians(0.0), height(1.0), width(1.0), length(1.0)
{ 
  // Geometry reference frame initialization
  geometryReferenceFrame = new MBSim::FixedRelativeFrame("GR",fmatvec::Vec(3,fmatvec::INIT,0.0),fmatvec::SqrMat(3,fmatvec::EYE),this->getFrameC());
  this->addFrame(geometryReferenceFrame);
  
  // Wagon connection point frame initialization
  wagonConnectionPointPosition.resize(3);
  wagonConnectionPointPosition.init(0.0);
  wagonConnectionFrame = new MBSim::FixedRelativeFrame("WCP",wagonConnectionPointPosition,fmatvec::SqrMat(3,fmatvec::EYE),this->getFrame("GR"));
  this->addFrame(wagonConnectionFrame);
  
  // Initialize contact planes
  leftWedgePlane = new MBSim::Plane("Contact plane left");
  rightWedgePlane = new MBSim::Plane("Contact plane right");
  leftWedgePlane->enableOpenMBV();
  rightWedgePlane->enableOpenMBV();
}

// setAngles
void Bolster::setAngle(double ang_)
{
  angle1Radians = ang_;
}

// setHeight
void Bolster::setHeight(double height_)
{
  height = height_;
}

// setWidth
void Bolster::setWidth(double width_)
{
  width = width_;
}

// setLength
void Bolster::setLength(double length_)
{
  length = length_;
}

// setConnectionPointPosition
void Bolster::setWagonConnectionPointPosition(fmatvec::Vec pos_)
{
  // consistency check
  if (pos_.size() != 3){
    std::cout << "The input vector should have three dimensions!" << std::endl;
    std::cout << "Position vector will remain unchanged" << std::endl;
    return;
  }
  
  // change position vector
  wagonConnectionPointPosition = pos_;
  updateWagonConnectionPointPosition();
}

void Bolster::setWagonConnectionPointPosition(double x_, double y_, double z_)
{
  fmatvec::Vec r_(3,fmatvec::INIT,0.0);
  r_(0) = x_;
  r_(1) = y_;
  r_(2) = z_;
  
  setWagonConnectionPointPosition(r_);
}


// updateWagonConnectionPointPosition
void Bolster::updateWagonConnectionPointPosition()
{
  wagonConnectionFrame->setRelativePosition(wagonConnectionPointPosition);
}

// setGeometryReferenceFramePosition
void Bolster::setGeometryReferenceFramePosition(fmatvec::Vec pos_)
{
  geometryReferenceFrame->setRelativePosition(pos_);
}

void Bolster::setGeometryReferenceFramePosition(double x_, double y_, double z_)
{
  fmatvec::Vec r_(3,fmatvec::INIT,0.0);
  r_(0) = x_; r_(1) = y_; r_(2) = z_;
  setGeometryReferenceFramePosition(r_);
}

// setContactPlanes
void Bolster::setContactPlanes()
{
  // settings for the left wedge plane
  MBSim::RotationAboutZAxis rotationZ; // rotation around Z
  fmatvec::Vec rotationVector(1,fmatvec::INIT,angle1Radians - M_PI); // rotation magnitude
  fmatvec::Vec planePosition(3,fmatvec::INIT,0.0); // reference position for the plane
  planePosition(0) = -width / 2 + ( height / 2 ) * tan(angle1Radians);
  planePosition(1) = 0;
  planePosition(2) = length / 2;
  
  this->addContour(leftWedgePlane, planePosition, rotationZ(rotationVector,0.0,NULL),geometryReferenceFrame);
  
  //settings for the right wedge plane
  planePosition(0) = -planePosition(0);
  rotationVector(0) = -angle1Radians;
  
  this->addContour(rightWedgePlane,planePosition,rotationZ(rotationVector,0.0,NULL),geometryReferenceFrame);
}

// enableOpenMBV
#ifdef HAVE_OPENMBVCPPINTERFACE
  void Bolster::enableOpenMBV(bool enable)
  {
    if(enable){
      std::vector<OpenMBV::PolygonPoint*> *vecPoint = new std::vector<OpenMBV::PolygonPoint*>(4);
      double x,y,z;
      // first point: upper left
      x = -width / 2;
      y = height / 2;
      z = 0;
      vecPoint->at(0) = new OpenMBV::PolygonPoint(x,y,z);
      // second point: upper right
      x = width / 2;
      y = height / 2;
      vecPoint->at(1) = new OpenMBV::PolygonPoint(x,y,z);
      // third point: lower right
      x = width / 2 - height * tan( angle1Radians );
      y = - height / 2;
      vecPoint->at(2) = new OpenMBV::PolygonPoint(x,y,z);
      // fourth point: lower left
      x = - ( width / 2 - height * tan( angle1Radians ) );
      y = - height / 2;
      vecPoint->at(3) = new OpenMBV::PolygonPoint(x,y,z);
      
      OpenMBV::Extrusion *openMBVBolster = new OpenMBV::Extrusion();
      openMBVBolster->setStaticColor(OpenMBV::ScalarParameter("color4",1));
      openMBVBolster->setHeight(length);
      openMBVBolster->addContour(vecPoint);
      x = geometryReferenceFrame->getRelativePosition()(0);
      y = geometryReferenceFrame->getRelativePosition()(1);
      z = geometryReferenceFrame->getRelativePosition()(2);
      openMBVBolster->setInitialTranslation(x,y,z);
      this->setOpenMBVRigidBody(openMBVBolster);
    }
  }
#endif