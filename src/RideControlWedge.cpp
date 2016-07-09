/*
 * RideControlWedge.cpp
 *
 *  Created on: May 19, 2016
 *      Author: leonardo
 */

#include "RideControlWedge.h"


RideControlWedge::RideControlWedge(const std::string &name) : Wedge(name) {
	// TODO Auto-generated constructor stub

	// adds top frame to connect spring
	Vec3 wedgeSpringMountRelativePosition ( INIT, 0.0 );
	wedgeSpringMountRelativePosition ( 1 ) = this->getHeight() / 2.0;
	wedgeSpringMount = new FixedRelativeFrame( "WSM" ,
	wedgeSpringMountRelativePosition,
	SqrMat(3,fmatvec::EYE),
	this->getFrame("C"));
	this->addFrame ( wedgeSpringMount );
	this->getFrame ( "WSM" )->enableOpenMBV();
	wedgeSpringMountRelativePosition ( 1 ) = 1.0;
	wedgeSpringMount->setRelativePosition( wedgeSpringMountRelativePosition );

}

void RideControlWedge::setHeight(double h_){
	this->Wedge::setHeight(h_);

	// updates wedge spring mount position
	Vec3 wedgeSpringMountRelativePosition ( INIT, 0.0 );
	wedgeSpringMountRelativePosition ( 1 ) = h_ / 2.0;
	this->wedgeSpringMount->setRelativePosition( wedgeSpringMountRelativePosition );

}
