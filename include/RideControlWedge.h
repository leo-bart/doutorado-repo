/*
 * RideControlWedge.h
 *
 *  Created on: May 19, 2016
 *      Author: leonardo
 */

#ifndef RIDECONTROLWEDGE_H_
#define RIDECONTROLWEDGE_H_

#include <wedge.h>

using namespace MBSim;
using namespace fmatvec;

class RideControlWedge: public Wedge {
public:
	RideControlWedge(const std::string &name);
	void setHeight(double h_);
private:
	FixedRelativeFrame *wedgeSpringMount;
};

#endif /* RIDECONTROLWEDGE_H_ */
