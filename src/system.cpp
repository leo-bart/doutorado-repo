/*
 System: instances a friction wedge 2D dynamical system
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

#include "system.h"

// next variables control the type of simulation
#define ALTERNATED_MOTION
//#define MOVING_MASS

System::System(const string& projectName, const string& inputFileName) :
    DynamicSystemSolver(projectName)
{
  /// --------------- INITIALIZATION ------------------------------------------
  if (initializeFromFile(inputFileName))
    {
      std::cout << "Error while initializing from file. File not found!"
          << std::endl;
    }

  /// --------------- PARAMETERS ----------------------------------------------

  // set the acceleration of gravity
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(Vec("[0;-9.81;0]"));

  this->getFrame("I")->enableOpenMBV();

  /// ----------------- TRUCKS ------------------------------------------------
  Vec3 position(3, INIT, 0.0);
  position(0) = -truckBaseDistance / 2;
  /*BarberTruck* frontTruck = new BarberTruck("Front truck");*/
  RideControlTruck* frontTruck = new RideControlTruck("Front truck",
      frictionCoefficient,
      truckWheelBase);
  frontTruck->setPosition(position);
  this->addGroup(frontTruck);

  frontTruck->bolster->getFrame("WCP")->enableOpenMBV();

  RideControlTruck* rearTruck = new RideControlTruck("Rear truck",
      frictionCoefficient,
      truckWheelBase);
  //BarberTruck* rearTruck = new BarberTruck("Rear truck");
  position(0) = truckBaseDistance / 2;
  rearTruck->setPosition(position);
  this->addGroup(rearTruck);

  /// ----------------- WAGON BOX ---------------------------------------------
#ifdef MOVING_MASS
  //WagonMovMass* wagon = new WagonMovMass("Wagon",mmassfraction);
  WagonSloshing* wagon = new WagonSloshing("Wagon");
  wagon->setFillRatio(fillRatio);
#else
  WagonSimple* wagon = new WagonSimple("Wagon");
#endif
  wagon->setTotalMass(wagonMass);
  wagon->getWagonBox()->setInertiaTensor(wagonInertiaTensor);
  wagon->getWagonBox()->setTranslation(
      new LinearTranslation(Mat("[1,0;0,1;0,0]")));
  wagon->getWagonBox()->setRotation(new RotationAboutZAxis());

  // inertia reference frame
  position(0) = 0.0;
  position(1) = 2.152;
  wagon->setPosition(position);
  this->addGroup(wagon);
  wagon->getWagonBox()->setFrameForKinematics(
      wagon->getWagonBox()->getFrameC());
  wagon->getWagonBox()->getFrame("FBC")->enableOpenMBV();
  wagon->getWagonBox()->getFrame("RBC")->enableOpenMBV();
  wagon->getWagonBox()->getFrame("GF")->enableOpenMBV();
  wagon->getWagonBox()->getFrame("C")->enableOpenMBV();

  // configure connection points
  wagon->setFrontBolsterConnectionPosition(-truckBaseDistance / 2, -1.2, 0.);
  wagon->setRearBolsterConnectionPosition(truckBaseDistance / 2, -1.2, 0.);

  // configure geometric representation
  wagon->setHeight(1.908);
  wagon->setWidth(3.113);
  wagon->setLength(8.747);
#ifdef MOVING_MASS
  wagon->Initialize();
#endif
  wagon->enableOpenMBV(true);

  /// ---------------- JOINTS BETWEEN BOLSTERS AND BOX -----------------------
  // Front connection plate joint
  IsotropicRotationalSpringDamper *frontPlate =
		  new IsotropicRotationalSpringDamper("Bushing: front plate");
  frontPlate->connect(wagon->getWagonBox()->getFrame("FBC"),
        frontTruck->bolster->getFrame("WCP"));
  frontPlate->setMomentDirection(Mat("[0;0;1]"));
  frontPlate->setParameters(500.e3,0.,0.);
  this->addLink(frontPlate);

  Joint *frontCylindrical = new Joint("Joint: Front plate");
  frontCylindrical->setForceDirection(Mat("[1,0;0,1;0,0]"));
  frontCylindrical->setForceLaw(new BilateralConstraint());
  frontCylindrical->setImpactForceLaw(new BilateralImpact());
  frontCylindrical->connect(wagon->getWagonBox()->getFrame("FBC"),
      frontTruck->bolster->getFrame("WCP"));
  this->addLink(frontCylindrical);

  // Rear connection plate joint
  IsotropicRotationalSpringDamper *rearPlate =
		  new IsotropicRotationalSpringDamper("Bushing: rear plate");
  rearPlate->connect(wagon->getWagonBox()->getFrame("RBC"),
		  rearTruck->bolster->getFrame("WCP"));
  rearPlate->setMomentDirection(Mat("[0;0;1]"));
  rearPlate->setParameters(500.e3,0.,0.);
  this->addLink(rearPlate);

  Joint *rearCylindrical = new Joint("Joint: Rear plate");
  rearCylindrical->setForceDirection(Mat("[1,0;0,1;0,0]"));
  rearCylindrical->setForceLaw(new BilateralConstraint());
  rearCylindrical->setImpactForceLaw(new BilateralImpact());
  rearCylindrical->connect(wagon->getWagonBox()->getFrame("RBC"),
      rearTruck->bolster->getFrame("WCP"));
  this->addLink(rearCylindrical);

  /// ----------------- WHEEL MOTIONS -----------------------------------------
  /// ------------- IMPOSED MOTION PARAMETERS ---------------------------------

#ifdef ALTERNATED_MOTION
  double forwardVelocity = 40 / 3.6; // km/h * 3.6 = m/s

  wheel1 = new SinusoidalMovement(angSpeed, amplitude, t0);
  wheel2 = new SinusoidalMovement(angSpeed, amplitude,
      t0 + truckWheelBase / forwardVelocity);
  wheel3 = new SinusoidalMovement(angSpeed, amplitude,
      t0 + (truckBaseDistance - truckWheelBase) / forwardVelocity);
  wheel4 = new SinusoidalMovement(angSpeed, amplitude,
      t0 + (truckBaseDistance + truckWheelBase) / forwardVelocity);
#else
  wheel1 = new SinusoidalMovement(angSpeed, amplitude,t0);
  wheel2 = new SinusoidalMovement(angSpeed, amplitude,t0);
  wheel3 = new SinusoidalMovement(angSpeed, amplitude,t0);
  wheel4 = new SinusoidalMovement(angSpeed, amplitude,t0);
#endif
  /// -------------- MOTION DEFINITION ----------------------------------------
  // front wheel, front truck = wheel 1
  frontTruck->wheelL->setTranslation(new SinusoidalMovement::Position(*wheel1));
  frontTruck->wheelL->setGuidingVelocityOfTranslation(
      new SinusoidalMovement::Velocity(*wheel1));
  frontTruck->wheelL->setDerivativeOfGuidingVelocityOfTranslation(
      new SinusoidalMovement::Acceleration(*wheel1));
  // rear wheel, front truck = wheel2
  frontTruck->wheelR->setTranslation(new SinusoidalMovement::Position(*wheel2));
  frontTruck->wheelR->setGuidingVelocityOfTranslation(
      new SinusoidalMovement::Velocity(*wheel2));
  frontTruck->wheelR->setDerivativeOfGuidingVelocityOfTranslation(
      new SinusoidalMovement::Acceleration(*wheel2));
  // front wheel, rear truck = wheel3
  rearTruck->wheelL->setTranslation(new SinusoidalMovement::Position(*wheel3));
  rearTruck->wheelL->setGuidingVelocityOfTranslation(
      new SinusoidalMovement::Velocity(*wheel3));
  rearTruck->wheelL->setDerivativeOfGuidingVelocityOfTranslation(
      new SinusoidalMovement::Acceleration(*wheel3));
  // rear wheel, rear truck = wheel4
  rearTruck->wheelR->setTranslation(new SinusoidalMovement::Position(*wheel4));
  rearTruck->wheelR->setGuidingVelocityOfTranslation(
      new SinusoidalMovement::Velocity(*wheel4));
  rearTruck->wheelR->setDerivativeOfGuidingVelocityOfTranslation(
      new SinusoidalMovement::Acceleration(*wheel4));

  /// ------------- INITIAL CONDITIONS --------------------------------
  Vec3 wagonOffset(INIT,0.0);
  wagonOffset ( 1 ) = 0.060;

  wagon->getWagonBox()->setInitialGeneralizedPosition(wagonOffset);
  // next, only the bolster positions have to be set because wedges are
  // place relative to it
  frontTruck->bolster->setInitialGeneralizedPosition(wagonOffset);
  rearTruck->bolster->setInitialGeneralizedPosition(wagonOffset);
}

int System::initializeFromFile(const string& inputFileName)
{
  amplitude = atof(searchParameter(inputFileName, "AMPLITUDE").c_str());

  angSpeed = atof(searchParameter(inputFileName, "ANGULAR_VELOCITY").c_str());

  t0 = atof(searchParameter(inputFileName, "MOVEMENT_DELAY").c_str());

  truckBaseDistance = atof(
      searchParameter(inputFileName, "CAR_WHEEL_BASE").c_str());

  /* TODO this implementation has no effect because wheelBase is being set
   *  internally in BarberTruck class
   */
  truckWheelBase = atof(
      searchParameter(inputFileName, "TRUCK_WHEEL_BASE").c_str());

  wagonMass = atof(searchParameter(inputFileName, "BOX_MASS").c_str());

  fillRatio = atof(searchParameter(inputFileName, "FILL_RATIO").c_str());

  wagonInertiaTensor.resize(3);
  wagonInertiaTensor.init(fmatvec::EYE);
  wagonInertiaTensor(0, 0) = atof(
      searchParameter(inputFileName, "BOX_Ixx").c_str());
  wagonInertiaTensor(1, 1) = atof(
      searchParameter(inputFileName, "BOX_Iyy").c_str());
  wagonInertiaTensor(2, 2) = atof(
      searchParameter(inputFileName, "BOX_Izz").c_str());

  frictionCoefficient = atof(
      searchParameter(inputFileName, "FRICTION_COEFF").c_str());

  //wagonType = atof(searchParameter(inputFileName, "WAGON_TYPE").c_str());

  return 0;
}
