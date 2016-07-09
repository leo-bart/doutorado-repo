/*
    Ride Control truck object
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


#include "ridecontroltruck.h"

RideControlTruck::RideControlTruck ( const std::string& projectName,
    double frictionCoeff_,
    double wheelBase_) :
    Truck(projectName),
    frictionCoefficient(frictionCoeff_),
    wheelBase(wheelBase_)
{
  setModelType("Barber");
  /// ---------------------------- SYSTEM PARAMETERS --------------------------
  /// -------------------------------------------------------------------------
  /// All positions are given relative to ground

  double angleBolster = ( 37.5 ) *M_PI/180.; // [rad]
  double angleSideframe = ( 0.0 ) *M_PI/180.; // [rad]
  double bolsterWidth = 0.498; // taken at the widest part [m]
  double bolsterHeight = .152; // [m]
  double bolsterMass = 342.; // [kg]
  double bolsterEmptyHeight = 0.581; // [m] with empty wagon
  fmatvec::SymMat bolsterInertiaTensor(3,fmatvec::EYE); // [kg.m²]
  bolsterInertiaTensor(0,0) = 97;
  bolsterInertiaTensor(1,1) = 99;
  bolsterInertiaTensor(2,2) = 7;
  //double frictionCoefficient = 0.2; // [-]
  double wedgeMass = 12; // [kg] times two to represent 4 wedges per bolster
  double wedgeHeight = 0.112; // [m]
  double wedgeDepth = 0.12; // [m]
  double wedgeSpringStiffness = 344000; // [N/m]
  double wedgeSpringFreeLength = 0.1857; // [m]
  double wedgeSpringDesignLength = 0.1381; // [m]
  double wheelCenterHeight = 0.965/2.0; // [m]
  // double wheelRadius = 0.45;
  // side frame mass properties are doubled to represent both sides of the truck
  double sideFrameHeight = .495; // [m]
  double sideFrameWidth = .500; // [m]
  double sideFrameMass = 2 * 290; // [kg]
  double sideFrameSpringBedBottomHeight = 0.238; // [m]
  double sideFrameLateralDistance = 2.278; // [m]
  fmatvec::SymMat sideFrameInertiaTensor(3,fmatvec::EYE); // [kg.m²]
  sideFrameInertiaTensor(0,0) = 2 * 4;
  sideFrameInertiaTensor(1,1) = 2 * 91;
  sideFrameInertiaTensor(2,2) = 2 * 106;
  double truckTrack = 1.600; // [m]
  double bolsterSpringStiffness = 527833.0; // [N/m]
  double bolsterSpringDistance = 0.143; // [m]
  double bolsterSpringFreeLength = 0.327; // [m] - 0.267 design length
  double bolsterSpringDesignLength = 0.267;
  //   double t1,t2,t3 = 0; // temporary storage
  double coefRestitution = 0.05;  // TODO modify this parameter to be setted externally

  //acceleration of gravity
  MBSimEnvironment::getInstance()->setAccelerationOfGravity ( Vec ( "[0.;-9.810;0]" ) );

  /// ---------------------------- DEFINITION OF BODIES -----------------------
  /// -------------------------------------------------------------------------

  wedgeRearLeft = new RideControlWedge ( "Wedge Rear Left" );
  wedgeRearRight = new RideControlWedge ( "Wedge Rear Right" );
  wedgeFrontLeft = new RideControlWedge ( "Wedge Front Left" );
  wedgeFrontRight = new RideControlWedge ( "Wedge Front Right" );
  sideFrame = new RigidBody ( "Side frame" );
  bolster = new Bolster ( "Bolster" );
  wheelL = new Wheelset ( "Wheel Left ");
  wheelR = new Wheelset ( "Wheel Right" );

  this->addObject ( wedgeRearLeft );
  this->addObject ( wedgeRearRight );
  this->addObject ( wedgeFrontLeft );
  this->addObject ( wedgeFrontRight );
  this->addObject ( bolster );
  this->addObject ( sideFrame );
  this->addObject ( wheelL );
  this->addObject ( wheelR );

  /// ------------------------------ FRAMES -----------------------------------
  /// ------------ Frames on environment --------------------------------------
  Vec pos ( 3,INIT,0. );
  this->addFrame ( new FixedRelativeFrame ( "L",Vec ( "[0.;0.;0.]" ),SqrMat ( 3,EYE ) ) );
  /// ------------ Frames on the wedge ----------------------------------------
  pos ( 0 ) = -.221;
  pos ( 1 ) = 0.002;
  pos ( 2 ) = -sideFrameLateralDistance / 2;

  Vec wedgeSpringDir ( 3, INIT, 0.0 );
  wedgeSpringDir ( 1 ) = (wedgeHeight / 2 - wedgeSpringDesignLength);

  SqrMat rot180aroundY("[-1,0,0;0,1,0;0,0,-1]");

  bolster->addFrame ( new FixedRelativeFrame ( "WRL",pos,rot180aroundY ) );
  bolster->addFrame ( new FixedRelativeFrame ( "NL",pos + wedgeSpringDir,
      SqrMat ( 3,EYE ) ) );
  pos ( 0 ) = -pos ( 0 );
  bolster->addFrame ( new FixedRelativeFrame ( "WFL",pos,SqrMat ( 3,EYE ) ) );
  bolster->addFrame ( new FixedRelativeFrame ( "ML",pos + wedgeSpringDir,
      SqrMat ( 3,EYE ) ) );
  pos ( 2 ) = -pos( 2 );
  bolster->addFrame ( new FixedRelativeFrame ( "WFR",pos,SqrMat ( 3,EYE ) ) );
  bolster->addFrame ( new FixedRelativeFrame ( "NR",pos + wedgeSpringDir,
      SqrMat ( 3,EYE ) ) );
  pos ( 0 ) = -pos ( 0 );
  bolster->addFrame ( new FixedRelativeFrame ( "WRR",pos,rot180aroundY ) );
  bolster->addFrame ( new FixedRelativeFrame ( "MR",pos + wedgeSpringDir,
      SqrMat ( 3,EYE ) ) );

  /// ------------ Frames on the sideframe --------------------------------------
  pos ( 0 ) = 0;
  pos ( 1 ) = bolsterEmptyHeight;
  pos ( 2 ) = 0.0;
  this->addFrame ( new FixedRelativeFrame ( "B",pos,SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "B",pos,SqrMat ( 3,EYE ) ) );
  /// ------------ Bolster spring frames
  /// 1. LEFT SIDE
  Vec bSpring ( 3, INIT, 0.0);
  bSpring ( 1 ) = - bolsterEmptyHeight + bolsterSpringDesignLength;
  /// 1.1. Spring 1
  pos ( 0 ) = bolsterSpringDistance;
  pos ( 1 ) = sideFrameSpringBedBottomHeight;
  pos ( 2 ) = - ( sideFrameLateralDistance / 2 + bolsterSpringDistance );
  sideFrame->addFrame ( new FixedRelativeFrame ( "LBS11", pos, SqrMat ( 3,EYE ) ) );
  bolster->addFrame ( new FixedRelativeFrame ( "LBS12", pos + bSpring, SqrMat ( 3,EYE ) ) );
  /// 1.2. Spring 2
  pos ( 2 ) += bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "LBS22", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "LBS21", pos, SqrMat ( 3,EYE ) ) );
  /// 1.3. Spring 3
  pos ( 2 ) += bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "LBS32", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "LBS31", pos, SqrMat ( 3,EYE ) ) );
  /// 1.4. Spring 4
  pos ( 2 ) -= 2*bolsterSpringDistance;
  pos ( 0 ) = 0;
  bolster->addFrame ( new FixedRelativeFrame ( "LBS42", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "LBS41", pos, SqrMat ( 3,EYE ) ) );
  /// 1.5. Spring 5
  pos ( 2 ) += bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "LBS52", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "LBS51", pos, SqrMat ( 3,EYE ) ) );
  /// 1.6. Spring 6
  pos ( 2 ) += bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "LBS62", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "LBS61", pos, SqrMat ( 3,EYE ) ) );
  /// 1.7. Spring 7
  pos ( 2 ) -= 2*bolsterSpringDistance;
  pos ( 0 ) = - bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "LBS72", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "LBS71", pos, SqrMat ( 3,EYE ) ) );
  /// 1.8. Spring 8
  pos ( 2 ) += bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "LBS82", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "LBS81", pos, SqrMat ( 3,EYE ) ) );
  /// 1.9. Spring 9
  pos ( 2 ) += bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "LBS92", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "LBS91", pos, SqrMat ( 3,EYE ) ) );
  /// 2. RIGHT SIDE
  /// 2.1. Spring 1
  pos ( 0 ) = bolsterSpringDistance;
  pos ( 1 ) = sideFrameSpringBedBottomHeight;
  pos ( 2 ) =  ( sideFrameLateralDistance / 2 - bolsterSpringDistance );
  sideFrame->addFrame ( new FixedRelativeFrame ( "RBS11", pos, SqrMat ( 3,EYE ) ) );
  bolster->addFrame ( new FixedRelativeFrame ( "RBS12", pos + bSpring, SqrMat ( 3,EYE ) ) );
  /// 2.2. Spring 2
  pos ( 2 ) += bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "RBS22", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "RBS21", pos, SqrMat ( 3,EYE ) ) );
  /// 2.3. Spring 3
  pos ( 2 ) += bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "RBS32", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "RBS31", pos, SqrMat ( 3,EYE ) ) );
  /// 2.4. Spring 4
  pos ( 2 ) -= 2*bolsterSpringDistance;
  pos ( 0 ) = 0;
  bolster->addFrame ( new FixedRelativeFrame ( "RBS42", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "RBS41", pos, SqrMat ( 3,EYE ) ) );
  /// 2.5. Spring 5
  pos ( 2 ) += bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "RBS52", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "RBS51", pos, SqrMat ( 3,EYE ) ) );
  /// 2.6. Spring 6
  pos ( 2 ) += bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "RBS62", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "RBS61", pos, SqrMat ( 3,EYE ) ) );
  /// 2.7. Spring 7
  pos ( 2 ) -= 2*bolsterSpringDistance;
  pos ( 0 ) = - bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "RBS72", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "RBS71", pos, SqrMat ( 3,EYE ) ) );
  /// 2.8. Spring 8
  pos ( 2 ) += bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "RBS82", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "RBS81", pos, SqrMat ( 3,EYE ) ) );
  /// 2.9. Spring 9
  pos ( 2 ) += bolsterSpringDistance;
  bolster->addFrame ( new FixedRelativeFrame ( "RBS92", pos + bSpring, SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "RBS91", pos, SqrMat ( 3,EYE ) ) );
  // Wheelset frames
  pos(0) = -wheelBase/2;
  pos(1) = wheelCenterHeight;
  pos(2) = 0.0;
  this->addFrame(new FixedRelativeFrame("RL",pos,SqrMat(3,EYE)));
  sideFrame->addFrame(new FixedRelativeFrame("RL",pos,SqrMat(3,EYE)));
  pos(0) = wheelBase / 2;
  this->addFrame(new FixedRelativeFrame("RR",pos,SqrMat(3,EYE)));
  sideFrame->addFrame(new FixedRelativeFrame("RR",pos,SqrMat(3,EYE)));



  /// ---------------- DEFINITION OF THE WEDGES -------------------------------

  // Rear wedges
  this->setupWedge( wedgeRearLeft,
      wedgeMass,
      angleSideframe,
      angleBolster,
      wedgeHeight,
      wedgeDepth,
      bolster->getFrame("WRL"));
  this->setupWedge( wedgeRearRight,
      wedgeMass,
      angleSideframe,
      angleBolster,
      wedgeHeight,
      wedgeDepth,
      bolster->getFrame("WRR"));

  // Front wedges
  this->setupWedge( wedgeFrontLeft,
      wedgeMass,
      angleSideframe,
      angleBolster,
      wedgeHeight,
      wedgeDepth,
      bolster->getFrame("WFL"));
  this->setupWedge( wedgeFrontRight,
      wedgeMass,
      angleSideframe,
      angleBolster,
      wedgeHeight,
      wedgeDepth,
      bolster->getFrame("WFR"));


  /// ---------------- DEFINITION OF THE BOLSTER ------------------------------
  // inertia properties
  bolster->setMass(bolsterMass);
  bolster->setInertiaTensor( bolsterInertiaTensor );
  // frames
  bolster->setFrameForKinematics(bolster->getFrameC());
  bolster->setFrameOfReference(this->getFrame("B"));
  this->getFrame("B")->enableOpenMBV(true);
  // geometry
  bolster->setWidth(bolsterWidth);
  bolster->setHeight(bolsterHeight);
  bolster->setLength(sideFrameLateralDistance);
  bolster->setAngle(angleBolster);
  bolster->setWagonConnectionPointPosition(0,bolsterHeight/2,sideFrameLateralDistance/2);
  // update geometry reference frame
  bolster->setGeometryReferenceFramePosition(0.0,0.0,-sideFrameLateralDistance/2);
  // display body
  bolster->enableOpenMBV(true);
  // set the contact planes
  bolster->setContactPlanes();
  // degrees of freedom
  bolster->setTranslation ( new LinearTranslation ( Mat ( "[1,0;0,1;0,0]" ) ) );
  bolster->setRotation ( new RotationAboutZAxis() );

  /// ------------------- DEFINITION OF THE SIDEFRAME -------------------------
  sideFrame->setMass ( sideFrameMass );
  sideFrame->setInertiaTensor( sideFrameInertiaTensor );
  sideFrame->setFrameOfReference(this->getFrameI());
  sideFrame->setFrameForKinematics ( sideFrame->getFrameC() );
  // sideFrame->getFrameC()->enableOpenMBV();
  sideFrame->setTranslation( new LinearTranslation ( Mat ( "[1,0;0,1;0,0]" ) ) );
  sideFrame->setRotation( new RotationAboutZAxis() );

  // Contact plane sideframe-rear wedge definition
  Plane *sideframeR = new Plane ( "Side frame left" );
  sideframeR->enableOpenMBV();
  RotationAboutZAxis *tp;
  tp = new RotationAboutZAxis();
  Vec *rotvec;
  rotvec = new Vec ( 1,INIT,-angleSideframe );
  pos ( 0 ) = -sideFrameWidth/2;
  pos ( 1 ) = sideFrameHeight/2;
  pos ( 2 ) = -sideFrameLateralDistance / 2;
  sideFrame->addContour ( sideframeR,pos,tp->operator() ( *rotvec,0.0,NULL ),sideFrame->getFrameC() );
  delete rotvec;
  delete tp;
  tp = 0;
  rotvec=0;

  // Contact plane sideframe-front wedge definition
  Plane *sideframeF = new Plane ( "Side frame right" );
  sideframeF->enableOpenMBV();
  tp = new RotationAboutZAxis();
  rotvec = new Vec ( 1,INIT,angleSideframe+M_PI );
  pos ( 0 ) = -pos ( 0 );
  sideFrame->addContour ( sideframeF,pos,tp->operator() ( *rotvec,0.0,NULL ),sideFrame->getFrameC() );
  delete rotvec;
  rotvec=0;

  Plane *ground = new Plane ( "Ground" );
  ground->enableOpenMBV();
  rotvec = new Vec ( 1,INIT,0.5*M_PI );
  pos ( 0 ) = 0;
  pos ( 1 ) = 0.238;
  sideFrame->addContour ( ground,pos, ( *tp ) ( *rotvec,0.0,NULL ) );
  delete tp;
  tp = 0;

  /// ---------------- DEFINITION OF THE WHELLS -------------------------------
  wheelL->setFrameOfReference(this->getFrame("RL"));
  wheelL->setFrameForKinematics(wheelL->getFrameC());
  wheelL->setMass(2000);
  wheelL->setTranslation( new LinearTranslation ( Mat ( "[1,0;0,1;0,0]" ) ) );
  wheelL->setTrack(truckTrack);
  wheelL->enableOpenMBV();

  wheelR->setFrameOfReference(this->getFrame("RR"));
  wheelR->setFrameForKinematics(wheelR->getFrameC());
  wheelR->setMass(2000);
  wheelR->setTranslation( new LinearTranslation ( Mat ( "[1,0;0,1;0,0]" ) ) );
  wheelR->setTrack(truckTrack);
  wheelR->enableOpenMBV();

  /// ---------------- DEFINITION OF JOINTS -----------------------------------
  //  Cylindrical joint left wheel-sideframe
  Joint* cylindrical1 = new Joint("CY1");
  cylindrical1->setForceDirection("[1,0;0,1;0,0]");
  cylindrical1->setForceLaw(new  BilateralConstraint());
  cylindrical1->setImpactForceLaw(new BilateralImpact());
  cylindrical1->connect(sideFrame->getFrame("RL"),wheelL->getFrameC());
  addLink(cylindrical1);

  // Cylindrical joint right wheel-sideframe
  Joint* cylindrical2 = new Joint("CY2");
  cylindrical2->setForceDirection("[1,0;0,1;0,0]");
  cylindrical2->setForceLaw(new  BilateralConstraint());
  cylindrical2->setImpactForceLaw(new BilateralImpact());
  cylindrical2->connect(sideFrame->getFrame("RR"),wheelR->getFrameC());
  addLink(cylindrical2);

  /// ---------------- DEFINITION OF THE SPRINGS
  // Wedge spring group force law
  LinearSpringDamperForce *wedgeSpringLaw =
      new LinearSpringDamperForce(wedgeSpringStiffness,1.,wedgeSpringFreeLength);

  int wedgeSpringNumberOfCoils = 8;
  double wedgeSpringCrossSectionRadius = 0.0075;
  double wedgeSpringRadius = 0.015;

  // Spring group connecting rear left wedge to sideframe
  SpringDamper *spring1 = new SpringDamper ( "Spring: rear left wedge" );
  spring1->connect ( bolster->getFrame ( "NL" ),wedgeRearLeft->getFrame ( "WSM" ) );
  spring1->setForceFunction ( wedgeSpringLaw );
  this->addLink ( spring1 );
  springOpenMBVgraphics(spring1,wedgeSpringNumberOfCoils,
      wedgeSpringCrossSectionRadius,wedgeSpringRadius);

  SpringDamper *spring11 = new SpringDamper ( "Spring: rear right wedge" );
  spring11->connect ( bolster->getFrame ( "MR" ),wedgeRearRight->getFrame ( "WSM" ) );
  spring11->setForceFunction ( wedgeSpringLaw );
  this->addLink ( spring11 );
  springOpenMBVgraphics(spring11,wedgeSpringNumberOfCoils,
      wedgeSpringCrossSectionRadius,wedgeSpringRadius);

  // Spring group connecting front wedges to sideframe
  SpringDamper *spring2 = new SpringDamper ( "Spring: front left wedge" );
  spring2->connect ( bolster->getFrame ( "ML" ),wedgeFrontLeft->getFrame ( "WSM" ) );
  spring2->setForceFunction ( wedgeSpringLaw );
  this->addLink ( spring2 );
  springOpenMBVgraphics(spring2,wedgeSpringNumberOfCoils,
      wedgeSpringCrossSectionRadius,wedgeSpringRadius);

  SpringDamper *spring21 = new SpringDamper ( "Spring: front right wedge" );
  spring21->connect ( bolster->getFrame ( "NR" ),wedgeFrontRight->getFrame ( "WSM" ) );
  spring21->setForceFunction ( wedgeSpringLaw );
  this->addLink ( spring21 );
  springOpenMBVgraphics(spring21,wedgeSpringNumberOfCoils,
      wedgeSpringCrossSectionRadius,wedgeSpringRadius);


  int bolsterSpringNumberOfCoils = 6;
  double bolsterSpringCrossSectionRadius = 0.035;
  double bolsterSpringRadius = bolsterSpringDistance / 2 - 0.035;


  // Spring groups connecting bolster to sideframe
  LinearSpringDamperForce *bolsterSpringLaw =
      new LinearSpringDamperForce ( bolsterSpringStiffness,1.,
          bolsterSpringFreeLength );
  /// LEFT SIDE
  /// Spring 1
  SpringDamper *springBolsterL1 = new SpringDamper ( "Spring bolster L1" );
  springBolsterL1->connect ( sideFrame->getFrame ( "LBS11" ),bolster->getFrame ( "LBS12" ) );
  springBolsterL1->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterL1 );
  springOpenMBVgraphics(springBolsterL1,bolsterSpringNumberOfCoils,
        bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 2
  SpringDamper *springBolsterL2 = new SpringDamper ( "Spring bolster L2" );
  springBolsterL2->connect ( sideFrame->getFrame ( "LBS21" ),bolster->getFrame ( "LBS22" ) );
  springBolsterL2->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterL2 );
  springOpenMBVgraphics(springBolsterL1,bolsterSpringNumberOfCoils,
          bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 3
  SpringDamper *springBolsterL3 = new SpringDamper ( "Spring bolster L3" );
  springBolsterL3->connect ( sideFrame->getFrame ( "LBS31" ),bolster->getFrame ( "LBS32" ) );
  springBolsterL3->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterL3 );
  springOpenMBVgraphics(springBolsterL3,bolsterSpringNumberOfCoils,
            bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 4
  SpringDamper *springBolsterL4 = new SpringDamper ( "Spring bolster L4" );
  springBolsterL4->connect ( sideFrame->getFrame ( "LBS41" ),bolster->getFrame ( "LBS42" ) );
  springBolsterL4->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterL4 );
  springOpenMBVgraphics(springBolsterL4,bolsterSpringNumberOfCoils,
            bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 5
  SpringDamper *springBolsterL5 = new SpringDamper ( "Spring bolster L5" );
  springBolsterL5->connect ( sideFrame->getFrame ( "LBS51" ),bolster->getFrame ( "LBS52" ) );
  springBolsterL5->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterL5 );
  springOpenMBVgraphics(springBolsterL5,bolsterSpringNumberOfCoils,
            bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 6
  SpringDamper *springBolsterL6 = new SpringDamper ( "Spring bolster L6" );
  springBolsterL6->connect ( sideFrame->getFrame ( "LBS61" ),bolster->getFrame ( "LBS62" ) );
  springBolsterL6->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterL6 );
  springOpenMBVgraphics(springBolsterL6,bolsterSpringNumberOfCoils,
            bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 7
  SpringDamper *springBolsterL7 = new SpringDamper ( "Spring bolster L7" );
  springBolsterL7->connect ( sideFrame->getFrame ( "LBS71" ),bolster->getFrame ( "LBS72" ) );
  springBolsterL7->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterL7 );
  springOpenMBVgraphics(springBolsterL7,bolsterSpringNumberOfCoils,
            bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 8
  SpringDamper *springBolsterL8 = new SpringDamper ( "Spring bolster L8" );
  springBolsterL8->connect ( sideFrame->getFrame ( "LBS81" ),bolster->getFrame ( "LBS82" ) );
  springBolsterL8->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterL8 );
  springOpenMBVgraphics(springBolsterL8,bolsterSpringNumberOfCoils,
            bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 9
  SpringDamper *springBolsterL9 = new SpringDamper ( "Spring bolster L9" );
  springBolsterL9->connect ( sideFrame->getFrame ( "LBS91" ),bolster->getFrame ( "LBS92" ) );
  springBolsterL9->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterL9 );
  springOpenMBVgraphics(springBolsterL9,bolsterSpringNumberOfCoils,
            bolsterSpringCrossSectionRadius,bolsterSpringRadius);

  /// RIGHT SIDE
  /// Spring 1
  SpringDamper *springBolsterR1 = new SpringDamper ( "Spring bolster R1" );
  springBolsterR1->connect ( sideFrame->getFrame ( "RBS11" ),bolster->getFrame ( "RBS12" ) );
  springBolsterR1->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterR1 );
  springOpenMBVgraphics(springBolsterR1,bolsterSpringNumberOfCoils,
            bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 2
  SpringDamper *springBolsterR2 = new SpringDamper ( "Spring bolster R2" );
  springBolsterR2->connect ( sideFrame->getFrame ( "RBS21" ),bolster->getFrame ( "RBS22" ) );
  springBolsterR2->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterR2 );
  springOpenMBVgraphics(springBolsterR2,bolsterSpringNumberOfCoils,
              bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 3
  SpringDamper *springBolsterR3 = new SpringDamper ( "Spring bolster R3" );
  springBolsterR3->connect ( sideFrame->getFrame ( "RBS31" ),bolster->getFrame ( "RBS32" ) );
  springBolsterR3->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterR3 );
  springOpenMBVgraphics(springBolsterR3,bolsterSpringNumberOfCoils,
              bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 4
  SpringDamper *springBolsterR4 = new SpringDamper ( "Spring bolster R4" );
  springBolsterR4->connect ( sideFrame->getFrame ( "RBS41" ),bolster->getFrame ( "RBS42" ) );
  springBolsterR4->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterR4 );
  springOpenMBVgraphics(springBolsterR4,bolsterSpringNumberOfCoils,
              bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 5
  SpringDamper *springBolsterR5 = new SpringDamper ( "Spring bolster R5" );
  springBolsterR5->connect ( sideFrame->getFrame ( "RBS51" ),bolster->getFrame ( "RBS52" ) );
  springBolsterR5->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterR5 );
  springOpenMBVgraphics(springBolsterR5,bolsterSpringNumberOfCoils,
              bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 6
  SpringDamper *springBolsterR6 = new SpringDamper ( "Spring bolster R6" );
  springBolsterR6->connect ( sideFrame->getFrame ( "RBS61" ),bolster->getFrame ( "RBS62" ) );
  springBolsterR6->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterR6 );
  springOpenMBVgraphics(springBolsterR6,bolsterSpringNumberOfCoils,
              bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 7
  SpringDamper *springBolsterR7 = new SpringDamper ( "Spring bolster R7" );
  springBolsterR7->connect ( sideFrame->getFrame ( "RBS71" ),bolster->getFrame ( "RBS72" ) );
  springBolsterR7->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterR7 );
  springOpenMBVgraphics(springBolsterR7,bolsterSpringNumberOfCoils,
              bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 8
  SpringDamper *springBolsterR8 = new SpringDamper ( "Spring bolster R8" );
  springBolsterR8->connect ( sideFrame->getFrame ( "RBS81" ),bolster->getFrame ( "RBS82" ) );
  springBolsterR8->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterR8 );
  springOpenMBVgraphics(springBolsterR8,bolsterSpringNumberOfCoils,
              bolsterSpringCrossSectionRadius,bolsterSpringRadius);
  /// Spring 9
  SpringDamper *springBolsterR9 = new SpringDamper ( "Spring bolster R9" );
  springBolsterR9->connect ( sideFrame->getFrame ( "RBS91" ),bolster->getFrame ( "RBS92" ) );
  springBolsterR9->setForceFunction ( bolsterSpringLaw );
  this->addLink ( springBolsterR9 );
  springOpenMBVgraphics(springBolsterR9,bolsterSpringNumberOfCoils,
              bolsterSpringCrossSectionRadius,bolsterSpringRadius);

  /// ------------------------ DEFITION OF THE CONTACTS -----------------------

  /// Establish contacts

  /// Rear left wedge
  // contacts between rear left wedge and sideframe
  std::vector<Contact*>* contacts1 = new std::vector<Contact*>;
  contacts1 = wedgeRearLeft->contactRight(sideframeR,frictionCoefficient,
      coefRestitution,"Contact-RearLeftWedge-Sideframe");
  for ( unsigned i = 0; i < contacts1->size();i++) this->addLink(contacts1->at(i));

  // contacts between rear left wedge and bolster
  contacts1->clear();
  contacts1 = wedgeRearLeft->contactLeft(bolster->getContour("Contact plane left"),
      frictionCoefficient, coefRestitution,"Contact-RearLeftWedge-Bolster");
  for ( unsigned i = 0; i < contacts1->size();i++) this->addLink(contacts1->at(i));

  /// Rear right wedge
  // contacts between rear right wedge and sideframe
  contacts1->clear();
  contacts1 = wedgeRearRight->contactRight(sideframeR,frictionCoefficient,
      coefRestitution,"Contact-RearRightWedge-Sideframe");
  for ( unsigned i = 0; i < contacts1->size();i++) this->addLink(contacts1->at(i));

  // contacts between rear right wedge and bolster
  contacts1->clear();
  contacts1 = wedgeRearRight->contactLeft(bolster->getContour("Contact plane left"),
      frictionCoefficient, coefRestitution,"Contact-RearRightWedge-Bolster");
  for ( unsigned i = 0; i < contacts1->size();i++) this->addLink(contacts1->at(i));

  /// Front left wedge
  // contacts between front left wedge and sideframe
  contacts1->clear();
  contacts1 = wedgeFrontLeft->contactRight(sideframeF,frictionCoefficient,
      coefRestitution,"Contact-FrontLeftWedge-Sideframe");
  for ( unsigned i = 0; i < contacts1->size();i++) this->addLink(contacts1->at(i));

  // contacts between front left wedge and bolster
  contacts1->clear();
  contacts1 = wedgeFrontLeft->contactLeft(bolster->getContour("Contact plane right"),
      frictionCoefficient, coefRestitution,"Contact-FrontLeftWedge-Bolster");
  for ( unsigned i = 0; i < contacts1->size();i++) this->addLink(contacts1->at(i));

  /// Front right wedge
  // contacts between front right wedge and sideframe
  contacts1->clear();
  contacts1 = wedgeFrontRight->contactRight(sideframeF,frictionCoefficient,
      coefRestitution,"Contact-FrontRightWedge-Sideframe");
  for ( unsigned i = 0; i < contacts1->size();i++) this->addLink(contacts1->at(i));

  // contacts between front right wedge and bolster
  contacts1->clear();
  contacts1 = wedgeFrontRight->contactLeft(bolster->getContour("Contact plane right"),
      frictionCoefficient, coefRestitution,"Contact-FrontRightWedge-Bolster");
  for ( unsigned i = 0; i < contacts1->size();i++) this->addLink(contacts1->at(i));

}

// getWheelBase
double RideControlTruck::getWheelBase()
{
  return wheelBase;
}

// setWheelBase
void RideControlTruck::setWheelBase(double wB_)
{
  wheelBase = wB_;
}

void RideControlTruck::setupWedge ( RideControlWedge* wedge,
    double mass,
    double angleSideframe,
    double angleBolster,
    double height,
    double depth,
    Frame* refFrame)
{
  wedge->setMass ( mass );
  Vec wedgeAngles ( 2,INIT,0.0 );
  wedgeAngles ( 0 ) = angleSideframe;
  wedgeAngles ( 1 ) = angleBolster + M_PI;
  wedge->setAngles ( wedgeAngles );
  wedge->setHeight ( height );
  wedge->setDepth ( depth );
  wedge->setInertiaTensor ( SymMat ( 3,EYE ) );
  wedge->buildContour();
  wedge->setFrameOfReference ( refFrame );
  wedge->setFrameForKinematics ( wedge->getFrame ( "C" ) );
  wedge->getFrame ( "C" )->enableOpenMBV();
  wedge->setTranslation ( new LinearTranslation ( Mat ( "[1,0;0,1;0,0]" ) ) );
  wedge->setRotation ( new RotationAboutZAxis() );
  wedge->enableOpenMBV ( true );

  fmatvec::SqrMat3 pinFrameOrientation(fmatvec::EYE);
  pinFrameOrientation(0,0) = cos(angleBolster);
  pinFrameOrientation(0,1) = sin(angleBolster);
  pinFrameOrientation(1,0) = -sin(angleBolster);
  pinFrameOrientation(0,0) = cos(angleBolster);
  FixedRelativeFrame *gframe = new FixedRelativeFrame(wedge->getName() + "PinFrame",
      fmatvec::Vec3("[0;0;0]"),pinFrameOrientation);
  wedge->addFrame(gframe);
  gframe->Frame::enableOpenMBV();
}
