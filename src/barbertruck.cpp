/*
    Barber truck object
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


#include "babertruck.h"

BarberTruck::BarberTruck ( const std::string& projectName ) : Truck(projectName), wheelBase(2.5)
{
  setModelType("Barber");
  /// ---------------------------- SYSTEM PARAMETERS --------------------------
  /// -------------------------------------------------------------------------

  double angleBolster = ( 36.3 ) *M_PI/180.; // [rad]
  double angleSideframe = ( 1.0 ) *M_PI/180.; // [rad]
  double bolsterWidth = 0.41; // taken at the widest part [m]
  double bolsterHeight = .19; // [m]
  double bolsterMass = 342.; // [kg]
  fmatvec::SymMat bolsterInertiaTensor(3,fmatvec::EYE); // [kg.m²]
    bolsterInertiaTensor(0,0) = 97;
    bolsterInertiaTensor(1,1) = 99;
    bolsterInertiaTensor(2,2) = 7;
  double frictionCoefficient = 0.25; // [-]
  double wedgeMass = 2 * 12; // [kg] times two to represent 4 wedges per bolster
  double wedgeHeight = 0.2; // [m]
  double wedgeDepth = 0.10; // [m]
  double wedgeSpringStiffness = 2 * 240000; // [N/m]
  double wheelBase = 1.725;
  // double wheelRadius = 0.45;
  // side frame mass properties are doubled to represent both sides of the truck
  double sideFrameHeight = .495; // [m]
  double sideFrameWidth = .45; // [m]
  double sideFrameMass = 2 * 290; // [kg]
  fmatvec::SymMat sideFrameInertiaTensor(3,fmatvec::EYE); // [kg.m²]
    sideFrameInertiaTensor(0,0) = 2 * 4;
    sideFrameInertiaTensor(1,1) = 2 * 91;
    sideFrameInertiaTensor(2,2) = 2 * 106;
  double truckTrack = 1.575; // [m]
  double bolsterSpringStiffness = 2 * 7 * 550000; // [N/m]
//   double t1,t2,t3 = 0; // temporary storage
  double coefRestitution = 0.05;  // TODO modify this parameter to be setted externally

  //acceleration of gravity
  MBSimEnvironment::getInstance()->setAccelerationOfGravity ( Vec ( "[0.;-9.810;0]" ) );

  /// ---------------------------- DEFINITION OF BODIES -----------------------
  /// -------------------------------------------------------------------------

  wedgeL = new Wedge ( "Wedge Left" );
  wedgeR = new Wedge ( "Wedge Right" );
  sideFrame = new RigidBody ( "Side frame" );
  bolster = new Bolster ( "Bolster" );
  wheelL = new Wheelset ( "Wheel Left ");
  wheelR = new Wheelset ( "Wheel Right" );

  this->addObject ( wedgeL );
  this->addObject ( wedgeR );
  this->addObject ( bolster );
  this->addObject ( sideFrame );
  this->addObject ( wheelL );
  this->addObject ( wheelR );

  /// ------------------------------ FRAMES -----------------------------------
  /// ------------ Frames on environment --------------------------------------
  Vec pos ( 3,INIT,0. );
  this->addFrame ( new FixedRelativeFrame ( "L",Vec ( "[0.;0.;0.]" ),SqrMat ( 3,EYE ) ) );
  /// ------------ Frames on the wedge ----------------------------------------
  pos ( 0 ) = -.17102;
  pos ( 1 ) = -0.0275;
  pos ( 2 ) = 0.0;
  this->addFrame ( new FixedRelativeFrame ( "WL",pos,SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "WL",pos,SqrMat ( 3,EYE ) ) );
  pos ( 0 ) = -pos ( 0 );
  this->addFrame ( new FixedRelativeFrame ( "WR",pos,SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "WR",pos,SqrMat ( 3,EYE ) ) );
  /// ------------ Frames on the bolster --------------------------------------
  pos ( 0 ) = 0;
  pos ( 1 ) = -0.00787;
  pos ( 2 ) = 0.0;
  this->addFrame ( new FixedRelativeFrame ( "B",pos,SqrMat ( 3,EYE ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "B",pos,SqrMat ( 3,EYE ) ) );
  this->addFrame ( new FixedRelativeFrame ( "BS",Vec ( "[0.0;-.18663;0.0]" ),SqrMat ( 3,EYE ),this->getFrame ( "B" ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "BS",Vec ( "[0.0;-.18663;0.0]" ),SqrMat ( 3,EYE ),sideFrame->getFrame ( "B" ) ) );
  //
  this->addFrame ( new FixedRelativeFrame ( "M",Vec ( "[0.0;-.200;0]" ),SqrMat ( 3,EYE ),this->getFrame ( "WL" ) ) );
  this->addFrame ( new FixedRelativeFrame ( "N",Vec ( "[0.0;-.200;0]" ),SqrMat ( 3,EYE ),this->getFrame ( "WR" ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "M",Vec ( "[0.0;-.200;0]" ),SqrMat ( 3,EYE ),sideFrame->getFrame ( "WL" ) ) );
  sideFrame->addFrame ( new FixedRelativeFrame ( "N",Vec ( "[0.0;-.200;0]" ),SqrMat ( 3,EYE ),sideFrame->getFrame ( "WR" ) ) );
  //
  pos(0) = -wheelBase/2;
  pos(1) = -0.1;
  this->addFrame(new FixedRelativeFrame("RL",pos,SqrMat(3,EYE)));
  sideFrame->addFrame(new FixedRelativeFrame("RL",pos,SqrMat(3,EYE)));
  pos(0) = wheelBase / 2;
  this->addFrame(new FixedRelativeFrame("RR",pos,SqrMat(3,EYE)));
  sideFrame->addFrame(new FixedRelativeFrame("RR",pos,SqrMat(3,EYE)));
  
					       
					       
  /// ---------------- DEFINITION OF THE WEDGES -------------------------------
  
  // Left wedge
  wedgeL->setMass ( wedgeMass );
  Vec wedgeAngles ( 2,INIT,0.0 );
  wedgeAngles ( 0 ) = angleBolster + M_PI;
  wedgeAngles ( 1 ) = angleSideframe;
  wedgeL->setAngles ( wedgeAngles );
  wedgeL->setHeight ( wedgeHeight );
  wedgeL->setDepth ( wedgeDepth );
  wedgeL->setInertiaTensor ( SymMat ( 3,EYE ) );
  wedgeL->buildContour();
  wedgeL->setFrameOfReference ( getFrame ( "WL" ) );
  wedgeL->setFrameForKinematics ( wedgeL->getFrame ( "C" ) );
  wedgeL->getFrame ( "C" )->enableOpenMBV();
  wedgeL->setTranslation ( new LinearTranslation ( Mat ( "[1,0;0,1;0,0]" ) ) );
  wedgeL->setRotation ( new RotationAboutZAxis() );
  wedgeL->enableOpenMBV ( true );

  // Right wedge
  wedgeR->setMass ( wedgeMass );
  wedgeAngles ( 0 ) = angleSideframe;
  wedgeAngles ( 1 ) = angleBolster + M_PI;
  wedgeR->setAngles ( wedgeAngles );
  wedgeR->setHeight ( wedgeHeight );
  wedgeR->setDepth ( wedgeDepth );
  wedgeR->setInertiaTensor ( SymMat ( 3,EYE ) );
  wedgeR->buildContour();
  wedgeR->setFrameOfReference ( getFrame ( "WR" ) );
  wedgeR->setFrameForKinematics ( wedgeR->getFrame ( "C" ) );
  wedgeR->getFrame ( "C" )->enableOpenMBV();
  wedgeR->setTranslation ( new LinearTranslation ( Mat ( "[1,0;0,1;0,0]" ) ) );
  wedgeR->setRotation ( new RotationAboutZAxis() );
  wedgeR->enableOpenMBV ( true );


  /// ---------------- DEFINITION OF THE BOLSTER ------------------------------
  // inertia properties
  bolster->setMass(bolsterMass);
  bolster->setInertiaTensor( bolsterInertiaTensor );
  // frames
  bolster->setFrameForKinematics(bolster->getFrameC());
  bolster->setFrameOfReference(this->getFrame("B"));
  // geometry
  bolster->setWidth(bolsterWidth);
  bolster->setHeight(bolsterHeight);
  bolster->setLength(truckTrack);
  bolster->setAngle(angleBolster);
  bolster->setWagonConnectionPointPosition(0,bolsterHeight/2,truckTrack/2);
  // update geometry reference frame
  bolster->setGeometryReferenceFramePosition(0.0,0.0,-truckTrack/2);
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
  
  // Contact plane sideframe-left wedge definition
  Plane *sideframeL = new Plane ( "Side frame left" );
  sideframeL->enableOpenMBV();
  RotationAboutZAxis *tp;
  tp = new RotationAboutZAxis();
  Vec *rotvec;
  rotvec = new Vec ( 1,INIT,-angleSideframe );
  pos ( 0 ) = -sideFrameWidth/2;
  pos ( 1 ) = -sideFrameHeight/2;
  sideFrame->addContour ( sideframeL,pos,tp->operator() ( *rotvec,0.0,NULL ),sideFrame->getFrameC() );
  delete rotvec;
  delete tp;
  tp = 0;
  rotvec=0;

  // Contact plane sideframe-right wedge definition
  Plane *sideframeR = new Plane ( "Side frame right" );
  sideframeR->enableOpenMBV();
  tp = new RotationAboutZAxis();
  rotvec = new Vec ( 1,INIT,angleSideframe+M_PI );
  pos ( 0 ) = -pos ( 0 );
  sideFrame->addContour ( sideframeR,pos,tp->operator() ( *rotvec,0.0,NULL ),sideFrame->getFrameC() );
  delete rotvec;
  rotvec=0;

  Plane *ground = new Plane ( "Ground" );
  ground->enableOpenMBV();
  rotvec = new Vec ( 1,INIT,0.5*M_PI );
  pos ( 0 ) = 0;
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
  LinearSpringDamperForce *wedgeSpringLaw = new LinearSpringDamperForce(wedgeSpringStiffness,1.,0.22);
  
  // Spring group connecting left wedge to sideframe
  SpringDamper *spring1 = new SpringDamper ( "S1" );
  spring1->connect ( sideFrame->getFrame ( "M" ),wedgeL->getFrame ( "C" ) );
  spring1->setForceFunction ( wedgeSpringLaw );
  this->addLink ( spring1 );
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::CoilSpring *openMBVSpring1 = new OpenMBV::CoilSpring();
  openMBVSpring1->setCrossSectionRadius ( .005 );
  openMBVSpring1->setNumberOfCoils ( 5 );
  openMBVSpring1->setSpringRadius ( 0.01 );
  spring1->setOpenMBVSpring ( openMBVSpring1 );
#endif

  // Spring group connecting right wedge to sideframe
  SpringDamper *spring2 = new SpringDamper ( "S2" );
  spring2->connect ( sideFrame->getFrame ( "N" ),wedgeR->getFrame ( "C" ) );
  spring2->setForceFunction ( wedgeSpringLaw );
  this->addLink ( spring2 );
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::CoilSpring *openMBVSpring2 = new OpenMBV::CoilSpring();
  openMBVSpring2->setCrossSectionRadius ( .005 );
  openMBVSpring2->setNumberOfCoils ( 5 );
  openMBVSpring2->setSpringRadius ( 0.01 );
  spring2->setOpenMBVSpring ( openMBVSpring2 );
#endif

  // Spring group connecting bolster to sideframe
  // TODO include side groups also. Only the central one is represented
  SpringDamper *springBolster = new SpringDamper ( "Spring bolster" );
  springBolster->connect ( sideFrame->getFrame ( "BS" ),bolster->getFrame ( "C" ) );
  springBolster->setForceFunction ( new LinearSpringDamperForce ( bolsterSpringStiffness,1.,.20 ) );
  this->addLink ( springBolster );
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::CoilSpring *openMBVSpringBolster = new OpenMBV::CoilSpring();
  openMBVSpringBolster->setCrossSectionRadius ( .005 );
  openMBVSpringBolster->setNumberOfCoils ( 5 );
  openMBVSpringBolster->setSpringRadius ( 0.01 );
  springBolster->setOpenMBVSpring ( openMBVSpringBolster );
#endif
  IsotropicRotationalSpringDamper *rotSpringBolster =
  		  new IsotropicRotationalSpringDamper("RSPRING: bolster");
    rotSpringBolster->connect( sideFrame->getFrame ( "BS" ),bolster->getFrame ( "C" ) );
    rotSpringBolster->setMomentDirection(Mat("[0;0;1]"));
    rotSpringBolster->setParameters(500.e3,0.,0.);
    this->addLink(rotSpringBolster);

  /// ------------------------ DEFITION OF THE CONTACTS -----------------------

  // Establish contacts

  // contacts between left wedge and sideframe
  std::vector<Contact*>* contacts1 = new std::vector<Contact*>;
  int numberOfContacts = wedgeL->getContours().size() / 2;
  contacts1->resize ( numberOfContacts );
  std::vector<int> idx(4,0);
  idx[0] = 0;
  idx[1] = 1;
  idx[2] = 4;
  idx[3] = 5;
  
  for ( int i = 0; i < numberOfContacts; i++ )
    {
      stringstream s;
      string cNumber;
      s << i+1;
      cNumber = s.str();

      contacts1->at ( i ) = new Contact ( std::string ( "WedgeSideframe_Left-" ) + cNumber );
      contacts1->at ( i )->connect ( wedgeL->getContours() [idx[i]],sideframeL );
      contacts1->at ( i )->setContactForceLaw ( new UnilateralConstraint() );
      contacts1->at ( i )->setContactImpactLaw ( new UnilateralNewtonImpact ( coefRestitution ) );
      contacts1->at ( i )->setFrictionImpactLaw ( new PlanarCoulombImpact ( frictionCoefficient ) );
      contacts1->at ( i )->setFrictionForceLaw ( new PlanarCoulombFriction ( frictionCoefficient ) );
//     contacts1->at(i)->enableOpenMBVContactPoints();
      this->addLink ( contacts1->at ( i ) );
    }

  // contacts between left wedge and bolster  
  std::vector<Contact*>* contacts2 = new std::vector<Contact*>;
  contacts2->resize ( numberOfContacts );
  idx[0] = 2;
  idx[1] = 3;
  idx[2] = 6;
  idx[3] = 7;

  for ( int i = 0; i < numberOfContacts; i++ )
    {
      stringstream s;
      string cNumber;
      s << i+1;
      cNumber = s.str();

      contacts2->at ( i ) = new Contact ( std::string ( "WedgeBolster_Left-" ) + cNumber );
      contacts2->at ( i )->connect ( wedgeL->getContours() [idx[i]],bolster->getContour("Contact plane left") );
      contacts2->at ( i )->setContactForceLaw ( new UnilateralConstraint() );
      contacts2->at ( i )->setContactImpactLaw ( new UnilateralNewtonImpact ( coefRestitution ) );
      contacts2->at ( i )->setFrictionImpactLaw ( new PlanarCoulombImpact ( frictionCoefficient ) );
      contacts2->at ( i )->setFrictionForceLaw ( new PlanarCoulombFriction ( frictionCoefficient ) );
//     contacts2->at(i)->enableOpenMBVContactPoints();
      this->addLink ( contacts2->at ( i ) );
    }

  // contacts between right wedge and bolster  
  std::vector<Contact*>* contacts3 = new std::vector<Contact*>;
  contacts3->resize ( numberOfContacts );
  idx[0] = 0;
  idx[1] = 1;
  idx[2] = 4;
  idx[3] = 5;

  for ( int i = 0; i < numberOfContacts; i++ )
    {
      stringstream s;
      string cNumber;
      s << i+1;
      cNumber = s.str();

      contacts3->at ( i ) = new Contact ( std::string ( "WedgeBolster_Right-" ) + cNumber );
      contacts3->at ( i )->connect ( wedgeR->getContours() [idx[i]],bolster->getContour("Contact plane right") );
      contacts3->at ( i )->setContactForceLaw ( new UnilateralConstraint() );
      contacts3->at ( i )->setContactImpactLaw ( new UnilateralNewtonImpact ( coefRestitution ) );
      contacts3->at ( i )->setFrictionImpactLaw ( new PlanarCoulombImpact ( frictionCoefficient ) );
      contacts3->at ( i )->setFrictionForceLaw ( new PlanarCoulombFriction ( frictionCoefficient ) );
//     contacts3->at(i)->enableOpenMBVContactPoints();
      this->addLink ( contacts3->at ( i ) );
    }

  // contacts between right wedge and sideframe  
  std::vector<Contact*>* contacts4 = new std::vector<Contact*>;
  contacts4->resize ( numberOfContacts );
  idx[0] = 2;
  idx[1] = 3;
  idx[2] = 6;
  idx[3] = 7;

  for ( int i = 0; i < numberOfContacts; i++ )
    {
      stringstream s;
      string cNumber;
      s << i+1;
      cNumber = s.str();

      contacts4->at ( i ) = new Contact ( std::string ( "WedgeSideframe_Right-" ) + cNumber );
      contacts4->at ( i )->connect ( wedgeR->getContours() [idx[i]],sideframeR );
      contacts4->at ( i )->setContactForceLaw ( new UnilateralConstraint() );
      contacts4->at ( i )->setContactImpactLaw ( new UnilateralNewtonImpact ( coefRestitution ) );
      contacts4->at ( i )->setFrictionImpactLaw ( new PlanarCoulombImpact ( frictionCoefficient ) );
      contacts4->at ( i )->setFrictionForceLaw ( new PlanarCoulombFriction ( frictionCoefficient ) );
//     contacts4->at(i)->enableOpenMBVContactPoints();
      this->addLink ( contacts4->at ( i ) );
    }
}

// getWheelBase
double BarberTruck::getWheelBase()
{
  return wheelBase;
}

// setWheelBase
void BarberTruck::setWheelBase(double wB_)
{
  wheelBase = wB_;
}



