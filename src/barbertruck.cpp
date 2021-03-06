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


#include <barbertruck.h>
#include "mbsim/observers/contact_observer.h"
#include "mbsim/observers/mechanical_link_observer.h"
#include "mbsim/functions/kinetics/linear_elastic_function.h"
#include "mbsim/links/generalized_elastic_connection.h"

BarberTruck::BarberTruck( const std::string& projectName ): Truck(projectName) {
	BarberTruck (projectName,false);
};

BarberTruck::BarberTruck ( const std::string& projectName, bool withBushings ) : Truck(projectName), wheelBase(2.5), bolsterWithBushings(withBushings)
{
	setModelType("Barber");
	/// -------------------------- SYSTEM PARAMETERS --------------------------
	/// -------------------------------------------------------------------------

	double angleBolster = ( 36.3 ) *M_PI/180.; // [rad]
	double angleSideframe = ( 1.0 ) *M_PI/180.; // [rad]
	double bolsterWidth = 0.41; // taken at the widest part [m]
	double bolsterHeight = .19; // [m]
	double bolsterLength = 2.654; // [m]
	double bolsterMass = 342. + 6500; // [kg]
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
	double sideFrameTrack = 2.197; // [m]
	fmatvec::SymMat sideFrameInertiaTensor(3,fmatvec::EYE); // [kg.m²]
	sideFrameInertiaTensor(0,0) = 2 * 4;
	sideFrameInertiaTensor(1,1) = 2 * 91;
	sideFrameInertiaTensor(2,2) = 2 * 106;
	double truckTrack = 1.575; // [m]
	double bolsterSpringStiffness = 484000; // [N/m]
	double springBedOffset = 0.143; // [m]
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
	this->addFrame ( new FixedRelativeFrame ( "L", Vec ("[0;0;0]"), SqrMat (3,EYE)));
	// "L",Vec ( "[0.;0.;0.]" ),SqrMat ( 3,EYE ) ) );
	/// ------------ Frames on the wedge ----------------------------------------
	pos ( 0 ) = -.17102 + 0.1 * tan(angleSideframe);
	pos ( 1 ) = -0.0275 + 0.1;
	pos ( 2 ) = 0.0;
	this->addFrame ( new FixedRelativeFrame ( "WL",pos,SqrMat ( 3,EYE ) ) );
	sideFrame->addFrame ( new FixedRelativeFrame ( "WL",pos,SqrMat ( 3,EYE ) ) );
	pos ( 0 ) = -pos ( 0 );
	this->addFrame ( new FixedRelativeFrame ( "WR",pos,SqrMat ( 3,EYE ) ) );
	sideFrame->addFrame ( new FixedRelativeFrame ( "WR",pos,SqrMat ( 3,EYE ) ) );
	/// ------------ Frames on the bolster --------------------------------------
	pos ( 0 ) = 0;
	pos ( 1 ) = -0.00785 + 0.1 + 0.00162;
	pos ( 2 ) = 0.0;
	this->addFrame ( new FixedRelativeFrame ( "B",pos,SqrMat ( 3,EYE ) ) );
	sideFrame->addFrame ( new FixedRelativeFrame ( "B",pos,SqrMat ( 3,EYE ) ) );
	sideFrame->addFrame ( new FixedRelativeFrame ( "BS",Vec ( "[0.0;-.18663;0.0]" ),SqrMat ( 3,EYE ),sideFrame->getFrame ( "B" ) ) );
	//
	this->addFrame ( new FixedRelativeFrame ( "M",Vec ( "[0.0;-.200;0]" ),SqrMat ( 3,EYE ),this->getFrame ( "WL" ) ) );
	this->addFrame ( new FixedRelativeFrame ( "N",Vec ( "[0.0;-.200;0]" ),SqrMat ( 3,EYE ),this->getFrame ( "WR" ) ) );
	sideFrame->addFrame ( new FixedRelativeFrame ( "M",Vec ( "[0.0;-.200;0]" ),SqrMat ( 3,EYE ),sideFrame->getFrame ( "WL" ) ) );
	sideFrame->addFrame ( new FixedRelativeFrame ( "N",Vec ( "[0.0;-.200;0]" ),SqrMat ( 3,EYE ),sideFrame->getFrame ( "WR" ) ) );
	//
	pos(0) = -wheelBase/2;
	pos(1) = -0.0;
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
	wedgeL->setTranslation ( new MBSim::LinearTranslation<VecV> (  ( "[1,0;0,1;0,0]" ) ) );
	wedgeL->setRotation ( new RotationAboutZAxis<VecV>() );
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
	wedgeR->setTranslation ( new LinearTranslation<VecV> ( "[1,0;0,1;0,0]"  ) );
	wedgeR->setRotation ( new RotationAboutZAxis<VecV>() );
	wedgeR->enableOpenMBV ( true );


	/// ---------------- DEFINITION OF THE BOLSTER ------------------------------
	// inertia properties
	bolster->setMass(bolsterMass);
	bolster->setInertiaTensor( bolsterInertiaTensor );
	// frames
	bolster->setFrameForKinematics(bolster->getFrameC());
	bolster->setFrameOfReference(getFrame("B"));
	// geometry
	bolster->setWidth(bolsterWidth);
	bolster->setHeight(bolsterHeight);
	bolster->setLength(bolsterLength);
	bolster->setAngle(angleBolster);
	bolster->setWagonConnectionPointPosition(0,bolsterHeight/2,bolsterLength/2);
	// update geometry reference frame
	bolster->setGeometryReferenceFramePosition(0.0,0.0,-bolsterLength/2);
	// display body
	bolster->enableOpenMBV(true);
	bolster->getFrameOfReference()->enableOpenMBV();
	// set the contact planes
	bolster->setContactPlanes();
	// degrees of freedom
	bolster->setTranslation ( new LinearTranslation<VecV> ( "[1,0,0;0,1,0;0,0,1]") );
	bolster->setRotation ( new RotationAboutAxesXZ<VecV>() );

	/// ------------------- DEFINITION OF THE SIDEFRAME -------------------------
	sideFrame->setMass ( sideFrameMass );
	sideFrame->setInertiaTensor( sideFrameInertiaTensor );
	sideFrame->setFrameOfReference(this->getFrameI());
	sideFrame->setFrameForKinematics ( sideFrame->getFrameC() );
	sideFrame->getFrameC()->enableOpenMBV();
	sideFrame->setTranslation( new LinearTranslation<VecV> ("[1,0;0,1;0,0]") );
	sideFrame->setRotation( new RotationAboutZAxis<VecV>() );

	// Contact plane sideframe-left wedge definition
	pos ( 0 ) = -sideFrameWidth/2;
	pos ( 1 ) = -sideFrameHeight/2;
	fmatvec::SqrMat3 rodaroda = BasicRotAIKz(-angleSideframe);
	sideFrame->addFrame(new FixedRelativeFrame("sframe_l_contact_plane",
			pos,
			rodaroda));
	Plane *sideframeL = new Plane ( "Side frame left",
			sideFrame->getFrame("sframe_l_contact_plane") );
	sideframeL->enableOpenMBV(_transparency=0.6);
	sideFrame->addContour ( sideframeL );

	// Contact plane sideframe-right wedge definition
	pos ( 0 ) = -pos ( 0 );
	sideFrame->addFrame(new FixedRelativeFrame("sframe_r_contact_plane",
			pos,
			BasicRotAKIz(-angleSideframe + M_PI)));
	Plane *sideframeR = new Plane ( "Side frame right",
			sideFrame->getFrame("sframe_r_contact_plane") );
	sideframeR->enableOpenMBV(_transparency=0.6);
	sideFrame->addContour ( sideframeR );


	pos ( 0 ) = 0.0;
	sideFrame->addFrame(new FixedRelativeFrame("sframe_central_contact_plane",
			pos,
			BasicRotAKIz(0.5 * M_PI)));
	Plane *ground = new Plane ( "Ground",
			sideFrame->getFrame("sframe_central_contact_plane") );
	ground->enableOpenMBV(_transparency=0.6);
	sideFrame->addContour ( ground );


	/// ---------------- DEFINITION OF THE WHELLS -------------------------------
	wheelL->setFrameOfReference(this->getFrame("RL"));
	wheelL->setFrameForKinematics(wheelL->getFrameC());
	wheelL->setMass(2000);
	wheelL->setTrack(truckTrack);
	wheelL->enableOpenMBV();

	wheelR->setFrameOfReference(this->getFrame("RR"));
	wheelR->setFrameForKinematics(wheelR->getFrameC());
	wheelR->setMass(2000);
	wheelR->setTrack(truckTrack);
	wheelR->enableOpenMBV();

	/// ---------------- DEFINITION OF JOINTS -----------------------------------
	//  Cylindrical joint left wheel-sideframe
	Joint* cylindrical1 = new Joint("CY1");
	cylindrical1->setForceDirection("[1,0;0,1;0,0]");
//	cylindrical1->setMomentDirection("[1;0;0]");
	cylindrical1->setForceLaw(new  BilateralConstraint());
//	cylindrical1->setMomentLaw(new BilateralConstraint());
	//cylindrical1->setImpactForceLaw(new BilateralImpact());
	cylindrical1->connect(sideFrame->getFrame("RL"),wheelL->getFrameC());
	addLink(cylindrical1);

	// Cylindrical joint right wheel-sideframe
	Joint* cylindrical2 = new Joint("CY2");
	cylindrical2->setForceDirection("[1,0;0,1;0,0]");
	cylindrical2->setForceLaw(new  BilateralConstraint());
	//cylindrical2->setImpactForceLaw(new BilateralImpact());
	cylindrical2->connect(sideFrame->getFrame("RR"),wheelR->getFrameC());
	addLink(cylindrical2);

	/// ---------------- DEFINITION OF THE SPRINGS
	///
	setSpringConnectionPointsFrames(bolster, springBedOffset, sideFrameTrack/2,bolsterHeight/2,1);
	setSpringConnectionPointsFrames(bolster, springBedOffset, -sideFrameTrack/2,bolsterHeight/2,2);
	setSpringConnectionPointsFrames(sideFrame, springBedOffset, sideFrameTrack/2,sideFrameHeight/2,1);
	setSpringConnectionPointsFrames(sideFrame, springBedOffset, -sideFrameTrack/2,sideFrameHeight/2,2);

	// Wedge spring group force law
	LinearSpringDamperForce *wedgeSpringLaw =
			new LinearSpringDamperForce(wedgeSpringStiffness,0.002 * wedgeSpringStiffness);

	// Bolster spring group force law
	LinearSpringDamperForce *bolsterSpringLaw =
			new LinearSpringDamperForce(bolsterSpringStiffness,0.002 * bolsterSpringStiffness);

	// Spring group connecting left wedge to sideframe
	SpringDamper *spring1 = new SpringDamper ( "S1" );
	spring1->setUnloadedLength(0.37);
	spring1->connect ( sideFrame->getFrame ( "M" ),wedgeL->getFrame ( "C" ) );
	spring1->setForceFunction ( wedgeSpringLaw );
	this->addLink ( spring1 );
	spring1->enableOpenMBV(_springRadius=springBedOffset / 2,
			_crossSectionRadius=0.005,
			_numberOfCoils=5);

	// Spring group connecting right wedge to sideframe
	SpringDamper *spring2 = new SpringDamper ( "S2" );
	spring2->setUnloadedLength(0.37);
	spring2->connect ( sideFrame->getFrame ( "N" ),wedgeR->getFrame ( "C" ) );
	spring2->setForceFunction ( wedgeSpringLaw );
	this->addLink ( spring2 );
	spring2->enableOpenMBV(_springRadius=springBedOffset / 2,
			_crossSectionRadius=0.005,
			_numberOfCoils=5);

	// Spring group connecting bolster to sideframe
	std::stringstream springName;
	std::stringstream frameName;
//	std::stringstream dummyName;
	for (unsigned k=0; k < 2; k++){
		for (unsigned i=0; i < 3; i++){
			for (unsigned j=0; j < 3; j++){
				springName.str("");
				frameName.str("");
				springName << "Spring-Bolster_" << i+1 << j+1 << "_" << static_cast<char> (k+65);
				frameName << "_SpringBed_" << i+1 << j+1 << "_" << static_cast<char> (65 + k);
				SpringDamper *springBolster = new SpringDamper (springName.str());
				springBolster->setForceFunction(bolsterSpringLaw);
				springBolster->setUnloadedLength(0.254);
				springBolster->enableOpenMBV(_springRadius=springBedOffset / 2,
						_crossSectionRadius=0.010,
						_numberOfCoils=5);
				springBolster->connect(
						sideFrame->getFrame(sideFrame->getName() + frameName.str()),
						bolster->getFrame(bolster->getName() + frameName.str())
				);
				// barber trucks doesn't have bolster springs under the wedges
				// therefore, springs on position 02 and 12 doesn't exist
				if ( j == 0 || j == 1) this->addLink(springBolster);
				else if ( i == 2) this->addLink(springBolster);
				springBolster->setPlotFeature("generalizedForce", enabled);
				springBolster->setPlotFeature("deflection",enabled);

				// IF ELASTIC CONNECTIONS

				if (bolsterWithBushings){
					LinearElasticFunction *stiffnessFcn = new LinearElasticFunction();
					SymMat3 stiffnessMatrix(INIT,0.0);
					stiffnessMatrix(0,0) = 40900;  	//stiffnessMatrix(0,2) = 23300;
					stiffnessMatrix(1,0) = -5250;	//stiffnessMatrix(1,2) = -5250;
					stiffnessMatrix(2,0) = 23300;	//stiffnessMatrix(2,2) = 40900;

					stiffnessFcn->setStiffnessMatrix(stiffnessMatrix);
					stiffnessFcn->setDampingMatrix(stiffnessMatrix * 0.004);

					springName << "-Bushing";
					ElasticJoint *bushingBolster =
							new ElasticJoint (springName.str());
					bushingBolster->setGeneralizedForceFunction(stiffnessFcn);
					bushingBolster->setForceDirection("[1,0,0;0,1,0;0,0,1");

					/// The elastic connection element doesn't have an unloaded
					/// length definition. Therefore, because on the spring definition
					/// the frames are offset from each other, there is always a
					/// vertical relative displacement, which causes the force to be
					/// non-zero even if there is no radial deflection. To cope with
					/// this issue, a dummy frame is created in the bolster, but
					/// on the same position that the connection point of the
					/// sideframe is, leading to zero deflection.
					FixedRelativeFrame *dummyFrame = new FixedRelativeFrame(
							bolster->getName() + frameName.str() + "-2",
							Vec3("[0;-0.24627;0]"),SqrMat3(EYE),
							bolster->getFrame(bolster->getName() + frameName.str()));
					bolster->addFrame(dummyFrame);
					bushingBolster->connect(
							sideFrame->getFrame(sideFrame->getName() + frameName.str()),
							dummyFrame);
					if ( j == 0 || j == 1) this->addLink(bushingBolster);
					else if ( i == 2) this->addLink(bushingBolster);
					bushingBolster->setPlotFeatureRecursive("generalizedForce", enabled);
					bushingBolster->setPlotFeatureRecursive("generalizedRelativePosition",enabled);
					bushingBolster->setPlotFeatureRecursive("generalizedRelativeVelocity",enabled);
				}
			}
		}
	}


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

		contacts1->at ( i ) = new Contact ( std::string ( "Contact_WedgeSideframe_Left-" ) + cNumber );
		contacts1->at ( i )->connect ( wedgeL->getContours() [idx[i]],sideframeL );
		contacts1->at ( i )->setNormalForceLaw ( new UnilateralConstraint() );
		contacts1->at ( i )->setNormalImpactLaw ( new UnilateralNewtonImpact ( coefRestitution ) );
		contacts1->at ( i )->setTangentialImpactLaw ( new PlanarCoulombImpact ( frictionCoefficient ) );
		contacts1->at ( i )->setTangentialForceLaw ( new PlanarCoulombFriction ( frictionCoefficient ) );
		this->addLink ( contacts1->at ( i ) );
		contacts1->at ( i )->setPlotFeature("generalizedForce",enabled);
		contacts1->at ( i )->setPlotFeature("generalizedRelativePosition",enabled);
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

		contacts2->at ( i ) = new Contact ( std::string ( "Contact_WedgeBolster_Left-" ) + cNumber );
		contacts2->at ( i )->connect ( wedgeL->getContours() [idx[i]],bolster->getContour("Contact plane left") );
		contacts2->at ( i )->setNormalForceLaw ( new UnilateralConstraint() );
		contacts2->at ( i )->setNormalImpactLaw ( new UnilateralNewtonImpact ( coefRestitution ) );
		contacts2->at ( i )->setTangentialImpactLaw ( new PlanarCoulombImpact ( frictionCoefficient ) );
		contacts2->at ( i )->setTangentialForceLaw ( new PlanarCoulombFriction ( frictionCoefficient ) );
		this->addLink ( contacts2->at ( i ) );
		contacts2->at ( i )->setPlotFeature("generalizedForce",enabled);
		contacts2->at ( i )->setPlotFeature("generalizedRelativePosition",enabled);

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

		contacts3->at ( i ) = new Contact ( std::string ( "Contact_WedgeBolster_Right-" ) + cNumber );
		contacts3->at ( i )->connect ( wedgeR->getContours() [idx[i]],bolster->getContour("Contact plane right") );
		contacts3->at ( i )->setNormalForceLaw ( new UnilateralConstraint() );
		contacts3->at ( i )->setNormalImpactLaw ( new UnilateralNewtonImpact ( coefRestitution ) );
		contacts3->at ( i )->setTangentialImpactLaw ( new PlanarCoulombImpact ( frictionCoefficient ) );
		contacts3->at ( i )->setTangentialForceLaw ( new PlanarCoulombFriction ( frictionCoefficient ) );
		this->addLink ( contacts3->at ( i ) );
		contacts3->at ( i )->setPlotFeature("generalizedForce",enabled);
		contacts3->at ( i )->setPlotFeature("generalizedRelativePosition",enabled);

		ContactObserver *observer = new ContactObserver("Observer-2-"+cNumber);
		observer->setContact(contacts3->at(i));
		observer->enableOpenMBVContactPoints(0.01);
		//      observer->enableOpenMBVNormalForce();
		observer->setPlotFeature("generalizedForce",enabled);
		addObserver(observer);
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

		contacts4->at ( i ) = new Contact ( std::string ( "Contact_WedgeSideframe_Right-" ) + cNumber );
		contacts4->at ( i )->connect ( wedgeR->getContours() [idx[i]],sideframeR );
		contacts4->at ( i )->setNormalForceLaw ( new UnilateralConstraint() );
		contacts4->at ( i )->setNormalImpactLaw ( new UnilateralNewtonImpact ( coefRestitution ) );
		contacts4->at ( i )->setTangentialImpactLaw ( new PlanarCoulombImpact ( frictionCoefficient ) );
		contacts4->at ( i )->setTangentialForceLaw ( new PlanarCoulombFriction ( frictionCoefficient ) );
		this->addLink ( contacts4->at ( i ) );
		contacts4->at ( i )->setPlotFeature("generalizedForce",enabled);
		contacts4->at ( i )->setPlotFeature("generalizedRelativePosition",enabled);
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

void BarberTruck::setSpringConnectionPointsFrames(MBSim::RigidBody *_body,
		double _distanceBetweenSprings,
		double _lateralOffset,
		double _height,
		int lado)
{

	fmatvec::Vec3 offsetTable(fmatvec::INIT,0);
	offsetTable(0) = -1;
	offsetTable(1) = 1;


	fmatvec::Vec3 relativePosition(fmatvec::INIT,0);

	std::stringstream frameName;

	relativePosition(2) = _lateralOffset;
	relativePosition(1) = - _height;

	frameName.str("");
	frameName << "Frame_BolsterSpringCenter_" << static_cast<char> (64 + lado);
	_body->addFrame( new FixedRelativeFrame(frameName.str(),
			relativePosition,
			fmatvec::SqrMat(3,fmatvec::EYE)));
	Frame *refFrame;
	refFrame = _body->getFrame(frameName.str());
	refFrame->enableOpenMBV(_size=0.3);

	for ( unsigned i = 0; i < 3; i++) {
		for ( unsigned j = 0; j < 3; j++){
			relativePosition.init(0);
			frameName.str("");
			frameName << _body->getName() << "_SpringBed_" << i+1 << j+1 << "_" << static_cast<char> (64 + lado);
			relativePosition(0) = _distanceBetweenSprings * offsetTable(i);
			relativePosition(2) = _distanceBetweenSprings * offsetTable(j);

			_body->addFrame(new FixedRelativeFrame(frameName.str(),
					relativePosition,
					fmatvec::SqrMat(3, fmatvec::EYE),
					refFrame));
			//_body->getFrame(frameName.str())->enableOpenMBV(_size=0.3);

		}
	}

}



