#include "aBehaviors.h"

#include <math.h>
#include "GL/glew.h"
#include "GL/glut.h"

// Base Behavior
///////////////////////////////////////////////////////////////////////////////
Behavior::Behavior()
{
}

Behavior::Behavior( char* name) 
{
	m_name = name;
	m_pTarget = NULL;
}

Behavior::Behavior( Behavior& orig) 
{
	m_name = orig.m_name;
	m_pTarget = NULL;
}

string& Behavior::GetName() 
{
    return m_name;
}

// Behaviors derived from Behavior
//----------------------------------------------------------------------------//
// Seek behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Seek returns a maximum velocity towards the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position


Seek::Seek( AJoint* target) 
{
	m_name = "seek";
	m_pTarget = target;

}

Seek::Seek( Seek& orig) 
{
	m_name = "seek";
	m_pTarget = orig.m_pTarget;
}


Seek::~Seek()
{
}

vec3 Seek::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	double mag = actor->gMaxSpeed;
	vec3 dir = (targetPos - actorPos).Normalize();
	Vdesired = mag * dir;

	return Vdesired;
}


// Flee behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Flee calculates a a maximum velocity away from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position

Flee::Flee( AJoint* target) 
{
	m_name = "flee";
	m_pTarget = target;
}

Flee::Flee( Flee& orig) 
{
	m_name = "flee";
	m_pTarget = orig.m_pTarget;
}

Flee::~Flee()
{
}

vec3 Flee::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	double mag = actor->gMaxSpeed;
	vec3 dir = (actorPos - targetPos).Normalize();
	Vdesired = mag * dir;



	return Vdesired;

}

// Arrival behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// the actors distance from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Arrival strength is in BehavioralController::KArrival


Arrival::Arrival( AJoint* target) 
{
	m_name = "arrival";
	m_pTarget = target;
}

Arrival::Arrival( Arrival& orig) 
{
	m_name = "arrival";
	m_pTarget = orig.m_pTarget;
}

Arrival::~Arrival()
{
}

vec3 Arrival::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	double k = actor->KArrival;
	vec3 e = targetPos - actorPos;
	Vdesired = k * e;

	return Vdesired;
}


// Departure behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// 1/(actor distance) from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Departure strength is in BehavioralController::KDeparture

Departure::Departure(AJoint* target) 
{
	m_name = "departure";
	m_pTarget = target;
}

Departure::Departure( Departure& orig) 
{
	m_name = "departure";
	m_pTarget = orig.m_pTarget;
}

Departure::~Departure()
{
}

vec3 Departure::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	double k = actor->KDeparture;
	vec3 e = targetPos - actorPos;
	Vdesired = k * ((-1.0 * e) / e.SqrLength());

	return Vdesired;
}


// Avoid behavior
///////////////////////////////////////////////////////////////////////////////
//  For the given the actor, return a desired velocity in world coordinates
//  If an actor is near an obstacle, avoid adds a normal response velocity to the 
//  the desired velocity vector computed using arrival
//  Agent bounding sphere radius is in BehavioralController::radius
//  Avoidance parameters are  BehavioralController::TAvoid and BehavioralController::KAvoid

Avoid::Avoid(AJoint* target, vector<Obstacle>* obstacles) 
{
	m_name = "avoid";
	m_pTarget = target;
	mObstacles = obstacles;
}

Avoid::Avoid( Avoid& orig) 
{
	m_name = "avoid";
	m_pTarget = orig.m_pTarget;
	mObstacles = orig.mObstacles;
}

Avoid::~Avoid()
{
}

vec3 Avoid::calcDesiredVel( BehaviorController* actor)
{

	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	m_actorPos = actor->getPosition();
	m_actorVel = actor->getVelocity();

	//TODO: add your code here
	vec3 Varrival(0, 0, 0);
	// Step 1. compute initial value for Vdesired = Varrival so agent moves toward target

	vec3 targetPos = m_pTarget->getLocalTranslation();
	double k = actor->KArrival;
	vec3 e = targetPos - m_actorPos;
	Varrival = k * e;
	Vdesired = Varrival;

	vec3 Vavoid(0, 0, 0);
	//TODO: add your code here to compute Vavoid 

	// Step 2. compute Lb
	//TODO: add your code here
	double Lb = actor->TAvoid * actor->getVelocity().Length();


	// Step 3. find closest obstacle 
	//TODO: add your code here
	Obstacle closestObs;
	double minDis = INFINITY;
	for (int i = 0; i < mObstacles->size(); i++) {
		vec3 obsDis = mObstacles->at(i).m_Center.getLocalTranslation();
		double currDis = (obsDis - m_actorPos).Length();
		if (currDis < minDis) {
			closestObs = mObstacles->at(i);
		}
	}

	// Step 4. determine whether agent will collide with closest obstacle (only consider obstacles in front of agent)
	//TODO: add your code here
	vec3 dWorld = closestObs.m_Center.getLocalTranslation() - m_actorPos;
	double theta_d_v = acos((Dot(m_actorVel, dWorld) / (m_actorVel.Length() * dWorld.Length())));
	vec3 dx = dWorld * cos(theta_d_v);
	vec3 dy = dWorld * sin(theta_d_v);
	bool willCollide = false;
	
	if (dx.Length() <= (Lb + (closestObs.m_Radius + actor->gAgentRadius))) {
		if (dy.Length() <= (closestObs.m_Radius + actor->gAgentRadius)) {
			willCollide = true;
		}
	}
	// Step 5.  if potential collision detected, compute Vavoid and set Vdesired = Varrival + Vavoid
	//TODO: add your code here
	if (willCollide == true) {
		vec3 avoidDir = -1.0 * dy / dy.Length();
		double avoidMag = (actor->KAvoid * ((closestObs.m_Radius + actor->gAgentRadius) - dy.Length())) / (closestObs.m_Radius + actor->gAgentRadius);
		Vavoid = avoidMag * avoidDir;
		Vdesired = Varrival + Vavoid;
	}

	return Vdesired;
	
}

void Avoid::display( BehaviorController* actor)
{
	//  Draw Debug info
	vec3 angle = actor->getOrientation();
	vec3 vel = actor->getVelocity();
	vec3 dir = vec3(cos(angle[1]), 0, sin(angle[1]));
	vec3 probe = dir * (vel.Length()/BehaviorController::gMaxSpeed)*BehaviorController::TAvoid;
	
	glBegin(GL_LINES);
	glColor3f(0, 0, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_obstaclePos[0], m_obstaclePos[1], m_obstaclePos[2]);
	glColor3f(0, 1, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_actorPos[0] + probe[0], m_actorPos[1] + probe[1], m_actorPos[2] + probe[2]);
	glEnd();
}


// Wander Behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Wander returns a desired velocity vector whose direction changes at randomly from frame to frame
// Wander strength is in BehavioralController::KWander

Wander::Wander() 
{
	m_name = "wander";
	m_Wander = vec3(1.0, 0.0, 0.0);
}

Wander::Wander( Wander& orig) 
{
	m_name = "wander";
	m_Wander = orig.m_Wander;
}

Wander::~Wander()
{
}

vec3 Wander::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();

	// compute Vdesired = Vwander

	// Step. 1 find a random direction
	//TODO: add your code here

	//random angle between 0-360
	float deg = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 360.0));
	deg = deg * M_PI / 180.0;
	vec3 n = vec3(cos(deg), 0.0, sin(deg));


	// Step2. scale it with a noise factor
	//TODO: add your code here
	vec3 rNoise = actor->KNoise * (n / n.Length());

	// Step3. change the current Vwander  to point to a random direction
	//TODO: add your code here
	//update m_wander
	m_Wander = actor->KWander * ((m_Wander + rNoise) / (m_Wander + rNoise).Length()); //m_velocity in wander


	// Step4. scale the new wander velocity vector and add it to the nominal velocity
	//TODO: add your code here
	//(0 0 1)
	Vdesired = vec3(0, 0, 1) + m_Wander;

	return Vdesired;
}


// Alignment behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity vector in world coordinates
// Alignment returns the average velocity of all active agents in the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Alignment parameters are in BehavioralController::RNeighborhood and BehavioralController::KAlign


Alignment::Alignment(AJoint* target, vector<AActor>* agents) 
{
	m_name = "alignment";
	m_pAgentList = agents;
	m_pTarget = target;
}



Alignment::Alignment( Alignment& orig) 
{
	m_name = orig.m_name;
	m_pAgentList = orig.m_pAgentList;
	m_pTarget = orig.m_pTarget;

}

Alignment::~Alignment()
{
}

vec3 Alignment::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_pAgentList;
	

	// compute Vdesired 
	
	// Step 1. compute value of Vdesired for fist agent (i.e. m_AgentList[0]) using an arrival behavior so it moves towards the target
	 
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
	//TODO: add your code here
	double k = leader->KArrival;
	vec3 e = targetPos - leader->getPosition();
	
	if (actor == leader) {
		return k * e;
	}

	// Step 2. if not first agent compute Valign as usual
	//TODO: add your code here
	double kAlign = actor->KAlignment;
	vec3 velSum = vec3(0.0, 0.0, 0.0);
	double weight = 1.0;
	double sumWeight = 0.0;

	for (int i = 0; i < agentList.size(); i++) {
		// check if agent is within the radius 
		if ((actorPos - agentList[i].getBehaviorController()->getPosition()).Length() <= leader->gKNeighborhood) {
			velSum += (agentList[i].getBehaviorController()->getVelocity() * weight);
			sumWeight += weight;
		}
	}
	
	if (sumWeight != 0) {
		Vdesired = kAlign * (velSum / sumWeight);
	}
	
	return Vdesired;
}

// Separation behavior
///////////////////////////////////////////////////////////////////////////////
// For the given te actor, return a desired velocity vector in world coordinates
// Separation tries to maintain a constant distance between all agents
// within the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Separation settings are in BehavioralController::RNeighborhood and BehavioralController::KSeperate

 

Separation::Separation( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "separation";
	m_AgentList = agents;
	m_pTarget = target;
}

Separation::~Separation()
{
}

Separation::Separation( Separation& orig) 
{
	m_name = "separation";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

vec3 Separation::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	
	// compute Vdesired = Vseparate
	// TODO: add your code here to compute Vdesired 
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader

	double kSeperate = actor->KSeparation;
	double weight = 1.0;
	for (int i = 0; i < agentList.size(); i++) {
		// check if agent is within the radius 
		if ((actorPos - agentList[i].getBehaviorController()->getPosition()).Length() <= leader->gKNeighborhood) {
			vec3 distance = actorPos - agentList[i].getBehaviorController()->getPosition();
			if (distance.Length() == 0) {
				continue;
			}
			vec3 d = (distance / distance.SqrLength());
			Vdesired = Vdesired + (weight * d);
		}
	}
	if (agentList.size() == 1) {
		return Vdesired;
	}
	Vdesired = kSeperate * Vdesired;


	if (Vdesired.Length() < 5.0)
		Vdesired = 0.0;
	
	return Vdesired;
}


// Cohesion behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// Cohesion moves actors towards the center of the group of agents in the neighborhood
//  agents[i] gives the pointer to the ith agent in the environment
//  Cohesion parameters are in BehavioralController::RNeighborhood and BehavioralController::KCohesion


Cohesion::Cohesion( vector<AActor>* agents) 
{
	m_name = "cohesion";
	m_AgentList = agents;
}

Cohesion::Cohesion( Cohesion& orig) 
{
	m_name = "cohesion";
	m_AgentList = orig.m_AgentList;
}

Cohesion::~Cohesion()
{
}

vec3 Cohesion::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;

	//double rn = actor->RNeighborhood;
	
	// compute Vdesired = Vcohesion
	// TODO: add your code here 
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader

	// Compute Center of Mass
	vec3 xcm = vec3(0.0, 0.0, 0.0);
	double weightSum = 0.0;
	vec3 xSum = vec3(0.0, 0.0, 0.0);
	for (int i = 0; i < agentList.size(); i++) {
		// check if agent is within the radius
		if ((actorPos - agentList[i].getBehaviorController()->getPosition()).Length() <= leader->gKNeighborhood) {
			xSum += agentList[i].getBehaviorController()->getPosition();
			weightSum += 1.0; // assume w is 1s
		}
	}
	if (weightSum != 0) {
		xcm = xSum / weightSum;
	}
	Vdesired = leader->KCohesion * (xcm - actorPos);

	return Vdesired;
}

// Flocking behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector  in world coordinates
// Flocking combines separation, cohesion, and alignment behaviors
//  Utilize the Separation, Cohesion and Alignment behaviors to determine the desired velocity vector


Flocking::Flocking( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "flocking";
	m_AgentList = agents;
	m_pTarget = target;
}

Flocking::Flocking( Flocking& orig) 
{
	m_name = "flocking";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Flocking::~Flocking()
{
}

vec3 Flocking::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	vec3 targetPos = m_pTarget->getLocalTranslation();

	// compute Vdesired = Vflocking
	// TODO: add your code here 
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader

	// Vseperate component
	vec3 vSep = vec3(0.0, 0.0, 0.0);
	double kSeperate = actor->KSeparation;
	double weight = 1.0;
	for (int i = 0; i < agentList.size(); i++) {
		// check if agent is within the radius 
		if ((actorPos - agentList[i].getBehaviorController()->getPosition()).Length() <= leader->gKNeighborhood) {
			vec3 distance = actorPos - agentList[i].getBehaviorController()->getPosition();
			if (distance.Length() == 0) {
				continue;
			}
			vec3 d = (distance / distance.SqrLength());
			vSep = vSep + (weight * d);
		}
	}
	if (agentList.size() > 1) {
		vSep = kSeperate * vSep;
	}

	if (vSep.Length() < 5.0)
		vSep = 0.0;

	//vSep = vSep.Normalize();

	//// Valign component
	vec3 vAlign = vec3(0.0, 0.0, 0.0);
	double k = leader->KArrival;
	vec3 e = targetPos - leader->getPosition();

	if (actor == leader) {
		vAlign = k * e;
	}
	else {
		double kAlign = actor->KAlignment;
		vec3 velSum = vec3(0.0, 0.0, 0.0);
		double sumWeight = 0.0;

		for (int i = 0; i < agentList.size(); i++) {
			// check if agent is within the radius 
			if ((actorPos - agentList[i].getBehaviorController()->getPosition()).Length() <= leader->gKNeighborhood) {
				velSum += (agentList[i].getBehaviorController()->getVelocity() * weight);
				sumWeight += weight;
			}
		}

		if (sumWeight != 0) {
			vAlign = kAlign * (velSum / sumWeight);
		}
	}

	//vAlign = vAlign.Normalize();

	// Vcohesion componentv
	vec3 vCoh = vec3(0.0, 0.0, 0.0);
	vec3 xcm = vec3(0.0, 0.0, 0.0);
	double weightSum = 0.0;
	vec3 xSum = vec3(0.0, 0.0, 0.0);
	for (int i = 0; i < agentList.size(); i++) {
		// check if agent is within the radius
		if ((actorPos - agentList[i].getBehaviorController()->getPosition()).Length() <= leader->gKNeighborhood) {
			xSum += agentList[i].getBehaviorController()->getPosition();
			weightSum += 1.0; // assume w is 1
		}
	}
	if (weightSum > 0) {
		xcm = xSum / weightSum;
	}
	vCoh = actor->KCohesion * (xcm - actorPos);

	//vCoh = vCoh.Normalize();

	//Vdesired = 0.4 * vSep + 1.0 * vAlign + 1.8 * vCoh;
	Vdesired = 0.8 * vSep + 1.0 * vAlign + 1.8 * vCoh;
	//Vdesired = 1.6 * vSep + 1.8 * vAlign + 2.8 * vCoh;

	return  Vdesired;
}

//	Leader behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// If the agent is the leader, move towards the target; otherwise, 
// follow the leader at a set distance behind the leader without getting to close together
//  Utilize Separation and Arrival behaviors to determine the desired velocity vector
//  You need to find the leader, who is always agents[0]

Leader::Leader( AJoint* target, vector<AActor>* agents) 
{
	m_name = "leader";
	m_AgentList = agents;
	m_pTarget = target;
}

Leader::Leader( Leader& orig) 
{
	m_name = "leader";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Leader::~Leader()
{
}

vec3 Leader::calcDesiredVel( BehaviorController* actor)
{
	
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;

	// TODO: compute Vdesired  = Vleader
	// followers should stay directly behind leader at a distance of -200 along the local z-axis

	float CSeparation = 4.0;  float CArrival = 2.0;

	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
	mat3 Rmat = leader->getGuide().getLocalRotation();  // is rotattion matrix of lead agent

	// Seperate
	vec3 vSep = vec3(0.0, 0.0, 0.0);
	double kSeperate = actor->KSeparation;
	double weight = 1.0;
	for (int i = 0; i < agentList.size(); i++) {
		// check if agent is within the radius 
		if ((actorPos - agentList[i].getBehaviorController()->getPosition()).Length() <= leader->gKNeighborhood) {
			vec3 distance = actorPos - agentList[i].getBehaviorController()->getPosition();
			if (distance.Length() == 0) {
				continue;
			}
			vec3 d = (distance / distance.SqrLength());
			vSep = vSep + (weight * d);
		}
	}
	if (agentList.size() != 1) {
		vSep = kSeperate * vSep;
	}

	if (vSep.Length() < 5.0)
		vSep = 0.0;

	// Arrival
	// if leader
	vec3 vArrival = vec3(0.0, 0.0, 0.0);
	if (actor == leader) {
		vec3 targetPos = m_pTarget->getGlobalTranslation();
		double k = actor->KArrival;
		vec3 e = targetPos - actorPos;
		vArrival = k * e;
		vSep = 0.0;
	}
	else {
		// target is -200 behind leader in z 
		vec3 targetPos = Rmat* (leader->getPosition() + vec3(0.0, 0.0, -200));
		double k = actor->KArrival;
		vec3 e = targetPos - actorPos;
		vArrival = k * e;
	}

	Vdesired = CSeparation * vSep + CArrival * vArrival;

	return Vdesired;
}

///////////////////////////////////////////////////////////////////////////////

