/** @file Data.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */
#include <Spam/Core/JContact.h>
#include <boost/lexical_cast.hpp>

//------------------------------------------------------------------------------

using namespace golem;
using namespace spam;

//------------------------------------------------------------------------------

const std::string spam::FTGuard::ChainName[] = {
	"UNKNOWN",
	"THUMB",
	"INDEX",
	"MIDDLE",
	"RING",
	"PINKY",
};

const char* spam::FTGuard::ModeName[] = {
	"Disable",
	"Enable",
	"Contact",
};

//------------------------------------------------------------------------------

FTGuard* FTGuard::Desc::create() const {
	FTGuard* guard(new FTGuard());
	guard->create(*this);
	return guard;
}

//------------------------------------------------------------------------------

FTGuard::FTGuard() {
}

std::string FTGuard::str() const {
//	std::string ss = getGuardName(getHandChain()) + " joint=" + boost::lexical_cast<std::string>(getHandJoint()) + " measured_force=" + boost::lexical_cast<std::string>(force) + (type == FTGUARD_ABS ? " |> " : type == FTGUARD_LESSTHAN ? " < " : " > ") + boost::lexical_cast<std::string>(threshold);
	std::string ss = "FTGuard[" + ChainName[this->chain] + "]: mode = " + ModeName[mode] + " measured_force = " + strForces().c_str();
//	printf("%s\n", ss.c_str());
	return ss;
}

std::string FTGuard::strForces() const {
	//	std::string ss = getGuardName(getHandChain()) + " joint=" + boost::lexical_cast<std::string>(getHandJoint()) + " measured_force=" + boost::lexical_cast<std::string>(force) + (type == FTGUARD_ABS ? " |> " : type == FTGUARD_LESSTHAN ? " < " : " > ") + boost::lexical_cast<std::string>(threshold);
	std::string ss = "<" + boost::lexical_cast<std::string>(wrench.getV().x) + " " + boost::lexical_cast<std::string>(wrench.getV().y) + " " + boost::lexical_cast<std::string>(wrench.getV().y) + " " +
		boost::lexical_cast<std::string>(wrench.getW().x) + " " + boost::lexical_cast<std::string>(wrench.getW().y) + " " + boost::lexical_cast<std::string>(wrench.getW().z) + ">" + (type == FTGUARD_ABS ? " |> " : type == FTGUARD_LESSTHAN ? " < " : " > ") + "[" +
		boost::lexical_cast<std::string>(limits[0]) + " " + boost::lexical_cast<std::string>(limits[1]) + " " + boost::lexical_cast<std::string>(limits[2]) + " " + boost::lexical_cast<std::string>(limits[3]) + " " +
		boost::lexical_cast<std::string>(limits[4]) + " " + boost::lexical_cast<std::string>(limits[5]) + "]";
	return ss;
}


void FTGuard::create(const FTGuard::Desc& desc) {
	chain = desc.chain;
	jointIdx = desc.jointIdx;
	type = desc.type;
	mode = desc.mode;
	limits = desc.limits;
	//	printf("FTGuard(armJoints=%u, handChains=%u, fingerJoints=%u) chain=%u, chain joint=%u, joint=%u\n", armIdx, handChains, fingerJoints, (handIdx / fingerJoints) + 1, handIdx % fingerJoints, i);
}

//void XMLData(FTGuard::Desc& val, XMLContext* xmlcontext, bool create) {
//	golem::XMLData("name", val.name, xmlcontext, create);
////	golem::XMLData("joint", val.joint, xmlcontext, create);
//	golem::XMLData("type", val.type, xmlcontext, create);
////	golem::XMLData("value", val.value, xmlcontext, create);
//}

