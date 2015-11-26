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

static std::string getGuardName(const U32 finger) {
	if (finger == HandChains::UNKNOWN) return "UNKNOWN";
	if (finger == HandChains::THUMB) return "THUMB";
	if (finger == HandChains::INDEX) return "INDEX";
	if (finger == HandChains::MIDDLE) return "MIDDLE";
	if (finger == HandChains::RING) return "RING";
	if (finger == HandChains::PINKY) return "PINKY";
	return "";
}

static HandChains bound2Chain(const U32 finger) {
	if (7 < finger && finger < 10)
		return HandChains::THUMB;
	if (11 < finger && finger < 14)
		return HandChains::INDEX;
	if (15 < finger && finger < 18)
		return HandChains::MIDDLE;
	if (19 < finger && finger < 22)
		return HandChains::RING;
	if (22 < finger && finger < 25)
		return HandChains::PINKY;
	return HandChains::UNKNOWN;

}

//------------------------------------------------------------------------------

FTGuard::FTGuard(const grasp::Manipulator& manipulator) {
	// arm joint indices [0, 6]
	armIdx = manipulator.getArmInfo().getJoints().end()-1;
	handChains = (U32)manipulator.getHandInfo().getChains().size();
	fingerJoints = U32(manipulator.getHandInfo().getJoints().size() / handChains);
	wrenchThr.resize(6);
//	manipulator.getContext().write("FTGuard armJoints=%u, handChains=%u\n", armIdx, handChains);
}

std::string FTGuard::str() const {
	std::string ss = getGuardName(getHandChain()) + " joint=" + boost::lexical_cast<std::string>(getHandJoint()) + " measured_force=" + boost::lexical_cast<std::string>(force) + (type == FTGUARD_ABS ? " |> " : type == FTGUARD_LESSTHAN ? " < " : " > ") + boost::lexical_cast<std::string>(threshold);
	printf("%s\n", ss.c_str());
	return ss;
}

void FTGuard::create(const golem::Configspace::Index& i) {
	jointIdx = i;
	const size_t handIdx = i - armIdx;
	threshold = 0.2;
	force = REAL_ZERO;
	type = FTGUARD_ABS;
	wrenchThr[0] = Real(0.01);
	wrenchThr[1] = Real(0.01);
	wrenchThr[2] = Real(0.01);
	wrenchThr[3] = Real(0.01);
	wrenchThr[4] = Real(0.01);
	wrenchThr[5] = Real(0.01);
	//	printf("FTGuard(armJoints=%u, handChains=%u, fingerJoints=%u) chain=%u, chain joint=%u, joint=%u\n", armIdx, handChains, fingerJoints, (handIdx / fingerJoints) + 1, handIdx % fingerJoints, i);
}

void XMLData(FTGuard &val, XMLContext* xmlcontext, bool create) {
	golem::XMLData("name", val.name, xmlcontext, create);
//	golem::XMLData("joint", val.joint, xmlcontext, create);
//	golem::XMLData("type", val.type, xmlcontext, create);
//	golem::XMLData("value", val.value, xmlcontext, create);
}

