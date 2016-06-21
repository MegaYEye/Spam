/** @file PosePlanner.cpp
 * 
 * @author	Claudio Zito
 *
 * @version 1.0
 *
 */

#include <Spam/Data/Belief/Belief.h>
#include <Golem/Tools/XMLData.h>
#include <boost/algorithm/string.hpp>

//-----------------------------------------------------------------------------

using namespace golem;
using namespace spam;
using namespace spam::data;

//-----------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* graspDescLoader() {
	// Create description
	return new HandlerBelief::Desc();
}

//------------------------------------------------------------------------------

spam::data::ItemBelief::ItemBelief (HandlerBelief& handler) :
Item(handler), handler(handler), dataFile(handler.file)
{}

grasp::data::Item::Ptr spam::data::ItemBelief::clone() const {
	throw Message(Message::LEVEL_ERROR, "ItemBelief::clone(): not implemented");
}

void spam::data::ItemBelief::createRender() {
	handler.createRender(*this);
}

void spam::data::ItemBelief::load(const std::string& prefix, const golem::XMLContext* xmlcontext) {
	// belief
	std::string beliefSuffix;
	golem::XMLData("belief", beliefSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (beliefSuffix.length() > 0) {
		dataFile.load(prefix + beliefSuffix, [&](const std::string& path) {
			FileReadStream frs(path.c_str());
			poses.clear();
			frs.read(poses, poses.end());
			hypotheses.clear();
			frs.read(hypotheses, hypotheses.end());
		});
	}
}

void spam::data::ItemBelief::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	// belief xml
	std::string beliefSuffix = !poses.empty() ? handler.beliefSuffix : "";
	golem::XMLData("ext", beliefSuffix, xmlcontext, true);

	// query binary
	if (beliefSuffix.length() > 0) {
		dataFile.save(prefix + beliefSuffix, [=](const std::string& path) {
			FileWriteStream fws(path.c_str());
			fws.write(poses.begin(), poses.end());
			fws.write(hypotheses.begin(), hypotheses.end());
		});
	}
	else
		dataFile.remove();
}

//------------------------------------------------------------------------------

Belief::Desc::Ptr spam::data::ItemBelief::getBeliefDesc() const {
	return handler.desc.pBeliefDescPtr;
}

void spam::data::ItemBelief::set(const golem::Mat34 modelFrame, const golem::Mat34 queryTransform, const grasp::RBPose::Sample::Seq& poses, const grasp::RBPose::Sample::Seq& hypotheses) {
	this->modelFrame = modelFrame;
	this->queryTransform = queryTransform;
	this->poses = poses;
	this->hypotheses = hypotheses;
}

//------------------------------------------------------------------------------

void spam::data::HandlerBelief::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	grasp::data::Handler::Desc::load(context, xmlcontext);

	golem::XMLData("ext_suffix", beliefSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("belief", false);
	XMLData(*pBeliefDescPtr.get(), pxmlcontext, false);
}

grasp::data::Handler::Ptr spam::data::HandlerBelief::Desc::create(golem::Context &context) const {
	Handler::Ptr handler(new HandlerBelief(context));
	grasp::to<HandlerBelief>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

spam::data::HandlerBelief::HandlerBelief(golem::Context &context) : Handler(context) {
}

void spam::data::HandlerBelief::create(const Desc& desc) {
	grasp::data::Handler::create(desc);

	this->desc = desc;
	beliefSuffix = desc.beliefSuffix;
}

void spam::data::HandlerBelief::createRender(const spam::data::ItemBelief& item) {

}

//------------------------------------------------------------------------------

std::string spam::data::HandlerBelief::getFileExtBelief() {
	return std::string(".hbs");
}

grasp::data::Item::Ptr spam::data::HandlerBelief::create() const {
	return grasp::data::Item::Ptr(new spam::data::ItemBelief(*const_cast<spam::data::HandlerBelief*>(this))); // safe: no modification here - only passing reference
}