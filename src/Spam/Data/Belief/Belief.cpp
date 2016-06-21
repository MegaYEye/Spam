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
Item(handler), handler(handler), dataFile(handler.file), belief(nullptr), 
modelItem(), queryItem(), showMeanPoseOnly(false), showQueryDistribution(false)
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
	golem::XMLData("ext_suffix", beliefSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	//golem::XMLData("model_item", modelItem, const_cast<golem::XMLContext*>(xmlcontext), false);
	//golem::XMLData("query_item", queryItem, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (beliefSuffix.length() > 0) {
		dataFile.load(prefix + beliefSuffix, [&](const std::string& path) {
			FileReadStream frs(path.c_str());
			frs.read(modelItem);
			frs.read(modelFrame);
			frs.read(queryItem);
			frs.read(queryTransform);
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
	golem::XMLData("ext_suffix", beliefSuffix, xmlcontext, true);

	// belief binary
	if (beliefSuffix.length() > 0) {
		dataFile.save(prefix + beliefSuffix, [=](const std::string& path) {
			FileWriteStream fws(path.c_str());
			fws.write(modelItem);
			fws.write(modelFrame);
			fws.write(queryItem);
			fws.write(queryTransform);
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
	dataFile.setModified(true);
}

void spam::data::ItemBelief::setModelPoints(const std::string modelItem, const grasp::Cloud::PointSeq& points) {
	this->modelItem = modelItem;
	this->modelPoints = points;
	dataFile.setModified(true);
}

void spam::data::ItemBelief::setQueryPoints(const std::string queryItem, const grasp::Cloud::PointSeq& points) {
	this->queryItem = queryItem;
	this->queryPoints = points;
	dataFile.setModified(true);
}
//------------------------------------------------------------------------------

void spam::data::HandlerBelief::Appearance::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	golem::XMLData("show_frame", showFrame, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("show_points", showPoints, const_cast<golem::XMLContext*>(xmlcontext), false);
	try {
		golem::XMLData("distrib_samples", samples, const_cast<golem::XMLContext*>(xmlcontext), false);
	}
	catch (const golem::Message& msg) {}
	appearance.xmlData(const_cast<golem::XMLContext*>(xmlcontext), false);
}

//------------------------------------------------------------------------------

void spam::data::HandlerBelief::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	grasp::data::Handler::Desc::load(context, xmlcontext);

	golem::XMLData("ext_suffix", beliefSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("belief", false);
	XMLData(*pBeliefDescPtr.get(), pxmlcontext, false);

	posesAppearance.load(context, xmlcontext->getContextFirst("poses_appearance", false));
	meanPoseAppearance.load(context, xmlcontext->getContextFirst("meanpose_appearance", false));
	hypothesisAppearance.load(context, xmlcontext->getContextFirst("hypothesis_appearance", false));
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

//------------------------------------------------------------------------------

std::string spam::data::HandlerBelief::getFileExtBelief() {
	return std::string(".hbs");
}

grasp::data::Item::Ptr spam::data::HandlerBelief::create() const {
	return grasp::data::Item::Ptr(new spam::data::ItemBelief(*const_cast<spam::data::HandlerBelief*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------

void spam::data::HandlerBelief::createRender(const spam::data::ItemBelief& item) {
	printf("render\n");
	grasp::UI::CriticalSectionWrapper cs(getUICallback());
	if (item.hypotheses.empty() && !item.belief)
		return;

	// draw hypothesis
	for (Hypothesis::Seq::const_iterator i = item.belief->getHypotheses().begin(); i != item.belief->getHypotheses().end(); ++i) {
		Appearance& appearance = i == item.belief->getHypotheses().begin() ? desc.meanPoseAppearance : desc.hypothesisAppearance;
		renderer.addAxes((*i)->toRBPoseSampleGF().toMat34(), appearance.appearance.frameSize);
		appearance.appearance.draw((*i)->getCloud(), renderer);
		if (item.showMeanPoseOnly) // this parameter can be control by outside
			break;
	}

	// draw query distribution as set of particles (use to debug the belief update)
	if (item.showQueryDistribution && desc.posesAppearance.showPoints) {
		const Real max = item.belief->maxWeight(true);
		//1 / (2 * (sigma*sqrt(2 * pi)))*exp(-.5*((x - mu) / sigma). ^ 2);
		const Real sigma = 0.2, norm = 1 / (2 * (sigma*Math::sqrt(2 * REAL_PI)));
		for (auto i = 0; i < item.belief->getSamples().size(); ++i) {
			if (item.belief->getSamples().size() > 10 && i % 10 != 0) continue;
			const grasp::RBPose::Sample &j = item.belief->getSamples()[i];
			renderer.addAxes(j.toMat34() * item.modelFrame, desc.posesAppearance.appearance.frameSize*item.belief->normalise(j));
			desc.posesAppearance.appearance.colourOverride = true;
			const Real weight = item.belief->normalise(j) / max;
			const Real red = norm*Math::exp(-.5*Math::sqr((weight - 1) / sigma)) * 255;
			//context.write("red = %f, U8(red)=%f\n", norm*Math::exp(-.5*Math::sqr((weight - 1) / sigma)), U8(norm*Math::exp(-.5*Math::sqr((weight - 1) / sigma))));
			const Real green = norm*Math::exp(-.5*Math::sqr((weight - .5) / sigma)) * 255;
			const Real blue = norm*Math::exp(-.5*Math::sqr((weight - .0) / sigma)) * 255;
			///*j.weight*/Math::log2(1+pBelief->normalise(j)/max)*255;
			//context.debug("weight=%f, normalised=%f, red=%u, green=%u, blue=%u\n", pBelief->normalise(j), pBelief->normalise(j) / max, U8(red), U8(green), U8(blue));
			desc.posesAppearance.appearance.colour = weight > REAL_ZERO ? RGBA(U8(red), U8(green), U8(blue), 200) : RGBA::BLACK;
			grasp::Cloud::PointSeq m;
			grasp::Cloud::transform(j.toMat34(), item.modelPoints, m);
			//						if (weight > 0.7)
			desc.posesAppearance.appearance.draw(m, renderer);
		}
	}
	// draw query distribution as set of (randomly sampled) frames
	if (item.showQueryDistribution && desc.posesAppearance.showFrame) {
		for (size_t i = 0; i < desc.posesAppearance.samples; ++i)
			renderer.addAxes(item.belief->sample().toMat34() * item.modelFrame, desc.posesAppearance.appearance.frameSize);
	}
}

void spam::data::HandlerBelief::render() const {
	if (hasRenderBlock()) return;

	renderer.render();
}

void spam::data::HandlerBelief::customRender() const {
}

//------------------------------------------------------------------------------

void spam::data::HandlerBelief::mouseHandler(int button, int state, int x, int y) {
	if (hasRenderBlock()) return;

}

void spam::data::HandlerBelief::motionHandler(int x, int y) {
}

void spam::data::HandlerBelief::keyboardHandler(int key, int x, int y) {
	if (hasRenderBlock()) return;

	
}