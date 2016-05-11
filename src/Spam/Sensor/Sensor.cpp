/** @file Data.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */
#include <Spam/Sensor/Sensor.h>
#include <boost/lexical_cast.hpp>
#include <iomanip>
#include <algorithm>
#include <boost/tokenizer.hpp>


//------------------------------------------------------------------------------

using namespace golem;
using namespace spam;

//------------------------------------------------------------------------------

void SensorBundle::Reader::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {

}

SensorBundle::Reader::Reader(golem::Context& context) : context(context) {}

void SensorBundle::Reader::create(const SensorBundle::Reader::Desc& desc) {
	defaultForceReader = [&](grasp::RealSeq& force) {
		return force;
		/*for (auto i = 0; i < force.size(); ++i)
			force[i] = collisionPtr->getFTBaseSensor().ftMedian[i] + (2 * rand.nextUniform<Real>()*collisionPtr->getFTBaseSensor().ftStd[i] - collisionPtr->getFTBaseSensor().ftStd[i]);

		golem::Controller::State dflt = manipulator->getController()->createState();
		manipulator->getController()->setToDefault(dflt);

		manipulator->getController()->lookupState(golem::SEC_TM_REAL_MAX, dflt);
		dflt.cvel.setToDefault(manipulator->getController()->getStateInfo().getJoints());
		dflt.cacc.setToDefault(manipulator->getController()->getStateInfo().getJoints());
		(void)collisionPtr->simulateFT(debugRenderer, desc.objCollisionDescPtr->flannDesc, rand, manipulator->getConfig(dflt), force, false);*/
	};
	// set the reader
	forceReader = desc.forceReader ? desc.forceReader : defaultForceReader;
}

grasp::RealSeq SensorBundle::Reader::read(grasp::RealSeq& force) {
	grasp::RealSeq output;
	output.assign(force.size(), REAL_ZERO);
	{
		golem::CriticalSectionWrapper csw(cs);
		output = forceReader(force);
	}
	return output;
}

//------------------------------------------------------------------------------

void SensorBundle::Filter::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
}

void SensorBundle::Filter::create(const SensorBundle::Filter::Desc& desc) {
	dimensionality = desc.dimensionality;

	steps = desc.steps;
	windowSize = desc.windowSize;
	grasp::RealSeq v;
	v.assign(windowSize + 1, REAL_ZERO);
	delta = desc.delta;
	sigma = desc.sigma;
	Real i = -2;
	for (I32 j = 0; j < windowSize + 1; j++) {
		v[j] = N(i, sigma);
		i += delta;
	}

	// compute the derivative mask=diff(v)
	mask.assign(windowSize, REAL_ZERO);
	for (I32 i = 0; i < windowSize; ++i)
		mask[i] = v[i + 1] - v[i];
//	filteredForce.assign(dimensions(), REAL_ZERO);
	// initialise table to memorise read forces size: dimensions() x windowSize
	forceInpSensorSeq.resize(dimensions());
	for (std::vector<grasp::RealSeq>::iterator i = forceInpSensorSeq.begin(); i != forceInpSensorSeq.end(); ++i)
		i->assign(windowSize, REAL_ZERO);

	defaultForceFilter = [&]() -> grasp::RealSeq {
		// find index as for circular buffer
		auto findIndex = [&](const I32 idx) -> I32 {
			return (I32)((steps + idx) % windowSize);
		};
		// high pass filter implementation
		auto hpFilter = [&]() {
			Real b = 1 / windowSize;

		};
		// gaussian filter
		auto conv = [&]() -> grasp::RealSeq {
			grasp::RealSeq output;
			output.assign(dimensions(), REAL_ZERO);
			{
				golem::CriticalSectionWrapper csw(csInp);
				for (I32 i = 0; i < dimensions(); ++i) {
					Real tmp = REAL_ZERO;
					grasp::RealSeq& seq = forceInpSensorSeq[i];
					// conv y[k] = x[k]h[0] + x[k-1]h[1] + ... + x[0]h[k]
					for (I32 j = 0; j < windowSize; ++j) {
						tmp += seq[findIndex(windowSize - j - 1)] * mask[j];
					}
					output[i] = tmp;
				}
			}
			return output;
		};
		grasp::RealSeq filteredforce;
		filteredforce.assign(dimensions(), REAL_ZERO);
		filteredforce = conv();
		return filteredforce;
	};

	// set the filter
	forceFilter = desc.forceFilter ? desc.forceFilter : defaultForceFilter;
}

grasp::RealSeq SensorBundle::Filter::filter(const grasp::RealSeq& force) {
	{
		golem::CriticalSectionWrapper csw(csInp);
		for (I32 i = 0; i < dimensions(); ++i)
			forceInpSensorSeq[i][steps] = force[i];
	}
	steps = (I32)(++steps % windowSize);
	grasp::RealSeq filteredforce;
	filteredforce.assign(dimensions(), REAL_ZERO);
	filteredforce = forceFilter();
	return filteredforce;
}

//------------------------------------------------------------------------------

SensorBundle::SensorBundle(Controller& ctrl, grasp::Manipulator& manipulator) : controller(&ctrl), context(ctrl.getContext()), rand(ctrl.getContext().getRandSeed()) {
	this->manipulator.reset(&manipulator);
};

void SensorBundle::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	xmlcontext = xmlcontext->getContextFirst("sensor_bundle");
	readerDescPtr->load(context, xmlcontext->getContextFirst("reader"));
	filterDescPtr->load(context, xmlcontext->getContextFirst("filter"));
	spam::XMLData(*objCollisionDescPtr, xmlcontext->getContextFirst("collision"));
}

void SensorBundle::create(const Desc& desc) {
	//setForceReaderHandler([=]() {
	//	RealSeq force;
	//	force.assign(dimensions(), REAL_ZERO);
	//	size_t k = 0;
	//	for (auto i = ftSensorSeq.begin(); i < ftSensorSeq.end(); ++i) {
	//		FT::Data data;
	//		(*i)->read(data);
	//	
	//		data.wrench.v.getColumn3(&force[k]);
	//		data.wrench.w.getColumn3(&force[k+3]);
	//		k += 6;
	//	}
	//	collectInp(force);
	//});

	collisionPtr.reset();
	collisionPtr = desc.objCollisionDescPtr->create(*this->manipulator.get());

	reader = desc.readerDescPtr->create(context);
	reader->setForceReader([&](grasp::RealSeq& force) -> grasp::RealSeq {
		for (auto i = 0; i < force.size(); ++i)
		force[i] = collisionPtr->getFTBaseSensor().ftMedian[i] + (2 * rand.nextUniform<Real>()*collisionPtr->getFTBaseSensor().ftStd[i] - collisionPtr->getFTBaseSensor().ftStd[i]);

		golem::Controller::State dflt = manipulator->getController().createState();
		manipulator->getController().setToDefault(dflt);

		manipulator->getController().lookupState(golem::SEC_TM_REAL_MAX, dflt);
		dflt.cvel.setToDefault(manipulator->getController().getStateInfo().getJoints());
		dflt.cacc.setToDefault(manipulator->getController().getStateInfo().getJoints());
		(void)collisionPtr->simulateFT(debugRenderer, desc.objCollisionDescPtr->flannDesc, rand, manipulator->getConfig(dflt), force, false);

		return force;
	});
	forceMode = ForceMode::FORCE_MODE_JOINT;
	mode = ForceMode::FORCE_MODE_DISABLED;
	//reader->setForceReader([&](grasp::RealSeq& force) -> grasp::RealSeq {
	//	size_t k = 0;
	//	for (auto i = ftSensorSeq.begin(); i < ftSensorSeq.end(); ++i) {
	//		grasp::FT::Data data;
	//		(*i)->read(data);
	//		data.wrench.v.getColumn3(&force[k]);
	//		data.wrench.w.getColumn3(&force[k + 3]);
	//		k += 6;
	//	}
	//});


	filter = desc.filterDescPtr->create(context);


	// Thread parameters
	bTerminate = false;
	tCycle = desc.tCycle;
	tIdle = desc.tIdle;
	tRead = context.getTimer().elapsed();

	this->desc = desc;
	// user create
	userCreate();

	// launch thread
	if (!thread.start(this))
		throw golem::Message(Message::LEVEL_CRIT, "SensorBundle::create(): Unable to launch ft thread");
	if (!thread.setPriority(desc.threadPriority))
		throw golem::Message(Message::LEVEL_CRIT, "SensorBundle::create(): Unable to change SensorBundle thread priority");
}

void SensorBundle::run() {
	//if (ftSensorSeq.empty())
	//	return;
	auto strTorques = [=](std::ostream& ostr, const grasp::RealSeq& forces, const SecTmReal t) {
		ostr << t << "\t";
		for (auto i = 0; i < forces.size(); ++i)
			ostr << forces[i] << "\t";
		ostr << std::endl;
	};

	sleep.reset(new golem::Sleep);

	while (!bTerminate) {
		SecTmReal t = context.getTimer().elapsed();
		if (t - tRead > tCycle) {
			grasp::RealSeq force;
			{
				golem::CriticalSectionWrapper csw(cs);
				force = filteredForce;
			}
			// read raw forces
			if (mode != ForceMode::FORCE_MODE_DISABLED) 
				reader->read(force);
			
			// gaussian filter
			grasp::RealSeq output;
			output.assign(filter->dimensions(), REAL_ZERO);
			if (mode != ForceMode::FORCE_MODE_DISABLED)
				output = filter->filter(force);
			
			{
				golem::CriticalSectionWrapper csw(cs);
				filteredForce = output;
			}
			tRead = t;
		}
		else
			sleep->sleep(tIdle);
	}
}

void SensorBundle::setSensorSeq(const grasp::Sensor::Seq& sensorSeq) {
	for (auto i = sensorSeq.begin(); i != sensorSeq.end(); ++i) {
		if (i == sensorSeq.begin()) continue;
		grasp::FT* sensor = grasp::is<grasp::FT>(*i);
		if (sensor)
			ftSensorSeq.push_back(sensor);
		else
			context.write("FT sensor is not available\n");
	}

}

void SensorBundle::release() {
}
