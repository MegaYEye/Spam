/** @file ShapePlanner.cpp
 * 
 * @author	Claudio Zito
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/Phys/Data.h>
#include <Golem/Plan/Data.h>
#include <Spam/ShapePlanner/ShapePlanner.h>
#include <sstream>
#include <iomanip>

using namespace golem;
using namespace spam;

//------------------------------------------------------------------------------

void spam::ShapePlanner::Data::xmlData(golem::XMLContext* context, bool create) const {
	PosePlanner::Data::xmlData(context, create);

	try {
		if (!create || !graspConfigs.empty()) {
			{
				const std::string name("grasp_config");
				create ? xmlDataSave(context->getContextFirst(name.c_str(), create), grasp::makeString("%s%s%s%s", getName().c_str(), sepName.c_str(), name.c_str(), extGraspConfig.c_str()), graspConfigs) : xmlDataLoad(context->getContextFirst(name.c_str(), create), this->path, const_cast<grasp::Grasp::Config::Seq&>(graspConfigs));
			}
			{
				const std::string name("grasp_cluster");
				create ? xmlDataSave(context->getContextFirst(name.c_str(), create), grasp::makeString("%s%s%s%s", getName().c_str(), sepName.c_str(), name.c_str(), extGraspCluster.c_str()), graspClusters) : xmlDataLoad(context->getContextFirst(name.c_str(), create), this->path, const_cast<grasp::Cluster::Seq&>(graspClusters));
			}
		}
	}
	catch (const golem::MsgXMLParser& msg) {
		if (create)
			throw msg;
	}
}

grasp::Director::Data::Ptr spam::ShapePlanner::createData() const {
	return Data::Ptr(new Data(*grasp::to<Data>(data))); // assuming data has been initialised properly
}

void spam::ShapePlanner::Data::setGraspData(const grasp::Grasp::Map& map, grasp::Grasp::Data::Map::const_iterator data) {
	golem::U32 n = 0;

	// search for index
	for (grasp::Grasp::Map::const_iterator i = map.begin(); i != map.end(); ++i)
		for (grasp::Grasp::Data::Map::const_iterator j = i->second->getDataMap().begin(); j != i->second->getDataMap().end(); ++j, ++n)
			if (j == data) {
				graspDataPtr = n;
				return;
			}

	throw Message(Message::LEVEL_ERROR, "ShapePlanner::Data::setGraspData(): unable to find grasp data pointer");
}

void spam::ShapePlanner::Data::printGraspInfo(const grasp::Manipulator& manipulator) const {
	if (!hasGraspConfigs()) {
		manipulator.getContext().write("No grasp configs available\n");
		return;
	}

	const grasp::Grasp::Config& graspConfig = **getGraspConfig();
	const grasp::Manipulator::Pose& graspPose = graspConfig.path.getGrip();
	
	std::stringstream str;
	for (size_t i = 0; i < manipulator.getJoints(); ++i)
		str << " c" << (i + 1) << "=\"" << graspPose.jc[i] << "\"";

	//manipulator.getContext().write("<pose v1=\"%.7f\" v2=\"%.7f\" v3=\"%.7f\" q0=\"%.7f\" q1=\"%.7f\" q2=\"%.7f\" q3=\"%.7f\" dim=\"%d\"%s/>\n", frame.p.x, frame.p.y, frame.p.z, frame.q.q0, frame.q.q1, frame.q.q2, frame.q.q3, robot->getStateInfo().getJoints().size(), str.str().c_str());
	manipulator.getContext().write("Grasp config: type=%s, cluster=%d/%d, config=%d/%d, likelihood_{value=%f, active=%f, inactive=%f, contacts=%f, config=%f, collision=%f}, valid=%s\n",
		graspConfig.type.c_str(),
		graspClusterPtr + 1, (golem::U32)graspClusters.size(), graspConfigPtr + 1, (graspClusters[graspClusterPtr].end - graspClusters[graspClusterPtr].begin),
		graspConfig.likelihood.value, graspConfig.likelihood.valueActive, graspConfig.likelihood.valueInactive, graspConfig.likelihood.getActiveContactsAverageValue(), graspConfig.likelihood.config, graspConfig.likelihood.collision,
		graspConfig.getGrasp() ? graspConfig.likelihood.isValid(graspConfig.getGrasp()->getDesc().optimisationDesc->epsilon) ? "yes" : "no" : "unknown");
}

//------------------------------------------------------------------------------

const char* spam::ShapePlanner::ModeName[MODE_SIZE + 1] = {"Disabled", "Training data", "Cluster", "Config", "Config Model", "Contact Model",};

spam::ShapePlanner::ShapePlanner(Scene &scene) : PosePlanner(scene) {
}

bool spam::ShapePlanner::create(const Desc& desc) {
	PosePlanner::create(desc); // throws
	
	manipulator = desc.manipulatorDesc->create(*robot->getPlanner());
	clusterDesc = desc.clusterDesc;

	demoMap = desc.demoMap;

	extGraspClass = desc.extGraspClass;

	grasp::to<Data>(data)->appearanceData = desc.appearanceData;
	grasp::to<Data>(data)->appearanceConfig = desc.appearanceConfig;

	classifierDescMap = desc.classifierDescMap;
	classifier = load(*classifierDescMap.begin()->second);

	resetDataPointers();

	scene.getHelp().insert(Scene::StrMapVal("090", "  G                                       grasp operations\n"));
	scene.getHelp().insert(Scene::StrMapVal("091", "  7                                       grasp mode\n"));
	scene.getHelp().insert(Scene::StrMapVal("091", "  8                                       grasp training features\n"));
	scene.getHelp().insert(Scene::StrMapVal("091", "  9                                       grasp training path\n"));
	scene.getHelp().insert(Scene::StrMapVal("092", "  <mouse_wheel> <mouse_middle_button>     grasp path subspace\n"));
	scene.getHelp().insert(Scene::StrMapVal("092", "  ()                                      grasp mode index\n"));

	return true;
}

void spam::ShapePlanner::render() {
	PosePlanner::render();
	
	golem::CriticalSectionWrapper csw(csDataRenderer);
	graspRenderer.render();
}

void spam::ShapePlanner::mouseHandler(int button, int state, int x, int y) {
	grasp::Player::mouseHandler(button, state, x, y);

	if (state != 1 || !(button == 1 || button == 3 || button == 4)) return;

	grasp::Configuration::Path::Appearance* appearance =
		grasp::to<Data>(currentDataPtr)->graspMode == MODE_DATA ? & grasp::to<Data>(currentDataPtr)->appearanceData.path :
		grasp::to<Data>(currentDataPtr)->graspMode == MODE_CLUSTER || grasp::to<Data>(currentDataPtr)->graspMode == MODE_CONFIG || grasp::to<Data>(currentDataPtr)->graspMode == MODE_CONFIG_MODEL || grasp::to<Data>(currentDataPtr)->graspMode == MODE_CONTACT_MODEL ? &grasp::to<Data>(currentDataPtr)->appearanceConfig.configPath :
		nullptr;

	if (!appearance) return;

	appearance->subspaceDist = button == 3 ? appearance->subspaceDist + 1 : button == 4 ? appearance->subspaceDist - 1 : 0;

	golem::CriticalSectionWrapper csw(csDataRenderer);
	// data pointer
	switch (grasp::to<Data>(currentDataPtr)->graspMode) {
	case MODE_DATA:
	{
		grasp::Grasp::Map::const_iterator grasp;
		grasp::Grasp::Data::Map::const_iterator data;
		if (grasp::to<Data>(currentDataPtr)->getGraspData(classifier->getGraspMap(), grasp, data)) {
			graspRenderer.reset();
			grasp::to<Data>(currentDataPtr)->appearanceData.path.pathDelta = grasp->second->getConfiguration()->getDesc().distanceStdDev;
			data->second.draw(*grasp->second, grasp::to<Data>(currentDataPtr)->appearanceData, graspRenderer);
			context.write("Training data path: size=%u, dist_{first=%f, last=%f, curr=%f} dist_range_{lo=%f, hi=%f}\n",
				data->second.path.size(),
				data->second.path.front().getDistance(), data->second.path.back().getDistance(), grasp::to<Data>(currentDataPtr)->appearanceData.path.subspaceDistVal,
				grasp::to<Data>(currentDataPtr)->appearanceData.path.subspaceDistLo, grasp::to<Data>(currentDataPtr)->appearanceData.path.subspaceDistHi);
		}
		break;
	}
	case MODE_CLUSTER:
	case MODE_CONFIG:
	case MODE_CONFIG_MODEL:
	case MODE_CONTACT_MODEL:
		if (grasp::to<Data>(currentDataPtr)->hasGraspConfigs()) {
			graspRenderer.reset();
			grasp::Grasp::Config::Seq::const_iterator ptr = grasp::to<Data>(currentDataPtr)->getGraspConfig();
			
			if ((*ptr)->getGrasp() && (grasp::to<Data>(currentDataPtr)->graspMode == MODE_CLUSTER || grasp::to<Data>(currentDataPtr)->graspMode == MODE_CONFIG))
				grasp::to<Data>(currentDataPtr)->appearanceConfig.configPath.pathDelta = (*ptr)->getGrasp()->getConfiguration()->getDesc().distanceStdDev;
			
			(*ptr)->draw(*manipulator, grasp::to<Data>(currentDataPtr)->appearanceConfig, graspRenderer);
			
			if ((*ptr)->getGrasp() && (grasp::to<Data>(currentDataPtr)->graspMode == MODE_CLUSTER || grasp::to<Data>(currentDataPtr)->graspMode == MODE_CONFIG))
				context.write("Grasp config path: size=%u, dist_{first=%f, last=%f, curr=%f} dist_range_{lo=%f, hi=%f}\n",
					(*ptr)->path.size(),
					(*ptr)->path.front().getDistance(), (*ptr)->path.back().getDistance(), grasp::to<Data>(currentDataPtr)->appearanceConfig.configPath.subspaceDistVal,
					grasp::to<Data>(currentDataPtr)->appearanceConfig.configPath.subspaceDistLo, grasp::to<Data>(currentDataPtr)->appearanceConfig.configPath.subspaceDistHi);
		}
		else
			context.write("Grasp configs not available!\n");
		break;
	};
}

//------------------------------------------------------------------------------

void ShapePlanner::renderData(Data::Map::const_iterator dataPtr) {
	PosePlanner::renderData(dataPtr);

	golem::CriticalSectionWrapper csw(csDataRenderer);
	graspRenderer.reset();

	// data pointer
	switch (grasp::to<Data>(dataPtr)->graspMode) {
	case MODE_DATA:
	{
		grasp::Grasp::Map::const_iterator grasp;
		grasp::Grasp::Data::Map::const_iterator data;
		if (grasp::to<Data>(dataPtr)->getGraspData(classifier->getGraspMap(), grasp, data)) {
			grasp::to<Data>(currentDataPtr)->appearanceData.path.pathDelta = grasp->second->getConfiguration()->getDesc().distanceStdDev;
			data->second.draw(*grasp->second, grasp::to<Data>(dataPtr)->appearanceData, graspRenderer);
			context.write("Training data: estimator=%s, type=%s, name=%s\n", grasp->second->getName().c_str(), grasp->first.c_str(), data->first.c_str());
		}
		else
			context.write("Training data not available!\n");
		break;
	}
	case MODE_CLUSTER:
	case MODE_CONFIG:
	case MODE_CONFIG_MODEL:
	case MODE_CONTACT_MODEL:
		if (grasp::to<Data>(dataPtr)->hasGraspConfigs()) {
			grasp::Grasp::Config::Seq::const_iterator ptr = grasp::to<Data>(dataPtr)->getGraspConfig();
			
			grasp::to<Data>(dataPtr)->appearanceConfig.showConfig = false;
			grasp::to<Data>(dataPtr)->appearanceConfig.showModelConfig = false;
			grasp::to<Data>(dataPtr)->appearanceConfig.showModelContact = false;
			switch (grasp::to<Data>(dataPtr)->graspMode) {
			case MODE_CLUSTER:
			case MODE_CONFIG:
				grasp::to<Data>(dataPtr)->appearanceConfig.showConfig = true;
				grasp::to<Data>(dataPtr)->printGraspInfo(*manipulator);
				break;
			case MODE_CONFIG_MODEL:
				grasp::to<Data>(dataPtr)->appearanceConfig.showModelConfig = true;
				break;
			case MODE_CONTACT_MODEL:
				grasp::to<Data>(dataPtr)->appearanceConfig.showModelContact = true;
				const grasp::Grasp* grasp = (*ptr)->getGrasp();
				if (grasp) {
					U32& ptr = grasp::to<Data>(dataPtr)->appearanceConfig.modelSelectionIndex;
					while (ptr < grasp->getContacts().size() && grasp->getContacts()[ptr]->getPoses().empty()) ++ptr;
					if (ptr > grasp->getContacts().size()) ptr = grasp->getContacts().size();
					const grasp::Contact* contact = ptr < grasp->getContacts().size() ? grasp->getContacts()[ptr] : nullptr;
					if (contact)
						context.write("Contact: type=%s, name=%s, size_{model=%u, query=%u}\n", grasp->getType().c_str(), contact->getTypeDesc().getName().c_str(), contact->getFeatures().size(), contact->getPoses().size());
					else
						context.write("Contact: disabled\n");
				}
				break;
			};
			
			(*ptr)->draw(*manipulator, grasp::to<Data>(dataPtr)->appearanceConfig, graspRenderer);
		}
		else
			context.write("Grasp configs not available!\n");
		break;
	};
}

void spam::ShapePlanner::resetDataPointers() {
	PosePlanner::resetDataPointers();
}

//------------------------------------------------------------------------------

grasp::Classifier::Ptr spam::ShapePlanner::load(const grasp::Classifier::Desc& desc) {
	// create classifier
	grasp::Classifier::Ptr classifier(desc.create(*manipulator));

	// load data
	const std::string path = grasp::makeString("%s%s", desc.name.c_str(), extGraspClass.c_str());
	try {
		FileReadStream(path.c_str()) >> (golem::Serializable&)*classifier;
		context.info("ShapePlanner::load(): Classifier data loaded from: %s\n", path.c_str());
	}
	catch (const std::exception&) {
		context.warning("ShapePlanner::load(): Unable to load classifier data from: %s\n", path.c_str());
	}

	return classifier;
}

void spam::ShapePlanner::run(const Demo::Map::value_type& demo) {
#ifdef _GRASP_CAMERA_DEPTH
	auto breakPoint = [&] () { if (waitKey(10) == 27) throw Cancel("Demo cancelled"); };

	context.write("================================================================================================================================\nRunning demo %s\n", demo.first.c_str());
	
	{
	grasp::ScopeGuard cleanup([&]() {
		grasp::to<Data>(currentDataPtr)->graspMode = MODE_DISABLED;
		renderData(currentDataPtr);
	});

	grasp::Cloud::RawPointSeqVal val;
	val.first = grasp::to<Data>(currentDataPtr)->labelMap.empty() ? grasp::Cloud::LABEL_OBJECT : grasp::to<Data>(currentDataPtr)->labelMap.rbegin()->first + 1;
	const grasp::Cloud::RawPointSeqMultiMap::iterator points = grasp::to<Data>(currentDataPtr)->insertPoints(val, demo.first);
	grasp::Cloud::RawPointSeq prev, next;
	grasp::Cloud::RawPointSeqQueue rawPointSeqQueue(cloudDesc.filterDesc.window);

	// capture depth image
	for (Demo::ObjectDetection::Seq::const_iterator j = demo.second.objectDetectionSeq.begin(); j != demo.second.objectDetectionSeq.end(); ++j) {
		// go to object detection pose
		gotoPose(j->scanPose);
		breakPoint();
		grasp::RobotPose robotPose;
		getPose(robotPose);

		// find camera
		grasp::Recorder* recorder = nullptr;
		grasp::DepthCamera* camera = nullptr;
		for (grasp::Recorder::Seq::iterator i = cameraSeq.begin(); i != cameraSeq.end(); ++i)
			if ((*i)->getCamera()->getName() == j->cameraName) {
				recorder = i->get();
				camera = recorder->getDepthCamera();
				break;
			}
		if (!camera)
			throw Message(Message::LEVEL_ERROR, "ShapePlanner::run(): unknown camera %s", j->cameraName.c_str());
		// set camera frame
		robotPose.multiply(camera->getCalibration()->getDeformation(robotPose), robotPose);
		Mat34 cameraFrame = camera->getCalibration()->getParameters().pose;
		if (recorder->isHand()) cameraFrame.multiply(robotPose, cameraFrame);
		camera->setFrame(cameraFrame);
		// Object region in global coordinates
		golem::Bounds::Seq objectRegion;
		for (golem::Bounds::Desc::Seq::const_iterator i = cloudDesc.objectRegionDesc.begin(); i != cloudDesc.objectRegionDesc.end(); ++i)
			objectRegion.push_back((*i)->create());
		// Move bounds to camera frame
		Mat34 cameraFrameInv;
		cameraFrameInv.setInverse(cameraFrame);
		for (golem::Bounds::Seq::const_iterator i = objectRegion.begin(); i != objectRegion.end(); ++i)
			(*i)->multiplyPose(cameraFrameInv, (*i)->getPose());
		// start camera
		grasp::ScopeGuard guardCamera([&]() { camera->set(grasp::Camera::CMD_STOP); });
		camera->setProperty(camera->getDepthProperty());
		camera->set(grasp::Camera::CMD_VIDEO);
		// detect changes
		grasp::Image::Ptr image;
		rawPointSeqQueue.clear();
		prev.clear();
		U32 prevSize = 0;
		for (bool accept = false; !accept;) {
			breakPoint();
			// grab image
			camera->peek_back(image);
			grasp::Image::assertImageData(image->dataDepth);
			try {
				grasp::Cloud::copy(objectRegion, false, *image->dataDepth, next);
			}
			catch (const Message&) {}
			grasp::Cloud::transform(cameraFrame, next, next);
			grasp::Cloud::setSensorFrame(cameraFrame, next);
			rawPointSeqQueue.push_back(next);
			if (!rawPointSeqQueue.full())
				continue;
			grasp::Cloud::filter(context, cloudDesc.filterDesc, rawPointSeqQueue, next);
			// render points
			{
				golem::CriticalSectionWrapper csw(csDataRenderer);
				pointRenderer.reset();
				grasp::to<Data>(currentDataPtr)->draw(next, pointRenderer);
			}
			// count points
			U32 nextSize = 0;
			for (grasp::Cloud::RawPointSeq::const_iterator i = next.begin(); i != next.end(); ++i)
				if (!grasp::Cloud::isNanXYZ(*i))
					++nextSize;
			if (nextSize < j->minSize)
				continue;
			//context.debug("ShapePlanner::run(): object detected (size=%u)\n", nextSize);
			if (prev.size() == next.size()) {
				size_t sharedSize = 0;
				Real deltaDepth = REAL_ZERO;
				for (size_t i = 0; i < next.size(); ++i) {
					const grasp::Cloud::RawPoint p = prev[i], n = next[i];
					if (grasp::Cloud::isNanXYZ(p) || grasp::Cloud::isNanXYZ(n))
						continue;
					++sharedSize;
					deltaDepth += Math::sqrt(Math::sqr(p.x - n.x) + Math::sqr(p.y - n.y) + Math::sqr(p.z - n.z));
				}
				if (nextSize - sharedSize < j->deltaSize && deltaDepth/sharedSize < j->deltaDepth) {
					context.write("Object detected (size=%u, delta_size=%u, delta_depth=%f)\n", nextSize, nextSize - sharedSize, deltaDepth/sharedSize);
					grasp::Cloud::nanRem(context, next, grasp::Cloud::isNanXYZ<grasp::Cloud::RawPoint>); // required by flann in linux
					grasp::Cloud::normal(context, cloudDesc.normal, next, next);
					grasp::Cloud::nanRem(context, next, grasp::Cloud::isNanXYZNormal<grasp::Cloud::RawPoint>);
					points->second.insert(points->second.end(), next.begin(), next.end());
					accept = true;
					// render
					renderData(currentDataPtr);
				}
			}
			if (!accept) {
				prev = next;
				prevSize = nextSize;
			}
		}
	}

	breakPoint();

	// find approach trajectory
	golem::Real err = REAL_MAX;
	golem::Controller::State::Seq approach;
	if (demo.second.poseEstimation) {
		// load model
		grasp::Cloud::RawPointSeq modelPoints;
		Data::pcdRead(demo.second.modelObject, demo.second.modelObject, modelPoints);
		pRBPose->createModel(modelPoints);
		// estimate pose
		breakPoint();
		pRBPose->createQuery(points->second);
		const Mat34 queryTransform = pRBPose->maximum().toMat34();
		grasp::Cloud::transform(queryTransform, modelPoints, modelPoints);
		points->second.insert(points->second.end(), modelPoints.begin(), modelPoints.end());
		// render
		renderData(currentDataPtr);
		// find feasible trajectory
		for (grasp::StringSeq::const_iterator i = demo.second.modelTrajectories.begin(); i != demo.second.modelTrajectories.end() && err > REAL_ONE; ++i) {
			breakPoint();
			grasp::RobotState::Seq raw;
			golem::FileReadStream frs(i->c_str());
			frs.read(raw, raw.end(), grasp::RobotState(*robot->getController()));
			golem::Controller::State::Seq inp = grasp::RobotState::makeCommand(raw);
			// choose first with acceptable error
			approach.clear();
			const grasp::RBDist dist = robot->transformTrajectory(queryTransform, inp.begin(), inp.end(), approach);
			err = manipulator->getDesc().trajectoryErr.dot(dist);
			context.write("Trajectory error: lin=%.9f, ang=%.9f, total=%.9f\n", dist.lin, dist.ang, err);
		}
	}
	else {
		grasp::Classifier::Desc::Map::const_iterator ptr = classifierDescMap.find(demo.second.classifierName);
		if (ptr == classifierDescMap.end())
			throw Message(Message::LEVEL_ERROR, "ShapePlanner::run(): unknown classifier %s", demo.second.classifierName.c_str());
		classifier = load(*ptr->second);
		// estimate curvatures
		grasp::Cloud::PointSeq features;
		grasp::Cloud::copy(Bounds::Seq(), false, points->second, features);
		grasp::Cloud::curvature(context, cloudDesc.curvature, features);
		grasp::Cloud::nanRem(context, features, grasp::Cloud::isNanXYZNormalCurvature<grasp::Cloud::Point>);
		// estimate grasp
		classifier->find(features, grasp::to<Data>(currentDataPtr)->graspConfigs);
		grasp::Cluster::findLikelihood(clusterDesc, grasp::to<Data>(currentDataPtr)->graspConfigs, grasp::to<Data>(currentDataPtr)->graspClusters);
		// display
		grasp::to<Data>(currentDataPtr)->resetDataPointers();
		grasp::to<Data>(currentDataPtr)->graspMode = MODE_CONFIG;
		// trajectory
		CollisionBounds collisionBounds(*this, true);
		golem::U32 size = std::min(grasp::to<Data>(currentDataPtr)->getGraspConfigSize() - grasp::to<Data>(currentDataPtr)->graspConfigPtr, manipulator->getDesc().trajectoryClusterSize), j = grasp::to<Data>(currentDataPtr)->graspConfigPtr;
		const std::pair<grasp::Grasp::Config::Seq::const_iterator, grasp::RBDistEx> val = manipulator->find<grasp::Grasp::Config::Seq::const_iterator>(grasp::to<Data>(currentDataPtr)->getGraspConfigBegin(), grasp::to<Data>(currentDataPtr)->getGraspConfigBegin() + size, [&](const grasp::Grasp::Config::Ptr& config) -> grasp::RBDistEx {
			breakPoint();
			grasp::to<Data>(currentDataPtr)->graspConfigPtr = j++;
			renderData(currentDataPtr);
			const grasp::RBDist dist(manipulator->find(config->path));
			const grasp::RBDistEx distex(dist, manipulator->getDesc().trajectoryErr.collision && collisionBounds.collides(manipulator->getConfig(config->path.getApproach())));
			err = manipulator->getDesc().trajectoryErr.dot(dist);
			context.write("#%03u: Trajectory error: lin=%.9f, ang=%.9f, collision=%s, total=%f\n", j, distex.lin, distex.ang, distex.collision ? "yes" : "no", err);
			return distex;
		});
		manipulator->copy((*val.first)->path, approach);
		grasp::to<Data>(currentDataPtr)->graspConfigPtr = val.first - grasp::to<Data>(currentDataPtr)->getGraspConfigBegin();
		context.write("#%03u: Best trajectory\n", grasp::to<Data>(currentDataPtr)->graspConfigPtr + 1);
		renderData(currentDataPtr);
	}
	if (err > REAL_ONE) {
		context.write("No feasible trajectory has been found\n");
		return;
	}
	grasp::to<Data>(currentDataPtr)->trajectory[demo.first] = grasp::RobotState::make(manipulator->getController(), approach); // update trajectory collection

	// overwrite not used joints
	const grasp::RealSeq& cpos = demo.second.objectDetectionSeq.back().scanPose.c;
	for (golem::Controller::State::Seq::iterator i = approach.begin(); i != approach.end(); ++i)
		i->cpos.set(cpos.data() + manipulator->getJoints(), cpos.data() + cpos.size(), manipulator->getJoints());

	// find manipulation trajectory
	golem::Controller::State::Seq trajectory;
	createWithManipulation(approach, trajectory, true);
	breakPoint();

	// collision bounds
	CollisionBounds collisionBounds(*this, points->second);

	// perform
	try {
		perform(demo.first, trajectory, true);
	}
	catch (const Message& msg) {
		context.write("%s\n", msg.str().c_str());
		return;
	}
	breakPoint();

	// cleanup
	}

	// goto remaining poses
	for (Robot::Pose::Seq::const_iterator i = demo.second.poses.begin(); i != demo.second.poses.end(); ++i) {
		gotoPose(*i);
		breakPoint();
	}

#endif
}

void spam::ShapePlanner::function(Data::Map::iterator& dataPtr, int key) {
	switch (key) {
	case 'G':
	{
		switch (waitKey("LSARECUBXD", "Press a key to (L)oad/(S)ave classifier, (A)dd/(R)emove training data, (E)stimate/(C)luster grasp, find c(U)rrent/(B)est trajectory, e(X)port data, run (D)emo...")) {
		case 'L':
			classifier = load(*select(classifierDescMap.begin(), classifierDescMap.end(), "Available classifiers:\n", [](grasp::Classifier::Desc::Map::const_iterator ptr) -> const std::string&{
				return ptr->first;
			})->second);
			break;
		case 'S':
		{
			const std::string path = grasp::makeString("%s%s", classifier->getName().c_str(), extGraspClass.c_str());
			// save to file
			context.write("Saving training data to: %s\n", path.c_str());
			FileWriteStream fws(path.c_str());
			fws << (const golem::Serializable&)*classifier;
			break;
		}
		case 'A':
		{
			// point cloud
			const grasp::Cloud::PointSeq& points = queryDataPtr != dataPtr || grasp::to<Data>(queryDataPtr)->queryPoints.empty() ? getPoints(dataPtr)->second : grasp::to<Data>(queryDataPtr)->queryPoints;
			// trajectory
			const grasp::RobotState::Map::const_iterator trajectory = selectTrajectory(grasp::to<Data>(dataPtr)->trajectory.begin(), grasp::to<Data>(dataPtr)->trajectory.end());
			// grasp type
			const grasp::Grasp::Map::const_iterator grasp = select(classifier->getGraspMap().begin(), classifier->getGraspMap().end(), "Available grasp types:\n", [](grasp::Grasp::Map::const_iterator ptr) -> const std::string&{
				return ptr->first;
			});
			// training data name
			std::string name = grasp::to<Data>(dataPtr)->getName() + '_' + grasp->first;
			readString("Enter training data name: ", name);

			// parameters
			grasp::ParameterGuard<Real> distanceScale(grasp->second->getDesc().configurationDesc->distanceScale);
			readNumber("Enter distance scale: ", distanceScale);
			
			// add
			context.write("Adding training data...\n");
			grasp::to<Data>(dataPtr)->setGraspData(classifier->getGraspMap(), classifier->add(grasp->first, name, points, grasp::RobotState::makeCommand(trajectory->second), 0));
			
			// display
			grasp::to<Data>(dataPtr)->resetDataPointers();
			grasp::to<Data>(dataPtr)->graspMode = MODE_DATA;
			grasp::to<Data>(dataPtr)->appearanceData.featureRelation = grasp::Feature::RELATION_DFLT;
			renderData(dataPtr);
			break;
		}
		case 'R':
			if (grasp::to<Data>(dataPtr)->graspMode == MODE_DATA) {
				grasp::Grasp::Map::iterator grasp;
				grasp::Grasp::Data::Map::iterator data;
				if (grasp::to<Data>(dataPtr)->getGraspData(classifier->getGraspMap(), grasp, data)) {
					context.write("Removing training data: estimator=%s, type=%s, name=%s\n", grasp->second->getName().c_str(), grasp->first.c_str(), data->first.c_str());
					grasp->second->getDataMap().erase(data);
					renderData(dataPtr);
				}
			}
			else
				context.write("Select %s mode\n", ModeName[(size_t)MODE_DATA]);
			break;
		case 'E':
		{
			// available point cloud
			const grasp::Cloud::PointSeq& points = queryDataPtr != dataPtr || grasp::to<Data>(queryDataPtr)->queryPoints.empty() ? getPoints(dataPtr)->second : grasp::to<Data>(queryDataPtr)->queryPoints;
			// available grasp types
			grasp::StringSet availables;
			for (grasp::Grasp::Map::const_iterator i = classifier->getGraspMap().begin(); i != classifier->getGraspMap().end(); ++i)
				if (!i->second->getDataMap().empty())
					availables.insert(i->first);
			if (availables.empty())
				throw Message(Message::LEVEL_NOTICE, "No available grasp estimators with training data!");

			grasp::StringSet types;
			try {
				types.insert(*select(availables.begin(), availables.end(), "Select grasp type:\n  [0]  all types\n", [](grasp::StringSet::const_iterator ptr) -> const std::string&{ return *ptr; }, -1));
			}
			catch (const Message&) {}

			/*ParameterSeqGuard<U32, Real> kernels(grasp.second->getContacts().begin(), grasp.second->getContacts().end(), [] (Contact* contact) -> U32* { return &contact->getDesc().kernels; });
			readNumber("Enter kernels scale: ", kernels);
			kernels.update();
			ParameterSeqGuard<U32, Real> samples(grasp.second->getContacts().begin(), grasp.second->getContacts().end(), [] (Contact* contact) -> U32* { return &contact->getDesc().samples; });
			readNumber("Enter samples scale: ", samples);
			samples.update();
			ParameterGuard<size_t> runs(grasp.second->getDesc().optimisationDesc->runs);
			readNumber("Enter runs: ", runs);
			ParameterGuard<size_t> steps(grasp.second->getDesc().optimisationDesc->steps);
			readNumber("Enter steps: ", steps);*/

			context.write("Estimating grasp...\n");
			classifier->find(points, grasp::to<Data>(dataPtr)->graspConfigs, &types);

			grasp::to<Data>(dataPtr)->resetDataPointers();
			grasp::to<Data>(dataPtr)->graspMode = MODE_CONFIG;

			if (types.empty())
				grasp::Cluster::findLikelihood(clusterDesc, grasp::to<Data>(dataPtr)->graspConfigs, grasp::to<Data>(dataPtr)->graspClusters);
			else {
				grasp::Cluster::findType(clusterDesc, grasp::to<Data>(dataPtr)->graspConfigs, grasp::to<Data>(dataPtr)->graspClusters);
				while (grasp::to<Data>(dataPtr)->graspClusterPtr < grasp::to<Data>(dataPtr)->graspClusters.size() && grasp::to<Data>(dataPtr)->getGraspConfig()->get()->type != *types.begin()) ++grasp::to<Data>(dataPtr)->graspClusterPtr;
			}

			// display
			renderData(dataPtr);
			break;
		}
		case 'C':
		{
			if (!grasp::to<Data>(dataPtr)->hasGraspConfigs())
				throw Message(Message::LEVEL_ERROR, "No grasp data!");
			switch (waitKey("TLC", "Sort by (T)ype/(L)ikelihood, (C)luster...")) {
			case 'T':
				grasp::Cluster::findType(clusterDesc, grasp::to<Data>(dataPtr)->graspConfigs, grasp::to<Data>(dataPtr)->graspClusters);
				grasp::to<Data>(dataPtr)->resetDataPointers();
				grasp::to<Data>(dataPtr)->graspMode = MODE_CLUSTER;
				break;
			case 'L':
				grasp::Cluster::findLikelihood(clusterDesc, grasp::to<Data>(dataPtr)->graspConfigs, grasp::to<Data>(dataPtr)->graspClusters);
				grasp::to<Data>(dataPtr)->resetDataPointers();
				grasp::to<Data>(dataPtr)->graspMode = MODE_CONFIG;
				break;
			case 'C': {
				Real radius = clusterDesc.radius;
				readNumber("Enter cluster radius: ", radius);
				size_t density = clusterDesc.density;
				readNumber("Enter cluster density: ", density);
				grasp::Cluster::findConfiguration(clusterDesc, grasp::to<Data>(dataPtr)->graspConfigs, grasp::to<Data>(dataPtr)->graspClusters, radius, density);
				grasp::to<Data>(dataPtr)->resetDataPointers();
				grasp::to<Data>(dataPtr)->graspMode = MODE_CLUSTER;
				break;
			}
			};
			// display
			renderData(dataPtr);
			break;
		}
		case 'U':
		{
			if (!grasp::to<Data>(dataPtr)->hasGraspConfigs())
				throw Message(Message::LEVEL_ERROR, "No grasp data!");
			if (grasp::to<Data>(dataPtr)->graspMode == MODE_CLUSTER || grasp::to<Data>(dataPtr)->graspMode == MODE_CONFIG) {
				CollisionBounds collisionBounds(*this, true);
				grasp::Grasp::Config* config = grasp::to<Data>(dataPtr)->getGraspConfig()->get();
				const grasp::RBDist dist(manipulator->find(config->path));
				const grasp::RBDistEx distex(dist, manipulator->getDesc().trajectoryErr.collision && collisionBounds.collides(manipulator->getConfig(config->path.getApproach())));
				context.write("#%03u: Trajectory error: lin=%.9f, ang=%.9f, collision=%s\n", grasp::to<Data>(dataPtr)->graspConfigPtr + 1, distex.lin, distex.ang, distex.collision ? "yes" : "no");
				Controller::State::Seq trajectory;
				manipulator->copy((*grasp::to<Data>(dataPtr)->getGraspConfig())->path, trajectory);
				std::string name = (*grasp::to<Data>(dataPtr)->getGraspConfig())->type;
				readString("Enter trajectory name: ", name);
				grasp::to<Data>(dataPtr)->trajectory[name] = grasp::RobotState::make(manipulator->getController(), trajectory); // update trajectory collection
			}
			else
				context.write("Select %s mode\n", ModeName[(size_t)MODE_CONFIG]);
			break;
		}
		case 'B':
		{
			if (!grasp::to<Data>(dataPtr)->hasGraspConfigs())
				throw Message(Message::LEVEL_ERROR, "No grasp data!");
			if (grasp::to<Data>(dataPtr)->graspMode == MODE_CLUSTER || grasp::to<Data>(dataPtr)->graspMode == MODE_CONFIG) {
				const grasp::Grasp::Config::Seq::const_iterator begin = grasp::to<Data>(dataPtr)->getGraspConfigBegin() + grasp::to<Data>(dataPtr)->graspConfigPtr;
				const grasp::Grasp::Config::Seq::const_iterator end = begin + std::min(grasp::to<Data>(dataPtr)->getGraspConfigSize() - grasp::to<Data>(dataPtr)->graspConfigPtr, manipulator->getDesc().trajectoryClusterSize);
				golem::U32 j = grasp::to<Data>(currentDataPtr)->graspConfigPtr;
				CollisionBounds collisionBounds(*this, true);
				const std::pair<grasp::Grasp::Config::Seq::const_iterator, grasp::RBDistEx> val = manipulator->find<grasp::Grasp::Config::Seq::const_iterator>(grasp::to<Data>(dataPtr)->getGraspConfigBegin(), end, [&](const grasp::Grasp::Config::Ptr& config) -> grasp::RBDistEx {
					grasp::to<Data>(currentDataPtr)->graspConfigPtr = j++;
					renderData(currentDataPtr);
					const grasp::RBDist dist(manipulator->find(config->path));
					const grasp::RBDistEx distex(dist, manipulator->getDesc().trajectoryErr.collision && collisionBounds.collides(manipulator->getConfig(config->path.getApproach())));
					context.write("#%03u: Trajectory error: lin=%.9f, ang=%.9f, collision=%s\n", j, distex.lin, distex.ang, distex.collision ? "yes" : "no");
					return distex;
				});
				grasp::to<Data>(dataPtr)->graspConfigPtr = val.first - begin;
				context.write("#%03u: Best trajectory\n", grasp::to<Data>(dataPtr)->graspConfigPtr + 1);
				renderData(dataPtr);
				Controller::State::Seq trajectory;
				manipulator->copy((*val.first)->path, trajectory);
				std::string name = (*val.first)->type;
				readString("Enter trajectory name: ", name);
				grasp::to<Data>(dataPtr)->trajectory[name] = grasp::RobotState::make(manipulator->getController(), trajectory); // update trajectory collection
			}
			else
				context.write("Select %s mode\n", ModeName[(size_t)MODE_CONFIG]);
			break;
		}
		/*case 'X':
		{
			const char SEP = ' ';
			size_t n = 0;
			std::for_each(grasp.second->getContacts().begin(), grasp.second->getContacts().end(), [&] (const Contact* contact) {
				++n;
				if (contact->getFeatures().empty()) return;
				context.debug("Writing contact distribution #%d\n", n);
				// model distribution
				std::ofstream mfile(makeString("%s%s_model_%02d_%05d.txt", data->dir.c_str(), dataPtr->first.c_str(), n, contact->getFeatures().size()));
				mfile << std::scientific << "#" << SEP << "weight" << SEP << "p.x" << SEP << "p.y" << SEP << "p.z" << SEP << "q.x" << SEP << "q.y" << SEP << "q.z" << SEP << "q.w" <<  SEP << "r.x" <<  SEP << "r.y" << std::endl;
				std::for_each(contact->getFeatures().begin(), contact->getFeatures().end(), [&] (const Feature& feature) {
					mfile << feature.weight << SEP << feature.frame.p.x << SEP << feature.frame.p.y << SEP << feature.frame.p.z << SEP << feature.frame.q.x << SEP << feature.frame.q.y << SEP << feature.frame.q.z << SEP << feature.frame.q.w <<  SEP << feature.curvature.x <<  SEP << feature.curvature.y << std::endl;
				});
				// query distribution
				std::ofstream qfile(makeString("%s%s_query_%02d_%05d.txt", data->dir.c_str(), dataPtr->first.c_str(), n, contact->getPoses().size()));
				qfile << std::scientific << "#" << SEP << "weight" << SEP << "p.x" << SEP << "p.y" << SEP << "p.z" << SEP << "q.x" << SEP << "q.y" << SEP << "q.z" << SEP << "q.w" << std::endl;
				std::for_each(contact->getPoses().begin(), contact->getPoses().end(), [&] (const Contact::Pose& pose) {
					qfile << pose.weight << SEP << pose.p.x << SEP << pose.p.y << SEP << pose.p.z << SEP << pose.q.x << SEP << pose.q.y << SEP << pose.q.z << SEP << pose.q.w << std::endl;
				});
			});
			break;
		}*/
		case 'D':
			while (!demoMap.empty())
				for (Demo::Map::const_iterator i = demoMap.begin(); i != demoMap.end(); ++i)
					run(*i);
			break;
		}
		context.write("Done!\n");
		return;
	}
	case '7':
		grasp::to<Data>(dataPtr)->graspMode = (grasp::to<Data>(dataPtr)->graspMode + 1) % (MODE_SIZE + 1);
		context.write("Mode %s\n", ModeName[(size_t)grasp::to<Data>(dataPtr)->graspMode]);
		renderData(dataPtr);
		return;
	case '8':
		switch (grasp::to<Data>(dataPtr)->graspMode) {
		case MODE_DATA:
		{
			grasp::to<Data>(dataPtr)->appearanceData.featureRelation = grasp::to<Data>(dataPtr)->appearanceData.featureRelation == grasp::Feature::RELATION_VERTEX ? grasp::Feature::RELATION_NONE : (grasp::Feature::Relation)(grasp::to<Data>(dataPtr)->appearanceData.featureRelation + 1);
			context.write("Training data features %s\n", grasp::Feature::name[(size_t)grasp::to<Data>(dataPtr)->appearanceData.featureRelation]);
			break;
		}
		case MODE_CLUSTER:
			break;
		case MODE_CONFIG:
			break;
		case MODE_CONFIG_MODEL:
			break;
		case MODE_CONTACT_MODEL:
			break;
		};
		renderData(dataPtr);
		return;
	case '9':
		switch (grasp::to<Data>(dataPtr)->graspMode) {
		case MODE_DATA:
		{
			grasp::to<Data>(dataPtr)->appearanceData.showPath = !grasp::to<Data>(dataPtr)->appearanceData.showPath;
			context.write("Training data path %s\n", grasp::to<Data>(dataPtr)->appearanceData.showPath ? "ON" : "OFF");
			break;
		}
		case MODE_CLUSTER:
			break;
		case MODE_CONFIG:
			break;
		case MODE_CONFIG_MODEL:
			break;
		case MODE_CONTACT_MODEL:
			break;
		};
		renderData(dataPtr);
		return;
	case '(': case ')': 
		if (grasp::to<Data>(dataPtr)->graspMode != MODE_DISABLED && grasp::to<Data>(dataPtr)->graspMode != MODE_CONFIG_MODEL) {
			U32& ptr =
				grasp::to<Data>(dataPtr)->graspMode == MODE_DATA ? grasp::to<Data>(dataPtr)->graspDataPtr :
				grasp::to<Data>(dataPtr)->graspMode == MODE_CLUSTER ? grasp::to<Data>(dataPtr)->graspClusterPtr :
				grasp::to<Data>(dataPtr)->graspMode == MODE_CONFIG ? grasp::to<Data>(dataPtr)->graspConfigPtr : grasp::to<Data>(dataPtr)->appearanceConfig.modelSelectionIndex;
			if (key == '(' && ptr > 0) --ptr;
			if (key == ')') ++ptr;

			if (grasp::to<Data>(dataPtr)->graspMode == MODE_CONTACT_MODEL) {
				const grasp::Grasp* grasp = (*grasp::to<Data>(dataPtr)->getGraspConfig())->getGrasp();
				while (grasp && key == '(' && ptr > 0 && ptr < grasp->getContacts().size() && grasp->getContacts()[ptr]->getPoses().empty()) --ptr;
			}
		}
		renderData(dataPtr);
		return;
	// hand collision debug
	//case 'c':
	//{
	//	Manipulator::Pose pose = manipulator->getPose(robot->recvState().config);
	//	const Bounds::Seq boundsSeq = manipulator->getBounds(pose);
	//	for (auto bounds: boundsSeq) {
	//		int c = 0;
	//		for (auto j: to<Data>(dataPtr)->pointCloud)
	//			if (bounds->intersect(j.frame.p)) ++c;
	//		context.write("%s: %d\n", bounds->getName(), c);
	//	}
	//	Collision collision(*manipulator);
	//	Manipulator::RBPose rbpose(pose);
	//	Real likelihoodCol = collision.evaluate(grasp.second->getDesc().collisionDesc.waypoints[0], to<Data>(dataPtr)->pointCloud, rand, rbpose, true);
	//	golem::CriticalSectionWrapper csw(csDataRenderer);
	//	auto render = [] (const Collision::Bounds& bounds, DebugRenderer& renderer) {
	//		for (auto j: bounds.getTriangles())
	//			for (auto k: j)
	//				renderer.addLine(k.point, k.point + k.normal * 0.01, golem::RGBA::CYAN);
	//	};
	//	graspQueryRenderer.reset();
	//	for (U32 i = 0; i < Manipulator::JOINTS + 1; ++i)
	//		render(collision.getBounds(i), graspQueryRenderer);
	//	break;
	//}
	// rigid body transform debug
	//case 'c':
	//{
	//	golem::CriticalSectionWrapper csw(csDataRenderer);
	//	pointFeatureRenderer.reset();
	//	// global frame
	//	const U8 a = 20;
	//	const Real size = 0.1;
	//	pointFeatureRenderer.addAxes3D(Mat34::identity(), Vec3(size), 127);
	//	// translation
	//	Vec3 p1(0.0, 0.0, 0.0);
	//	Vec3 p2(0.1, 0.2, 0.1);
	//	RGBA colour = RGBA(127, 127, 127, 127);
	//	//pointFeatureRenderer.addAxis3D(p1, p2, colour, size*size/p2.magnitude());
	//	// rotation
	//	Mat33 rot;
	//	rot.rotX(0.7);
	//	Mat34 trn(rot, p2);
	//	//pointFeatureRenderer.addAxes3D(trn, Vec3(size), 127);
	//	for (U32 i = 0; i < 10000; ++i) {
	//		Mat34 pose;
	//		Vec3 v;
	//		// Linear component
	//		v.next(rand); // |v|==1
	//		v.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, 0.00025)), v);
	//		pose.p.add(p2, v);
	//		// Angular component
	//		Quat q;
	//		q.next(rand, 25000.0);
	//		pose.R.multiply(rot, q);
	//		// visualisation
	//		pointFeatureRenderer.addLine(p1, pose.p, RGBA(colour._rgba.r, colour._rgba.g, colour._rgba.b, a));
	//		pointFeatureRenderer.addAxes(pose, Vec3(size), RGBA(255, 0, 0, a), RGBA(0, 255, 0, a), RGBA(0, 0, 255, a));
	//	}
	//	pointFeatureRenderer.setLineWidth(3.0);
	//	break;
	//}
	}

	PosePlanner::function(dataPtr, key);
}

//------------------------------------------------------------------------------

void spam::XMLData(ShapePlanner::Demo::ObjectDetection &val, golem::XMLContext* xmlcontext, bool create) {
	XMLData(val.scanPose, xmlcontext->getContextFirst("scan_pose"), create);
	golem::XMLData("camera_name", val.cameraName, xmlcontext, create);
	golem::XMLData("min_size", val.minSize, xmlcontext, create);
	golem::XMLData("delta_size", val.deltaSize, xmlcontext, create);
	golem::XMLData("delta_depth", val.deltaDepth, xmlcontext, create);
}

void spam::XMLData(ShapePlanner::Demo &val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("pose_estimation", val.poseEstimation, xmlcontext, create);
	golem::XMLData("classifier_name", val.classifierName, xmlcontext, create);
	golem::XMLData("model_object", val.modelObject, xmlcontext, create);
	
	val.modelTrajectories.clear();
	std::pair<XMLContext::XMLContextMap::const_iterator, XMLContext::XMLContextMap::const_iterator> range = xmlcontext->getContextMap().equal_range("model_trajectory");
	for (XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second; ++i) {
		std::string name;
		golem::XMLData("name", name, const_cast<XMLContext*>(&i->second), create);
		val.modelTrajectories.push_back(name);
	}

	golem::XMLData(val.objectDetectionSeq, val.objectDetectionSeq.max_size(), xmlcontext, "object_detection", create);

	try {
		golem::XMLData(val.poses, val.poses.max_size(), xmlcontext, "pose", create);
	}
	catch (const golem::MsgXMLParser&) {}
}

void spam::XMLData(ShapePlanner::Demo::Map::value_type &val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("name", const_cast<std::string&>(val.first), xmlcontext, create);
	XMLData(val.second, xmlcontext, create);
}

void spam::XMLData(ShapePlanner::Desc &val, Context* context, XMLContext* xmlcontext, bool create) {
	XMLData((PosePlanner::Desc&)val, context, xmlcontext, create);

	xmlcontext = xmlcontext->getContextFirst("shape_planner");
	
	val.manipulatorDesc.reset(new grasp::Manipulator::Desc);
	grasp::XMLData(*val.manipulatorDesc, xmlcontext->getContextFirst("manipulator"), create);

	val.classifierDescMap.clear();
	golem::XMLData(val.classifierDescMap, val.classifierDescMap.max_size(), xmlcontext, "classifier", create);
	golem::XMLData("ext_grasp_class", val.extGraspClass, xmlcontext, create);

	val.demoMap.clear();
	try {
		golem::XMLData(val.demoMap, val.demoMap.max_size(), xmlcontext, "demo", create);
	}
	catch (const golem::MsgXMLParser&) {}

	grasp::XMLData(val.appearanceData, xmlcontext->getContextFirst("appearance data"), create);
	grasp::XMLData(val.appearanceConfig, xmlcontext->getContextFirst("appearance"), create);
}

//------------------------------------------------------------------------------

void ShapePlannerApp::run(int argc, char *argv[]) {
	// Setup ShapePlanner
	ShapePlanner::Desc shapeHandDesc;
	XMLData(shapeHandDesc, context(), xmlcontext());

	ShapePlanner *pShapePlanner = dynamic_cast<ShapePlanner*>(scene()->createObject(shapeHandDesc)); // throws
	if (pShapePlanner == nullptr)
		throw Message(Message::LEVEL_CRIT, "ShapePlannerApp::run(): Unable to cast to ShapePlanner");

	// Random number generator seed
	context()->info("Random number generator seed %d\n", context()->getRandSeed()._U32[0]);
	
	try {
		pShapePlanner->main();
	}
	catch (const grasp::Interrupted&) {
	}
	
	context()->info("Good bye!\n");
	scene()->releaseObject(*pShapePlanner);
};

//------------------------------------------------------------------------------

#ifdef _SPAM_SHAPEPLANNER_MAIN_
int main(int argc, char *argv[]) {
	return spam::ShapePlannerApp().main(argc, argv);
}
#endif // _SPAM_SHAPEPLANNER_MAIN_