/** @file ShapePlanner.cpp
 * 
 * @author	Marek Kopicki
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/Phys/Data.h>
#include <Golem/Plan/Data.h>
#include <Golem/Device/RobotJustin/RobotJustin.h>
#include <Spam/ShapePlanner/ShapePlanner.h>
#include <sstream>
#include <iomanip>

using namespace golem;
using namespace spam;

//------------------------------------------------------------------------------

void spam::ShapePlanner::Data::xmlData(golem::XMLContext* context, bool create) const {
	PosePlanner::Data::xmlData(context, create);

	try {
		if (!create || !graspPoses.empty()) {
			{
				const std::string name("grasp_pose");
				create ? xmlDataSave(name, grasp::makeString("%s%s%s%s%s", dir.c_str(), this->name.c_str(), sepName.c_str(), name.c_str(), extGraspPose.c_str()), context, graspPoses) : xmlDataLoad(name, context,  const_cast<grasp::Grasp::Pose::Seq&>(graspPoses));
			}
			{
				const std::string name("grasp_cluster");
				create ? xmlDataSave(name, grasp::makeString("%s%s%s%s%s", dir.c_str(), this->name.c_str(), sepName.c_str(), name.c_str(), extGraspCluster.c_str()), context, graspClusters) : xmlDataLoad(name, context,  const_cast<grasp::Grasp::Cluster::Seq&>(graspClusters));
			}
		}
	}
	catch (const golem::MsgXMLParser& msg) {
		if (create)
			throw msg;
	}
}

spam::ShapePlanner::Data::Ptr spam::ShapePlanner::Data::clone() const {
	return Ptr(new Data(*this));
}

//------------------------------------------------------------------------------

const char* spam::ShapePlanner::GraspModeName[GRASP_MODE_CONTACT + 1] = {"Disabled", "Grip", "Configuration", "Contact",};
const char* spam::ShapePlanner::GraspContactName[GRASP_CONTACT_SELECT + 1] = {"Disabled", "All", "Selected",};
const std::string spam::ShapePlanner::GRASP_NAME_DFLT = "Default";
const std::string spam::ShapePlanner::CLASSIFIER_NAME_DFLT = "Default";

spam::ShapePlanner::ShapePlanner(Scene &scene) : PosePlanner(scene) {
}

bool spam::ShapePlanner::create(const Desc& desc) {
	PosePlanner::create(desc); // throws
	
	manipulator.reset(new grasp::Manipulator(robot->getController()));
	
	demoDescMap = desc.demoDescMap;

	appearance = desc.appearance;
	extGraspClass = desc.extGraspClass;

	graspDescMap = desc.graspDescMap;
	GraspDescMap::const_iterator graspDesc = graspDescMap.find(GRASP_NAME_DFLT);
	if (graspDesc == graspDescMap.end())
		throw Message(Message::LEVEL_CRIT, "ShapePlanner::create(): unable to find %s grasp description", GRASP_NAME_DFLT.c_str());
	grasp = GraspMap::value_type(GRASP_NAME_DFLT, graspDesc->second->create(*manipulator));
	
	classifierDescMap = desc.classifierDescMap;
	ClassifierDescMap::const_iterator classifierDesc = classifierDescMap.find(GRASP_NAME_DFLT);
	if (classifierDesc == classifierDescMap.end())
		throw Message(Message::LEVEL_CRIT, "ShapePlanner::create(): unable to find %s classifier description", CLASSIFIER_NAME_DFLT.c_str());
	classifier = ClassifierMap::value_type(CLASSIFIER_NAME_DFLT, classifierDesc->second->create(*manipulator));
	try {
		FileReadStream(grasp::makeString("%s%s%s", data->dir.c_str(), classifierDesc->first.c_str(), extGraspClass.c_str()).c_str()) >> (golem::Serializable&)*classifier.second;
	}
	catch (const std::exception& ex) {
		context.warning("Unable to load classifier data: %s\n", ex.what());
	}

	resetDataPointers();

	// create table
	golem::RobotJustin* robotJustin = robot->getRobotJustin();
	if (robotJustin) {
		Actor::Desc tableDesc;
		BoundingBox::Desc tableBoxDesc;
		tableBoxDesc.dimensions.set(0.68*REAL_HALF, 1.27*REAL_HALF, 0.76*REAL_HALF);
		//for (auto i = robotJustin->getObjects().begin(); i != robotJustin->getObjects().end(); ++i)
		//	context.write("Object: %s\n", i->object_id.c_str());
		const char* names[] = {"table", "gert_table", nullptr};
		for (size_t i = 0;; ++i)
			try {
				if (names[i]) {
					robotJustin->getFrameObject(tableBoxDesc.pose, names[i]);
					context.debug("ShapePlanner::create(): Justin \"%s\"\n", names[i]);
					break;
				}
				else {
					context.error("ShapePlanner::create(): no Justin table defined\n");
					return true;
				}
			}
			catch (const std::exception&) {}
		tableBoxDesc.pose.p.x -= tableBoxDesc.dimensions.x;
		tableBoxDesc.pose.p.y += tableBoxDesc.dimensions.y;
		tableBoxDesc.pose.p.z += tableBoxDesc.dimensions.z;
		tableDesc.nxActorDesc.shapes.pushBack(scene.createNxShapeDesc(Bounds::Desc::Ptr(new BoundingBox::Desc(tableBoxDesc))));
		scene.createObject(tableDesc);
	}
			
	scene.getHelp().insert(Scene::StrMapVal("090", "  G                                       grasp operations\n"));
	scene.getHelp().insert(Scene::StrMapVal("090", "  C                                       grasp classifier operations\n"));
	scene.getHelp().insert(Scene::StrMapVal("091", "  7                                       grasp model mode\n"));
	scene.getHelp().insert(Scene::StrMapVal("091", "  8                                       grasp model feature type\n"));
	scene.getHelp().insert(Scene::StrMapVal("091", "  <mouse_wheel>                           grasp model approach trajectory\n"));
	scene.getHelp().insert(Scene::StrMapVal("092", "  9                                       grasp query contact type\n"));
	scene.getHelp().insert(Scene::StrMapVal("092", "  0                                       grasp query cluster index\n"));
	scene.getHelp().insert(Scene::StrMapVal("092", "  <mouse_wheel>                           grasp query approach trajectory\n"));
	scene.getHelp().insert(Scene::StrMapVal("092", "  ()                                      grasp query cluster solution index\n"));

	return true;
}

void spam::ShapePlanner::render() {
	PosePlanner::render();
	
	if (shapeDataRender) {
		golem::CriticalSectionWrapper csw(csDataRenderer);
		graspModelContactRenderer.render();
		//graspModelConfigRenderer.setLineWidth(2.0);
		graspModelConfigRenderer.render();

		const bool handSolid = graspMode == GRASP_MODE_GRIP ? appearance.gripSolid : graspMode == GRASP_MODE_CONFIG ? appearance.configDistribSolid : appearance.contactDistribSolid;
		if (handSolid) {
			boundsRenderer.setSolidColour(appearance.solidColourSelect);
			boundsRenderer.renderSolid(handBoundsSelect.begin(), handBoundsSelect.end());
			boundsRenderer.setSolidColour(appearance.solidColour);
			boundsRenderer.renderSolid(handBounds.begin(), handBounds.end());
		}
		else {
			boundsRenderer.setWireColour(appearance.wireColourSelect);
			boundsRenderer.renderWire(handBoundsSelect.begin(), handBoundsSelect.end());
			boundsRenderer.setLineWidth(appearance.wireWidth);
			boundsRenderer.setWireColour(appearance.wireColour);
			boundsRenderer.renderWire(handBounds.begin(), handBounds.end());
		}

		graspQueryRenderer.setPointSize(appearance.samplesPointSize);
		graspQueryRenderer.render();
	}
}

void spam::ShapePlanner::mouseHandler(int button, int state, int x, int y) {
	grasp::Player::mouseHandler(button, state, x, y);

	golem::CriticalSectionWrapper csw(csDataRenderer);
	if (currentDataPtr == getData().end() && grasp.second->getConfiguration()->getPaths().empty())
		return;
	grasp::Grasp::Data::Map::const_iterator graspDataPtr = grasp.second->getDataMap().find(currentDataPtr->first);
	const bool bModel = graspDataPtr != grasp.second->getDataMap().end();
	const bool bQuery = currentDataPtr == targetDataPtr && graspMode == GRASP_MODE_GRIP;
	
	if ((bModel || bQuery) && state == 1 && (button == 1 || button == 3 || button == 4)) {
		const I32 lo = bQuery ? I32(0) : -I32(appearance.pathSegments);
		const I32 hi = appearance.pathSegments;
		const Real delta = (bQuery ? grasp.second->getDesc().collisionDesc.waypoints.back().pathDist : grasp.second->getDesc().configurationDesc->distanceStdDev)/appearance.pathSegments;
		
		grasp::Configuration::Waypoint::Seq path;
		if (bQuery)
			grasp.second->getConfiguration()->create(*grasp::to<Data>(targetDataPtr)->getGraspPose(), path);
		else
			path = grasp.second->getConfiguration()->create(graspDataPtr->second.approach.begin(), graspDataPtr->second.approach.end());
		
		poseSubspaceDist = Math::clamp(button == 3 ? poseSubspaceDist + 1 : button == 4 ? poseSubspaceDist - 1 : 0, lo, hi);
		context.verbose("Path distance = %f\n", delta*poseSubspaceDist);

		graspModelConfigRenderer.reset();
		grasp::Manipulator::RBPose pose[2];
		Mat34 trn[2][grasp::Manipulator::JOINTS];
		
		for (I32 i = lo; i <= hi; ++i) {
			const bool showBounds = i == poseSubspaceDist;
			const bool showFrame = graspMode == GRASP_MODE_DISABLED && (showBounds || i == lo || i == hi);
			const bool showVertex = graspMode == GRASP_MODE_DISABLED && (i > lo);

			pose[1] = grasp.second->getConfiguration()->interpolate(path, delta*i);
			manipulator->getPoses(pose[1], trn[1]);
			
			if (showBounds) {
				handBounds.clear();
				handBoundsSelect.clear();
				addBounds(grasp::Manipulator::Pose(pose[1]));
			}
			if (showFrame)
				graspModelConfigRenderer.addAxes(Mat34(Mat33(pose[1].q), pose[1].p), distribFrameSize);
			for (U32 j = 1; j < manipulator->getChains(); ++j) {
				const U32 k = manipulator->getJoint(j + 1) - 1;
				if (showFrame)
					graspModelConfigRenderer.addAxes(trn[1][k], distribFrameSize);
				if (showVertex)
					graspModelConfigRenderer.addLine(trn[0][k].p, trn[1][k].p, appearance.pathColour);
			}

			pose[0] = pose[1];
			std::copy(trn[1], trn[1] + manipulator->getJoints(), trn[0]);
		}
	}
}

//------------------------------------------------------------------------------

void spam::ShapePlanner::renderData(Data::Map::const_iterator dataPtr) {
	PosePlanner::renderData(dataPtr);

	golem::CriticalSectionWrapper csw(csDataRenderer);

	graspModelContactRenderer.reset();
	graspModelConfigRenderer.reset();
	graspQueryRenderer.reset();
	handBounds.clear();
	handBoundsSelect.clear();

	// learning data pointer
	grasp::Grasp::Data::Map::const_iterator graspDataPtr = grasp.second->getDataMap().find(dataPtr->first);

	// Model contacts
	if (graspDataPtr != grasp.second->getDataMap().end() && showFeatureRelation != grasp::Feature::RELATION_NONE) {
		std::for_each(appearance.graspData.jointFeatureAppearance.begin(), appearance.graspData.jointFeatureAppearance.end(), [=] (grasp::Feature::Appearance& appearance) { appearance.relation = showFeatureRelation; });
		//grasp.second->getDesc().learningDataAppearance.jointFeatureAppearance.relation = showFeatureRelation;
		appearance.graspData.baseFeatureAppearance.relation = showFeatureRelation;
		graspDataPtr->second.draw(*manipulator, grasp.second->getContacts(), appearance.graspData, graspModelContactRenderer);
	}
	
	// Query grip
	if (graspMode == GRASP_MODE_GRIP && grasp::to<Data>(dataPtr)->hasGraspPoses()) {
		const grasp::Grasp::Pose::Seq::const_iterator graspPose = grasp::to<Data>(dataPtr)->getGraspPose();
		addBounds(*graspPose);
		graspQueryRenderer.addAxes(graspPose->toMat34(), distribFrameSize);
		//const Manipulator::RBPose pose = manipulator->getPose(robot->recvState().config);
		//context.debug("Current grip pose likelihood = %e\n", grasp.second->evaluate(pose));
	}
	// Config distribution 
	else if (graspMode == GRASP_MODE_CONFIG && graspDataPtr != grasp.second->getDataMap().end()) {
		for (size_t i = 0; i < appearance.distribSamples; ++i) {
			const grasp::Manipulator::Pose pose = grasp.second->getConfiguration()->sample();
			Vec3 frameSize = distribFrameSize;

			if (i < appearance.distribBounds) {
				addBounds(pose);
				frameSize *= 5.0;
			}

			Mat34 frames[grasp::Manipulator::JOINTS];
			manipulator->getPoses(pose, frames);
			for (U32 j = 1; j < manipulator->getChains(); ++j) {
				const U32 k = manipulator->getJoint(j + 1) - 1;
				appearance.showFrames ? graspQueryRenderer.addAxes(frames[k], frameSize) : graspQueryRenderer.addPoint(frames[k].p, appearance.samplesColour);
			}
		}
	}
	else
		// Configuration manifolds
		mouseHandler(1, 1, 0, 0);

	// contact distribution
	if (graspMode == GRASP_MODE_CONTACT && targetDataPtr == dataPtr && contactIndex < grasp.second->getContacts().size() && !grasp.second->getContacts().empty() && !grasp.second->getContacts()[contactIndex]->getPoses().empty()) {
		for (size_t i = 0; i < appearance.distribSamples; ++i) {
			const grasp::RBCoord rbframe = grasp.second->getContacts()[contactIndex]->sample();
			const Mat34 frame(Mat33(rbframe.q), rbframe.p);
			Vec3 frameSize = distribFrameSize;

			if (i < appearance.distribBounds) {
				grasp::Manipulator::Pose pose(grasp.second->getConfiguration()->sample());
				
				// joints
				if (contactIndex < manipulator->getJoints()) {
					Mat34 trn[grasp::Manipulator::JOINTS], delta;
					manipulator->getPoses(pose, trn);
					// trn*delta = pose ==> delta = trn^-1*pose
					delta.setInverseRT(trn[contactIndex]);
					delta.multiply(delta, pose);
					pose.multiply(frame, delta);
				}
				// base
				else {
					pose = frame;
				}

				addBounds(pose);
				frameSize *= 5.0;
			}

			appearance.showFrames ? graspQueryRenderer.addAxes(frame, frameSize) : graspQueryRenderer.addPoint(frame.p, appearance.samplesColour);
		}
	}
}

void spam::ShapePlanner::resetDataPointers() {
	PosePlanner::resetDataPointers();

	showFeatureRelation = grasp::Feature::RELATION_NONE;
	grasp.second->getDataMap().clear();
	graspMode = GRASP_MODE_DISABLED;
	graspContact = GRASP_CONTACT_DISABLED;
	configIndex = 0;
	poseSubspaceDist = 0;
	contactIndex = 0;

	shapeDataRender = true;

	targetDataPtr = getData().end();
}

void spam::ShapePlanner::profile(Data::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) const {
	if (targetDataPtr != dataPtr)
		PosePlanner::profile(dataPtr, inp, dur, idle);
	else
		Player::profile(dataPtr, inp, dur, idle);
}

void spam::ShapePlanner::printGripInfo() {
	auto ptr = getPtr<Data>(currentDataPtr);
	if (ptr != nullptr) {
		const grasp::Grasp::Pose::Seq::const_iterator graspPose = ptr->getGraspPose();
	
		std::stringstream str;
		for (size_t i = 0; i < manipulator->getJoints(); ++i)
			str << " c" << (i + 1) << "=\"" << graspPose->jc[i] << "\"";
		const grasp::RBCoord frame(*graspPose * robot->getController()->getChains()[robot->getStateArmInfo().getChains().begin()]->getReferencePose());

		//context.write("<pose v1=\"%.7f\" v2=\"%.7f\" v3=\"%.7f\" q0=\"%.7f\" q1=\"%.7f\" q2=\"%.7f\" q3=\"%.7f\" dim=\"%d\"%s/>\n", frame.p.x, frame.p.y, frame.p.z, frame.q.q0, frame.q.q1, frame.q.q2, frame.q.q3, robot->getStateInfo().getJoints().size(), str.str().c_str());
		context.write("Pose cluster=%d/%d, solution=%d/%d, likelihood_{total, pose, collision}={%e, %e, %e}\n",
			ptr->graspClusterPtr + 1, ptr->graspClusters.size(), ptr->graspClusterSolutionPtr + 1, (ptr->graspClusters[ptr->graspClusterPtr].end - ptr->graspClusters[ptr->graspClusterPtr].begin),
			graspPose->likelihood.product, (graspPose->likelihood.product - graspPose->likelihood.collision), graspPose->likelihood.collision);
	}
}

void spam::ShapePlanner::addBounds(const grasp::Manipulator::Pose& pose) {
	manipulator->getBaseBounds(pose, handBounds);//!grasp.second->getContacts().empty() && (graspContact == GRASP_CONTACT_ALL || graspContact == GRASP_CONTACT_SELECT && contactIndex >= manipulator->getJoints()) ? manipulator->getBaseBounds(pose, handBoundsSelect) : manipulator->getBaseBounds(pose, handBounds);
	golem::Mat34 trn[ grasp::Manipulator::JOINTS];
	manipulator->getPoses(pose, trn);
	for (U32 i = manipulator->getArmJoints(); i < manipulator->getJoints(); ++i)
		i < grasp.second->getContacts().size() && !grasp.second->getContacts()[i]->getPoses().empty() && (graspContact == GRASP_CONTACT_ALL || graspContact == GRASP_CONTACT_SELECT && i == contactIndex) ? manipulator->getJointBounds(i, trn[i], handBoundsSelect) : manipulator->getJointBounds(i, trn[i], handBounds);
}

//------------------------------------------------------------------------------

void spam::ShapePlanner::function(Data::Map::iterator& dataPtr, int key) {
	switch (key) {
	case 'G':
	{
		switch (waitKey("LARECUBX", "Press a key to (L)oad grasp, (A)dd/(R)emove training data, (E)stimate/(C)luster grip pose, find c(U)rrent/(B)est trajectory, e(X)port data...")) {
		case 'L':
		{
			// load grasp description
			context.write("Grasp description name:\n");
			for (ShapePlanner::GraspDescMap::const_iterator i = graspDescMap.begin(); i != graspDescMap.end(); ++i) context.write("  %s\n", i->first.c_str());
			std::string graspName = GRASP_NAME_DFLT;
			readString("Enter grasp description name: ", graspName);
			GraspDescMap::const_iterator graspDesc = graspDescMap.find(graspName);
			if (graspDesc == graspDescMap.end())
				throw Message(Message::LEVEL_ERROR, "Unable to find %s grasp description", graspName.c_str());
			grasp = GraspMap::value_type(graspName, graspDesc->second->create(*manipulator));

			resetDataPointers();
			//golem::Scene::OpenGL openGL;
			//scene.getOpenGL(openGL);
			renderData(dataPtr);
			break;
		}
		case 'A':
		{
			const  grasp::Cloud::PointSeq& points = queryDataPtr != dataPtr || grasp::to<Data>(queryDataPtr)->queryPoints.empty() ? getPoints(dataPtr)->second : grasp::to<Data>(queryDataPtr)->queryPoints;

			grasp::ParameterGuard<Real> distanceScale(grasp.second->getDesc().configurationDesc->distanceScale);
			readNumber("Enter distanceScale: ", distanceScale);
			
			context.write("Learning grasp...\n");
			grasp.second->add(dataPtr->first,  grasp::to<Data>(dataPtr)->actionApproach, grasp::to<Data>(dataPtr)->actionManip, points);
			
			poseSubspaceDist = 0;
			showFeatureRelation = grasp::Feature::RELATION_DFLT;
			break;
		}
		case 'R':
			poseSubspaceDist = 0;
			showFeatureRelation = grasp::Feature::RELATION_NONE;
			grasp.second->getDataMap().clear();
			break;
		case 'E':
		{
			const grasp::Cloud::PointSeq& points = queryDataPtr != dataPtr || grasp::to<Data>(queryDataPtr)->queryPoints.empty() ? getPoints(dataPtr)->second : grasp::to<Data>(queryDataPtr)->queryPoints;

			grasp::ParameterSeqGuard<U32, Real> kernels(grasp.second->getContacts().begin(), grasp.second->getContacts().end(), [] (grasp::Contact* contact) -> U32* { return &contact->getDesc().kernels; });
			readNumber("Enter kernels scale: ", kernels);
			kernels.update();
			grasp::ParameterSeqGuard<U32, Real> samples(grasp.second->getContacts().begin(), grasp.second->getContacts().end(), [] (grasp::Contact* contact) -> U32* { return &contact->getDesc().samples; });
			readNumber("Enter samples scale: ", samples);
			samples.update();
			grasp::ParameterGuard<size_t> runs(grasp.second->getDesc().optimisationDesc->runs);
			readNumber("Enter runs: ", runs);
			grasp::ParameterGuard<size_t> steps(grasp.second->getDesc().optimisationDesc->steps);
			readNumber("Enter steps: ", steps);

			context.write("Estimating grasp...\n");
			targetDataPtr = getData().end();
			grasp.second->findGrip(points, points, grasp::to<Data>(dataPtr)->graspPoses);
			grasp.second->findGripClusters(grasp::to<Data>(dataPtr)->graspPoses, grasp::to<Data>(dataPtr)->graspClusters);
			targetDataPtr = dataPtr;

			grasp::to<Data>(dataPtr)->resetDataPointers();
			graspMode = GRASP_MODE_GRIP;
			printGripInfo();
			break;
		}
		case 'C':
		{
			if (!grasp::to<Data>(dataPtr)->hasGraspPoses())
				throw Message(Message::LEVEL_ERROR, "No grasp data!");
			Real radius = grasp.second->getDesc().clusteringDesc.radius;
			readNumber("Enter cluster radius: ", radius);
			size_t density = grasp.second->getDesc().clusteringDesc.density;
			readNumber("Enter cluster density: ", density);
			grasp.second->findGripClusters(grasp::to<Data>(dataPtr)->graspPoses, grasp::to<Data>(dataPtr)->graspClusters, radius, density);
			grasp::to<Data>(dataPtr)->resetDataPointers();
			printGripInfo();
			break;
		}
		case 'U':
		{
			if (! grasp::to<Data>(dataPtr)->hasGraspPoses())
				throw Message(Message::LEVEL_ERROR, "No grasp data!");
			(void)grasp.second->findTrajectory(*robot,  grasp::to<Data>(dataPtr)->getGraspPose(),  grasp::to<Data>(dataPtr)->getGraspPose() + 1,  grasp::to<Data>(dataPtr)->actionApproach,  grasp::to<Data>(dataPtr)->actionManip);
			printGripInfo();
			break;
		}
		case 'B':
		{
			if (!grasp::to<Data>(dataPtr)->hasGraspPoses())
				throw Message(Message::LEVEL_ERROR, "No grasp data!");
			const grasp::Grasp::Pose::Seq::const_iterator ptr = grasp.second->findTrajectory(*robot, grasp::to<Data>(dataPtr)->getGraspPoseBegin(), grasp::to<Data>(dataPtr)->getGraspPoseEnd(), grasp::to<Data>(dataPtr)->actionApproach, grasp::to<Data>(dataPtr)->actionManip);
			grasp::to<Data>(dataPtr)->graspClusterSolutionPtr = size_t(ptr - grasp::to<Data>(dataPtr)->getGraspPoseBegin());
			printGripInfo();
			break;
		}
		case 'X':
		{
			const char SEP = ' ';
			size_t n = 0;
			std::for_each(grasp.second->getContacts().begin(), grasp.second->getContacts().end(), [&] (const grasp::Contact* contact) {
				++n;
				if (contact->getFeatures().empty()) return;
				context.debug("Writing contact distribution #%d\n", n);
				// model distribution
				std::ofstream mfile(grasp::makeString("%s%s_model_%02d_%05d.txt", data->dir.c_str(), dataPtr->first.c_str(), n, contact->getFeatures().size()));
				mfile << std::scientific << "#" << SEP << "weight" << SEP << "p.x" << SEP << "p.y" << SEP << "p.z" << SEP << "q.x" << SEP << "q.y" << SEP << "q.z" << SEP << "q.w" <<  SEP << "r.x" <<  SEP << "r.y" << std::endl;
				std::for_each(contact->getFeatures().begin(), contact->getFeatures().end(), [&] (const grasp::Feature& feature) {
					mfile << feature.weight << SEP << feature.frame.p.x << SEP << feature.frame.p.y << SEP << feature.frame.p.z << SEP << feature.frame.q.x << SEP << feature.frame.q.y << SEP << feature.frame.q.z << SEP << feature.frame.q.w <<  SEP << feature.curvature.x <<  SEP << feature.curvature.y << std::endl;
				});
				// query distribution
				std::ofstream qfile(grasp::makeString("%s%s_query_%02d_%05d.txt", data->dir.c_str(), dataPtr->first.c_str(), n, contact->getPoses().size()));
				qfile << std::scientific << "#" << SEP << "weight" << SEP << "p.x" << SEP << "p.y" << SEP << "p.z" << SEP << "q.x" << SEP << "q.y" << SEP << "q.z" << SEP << "q.w" << std::endl;
				std::for_each(contact->getPoses().begin(), contact->getPoses().end(), [&] (const grasp::Contact::Pose& pose) {
					qfile << pose.weight << SEP << pose.p.x << SEP << pose.p.y << SEP << pose.p.z << SEP << pose.q.x << SEP << pose.q.y << SEP << pose.q.z << SEP << pose.q.w << std::endl;
				});
			});
			break;
		}
		}
		renderData(dataPtr);
		context.write("Done!\n");
		return;
	}
	case 'S':
	{
		switch (waitKey("LSPARED", "Press a key to (S)ave/(L)oad classifier, (P)rint/(A)dd/(R)emove training data, (E)stimate grasp, (D)emo...")) {
		case 'L':
		{
			context.write("Classifier description name:\n");
			for (ShapePlanner::ClassifierDescMap::const_iterator i = classifierDescMap.begin(); i != classifierDescMap.end(); ++i) context.write("  %s\n", i->first.c_str());
			std::string classifierName = CLASSIFIER_NAME_DFLT;
			readString("Enter classifier name: ", classifierName);
			const std::string path = grasp::makeString("%s%s%s", data->dir.c_str(), classifierName.c_str(), extGraspClass.c_str());
			// load from file
			context.write("Loading classifier data from: %s\n", path.c_str());
			FileReadStream frs(path.c_str());
			frs >> (golem::Serializable&)*classifier.second;
			break;
		}
		case 'S':
		{
			const std::string path = grasp::makeString("%s%s%s", data->dir.c_str(), classifier.first.c_str(), extGraspClass.c_str());
			// save to file
			context.write("Saving classifier data to: %s\n", path.c_str());
			FileWriteStream fws(path.c_str());
			fws << (const golem::Serializable&)*classifier.second;
			break;
		}
		case 'P':
			context.write("Classifier training data:\n");
			for (grasp::Classifier::DataSet::const_iterator i = classifier.second->getTrainingSet().begin(); i != classifier.second->getTrainingSet().end(); ++i) {
				auto estimator = classifier.second->getDesc().graspSet.find(i->first);
				if (estimator == classifier.second->getDesc().graspSet.end())
					throw Message(Message::LEVEL_ERROR, "Unable to find estimator name for grasp type %s", i->first.c_str());
				auto data = classifier.second->getGraspDataSet().find(i->first);
				if (data == classifier.second->getGraspDataSet().end())
					throw Message(Message::LEVEL_ERROR, "Unable to find data set for grasp type %s", i->first.c_str());
				context.write("  type: %s, estimator: %s, data size: %d\n", i->first.c_str(), estimator->second.c_str(), data->second.size());
			}
			break;
		case 'A':
		{
			if (!grasp::to<Data>(dataPtr)->hasGraspPoses())
				throw Message(Message::LEVEL_ERROR, "No grasp data!");
			context.write("Grasp type name:\n");
			for (grasp::Classifier::GraspSet::const_iterator i = classifier.second->getDesc().graspSet.begin(); i != classifier.second->getDesc().graspSet.end(); ++i) context.write("  %s\n", i->first.c_str());
			std::string type = classifier.second->getDesc().graspSet.begin()->first;
			readString("Enter grasp type: ", type);
			classifier.second->add(type, grasp.first, grasp.second->getDataMap(), * grasp::to<Data>(dataPtr)->getGraspPose());
			break;
		}
		case 'R':
			context.write("Removing classifier data...\n");
			classifier.second->clear();
			break;
		case 'E':
		{
			context.write("Preparing classifier data...\n");
			const grasp::Cloud::PointSeq& points = queryDataPtr != dataPtr || grasp::to<Data>(queryDataPtr)->queryPoints.empty() ? getPoints(dataPtr)->second : grasp::to<Data>(queryDataPtr)->queryPoints;
			targetDataPtr = dataPtr;
			{
				// protect the default grasp estimator
				std::pair<std::string, grasp::Grasp::Ptr> grasptmp = grasp; // 2 references - grasp wont be released
				grasp::ScopeGuard guard([&] () { grasp = grasptmp; });

				// load grasps and perform queries
				classifier.second->create([&] (const std::string& estimator, const grasp::Grasp::Data::Map& data, grasp::Grasp::Pose& pose) {
					// create grasp estimator
					GraspDescMap::const_iterator graspDesc = graspDescMap.find(estimator);
					if (graspDesc == graspDescMap.end())
						throw Message(Message::LEVEL_ERROR, "Unable to find %s grasp estimator", estimator.c_str());
					grasp = GraspMap::value_type(estimator, graspDesc->second->create(*manipulator));
						
					// add training data
					for (grasp::Grasp::Data::Map::const_iterator i = data.begin(); i != data.end(); ++i) grasp.second->add(*i);

					// estimate grasp configuration
					grasp.second->findGrip(points, points, grasp::to<Data>(dataPtr)->graspPoses);
					grasp.second->findGripClusters(grasp::to<Data>(dataPtr)->graspPoses, grasp::to<Data>(dataPtr)->graspClusters);
					grasp::to<Data>(dataPtr)->resetDataPointers();
					graspMode = GRASP_MODE_GRIP;
					poseSubspaceDist = 0;

					// choose grasp configuration
					for (;;) {
						printGripInfo();
						renderData(dataPtr);
						const int key = waitKey("0(C)", "Press (C) to select and continue...");
						if (key == 'C') break;
						else if (key == '0')
							grasp::to<Data>(dataPtr)->graspClusterPtr = (grasp::to<Data>(dataPtr)->graspClusterPtr + 1)%grasp::to<Data>(dataPtr)->graspClusters.size();
						else {
							size_t& solution = grasp::to<Data>(dataPtr)->graspClusterSolutionPtr;
							if (key == '(' && solution > 0) --solution;
							if (key == ')' && solution + 1 < grasp::to<Data>(dataPtr)->graspClusters[grasp::to<Data>(dataPtr)->graspClusterPtr].size()) ++solution;
						}
					}
					pose = *grasp::to<Data>(dataPtr)->getGraspPose();
				});
			}
			// run classifier
			context.write("Running classifier...\n");
			 grasp::Classifier::DataSet::value_type solution;
			classifier.second->find(solution);
			grasp::to<Data>(dataPtr)->resetDataPointers();
			grasp::to<Data>(dataPtr)->graspPoses.clear();
			grasp::to<Data>(dataPtr)->graspPoses.push_back(solution.second);
			grasp::to<Data>(dataPtr)->graspClusters.clear();
			grasp::to<Data>(dataPtr)->graspClusters.push_back(grasp::Grasp::Cluster(0, grasp::to<Data>(dataPtr)->graspPoses.size()));
			// update current grasp with the data used for training
			grasp::Classifier::GraspDataSet::const_iterator data = classifier.second->getGraspDataSet().find(solution.first);
			if (data != classifier.second->getGraspDataSet().end())
				for (auto i: data->second) grasp.second->add(i);
			// display data
			poseSubspaceDist = 0;
			printGripInfo();
			renderData(dataPtr);
			break;
		}
		case 'D':
		{
			break;
		}
		}
		context.write("Done!\n");
		return;
	}
	case 'Q':
	{
		PosePlanner::function(dataPtr, key); // dataPtr --> queryDataPtr
		// if model has computed grasps
		if (grasp::to<Data>(modelDataPtr)->hasGraspPoses()) {
			// copy grasps to query
			grasp::to<Data>(queryDataPtr)->resetDataPointers();
			grasp::to<Data>(queryDataPtr)->graspPoses = grasp::to<Data>(modelDataPtr)->graspPoses;
			grasp::to<Data>(queryDataPtr)->graspClusters = grasp::to<Data>(modelDataPtr)->graspClusters;
			// and transform it
			const grasp::RBCoord trn(grasp::to<Data>(queryDataPtr)->actionFrame);
			for (grasp::Grasp::Pose::Seq::iterator i = grasp::to<Data>(queryDataPtr)->graspPoses.begin(); i != grasp::to<Data>(queryDataPtr)->graspPoses.end(); ++i)
				i->multiply(trn, *i);
			graspMode = GRASP_MODE_GRIP;
		}
		return;
	}
	case '7':
		graspMode = (graspMode + 1)%(GRASP_MODE_CONTACT + 1);
		context.write("Grasp mode %s\n", GraspModeName[(size_t)graspMode]);
		renderData(dataPtr);
		return;
	case '8':
		showFeatureRelation = showFeatureRelation == grasp::Feature::RELATION_VERTEX ? grasp::Feature::RELATION_NONE : (grasp::Feature::Relation)(showFeatureRelation + 1);
		context.write("Grasp features %s\n", grasp::Feature::name[(size_t)showFeatureRelation]);
		renderData(dataPtr);
		return;
	case '9':
		graspContact = (graspContact + 1)%(GRASP_CONTACT_SELECT + 1);
		context.write("Grasp contact %s\n", GraspContactName[(size_t)graspContact]);
		renderData(dataPtr);
		return;
	case '0':
		if (graspMode == GRASP_MODE_GRIP && grasp::to<Data>(dataPtr)->hasGraspPoses()) {
			grasp::to<Data>(dataPtr)->graspClusterPtr = (grasp::to<Data>(dataPtr)->graspClusterPtr + 1)%grasp::to<Data>(dataPtr)->graspClusters.size();
			grasp::to<Data>(dataPtr)->graspClusterSolutionPtr = 0;
			poseSubspaceDist = 0;
			printGripInfo();
		}
		if (graspMode == GRASP_MODE_CONFIG && grasp.second->getConfiguration()->getPaths().size() > 0) {
			configIndex = (configIndex + 1)%grasp.second->getConfiguration()->getPaths().size();
			poseSubspaceDist = 0;
			context.write("Configuration #%d/%d: size=%d\n", configIndex + 1, grasp.second->getConfiguration()->getPaths().size(), grasp.second->getConfiguration()->getPaths()[configIndex].size());
		}
		if (graspMode == GRASP_MODE_CONTACT && grasp.second->getContacts().size() > 0) {
			for (size_t stopIndex = contactIndex;;) {
				contactIndex = (contactIndex + 1)%grasp.second->getContacts().size();
				if (stopIndex == contactIndex) return;
				if (!grasp.second->getContacts()[contactIndex]->getPoses().empty()) break;
			}
			context.write("Contact #%d/%d\n", contactIndex + 1, grasp.second->getContacts().size());
		}
		renderData(dataPtr);
		return;
	case '(': case ')':
		if (graspMode == GRASP_MODE_GRIP && grasp::to<Data>(dataPtr)->hasGraspPoses()) {
			size_t& solution = grasp::to<Data>(dataPtr)->graspClusterSolutionPtr;
			if (key == '(' && solution > 0) --solution;
			if (key == ')' && solution + 1 < grasp::to<Data>(dataPtr)->graspClusters[grasp::to<Data>(dataPtr)->graspClusterPtr].size()) ++solution;
			poseSubspaceDist = 0;
			printGripInfo();
		}
		renderData(dataPtr);
		return;
	case '_':
		context.debug("Current collision likelihood = %e\n",  grasp::Collision(*manipulator).evaluate(grasp.second->getDesc().collisionDesc.waypoints[0], getPoints(dataPtr)->second, rand, manipulator->getPose(robot->recvState().config), true));
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

void spam::XMLData(ShapePlanner::Appearance &val, golem::XMLContext* xmlcontext, bool create) {
	grasp::XMLData(val.graspData, xmlcontext, create);

	golem::XMLData("distrib_samples", val.distribSamples, xmlcontext, create);
	golem::XMLData("distrib_bounds", val.distribBounds, xmlcontext, create);

	golem::XMLData(val.samplesColour, xmlcontext->getContextFirst("samples_colour"), create);
	golem::XMLData("samples_point_size", val.samplesPointSize, xmlcontext, create);

	golem::XMLData("path_segments", val.pathSegments, xmlcontext, create);
	golem::XMLData(val.pathColour, xmlcontext->getContextFirst("path_colour"), create);

	golem::XMLData("show_frames", val.showFrames, xmlcontext, create);
	golem::XMLData("grip_solid", val.gripSolid, xmlcontext, create);
	golem::XMLData("config_distrib_solid", val.configDistribSolid, xmlcontext, create);
	golem::XMLData("contact_distrib_solid", val.contactDistribSolid, xmlcontext, create);

	golem::XMLData(val.solidColour, xmlcontext->getContextFirst("solid_colour"), create);
	golem::XMLData(val.solidColourSelect, xmlcontext->getContextFirst("solid_colour_select"), create);
	golem::XMLData(val.wireColour, xmlcontext->getContextFirst("wire_colour"), create);
	golem::XMLData(val.wireColourSelect, xmlcontext->getContextFirst("wire_colour_select"), create);
	golem::XMLData("wire_width", val.wireWidth, xmlcontext, create);
}

void spam::XMLData(ShapePlanner::GraspDescMap::value_type &val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("name", const_cast<std::string&>(val.first), xmlcontext, create);
	val.second.reset(new grasp::Grasp::Desc());
	grasp::XMLData(*val.second, xmlcontext, create);
}

void spam::XMLData(ShapePlanner::ClassifierDescMap::value_type &val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("name", const_cast<std::string&>(val.first), xmlcontext, create);
	val.second.reset(new grasp::Classifier::Desc());
	grasp::XMLData(*val.second, xmlcontext, create);
}

void spam::XMLData(ShapePlanner::DemoDesc &val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("data_model_name", val.dataModelName, xmlcontext, create);
	golem::XMLData("data_query_name", val.dataQueryName, xmlcontext, create);
	golem::XMLData("approach_extrapol_fac", val.trjApproachExtrapolFac, xmlcontext, create);
	try {
		golem::XMLData(val.frames, val.frames.max_size(), xmlcontext, "frame", create);
	}
	catch (const golem::MsgXMLParser&) {}
	golem::XMLData("trj_cluster_size", val.trjClusterSize, xmlcontext, create);
	golem::XMLData("estimate_pose", val.estimatePose, xmlcontext, create);
}

void spam::XMLData(ShapePlanner::DemoDesc::Map::value_type &val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("name", const_cast<std::string&>(val.first), xmlcontext, create);
	XMLData(val.second, xmlcontext, create);
}

void spam::XMLData(ShapePlanner::Desc &val, Context* context, XMLContext* xmlcontext, bool create) {
	XMLData((PosePlanner::Desc&)val, context, xmlcontext, create);

	xmlcontext = xmlcontext->getContextFirst("shape_planner");
	
	val.graspDescMap.clear();
	golem::XMLData(val.graspDescMap, val.graspDescMap.max_size(), xmlcontext, "grasp", create);

	val.classifierDescMap.clear();
	golem::XMLData(val.classifierDescMap, val.classifierDescMap.max_size(), xmlcontext, "classifier", create);
	golem::XMLData("ext_grasp_class", val.extGraspClass, xmlcontext, create);

	val.demoDescMap.clear();
	golem::XMLData(val.demoDescMap, val.demoDescMap.max_size(), xmlcontext, "demo", create);

	spam::XMLData(val.appearance, xmlcontext->getContextFirst("appearance"), create);
}

//------------------------------------------------------------------------------

void ShapePlannerApp::run(int argc, char *argv[]) {
	// Setup ShapePlanner
	ShapePlanner::Desc shapeHandDesc;
	XMLData(shapeHandDesc, context(), xmlcontext());

	ShapePlanner *pShapePlanner = dynamic_cast<ShapePlanner*>(scene()->createObject(shapeHandDesc)); // throws
	if (pShapePlanner == NULL)
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
#endif // _GRASP_SHAPEPLANNER_MAIN_