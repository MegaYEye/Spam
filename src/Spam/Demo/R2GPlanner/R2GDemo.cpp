/** @file PosePlanner.cpp
 * 
 * @author	Claudio Zito
 *
 * @version 1.0
 *
 */

#include <Spam/Demo/R2GPlanner/R2GDemo.h>

//------------------------------------------------------------------------------

using namespace golem;
using namespace grasp;
using namespace spam;

//------------------------------------------------------------------------------

void R2GDemo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	R2GPlanner::Desc::load(context, xmlcontext);

	try {
		xmlcontext = xmlcontext->getContextFirst("demo");
	}
	catch (const golem::MsgXMLParser& msg) { context.write("%s\n", msg.str().c_str()); }
}

//------------------------------------------------------------------------------

R2GDemo::R2GDemo(Scene &scene) : R2GPlanner(scene) {
}

R2GDemo::~R2GDemo() {
}

void R2GDemo::create(const Desc& desc) {
	desc.assertValid(Assert::Context("R2GDemo::Desc."));

	// create object
	R2GPlanner::create(desc); // throws
}

//------------------------------------------------------------------------------

void R2GDemo::render() const {
	R2GPlanner::render();
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return R2GDemo::Desc().main(argc, argv);
}

//------------------------------------------------------------------------------
