#include <iostream>
#include <aris.hpp>
#include<atomic>
#include<string>
#include<filesystem>


using namespace aris::model;


int main()
{

	core::Calculator c;

	auto& cs = aris::system::ControlSystem::instance();
	cs.resetController(createVrepController().release());
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.resetModelPool(createModelPool().release());
	cs.resetCmdRoot(createCmdRoot().release());
	cs.resetErrorInfoPool(createErrorInfoPool().release());
	cs.saveXmlFile(std::string("kaanh.xml"));
	cs.model().saveXmlFile(std::string("model.xml"));
	cs.model().pointPool().saveXmlFile(std::string("data.xml"));

	cs.interfacePool().add<aris::system::WebInterface>("ControlSock", "5866", core::Socket::TCP);
	cs.interfacePool().add<aris::system::StateRtInterface>("StateSock", "5867", core::Socket::TCP);
	cs.interfacePool().add<aris::system::WebInterface>("ErrorSock", "5868", core::Socket::TCP);


	cs.loadXmlFile(std::string("kaanh.xml"));
	cs.model().loadXmlFile(std::string("model.xml"));
	cs.model().pointPool().loadXmlFile(std::string("data.xml"));



	cs.init();
	cs.model().variablePool().add<aris::model::MatrixVariable>("fine", core::Matrix(1, 2, 0.0));

	auto& cal = cs.model().calculator();
	createUserDataType(cal);
	//cs.start();


#ifdef WIN32
	for (auto& m : cs.controller().motionPool())
	{
		dynamic_cast<aris::control::VrepMotor&>(m).setVirtual(true);
	}
#endif // WIN32

	cs.open();//启动socket


	cs.runCmdLine();
	


	return 0;
}