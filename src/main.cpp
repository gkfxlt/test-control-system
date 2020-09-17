#include <iostream>
#include <aris.hpp>
#include<atomic>
#include<string>
#include<filesystem>


using namespace std;
using namespace core;
using namespace aris::controller;
using namespace aris::model;
using namespace aris::system;

int main()
{
	auto& cs = aris::system::ControlSystem::instance();
	cs.resetController(createVrepController().release());
	cs.resetNrtControllerPool(createNrtControllerPool().release());
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.resetModelPool(createModelPool().release());
	cs.resetIOModelPool(createIOModelPool().release());
	cs.resetCmdRoot(createCmdRoot().release());
	cs.resetErrorInfoPool(createErrorInfoPool().release());

	cs.interfacePool().add<aris::system::ProgramWebInterface>("ControlSock", "5866", core::Socket::TCP);
	//cs.interfacePool().add<aris::cmdtarget::ProInterface>("ControlSock", "5866", core::Socket::WEB);

	cs.interfacePool().add<aris::system::StateRtInterface>("StateSock", "5867", core::Socket::TCP);
	cs.interfacePool().add<aris::system::WebInterface>("ErrorSock", "5868", core::Socket::TCP);
	cs.interfacePool().add<aris::system::ComInterface>("COM", 1, 9600);



	cs.saveXmlFile(std::string("kaanh.xml"));
	cs.model().saveXmlFile(std::string("model.xml"));
	cs.model().pointPool().saveXmlFile(std::string("data.xml"));


	cs.loadXmlFile(std::string("kaanh.xml"));
	cs.model().loadXmlFile(std::string("model.xml"));
	cs.model().pointPool().loadXmlFile(std::string("data.xml"));


	cs.init();
	cs.model().variablePool().add<aris::model::MatrixVariable>("fine", core::Matrix(1, 2, 0.0));

	auto& cal = cs.model().calculator();
	createUserDataType(cal);
	//cs.start();
	cs.setErrorinfoVer(1);

#ifdef WIN32
	for (auto& m : cs.controller().motionPool())
	{
		dynamic_cast<aris::controller::VrepMotor&>(m).setVirtual(true);
	}

	for (auto& m : cs.controller().ioPool())
	{
		dynamic_cast<aris::controller::EthercatIO&>(m).setVirtual(true);
	}
#endif // WIN32

	cs.open();//启动socket


	cs.runCmdLine();
	/*for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			cs.executeCmd(command_in);
		}
		catch (std::exception& e)
		{
			std::cout << e.what() << std::endl;
			LOG_ERROR << e.what() << std::endl;
		}
	}*/


	return 0;
}