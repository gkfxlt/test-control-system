#ifndef BASIC_SYSTEM_H_
#define BASIC_SYSTEM_H_
#include <codeit.hpp>


using namespace codeit::function;
using namespace codeit::model;

namespace codeit::system
{
	class NumList
	{
	public:
		int di_num;	
		int do_num;	
		int ai_num;
		int ao_num;
		int motor_num;
		int subsys_num;
	};

	//*************************创建控制器***********************//////
	auto createVrepController()->std::unique_ptr<codeit::controller::Controller>;
	auto createEcatController()->std::unique_ptr<codeit::controller::Controller>;
    auto createZeroErrEcatController()->std::unique_ptr<codeit::controller::Controller>;

    /*auto createComController(const NumList* num = nullptr, Size nrt_id=0, const std::string& name = "com_controller", UINT  portNo = 3, UINT  baud = CBR_19200, char  parity = 'N', UINT  databits = 8, \
		UINT  stopsbits = 1, DWORD dwCommEvents = EV_RXCHAR)->std::unique_ptr<codeit::controller::NrtController>;*/
	auto createSocketController(const NumList* num = nullptr, std::string name="socket_controller", std::string ip = "", std::string port="", controller::SocketMaster::TYPE type= controller::SocketMaster::TYPE::TCP, Size nrt_id=0)->std::unique_ptr<codeit::controller::NrtController>;
	auto createNrtControllerPool()->std::unique_ptr<core::ObjectPool<codeit::controller::NrtController>>;
	//*********************************************************//////

	//*************************创建模型************************//////
	auto createPumaModel(std::string name="model")->std::unique_ptr<codeit::model::Model>;
	auto createScaraModel(std::string name="model")->std::unique_ptr<codeit::model::Model>;
	auto createUrModel(std::string name="model")->std::unique_ptr<codeit::model::Model>;
	auto createIOModel(const NumList* num = nullptr, std::string name="model")->std::unique_ptr<codeit::model::IOModel>;
	
	auto createModelPool()->std::unique_ptr<core::ObjectPool<codeit::model::Model>>;
	auto createIOModelPool()->std::unique_ptr<core::ObjectPool<codeit::model::IOModel>>;
	////*********************************************************//////



	//*************************创建异常信息池************************//////
	auto createErrorInfoPool()->std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>>;
	////*********************************************************//////



	//*************************创建指令集************************//////
	auto createFuncRoot()->std::unique_ptr<codeit::function::FuncRoot>;
	////*********************************************************//////


	//*************************创建临时异常信息池************************//////
	auto tmpErrorInfoPool(core::ObjectPool<codeit::system::ErrorInfo>& infoPool, std::map<std::int32_t, std::string>& errMap)->void;
	////*********************************************************//////



	//*************************创建临时指令集************************//////
	auto tmpFuncRoot(codeit::function::FuncRoot& cmdRoot)->void;
	////*********************************************************//////

	auto createUserDataType(core::Calculator& cal)->void;
	auto createDefaultData(codeit::model::Model& model)->void;
}
#endif
