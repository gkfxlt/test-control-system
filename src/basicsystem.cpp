#include "basicsystem.hpp"

using namespace aris::controller;
using namespace aris::cmdtarget;
using namespace aris::model;

namespace aris::system
{
	//*************************创建控制器***********************//////
	auto createVrepController()->std::unique_ptr<aris::controller::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
{
	std::unique_ptr<aris::controller::Controller> controller(new aris::controller::VrepController);

	for (Size i = 0; i < 6; ++i)
	{
		double zero_offset[6]
		{
			0,0*PI/ 2,0,0*PI/2,0,0
		};
		double pos_offset[6]
		{
			0,0,0,0,0,0
		};
		double pos_factor[6]
		{
			360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI
		};
		double max_pos[6]
		{
			170.0 / 360 * 2 * PI, 170.0 / 360 * 4 * PI,	170.0 / 360 * 2 * PI, 170.0 / 360 * 4 * PI, 117.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
		};
		double min_pos[6]
		{
			-170.0 / 360 * 2 * PI, -170.0 / 360 * 4 * PI, -170.0 / 360 * 2 * PI, -170.0 / 360 * 4 * PI, -117.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
		};
		double max_vel[6]
		{
			310.0 / 360 * 2 * PI, 240.0 / 360 * 2 * PI, 310.0 / 360 * 2 * PI, 250.0 / 360 * 2 * PI, 295.0 / 360 * 2 * PI, 500.0 / 360 * 2 * PI,
		};
		double max_acc[6]
		{
			15000.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 17500.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 25000.0 / 360 * 2 * PI,
		};
		double tor_const[6]
		{
			0.283 * 4808,0.283 * 4808,0.276 * 2546,0.226 * 1556,0.219 * 849,0.219 * 849
		};
		
		//" zero_offset=\"" + std::to_string(zero_offset[i]) + "\""
		//+std::to_string(min_pos[i]) +
		std::string joint_handle = "UR5_joint"+std::to_string(i+1);
		std::string xml_str =
			"<VrepMotor phy_id=\"" + std::to_string(i) + "\" handle=\""+joint_handle+"\""
			" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\" zero_offset=\"" + std::to_string(zero_offset[i])+"\""
			" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
			" tor_const=\"" + std::to_string(tor_const[i]) + "\""
			" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
			"	<SyncManagerPoolObject>"
			"		<SyncManager is_tx=\"false\"/>"
			"		<SyncManager is_tx=\"true\"/>"
			"		<SyncManager is_tx=\"false\">"
			"			<Pdo index=\"0x1600\" is_tx=\"false\">"
			"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
			"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"		<SyncManager is_tx=\"true\">"
			"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
			"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
			"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"	</SyncManagerPoolObject>"
			"</VrepMotor>";
		controller->slavePool().add<aris::controller::VrepMotor>().loadXmlStr(xml_str);
#ifndef ARIS_USE_ETHERCAT_SIMULATION
		//dynamic_cast<aris::control::VrepMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
#endif
	}
	
	//for (Size i = 0; i < 6; ++i)
	//{
	//	double pos_offset[6]
	//	{
	//		0,0,0,0,0,0
	//	};
	//	double pos_factor[6]
	//	{
	//		360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI
	//	};
	//	double max_pos[6]
	//	{
	//		170.0 / 360 * 2 * PI, 170.0 / 360 * 2 * PI,	170.0 / 360 * 2 * PI, 170.0 / 360 * 2 * PI, 117.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
	//	};
	//	double min_pos[6]
	//	{
	//		-170.0 / 360 * 2 * PI, -170.0 / 360 * 2 * PI, -170.0 / 360 * 2 * PI, -170.0 / 360 * 2 * PI, -117.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
	//	};
	//	double max_vel[6]
	//	{
	//		310.0 / 360 * 2 * PI, 240.0 / 360 * 2 * PI, 310.0 / 360 * 2 * PI, 250.0 / 360 * 2 * PI, 295.0 / 360 * 2 * PI, 500.0 / 360 * 2 * PI,
	//	};
	//	double max_acc[6]
	//	{
	//		1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1750.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 2500.0 / 360 * 2 * PI,
	//	};
	//	//+std::to_string(min_pos[i]) +
	//	std::string joint_handle = "joint_" + std::to_string(i + 1)+"#0";
	//	std::string xml_str =
	//		"<VrepMotor phy_id=\"" + std::to_string(i+6) + "\" handle=\"" + joint_handle + "\""
	//		" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
	//		" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
	//		" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
	//		"	<SyncManagerPoolObject>"
	//		"		<SyncManager is_tx=\"false\"/>"
	//		"		<SyncManager is_tx=\"true\"/>"
	//		"		<SyncManager is_tx=\"false\">"
	//		"			<Pdo index=\"0x1600\" is_tx=\"false\">"
	//		"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
	//		"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
	//		"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
	//		"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
	//		"				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
	//		"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
	//		"			</Pdo>"
	//		"		</SyncManager>"
	//		"		<SyncManager is_tx=\"true\">"
	//		"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
	//		"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
	//		"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
	//		"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
	//		"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
	//		"				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
	//		"			</Pdo>"
	//		"		</SyncManager>"
	//		"	</SyncManagerPoolObject>"
	//		"</VrepMotor>";
	//	controller->slavePool().add<aris::control::VrepMotor>().loadXmlStr(xml_str);
	//	//#ifndef ARIS_USE_ETHERCAT_SIMULATION
	//	//		dynamic_cast<aris::control::VrepMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
	//	//#endif
	//}
	
	
	//std::unique_ptr<aris::control::Controller> controller(robot::createControllerRokaeXB4());/*创建std::unique_ptr实例*/
	//std::cout << controller->xmlString() << endl;
	////ATI force sensor//
	//std::string xml_str =
	//	"<VrepSlave phy_id=\"6\">"
	//	"	<SyncManagerPoolObject>"
	//	"		<SyncManager is_tx=\"false\"/>"
	//	"		<SyncManager is_tx=\"true\"/>"
	//	"		<SyncManager is_tx=\"false\">"
	//	"			<Pdo index=\"0x1601\" is_tx=\"false\">"
	//	"				<PdoEntry name=\"Control_1\" index=\"0x7010\" subindex=\"0x01\" size=\"32\"/>"
	//	"				<PdoEntry name=\"Control_2\" index=\"0x7010\" subindex=\"0x02\" size=\"32\"/>"
	//	"			</Pdo>"
	//	"		</SyncManager>"
	//	"		<SyncManager is_tx=\"true\">"
	//	"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
	//	"				<PdoEntry name=\"Int_Input_Fx\" index=\"0x6000\" subindex=\"0x01\" size=\"32\"/>"
	//	"				<PdoEntry name=\"Int_Input_Fy\" index=\"0x6000\" subindex=\"0x02\" size=\"32\"/>"
	//	"				<PdoEntry name=\"Int_Input_Fz\" index=\"0x6000\" subindex=\"0x03\" size=\"32\"/>"
	//	"				<PdoEntry name=\"Int_Input_Mx\" index=\"0x6000\" subindex=\"0x04\" size=\"32\"/>"
	//	"				<PdoEntry name=\"Int_Input_My\" index=\"0x6000\" subindex=\"0x05\" size=\"32\"/>"
	//	"				<PdoEntry name=\"Int_Input_Mz\" index=\"0x6000\" subindex=\"0x06\" size=\"32\"/>"
	//	"				<PdoEntry name=\"Status_Code\" index=\"0x6010\" subindex=\"0x00\" size=\"32\"/>"
	//	"				<PdoEntry name=\"Sample_Counter\" index=\"0x6020\" subindex=\"0x00\" size=\"32\"/>"
	//	"			</Pdo>"
	//	"		</SyncManager>"
	//	"	</SyncManagerPoolObject>"
	//	"</VrepSlave>";
	//controller->slavePool().add<aris::control::VrepSlave>().loadXmlStr(xml_str);

	//for (Size i = 0; i < 4; ++i)
	//{
	//	double zero_offset[6]
	//	{
	//		0,0 * PI / 2,0,0 * PI / 2,0,0
	//	};
	//	double pos_offset[6]
	//	{
	//		0,0,0,0,0,0
	//	};
	//	double pos_factor[6]
	//	{
	//		360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI, 360 / 2 / PI
	//	};
	//	double max_pos[6]
	//	{
	//		170.0 / 360 * 2 * PI, 170.0 / 360 * 4 * PI,	170.0 / 360 * 2 * PI, 170.0 / 360 * 4 * PI, 117.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
	//	};
	//	double min_pos[6]
	//	{
	//		-170.0 / 360 * 2 * PI, -170.0 / 360 * 4 * PI, -170.0 / 360 * 2 * PI, -170.0 / 360 * 4 * PI, -117.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
	//	};
	//	double max_vel[6]
	//	{
	//		310.0 / 360 * 2 * PI, 240.0 / 360 * 2 * PI, 310.0 / 360 * 2 * PI, 250.0 / 360 * 2 * PI, 295.0 / 360 * 2 * PI, 500.0 / 360 * 2 * PI,
	//	};
	//	double max_acc[6]
	//	{
	//		15000.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 17500.0 / 360 * 2 * PI, 15000.0 / 360 * 2 * PI, 25000.0 / 360 * 2 * PI,
	//	};
	//	//" zero_offset=\"" + std::to_string(zero_offset[i]) + "\""
	//	//+std::to_string(min_pos[i]) +
	//	std::string joint_handle = "MTB_axis" + std::to_string(i + 1);
	//	std::string xml_str =
	//		"<VrepMotor phy_id=\"" + std::to_string(i) + "\" handle=\"" + joint_handle + "\""
	//		" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\" zero_offset=\"" + std::to_string(zero_offset[i]) + "\""
	//		" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
	//		" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
	//		"	<SyncManagerPoolObject>"
	//		"		<SyncManager is_tx=\"false\"/>"
	//		"		<SyncManager is_tx=\"true\"/>"
	//		"		<SyncManager is_tx=\"false\">"
	//		"			<Pdo index=\"0x1600\" is_tx=\"false\">"
	//		"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
	//		"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
	//		"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
	//		"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
	//		"				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
	//		"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
	//		"			</Pdo>"
	//		"		</SyncManager>"
	//		"		<SyncManager is_tx=\"true\">"
	//		"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
	//		"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
	//		"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
	//		"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
	//		"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
	//		"				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
	//		"			</Pdo>"
	//		"		</SyncManager>"
	//		"	</SyncManagerPoolObject>"
	//		"</VrepMotor>";
	//	controller->slavePool().add<aris::controller::VrepMotor>().loadXmlStr(xml_str);
	//	//#ifndef ARIS_USE_ETHERCAT_SIMULATION
	//	//		dynamic_cast<aris::control::VrepMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
	//	//#endif
	//}

	return controller;
}
	auto createEcatController()->std::unique_ptr<aris::controller::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
{
		std::unique_ptr<aris::controller::Controller> controller(new aris::controller::EthercatController);/*创建std::unique_ptr实例*/
		controller->slavePool().clear();	//清除slavePool中的元素，后面重新添加
		for (Size i = 0; i < 6; ++i)
		{
#ifdef WIN32
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
#endif
#ifdef UNIX
			double pos_offset[6]
			{
				0.0345045068966465,   0.151295566371175,   -0.181133422007823,   0.00569660673541914,   0.0119907348546894,   0.0908806917782888
			};
#endif
			double pos_factor[6]
			{
				131072.0 * 129.6 / 2 / PI, -131072.0 * 100 / 2 / PI, 131072.0 * 101 / 2 / PI, 131072.0 * 81.6 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 51 / 2 / PI
			};
			double max_pos[6]
			{
				170.0 / 360 * 2 * PI, 40.0 / 360 * 2 * PI,	150.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 125.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI
			};
			double min_pos[6]
			{
				-170.0 / 360 * 2 * PI, -165.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				230.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 375.0 / 360 * 2 * PI, 600.0 / 360 * 2 * PI
			};
			double max_acc[6]
			{
				1150.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1875.0 / 360 * 2 * PI, 3000.0 / 360 * 2 * PI
			};

			std::string xml_str =
				"<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x01\""
				" vendor_id=\"0x00000748\" revision_num=\"0x0002\" dc_assign_activate=\"0x0300\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"	<SyncManagerPoolObject>"
				"		<SyncManager is_tx=\"false\"/>"
				"		<SyncManager is_tx=\"true\"/>"
				"		<SyncManager is_tx=\"false\">"
				"			<Pdo index=\"0x1600\" is_tx=\"false\">"
				"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"target_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"dummy_byte\" index=\"0x5FFE\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"touch_probe_function\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"pos_offset\" index=\"0x60B0\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"tor_offset\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"		<SyncManager is_tx=\"true\">"
				"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
				"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"dummy_byte\" index=\"0x5FFF\" subindex=\"0x00\" size=\"8\"/>"
				"				<PdoEntry name=\"following_error\" index=\"0x60F4\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"digital_inputs\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
				"				<PdoEntry name=\"digital_inputs\" index=\"0x60B9\" subindex=\"0x00\" size=\"16\"/>"
				"				<PdoEntry name=\"digital_inputs\" index=\"0x60BA\" subindex=\"0x00\" size=\"32\"/>"
				"			</Pdo>"
				"		</SyncManager>"
				"	</SyncManagerPoolObject>"
				"</EthercatMotor>";

			controller->slavePool().add<aris::controller::EthercatMotor>().loadXmlStr(xml_str);
		}


		//ATI force sensor//
		std::string xml_strDI =
			"<EthercatDI phy_id=\"6\" product_code=\"0x26483053\""
			" vendor_id=\"0x00000732\" revision_num=\"0x00000001\" dc_assign_activate=\"0x00\">"
			"	<SyncManagerPoolObject>"
			"		<SyncManager is_tx=\"true\">"
			"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
			"				<PdoEntry name=\"Sample_Counter\" index=\"0x6020\" subindex=\"0x00\" size=\"8\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"	</SyncManagerPoolObject>"
			"</EthercatDI>";

		controller->slavePool().add<aris::controller::EthercatDI>().loadXmlStr(xml_strDI);


		std::string xml_strDO =
			"<EthercatDO phy_id=\"7\" product_code=\"0x26483053\""
			" vendor_id=\"0x00000732\" revision_num=\"0x00000001\" dc_assign_activate=\"0x00\">"
			"	<SyncManagerPoolObject>"
			"		<SyncManager is_tx=\"true\">"
			"			<Pdo index=\"0x1A00\" is_tx=\"true\">"
			"				<PdoEntry name=\"Sample_Counter\" index=\"0x6020\" subindex=\"0x00\" size=\"8\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"	</SyncManagerPoolObject>"
			"</EthercatDO>";

		controller->slavePool().add<aris::controller::EthercatDO>().loadXmlStr(xml_strDO);


		return controller;
}
	/*auto createComController(const NumList* numList,Size nrt_id, const std::string& name, UINT  portNo, UINT  baud, char  parity, UINT  databits, \
		UINT  stopsbits, DWORD dwCommEvents)->std::unique_ptr<aris::controller::NrtController>
	{
		static const NumList com_num_list = { 0 };
		numList = numList ? numList : &com_num_list;

		std::unique_ptr<aris::controller::NrtController> controller(new aris::controller::ComController(nrt_id, name,portNo,baud));
		
		int begin_index = 0;
		for (Size i = 0; i < numList->di_num; ++i)
		{
			std::string xml_str =
				"<ComDI phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</ComDI>";
			controller->slavePool().add<aris::controller::ComDI>().loadXmlStr(xml_str);
			begin_index++;
		}
		
		for (Size i = 0; i < numList->do_num; ++i)
		{
			std::string xml_str =
				"<ComDO phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</ComDO>";
			controller->slavePool().add<aris::controller::ComDO>().loadXmlStr(xml_str);
			begin_index++;
		}

		double res[6] = { 0.1,0.2,0.3,0.4,0.5,0.6 };
		for (Size i = 0; i < numList->ai_num; ++i)
		{
			std::string xml_str =
				"<ComAI phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\" resolution=\"" + std::to_string(res[i]) + "\">"
				"</ComAI>";
			controller->slavePool().add<aris::controller::ComAI>().loadXmlStr(xml_str);
			begin_index++;
		}
		for (Size i = 0; i < numList->ao_num; ++i)
		{
			std::string xml_str =
				"<ComAO phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\" resolution=\"" + std::to_string(res[i]) + "\">"
				"</ComAO>";
			controller->slavePool().add<aris::controller::ComAO>().loadXmlStr(xml_str);
			begin_index++;
		}


		for (Size i = 0; i < numList->motor_num; ++i)
		{
#ifdef WIN32
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
#endif
#ifdef UNIX
			double pos_offset[6]
			{
				0.0345045068966465,   0.151295566371175,   -0.181133422007823,   0.00569660673541914,   0.0119907348546894,   0.0908806917782888
			};
#endif
			double pos_factor[6]
			{
				131072.0 * 129.6 / 2 / PI, -131072.0 * 100 / 2 / PI, 131072.0 * 101 / 2 / PI, 131072.0 * 81.6 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 51 / 2 / PI
			};
			double max_pos[6]
			{
				170.0 / 360 * 2 * PI, 40.0 / 360 * 2 * PI,	150.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 125.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI
			};
			double min_pos[6]
			{
				-170.0 / 360 * 2 * PI, -165.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				230.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 375.0 / 360 * 2 * PI, 600.0 / 360 * 2 * PI
			};
			double max_acc[6]
			{
				1150.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1875.0 / 360 * 2 * PI, 3000.0 / 360 * 2 * PI
			};

			std::string xml_str =
				"<ComMotor phy_id=\"" + std::to_string(begin_index) + "\" "
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"</ComMotor>";

			controller->slavePool().add<aris::controller::ComMotor>().loadXmlStr(xml_str);
			begin_index++;
		}

		for (Size i = 0; i < numList->subsys_num; ++i)
		{
			std::string xml_str =
				"<ComSubSystem phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</ComSubSystem>";
			controller->slavePool().add<aris::controller::ComSubSystem>().loadXmlStr(xml_str);
			begin_index++;
		}

		return controller;

	}*/
	auto createSocketController(const NumList* numList,std::string name, std::string ip, std::string port, SocketMaster::TYPE type, Size nrt_id)->std::unique_ptr<aris::controller::NrtController>
	{
		static const NumList socket_num_list= { 0 };
		numList = numList ? numList : &socket_num_list;


		std::unique_ptr<aris::controller::NrtController> controller(new aris::controller::SocketController(name,ip,port,type,nrt_id));

		int begin_index = 0;
		for (Size i = 0; i < numList->di_num; ++i)
		{
			std::string xml_str =
				"<SocketDI phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</SocketDI>";
			controller->slavePool().add<aris::controller::SocketDI>().loadXmlStr(xml_str);
			begin_index++;
		}
	
		for (Size i = 0; i < numList->do_num; ++i)
		{
			std::string xml_str =
				"<SocketDO phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</SocketDO>";
			controller->slavePool().add<aris::controller::SocketDO>().loadXmlStr(xml_str);
			begin_index++;
		}


		double res[6] = { 0.1,0.2,0.3,0.4,0.5,0.6 };
		
		for (Size i = 0; i < numList->ai_num; ++i)
		{
			std::string xml_str =
				"<SocketAI phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\" resolution=\"" + std::to_string(res[i]) + "\">"
				"</SocketAI>";
			controller->slavePool().add<aris::controller::SocketAI>().loadXmlStr(xml_str);
			begin_index++;
		}

		for (Size i = 0; i < numList->ao_num; ++i)
		{
			std::string xml_str =
				"<SocketAO phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\" resolution=\"" + std::to_string(res[i]) + "\">"
				"</SocketAO>";
			controller->slavePool().add<aris::controller::SocketAO>().loadXmlStr(xml_str);
			begin_index++;
		}

		for (Size i = 0; i < numList->motor_num; ++i)
		{
#ifdef WIN32
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
#endif
#ifdef UNIX
			double pos_offset[6]
			{
				0.0345045068966465,   0.151295566371175,   -0.181133422007823,   0.00569660673541914,   0.0119907348546894,   0.0908806917782888
			};
#endif
			double pos_factor[6]
			{
				131072.0 * 129.6 / 2 / PI, -131072.0 * 100 / 2 / PI, 131072.0 * 101 / 2 / PI, 131072.0 * 81.6 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 51 / 2 / PI
			};
			double max_pos[6]
			{
				170.0 / 360 * 2 * PI, 40.0 / 360 * 2 * PI,	150.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 125.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI
			};
			double min_pos[6]
			{
				-170.0 / 360 * 2 * PI, -165.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI, -125.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				230.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 300.0 / 360 * 2 * PI, 375.0 / 360 * 2 * PI, 600.0 / 360 * 2 * PI
			};
			double max_acc[6]
			{
				1150.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1875.0 / 360 * 2 * PI, 3000.0 / 360 * 2 * PI
			};

			std::string xml_str =
				"<SocketMotor phy_id=\"" + std::to_string(begin_index) + "\" "
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"</SocketMotor>";

			controller->slavePool().add<aris::controller::SocketMotor>().loadXmlStr(xml_str);
			begin_index++;
		}

		for (Size i = 0; i < numList->subsys_num; ++i)
		{
			std::string xml_str =
				"<SocketSubSystem phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</SocketSubSystem>";
			controller->slavePool().add<aris::controller::SocketSubSystem>().loadXmlStr(xml_str);
			begin_index++;
		}

		return controller;

	}
	
	auto createNrtControllerPool()->std::unique_ptr<core::ObjectPool<aris::controller::NrtController>>
	{
		std::unique_ptr<core::ObjectPool<aris::controller::NrtController>> nrtControllerPool(new core::ObjectPool<aris::controller::NrtController>);
		
		int i = 0;
		NumList num0{ 6,6,6,6,6,1 };
		auto sock1 = createSocketController(&num0,"state", "", "6001", SocketMaster::TYPE::TCP,i++).release();
		NumList num1{ 0,0,0,0,0,1 };
		auto sock0 = createSocketController(&num1,"command", "", "6000", SocketMaster::TYPE::TCP,i++).release();
		//auto com0 = createComController(&num0,i++,"com", 4, 9600).release();


		//nrtControllerPool->add(com0);
		nrtControllerPool->add(sock1);
		nrtControllerPool->add(sock0);
		

		return std::move(nrtControllerPool);


	}
	//*********************************************************//////

	//*************************创建模型************************//////
	auto createPumaModel(std::string name)->std::unique_ptr<aris::model::Model>
{
	aris::model::PumaParam param;
	param.d1 = 0.322;
	param.a1 = 0.088;
	param.a2 = 0.46;
	param.d3 = 0.0;
	param.a3 = 0.0418;
	param.d4 = 0.4342;

	param.tool0_pe[2] = 0.27544;

	/*param.iv_vec =
	{
		{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.59026333537827,   0.00000000000000,   0.00000000000000,   0.00000000000000 },
		{ 0.00000000000000, -0.02551872200978,   0.00000000000000,   3.05660683326413,   2.85905166943306,   0.00000000000000,   0.00000000000000, -0.00855352993039, -0.09946674483372, -0.00712210734359 },
		{ 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.02733022277747,   0.00000000000000,   0.37382629693302,   0.00000000000000,   0.00312006493276, -0.00578410451516,   0.00570606128540 },
		{ 0.00000000000000,   1.06223330086669,   0.00000000000000,   0.00311748242960,   0.00000000000000,   0.24420385558544,   0.24970286555981,   0.00305759215246, -0.66644096559686,   0.00228253380852 },
		{ 0.00000000000000,   0.05362286897910,   0.00528925153464, -0.00842588023014,   0.00128498153337, -0.00389810210572,   0.00000000000000, -0.00223677867576, -0.03365036368035, -0.00415647085627 },
		{ 0.00000000000000,   0.00000000000000,   0.00066049870832,   0.00012563800445, -0.00085124094833,   0.04209529937135,   0.04102481443654, -0.00067596644891,   0.00017482449876, -0.00041025776053 },
	};*/

	param.mot_frc_vec =
	{
		{ 1, 1, 1.00000000000000 },
		{ 1, 1, 1 },
		{ 1, 1, 1 },
		{ 1, 1, 1 },
		{ 1, 1, 1 },
		{ 1, 1, 1 },
	};

	auto model = aris::model::createModelPuma(param,name);


	return std::move(model);
}
	auto createScaraModel(std::string name)->std::unique_ptr<aris::model::Model>
	{
	aris::model::ScaraParam param;
	param.a2 = 0.467;
	param.a3 = 0.4005;
	param.d3 = 0.08;
	auto model = aris::model::createModelScara(param, name);

	return std::move(model);
	}
	auto createUrModel(std::string name)->std::unique_ptr<aris::model::Model>
{
		aris::model::UrParam param;
		param.L1 = 0.425;
		param.L2 = 0.39225;
		param.W1 = 0.13585 - 0.1197 + 0.093;
		param.W2 = 0.0823;
		param.H1 = 0.089159;
		param.H2 = 0.09465;
		
		auto model = aris::model::createModelUr(param, name);

		return std::move(model);
};
	auto createIOModel(const NumList* numList, std::string name)->std::unique_ptr<aris::model::IOModel>
	{
		static const NumList model_num_list = { 0 };
		numList = numList ? numList : &model_num_list;
		std::unique_ptr<aris::model::IOModel> iomodel(new aris::model::IOModel(name));
		
		for (Size i = 0; i < numList->di_num; ++i)
			iomodel->ioPool().add<model::DI>(name + "_di" +std::to_string(i));
		
		for (Size i = 0; i < numList->do_num; ++i)
			iomodel->ioPool().add<model::DO>(name + "_do" + std::to_string(i));
		
		for (Size i = 0; i < numList->ao_num; ++i)
			iomodel->ioPool().add<model::AI>(name + "_ai" + std::to_string(i));
		
		for (Size i = 0; i < numList->ai_num; ++i)
			iomodel->ioPool().add<model::AO>(name + "_ao" + std::to_string(i));
		
		/*for (Size i = 0; i < numList->motor_num; ++i)
			iomodel->motorPool().add<model::Motion>(name + "motor" + std::to_string(i));
		*/

		for (Size i = 0; i < numList->subsys_num; ++i)
			iomodel->subsysPool().add<model::SubSysElement>(name + "_subsys" + std::to_string(i));
		
		return std::move(iomodel);
	}

	auto createModelPool()->std::unique_ptr<core::ObjectPool<aris::model::Model>>
{
	std::unique_ptr<core::ObjectPool<aris::model::Model>> modelPool(new core::ObjectPool<aris::model::Model>);
	auto model0 = createPumaModel("MJ08").release();
	auto model1= createUrModel("UR5").release();
	auto model2 = createScaraModel("Scara").release();
	modelPool->add(model1);

	//modelPool->add(model1);
	for(Size i=0;i<modelPool->size();i++)
		createDefaultData(modelPool->at(i));

	return std::move(modelPool);


}
	auto createIOModelPool()->std::unique_ptr<core::ObjectPool<aris::model::IOModel>>
	{
		std::unique_ptr<core::ObjectPool<aris::model::IOModel>> iomodelPool(new core::ObjectPool<aris::model::IOModel>);
		NumList num0{ 6,6,6,6,6,1 };
		auto iomodel0 = createIOModel(&num0,"com0").release();
		NumList num1{ 0,0,0,0,0,1 };
		auto iomodel1 = createIOModel(&num1,"sock1").release();
		auto iomodel2 = createIOModel(&num0,"sock0").release();
		iomodelPool->add(iomodel0);
		iomodelPool->add(iomodel2);
		iomodelPool->add(iomodel1);

		NumList num2{ 1,1,1,1,0,0 };
		auto iomodel3 = createIOModel(&num1, "ecat").release();
		iomodelPool->add(iomodel3);

		return std::move(iomodelPool);
	}
	////*********************************************************//////


	//*************************创建异常信息池************************//////
	auto createErrorInfoPool()->std::unique_ptr<core::ObjectPool<aris::system::ErrorInfo>>
{
	std::unique_ptr<core::ObjectPool<aris::system::ErrorInfo>> errorinfoPool(new core::ObjectPool<aris::system::ErrorInfo>);

	/////慎用冒号，它要放在句尾，后面跟数字代表轴号,后面的内容都会在多语言中显示出来

	////****对errorinfo本身的异常报错
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unknown errorinfo", -2, "ERROR", "该异常信息未注册", "unregistered errorinfo");
	///****对errorinfo本身的异常报错--------------------------------------




	///****关节参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("input pos beyond range-joint", -2, "ERROR", "输入位置超限", "input position beyond range-joint");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("input vel beyond range-joint", -2, "ERROR", "输入速度超限", "input velocity beyond range-joint");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("input acc beyond range-joint", -2, "ERROR", "输入加速度超限", "input acc beyond range-joint");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("input dec beyond range-joint", -2, "ERROR", "输入减速度超限", "input dec beyond range-joint");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("acc dimension mismatch-joint", -2, "ERROR", "加速度维数不匹配", "acc dimension mismatch-joint");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("dec dimension mismatch-joint", -2, "ERROR", "减速度维数不匹配", "dec dimension mismatch-joint");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("jerk dimension mismatch-joint", -2, "ERROR", "加加速度维数不匹配", "jerk dimension mismatch-joint");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pos dimension mismatch-joint", -2, "ERROR", "位置维数不匹配", "pos dimension mismatch-joint");
	///****关节参数-----------------------------------------------
	

	///****目标位置
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported displacement dimension-robottarget", -2, "ERROR", "不支持的位移量纲", "unsupported displacement dimension");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported angle dimension-robottarget", -2, "ERROR", "不支持的角度量纲", "unsupported angle dimension");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("euler angle type invalidiy-robottarget", -2, "ERROR", "欧拉角类型异常", "euler angle type invalidiy");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pe dimension mismatch-robottarget", -2, "ERROR", "位置-欧拉角维数不匹配", "pe dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pm dimension mismatch-robottarget", -2, "ERROR", "旋转矩阵维数不匹配", "pm dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pq dimension mismatch-robottarget", -2, "ERROR", "位置-四元数维数不匹配", "pq dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pa dimension mismatch-robottarget", -2, "ERROR", "位置-轴角维数不匹配", "pa dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported pose input-robottarget", -2, "ERROR", "不支持的位姿输入", "unsupported pose input");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("input joint dimension mismatch-robottarget", -2, "ERROR", "输入的关节角度维数不匹配", "input joint dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("forward kinematic failed-robottarget", -2, "ERROR", "运动学正解失败", "forward kinematic failed");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param not exist-robottarget", -2, "ERROR", "robottarget参数不存在", "robottarget param not exist");
	///****目标位置----------------------------------------------

	///****中间位置
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported displacement dimension-midrobottarget", -2, "ERROR", "不支持的位移量纲", "unsupported displacement dimension");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported angle dimension-midrobottarget", -2, "ERROR", "不支持的角度量纲", "unsupported angle dimension");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("euler angle type invalidiy-midrobottarget", -2, "ERROR", "欧拉角类型异常", "euler angle type invalidiy");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pe dimension mismatch-midrobottarget", -2, "ERROR", "位置-欧拉角维数不匹配", "pe dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pm dimension mismatch-midrobottarget", -2, "ERROR", "旋转矩阵维数不匹配", "pm dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pq dimension mismatch-midrobottarget", -2, "ERROR", "位置-四元数维数不匹配", "pq dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pa dimension mismatch-midrobottarget", -2, "ERROR", "位置-轴角维数不匹配", "pa dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported pose input-midrobottarget", -2, "ERROR", "不支持的位姿输入", "unsupported pose input");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("input joint dimension mismatch-midrobottarget", -2, "ERROR", "输入的关节角度维数不匹配", "input joint dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("forward kinematic failed-midrobottarget", -2, "ERROR", "运动学正解失败", "forward kinematic failed");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param not exist-midrobottarget", -2, "ERROR", "robottarget参数不存在", "robottarget param not exist");
	///****中间位置-----------------------------------------------

	///****工具坐标系
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported displacement dimension-tool", -2, "ERROR", "不支持的位移量纲", "unsupported displacement dimension");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported angle dimension-tool", -2, "ERROR", "不支持的角度量纲", "unsupported angle dimension");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("euler angle type invalidiy-tool", -2, "ERROR", "欧拉角类型异常", "euler angle type invalidiy");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pe dimension mismatch-tool", -2, "ERROR", "位置-欧拉角维数不匹配", "pe dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pm dimension mismatch-tool", -2, "ERROR", "旋转矩阵维数不匹配", "pm dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pq dimension mismatch-tool", -2, "ERROR", "位置-四元数维数不匹配", "pq dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pa dimension mismatch-tool", -2, "ERROR", "位置-轴角维数不匹配", "pa dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported pose input-tool", -2, "ERROR", "不支持的位姿输入", "unsupported pose input");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param not exist-tool", -2, "ERROR", "tool参数不存在", "tool param not exist");

	///****工具坐标系---------------------------------------------

	
	///****Wobj坐标系
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported displacement dimension-wobj", -2, "ERROR", "不支持的位移量纲", "unsupported displacement dimension");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported angle dimension-wobj", -2, "ERROR", "不支持的角度量纲", "unsupported angle dimension");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("euler angle type invalidiy-wobj", -2, "ERROR", "欧拉角类型异常", "euler angle type invalidiy");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pe dimension mismatch-wobj", -2, "ERROR", "位置-欧拉角维数不匹配", "pe dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pm dimension mismatch-wobj", -2, "ERROR", "旋转矩阵维数不匹配", "pm dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pq dimension mismatch-wobj", -2, "ERROR", "位置-四元数维数不匹配", "pq dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pa dimension mismatch-wobj", -2, "ERROR", "位置-轴角维数不匹配", "pa dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported pose input-wobj", -2, "ERROR", "不支持的位姿输入", "unsupported pose input");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param not exist-wobj", -2, "ERROR", "wobj参数不存在", "wobj param not exist");

	///****Wobj坐标系--------------------------------------------

	///****zone参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param not exist-zone", -2, "ERROR", "zone参数不存在", "zone param not exist");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("out of range-zone", -2, "ERROR", "zone参数越界", "zone out of range");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("dimension mismatch-zone", -2, "ERROR", "zone参数维数不匹配", "zone dimension mismatch");
	///****zone参数-----------------------------------------------
	
	///****speed参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param not exist-speed", -2, "ERROR", "速度参数不存在", "speed param not exist");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("dimension mismatch-speed", -2, "ERROR", "速度参数维数不匹配", "speed dimension mismatch");
	///****speed参数---------------------------------------------

	///****load参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param not exist-load", -2, "ERROR", "load参数不存在", "load param not exist");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("dimension mismatch-load", -2, "ERROR", "load参数维数不匹配", "load dimension mismatch");
	///****load参数----------------------------------------------


	///****jointtarget参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param not exist-jointtarget", -2, "ERROR", "关节目标参数不存在", "jointtarget param not exist");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("dimension mismatch-jointtarget", -2, "ERROR", "关节目标参数维数不匹配", "jointtarget dimension mismatch");
	///****jointarget参数--------------------------------------

	///****速度、加速度、减速度、加加速度参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("acc is negative", -2, "ERROR", "加速度为负数", "acc is negative");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("dec is positive", -2, "ERROR", "减速度为正数", "dec is positive");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("jerk is negative", -2, "ERROR", "加加速度为负数", "jerk is negative");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("jmax is negative", -2, "ERROR", "加加速度为负数", "jmax is negative");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("jmin is positive", -2, "ERROR", "减减速度为正数", "jmin is positive");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("angular_jmax is negative", -2, "ERROR", "角加加速度为负数", "angular_jmax is negative");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("angular_jmin is positive", -2, "ERROR", "角减减速度为正数", "angular_jmin is positive");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("angular_acc is negative", -2, "ERROR", "角加速度为负数", "angular_acc is negative");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("angular_dec is positive", -2, "ERROR", "角减速度为正数", "angular_dec is positive");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("angular_jerk is negative", -2, "ERROR", "角加加速度为负数", "angular_jerk is negative");
	///****速度、加速度、减速度、加加速度参数------------------------------------------


	///****示教点参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param already exist-teachpoint", -2, "ERROR", "该示教点名称已存在", "param already exist-teachpoint");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param not exist-teachpoint", -2, "ERROR", "该示教点名称不存在", "param not exist-teachpoint");
	///****示教点参数------------------------------------------

	///****变量操作参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported displacement dimension-definevar", -2, "ERROR", "不支持的位移量纲", "unsupported displacement dimension-definevar");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported angle dimension-definevar", -2, "ERROR", "不支持的角度量纲", "unsupported angle dimension-definevar");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("dimension mismatch-definespeed", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-definespeed");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("dimension mismatch-definezone", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-definezone");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("dimension mismatch-defineload", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-defineload");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("dimension mismatch-definejointtarget", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-definejointtarget");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("type error-definevar", -2, "ERROR", "类型错误", "type error-definevar");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("euler angle type invalidiy", -2, "ERROR", "欧拉角类型异常", "euler angle type invalidiy");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pe dimension mismatch", -2, "ERROR", "位置-欧拉角维数不匹配", "pe dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pm dimension mismatch", -2, "ERROR", "旋转矩阵维数不匹配", "pm dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pq dimension mismatch", -2, "ERROR", "位置-四元数维数不匹配", "pq dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pa dimension mismatch", -2, "ERROR", "位置-轴角维数不匹配", "pa dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported pose input", -2, "ERROR", "不支持的位姿输入", "unsupported pose input");
	///****变量操作参数------------------------------------------

	///****参数操作参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported displacement dimension-saveparam", -2, "ERROR", "不支持的位移量纲", "unsupported displacement dimension-saveparam");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unsupported angle dimension-saveparam", -2, "ERROR", "不支持的角度量纲", "unsupported angle dimension-saveparam");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("dimension mismatch-savespeed", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-savespeed");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("dimension mismatch-savezone", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-savezone");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("dimension mismatch-saveload", -2, "ERROR", "不支持的角度量纲", "dimension mismatch-saveload");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param already exist-saveparam", -2, "ERROR", "该示教点名称已存在", "param already exist-saveparam");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("type error-saveparam", -2, "ERROR", "类型错误", "type error-saveparam");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param not exist-deleteparam", -2, "ERROR", "该示教点名称已存在", "param not exist-deleteparam");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("type error-deleteparam", -2, "ERROR", "类型错误", "type error-deleteparam");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param not exist-renameparam", -2, "ERROR", "该示教点名称已存在", "param not exist-renameparam");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("new param already exist-renameparam", -2, "ERROR", "该示教点名称已存在", "new param already exist-renameparam");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("type error-renameparam", -2, "ERROR", "类型错误", "type error-renameparam");
	///****参数操作参数------------------------------------------

	///****JogJ、JogC参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("last_count is negative", -2, "ERROR", "持续时间为负数", "last_count is negative");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("error direction", -2, "ERROR", "jog方向设置错误", "error direction");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("joint beyond range", -2, "ERROR", "关节选择超限", "joint beyond range");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("coordinate error", -2, "ERROR", "坐标系设定错误", "coordinate error");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("motion type error", -2, "ERROR", "运动类型设定错误", "motion type error");
	///****JogJ、JogC参数--------------------------------------

	///****ServoJ参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("time is negative", -2, "ERROR", "时间间隔设为负数", "time is negative");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("acc dimension mismatch-servoJ", -2, "ERROR", "加速度维数不匹配", "acc dimension mismatch-servoJ");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("look_ahead_time is negative", -2, "ERROR", "前瞻时间设为负数", "look_ahead_time is negative");
	///****ServoJ参数--------------------------------------


	///****IO 指令参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("DO name not exist", -2, "ERROR", "do 不存在", "DO name not exist");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("DI name not exist", -2, "ERROR", "di 不存在", "DI name not exist");
	///****IO 指令参数--------------------------------------


	///****MoveS,Calibrator参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pq set dimension mismatch", -2, "ERROR", "pq参数集维数不匹配", "pq set dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pm set dimension mismatch", -2, "ERROR", "pm参数集维数不匹配", "pm set dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pa set dimension mismatch", -2, "ERROR", "pa参数集维数不匹配", "pa set dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("pe set dimension mismatch", -2, "ERROR", "pe参数集维数不匹配", "pe set dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("joint set dimension mismatch", -2, "ERROR", "关节角度参数集维数不匹配", "joint set dimension mismatch");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("forward kinematic pm failed", -2, "ERROR", "正向运动学获取pm失败", "forward kinematic pm failed");

	errorinfoPool->add<aris::system::ErrorInfo>\
		("regressor matrix not full rank", -2, "ERROR", "回归矩阵不满秩", "regressor matrix not full rank");

	///****MoveS,Calibrator参数--------------------------------------


	///****其他指令参数
	errorinfoPool->add<aris::system::ErrorInfo>\
		("count is negative", -2, "ERROR", "持续时间为负数", "count is negative");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("program_rate", -2, "ERROR", "程序速率为负数", "program_rate");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("cs is running", -2, "ERROR", "实时线程在运行中", "cs is running, please stop the cs using cs_stop!");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("failed to stop server", -2, "ERROR", "停止服务器失败", "failed to stop server, because it is not running");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("failed to start server", -2, "ERROR", "启动服务器失败", "failed to start server, because it is already started");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("no model", -2, "ERROR", "该模型不存在", "no model");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("time is negative", -2, "ERROR", "等待时间为负数", "wait time is negative");

	///****其他指令参数--------------------------------------

	///****指令解析异常，源于command.cpp
	errorinfoPool->add<aris::system::ErrorInfo>\
		("brace not pair", -2, "ERROR", "没有成对的括号", "brace not pair");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("empty command string", -2, "ERROR", "字符串命令为空", "invalid command string: please at least contain a word");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("invalid command name", -2, "ERROR", "无效指令", "server does not have this command");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param should not start with '='", -2, "ERROR", "参数不能以=开始", "param should not start with '='");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("symbol '-' must be followed by an abbreviation of param", -2, "ERROR", "符号'-'后必须用缩写", "symbol '-' must be followed by an abbreviation of param");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("symbol '--' must be followed by a full name of param", -2, "ERROR", "符号'--'后必须用完整名称", "symbol '--' must be followed by a full name of param");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("param start with single '-' must be an abbreviation", -2, "ERROR", "以'-'开头的参数必须是缩写", "param start with single '-' must be an abbreviation");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("not a abbreviation of any valid param", -2, "ERROR", "不存在这样的缩写", "not a abbreviation of any valid param");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("not a valid param", -2, "ERROR", "不是一个有效的指令参数", "not a valid param of command");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("failed to find default param in command", -2, "ERROR", "指令中找不到默认参数", "failed to find default param in command");

	
	///****指令解析异常，源于command.cpp--------------------------------------

	///****// step 3.  execute //
	errorinfoPool->add<aris::system::ErrorInfo>\
		("server in error", -2, "ERROR", "服务器处于错误状态", "server in error, use cl to clear");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("server not started", -2, "ERROR", "服务器未启动", "server not started, use cs_start to start");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("command pool is full", -2, "ERROR", "指令缓冲满了", "command pool is full");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("failed to get current TARGET", -2, "ERROR", "获取当前目标失败", "failed to get current TARGET, because ControlServer is not running");
	///****// step 3.  execute //--------------------------------------
	
	///****// Motion Check //
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion is not in OPERATION_ENABLE mode", -2, "ERROR", "轴没有使能", "Motion is not in OPERATION_ENABLE mode");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion target position beyond MAX", -2, "ERROR", "轴的位置指令超过最大值", "Motion target position beyond MAX");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion target position beyond MIN", -2, "ERROR", "轴的位置指令超过最小值", "Motion target position beyond MIN");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion target position NOT CONTINUOUS", -2, "ERROR", "轴的位置指令一阶不连续", "Motion target position NOT CONTINUOUS");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion target position NOT SECOND CONTINUOUS", -2, "ERROR", "轴的位置指令二阶不连续", "Motion target position NOT SECOND CONTINUOUS");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion target position has FOLLOW ERROR", -2, "ERROR", "轴存在位置跟踪误差", "Motion target position has FOLLOW ERROR");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion target velocity beyond MAX", -2, "ERROR", "轴的速度指令超过最大值", "Motion target velocity beyond MAX");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion target velocity beyond MIN", -2, "ERROR", "轴的速度指令超过最小值", "Motion target velocity beyond MIN");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion target velocity NOT CONTINUOUS", -2, "ERROR", "轴的速度指令一阶不连续", "Motion target velocity NOT CONTINUOUS");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion target velocity has FOLLOW ERROR", -2, "ERROR", "轴的速度指令存在跟踪误差", "Motion target velocity has FOLLOW ERROR");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion actual position beyond MAX", -2, "ERROR", "轴的实际位置超过最大值", "Motion actual position beyond MAX");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion actual position beyond MIN", -2, "ERROR", "轴的实际位置超过最小值", "Motion actual position beyond MIN");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion actual velocity beyond MAX", -2, "ERROR", "轴的实际速度超过最大值", "Motion actual velocity beyond MAX");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion actual velocity beyond MIN", -2, "ERROR", "轴的实际速度超过最小值", "Motion actual velocity beyond MIN");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion actual velocity NOT CONTINUOUS", -2, "ERROR", "轴的实际速度一阶不连续", "Motion actual velocity NOT CONTINUOUS");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Motion MODE INVALID", -2, "ERROR", "轴的模式无效", "Motion MODE INVALID");
	///****// Motion Check //--------------------------------------


	///****// Master通信 //
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Master Lost Connection with", -2, "ERROR", "主站失去连接", "Master Lost Connection with");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Master failed start with", -2, "ERROR", "主站启动失败", "Master failed start with");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("master already running", -2, "ERROR", "主站已经启动", "master already running, so cannot start");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("master is not running", -2, "ERROR", "主站已经启动", "master is not running, so can't stop");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("master cannot set control strategy", -2, "ERROR", "主站已经启动", "master already running, cannot set control strategy");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("master cannot set control strategy", -2, "ERROR", "主站已经启动", "master already running, cannot set control strategy");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("phy id already exists", -2, "ERROR", "主站已经启动", "phy id already exists");


	errorinfoPool->add<aris::system::ErrorInfo>\
		("rt_task_create failed", -2, "ERROR", "实时任务创建失败", "rt_task_create failed");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("aris_rt_task_join failed", -2, "ERROR", "实时任务创建失败", "aris_rt_task_join failed");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("aris_nrt_task_join failed", -2, "ERROR", "实时任务创建失败", "aris_nrt_task_join failed");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("rt_task_start failed", -2, "ERROR", "实时任务启动失败", "rt_task_start failed");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Com controller initport fail", -2, "ERROR", "串口控制器端口初始化失败", "Com controller initport fail");
	///****// Master通信错误 //--------------------------------------


	///****// Interface错误 //
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Socket can't WSAstartup", -2, "ERROR", "Socket不能作为主站启动", "Socket can't Start as server, because it can't WSAstartup");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Socket has empty port", -2, "ERROR", "Socket端口号为空", "Socket has empty port");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Socket can't bind", -2, "ERROR", "Socket端口号为空", "Socket can't Start as server, because it can't bind");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Socket can't listen", -2, "ERROR", "Socket端口号为空", "Socket can't Start as server, because it can't listen");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Socket empty ip address", -2, "ERROR", "Socket端口号为空", "Socket can't connect, because it empty ip address");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Socket empty port", -2, "ERROR", "Socket端口号为空", "Socket can't connect, because it empty port");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Socket is busy now", -2, "ERROR", "Socket端口号为空", "Socket can't connect, because it is busy now, please close it");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Socket can't connect", -2, "ERROR", "Socket端口号为空", "Socket can't connect, because can't connect");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("socket setsockopt TCP_USER_TIMEOUT FAILED", -2, "ERROR", "主站已经启动", "socket setsockopt TCP_USER_TIMEOUT FAILED");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("socket setsockopt SO_KEEPALIVE FAILED", -2, "ERROR", "主站已经启动", "socket setsockopt SO_KEEPALIVE FAILED");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("setsockopt failed", -2, "ERROR", "主站已经启动", "setsockopt failed: SO_REUSEADDR");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Com interface open listen thread fail", -2, "ERROR", "主站已经启动", "Com interface open listen thread fail");
	errorinfoPool->add<aris::system::ErrorInfo>\
		("Com interface initPort fail", -2, "ERROR", "主站已经启动", "Com interface initPort fail");

	///****// Interface错误 //--------------------------------------


	///****// executerRT(),运动算法错误，需加入errMsgMap //
	auto& cs = ControlSystem::instance();
	auto& errMap = cs.errorMap();
	errorinfoPool->add<aris::system::ErrorInfo>\
		("unregistered key", -1000, "ERROR", "未注册的键值", "unregistered key");
	errMap.insert(pair<std::int32_t, string>(-1000, "unregistered key"));

	errorinfoPool->add<aris::system::ErrorInfo>\
		("plan over time", CmdBase::RetStatus::PLAN_OVER_TIME, "ERROR", "规划超时", "plan over time");
	errMap.insert(pair<std::int32_t, string>(CmdBase::RetStatus::PLAN_OVER_TIME, "plan over time"));
	
	errorinfoPool->add<aris::system::ErrorInfo>\
		("forward kinematic failed", CmdBase::RetStatus::FORWARD_KINEMATIC_POSITION_FAILED, "ERROR", "正解失败", "forward kinematic failed");
	errMap.insert(pair<std::int32_t, string>(CmdBase::RetStatus::FORWARD_KINEMATIC_POSITION_FAILED, "forward kinematic failed"));
	
	errorinfoPool->add<aris::system::ErrorInfo>\
		("inverse kinematic failed", CmdBase::RetStatus::INVERSE_KINEMATIC_POSITION_FAILED, "ERROR", "逆解失败", "inverse kinematic failed");
	errMap.insert(pair<std::int32_t, string>(CmdBase::RetStatus::INVERSE_KINEMATIC_POSITION_FAILED, "inverse kinematic failed"));

	errorinfoPool->add<aris::system::ErrorInfo>\
		("wrist singularity", CmdBase::RetStatus::WRIST_SINGULARITY, "ERROR", "腕部奇异点", "wrist singularity");
	errMap.insert(pair<std::int32_t, string>(CmdBase::RetStatus::WRIST_SINGULARITY, "wrist singularity"));

	errorinfoPool->add<aris::system::ErrorInfo>\
		("shoulder singularity", CmdBase::RetStatus::SHOULDER_SINGULARITY, "ERROR", "肩部奇异点", "shoulder singularity");
	errMap.insert(pair<std::int32_t, string>(CmdBase::RetStatus::SHOULDER_SINGULARITY, "shoulder singularity"));

	errorinfoPool->add<aris::system::ErrorInfo>\
		("elbow singularity", CmdBase::RetStatus::ELBOW_SINGULARITY, "ERROR", "肘部奇异点", "elbow singularity");
	errMap.insert(pair<std::int32_t, string>(CmdBase::RetStatus::ELBOW_SINGULARITY, "elbow singularity"));

	errorinfoPool->add<aris::system::ErrorInfo>\
		("three points collinear", CmdBase::RetStatus::THREE_POINTS_COLLINEAR, "ERROR", "三点共线", "three points collinear");
	errMap.insert(pair<std::int32_t, string>(CmdBase::RetStatus::THREE_POINTS_COLLINEAR, "three points collinear"));

	errorinfoPool->add<aris::system::ErrorInfo>\
		("no moveJ planner", CmdBase::RetStatus::NO_MOVEJ_PLANNER, "ERROR", "没有moveJ规划器", "no moveJ planner");
	errMap.insert(pair<std::int32_t, string>(CmdBase::RetStatus::NO_MOVEJ_PLANNER, "no moveJ planner"));

	errorinfoPool->add<aris::system::ErrorInfo>\
		("no moveL planner", CmdBase::RetStatus::NO_MOVEL_PLANNER, "ERROR", "没有moveL规划器", "no moveL planner");
	errMap.insert(pair<std::int32_t, string>(CmdBase::RetStatus::NO_MOVEL_PLANNER, "no moveL planner"));

	errorinfoPool->add<aris::system::ErrorInfo>\
		("no moveC planner", CmdBase::RetStatus::NO_MOVEC_PLANNER, "ERROR", "没有moveC规划器", "no moveC planner");
	errMap.insert(pair<std::int32_t, string>(CmdBase::RetStatus::NO_MOVEC_PLANNER, "no moveC planner"));

	errorinfoPool->add<aris::system::ErrorInfo>\
		("no moveS planner", CmdBase::RetStatus::NO_MOVES_PLANNER, "ERROR", "没有moveS规划器", "no moveS planner");
	errMap.insert(pair<std::int32_t, string>(CmdBase::RetStatus::NO_MOVES_PLANNER, "no moveS planner"));

	errorinfoPool->add<aris::system::ErrorInfo>\
		("no moveLL planner", CmdBase::RetStatus::NO_MOVELL_PLANNER, "ERROR", "没有moveLL规划器", "no moveLL planner");
	errMap.insert(pair<std::int32_t, string>(CmdBase::RetStatus::NO_MOVELL_PLANNER, "no moveLL planner"));

	errorinfoPool->add<aris::system::ErrorInfo>\
		("no servoJ planner", CmdBase::RetStatus::NO_SERVOJ_PLANNER, "ERROR", "没有servoJ规划器", "no servoJ planner");
	errMap.insert(pair<std::int32_t, string>(CmdBase::RetStatus::NO_SERVOJ_PLANNER, "no servoJ planner"));

	///****// executerRT(),运动算法错误 //--------------------------------------



	tmpErrorInfoPool(*errorinfoPool, errMap);

	return errorinfoPool;


}
	////*********************************************************//////

	//*************************创建指令集************************//////
	auto createCmdRoot()->std::unique_ptr<aris::cmdtarget::CmdRoot>
{
	std::unique_ptr<aris::cmdtarget::CmdRoot> cmd_root(new aris::cmdtarget::CmdRoot);

	///////////*************运动类**************////////////
	cmd_root->cmdPool().add<aris::cmdtarget::Enable>();
	cmd_root->cmdPool().add<aris::cmdtarget::Disable>();
	cmd_root->cmdPool().add<aris::cmdtarget::Mode>();
	cmd_root->cmdPool().add<aris::cmdtarget::JogJ>();
	cmd_root->cmdPool().add<aris::cmdtarget::ServoJ>();
	cmd_root->cmdPool().add<aris::cmdtarget::JogC>();
	cmd_root->cmdPool().add<aris::cmdtarget::MoveJ>();
	cmd_root->cmdPool().add<aris::cmdtarget::MoveL>();
	cmd_root->cmdPool().add<aris::cmdtarget::MoveC>();
	cmd_root->cmdPool().add<aris::cmdtarget::MoveS>();
	cmd_root->cmdPool().add<aris::cmdtarget::MoveLL>();
	cmd_root->cmdPool().add<aris::cmdtarget::Reset>();
	cmd_root->cmdPool().add<aris::cmdtarget::Home>();
	cmd_root->cmdPool().add<aris::cmdtarget::FCPressL>();



	///////////*************标定类**************////////////
	cmd_root->cmdPool().add <aris::cmdtarget::CalibT6P>();
	cmd_root->cmdPool().add <aris::cmdtarget::CalibT5P>();
	cmd_root->cmdPool().add <aris::cmdtarget::CalibT4P>();
	cmd_root->cmdPool().add <aris::cmdtarget::CalibJointZero>();
	cmd_root->cmdPool().add <aris::cmdtarget::CalibCurDyn>();



	///////////*************系统类**************////////////
	cmd_root->cmdPool().add<aris::cmdtarget::ConnectVrep>();
	cmd_root->cmdPool().add<aris::cmdtarget::DisconnectVrep>();
	cmd_root->cmdPool().add<aris::cmdtarget::Clear>();
	cmd_root->cmdPool().add<aris::cmdtarget::Show>();
	cmd_root->cmdPool().add<aris::cmdtarget::SetProgramRate>();
	cmd_root->cmdPool().add<aris::cmdtarget::Start>();
	cmd_root->cmdPool().add<aris::cmdtarget::Stop>();
	cmd_root->cmdPool().add<aris::cmdtarget::Pause>();
	cmd_root->cmdPool().add<aris::cmdtarget::CSstart>();
	cmd_root->cmdPool().add<aris::cmdtarget::CSstop>();
	cmd_root->cmdPool().add<aris::cmdtarget::SaveXml>();
	cmd_root->cmdPool().add<aris::cmdtarget::GetXml>();
	cmd_root->cmdPool().add<aris::cmdtarget::TeachPoint>();
	cmd_root->cmdPool().add<aris::cmdtarget::DeletePoint>();
	cmd_root->cmdPool().add<aris::cmdtarget::DefineVar>();
	cmd_root->cmdPool().add<aris::cmdtarget::DeleteVar>();
	cmd_root->cmdPool().add<aris::cmdtarget::SaveParam>();
	cmd_root->cmdPool().add<aris::cmdtarget::DeleteParam>();
	cmd_root->cmdPool().add<aris::cmdtarget::RenameParam>();
	cmd_root->cmdPool().add<aris::cmdtarget::GetGravity>();
	cmd_root->cmdPool().add<aris::cmdtarget::SetInstallAng>();
	cmd_root->cmdPool().add<aris::cmdtarget::Recover>();
	cmd_root->cmdPool().add<aris::cmdtarget::Get>();
	cmd_root->cmdPool().add<aris::cmdtarget::Sleep>();
	cmd_root->cmdPool().add<aris::cmdtarget::Wait>();
	cmd_root->cmdPool().add<aris::cmdtarget::GetInfo>();
	cmd_root->cmdPool().add<aris::cmdtarget::ProgramBlock>();
	cmd_root->cmdPool().add<aris::cmdtarget::Var>();
	cmd_root->cmdPool().add<aris::cmdtarget::NRTstart>();
	cmd_root->cmdPool().add<aris::cmdtarget::NRTstop>();
	//////////////////////*************IO类**************////////////
	cmd_root->cmdPool().add<aris::cmdtarget::DOSet>();
	cmd_root->cmdPool().add<aris::cmdtarget::DOPulse>();
	cmd_root->cmdPool().add<aris::cmdtarget::DIGet>();
	cmd_root->cmdPool().add<aris::cmdtarget::ReplayCmd>();

	////////////////////////*************示例**************////////////
	//cmd_root->cmdPool().add<aris::cmdtarget::MoveSine>();
	
	tmpCmdRoot(*cmd_root);
	
	//auto &rs = plan_root->planPool().add<kaanh::Reset>();
	//rs.command().findParam("pos")->setDefaultValue("{0.5,0.3925,0.7899,0.5,0.5,0.5}");
	//rs.command().findParam("pos")->setDefaultValue("{0.5,0.353,0.5,0.5,0.5,0.5}");
	return cmd_root;
}
	////*********************************************************//////

	auto tmpErrorInfoPool(core::ObjectPool<aris::system::ErrorInfo>& errorinfoPool, std::map<std::int32_t, std::string>& errMap)->void
	{

		errorinfoPool.add<aris::system::ErrorInfo>\
			("master name not exist", -2, "ERROR", "主站不存在", "master name not exist");


	}

	auto tmpCmdRoot(aris::cmdtarget::CmdRoot& cmd_root)->void
	{
		//////////////////////*************示例**************////////////
		cmd_root.cmdPool().add<aris::cmdtarget::MoveSine>();


	}

	auto createUserDataType(core::Calculator& cal)->void
{
	std::cout << "create user data!" << std::endl;
	cal.addTypename("array");
	cal.addFunction("array", std::vector<std::string>{"Matrix"}, "array", [](std::vector<std::any>& params)->std::any
		{
			return params[0];
		});
	
	cal.addTypename("load");
	cal.addFunction("load", std::vector<std::string>{"Matrix"}, "load", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 10)
			{
				THROW_FILE_LINE("input data error");
			}
			Load a;
			auto temp = std::any_cast<core::Matrix&>(params[0]).data();
			std::copy(temp, temp+10, &a.inertia[0]);
			/*std::copy(temp + 1, temp + 4, &a.cog[0]);
			std::copy(temp + 4, temp + 8, &a.pq[0]);
			std::copy(temp + 8, temp + 11, &a.iner[0]);*/

			return a;
		});
	cal.addBinaryOperatorFunction("=", "load", "Matrix", "load", [](std::any& left, std::any& right)->std::any
		{
			if (std::any_cast<core::Matrix>(right).size() != 10)
			{
				THROW_FILE_LINE("input data error");
			}
			Load load;
			auto temp = std::any_cast<core::Matrix&>(right).data();
			std::copy(temp, temp + 10, &load.inertia[0]);
			
			left = load;
			return left;
		});

	
	cal.addTypename("pose");
	cal.addFunction("pose", std::vector<std::string>{"Matrix"}, "pose", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 7)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
	cal.addTypename("jointtarget");
	cal.addFunction("jointtarget", std::vector<std::string>{"Matrix"}, "jointtarget", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != MAX_DOFS)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
	
	
	cal.addTypename("robottarget");
	cal.addFunction("robottarget", std::vector<std::string>{"Matrix"}, "robottarget", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 16)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
	
	cal.addTypename("TeachTarget");
	

	cal.addTypename("zone");
	cal.addFunction("zone", std::vector<std::string>{"Matrix"}, "zone", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 2)
			{
				THROW_FILE_LINE("input data error");
			}
			Zone z;
			auto temp = std::any_cast<core::Matrix&>(params[0]).data();
			std::copy(temp, temp + 1, &z.dis);
			std::copy(temp + 1, temp + 2, &z.per);

			return z;
		});
	cal.addBinaryOperatorFunction("=", "zone", "Matrix", "zone", [](std::any& left, std::any& right)->std::any
		{
			if (std::any_cast<core::Matrix>(right).size() != 2)
			{
				THROW_FILE_LINE("input data error");
			}
			Zone z;
			auto temp = std::any_cast<core::Matrix&>(right).data();
			std::copy(temp, temp + 1, &z.dis);
			std::copy(temp + 1, temp + 2, &z.per);

			left = z;
			return left;
		});
	// add zone variables
	
	cal.addVariable("fine", "zone", Zone({ 0.0, 0.0 }));
	cal.addVariable("z1", "zone", Zone({ 0.001, 0.01 }));
	cal.addVariable("z5", "zone", Zone({ 0.005, 0.03 }));
	cal.addVariable("z10", "zone", Zone({ 0.01, 0.05 }));
	cal.addVariable("z15", "zone", Zone({ 0.015, 0.08 }));
	cal.addVariable("z20", "zone", Zone({ 0.02, 0.1 }));
	cal.addVariable("z30", "zone", Zone({ 0.03, 0.15 }));
	cal.addVariable("z40", "zone", Zone({ 0.04, 0.2 }));
	cal.addVariable("z50", "zone", Zone({ 0.05, 0.25 }));
	cal.addVariable("z60", "zone", Zone({ 0.06, 0.3 }));
	cal.addVariable("z80", "zone", Zone({ 0.08, 0.4 }));
	cal.addVariable("z100", "zone", Zone({ 0.1, 0.45 }));
	cal.addVariable("z150", "zone", Zone({ 0.15, 0.45 }));
	cal.addVariable("z200", "zone", Zone({ 0.2, 0.45 }));
	

	cal.addTypename("speed");
	cal.addFunction("speed", std::vector<std::string>{"Matrix"}, "speed", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 5)
			{
				THROW_FILE_LINE("input data error");
			}
			Speed z;
			auto temp = std::any_cast<core::Matrix&>(params[0]).data();
			std::copy(temp, temp + 1, &z.w_per);
			std::copy(temp + 1, temp + 2, &z.v_tcp);
			std::copy(temp + 2, temp + 3, &z.w_tcp);
			std::copy(temp + 3, temp + 4, &z.w_ext);
			std::copy(temp + 4, temp + 5, &z.v_ext);

			return z;

		});
	cal.addBinaryOperatorFunction("=", "speed", "Matrix", "speed", [](std::any& left, std::any& right)->std::any
		{
			if (std::any_cast<core::Matrix>(right).size() != 5)
			{
				THROW_FILE_LINE("input data error");
			}
			Speed z;
			auto temp = std::any_cast<core::Matrix&>(right).data();
			std::copy(temp, temp + 1, &z.w_per);
			std::copy(temp + 1, temp + 2, &z.v_tcp);
			std::copy(temp + 2, temp + 3, &z.w_tcp);
			std::copy(temp + 3, temp + 4, &z.w_ext);
			std::copy(temp + 4, temp + 5, &z.v_ext);

			left = z;
			return left;
		});
	// add velocity variables
	
	cal.addVariable("v5", "speed", Speed({ 0.005, 0.005, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v10", "speed", Speed({ 0.01, 0.01, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v25", "speed", Speed({ 0.025, 0.025, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v30", "speed", Speed({ 0.03, 0.03, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v40", "speed", Speed({ 0.04, 0.04, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v50", "speed", Speed({ 0.05, 0.05, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v60", "speed", Speed({ 0.06, 0.06, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v80", "speed", Speed({ 0.08, 0.08, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v100", "speed", Speed({ 0.1, 0.1, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v150", "speed", Speed({ 0.15, 0.15, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v200", "speed", Speed({ 0.2, 0.2, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v300", "speed", Speed({ 0.3, 0.3, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v400", "speed", Speed({ 0.4, 0.4, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v500", "speed", Speed({ 0.5, 0.5, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v600", "speed", Speed({ 0.6, 0.6, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v800", "speed", Speed({ 0.8, 0.8, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v1000", "speed", Speed({ 1.0, 1.0, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v1500", "speed", Speed({ 1.0, 1.5, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v2000", "speed", Speed({ 1.0, 2.0, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v3000", "speed", Speed({ 1.0, 3.0, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v4000", "speed", Speed({ 1.0, 4.0, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v5000", "speed", Speed({ 1.0, 5.0, 200 * PI / 180, 0.0, 0.0 }));
	cal.addVariable("v6000", "speed", Speed({ 1.0, 6.0, 200 * PI / 180, 0.0, 0.0 }));
	

	cal.addTypename("tool");
	cal.addFunction("tool", std::vector<std::string>{"Matrix"}, "tool", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 16)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
	cal.addBinaryOperatorFunction("=", "tool", "Matrix", "tool", [](std::any& left, std::any& right)->std::any
		{
			if (std::any_cast<core::Matrix>(right).size() != 16)
			{
				THROW_FILE_LINE("input data error");
			}
			left = right;
			return left;
		});
	// add tool offset, inertia variables
	//cal.addVariable("tool0_axis_home", "tool", core::Matrix(1, 6, 0.0));
	//for (int i = 1; i < 17; ++i)
	//{
	//	cal.addVariable("tool" + std::to_string(i) + "_axis_offset", "tool", core::Matrix(1, 16, 0.0));
	//	//cal.addVariable("tool" + std::to_string(i) + "_inertia", "tool", core::Matrix(1, 10, 0.0));
	//}


	cal.addTypename("wobj");
	cal.addFunction("wobj", std::vector<std::string>{"Matrix"}, "wobj", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != 16)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
	cal.addBinaryOperatorFunction("=", "wobj", "Matrix", "wobj", [](std::any& left, std::any& right)->std::any
		{
			if (std::any_cast<core::Matrix>(right).size() != 16)
			{
				THROW_FILE_LINE("input data error");
			}
			left = right;
			return left;
		});


	cal.addTypename("zero_comp");
	cal.addFunction("zero_comp", std::vector<std::string>{"Matrix"}, "wobj", [](std::vector<std::any>& params)->std::any
		{
			if (std::any_cast<core::Matrix>(params[0]).size() != MAX_DOFS)
			{
				THROW_FILE_LINE("input data error");
			}
			return params[0];
		});
	cal.addBinaryOperatorFunction("=", "zero_comp", "Matrix", "zero_comp", [](std::any& left, std::any& right)->std::any
		{
			if (std::any_cast<core::Matrix>(right).size() != MAX_DOFS)
			{
				THROW_FILE_LINE("input data error");
			}
			left = right;
			return left;
		});

}
	auto createDefaultData(aris::model::Model& model)->void
{
	double zone[2] = { 0 };
	model.variablePool().add<aris::model::MatrixVariable>("fine", core::Matrix(1, 2, zone));
	zone[0] = 0.001; zone[1] = 0.01;
	model.variablePool().add<aris::model::MatrixVariable>("z1", core::Matrix(1, 2, zone));
	zone[0] = 0.005; zone[1] = 0.03;
	model.variablePool().add<aris::model::MatrixVariable>("z5", core::Matrix(1, 2, zone));
	zone[0] = 0.01; zone[1] = 0.05;
	model.variablePool().add<aris::model::MatrixVariable>("z10", core::Matrix(1, 2, zone));
	zone[0] = 0.015; zone[1] = 0.08;
	model.variablePool().add<aris::model::MatrixVariable>("z15", core::Matrix(1, 2, zone));
	zone[0] = 0.02; zone[1] = 0.1;
	model.variablePool().add<aris::model::MatrixVariable>("z20", core::Matrix(1, 2, zone));
	zone[0] = 0.03; zone[1] = 0.15;
	model.variablePool().add<aris::model::MatrixVariable>("z30", core::Matrix(1, 2, zone));
	zone[0] = 0.04; zone[1] = 0.2;
	model.variablePool().add<aris::model::MatrixVariable>("z40", core::Matrix(1, 2, zone));
	zone[0] = 0.05; zone[1] = 0.25;
	model.variablePool().add<aris::model::MatrixVariable>("z50", core::Matrix(1, 2, zone));
	zone[0] = 0.06; zone[1] = 0.3;
	model.variablePool().add<aris::model::MatrixVariable>("z60", core::Matrix(1, 2, zone));
	zone[0] = 0.08; zone[1] = 0.4;
	model.variablePool().add<aris::model::MatrixVariable>("z80", core::Matrix(1, 2, zone));
	zone[0] = 0.1; zone[1] = 0.45;
	model.variablePool().add<aris::model::MatrixVariable>("z100", core::Matrix(1, 2, zone));
	zone[0] = 0.2; zone[1] = 0.45;
	model.variablePool().add<aris::model::MatrixVariable>("z150", core::Matrix(1, 2, zone));
	zone[0] = 0.06; zone[1] = 0.3;
	model.variablePool().add<aris::model::MatrixVariable>("z200", core::Matrix(1, 2, zone));

	double v[5] = { 0 };
	v[0] = 0.005; v[1] = 0.005; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v5", core::Matrix(1, 5, v));
	v[0] = 0.01; v[1] = 0.01; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v10", core::Matrix(1, 5, v));
	v[0] = 0.025; v[1] = 0.025; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v25", core::Matrix(1, 5, v));
	v[0] = 0.03; v[1] = 0.03; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v30", core::Matrix(1, 5, v));
	v[0] = 0.04; v[1] = 0.04; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v40", core::Matrix(1, 5, v));
	v[0] = 0.05; v[1] = 0.05; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v50", core::Matrix(1, 5, v));
	v[0] = 0.06; v[1] = 0.06; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v60", core::Matrix(1, 5, v));
	v[0] = 0.08; v[1] = 0.08; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v80", core::Matrix(1, 5, v));
	v[0] = 0.1; v[1] = 0.1; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v100", core::Matrix(1, 5, v));
	v[0] = 0.15; v[1] = 0.15; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v150", core::Matrix(1, 5, v));
	v[0] = 0.2; v[1] = 0.2; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v200", core::Matrix(1, 5, v));
	v[0] = 0.3; v[1] = 0.3; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v300", core::Matrix(1, 5, v));
	v[0] = 0.4; v[1] = 0.4; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v400", core::Matrix(1, 5, v));
	v[0] = 0.5; v[1] = 0.5; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v500", core::Matrix(1, 5, v));
	v[0] = 0.6; v[1] = 0.6; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v600", core::Matrix(1, 5, v));
	v[0] = 0.8; v[1] = 0.8; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v800", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 1.0; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v1000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 1.5; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v1500", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 2.0; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v2000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 3.0; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v3000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 4.0; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v4000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 5.0; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v5000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 6.0; v[2] = 200 * PI / 180;
	model.variablePool().add<aris::model::MatrixVariable>("v6000", core::Matrix(1, 5, v));


	double load[10] = { 0 };
	model.variablePool().add<aris::model::MatrixVariable>("load0", core::Matrix(1, 10, load));

	double zero[MAX_DOFS] = { 0 };
	model.variablePool().add<aris::model::MatrixVariable>("zero_comp0", core::Matrix(1, MAX_DOFS, zero));

}
}
