#include "basicsystem.hpp"

using namespace codeit::controller;
using namespace codeit::function;
using namespace codeit::model;

namespace codeit::system
{
	//*************************����������***********************//////
	auto createVrepController()->std::unique_ptr<codeit::controller::Controller>	/*�������ص���һ����ָ�룬ָ��ָ��Controller,controller������������ָ��std::unique_ptr*/
{
	std::unique_ptr<codeit::controller::Controller> controller(new codeit::controller::VrepController);

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
		controller->slavePool().add<codeit::controller::VrepMotor>().loadXmlStr(xml_str);
#ifndef ARIS_USE_ETHERCAT_SIMULATION
		//dynamic_cast<codeit::control::VrepMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
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
	//	controller->slavePool().add<codeit::control::VrepMotor>().loadXmlStr(xml_str);
	//	//#ifndef ARIS_USE_ETHERCAT_SIMULATION
	//	//		dynamic_cast<codeit::control::VrepMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
	//	//#endif
	//}
	
	
	//std::unique_ptr<codeit::control::Controller> controller(robot::createControllerRokaeXB4());/*����std::unique_ptrʵ��*/
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
	//controller->slavePool().add<codeit::control::VrepSlave>().loadXmlStr(xml_str);

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
	//	controller->slavePool().add<codeit::controller::VrepMotor>().loadXmlStr(xml_str);
	//	//#ifndef ARIS_USE_ETHERCAT_SIMULATION
	//	//		dynamic_cast<codeit::control::VrepMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
	//	//#endif
	//}

	return controller;
}
	auto createEcatController()->std::unique_ptr<codeit::controller::Controller>	/*�������ص���һ����ָ�룬ָ��ָ��Controller,controller������������ָ��std::unique_ptr*/
{
		std::unique_ptr<codeit::controller::Controller> controller(new codeit::controller::EthercatController);/*����std::unique_ptrʵ��*/
		controller->slavePool().clear();	//���slavePool�е�Ԫ�أ������������
        for (Size i = 0; i < 1; ++i)
        {
#ifdef UNIX
            double pos_offset[6]
            {
               1.4327,   2.41069874505508,   0.762016940604939,   4.69692333813102,   0.0360485485152105,   1.43257027673246
            };
#endif
#ifdef WIN32
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
#endif
            double pos_factor[6]
            {
                1048576.0 / 2 / PI, 1048576.0 / 2 / PI, 1048576.0 / 2 / PI, -1048576.0 / 2 / PI, -524288.0 / 2 / PI, 524288.0 / 2 / PI
            };
            double max_pos[6]
            {
                180.0 / 360 * 2 * PI, 120.0 / 360 * 2 * PI,	60.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 120.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI
            };
            double min_pos[6]
            {
                -180.0 / 360 * 2 * PI, -120.0 / 360 * 2 * PI, -240.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI, -120.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI
            };
            double max_vel[6]
            {
                0.4, 0.4, 0.4, 0.4, 0.6, 0.6
            };
            double max_acc[6]
            {
//                3.3, 3.3*2, 3.3, 3.3, 7.5, 7.5
//                10, 10, 10, 10, 25, 25
                  20,20,20,20,50,50
            };

            std::string xml_str =
                "<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x201\""
                " vendor_id=\"0x000022D2\" revision_num=\"0x0a000002\" dc_assign_activate=\"0x0300\""
                " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
                " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
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
                "				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
                "			</Pdo>"
                "		</SyncManager>"
                "		<SyncManager is_tx=\"true\">"
                "			<Pdo index=\"0x1A00\" is_tx=\"true\">"
                "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
                "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
                "			</Pdo>"
                "		</SyncManager>"
                "	</SyncManagerPoolObject>"
                "</EthercatMotor>";
            controller->slavePool().add<codeit::controller::EthercatMotor>().loadXmlStr(xml_str);
        }

#ifdef UNIX
        dynamic_cast<codeit::controller::EthercatController*>(controller.get())->scanInfoForCurrentSlaves();

        dynamic_cast<codeit::controller::EthercatController*>(controller.get())->scanPdoForCurrentSlaves();
#endif
        std::cout << controller->xmlString() << std::endl;



		return controller;
}
    auto createZeroErrEcatController()->std::unique_ptr<codeit::controller::Controller>	/*�������ص���һ����ָ�룬ָ��ָ��Controller,controller������������ָ��std::unique_ptr*/
{
        std::unique_ptr<codeit::controller::Controller> controller(new codeit::controller::EthercatController);/*����std::unique_ptrʵ��*/
        controller->slavePool().clear();	//���slavePool�е�Ԫ�أ������������
        for (Size i = 0; i < 1; ++i)
        {
#ifdef UNIX
            double pos_offset[6]
            {
               1.4327,   2.41069874505508,   0.762016940604939,   4.69692333813102,   0.0360485485152105,   1.43257027673246
            };
#endif
#ifdef WIN32
			double pos_offset[6]
			{
				0,0,0,0,0,0
			};
#endif
            double pos_factor[6]
            {
                1048576.0 / 2 / PI, 1048576.0 / 2 / PI, 1048576.0 / 2 / PI, -1048576.0 / 2 / PI, -524288.0 / 2 / PI, 524288.0 / 2 / PI
            };
            double max_pos[6]
            {
                180.0 / 360 * 2 * PI, 120.0 / 360 * 2 * PI,	60.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI, 120.0 / 360 * 2 * PI, 180.0 / 360 * 2 * PI
            };
            double min_pos[6]
            {
                -180.0 / 360 * 2 * PI, -120.0 / 360 * 2 * PI, -240.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI, -120.0 / 360 * 2 * PI, -180.0 / 360 * 2 * PI
            };
            double max_vel[6]
            {
                0.4, 0.4, 0.4, 0.4, 0.6, 0.6
            };
            double max_acc[6]
            {
//                3.3, 3.3*2, 3.3, 3.3, 7.5, 7.5
//                10, 10, 10, 10, 25, 25
                  20,20,20,20,50,50
            };

            std::string xml_str =
                "<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x29252\""
                " vendor_id=\"0x5A65726F\" revision_num=\"0x00000001\" dc_assign_activate=\"0x0300\""
                " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
                " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
                " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
                "	<SyncManagerPoolObject>"
                "		<SyncManager is_tx=\"false\"/>"
                "		<SyncManager is_tx=\"true\"/>"
                "		<SyncManager is_tx=\"false\">"
                "			<Pdo index=\"0x1600\" is_tx=\"false\">"
                "               <PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
                "               <PdoEntry name=\"mode_of_operation\" index=\"0x60FE\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"control_tmp\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
                "			</Pdo>"
                "		</SyncManager>"
                "		<SyncManager is_tx=\"true\">"
                "			<Pdo index=\"0x1A00\" is_tx=\"true\">"
                "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"mode_of_display\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
                "			</Pdo>"
                "		</SyncManager>"
                "	</SyncManagerPoolObject>"
                "</EthercatMotor>";
            controller->slavePool().add<codeit::controller::EthercatMotor>().loadXmlStr(xml_str);
        }

#ifdef UNIX
        dynamic_cast<codeit::controller::EthercatController*>(controller.get())->scanInfoForCurrentSlaves();

        dynamic_cast<codeit::controller::EthercatController*>(controller.get())->scanPdoForCurrentSlaves();
#endif
        std::cout << controller->xmlString() << std::endl;



        return controller;
}

	auto createSocketController(const NumList* numList,std::string name, std::string ip, std::string port, SocketMaster::TYPE type, Size nrt_id)->std::unique_ptr<codeit::controller::NrtController>
	{
		static const NumList socket_num_list= { 0 };
		numList = numList ? numList : &socket_num_list;


		std::unique_ptr<codeit::controller::NrtController> controller(new codeit::controller::SocketController(name,ip,port,type,nrt_id));

		int begin_index = 0;
		for (Size i = 0; i < numList->di_num; ++i)
		{
			std::string xml_str =
				"<SocketDI phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</SocketDI>";
			controller->slavePool().add<codeit::controller::SocketDI>().loadXmlStr(xml_str);
			begin_index++;
		}
	
		for (Size i = 0; i < numList->do_num; ++i)
		{
			std::string xml_str =
				"<SocketDO phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</SocketDO>";
			controller->slavePool().add<codeit::controller::SocketDO>().loadXmlStr(xml_str);
			begin_index++;
		}


		double res[6] = { 0.1,0.2,0.3,0.4,0.5,0.6 };
		
		for (Size i = 0; i < numList->ai_num; ++i)
		{
			std::string xml_str =
				"<SocketAI phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\" resolution=\"" + std::to_string(res[i]) + "\">"
				"</SocketAI>";
			controller->slavePool().add<codeit::controller::SocketAI>().loadXmlStr(xml_str);
			begin_index++;
		}

		for (Size i = 0; i < numList->ao_num; ++i)
		{
			std::string xml_str =
				"<SocketAO phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\" resolution=\"" + std::to_string(res[i]) + "\">"
				"</SocketAO>";
			controller->slavePool().add<codeit::controller::SocketAO>().loadXmlStr(xml_str);
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

			controller->slavePool().add<codeit::controller::SocketMotor>().loadXmlStr(xml_str);
			begin_index++;
		}

		for (Size i = 0; i < numList->subsys_num; ++i)
		{
			std::string xml_str =
				"<SocketSubSystem phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</SocketSubSystem>";
			controller->slavePool().add<codeit::controller::SocketSubSystem>().loadXmlStr(xml_str);
			begin_index++;
		}

		return controller;

	}
	auto createComController(const NumList* numList, const std::string& name, const core::SerialPort::ComOptions& options, Size pack_size, Size nrt_id)->std::unique_ptr<codeit::controller::NrtController>
	{
		static const NumList com_num_list = { 0 };
		numList = numList ? numList : &com_num_list;

		std::unique_ptr<codeit::controller::NrtController> controller(new codeit::controller::ComController(name, options, pack_size, nrt_id));

		int begin_index = 0;
		for (Size i = 0; i < numList->di_num; ++i)
		{
			std::string xml_str =
				"<ComDI phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</ComDI>";
			controller->slavePool().add<codeit::controller::ComDI>().loadXmlStr(xml_str);
			begin_index++;
		}

		for (Size i = 0; i < numList->do_num; ++i)
		{
			std::string xml_str =
				"<ComDO phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</ComDO>";
			controller->slavePool().add<codeit::controller::ComDO>().loadXmlStr(xml_str);
			begin_index++;
		}

		double res[6] = { 0.1,0.2,0.3,0.4,0.5,0.6 };
		for (Size i = 0; i < numList->ai_num; ++i)
		{
			std::string xml_str =
				"<ComAI phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\" resolution=\"" + std::to_string(res[i]) + "\">"
				"</ComAI>";
			controller->slavePool().add<codeit::controller::ComAI>().loadXmlStr(xml_str);
			begin_index++;
		}
		for (Size i = 0; i < numList->ao_num; ++i)
		{
			std::string xml_str =
				"<ComAO phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\" resolution=\"" + std::to_string(res[i]) + "\">"
				"</ComAO>";
			controller->slavePool().add<codeit::controller::ComAO>().loadXmlStr(xml_str);
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

			controller->slavePool().add<codeit::controller::ComMotor>().loadXmlStr(xml_str);
			begin_index++;
		}

		for (Size i = 0; i < numList->subsys_num; ++i)
		{
			std::string xml_str =
				"<ComSubSystem phy_id=\"" + std::to_string(begin_index) + "\"is_virtual=\"false\">"
				"</ComSubSystem>";
			controller->slavePool().add<codeit::controller::ComSubSystem>().loadXmlStr(xml_str);
			begin_index++;
		}

		return controller;

	}

	auto createNrtControllerPool()->std::unique_ptr<core::ObjectPool<codeit::controller::NrtController>>
	{
		std::unique_ptr<core::ObjectPool<codeit::controller::NrtController>> nrtControllerPool(new core::ObjectPool<codeit::controller::NrtController>);
		
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

	//*************************����ģ��************************//////
	auto createPumaModel(std::string name)->std::unique_ptr<codeit::model::Model>
{
	codeit::model::PumaParam param;
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

	auto model = codeit::model::createModelPuma(param,name);


	return std::move(model);
}
	auto createScaraModel(std::string name)->std::unique_ptr<codeit::model::Model>
	{
	codeit::model::ScaraParam param;
	param.a2 = 0.467;
	param.a3 = 0.4005;
	param.d3 = 0.08;
	auto model = codeit::model::createModelScara(param, name);

	return std::move(model);
	}
	auto createUrModel(std::string name)->std::unique_ptr<codeit::model::Model>
{
		codeit::model::UrParam param;
		param.L1 = 0.425;
		param.L2 = 0.39225;
		param.W1 = 0.13585 - 0.1197 + 0.093;
		param.W2 = 0.0823;
		param.H1 = 0.089159;
		param.H2 = 0.09465;
		
		auto model = codeit::model::createModelUr(param, name);

		return std::move(model);
};
	auto createIOModel(const NumList* numList, std::string name)->std::unique_ptr<codeit::model::IOModel>
	{
		static const NumList model_num_list = { 0 };
		numList = numList ? numList : &model_num_list;
		std::unique_ptr<codeit::model::IOModel> iomodel(new codeit::model::IOModel(name));
		
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

	auto createModelPool()->std::unique_ptr<core::ObjectPool<codeit::model::Model>>
{
	std::unique_ptr<core::ObjectPool<codeit::model::Model>> modelPool(new core::ObjectPool<codeit::model::Model>);
	auto model0 = createPumaModel("MJ08").release();
	auto model1= createUrModel("UR5").release();
	auto model2 = createScaraModel("Scara").release();
	modelPool->add(model1);

	//modelPool->add(model1);
	for(Size i=0;i<modelPool->size();i++)
		createDefaultData(modelPool->at(i));

	return std::move(modelPool);


}
	auto createIOModelPool()->std::unique_ptr<core::ObjectPool<codeit::model::IOModel>>
	{
		std::unique_ptr<core::ObjectPool<codeit::model::IOModel>> iomodelPool(new core::ObjectPool<codeit::model::IOModel>);
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


	//*************************�����쳣��Ϣ��************************//////
	auto createErrorInfoPool()->std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>>
	{
		std::unique_ptr<core::ObjectPool<codeit::system::ErrorInfo>> errorinfoPool(new core::ObjectPool<codeit::system::ErrorInfo>);

		/////����ð�ţ���Ҫ���ھ�β����������ִ������,��������ݶ����ڶ���������ʾ����

		////****��errorinfo������쳣����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unknown errorinfo", -2, "ERROR", "���쳣��Ϣδע��", "unregistered errorinfo");
		///****��errorinfo������쳣����--------------------------------------




		///****�ؽڲ���
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input pos beyond range-joint", -2, "ERROR", "����λ�ó���", "input position beyond range-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input vel beyond range-joint", -2, "ERROR", "�����ٶȳ���", "input velocity beyond range-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input acc beyond range-joint", -2, "ERROR", "������ٶȳ���", "input acc beyond range-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input dec beyond range-joint", -2, "ERROR", "������ٶȳ���", "input dec beyond range-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("acc dimension mismatch-joint", -2, "ERROR", "���ٶ�ά����ƥ��", "acc dimension mismatch-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dec dimension mismatch-joint", -2, "ERROR", "���ٶ�ά����ƥ��", "dec dimension mismatch-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("jerk dimension mismatch-joint", -2, "ERROR", "�Ӽ��ٶ�ά����ƥ��", "jerk dimension mismatch-joint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pos dimension mismatch-joint", -2, "ERROR", "λ��ά����ƥ��", "pos dimension mismatch-joint");
		///****�ؽڲ���-----------------------------------------------


		///****Ŀ��λ��
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-robottarget", -2, "ERROR", "��֧�ֵ�λ������", "unsupported displacement dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-robottarget", -2, "ERROR", "��֧�ֵĽǶ�����", "unsupported angle dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy-robottarget", -2, "ERROR", "ŷ���������쳣", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch-robottarget", -2, "ERROR", "λ��-ŷ����ά����ƥ��", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch-robottarget", -2, "ERROR", "��ת����ά����ƥ��", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch-robottarget", -2, "ERROR", "λ��-��Ԫ��ά����ƥ��", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch-robottarget", -2, "ERROR", "λ��-���ά����ƥ��", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input-robottarget", -2, "ERROR", "��֧�ֵ�λ������", "unsupported pose input");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input joint dimension mismatch-robottarget", -2, "ERROR", "����ĹؽڽǶ�ά����ƥ��", "input joint dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("forward kinematic failed-robottarget", -2, "ERROR", "�˶�ѧ����ʧ��", "forward kinematic failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-robottarget", -2, "ERROR", "robottarget����������", "robottarget param not exist");
		///****Ŀ��λ��----------------------------------------------

		///****�м�λ��
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-midrobottarget", -2, "ERROR", "��֧�ֵ�λ������", "unsupported displacement dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-midrobottarget", -2, "ERROR", "��֧�ֵĽǶ�����", "unsupported angle dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy-midrobottarget", -2, "ERROR", "ŷ���������쳣", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch-midrobottarget", -2, "ERROR", "λ��-ŷ����ά����ƥ��", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch-midrobottarget", -2, "ERROR", "��ת����ά����ƥ��", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch-midrobottarget", -2, "ERROR", "λ��-��Ԫ��ά����ƥ��", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch-midrobottarget", -2, "ERROR", "λ��-���ά����ƥ��", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input-midrobottarget", -2, "ERROR", "��֧�ֵ�λ������", "unsupported pose input");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("input joint dimension mismatch-midrobottarget", -2, "ERROR", "����ĹؽڽǶ�ά����ƥ��", "input joint dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("forward kinematic failed-midrobottarget", -2, "ERROR", "�˶�ѧ����ʧ��", "forward kinematic failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-midrobottarget", -2, "ERROR", "robottarget����������", "robottarget param not exist");
		///****�м�λ��-----------------------------------------------

		///****��������ϵ
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-tool", -2, "ERROR", "��֧�ֵ�λ������", "unsupported displacement dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-tool", -2, "ERROR", "��֧�ֵĽǶ�����", "unsupported angle dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy-tool", -2, "ERROR", "ŷ���������쳣", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch-tool", -2, "ERROR", "λ��-ŷ����ά����ƥ��", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch-tool", -2, "ERROR", "��ת����ά����ƥ��", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch-tool", -2, "ERROR", "λ��-��Ԫ��ά����ƥ��", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch-tool", -2, "ERROR", "λ��-���ά����ƥ��", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input-tool", -2, "ERROR", "��֧�ֵ�λ������", "unsupported pose input");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-tool", -2, "ERROR", "tool����������", "tool param not exist");

		///****��������ϵ---------------------------------------------


		///****Wobj����ϵ
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-wobj", -2, "ERROR", "��֧�ֵ�λ������", "unsupported displacement dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-wobj", -2, "ERROR", "��֧�ֵĽǶ�����", "unsupported angle dimension");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy-wobj", -2, "ERROR", "ŷ���������쳣", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch-wobj", -2, "ERROR", "λ��-ŷ����ά����ƥ��", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch-wobj", -2, "ERROR", "��ת����ά����ƥ��", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch-wobj", -2, "ERROR", "λ��-��Ԫ��ά����ƥ��", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch-wobj", -2, "ERROR", "λ��-���ά����ƥ��", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input-wobj", -2, "ERROR", "��֧�ֵ�λ������", "unsupported pose input");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-wobj", -2, "ERROR", "wobj����������", "wobj param not exist");

		///****Wobj����ϵ--------------------------------------------

		///****zone����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-zone", -2, "ERROR", "zone����������", "zone param not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("out of range-zone", -2, "ERROR", "zone����Խ��", "zone out of range");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-zone", -2, "ERROR", "zone����ά����ƥ��", "zone dimension mismatch");
		///****zone����-----------------------------------------------

		///****speed����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-speed", -2, "ERROR", "�ٶȲ���������", "speed param not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-speed", -2, "ERROR", "�ٶȲ���ά����ƥ��", "speed dimension mismatch");
		///****speed����---------------------------------------------

		///****load����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-load", -2, "ERROR", "load����������", "load param not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-load", -2, "ERROR", "load����ά����ƥ��", "load dimension mismatch");
		///****load����----------------------------------------------


		///****jointtarget����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-jointtarget", -2, "ERROR", "�ؽ�Ŀ�����������", "jointtarget param not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-jointtarget", -2, "ERROR", "�ؽ�Ŀ�����ά����ƥ��", "jointtarget dimension mismatch");
		///****jointarget����--------------------------------------

		///****�ٶȡ����ٶȡ����ٶȡ��Ӽ��ٶȲ���
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("acc is negative", -2, "ERROR", "���ٶ�Ϊ����", "acc is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dec is positive", -2, "ERROR", "���ٶ�Ϊ����", "dec is positive");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("jerk is negative", -2, "ERROR", "�Ӽ��ٶ�Ϊ����", "jerk is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("jmax is negative", -2, "ERROR", "�Ӽ��ٶ�Ϊ����", "jmax is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("jmin is positive", -2, "ERROR", "�����ٶ�Ϊ����", "jmin is positive");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_jmax is negative", -2, "ERROR", "�ǼӼ��ٶ�Ϊ����", "angular_jmax is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_jmin is positive", -2, "ERROR", "�Ǽ����ٶ�Ϊ����", "angular_jmin is positive");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_acc is negative", -2, "ERROR", "�Ǽ��ٶ�Ϊ����", "angular_acc is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_dec is positive", -2, "ERROR", "�Ǽ��ٶ�Ϊ����", "angular_dec is positive");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("angular_jerk is negative", -2, "ERROR", "�ǼӼ��ٶ�Ϊ����", "angular_jerk is negative");
		///****�ٶȡ����ٶȡ����ٶȡ��Ӽ��ٶȲ���------------------------------------------


		///****ʾ�̵����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param already exist-teachpoint", -2, "ERROR", "��ʾ�̵������Ѵ���", "param already exist-teachpoint");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-teachpoint", -2, "ERROR", "��ʾ�̵����Ʋ�����", "param not exist-teachpoint");
		///****ʾ�̵����------------------------------------------

		///****������������
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-definevar", -2, "ERROR", "��֧�ֵ�λ������", "unsupported displacement dimension-definevar");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-definevar", -2, "ERROR", "��֧�ֵĽǶ�����", "unsupported angle dimension-definevar");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-definespeed", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-definespeed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-definezone", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-definezone");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-defineload", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-defineload");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-definejointtarget", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-definejointtarget");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("type error-definevar", -2, "ERROR", "���ʹ���", "type error-definevar");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("euler angle type invalidiy", -2, "ERROR", "ŷ���������쳣", "euler angle type invalidiy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe dimension mismatch", -2, "ERROR", "λ��-ŷ����ά����ƥ��", "pe dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm dimension mismatch", -2, "ERROR", "��ת����ά����ƥ��", "pm dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq dimension mismatch", -2, "ERROR", "λ��-��Ԫ��ά����ƥ��", "pq dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa dimension mismatch", -2, "ERROR", "λ��-���ά����ƥ��", "pa dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported pose input", -2, "ERROR", "��֧�ֵ�λ������", "unsupported pose input");
		///****������������------------------------------------------

		///****������������
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported displacement dimension-saveparam", -2, "ERROR", "��֧�ֵ�λ������", "unsupported displacement dimension-saveparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unsupported angle dimension-saveparam", -2, "ERROR", "��֧�ֵĽǶ�����", "unsupported angle dimension-saveparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-savespeed", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-savespeed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-savezone", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-savezone");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("dimension mismatch-saveload", -2, "ERROR", "��֧�ֵĽǶ�����", "dimension mismatch-saveload");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param already exist-saveparam", -2, "ERROR", "��ʾ�̵������Ѵ���", "param already exist-saveparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("type error-saveparam", -2, "ERROR", "���ʹ���", "type error-saveparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-deleteparam", -2, "ERROR", "��ʾ�̵������Ѵ���", "param not exist-deleteparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("type error-deleteparam", -2, "ERROR", "���ʹ���", "type error-deleteparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param not exist-renameparam", -2, "ERROR", "��ʾ�̵������Ѵ���", "param not exist-renameparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("new param already exist-renameparam", -2, "ERROR", "��ʾ�̵������Ѵ���", "new param already exist-renameparam");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("type error-renameparam", -2, "ERROR", "���ʹ���", "type error-renameparam");
		///****������������------------------------------------------

		///****JogJ��JogC����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("last_count is negative", -2, "ERROR", "����ʱ��Ϊ����", "last_count is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("error direction", -2, "ERROR", "jog�������ô���", "error direction");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("joint beyond range", -2, "ERROR", "�ؽ�ѡ����", "joint beyond range");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("coordinate error", -2, "ERROR", "����ϵ�趨����", "coordinate error");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("motion type error", -2, "ERROR", "�˶������趨����", "motion type error");
		///****JogJ��JogC����--------------------------------------

		///****ServoJ����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("time is negative", -2, "ERROR", "ʱ������Ϊ����", "time is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("acc dimension mismatch-servoJ", -2, "ERROR", "���ٶ�ά����ƥ��", "acc dimension mismatch-servoJ");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("look_ahead_time is negative", -2, "ERROR", "ǰհʱ����Ϊ����", "look_ahead_time is negative");
		///****ServoJ����--------------------------------------


		///****IO ָ�����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("DO name not exist", -2, "ERROR", "do ������", "DO name not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("DI name not exist", -2, "ERROR", "di ������", "DI name not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("model name not exist", -2, "ERROR", "ģ�Ͳ�����", "model name not exist");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("do_value dimension mismatch do_name", -2, "ERROR", "���ݳ��Ȳ�ƥ��", "do_value dimension mismatch do_name");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("cmd dimension mismatch model_name", -2, "ERROR", "cmd���Ȳ�ƥ��", "cmd dimension mismatch model_name");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no subsys", -2, "ERROR", "û��subsys", "no subsys");

		///****IO ָ�����--------------------------------------


		///****MoveS,Calibrator����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pq set dimension mismatch", -2, "ERROR", "pq������ά����ƥ��", "pq set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pm set dimension mismatch", -2, "ERROR", "pm������ά����ƥ��", "pm set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pa set dimension mismatch", -2, "ERROR", "pa������ά����ƥ��", "pa set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("pe set dimension mismatch", -2, "ERROR", "pe������ά����ƥ��", "pe set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("joint set dimension mismatch", -2, "ERROR", "�ؽڽǶȲ�����ά����ƥ��", "joint set dimension mismatch");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("forward kinematic pm failed", -2, "ERROR", "�����˶�ѧ��ȡpmʧ��", "forward kinematic pm failed");

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("regressor matrix not full rank", -2, "ERROR", "�ع��������", "regressor matrix not full rank");

		///****MoveS,Calibrator����--------------------------------------


		///****����ָ�����
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("count is negative", -2, "ERROR", "����ʱ��Ϊ����", "count is negative");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("program_rate", -2, "ERROR", "��������Ϊ����", "program_rate");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("cs is running", -2, "ERROR", "ʵʱ�߳���������", "cs is running, please stop the cs using cs_stop!");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("failed to stop server", -2, "ERROR", "ֹͣ������ʧ��", "failed to stop server, because it is not running");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("failed to start server", -2, "ERROR", "����������ʧ��", "failed to start server, because it is already started");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no model", -2, "ERROR", "��ģ�Ͳ�����", "no model");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("time is negative", -2, "ERROR", "�ȴ�ʱ��Ϊ����", "wait time is negative");

		///****����ָ�����--------------------------------------

		///****ָ������쳣��Դ��command.cpp
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("brace not pair", -2, "ERROR", "û�гɶԵ�����", "brace not pair");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("empty command string", -2, "ERROR", "�ַ�������Ϊ��", "invalid command string: please at least contain a word");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("invalid command name", -2, "ERROR", "��Чָ��", "server does not have this command");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param should not start with '='", -2, "ERROR", "����������=��ʼ", "param should not start with '='");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("symbol '-' must be followed by an abbreviation of param", -2, "ERROR", "����'-'���������д", "symbol '-' must be followed by an abbreviation of param");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("symbol '--' must be followed by a full name of param", -2, "ERROR", "����'--'���������������", "symbol '--' must be followed by a full name of param");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("param start with single '-' must be an abbreviation", -2, "ERROR", "��'-'��ͷ�Ĳ�����������д", "param start with single '-' must be an abbreviation");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("not a abbreviation of any valid param", -2, "ERROR", "��������������д", "not a abbreviation of any valid param");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("not a valid param", -2, "ERROR", "����һ����Ч��ָ�����", "not a valid param of command");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("failed to find default param in command", -2, "ERROR", "ָ�����Ҳ���Ĭ�ϲ���", "failed to find default param in command");


		///****ָ������쳣��Դ��command.cpp--------------------------------------

		///****// step 3.  execute //
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("server in error", -2, "ERROR", "���������ڴ���״̬", "server in error, use cl to clear");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("server not started", -2, "ERROR", "������δ����", "server not started, use cs_start to start");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("command pool is full", -2, "ERROR", "ָ�������", "command pool is full");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("failed to get current TARGET", -2, "ERROR", "��ȡ��ǰĿ��ʧ��", "failed to get current TARGET, because ControlServer is not running");
		///****// step 3.  execute //--------------------------------------

		///****// Motion Check //
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion is not in OPERATION_ENABLE mode", -2, "ERROR", "��û��ʹ��", "Motion is not in OPERATION_ENABLE mode");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position beyond MAX", -2, "ERROR", "���λ��ָ������ֵ", "Motion target position beyond MAX");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position beyond MIN", -2, "ERROR", "���λ��ָ�����Сֵ", "Motion target position beyond MIN");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position NOT CONTINUOUS", -2, "ERROR", "���λ��ָ��һ�ײ�����", "Motion target position NOT CONTINUOUS");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position NOT SECOND CONTINUOUS", -2, "ERROR", "���λ��ָ����ײ�����", "Motion target position NOT SECOND CONTINUOUS");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target position has FOLLOW ERROR", -2, "ERROR", "�����λ�ø������", "Motion target position has FOLLOW ERROR");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target velocity beyond MAX", -2, "ERROR", "����ٶ�ָ������ֵ", "Motion target velocity beyond MAX");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target velocity beyond MIN", -2, "ERROR", "����ٶ�ָ�����Сֵ", "Motion target velocity beyond MIN");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target velocity NOT CONTINUOUS", -2, "ERROR", "����ٶ�ָ��һ�ײ�����", "Motion target velocity NOT CONTINUOUS");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion target velocity has FOLLOW ERROR", -2, "ERROR", "����ٶ�ָ����ڸ������", "Motion target velocity has FOLLOW ERROR");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual position beyond MAX", -2, "ERROR", "���ʵ��λ�ó������ֵ", "Motion actual position beyond MAX");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual position beyond MIN", -2, "ERROR", "���ʵ��λ�ó�����Сֵ", "Motion actual position beyond MIN");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual velocity beyond MAX", -2, "ERROR", "���ʵ���ٶȳ������ֵ", "Motion actual velocity beyond MAX");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual velocity beyond MIN", -2, "ERROR", "���ʵ���ٶȳ�����Сֵ", "Motion actual velocity beyond MIN");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion actual velocity NOT CONTINUOUS", -2, "ERROR", "���ʵ���ٶ�һ�ײ�����", "Motion actual velocity NOT CONTINUOUS");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Motion MODE INVALID", -2, "ERROR", "���ģʽ��Ч", "Motion MODE INVALID");
		///****// Motion Check //--------------------------------------


		///****// Masterͨ�� //
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Master Lost Connection with", -2, "ERROR", "��վʧȥ����", "Master Lost Connection with");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Master failed start with", -2, "ERROR", "��վ����ʧ��", "Master failed start with");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("master already running", -2, "ERROR", "��վ�Ѿ�����", "master already running, so cannot start");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("master is not running", -2, "ERROR", "��վ�Ѿ�����", "master is not running, so can't stop");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("master cannot set control strategy", -2, "ERROR", "��վ�Ѿ�����", "master already running, cannot set control strategy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("master cannot set control strategy", -2, "ERROR", "��վ�Ѿ�����", "master already running, cannot set control strategy");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("phy id already exists", -2, "ERROR", "��վ�Ѿ�����", "phy id already exists");


		errorinfoPool->add<codeit::system::ErrorInfo>\
			("rt_task_create failed", -2, "ERROR", "ʵʱ���񴴽�ʧ��", "rt_task_create failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("codeit_rt_task_join failed", -2, "ERROR", "ʵʱ���񴴽�ʧ��", "codeit_rt_task_join failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("codeit_nrt_task_join failed", -2, "ERROR", "ʵʱ���񴴽�ʧ��", "codeit_nrt_task_join failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("rt_task_start failed", -2, "ERROR", "ʵʱ��������ʧ��", "rt_task_start failed");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Com controller initport fail", -2, "ERROR", "���ڿ������˿ڳ�ʼ��ʧ��", "Com controller initport fail");
		///****// Masterͨ�Ŵ��� //--------------------------------------


		///****// Interface���� //
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket can't WSAstartup", -2, "ERROR", "Socket������Ϊ��վ����", "Socket can't Start as server, because it can't WSAstartup");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket has empty port", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket has empty port");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket can't bind", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket can't Start as server, because it can't bind");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket can't listen", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket can't Start as server, because it can't listen");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket empty ip address", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket can't connect, because it empty ip address");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket empty port", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket can't connect, because it empty port");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket is busy now", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket can't connect, because it is busy now, please close it");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Socket can't connect", -2, "ERROR", "Socket�˿ں�Ϊ��", "Socket can't connect, because can't connect");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("socket setsockopt TCP_USER_TIMEOUT FAILED", -2, "ERROR", "��վ�Ѿ�����", "socket setsockopt TCP_USER_TIMEOUT FAILED");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("socket setsockopt SO_KEEPALIVE FAILED", -2, "ERROR", "��վ�Ѿ�����", "socket setsockopt SO_KEEPALIVE FAILED");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("setsockopt failed", -2, "ERROR", "��վ�Ѿ�����", "setsockopt failed: SO_REUSEADDR");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Com interface open listen thread fail", -2, "ERROR", "��վ�Ѿ�����", "Com interface open listen thread fail");
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("Com interface initPort fail", -2, "ERROR", "��վ�Ѿ�����", "Com interface initPort fail");

		///****// Interface���� //--------------------------------------


		///****// executerRT(),�˶��㷨���������errMsgMap //
		auto& cs = ControlSystem::instance();
		auto& errMap = cs.errorMap();
		errorinfoPool->add<codeit::system::ErrorInfo>\
			("unregistered key", -1000, "ERROR", "δע��ļ�ֵ", "unregistered key");
		errMap.insert(pair<std::int32_t, string>(-1000, "unregistered key"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("plan over time", codeit::function::BasisFunc::RetStatus::PLAN_OVER_TIME, "ERROR", "�滮��ʱ", "plan over time");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::PLAN_OVER_TIME, "plan over time"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("forward kinematic failed", codeit::function::BasisFunc::RetStatus::FORWARD_KINEMATIC_POSITION_FAILED, "ERROR", "����ʧ��", "forward kinematic failed");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::FORWARD_KINEMATIC_POSITION_FAILED, "forward kinematic failed"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("inverse kinematic failed", codeit::function::BasisFunc::RetStatus::INVERSE_KINEMATIC_POSITION_FAILED, "ERROR", "���ʧ��", "inverse kinematic failed");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::INVERSE_KINEMATIC_POSITION_FAILED, "inverse kinematic failed"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("wrist singularity", codeit::function::BasisFunc::RetStatus::WRIST_SINGULARITY, "ERROR", "�������", "wrist singularity");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::WRIST_SINGULARITY, "wrist singularity"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("shoulder singularity", codeit::function::BasisFunc::RetStatus::SHOULDER_SINGULARITY, "ERROR", "�粿�����", "shoulder singularity");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::SHOULDER_SINGULARITY, "shoulder singularity"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("elbow singularity", codeit::function::BasisFunc::RetStatus::ELBOW_SINGULARITY, "ERROR", "�ⲿ�����", "elbow singularity");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::ELBOW_SINGULARITY, "elbow singularity"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("three points collinear", codeit::function::BasisFunc::RetStatus::THREE_POINTS_COLLINEAR, "ERROR", "���㹲��", "three points collinear");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::THREE_POINTS_COLLINEAR, "three points collinear"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveJ planner", codeit::function::BasisFunc::RetStatus::NO_MOVEJ_PLANNER, "ERROR", "û��moveJ�滮��", "no moveJ planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVEJ_PLANNER, "no moveJ planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveL planner", codeit::function::BasisFunc::RetStatus::NO_MOVEL_PLANNER, "ERROR", "û��moveL�滮��", "no moveL planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVEL_PLANNER, "no moveL planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveC planner", codeit::function::BasisFunc::RetStatus::NO_MOVEC_PLANNER, "ERROR", "û��moveC�滮��", "no moveC planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVEC_PLANNER, "no moveC planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveS planner", codeit::function::BasisFunc::RetStatus::NO_MOVES_PLANNER, "ERROR", "û��moveS�滮��", "no moveS planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVES_PLANNER, "no moveS planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no moveLL planner", codeit::function::BasisFunc::RetStatus::NO_MOVELL_PLANNER, "ERROR", "û��moveLL�滮��", "no moveLL planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_MOVELL_PLANNER, "no moveLL planner"));

		errorinfoPool->add<codeit::system::ErrorInfo>\
			("no servoJ planner", codeit::function::BasisFunc::RetStatus::NO_SERVOJ_PLANNER, "ERROR", "û��servoJ�滮��", "no servoJ planner");
		errMap.insert(pair<std::int32_t, string>(codeit::function::BasisFunc::RetStatus::NO_SERVOJ_PLANNER, "no servoJ planner"));

		///****// executerRT(),�˶��㷨���� //--------------------------------------



		tmpErrorInfoPool(*errorinfoPool, errMap);

		return errorinfoPool;


	}
	////*********************************************************//////

	//*************************����ָ�************************//////
	auto createFuncRoot()->std::unique_ptr<codeit::function::FuncRoot>
	{
		std::unique_ptr<codeit::function::FuncRoot> cmd_root(new codeit::function::FuncRoot);

		///////////*************�˶���**************////////////
		cmd_root->funcPool().add<codeit::function::Enable>();
		cmd_root->funcPool().add<codeit::function::Disable>();
		cmd_root->funcPool().add<codeit::function::Mode>();
		cmd_root->funcPool().add<codeit::function::JogJ>();
		cmd_root->funcPool().add<codeit::function::ServoJ>();
		cmd_root->funcPool().add<codeit::function::JogC>();
		cmd_root->funcPool().add<codeit::function::MoveJ>();
		cmd_root->funcPool().add<codeit::function::MoveL>();
		cmd_root->funcPool().add<codeit::function::MoveC>();
		cmd_root->funcPool().add<codeit::function::MoveS>();
		cmd_root->funcPool().add<codeit::function::MoveLL>();
		cmd_root->funcPool().add<codeit::function::Reset>();
		cmd_root->funcPool().add<codeit::function::Home>();
		cmd_root->funcPool().add<codeit::function::FCPressL>();



		///////////*************�궨��**************////////////
		cmd_root->funcPool().add <codeit::function::CalibT6P>();
		cmd_root->funcPool().add <codeit::function::CalibT5P>();
		cmd_root->funcPool().add <codeit::function::CalibT4P>();
		cmd_root->funcPool().add <codeit::function::CalibJointZero>();
		cmd_root->funcPool().add <codeit::function::CalibCurDyn>();



		///////////*************ϵͳ��**************////////////
		cmd_root->funcPool().add<codeit::function::ConnectVrep>();
		cmd_root->funcPool().add<codeit::function::DisconnectVrep>();
		cmd_root->funcPool().add<codeit::function::Clear>();
		cmd_root->funcPool().add<codeit::function::Show>();
		cmd_root->funcPool().add<codeit::function::SetProgramRate>();
		cmd_root->funcPool().add<codeit::function::Start>();
		cmd_root->funcPool().add<codeit::function::Stop>();
		cmd_root->funcPool().add<codeit::function::Pause>();
		cmd_root->funcPool().add<codeit::function::CSstart>();
		cmd_root->funcPool().add<codeit::function::CSstop>();
		cmd_root->funcPool().add<codeit::function::SaveXml>();
		cmd_root->funcPool().add<codeit::function::GetXml>();
		cmd_root->funcPool().add<codeit::function::TeachPoint>();
		cmd_root->funcPool().add<codeit::function::DeletePoint>();
		cmd_root->funcPool().add<codeit::function::DefineVar>();
		cmd_root->funcPool().add<codeit::function::DeleteVar>();
		cmd_root->funcPool().add<codeit::function::SaveParam>();
		cmd_root->funcPool().add<codeit::function::DeleteParam>();
		cmd_root->funcPool().add<codeit::function::RenameParam>();
		cmd_root->funcPool().add<codeit::function::GetGravity>();
		cmd_root->funcPool().add<codeit::function::SetInstallAng>();
		cmd_root->funcPool().add<codeit::function::Recover>();
		cmd_root->funcPool().add<codeit::function::Get>();
		cmd_root->funcPool().add<codeit::function::Sleep>();
		cmd_root->funcPool().add<codeit::function::Wait>();
		cmd_root->funcPool().add<codeit::function::GetInfo>();
		cmd_root->funcPool().add<codeit::function::ProgramBlock>();
		cmd_root->funcPool().add<codeit::function::Var>();
		cmd_root->funcPool().add<codeit::function::OpenRNG>();
		cmd_root->funcPool().add<codeit::function::NRTstart>();
		cmd_root->funcPool().add<codeit::function::NRTstop>();
		cmd_root->funcPool().add<codeit::function::OpenInterface>();
		cmd_root->funcPool().add<codeit::function::CloseInterface>();
		//////////////////////*************IO��**************////////////
		cmd_root->funcPool().add<codeit::function::DOSet>();
		cmd_root->funcPool().add<codeit::function::DOPulse>();
		cmd_root->funcPool().add<codeit::function::DIGet>();
		cmd_root->funcPool().add<codeit::function::ReplayCmd>();

		////////////////////////*************ʾ��**************////////////
		//cmd_root->funcPool().add<codeit::function::MoveSine>();

		tmpFuncRoot(*cmd_root);

		//auto &rs = plan_root->planPool().add<kaanh::Reset>();
		//rs.command().findParam("pos")->setDefaultValue("{0.5,0.3925,0.7899,0.5,0.5,0.5}");
		//rs.command().findParam("pos")->setDefaultValue("{0.5,0.353,0.5,0.5,0.5,0.5}");
		return cmd_root;
	}
	////*********************************************************//////

	auto tmpErrorInfoPool(core::ObjectPool<codeit::system::ErrorInfo>& errorinfoPool, std::map<std::int32_t, std::string>& errMap)->void
	{

		errorinfoPool.add<codeit::system::ErrorInfo>\
			("master name not exist", -2, "ERROR", "��վ������", "master name not exist");

		errorinfoPool.add<codeit::system::ErrorInfo>\
			("Motion is in Interfer Region", -2, "ERROR", "�����˸�����", "Motion is in Interfer Region");

	}

	auto tmpFuncRoot(codeit::function::FuncRoot& cmd_root)->void
	{
		//////////////////////*************ʾ��**************////////////
		cmd_root.funcPool().add<codeit::function::MoveSine>();


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
	auto createDefaultData(codeit::model::Model& model)->void
{
	double zone[2] = { 0 };
	model.variablePool().add<codeit::model::MatrixVariable>("fine", core::Matrix(1, 2, zone));
	zone[0] = 0.001; zone[1] = 0.01;
	model.variablePool().add<codeit::model::MatrixVariable>("z1", core::Matrix(1, 2, zone));
	zone[0] = 0.005; zone[1] = 0.03;
	model.variablePool().add<codeit::model::MatrixVariable>("z5", core::Matrix(1, 2, zone));
	zone[0] = 0.01; zone[1] = 0.05;
	model.variablePool().add<codeit::model::MatrixVariable>("z10", core::Matrix(1, 2, zone));
	zone[0] = 0.015; zone[1] = 0.08;
	model.variablePool().add<codeit::model::MatrixVariable>("z15", core::Matrix(1, 2, zone));
	zone[0] = 0.02; zone[1] = 0.1;
	model.variablePool().add<codeit::model::MatrixVariable>("z20", core::Matrix(1, 2, zone));
	zone[0] = 0.03; zone[1] = 0.15;
	model.variablePool().add<codeit::model::MatrixVariable>("z30", core::Matrix(1, 2, zone));
	zone[0] = 0.04; zone[1] = 0.2;
	model.variablePool().add<codeit::model::MatrixVariable>("z40", core::Matrix(1, 2, zone));
	zone[0] = 0.05; zone[1] = 0.25;
	model.variablePool().add<codeit::model::MatrixVariable>("z50", core::Matrix(1, 2, zone));
	zone[0] = 0.06; zone[1] = 0.3;
	model.variablePool().add<codeit::model::MatrixVariable>("z60", core::Matrix(1, 2, zone));
	zone[0] = 0.08; zone[1] = 0.4;
	model.variablePool().add<codeit::model::MatrixVariable>("z80", core::Matrix(1, 2, zone));
	zone[0] = 0.1; zone[1] = 0.45;
	model.variablePool().add<codeit::model::MatrixVariable>("z100", core::Matrix(1, 2, zone));
	zone[0] = 0.2; zone[1] = 0.45;
	model.variablePool().add<codeit::model::MatrixVariable>("z150", core::Matrix(1, 2, zone));
	zone[0] = 0.06; zone[1] = 0.3;
	model.variablePool().add<codeit::model::MatrixVariable>("z200", core::Matrix(1, 2, zone));

	double v[5] = { 0 };
	v[0] = 0.005; v[1] = 0.005; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v5", core::Matrix(1, 5, v));
	v[0] = 0.01; v[1] = 0.01; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v10", core::Matrix(1, 5, v));
	v[0] = 0.025; v[1] = 0.025; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v25", core::Matrix(1, 5, v));
	v[0] = 0.03; v[1] = 0.03; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v30", core::Matrix(1, 5, v));
	v[0] = 0.04; v[1] = 0.04; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v40", core::Matrix(1, 5, v));
	v[0] = 0.05; v[1] = 0.05; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v50", core::Matrix(1, 5, v));
	v[0] = 0.06; v[1] = 0.06; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v60", core::Matrix(1, 5, v));
	v[0] = 0.08; v[1] = 0.08; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v80", core::Matrix(1, 5, v));
	v[0] = 0.1; v[1] = 0.1; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v100", core::Matrix(1, 5, v));
	v[0] = 0.15; v[1] = 0.15; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v150", core::Matrix(1, 5, v));
	v[0] = 0.2; v[1] = 0.2; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v200", core::Matrix(1, 5, v));
	v[0] = 0.3; v[1] = 0.3; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v300", core::Matrix(1, 5, v));
	v[0] = 0.4; v[1] = 0.4; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v400", core::Matrix(1, 5, v));
	v[0] = 0.5; v[1] = 0.5; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v500", core::Matrix(1, 5, v));
	v[0] = 0.6; v[1] = 0.6; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v600", core::Matrix(1, 5, v));
	v[0] = 0.8; v[1] = 0.8; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v800", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 1.0; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v1000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 1.5; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v1500", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 2.0; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v2000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 3.0; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v3000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 4.0; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v4000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 5.0; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v5000", core::Matrix(1, 5, v));
	v[0] = 1.0; v[1] = 6.0; v[2] = 200 * PI / 180;
	model.variablePool().add<codeit::model::MatrixVariable>("v6000", core::Matrix(1, 5, v));


	double load[10] = { 0 };
	model.variablePool().add<codeit::model::MatrixVariable>("load0", core::Matrix(1, 10, load));

	double zero[MAX_DOFS] = { 0 };
	model.variablePool().add<codeit::model::MatrixVariable>("zero_comp0", core::Matrix(1, MAX_DOFS, zero));

}
}
