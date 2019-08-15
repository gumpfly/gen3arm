#include "JacoArmCtrl.h"
using std::cout;
using std::endl;

JacoArmCtrl *pArm(nullptr);

JacoArmCtrl::JacoArmCtrl():pTransport(NULL),pRouter(NULL),pSessionMng(NULL),pBase(NULL)
{
}

JacoArmCtrl::~JacoArmCtrl()
{
	// Close API session
	pSessionMng->CloseSession();
	// Deactivate the router and cleanly disconnect from the transport object
	pRouter->SetActivationStatus(false);
	pTransport->disconnect();
	delete pBase;
	delete pSessionMng;
	delete pRouter;
	delete pTransport;
}
void JacoArmCtrl::Connect()
{
	// Setup API
	pTransport = new k_api::TransportClientUdp();
	pRouter = new k_api::RouterClient(pTransport, [](k_api::KError err){ std::cout << "_________ callback error _________" << err.toString(); });
	pTransport->connect(IP_ADDRESS, PORT);

	// Create session
	auto createSessionInfo = k_api::Session::CreateSessionInfo();
	createSessionInfo.set_username("admin");
	createSessionInfo.set_password("admin");
	createSessionInfo.set_session_inactivity_timeout(60000);   // (milliseconds)
	createSessionInfo.set_connection_inactivity_timeout(2000); // (milliseconds)

	std::cout << "\nCreating session for communication" << std::endl;
	pSessionMng = new k_api::SessionManager(pRouter);
	pSessionMng->CreateSession(createSessionInfo);
	std::cout << "Session created" << std::endl;

	// Create required services
	pBase = new k_api::Base::BaseClient(pRouter);
}
void JacoArmCtrl::Disconnect()
{
	if (connect_flag)
	{
		cout<<" stop arm control..."<<endl;
		DistroyArmInstance();
	}
	connect_flag=false;
}
void JacoArmCtrl::MoveHome()
{
	std::cout << "\nMoving the arm to a safe position before executing example" << std::endl;
	auto action_type = k_api::Base::RequestedActionType();
	action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
	auto action_list = pBase->ReadAllActions(action_type);
	auto action_handle = k_api::Base::ActionHandle();
	action_handle.set_identifier(0); 
	for( auto action : action_list.action_list())
	{
		if(action.name() == "Home")
		{
			action_handle = action.handle();
		}
	}

	if(action_handle.identifier() == 0)
	{
		std::cout << "\nCan't reach safe position. Exiting" << std::endl;		
	}
	else
	{
		pBase->ExecuteActionFromReference(action_handle);
		std::this_thread::sleep_for(std::chrono::seconds(2)); // Leave time to action to finish
	}
}

JacoArmCtrl *JacoArmCtrl::GetInstance()
{
		if(pArm==nullptr)
		{
				pArm=new JacoArmCtrl;
		}
		return pArm;
}
void JacoArmCtrl::DistroyArmInstance()
{
		if (pArm != nullptr)
		{
				delete pArm;
				pArm=nullptr;
		}
}
void JacoArmCtrl::CurrentVelocity(float x,float y,float z)
{
	auto pose=TwistPose.mutable_twist();
	pose->set_linear_x(x);
	pose->set_linear_y(y);
	pose->set_linear_z(z);
	pose->set_angular_x(0.0);
	pose->set_angular_y(0.0);
	pose->set_angular_z(0.0);
	pBase->SendTwistCommand(TwistPose);
}
k_api::Base::Pose JacoArmCtrl::GetArmPosition()
{
	auto pose = pBase->GetMeasuredCartesianPose();
	std::cout << "x: "<<pose.x() <<" y: "<<pose.y()<<" z: "<<pose.z()<<std::endl;
	std::cout << "theta_x: "<<pose.theta_x() <<" theta_y: "<<pose.theta_y()<<" theta_z: "<<pose.theta_z()<<std::endl;
	return pose;
}	
void JacoArmCtrl::CartesianCtrl(float x, float y,float z)
{
	
	auto action = k_api::Base::Action();
	action.set_name("Example Cartesian action movement");
	action.set_application_data("");
	auto constrainedPose = action.mutable_reach_pose();
	auto pose = constrainedPose->mutable_target_pose();
	pose->set_x(x);			  // x (meters)
	pose->set_y(y);			  // y (meters)
	pose->set_z(z);			 // z (meters)
	pose->set_theta_x(0.0);    // theta x (degrees)
	pose->set_theta_y(0.0);    // theta y (degrees)
	pose->set_theta_z(0.0);    // theta z (degrees)
	std::cout << "Executing action" << std::endl;
	pBase->ExecuteAction(action);
	std::cout << "Waiting 20 seconds for movement to finish ..." << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(20000)); 
	std::cout << "Cartesian movement completed" << std::endl;
}

