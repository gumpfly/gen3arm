#include <BaseClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <DeviceManagerClientRpc.h>
namespace k_api = Kinova::Api;
#define IP_ADDRESS "192.168.1.10"
#define PORT 10000
class JacoArmCtrl
{
public:
	JacoArmCtrl();
	~JacoArmCtrl();
	
	void Connect();
	void Disconnect();
	void MoveHome();
	void CurrentVelocity(float x,float y,float z);
	void CartesianCtrl(float x,float y,float z);
	k_api::Base::Pose GetArmPosition();
	// JointAngles GetArmAngle();
	// void SetProtectZone();
	static JacoArmCtrl *GetInstance();
	static void DistroyArmInstance();
private:
	k_api::Base::TwistCommand TwistPose;
	k_api::TransportClientUdp *pTransport=NULL;
	k_api::RouterClient *pRouter=NULL;
	k_api::SessionManager *pSessionMng=NULL;
	k_api::Base::BaseClient *pBase=NULL;
	bool connect_flag;
};
