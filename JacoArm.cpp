#include <iostream>
#include <Dragonfly.h>
#include <Dragonfly_config.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <sys/time.h>      //添加头文件
#include <pthread.h>
#include "JacoArmCtrl.h"
using std::cout;
using std::endl;
Dragonfly_Module* dfptr(nullptr);
CMessage msg;
CMessage PingAckMessage(MT_PING_ACK);
#define ARM
struct CartesianInfo
{
	float X;
	float Y;
	float Z;
};
// velocity from decode algorithm, send to arm every 5ms
CartesianInfo velocity;
void tick() {
		auto x = velocity.X;
		auto y = velocity.Y;
		auto z = velocity.Z;
		auto speed = sqrt(x * x + y * y + z * z);

		auto jaco = JacoArmCtrl::GetInstance();
		if (!jaco) return;

		// if (speed < -1E-5 || speed > 1E-5) {
			// jaco->CurrentVelocity(x,y,z,0);
		// }
		jaco->CurrentVelocity(x,y,z);

		Dragonfly_Module* df = dfptr;
		if (!df) 
		{
			cout<<"can't connect dragonfly"<<endl;
			return;
		}
		#ifdef ARM
			CMessage M1(MT_ROBOT_CONTROL_SPACE_ACTUAL_STATE_ARM);
			M1.AllocateData(sizeof(ROBOT_CONTROL_SPACE_ACTUAL_STATE));
			ROBOT_CONTROL_SPACE_ACTUAL_STATE data;
			// pos is the real arm position,feedback to the BCI system
			auto pos=JacoArmCtrl::GetInstance()->GetArmPosition();
			data.vel[0]=velocity.X;
			data.vel[1]=velocity.Y;
			data.vel[2]=velocity.Z;

			data.pos[0]=pos.x();
			data.pos[1]=pos.y();
			data.pos[2]=pos.z();
			M1.SetData(&data, sizeof(data));
			df->SendMessage(&M1);
		#endif
}

void* jaco_worker_thread(void *arg) {
	while (true) {
		usleep(5000);
		try {
			tick();
		}
		catch (...) {
			cout << "exception catched" << endl;
		}
	}
}

int main(int argc, char *argv[])
{
	JacoArmCtrl::GetInstance()->Connect();
	// bool flag_connect=JacoArmCtrl::GetInstance()->Connect();
	// if (!flag_connect) return -1;
	// JacoArmCtrl::GetInstance()->EraseAllProtectZones();
	// JacoArmCtrl::GetInstance()->SetProtectZone();
	cout<<"Jaco Arm connectd"<<endl;
	char mm_ip[] = "localhost:7111";
	PingAckMessage.AllocateData(sizeof(MDF_PING_ACK));
	cout << "Connecting to Dragonfly: ";
	MODULE_ID mod_id = MID_JACO_MOD;
	Dragonfly_Module dragonfly(mod_id, 0);
	dragonfly.ConnectToMMM(mm_ip);
	#ifdef ARM
		 dragonfly.Subscribe(MT_ROBOT_CONTROL_SPACE_ACTUAL_STATE_ARM);
	#endif
	dragonfly.Subscribe(MT_ROBOT_CONTROL_SPACE_ACTUAL_STATE);
	dragonfly.Subscribe(MT_PING);
	dragonfly.Subscribe(MT_EXIT);
	dragonfly.SendModuleReady();
	cout << "done." << endl;
	dfptr = &dragonfly;


	pthread_t th;
	if (pthread_create(&th, NULL, jaco_worker_thread, NULL) != 0 ) return -1;
	cout<<"new thread created"<<endl;	
	velocity.Z = 1;
	for (int i = 0; i < 200; i++) usleep(10000);
	velocity.Z = -1;
	for (int i = 0; i < 200; i++) usleep(10000);
	velocity.Z = 0;

	bool keep_running = true;
	while (keep_running)
	{
		CMessage M;
		dragonfly.ReadMessage(&M);
		std::cout << "Received message " << M.msg_type << std::endl;
		switch (M.msg_type)
		{
		case MT_ROBOT_CONTROL_SPACE_ACTUAL_STATE:
		{
			std::cout << "Velocity from BCI system: " << std::endl;
			ROBOT_CONTROL_SPACE_ACTUAL_STATE data;
			M.GetData(&data);
			std::cout << "  Data = [x: " << data.vel[0] << ", y: " << data.vel[1] << ", z: " << data.vel[2] << "]" << std::endl;

			velocity.X = data.vel[0];
			velocity.Y = data.vel[1];
			velocity.Z = data.vel[2];
			break;
		}
		case MT_PING:
		{
			char MODULE_NAME[] = "JacoArm";
			MDF_PING *pg = (MDF_PING *)M.GetDataPointer();
			if ((strcmp(pg->module_name, MODULE_NAME) == 0) ||
				(strcmp(pg->module_name, "*") == 0) ||
				(M.dest_mod_id == dragonfly.GetModuleID()))
			{
				MDF_PING_ACK *pa = (MDF_PING_ACK *)PingAckMessage.GetDataPointer();

				memset(pa, 0, sizeof(MDF_PING_ACK));
				for (int i = 0; i < strlen(MODULE_NAME); i++)
				{
					pa->module_name[i] = MODULE_NAME[i];
				}

				dragonfly.SendMessage(&PingAckMessage);
			}

			break;
		}
		case MT_EXIT:
		{
			if ((M.dest_mod_id == 0) || (M.dest_mod_id == dragonfly.GetModuleID()))
			{
				fprintf(stdout, "got exit!\n");
				dragonfly.SendSignal(MT_EXIT_ACK);
				dragonfly.DisconnectFromMMM();
			}
			keep_running = false;
			break;
		}
		}
	}
	JacoArmCtrl::GetInstance()->Disconnect();
	getchar();
	return 0;
}

