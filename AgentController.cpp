#include <string>  //stringをインクルード
#include "Controller.h"
#include "Rotation.h"
#include "quatanion.h"
#include "Logger.h"
#include "ControllerEvent.h"
#include "PSMoveData.h"

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define RAD2DEG(RAD) ( (RAD) * 180.0 / (PI) )

using namespace std;  //利用名前空間の定義

class AgentController : public Controller
{
public:
	//シミュレーション開始時に一度だけ呼出される関数onInitの利用を宣言します。
	void onInit(InitEvent &evt);
	double onAction(ActionEvent &evt);
	// メッセージを受信時に呼び出される関数onRecvMsgの利用を宣言します
	void onRecvMsg(RecvMsgEvent &evt);

	void moveByPSMove();
private:
	std::vector<MoveData> moves;
	MoveData moveData;

	SimObj *object;
	float init_qw, init_qx, init_qy, init_qz;
	//Vector3d ini_pos;
	//Vector3d ini_ros;
};

void AgentController::onInit(InitEvent &evt)
{
	try {
		SimObj *my = getObj(myname());
		if (!my->dynamics()) {

			// 左手を下に下げます
			my->setJointAngle("LARM_JOINT2", DEG2RAD(-90));

			// 右手を下に下げます
			my->setJointAngle("RARM_JOINT2", DEG2RAD(90));
		}
	} catch(SimObj::Exception &) {
		;
	}
	
}

double AgentController::onAction(ActionEvent &evt)
{
	return 1.0;
}


void AgentController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	//自分自身の取得
	SimObj *my = getObj(myname());

	//string型のメッセージを取得します
	string msg = evt.getMsg();
	if (msg =="Hello"){

		//腰の関節を45(deg)曲げます
		my->setJointQuaternion("WAIST_JOINT1", 0.707,0.707,0.0,0.0);
	}
	else {
		if(sender == "PSMoveAPIService"){
			moves = buildMoveData(msg);
			//LOG_MSG(("moves: %f, %f, %f, %f", moves[0].q.w, moves[0].q.x, moves[0].q.y, moves[0].q.z));
			moveByPSMove();
		}
	}
}

void AgentController::moveByPSMove()
{
	//SimObj *my = getObj(myname());
  
	object = getObj("petbottle_1");
	//object->getPosition(ini_pos);

	//Position
	double x = 4.0*moves[0].tracker.x;
	double y = 4.0*moves[0].tracker.y;
	double z = 4.0*moves[0].tracker.z;
	object->setPosition(x, y, z);

	//Init Quaternion
	unsigned int button_num;
	LOG_MSG(("button num : %x", moves[0].pressedButtons));
	if(moves[0].pressedButtons == 0x80000){
		//Invert sign of imaginary parts for PSMove coordinate system to SIGVerse coordinate system
		init_qw =  moves[0].q.w;
		init_qx = -moves[0].q.x;
		init_qy = -moves[0].q.y;
		init_qz = -moves[0].q.z;
	}
	else if(init_qw == 0)
	{
		init_qw = 1;
		init_qx = 0;
		init_qy = 0;
		init_qz = 0;
	}

	//Current Quaternion
	float qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
	float src_qw = 1.0, src_qx = 0.0, src_qy = 0.0, src_qz = 0.0;
	float rel_qw, rel_qx, rel_qy, rel_qz;
	float temp_qw, temp_qx, temp_qy, temp_qz;
	float obj_qw = 1.0, obj_qx = 0.0, obj_qy = 0.0, obj_qz = 0.0;
	float temp_obj_qw, temp_obj_qx, temp_obj_qy, temp_obj_qz;

	//Invert sign of imaginary parts for PSMove coordinate system to SIGVerse coordinate system
	src_qw =  moves[0].q.w;
	src_qx = -moves[0].q.x;
	src_qy = -moves[0].q.y;
	src_qz = -moves[0].q.z;
	
	
	quatMul(src_qw, src_qx, src_qy, src_qz, init_qw, -init_qx, -init_qy, -init_qz, &temp_qw, &temp_qx, &temp_qy, &temp_qz);
	//quatMul(obj_qw, obj_qx, obj_qy, obj_qz, init_qw, -init_qx, -init_qy, -init_qz, &temp_obj_qw, &temp_obj_qx, &temp_obj_qy, &temp_obj_qz);
	quatMul(init_qw, init_qx, init_qy, init_qz, obj_qw, -obj_qx, -obj_qy, -obj_qz, &temp_obj_qw, &temp_obj_qx, &temp_obj_qy, &temp_obj_qz);
	//quatMul(temp_obj_qw, temp_obj_qx, temp_obj_qy, temp_obj_qz, temp_qw, -temp_qx, -temp_qy, -temp_qz, &obj_qw, &obj_qx, &obj_qy, &obj_qz);
	quatMul(temp_qw, -temp_qx, -temp_qy, -temp_qz, temp_obj_qw, temp_obj_qx, temp_obj_qy, temp_obj_qz, &obj_qw, &obj_qx, &obj_qy, &obj_qz);
	quatMul(obj_qw, obj_qx, obj_qy, obj_qz, init_qw, -init_qx, -init_qy, -init_qz, &qw, &qx, &qy, &qz);
	
/*	//Invert sign of imaginary parts for PSMove coordinate system to SIGVerse coordinate system
	quatMul(src_qw, src_qx, src_qy, src_qz, init_qw, -init_qx, -init_qy, -init_qz, &temp_qw, &temp_qx, &temp_qy, &temp_qz);
	quatMul(obj_qw, obj_qx, obj_qy, obj_qz, temp_qw, -temp_qx, -temp_qy, -temp_qz, &qw, &qx, &qy, &qz);
*/
	
	//LOG_MSG(("&qw: %f,&qx: %f,&qy: %f,&qz: %f", rel_qw, rel_qx, rel_qy, rel_qz));
	//object->setRotation(Rotation(obj_qw, obj_qx, obj_qy, obj_qz));
	object->setRotation(Rotation(qw, qx, qy, qz));

}

extern "C"  Controller * createController ()
{
	return new AgentController;
}
