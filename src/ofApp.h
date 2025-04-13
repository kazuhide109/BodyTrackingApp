#pragma once


#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxJSON.h"


//#pragma comment(lib, "k4a.lib")
#include <k4a/k4a.h>
#include <k4abt.hpp>

//#define HOST "localhost" //送信先ホストのIPを設定
//#define PORT 8888 //送信先のポート番号を設定

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void exit();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		k4a_device_t device = NULL;
		k4abt_tracker_t tracker;
		k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		k4a_calibration_t sensor_calibration;
		int bodyNum = 0;
		int r_screenX = 0;
		int r_screenY = 0;
		int l_screenX = 0;
		int l_screenY = 0;
		int c_screenX = 0;
		int c_screenY = 0;
		int d_screenX = 0;
		int d_screenY = 0;

		ofVec2f r_pos2d;
		ofVec2f l_pos2d;
		ofVec2f c_pos2d;
		int pos_diff = 0;
		ofVec2f d_pos2d;
		ofVec2f setJointPosInWindow( k4a_float3_t& jointPosition);

		ofTexture colorTexture;
		ofTexture depthTexture;
		ofTexture setColorToTex(k4a_image_t img);
		ofTexture setDepthToTex(k4a_image_t img);

		//ofMesh pointCloud;
		ofEasyCam cam;           // 3Dカメラ
		std::vector<ofVec3f> joints; // 関節位置のベクトル
		ofVec3f pos_rightHand;
		ofVec3f pos_leftHand;
		int state = 0;
		int stableCnt = 0;
		int latestDistance = 0;

		//OSC
		ofxOscSender sender;

		std::string ip;
		int port;
		int limitDistance = 0;
		int diffThred = 100;
		int smoothingFrames = 6; // 平均を取るフレーム数（約1秒）
		int camFixDistance = 0; //カメラ位置調整用
		int adjWinPosY = 0;
		float rotateAngle = 0; //回転角度
		int sideAreaSize = 1000;
		void loadConfig();

		//時間平均
		std::deque<ofVec2f> rightHandHistory2d;
		std::deque<ofVec2f> leftHandHistory2d;
		std::deque<ofVec2f> centerHandHistory2d;
		std::deque<ofVec2f> adjustedHandHistory2d;
		ofVec2f getSmoothedPosition(std::deque<ofVec2f>& history, ofVec2f newPos);

		int depthGetErrorCount = 0;
		
};
