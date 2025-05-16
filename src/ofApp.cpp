#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
    ofSetVerticalSync(false);
    //ofSetFrameRate(60);

    connect();

    loadConfig();
    sender.setup(ip, port);
    ofxOscMessage m;
    m.setAddress("/start");
    m.addIntArg(1);
    sender.sendMessage(m);

    float angleRad = ofDegToRad(rotateAngle);
    float camY = -camFixDistance * tan(angleRad);
    cam.lookAt(ofVec3f(0, 0, 1000)); // �O������������
    cam.setPosition(0, camY, 0);

}

void ofApp::exit() {
	writeLofTxt("�A�v���P�[�V�������I�����܂���: ");
}


void ofApp::closeDevice() {
   
	ofLogNotice() << "�f�o�C�X���I��";
	//ofxOscMessage m;
	//m.setAddress("/end");
	//m.addIntArg(1);
	//sender.sendMessage(m);

	if (tracker != nullptr) {
		k4abt_tracker_shutdown(tracker);
		k4abt_tracker_destroy(tracker);
		tracker = nullptr;
	}
	ofLogNotice() << "�g���b�J�[������";

	if (device != nullptr) {
		k4a_device_stop_cameras(device);
		k4a_device_close(device);
		device = nullptr;
	}
	ofLogNotice() << "�f�o�C�X������";
}

void ofApp::connect() {
	ofLogNotice() << "�f�o�C�X�̐ڑ����J�n�B";

	uint32_t count = k4a_device_get_installed_count();
	ofLogNotice("�f�o�C�X�̐�") << count;

	if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device))) {
		ofLogFatalError() << "Azure Kinect �f�o�C�X�̃I�[�v���Ɏ��s���܂����B";
		ofExit();
		return;
	}

	// Get the size of the serial number
	size_t serial_size = 0;
	k4a_device_get_serialnum(device, NULL, &serial_size);
	char * serial = (char *)(malloc(serial_size));
	k4a_device_get_serialnum(device, serial, &serial_size);
	ofLogNotice("Open device") << serial;
	free(serial);

	// Start camera. Make sure depth camera is enabled.
	deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
	if (K4A_FAILED(k4a_device_start_cameras(device, &deviceConfig))) {
		ofLogFatalError() << "Kinect�J�����̊J�n�Ɏ��s���܂����B";
		ofExit();
		return;
	}

	if (K4A_FAILED(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration))) {
		ofLogFatalError() << "�J���[�o�����X�̎擾�Ɏ��s���܂����B";
		ofExit();
		return;
	}

	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
	if (K4A_FAILED(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker))) {
		ofLogFatalError() << "Body Tracking�̏������Ɏ��s���܂����B";
		ofExit();
		return;
	}

	ofLogNotice() << "Body Tracking�̏������ɐ������܂����B";
	
	deviceTimeoutCount = 0;
	isReconnecting = false;
}

void ofApp::writeLofTxt(string mm) {
	std::ofstream logFile("log.txt", std::ios::app); // append���[�h�ŊJ��
	if (logFile.is_open()) {
		logFile<< mm << mm << ofGetTimestampString() << std::endl;
		logFile.close();
	}
	else {
		ofLogError() << "���O�t�@�C�����J���܂���ł����B";
	}
}

void ofApp::reconnect(string reasonStr) {
	writeLofTxt(reasonStr);

	isReconnecting = true;
	closeDevice();
	ofLogNotice() << "�Đڑ����J�n�B";
	connect();
}




//--------------------------------------------------------------
void ofApp::update() {
    k4a_capture_t capture = nullptr;
    k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &capture, 1000);

	if (!isReconnecting) {

		if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED) {
			deviceTimeoutCount = 0;
			k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
			if (depth_image == NULL) {
				ofLogNotice() << "Failed to get depth image.";
				depthGetErrorCount += 1;
				if (depthGetErrorCount > sensorTimeoutThred) {
					string mm = "�f�v�X�J�����̃^�C���A�E�g���A�����܂����B�f�o�C�X���V���b�g�_�E�����A�Đڑ������݂܂��B ";
					std::cout << mm << endl;
					reconnect(mm);
				}
				return;
			}
			depthGetErrorCount = 0;

			k4a_image_t ir_image = k4a_capture_get_ir_image(capture);
			if (ir_image == NULL) {
				ofLogNotice() << "Failed to get ir image.";
				irGetErrorCount += 1;
				if (irGetErrorCount > sensorTimeoutThred) {
					string mm = "�ԊO���J�����̃^�C���A�E�g���A�����܂����B�f�o�C�X���V���b�g�_�E�����A�Đڑ������݂܂��B" ;
					std::cout << mm << endl;
					reconnect(mm);
				}
				return;
			}
			irGetErrorCount = 0;

			k4abt_frame_t bodyFrame = nullptr;
			k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0);
			if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED) {

				uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
				bodyNum = numBodies;

				if (bodyNum == 0) {
					stableCnt -= 1;
				}
				if (stableCnt < -10) {
					state = 0;
				}
				else {
					state = 1;
				}
				ofxOscMessage m;
				m.setAddress("/position/state");
				m.addIntArg(state);
				sender.sendMessage(m);

				//ofLogNotice("boduNum") << bodyNum;

				k4a_float2_t hand2D = { 0 };
				int valid = 0;
				ofQuaternion rotation;
				rotation.makeRotate(rotateAngle, ofVec3f(1, 0, 0));
				ofMatrix4x4 rotationMatrix;
				rotationMatrix.makeRotationMatrix(rotation);

				int latestDistance = limitDistance;
				for (uint32_t i = 0; i < bodyNum; i++) {
					//�X�P���g���̎擾
					k4abt_body_t body;
					k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton);
					body.id = k4abt_frame_get_body_id(bodyFrame, i);
					//�����Ɍ���
					int bodyCenter = body.skeleton.joints[0].position.xyz.x;
					if (bodyCenter < (sideAreaSize / 2) && bodyCenter > -(sideAreaSize / 2)) {

						//��Ԑ擪�ɂ���l�Ɍ���
						int baseDistance = body.skeleton.joints[0].position.xyz.z;
						if (baseDistance < latestDistance) {
							latestDistance = baseDistance;

							joints.clear();
							stableCnt = 0;

							for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++) {
								if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW) {
									k4a_float3_t & jointPosition = body.skeleton.joints[joint].position;
									//joints.push_back(ofVec3f(-jointPosition.xyz.x, -jointPosition.xyz.y, jointPosition.xyz.z));
									ofVec3f jointPos = ofVec3f(-jointPosition.xyz.x, -jointPosition.xyz.y, jointPosition.xyz.z);
									// ��]��K�p
									jointPos = rotationMatrix.preMult(jointPos);
									joints.push_back(jointPos);

									// �E��̍��W�v�Z
									if (joint == K4ABT_JOINT_WRIST_RIGHT) {
										pos_rightHand = ofVec3f(jointPos);
										r_pos2d = setJointPosInWindow(jointPosition);
									}

									// ����̍��W�v�Z
									if (joint == K4ABT_JOINT_WRIST_LEFT) {
										pos_leftHand = ofVec3f(jointPos);
										l_pos2d = setJointPosInWindow(jointPosition);
									}

									/*if (joint == K4ABT_JOINT_HEAD) {
                                    camFixDistance = jointPos.z;
                                }*/
								}
							}

							//��ʓ��̏c�����̒���
							r_pos2d.y = r_pos2d.y - adjWinPosY;
							l_pos2d.y = l_pos2d.y - adjWinPosY;

							// ���Ԉʒu�̍��W�v�Z
							c_pos2d = ofVec2f((int(float(r_pos2d.x + l_pos2d.x) / 2)), (int(float(r_pos2d.y + l_pos2d.y) / 2)));
							c_screenX = c_pos2d.x;
							c_screenY = c_pos2d.y;

							//����ʒu�̌���
							pos_diff = r_pos2d.distance(l_pos2d);
							if (pos_diff < diffThred) {
								d_pos2d = c_pos2d;
							}
							else {
								d_pos2d = ((l_pos2d.y - r_pos2d.y) < 0) ? l_pos2d : r_pos2d;
							}

							// �X���[�W���O�K�p
							ofVec2f smoothedRightHand = getSmoothedPosition(rightHandHistory2d, r_pos2d);
							r_pos2d = smoothedRightHand;
							r_screenX = r_pos2d.x;
							r_screenY = r_pos2d.y;
							ofVec2f smoothedLeftHand = getSmoothedPosition(leftHandHistory2d, l_pos2d);
							l_pos2d = smoothedLeftHand;
							l_screenX = l_pos2d.x;
							l_screenY = l_pos2d.y;
							ofVec2f smoothedCenterHand = getSmoothedPosition(centerHandHistory2d, c_pos2d);
							c_pos2d = smoothedCenterHand;
							c_screenX = c_pos2d.x;
							c_screenY = c_pos2d.y;
							ofVec2f smoothedAdjustedHand = getSmoothedPosition(adjustedHandHistory2d, d_pos2d);
							d_pos2d = smoothedAdjustedHand;
							d_screenX = d_pos2d.x;
							d_screenY = d_pos2d.y;

							ofxOscMessage m1;
							m1.setAddress("/position/rightHand");
							m1.addIntArg(r_screenX);
							m1.addIntArg(r_screenY);
							sender.sendMessage(m1);
							ofxOscMessage m2;
							m2.setAddress("/position/leftHand");
							m2.addIntArg(l_screenX);
							m2.addIntArg(l_screenY);
							sender.sendMessage(m2);
							ofxOscMessage m3;
							m3.setAddress("/position/centerHand");
							m3.addIntArg(c_screenX);
							m3.addIntArg(c_screenY);
							sender.sendMessage(m3);
							ofxOscMessage m4;
							m4.setAddress("/position/adjustedHand");
							m4.addIntArg(d_screenX);
							m4.addIntArg(d_screenY);
							sender.sendMessage(m4);
							//ofLogNotice("Send OSC");
						}
					}
				}

				k4abt_frame_release(bodyFrame);
			}

			//�[�x�摜�̍쐬
			if (depth_image != NULL) {
				depthTexture = setDepthToTex(depth_image);
			}

			// �摜�̃��������
			k4a_image_release(depth_image);
			k4a_image_release(ir_image);
			k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, capture, 0);
			k4a_capture_release(capture);
			if (queueCaptureResult == K4A_WAIT_RESULT_FAILED) {
				ofLogNotice() << "Error! Add capture to tracker process queue failed!";
				return;
			}

		} else if (getCaptureResult == K4A_WAIT_RESULT_FAILED) {
			ofLogFatalError() << "�f�o�C�X����̃L���v�`���擾�Ɏ��s���܂����iK4A_WAIT_RESULT_FAILED�j�B�A�v���P�[�V�������I�����܂��B";
			closeDevice();
			ofExit();
			return;
		} else if (getCaptureResult == K4A_WAIT_RESULT_TIMEOUT) {
			// �^�C���A�E�g�̏ꍇ�͖����A�������̓��g���C
			ofLogNotice() << "�L���v�`���擾���^�C���A�E�g���܂����B( K4A_WAIT_RESULT_TIMEOUT)" << deviceTimeoutCount;

			deviceTimeoutCount += 1;
			if (deviceTimeoutCount > timeoutThreshold) {
				string mm = "�^�C���A�E�g���A�����܂����B�f�o�C�X���V���b�g�_�E�����A�Đڑ������݂܂��B ";
				std::cout << mm << endl;
				reconnect(mm);
			}
			return;
		}
	}

}

//--------------------------------------------------------------
void ofApp::draw() {
    ofSetColor(ofColor::white);
    // �J���[�摜��\��
    int winW = ofGetWidth();
    int winH = ofGetHeight();
    if (depthTexture.isAllocated()) {
        depthTexture.draw(0, 0, winW, winH);
    }

	if (!isReconnecting) {

		// �`��
		cam.begin(); // 3D�J�����̊J�n

		// X���i�ԐF�j
		ofSetColor(255, 0, 0);
		ofDrawArrow(ofVec3f(0, 0, 0), ofVec3f(200, 0, 0), 10);
		// Y���i�ΐF�j
		ofSetColor(0, 255, 0);
		ofDrawArrow(ofVec3f(0, 0, 0), ofVec3f(0, 200, 0), 10);
		// Z���i�F�j
		ofSetColor(0, 0, 255);
		ofDrawArrow(ofVec3f(0, 0, 0), ofVec3f(0, 0, 200), 10);

		// �֐߂̕`��
		ofSetColor(0, 0, 255);
		ofDrawSphere(0, 0, 1500, 10);
		//ofLogNotice("joints size") << joints.size();
		if (joints.size() > 0) {
			ofSetColor(180, 0, 0);
			for (const auto & joint : joints) {
				ofDrawSphere(joint, 10); // �֐߂����̂ŕ`��
			}

			// �֐ߊԂ̐���`��i�K�v�ɉ����Ēǉ��j
			for (int i = 0; i < joints.size() - 1; i++) {
				ofDrawLine(joints[i], joints[i + 1]);
			}
		}

		cam.end(); // 3D�J�����̏I��

		// ��ʓ��̈ʒu�̕`��
		ofSetColor(0, 255, 0);
		ofDrawCircle(l_pos2d, 20);
		ofDrawCircle(r_pos2d, 20);
		ofSetColor(0, 200, 200);
		ofDrawCircle(c_pos2d, 20);
		ofSetColor(255, 0, 0);
		ofDrawCircle(d_pos2d, 16);
		ofDrawBitmapStringHighlight("diff:" + ofToString(pos_diff), c_pos2d);
	}

    //�m�F�p�̃p�����[�^
    int fps = ofGetFrameRate();
    ofDrawBitmapStringHighlight(ofToString(fps), 10, 20);
    ofDrawBitmapStringHighlight("BodyNum" + ofToString(bodyNum), 10, 40);
    ofDrawBitmapStringHighlight("State" + ofToString(state), 120, 40);
    ofDrawBitmapStringHighlight("RightHand screenX: " + ofToString(r_screenX), 10, 60);
    ofDrawBitmapStringHighlight("RightHand screenY: " + ofToString(r_screenY), 10, 80);
    ofDrawBitmapStringHighlight("LeftHand screenX: " + ofToString(l_screenX), 10, 100);
    ofDrawBitmapStringHighlight("LeftHand screenY : " + ofToString(l_screenY), 10, 120);
    ofDrawBitmapStringHighlight("Center screenX: " + ofToString(c_screenX), 10, 140);
    ofDrawBitmapStringHighlight("Center screenY : " + ofToString(c_screenY), 10, 160);
    ofDrawBitmapStringHighlight("LatestDistance : " + ofToString(latestDistance), 10, 200);
}

ofTexture ofApp::setColorToTex(k4a_image_t img) {
    int width = k4a_image_get_width_pixels(img);
    int height = k4a_image_get_height_pixels(img);
    uint8_t* buffer = k4a_image_get_buffer(img);
    ofTexture tex;
    if (buffer == nullptr) {
        ofLogError("buffer") << "Image buffer is null!";
        return tex;
    }
    ofPixels pixels;
    pixels.setFromExternalPixels(buffer, width, height, OF_PIXELS_BGRA);
    pixels.swapRgb();
    tex.allocate(pixels);
    return tex;
}

ofTexture ofApp::setDepthToTex(k4a_image_t img) {
    int width = k4a_image_get_width_pixels(img);
    int height = k4a_image_get_height_pixels(img);
    uint8_t* buffer = k4a_image_get_buffer(img);
    ofTexture tex;

    if (buffer == nullptr) {
        ofLogError("buffer") << "Image buffer is null!";
        return tex;
    }
    ofShortPixels depthPixels;
    depthPixels.setFromExternalPixels((unsigned short*)k4a_image_get_buffer(img), width, height, 1);

    // �[�x�摜�𐳋K�����ĕ`��\�Ȍ`���ɕϊ�
    unsigned char* depthPix = new unsigned char[width * height];
    for (int i = 0; i < width * height; i++) {
        depthPix[i] = (depthPixels[i] > 500) ? ofMap(depthPixels[i], 500, 2500, 255, 0, true) : 0;  // �K�؂Ȓl�ɒ���

    }

    tex.loadData(depthPix, width, height, GL_LUMINANCE);
    delete[] depthPix;
    return tex;
}

// --- �ړ����ς��v�Z����֐� ---
ofVec2f  ofApp::getSmoothedPosition(std::deque<ofVec2f>& history, ofVec2f newPos) {
    // �ŐV�̃f�[�^��ǉ�
    history.push_back(newPos);

    // �������ݒ�l��葽����΍ł��Â��f�[�^���폜
    if (history.size() > smoothingFrames) {
        history.pop_front();
    }

    // ���ϒl���v�Z
    ofVec2f smoothedPos(0, 0);
    for (const auto& pos : history) {
        smoothedPos += pos;
    }
    smoothedPos /= history.size();

    return smoothedPos;
}

ofVec2f ofApp::setJointPosInWindow(k4a_float3_t& jointPosition) {
    float screenX = 0;
    float screenY = 0;
    k4a_float2_t hand2D = { 0 };
    int valid = 0;

    k4a_result_t result = k4a_calibration_3d_to_2d(
        &sensor_calibration,
        &jointPosition,
        K4A_CALIBRATION_TYPE_DEPTH, // Depth�J������Ԃł̕ϊ�
        K4A_CALIBRATION_TYPE_DEPTH, // 2D���W���J���[�摜�Ƀ}�b�s���O
        &hand2D,
        &valid);
    if (result == K4A_RESULT_SUCCEEDED)
    {
        screenX = hand2D.xy.x;
        screenY = hand2D.xy.y;
    }
    else
    {
        ofLogNotice("Failed to project 3D position to 2D!");
    }
    return ofVec2f(screenX, screenY);
}

void ofApp::loadConfig() {
    ofxJSONElement json;
    if (!json.open("setting.json")) {
        ofLogError() << "Failed to load config.json";
        return;
    }

    ip = json["ip"].asString();
    port = json["port"].asInt();
    ofLogNotice() << "Loaded IP: " << ip << ", Port: " << port;
    limitDistance = json["distance"].asInt();
    diffThred = json["diffThred"].asInt();
    smoothingFrames = json["smoothingFrames"].asInt();
    camFixDistance = json["standPosition"].asInt();
    rotateAngle = json["rotateAngle"].asFloat();
    adjWinPosY = json["adjWinPosY"].asInt();
    sideAreaSize = json["sideAreaSize"].asInt();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
