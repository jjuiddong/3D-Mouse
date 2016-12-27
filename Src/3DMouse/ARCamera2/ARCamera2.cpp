//
// AR Camera 테스트
// 2개의 마커를 인식해, 매인 마커를 원점으로한, 2번째 마커의 상대 좌표를 계산한다.
//

#include "../../../../Common/Common/common.h"
#include "../../../../Common/Graphic/graphic.h"
#include "../../../../Common/Framework/framework.h"

using namespace common;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>


#ifdef _DEBUG
#pragma comment(lib, "opencv_core310d.lib")
#pragma comment(lib, "opencv_imgcodecs310d.lib")
#pragma comment(lib, "opencv_videoio310d.lib")
#pragma comment(lib, "opencv_highgui310d.lib")
#pragma comment(lib, "opencv_imgproc310d.lib")
#pragma comment(lib, "opencv_calib3d310d.lib")
#pragma comment(lib, "opencv_aruco310d.lib")
#else
#pragma comment(lib, "opencv_core310.lib")
#pragma comment(lib, "opencv_imgcodecs310.lib")
#pragma comment(lib, "opencv_videoio310.lib")
#pragma comment(lib, "opencv_highgui310.lib")
#pragma comment(lib, "opencv_imgproc310.lib")
#pragma comment(lib, "opencv_calib3d310.lib")
#pragma comment(lib, "opencv_aruco310.lib")
#endif


using namespace graphic;
using namespace cv;


class cViewer : public framework::cGameMain
{
public:
	cViewer();
	virtual ~cViewer();

	virtual bool OnInit() override;
	virtual void OnUpdate(const float elapseT) override;
	virtual void OnRender(const float elapseT) override;
	virtual void OnShutdown() override;
	virtual void OnMessageProc(UINT message, WPARAM wParam, LPARAM lParam) override;


private:
	int m_cameraMode; // 0=main cam, 1=sub cam (toggle enter key)
	cCamera *m_camera[4];
	LPD3DXSPRITE m_sprite;
	graphic::cSprite *m_videoSprite;
	graphic::cCharacter m_character;
	graphic::cCube m_cube;
	graphic::cCube2 m_cube2;
	graphic::cModel m_box;

	VideoCapture m_inputVideo;
	Mat m_camImage;

	struct sMarker
	{
		int id;
		Matrix44 tm;
	};
	vector<sMarker> m_markers;

	Ptr<aruco::DetectorParameters> m_detectorParams;
	Ptr<aruco::Dictionary> m_dictionary;
	Mat m_camMatrix;
	Mat m_distCoeffs;
	float m_markerLength = 75;

	Matrix44 m_zealotCameraView;
	bool m_toggleUpdateTm;

	POINT m_curPos;
	bool m_LButtonDown;
	bool m_RButtonDown;
	bool m_MButtonDown;
};

INIT_FRAMEWORK(cViewer);

const int WINSIZE_X = 640;
const int WINSIZE_Y = 480;

cViewer::cViewer() :
	m_character(1000)
	, m_cameraMode(0)
	, m_toggleUpdateTm(true)
	, m_box(0)
{
	m_windowName = L"AR Camera2";
	const RECT r = { 0, 0, WINSIZE_X, WINSIZE_Y };
	m_windowRect = r;
	m_LButtonDown = false;
	m_RButtonDown = false;
	m_MButtonDown = false;
}

cViewer::~cViewer()
{
	SAFE_DELETE(m_videoSprite);
	m_sprite->Release();
	graphic::ReleaseRenderer();

	for (int i = 1; i < 4; ++i)
		delete m_camera[i];
}


static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return true;
}


static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
	fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
	fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
	fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
	fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
	fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
	fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
	fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
	fs["minDistanceToBorder"] >> params->minDistanceToBorder;
	fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
	fs["doCornerRefinement"] >> params->doCornerRefinement;
	fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
	fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
	fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
	fs["markerBorderBits"] >> params->markerBorderBits;
	fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
	fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
	fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
	fs["minOtsuStdDev"] >> params->minOtsuStdDev;
	fs["errorCorrectionRate"] >> params->errorCorrectionRate;
	return true;
}


bool cViewer::OnInit()
{
	graphic::cResourceManager::Get()->SetMediaDirectory("../media/");

	m_cube.SetCube(m_renderer, Vector3(-1, -1, -1), Vector3(1, 1, 1));
	m_cube2.SetCube(m_renderer, Vector3(-1, -1, -1), Vector3(1, 1, 1));

	// start craft 2
	// zealot
	{
		m_character.Create(m_renderer, "zealot.dat");
		if (graphic::cMesh* mesh = m_character.GetMesh("Sphere001"))
			mesh->SetRender(false);
		m_character.SetShader(graphic::cResourceManager::Get()->LoadShader(m_renderer,
			"hlsl_skinning_using_texcoord_sc2.fx"));
		m_character.SetRenderShadow(true);

		vector<sActionData> actions;
		actions.reserve(16);
		actions.push_back(sActionData(CHARACTER_ACTION::NORMAL, "zealot_stand.ani"));
		actions.push_back(sActionData(CHARACTER_ACTION::RUN, "zealot_walk.ani"));
		actions.push_back(sActionData(CHARACTER_ACTION::ATTACK, "zealot_attack.ani"));
		m_character.SetActionData(actions);
		m_character.Action(CHARACTER_ACTION::RUN);
	}

	{
		m_box.Create(m_renderer, "cube.dat");
		Matrix44 tm;
		tm.SetScale(Vector3(1, 1, 1)*0.05f);
		m_box.SetTransform(tm);
	}

	D3DXCreateSprite(m_renderer.GetDevice(), &m_sprite);
	m_videoSprite = new graphic::cSprite(m_sprite, 0);
	m_videoSprite->SetTexture(m_renderer, "kim.jpg");
	m_videoSprite->SetPos(Vector3(0, 0, 0));

	for (int i = 0; i < 4; ++i)
	{
		if (i == 0)
			m_camera[i] = GetMainCamera();
		else
			m_camera[i] = new cCamera();

		m_camera[i]->Init(&m_renderer);
		m_camera[i]->SetCamera(Vector3(10, 10, -10), Vector3(0, 0, 0), Vector3(0, 1, 0));
		m_camera[i]->SetProjection(D3DX_PI / 4.f, (float)WINSIZE_X / (float)WINSIZE_Y, 1.f, 10000.0f);
	}

	GetMainLight().Init(cLight::LIGHT_DIRECTIONAL);
	GetMainLight().SetPosition(Vector3(5, 5, 5));
	GetMainLight().SetDirection(Vector3(1, -1, 1).Normal());

	m_renderer.GetDevice()->SetRenderState(D3DRS_NORMALIZENORMALS, TRUE);
	m_renderer.GetDevice()->LightEnable(0, true);

	m_detectorParams = aruco::DetectorParameters::create();
	readDetectorParameters("detector_params.yml", m_detectorParams);
	m_detectorParams->doCornerRefinement = true; // do corner refinement in markers
	m_dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);

	readCameraParameters("camera.yml", m_camMatrix, m_distCoeffs);

	m_inputVideo.open(0);

	m_markers.reserve(10);

	return true;
}


void cViewer::OnUpdate(const float elapseT)
{
	if (!m_inputVideo.grab())
		return;

	Mat image;
	m_inputVideo.retrieve(image);
	if (!image.data)
		return;

	vector< int > ids;
	vector< vector< Point2f > > corners, rejected;
	vector< Vec3d > rvecs, tvecs;
	m_markers.clear();
	set<int> dupTest;//중복테스트

	// detect markers and estimate pose
	aruco::detectMarkers(image, m_dictionary, corners, ids, m_detectorParams, rejected);

	if (ids.size() > 0)
	{
		aruco::estimatePoseSingleMarkers(corners, m_markerLength, m_camMatrix, m_distCoeffs, rvecs, tvecs);
		aruco::drawDetectedMarkers(image, corners, ids);

		for (unsigned int i = 0; i < ids.size(); i++)
		{
			if (dupTest.find(ids[i]) != dupTest.end())
				continue;
			dupTest.insert(ids[i]);

			aruco::drawAxis(image, m_camMatrix, m_distCoeffs, rvecs[i], tvecs[i], m_markerLength * 0.5f);

			// change aruco space to direct x space
			Mat rot;
			Rodrigues(rvecs[i], rot);
			Mat invRot;
			transpose(rot, invRot); // inverse matrix
			double *pinvR = invRot.ptr<double>();

			Matrix44 tm;
			tm.m[0][0] = -(float)pinvR[0];
			tm.m[0][1] = (float)pinvR[1];
			tm.m[0][2] = -(float)pinvR[2];

			tm.m[1][0] = -(float)pinvR[3];
			tm.m[1][1] = (float)pinvR[4];
			tm.m[1][2] = -(float)pinvR[5];

			tm.m[2][0] = -(float)pinvR[6];
			tm.m[2][1] = (float)pinvR[7];
			tm.m[2][2] = -(float)pinvR[8];

			Matrix44 rot2;
			rot2.SetRotationX(ANGLE2RAD(-90)); // y-z axis change

			Matrix44 trans;
			trans.SetPosition(Vector3((float)tvecs[i][0], -(float)tvecs[i][1], (float)tvecs[i][2]) * 0.01f);

			m_zealotCameraView = rot2 * tm * trans;

			m_markers.push_back({ ids[i], m_zealotCameraView });
		}
	}

	// display camera image to DirectX Texture
	D3DLOCKED_RECT lockRect;
	m_videoSprite->GetTexture()->Lock(lockRect);
	if (lockRect.pBits)
	{
		Mat BGRA = image.clone();
		cvtColor(image, BGRA, CV_BGR2BGRA, 4);
		const size_t sizeInBytes2 = BGRA.step[0] * BGRA.rows;
		memcpy(lockRect.pBits, BGRA.data, sizeInBytes2);
		m_videoSprite->GetTexture()->Unlock();
	}

	// keyboard
	if (GetAsyncKeyState('W'))
		m_camera[m_cameraMode]->MoveFront(2);
	else if (GetAsyncKeyState('A'))
		m_camera[m_cameraMode]->MoveRight(-2);
	else if (GetAsyncKeyState('D'))
		m_camera[m_cameraMode]->MoveRight(2);
	else if (GetAsyncKeyState('S'))
		m_camera[m_cameraMode]->MoveFront(-2);
	else if (GetAsyncKeyState('E'))
		m_camera[m_cameraMode]->MoveUp(2);
	else if (GetAsyncKeyState('C'))
		m_camera[m_cameraMode]->MoveUp(-2);

	//GetMainCamera()->Update();
	m_camera[m_cameraMode]->Update();

	m_character.Update(elapseT);
}


void cViewer::OnRender(const float elapseT)
{
	if (SUCCEEDED(m_renderer.GetDevice()->Clear(0, NULL, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER | D3DCLEAR_STENCIL, D3DCOLOR_XRGB(150, 150, 150), 1.0f, 0)))
	{
		m_renderer.GetDevice()->BeginScene();

		m_renderer.GetDevice()->SetRenderState(D3DRS_ZENABLE, 0);
		m_videoSprite->Render(m_renderer, Matrix44::Identity);
		m_renderer.GetDevice()->SetRenderState(D3DRS_ZENABLE, 1);

		m_renderer.RenderGrid();
		m_renderer.RenderAxis();

		// 박스 모델이 바닥 위에 적당하게 놓이기 위한 변환 행렬
		Matrix44 boxS;
		boxS.SetScale(Vector3(1, 1, 1)*0.05f);
		Matrix44 boxT;
		boxT.SetTranslate(Vector3(0, 5, 0));
		Matrix44 boxTm = boxT * boxS;		


		switch (m_cameraMode)
		{
		case 0:
		{
			// 마커 좌표계로 출력
			for (auto &m : m_markers)
			{
				m_camera[0]->SetViewMatrix(m.tm);
				m_camera[0]->UpdateParameterFromViewMatrix();
				m_renderer.GetDevice()->SetTransform(D3DTS_VIEW, (D3DMATRIX*)&m.tm);

				m_box.SetTransform(boxTm);
				m_box.Render(m_renderer);
			}
		}
		break;

		case 1:
		{
			GetMainCamera()->SetViewMatrix(m_camera[1]->GetViewMatrix());
			m_renderer.GetDevice()->SetTransform(D3DTS_VIEW, (D3DMATRIX*)&m_camera[1]->GetViewMatrix());

			// 마커, 카메라 출력, 제2 카메라로 이동
			for (auto &m : m_markers)
			{
				m_camera[0]->SetViewMatrix(m_camera[1]->GetViewMatrix());
				m_box.SetTransform(boxTm);
				m_box.Render(m_renderer);

				m_camera[0]->SetViewMatrix(m.tm);
				m_camera[0]->UpdateParameterFromViewMatrix();
				m_camera[0]->Render(m_renderer);
			}
		}
		break;

		case 2:
		{
			// 제3 카메라를 기준으로 마커 위치 계산
			for (auto &m : m_markers)
			{
				Matrix44 tm = m.tm * m_camera[2]->GetViewMatrix().Inverse();
				m_renderer.GetDevice()->SetTransform(D3DTS_VIEW, (D3DMATRIX*)&m_camera[2]->GetViewMatrix());
				GetMainCamera()->SetViewMatrix(m_camera[2]->GetViewMatrix());

				if (m_toggleUpdateTm)
					m_box.SetTransform(boxTm * tm);

				m_box.Render(m_renderer);
			}

		}
		break;

		case 3:
		{
			// 100번 마커를 기준으로, 마커 위치를 재계산 해서 출력한다.
			// 100번 마커가 원점이 된다.
			sMarker orig{ -1 };
			for (auto &m : m_markers)
			{
				if (m.id == 100)
				{
					orig = m;
					break;
				}
			}

			if (orig.id != 100)
				break;

			// 제3 카메라를 기준으로 마커 위치 계산
			for (auto &m : m_markers)
			{
				Matrix44 tm = m.tm * orig.tm.Inverse();
				m_renderer.GetDevice()->SetTransform(D3DTS_VIEW, (D3DMATRIX*)&m_camera[3]->GetViewMatrix());
				GetMainCamera()->SetViewMatrix(m_camera[3]->GetViewMatrix());
				m_box.SetTransform(boxTm * tm);
				m_box.Render(m_renderer);
			}
		}
		break;
		}

		//m_cube.Render(m_renderer, Matrix44::Identity);
		//m_cube2.Render(m_renderer, Matrix44::Identity);
		m_renderer.RenderFPS();

		m_renderer.GetDevice()->EndScene();
		m_renderer.GetDevice()->Present(NULL, NULL, NULL, NULL);
	}
}


void cViewer::OnShutdown()
{
}


void cViewer::OnMessageProc(UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_MOUSEWHEEL:
	{
		int fwKeys = GET_KEYSTATE_WPARAM(wParam);
		int zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
		dbg::Print("%d %d", fwKeys, zDelta);

		const float len = m_camera[m_cameraMode]->GetDistance();
		float zoomLen = (len > 100) ? 50 : (len / 4.f);
		if (fwKeys & 0x4)
			zoomLen = zoomLen / 10.f;

		m_camera[m_cameraMode]->Zoom((zDelta < 0) ? -zoomLen : zoomLen);
	}
	break;

	case WM_KEYDOWN:
		switch (wParam)
		{
		case VK_BACK:
			break;
		case VK_TAB:
		{
			static bool flag = false;
			m_renderer.GetDevice()->SetRenderState(D3DRS_CULLMODE, flag ? D3DCULL_CCW : D3DCULL_NONE);
			m_renderer.GetDevice()->SetRenderState(D3DRS_FILLMODE, flag ? D3DFILL_SOLID : D3DFILL_WIREFRAME);
			flag = !flag;
		}
		break;

		case VK_RETURN:
			m_cameraMode = (m_cameraMode == 0) ? 1 : 0;
			break;

		case VK_SPACE: m_toggleUpdateTm = !m_toggleUpdateTm; break;

		case '1': m_cameraMode = 0; break;
		case '2': m_cameraMode = 1; break;
		case '3': m_cameraMode = 2; break;
		case '4': m_cameraMode = 3; break;
		}
		break;

	case WM_LBUTTONDOWN:
	{
		m_LButtonDown = true;
		m_curPos.x = LOWORD(lParam);
		m_curPos.y = HIWORD(lParam);
	}
	break;

	case WM_LBUTTONUP:
		m_LButtonDown = false;
		break;

	case WM_RBUTTONDOWN:
	{
		SetCapture(m_hWnd);
		m_RButtonDown = true;
		m_curPos.x = LOWORD(lParam);
		m_curPos.y = HIWORD(lParam);
	}
	break;

	case WM_RBUTTONUP:
		ReleaseCapture();
		m_RButtonDown = false;
		break;

	case WM_MBUTTONDOWN:
		m_MButtonDown = true;
		m_curPos.x = LOWORD(lParam);
		m_curPos.y = HIWORD(lParam);
		break;

	case WM_MBUTTONUP:
		m_MButtonDown = false;
		break;

	case WM_MOUSEMOVE:
	{
		if (m_RButtonDown)
		{
			POINT pos = { LOWORD(lParam), HIWORD(lParam) };
			const int x = pos.x - m_curPos.x;
			const int y = pos.y - m_curPos.y;
			m_curPos = pos;

			m_camera[m_cameraMode]->Yaw2(x * 0.005f);
			m_camera[m_cameraMode]->Pitch2(y * 0.005f);
		}
		else if (m_MButtonDown)
		{
			const POINT point = { LOWORD(lParam), HIWORD(lParam) };
			const POINT pos = { point.x - m_curPos.x, point.y - m_curPos.y };
			m_curPos = point;

			const float len = m_camera[m_cameraMode]->GetDistance();
			m_camera[m_cameraMode]->MoveRight(-pos.x * len * 0.001f);
			m_camera[m_cameraMode]->MoveUp(pos.y * len * 0.001f);
		}

	}
	break;
	}
}

