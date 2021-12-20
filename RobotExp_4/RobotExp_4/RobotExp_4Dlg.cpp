
// RobotExp_4Dlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "RobotExp_4.h"
#include "RobotExp_4Dlg.h"
#include "afxdialogex.h"
#include "DataType.h"
#include <math.h>
#include "SystemMemory.h"
#include "GraphDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif
#define _CRT_SECURE_NO_WARNINGS


// 응용 프로그램 정보에 사용되는 CAboutDlg 대화 상자입니다.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

	// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

 // 구현입니다.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CRobotExp_4Dlg 대화 상자



CRobotExp_4Dlg::CRobotExp_4Dlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_ROBOTEXP_4_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRobotExp_4Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO_PORT, m_COMBO_PORT);
	DDX_Control(pDX, IDC_COMBO_BAUD, m_COMBO_BAUD);
	DDX_Control(pDX, IDC_EDIT_SEND, m_EDIT_SEND);
	DDX_Control(pDX, IDC_EDIT_RECV, m_EDIT_RECV);
	DDX_Control(pDX, IDC_EDIT_TAR_POS, m_EDIT_TAR_POS);
	DDX_Control(pDX, IDC_EDIT_TAR_POS2, m_EDIT_TAR_POS2);
	DDX_Control(pDX, IDC_EDIT_TAR_VEL, m_EDIT_TAR_VEL);
	DDX_Control(pDX, IDC_EDIT_TAR_TORQ, m_EDIT_TAR_TORQ);
	DDX_Control(pDX, IDC_EDIT_CURR_POS, m_EDIT_CURR_POS);
	DDX_Control(pDX, IDC_EDIT_CURR_POS2, m_EDIT_CURR_POS2);
	DDX_Control(pDX, IDC_EDIT_CURR_VEL, m_EDIT_CURR_VEL);
	DDX_Control(pDX, IDC_EDIT_CURR_TORQ, m_EDIT_CURR_TORQ);
	DDX_Control(pDX, IDC_EDIT_TAR_X, m_EDIT_TAR_X);
	DDX_Control(pDX, IDC_EDIT_TAR_Y, m_EDIT_TAR_Y);
	DDX_Control(pDX, IDC_EDIT_TAR_Z, m_EDIT_TAR_Z);
	DDX_Control(pDX, IDC_EDIT_CURR_X, m_EDIT_CURR_X);
	DDX_Control(pDX, IDC_EDIT_CURR_Y, m_EDIT_CURR_Y);
	DDX_Control(pDX, IDC_EDIT_CURR_Z, m_EDIT_CURR_Z);
	DDX_Control(pDX, IDC_CHECK_OPEN, m_CHECK_OPEN);
	DDX_Control(pDX, IDC_CHECK_SEND, m_CHECK_SEND);
}

BEGIN_MESSAGE_MAP(CRobotExp_4Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_CBN_DROPDOWN(IDC_COMBO_PORT, &CRobotExp_4Dlg::OnCbnDropdownComboPort)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON_CLEAR, &CRobotExp_4Dlg::OnBnClickedButtonClear)
	ON_BN_CLICKED(IDC_BUTTON_INIT, &CRobotExp_4Dlg::OnBnClickedButtonInit)
	ON_BN_CLICKED(IDC_BUTTON_SET, &CRobotExp_4Dlg::OnBnClickedButtonSet)
	ON_BN_CLICKED(IDC_BUTTON_FORWARD, &CRobotExp_4Dlg::OnBnClickedButtonForward)
	ON_BN_CLICKED(IDC_BUTTON_BACKWARD, &CRobotExp_4Dlg::OnBnClickedButtonBackward)
	ON_BN_CLICKED(IDC_BUTTON_GRAPH, &CRobotExp_4Dlg::OnBnClickedButtonGraph)
	ON_BN_CLICKED(IDC_CHECK_OPEN, &CRobotExp_4Dlg::OnBnClickedCheckOpen)
	ON_BN_CLICKED(IDC_CHECK_SEND, &CRobotExp_4Dlg::OnBnClickedCheckSend)
END_MESSAGE_MAP()


// CRobotExp_4Dlg 메시지 처리기

BOOL CRobotExp_4Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 시스템 메뉴에 "정보..." 메뉴 항목을 추가합니다.

	// IDM_ABOUTBOX는 시스템 명령 범위에 있어야 합니다.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 이 대화 상자의 아이콘을 설정합니다.  응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);         // 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);      // 작은 아이콘을 설정합니다.

	// TODO: 여기에 추가 초기화 작업을 추가합니다.
	m_EDIT_TAR_POS.SetWindowTextA("0.0");
	m_EDIT_TAR_POS2.SetWindowTextA("0.0");
	m_EDIT_TAR_VEL.SetWindowTextA("10.0");
	m_EDIT_TAR_TORQ.SetWindowTextA("0.1");
	m_EDIT_TAR_X.SetWindowTextA("0.0");
	m_EDIT_TAR_Y.SetWindowTextA("0.0");
	m_EDIT_TAR_Z.SetWindowTextA("0.0");

	m_EDIT_CURR_POS.SetWindowTextA("0.0");
	m_EDIT_CURR_POS2.SetWindowTextA("0.0");
	m_EDIT_CURR_VEL.SetWindowTextA("0.0");
	m_EDIT_CURR_TORQ.SetWindowTextA("0.0");
	m_EDIT_CURR_X.SetWindowTextA("0.0");
	m_EDIT_CURR_Y.SetWindowTextA("0.0");
	m_EDIT_CURR_Z.SetWindowTextA("0.0");

	SetTimer(1001, 33, NULL);         // Timer ID, MFC 실행 주기, Timer 실행 시 실행할 함수

	// NTGraph 초기화
	m_pGraphDlg = new CGraphDlg;
	m_pGraphDlg->Create(IDD_GRAPH_DIALOG);

	// 멀티 스레드 통신 초기화
	m_commWorker.SetPeriod(0.01);
	m_commWorker.SetWork(CreateWork<CCommWork>("Comm1Work"));

	// 통신 초기 값 설정
	ControlData_t tar_motor_data;
	tar_motor_data.position = 0.0;
	tar_motor_data.velocity = 10.0 * DEG2RAD;
	tar_motor_data.current = 0.1 / 0.0683;
	SET_SYSTEM_MEMORY("Comm1Work_Controller_Target", tar_motor_data);

	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CRobotExp_4Dlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다.  문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CRobotExp_4Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 아이콘을 그립니다.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CRobotExp_4Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CRobotExp_4Dlg::SolveForwardKinematics(double dAngle, double dAngle2, double* pdPos) {
	// Input Angle is Degree

	double link0 = 0.25;
	double link1 = 1.0;
	double link2 = 0.5;
	dAngle *= DEG2RAD;
	dAngle2 *= DEG2RAD;

	pdPos[0] = 0;
	pdPos[1] = link1 * sin(dAngle) + link2 * sin(dAngle + dAngle2);
	pdPos[2] = link0 + link1 * cos(dAngle) + link2 * cos(dAngle + dAngle2);
}


void CRobotExp_4Dlg::SolveInverseKinematics(double dX, double dY, double dZ, double* pdAngle) {
	double link0 = 0.25, link1 = 1.0, link2 = 0.5;

	if ((pow(dY, 2) + pow(dZ, 2) > pow(1.5, 2) + link0) || (pow(dY, 2) + pow(dZ, 2) < pow(0.5, 2) + link0)) {
		CString str;
		str.Format("Unreachable area");
		m_EDIT_RECV.SetWindowText(str);

		pdAngle[0] = 0.0;
		pdAngle[1] = 0.0;
		OnBnClickedButtonInit();
		return;
	}

	double c2 = (pow(dY, 2) + pow(dZ, 2) - pow(link1, 2) - pow(link2, 2)) / (2 * link1 * link2);
	double s2 = sqrt(1 - pow(c2, 2));

	double c1 = (dY * link2 * s2 + dZ * (link1 + link2 * c2)) / (pow(link1 + link2 * c2, 2) + pow(link2 * s2, 2));
	double s1 = (-dZ * link2 * s2 + dY * (link1 + link2 * c2)) / (pow(link1 + link2 * c2, 2) + pow(link2 * s2, 2));

	CString str;
	str.Format("Inverse Kinematics");
	m_EDIT_RECV.SetWindowText(str);

	pdAngle[0] = atan2(s1, c1);
	pdAngle[1] = atan2(s2, c2);
}


void CRobotExp_4Dlg::OnCbnDropdownComboPort()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CDeviceListReader reader;
	std::vector<std::string> list;

	// Combo box 초기화
	m_COMBO_PORT.ResetContent();

	// 컴퓨터에 연결된 시리얼 장비 리스트를 열어옴
	reader.UpdateDeviceList("SERIALCOMM");
	reader.GetDeviceList(list);

	// Combo box에 list 추가
	for (int i = 0; i < list.size(); i++) {
		m_COMBO_PORT.AddString(list[i].c_str());
	}
}


void CRobotExp_4Dlg::OnBnClickedCheckOpen()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	// 버튼의 체크 여부 확인
	if (m_CHECK_OPEN.GetCheck()) {
		// 체크된 경우
		// 선택된 포트이름과 통신속도를 가져옴
		CString port, baud;
		m_COMBO_PORT.GetLBText(m_COMBO_PORT.GetCurSel(), port);
		m_COMBO_BAUD.GetLBText(m_COMBO_BAUD.GetCurSel(), baud);
		int nTmp = atoi(baud.GetBuffer());

		// 포트 열기 시도
		if (((CCommWork*)m_commWorker.GetWork())->OpenPort(port.GetBuffer(), nTmp)) {
			// 성공하면 Thread를 만들고   버튼의 텍스트를 Close로 변경
			m_commWorker.StartWork();
			m_CHECK_OPEN.SetWindowText("Close");
			OnBnClickedButtonInit();
		}
		else {
			// 실패 시 에러 메세지를 띄우고 초기화
			AfxMessageBox("Can`t Open port");
			m_CHECK_OPEN.SetCheck(false);
		}
	}
	else {
		// 체크 해제된 경우
		// Thread를 제거하고 포트를 닫은 뒤 버튼 텍스트를 Open으로 변경
		m_commWorker.StopWork();
		((CCommWork*)m_commWorker.GetWork())->ClosePort();
		m_CHECK_OPEN.SetWindowText("Open");
	}
}


void CRobotExp_4Dlg::OnBnClickedCheckSend()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CRobotExp_4Dlg::OnTimer(UINT_PTR nIDEvent) {
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	DataType_t ode_data;
	ControlData_t curr_motor_data;
	GET_SYSTEM_MEMORY("JointData", ode_data);
	GET_SYSTEM_MEMORY("Comm1Work_Controller_Current", curr_motor_data);

	CString str;
	double dAngle = 0.0;
	double dAngle2 = 0.0;

	// ODE
	if (!m_CHECK_OPEN.GetCheck()) {
		str.Format("%.4f", ode_data.Q_cur[0] * RAD2DEG);
		m_EDIT_CURR_POS.SetWindowText(str);
		str.Format("%.4f", ode_data.Q_cur[1] * RAD2DEG);
		m_EDIT_CURR_POS2.SetWindowText(str);

		dAngle = ode_data.Q_cur[0] * RAD2DEG;
		dAngle2 = ode_data.Q_cur[1] * RAD2DEG;
	}
	else {
		// Motor Current
		while (curr_motor_data.position * RAD2DEG > 180) {
			curr_motor_data.position -= 360 * DEG2RAD;
		}
		while (curr_motor_data.position * RAD2DEG < -180) {
			curr_motor_data.position += 360 * DEG2RAD;
		}
		str.Format("%.4f", curr_motor_data.position * RAD2DEG);
		m_EDIT_CURR_POS.SetWindowText(str);
		/*m_EDIT_CURR_POS.GetWindowText(str);
		double curr_motor_pos = atof(str.GetBuffer());
		if (curr_motor_pos > 360) {
		   str.Format("%.4f", curr_motor_pos - 360.0);
		   m_EDIT_CURR_POS.SetWindowText(str);

		   curr_motor_data.position = curr_motor_pos - 360.0;
		}*/
		str.Format("%.4f", curr_motor_data.velocity * RAD2DEG);
		m_EDIT_CURR_VEL.SetWindowText(str);
		str.Format("%.4f", curr_motor_data.current * 0.0683);      // Torque constant
		m_EDIT_CURR_TORQ.SetWindowText(str);

		dAngle = curr_motor_data.position * RAD2DEG;
		dAngle2 = 0.0;

		ode_data.Q_tar[0] = curr_motor_data.position;
		ode_data.Q_tar[1] = 0.0;
		SET_SYSTEM_MEMORY("JointData", ode_data);
	}

	// Forward Kinematics
	double Pcur[3] = { 0, };
	SolveForwardKinematics(dAngle, dAngle2, Pcur);

	str.Format("%.4f", Pcur[0]);
	m_EDIT_CURR_X.SetWindowText(str);
	str.Format("%.4f", Pcur[1]);
	m_EDIT_CURR_Y.SetWindowText(str);
	str.Format("%.4f", Pcur[2]);
	m_EDIT_CURR_Z.SetWindowText(str);

	CDialogEx::OnTimer(nIDEvent);
}


void CRobotExp_4Dlg::OnBnClickedButtonClear()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	str.Format("");
	m_EDIT_RECV.SetWindowText(str);
}


void CRobotExp_4Dlg::OnBnClickedButtonInit()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	DataType_t joint_data;
	ControlData_t tar_motor_data;
	GET_SYSTEM_MEMORY("JointData", joint_data);
	GET_SYSTEM_MEMORY("Comm1Work_Controller_Target", tar_motor_data);

	joint_data.Q_tar[0] = 0.0;
	joint_data.Q_tar[1] = 0.0;
	tar_motor_data.position = 0.0;
	tar_motor_data.velocity = 10.0 * DEG2RAD;
	tar_motor_data.current = 0.1 / 0.0683;

	SET_SYSTEM_MEMORY("JointData", joint_data);
	SET_SYSTEM_MEMORY("Comm1Work_Controller_Target", tar_motor_data);

	m_EDIT_TAR_POS.SetWindowTextA("0.0");
	m_EDIT_TAR_POS2.SetWindowTextA("0.0");
	m_EDIT_TAR_VEL.SetWindowTextA("10.0");
	m_EDIT_TAR_TORQ.SetWindowTextA("0.1");
	m_EDIT_TAR_X.SetWindowTextA("0.0");
	m_EDIT_TAR_Y.SetWindowTextA("0.0");
	m_EDIT_TAR_Z.SetWindowTextA("0.0");

	m_EDIT_CURR_POS.SetWindowTextA("0.0");
	m_EDIT_CURR_POS2.SetWindowTextA("0.0");
	m_EDIT_CURR_VEL.SetWindowTextA("0.0");
	m_EDIT_CURR_TORQ.SetWindowTextA("0.0");
	m_EDIT_CURR_X.SetWindowTextA("0.0");
	m_EDIT_CURR_Y.SetWindowTextA("0.0");
	m_EDIT_CURR_Z.SetWindowTextA("0.0");
}


void CRobotExp_4Dlg::OnBnClickedButtonSet() {
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	DataType_t ode_data;
	GET_SYSTEM_MEMORY("JointData", ode_data);

	CString str;

	if (!m_CHECK_OPEN.GetCheck()) {
		m_EDIT_TAR_POS.GetWindowText(str);
		ode_data.Q_tar[0] = atof(str.GetBuffer()) * DEG2RAD;
		m_EDIT_TAR_POS2.GetWindowText(str);
		ode_data.Q_tar[1] = atof(str.GetBuffer()) * DEG2RAD;

		double pdPos[3] = { 0, 0, 0 };
		SolveForwardKinematics(ode_data.Q_tar[0] * RAD2DEG, ode_data.Q_tar[1] * RAD2DEG, pdPos);

		char pszTmp[512];
		sprintf_s(pszTmp, "%.2lf", pdPos[0]);
		m_EDIT_TAR_X.SetWindowTextA(pszTmp);
		sprintf_s(pszTmp, "%.2lf", pdPos[1]);
		m_EDIT_TAR_Y.SetWindowTextA(pszTmp);
		sprintf_s(pszTmp, "%.2lf", pdPos[2]);
		m_EDIT_TAR_Z.SetWindowTextA(pszTmp);
	}
	else {
		ControlData_t tar_motor_data;
		ControlData_t curr_motor_data;

		m_EDIT_TAR_POS.GetWindowText(str);
		// Position Exception handling
		double theta_ref = atof(str.GetBuffer());               // [degree]
		while (theta_ref > 180) {
			theta_ref -= 360;
		}
		while (theta_ref < -180) {
			theta_ref += 360;
		}
		char pszTmp[512];
		sprintf_s(pszTmp, "%.2lf", theta_ref);
		m_EDIT_TAR_POS.SetWindowTextA(pszTmp);

		// Set Motor Target
		tar_motor_data.position = theta_ref * DEG2RAD;
		m_EDIT_TAR_VEL.GetWindowText(str);
		tar_motor_data.velocity = atof(str.GetBuffer()) * DEG2RAD;
		m_EDIT_TAR_TORQ.GetWindowText(str);
		tar_motor_data.current = atof(str.GetBuffer()) / 0.0683;
		SET_SYSTEM_MEMORY("Comm1Work_Controller_Target", tar_motor_data);

		// Set ode data
		// to visualize motor movement at ode simulator
		GET_SYSTEM_MEMORY("Comm1Work_Controller_Current", curr_motor_data);
		ode_data.Q_tar[0] = curr_motor_data.position;
		ode_data.Q_tar[1] = 0.0;

		// Forward kinematics
		double pdPos[3] = { 0, 0, 0 };
		SolveForwardKinematics(tar_motor_data.position * RAD2DEG, 0.0, pdPos);
		sprintf_s(pszTmp, "%.2lf", pdPos[0]);
		m_EDIT_TAR_X.SetWindowTextA(pszTmp);
		sprintf_s(pszTmp, "%.2lf", pdPos[1]);
		m_EDIT_TAR_Y.SetWindowTextA(pszTmp);
		sprintf_s(pszTmp, "%.2lf", pdPos[2]);
		m_EDIT_TAR_Z.SetWindowTextA(pszTmp);
	}

	SET_SYSTEM_MEMORY("JointData", ode_data);
}


void CRobotExp_4Dlg::OnBnClickedButtonForward()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	char cTmp[10];
	double dTmp[2];

	m_EDIT_TAR_POS.GetWindowTextA(cTmp, 10);
	dTmp[0] = atof(cTmp);
	m_EDIT_TAR_POS2.GetWindowTextA(cTmp, 10);
	dTmp[1] = atof(cTmp);

	double pdPos[3] = { 0, 0, 0 };
	SolveForwardKinematics(dTmp[0], dTmp[1], pdPos);

	char pszTmp[512];
	sprintf_s(pszTmp, "%.2lf", pdPos[0]);
	m_EDIT_TAR_X.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", pdPos[1]);
	m_EDIT_TAR_Y.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", pdPos[2]);
	m_EDIT_TAR_Z.SetWindowTextA(pszTmp);

	DataType_t joint_data;
	GET_SYSTEM_MEMORY("JointData", joint_data);
	joint_data.Q_tar[0] = dTmp[0] * DEG2RAD;
	joint_data.Q_tar[1] = dTmp[1] * DEG2RAD;
	SET_SYSTEM_MEMORY("JointData", joint_data);
}


void CRobotExp_4Dlg::OnBnClickedButtonBackward()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	char cTmp[10];
	double dTmp[2];

	m_EDIT_TAR_Y.GetWindowTextA(cTmp, 10);
	dTmp[0] = atof(cTmp);
	m_EDIT_TAR_Z.GetWindowTextA(cTmp, 10);
	dTmp[1] = atof(cTmp) - 0.25;

	double pdAngle[2] = { 0, 0 };
	SolveInverseKinematics(0.0, dTmp[0], dTmp[1], pdAngle);

	char pszTmp[512];
	sprintf_s(pszTmp, "%.2lf", pdAngle[0] * RAD2DEG);
	m_EDIT_TAR_POS.SetWindowTextA(pszTmp);
	sprintf_s(pszTmp, "%.2lf", pdAngle[1] * RAD2DEG);
	m_EDIT_TAR_POS2.SetWindowTextA(pszTmp);

	DataType_t joint_data;
	GET_SYSTEM_MEMORY("JointData", joint_data);
	joint_data.Q_tar[0] = pdAngle[0];
	joint_data.Q_tar[1] = pdAngle[1];
	SET_SYSTEM_MEMORY("JointData", joint_data);
}


void CRobotExp_4Dlg::OnBnClickedButtonGraph()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	BOOL bCheck = m_pGraphDlg->IsWindowVisible();
	if (bCheck) {
		m_pGraphDlg->ShowWindow(SW_HIDE);
	}
	else {
		m_pGraphDlg->ShowWindow(SW_SHOW);
	}
}