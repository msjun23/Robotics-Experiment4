
// RobotExp_4Dlg.cpp : ���� ����
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


// ���� ���α׷� ������ ���Ǵ� CAboutDlg ��ȭ �����Դϴ�.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

	// ��ȭ ���� �������Դϴ�.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

 // �����Դϴ�.
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


// CRobotExp_4Dlg ��ȭ ����



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


// CRobotExp_4Dlg �޽��� ó����

BOOL CRobotExp_4Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// �ý��� �޴��� "����..." �޴� �׸��� �߰��մϴ�.

	// IDM_ABOUTBOX�� �ý��� ��� ������ �־�� �մϴ�.
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

	// �� ��ȭ ������ �������� �����մϴ�.  ���� ���α׷��� �� â�� ��ȭ ���ڰ� �ƴ� ��쿡��
	//  �����ӿ�ũ�� �� �۾��� �ڵ����� �����մϴ�.
	SetIcon(m_hIcon, TRUE);         // ū �������� �����մϴ�.
	SetIcon(m_hIcon, FALSE);      // ���� �������� �����մϴ�.

	// TODO: ���⿡ �߰� �ʱ�ȭ �۾��� �߰��մϴ�.
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

	SetTimer(1001, 33, NULL);         // Timer ID, MFC ���� �ֱ�, Timer ���� �� ������ �Լ�

	// NTGraph �ʱ�ȭ
	m_pGraphDlg = new CGraphDlg;
	m_pGraphDlg->Create(IDD_GRAPH_DIALOG);

	// ��Ƽ ������ ��� �ʱ�ȭ
	m_commWorker.SetPeriod(0.01);
	m_commWorker.SetWork(CreateWork<CCommWork>("Comm1Work"));

	// ��� �ʱ� �� ����
	ControlData_t tar_motor_data;
	tar_motor_data.position = 0.0;
	tar_motor_data.velocity = 10.0 * DEG2RAD;
	tar_motor_data.current = 0.1 / 0.0683;
	SET_SYSTEM_MEMORY("Comm1Work_Controller_Target", tar_motor_data);

	return TRUE;  // ��Ŀ���� ��Ʈ�ѿ� �������� ������ TRUE�� ��ȯ�մϴ�.
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

// ��ȭ ���ڿ� �ּ�ȭ ���߸� �߰��� ��� �������� �׸�����
//  �Ʒ� �ڵ尡 �ʿ��մϴ�.  ����/�� ���� ����ϴ� MFC ���� ���α׷��� ��쿡��
//  �����ӿ�ũ���� �� �۾��� �ڵ����� �����մϴ�.

void CRobotExp_4Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // �׸��⸦ ���� ����̽� ���ؽ�Ʈ�Դϴ�.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Ŭ���̾�Ʈ �簢������ �������� ����� ����ϴ�.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// �������� �׸��ϴ�.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// ����ڰ� �ּ�ȭ�� â�� ���� ���ȿ� Ŀ���� ǥ�õǵ��� �ý��ۿ���
//  �� �Լ��� ȣ���մϴ�.
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	CDeviceListReader reader;
	std::vector<std::string> list;

	// Combo box �ʱ�ȭ
	m_COMBO_PORT.ResetContent();

	// ��ǻ�Ϳ� ����� �ø��� ��� ����Ʈ�� �����
	reader.UpdateDeviceList("SERIALCOMM");
	reader.GetDeviceList(list);

	// Combo box�� list �߰�
	for (int i = 0; i < list.size(); i++) {
		m_COMBO_PORT.AddString(list[i].c_str());
	}
}


void CRobotExp_4Dlg::OnBnClickedCheckOpen()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	// ��ư�� üũ ���� Ȯ��
	if (m_CHECK_OPEN.GetCheck()) {
		// üũ�� ���
		// ���õ� ��Ʈ�̸��� ��żӵ��� ������
		CString port, baud;
		m_COMBO_PORT.GetLBText(m_COMBO_PORT.GetCurSel(), port);
		m_COMBO_BAUD.GetLBText(m_COMBO_BAUD.GetCurSel(), baud);
		int nTmp = atoi(baud.GetBuffer());

		// ��Ʈ ���� �õ�
		if (((CCommWork*)m_commWorker.GetWork())->OpenPort(port.GetBuffer(), nTmp)) {
			// �����ϸ� Thread�� �����   ��ư�� �ؽ�Ʈ�� Close�� ����
			m_commWorker.StartWork();
			m_CHECK_OPEN.SetWindowText("Close");
			OnBnClickedButtonInit();
		}
		else {
			// ���� �� ���� �޼����� ���� �ʱ�ȭ
			AfxMessageBox("Can`t Open port");
			m_CHECK_OPEN.SetCheck(false);
		}
	}
	else {
		// üũ ������ ���
		// Thread�� �����ϰ� ��Ʈ�� ���� �� ��ư �ؽ�Ʈ�� Open���� ����
		m_commWorker.StopWork();
		((CCommWork*)m_commWorker.GetWork())->ClosePort();
		m_CHECK_OPEN.SetWindowText("Open");
	}
}


void CRobotExp_4Dlg::OnBnClickedCheckSend()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
}


void CRobotExp_4Dlg::OnTimer(UINT_PTR nIDEvent) {
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	CString str;
	str.Format("");
	m_EDIT_RECV.SetWindowText(str);
}


void CRobotExp_4Dlg::OnBnClickedButtonInit()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	BOOL bCheck = m_pGraphDlg->IsWindowVisible();
	if (bCheck) {
		m_pGraphDlg->ShowWindow(SW_HIDE);
	}
	else {
		m_pGraphDlg->ShowWindow(SW_SHOW);
	}
}