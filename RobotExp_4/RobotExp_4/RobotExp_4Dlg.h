
// RobotExp_4Dlg.h : ��� ����
//

#pragma once
#include "Comm.h"
#include "CommWork.h"
#include "DeviceListReader.h"

#include "GraphDlg.h"
#include "ThreadWorker.h"
#include "SharedMemory.h"
#include "SystemMemory.h"
#include "DataType.h"


// CRobotExp_4Dlg ��ȭ ����
class CRobotExp_4Dlg : public CDialogEx
{
// �����Դϴ�.
public:
	CRobotExp_4Dlg(CWnd* pParent = NULL);	// ǥ�� �������Դϴ�.

// ��ȭ ���� �������Դϴ�.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ROBOTEXP_4_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV �����Դϴ�.


public:
	void SolveForwardKinematics(double dAngle, double dAngle2, double* pdPos);
	void SolveInverseKinematics(double dX, double dY, double dZ, double* pdAngle);

// �����Դϴ�.
protected:
	HICON m_hIcon;
	CComm m_comm;
	CGraphDlg* m_pGraphDlg;
	CThreadedWorker m_commWorker;

	// ������ �޽��� �� �Լ�
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	CComboBox m_COMBO_PORT;
	CComboBox m_COMBO_BAUD;
	CButton m_CHECK_OPEN;
	CButton m_CHECK_SEND;
	afx_msg void OnCbnDropdownComboPort();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	CEdit m_EDIT_SEND;
	CEdit m_EDIT_RECV;
	afx_msg void OnBnClickedButtonClear();
	CEdit m_EDIT_TAR_POS;
	CEdit m_EDIT_TAR_POS2;
	CEdit m_EDIT_TAR_VEL;
	CEdit m_EDIT_TAR_TORQ;
	CEdit m_EDIT_CURR_POS;
	CEdit m_EDIT_CURR_POS2;
	CEdit m_EDIT_CURR_VEL;
	CEdit m_EDIT_CURR_TORQ;
	CEdit m_EDIT_TAR_X;
	CEdit m_EDIT_TAR_Y;
	CEdit m_EDIT_TAR_Z;
	CEdit m_EDIT_CURR_X;
	CEdit m_EDIT_CURR_Y;
	CEdit m_EDIT_CURR_Z;
	afx_msg void OnBnClickedButtonInit();
	afx_msg void OnBnClickedButtonSet();
	afx_msg void OnBnClickedButtonForward();
	afx_msg void OnBnClickedButtonBackward();
	afx_msg void OnBnClickedButtonGraph();
	afx_msg void OnBnClickedCheckOpen();
	afx_msg void OnBnClickedCheckSend();
};
