// GraphDlg.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "RobotExp_4.h"
#include "GraphDlg.h"
#include "afxdialogex.h"
#include "SystemMemory.h"
#include "DataType.h"


// CGraphDlg 대화 상자입니다.

IMPLEMENT_DYNAMIC(CGraphDlg, CDialogEx)

CGraphDlg::CGraphDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_GRAPH_DIALOG, pParent)
{

}

CGraphDlg::~CGraphDlg()
{
}

void CGraphDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_NTGRAPH_POS, m_NTG_POS);
	DDX_Control(pDX, IDC_NTGRAPH_VEL, m_NTG_VEL);
	DDX_Control(pDX, IDC_NTGRAPH_TORQ, m_NTG_TORQ);
}


BEGIN_MESSAGE_MAP(CGraphDlg, CDialogEx)
	ON_WM_TIMER()
END_MESSAGE_MAP()


BOOL CGraphDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  여기에 추가 초기화 작업을 추가합니다.
	// Position
	m_NTG_POS.ClearGraph();
	m_NTG_POS.SetFrameStyle(0);
	m_NTG_POS.SetPlotAreaColor(WHITE);
	m_NTG_POS.SetShowGrid(TRUE);
	m_NTG_POS.SetFormatAxisBottom(_T("%.2f"));

	m_NTG_POS.SetCaption(_T("Position"));
	m_NTG_POS.SetXLabel(_T("Time[s]"));
	m_NTG_POS.SetYLabel(_T("Degree[deg]"));
	m_NTG_POS.AddElement();
	m_NTG_POS.SetElementWidth(3);
	m_NTG_POS.SetElementLineColor(RED);		// Target
	m_NTG_POS.AddElement();
	m_NTG_POS.SetElementWidth(3);
	m_NTG_POS.SetElementLineColor(BLUE);	// Current
	m_NTG_POS.SetRange(0.0, 25.0, 0.0, 360.0);
	m_NTG_POS.SetYGridNumber(4);

	// Velocity
	m_NTG_VEL.ClearGraph();
	m_NTG_VEL.SetFrameStyle(0);
	m_NTG_VEL.SetPlotAreaColor(WHITE);
	m_NTG_VEL.SetShowGrid(TRUE);
	m_NTG_VEL.SetFormatAxisBottom(_T("%.2f"));

	m_NTG_VEL.SetCaption(_T("Velocity"));
	m_NTG_VEL.SetXLabel(_T("Time[s]"));
	m_NTG_VEL.SetYLabel(_T("Velocity[deg/s]"));
	m_NTG_VEL.AddElement();
	m_NTG_VEL.SetElementWidth(4);
	m_NTG_VEL.SetElementLineColor(RED);		// Target
	m_NTG_VEL.AddElement();
	m_NTG_VEL.SetElementWidth(3);
	m_NTG_VEL.SetElementLineColor(BLUE);	// Current
	m_NTG_VEL.SetRange(0.0, 25.0, 0.0, 360.0);
	m_NTG_VEL.SetYGridNumber(4);

	// Torque
	m_NTG_TORQ.ClearGraph();
	m_NTG_TORQ.SetFrameStyle(0);
	m_NTG_TORQ.SetPlotAreaColor(WHITE);
	m_NTG_TORQ.SetShowGrid(TRUE);
	m_NTG_TORQ.SetFormatAxisBottom(_T("%.2f"));
	
	m_NTG_TORQ.SetCaption(_T("Torque"));
	m_NTG_TORQ.SetXLabel(_T("Time[s]"));
	m_NTG_TORQ.SetYLabel(_T("Torque[Nm]"));
	m_NTG_TORQ.AddElement();
	m_NTG_TORQ.SetElementWidth(4);
	m_NTG_TORQ.SetElementLineColor(RED);	// Target
	m_NTG_TORQ.AddElement();
	m_NTG_TORQ.SetElementWidth(3);
	m_NTG_TORQ.SetElementLineColor(BLUE);	// Current
	m_NTG_TORQ.SetRange(0.0, 25.0, 0.0, 360.0);
	m_NTG_TORQ.SetYGridNumber(4);

	// Timer
	SetTimer(1, 100, NULL);

	return TRUE;  // return TRUE unless you set the focus to a control
				  // 예외: OCX 속성 페이지는 FALSE를 반환해야 합니다.
}


// CGraphDlg 메시지 처리기입니다.
void CGraphDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	m_dCnt += 0.1;
	DataType_t jointData;
	GET_SYSTEM_MEMORY("JointData", jointData);

	if (m_dCnt >= 25.0) {
		m_NTG_POS.SetRange(m_dCnt - 25.0, m_dCnt, 0.0, 360.0);
	}
	else {
		m_NTG_POS.SetRange(0, m_dCnt, 0.0, 360.0);
	}

	m_NTG_POS.PlotXY(m_dCnt, jointData.Q_tar[0] * RAD2DEG, 1);
	//m_NTG_POS.PlotXY(m_dCnt, jointData.Q_cur[0] * RAD2DEG, 2);
	m_NTG_POS.PlotXY(m_dCnt, jointData.Q_tar[1] * RAD2DEG, 2);

	CDialogEx::OnTimer(nIDEvent);
}
