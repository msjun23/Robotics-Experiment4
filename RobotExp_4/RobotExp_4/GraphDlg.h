#pragma once
#include "NTGraph.h"
#include "CNTGRAPH_POS.h"
#include "CNTGRAPH_VEL.h"
#include "CNTGRAPH_TORQ.h"


#define RED        RGB(127,  0,  0)
#define GREEN      RGB(  0,127,  0)
#define BLUE       RGB(  0,  0,127)
#define LIGHTRED   RGB(255,  0,  0)
#define LIGHTGREEN RGB(  0,255,  0)
#define LIGHTBLUE  RGB(  0,  0,255)
#define BLACK      RGB(  0,  0,  0)
#define WHITE      RGB(255,255,255)
#define GRAY       RGB(192,192,192)


// CGraphDlg ��ȭ �����Դϴ�.

class CGraphDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CGraphDlg)

public:
	CGraphDlg(CWnd* pParent = NULL);   // ǥ�� �������Դϴ�.
	virtual ~CGraphDlg();

// ��ȭ ���� �������Դϴ�.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_GRAPH_DIALOG };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

	DECLARE_MESSAGE_MAP()
public:

	CNTGraph m_NTG_POS;
	CNTGraph m_NTG_VEL;
	CNTGraph m_NTG_TORQ;
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	virtual BOOL OnInitDialog();
	double m_dCnt = 0;
};
