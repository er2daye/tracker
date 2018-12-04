/*******************************************************
Copyright 2008-2011 Digital Technology Co., Ltd. 
文  件：	RealPlayDlg.cpp
开发单位：	杭州
编  写：	shizhiping
日  期：	2009.5
描  述：	实时预览对话框
修  改：	
********************************************************/

#include "stdafx.h"
#include "RealPlay.h"
#include "RealPlayDlg.h"
#include "DlgPTZCruise.h"
#include "Triangulation.h"
#include <algorithm>
#include <Eigen/Dense>


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	//{{AFX_DATA(CAboutDlg)
	enum { IDD = IDD_ABOUTBOX };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAboutDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	//{{AFX_MSG(CAboutDlg)
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
	//{{AFX_DATA_INIT(CAboutDlg)
	//}}AFX_DATA_INIT
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAboutDlg)
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	//{{AFX_MSG_MAP(CAboutDlg)
		// No message handlers
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CRealPlayDlg dialog

CRealPlayDlg::CRealPlayDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CRealPlayDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CRealPlayDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_nDevPort = 8000;
	m_csUser = _T("admin");
	m_csPWD = _T("123qweasd");
	m_bIsLogin = FALSE;
//	m_lLoginID = -1;;
	m_bIsPlaying = FALSE;
	m_bIsRecording = FALSE;
	m_iCurChanIndex = -1;
	m_lPlayHandle = -1;
	m_bIsOnCruise = FALSE;
    m_bTrackRun = FALSE;
	m_bAutoOn = FALSE;
	m_bLightOn = FALSE;
    m_bWiperOn= FALSE;
	m_bFanOn= FALSE;
	m_bHeaterOn= FALSE;
	m_bAuxOn1= FALSE;
	m_bAuxOn2= FALSE;
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	haveMove = 0;
	picNum = 0;
//	m_btnPtzUp.m_realPlay = this;
//	m_btnPtzDown.m_realPlay = this;
//	m_btnPtzLeft.m_realPlay = this;
//	m_btnPtzRight.m_realPlay = this;
}

void CRealPlayDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CRealPlayDlg)
	DDX_Control(pDX, IDC_COMBO_JPG_QUALITY, m_coJpgQuality);
	DDX_Control(pDX, IDC_COMBO_JPG_SIZE, m_coJpgSize);
	DDX_Control(pDX, IDC_COMBO_PIC_TYPE, m_coPicType);
	DDX_Control(pDX, IDC_COMBO_SEQ, m_comboSeq);
	DDX_Control(pDX, IDC_COMBO_PRESET, m_comboPreset);
	DDX_Control(pDX, IDC_COMBO_PTZ_SPEED, m_comboPTZSpeed);
	DDX_Control(pDX, IDC_TREE_CHAN, m_ctrlTreeChan);
	DDX_Control(pDX, IDC_IPADDRESS_DEV, m_ctrlDevIp);
	DDX_Text(pDX, IDC_EDIT_PORT, m_nDevPort);
	DDX_Text(pDX, IDC_EDIT_USER, m_csUser);
	DDX_Text(pDX, IDC_EDIT_PWD, m_csPWD);
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CRealPlayDlg, CDialog)
	//{{AFX_MSG_MAP(CRealPlayDlg)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_LOGIN, OnButtonLogin)
	ON_NOTIFY(NM_DBLCLK, IDC_TREE_CHAN, OnDblclkTreeChan)
	ON_NOTIFY(TVN_SELCHANGED, IDC_TREE_CHAN, OnSelchangedTreeChan)
	ON_BN_CLICKED(IDC_BUTTON_RECORD, OnButtonRecord)
	ON_BN_CLICKED(IDC_BUTTON_CAPTURE, OnButtonCapture)
	ON_BN_CLICKED(IDC_BUTTON_PLAY, OnButtonPlay)
	ON_CBN_SELCHANGE(IDC_COMBO_PRESET, OnSelchangeComboPreset)
	ON_BN_CLICKED(IDC_BUTTON_PRESET_GOTO, OnButtonPresetGoto)
	ON_BN_CLICKED(IDC_BUTTON_PRESET_SET, OnButtonPresetSet)
	ON_BN_CLICKED(IDC_BUTTON_PRESET_DEL, OnButtonPresetDel)
	ON_BN_CLICKED(IDC_BUTTON_SEQ_GOTO, OnButtonSeqGoto)
	ON_BN_CLICKED(IDC_BUTTON_SEQ_SET, OnButtonSeqSet)
	ON_BN_CLICKED(IDC_BUTTON_TRACK_RUN, OnButtonTrackRun)
	ON_BN_CLICKED(IDC_BUTTON_TRACK_START, OnButtonTrackStart)
	ON_BN_CLICKED(IDC_BUTTON_TRACK_STOP, OnButtonTrackStop)
	ON_BN_CLICKED(IDC_BTN_PTZ_AUTO, OnBtnPtzAuto)
	ON_BN_CLICKED(IDC_BTN_AUX_PWRON, OnBtnAuxPwron)
	ON_BN_CLICKED(IDC_BTN_FAN_PWRON, OnBtnFanPwron)
	ON_BN_CLICKED(IDC_BTN_HEATER_PWRON, OnBtnHeaterPwron)
	ON_BN_CLICKED(IDC_BTN_LIGHT_PWRON, OnBtnLightPwron)
	ON_BN_CLICKED(IDC_BTN_WIPER_PWRON, OnBtnWiperPwron)
	ON_BN_CLICKED(IDC_BTN_AUX_PWRON2, OnBtnAuxPwron2)
	ON_CBN_SELCHANGE(IDC_COMBO_PIC_TYPE, OnSelchangeComboPicType)
	ON_WM_CLOSE()
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BTN_CAMERA_OPEN, &CRealPlayDlg::OnBnClickedBtnCameraOpen)
	ON_MESSAGE(ON_COM_RECEIVE, &CRealPlayDlg::OnComReceive)
	ON_WM_TIMER()
	
	ON_BN_CLICKED(IDC_BUTTON1, &CRealPlayDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BTN_START_TRACKING, &CRealPlayDlg::OnBnClickedBtnStartTracking)
	ON_WM_LBUTTONDOWN()
	ON_WM_MOUSEMOVE()
	ON_WM_LBUTTONUP()
	ON_BN_CLICKED(IDC_BTN_UARM_LEFT, &CRealPlayDlg::OnBnClickedBtnUarmLeft)
	ON_BN_CLICKED(IDC_BTN_UARM_RIGHT, &CRealPlayDlg::OnBnClickedBtnUarmRight)
	ON_BN_CLICKED(IDC_BTN_UARM_STOP, &CRealPlayDlg::OnBnClickedBtnUarmStop)
	ON_BN_CLICKED(IDC_BTN_UARM_UP, &CRealPlayDlg::OnBnClickedBtnUarmUp)
	ON_BN_CLICKED(IDC_BTN_UARM_DOWN, &CRealPlayDlg::OnBnClickedBtnUarmDown)
	ON_BN_CLICKED(IDC_BTN_UARM_RESET, &CRealPlayDlg::OnBnClickedBtnUarmReset)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CRealPlayDlg message handlers

BOOL CRealPlayDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon
	
	// TODO: Add extra initialization here
	//默认IP
	m_ctrlDevIp.SetAddress(192,168,1,64);

	//云台速度
	m_comboPTZSpeed.SetCurSel(5);

    //云台按钮
	m_btnPtzUp.SubclassDlgItem(IDC_BTN_PTZ_UP,this);
	m_btnPtzDown.SubclassDlgItem(IDC_BTN_PTZ_DOWN,this);
	m_btnPtzLeft.SubclassDlgItem(IDC_BTN_PTZ_LEFT,this);
	m_btnPtzRight.SubclassDlgItem(IDC_BTN_PTZ_RIGHT,this);	
	m_btnZoomOut.SubclassDlgItem(IDC_BTN_ZOOM_OUT,this);
	m_btnZoomIn.SubclassDlgItem(IDC_BTN_ZOOM_IN,this);
	m_btnFocusNear.SubclassDlgItem(IDC_BTN_FOCUS_NEAR,this);
	m_btnFocusFar.SubclassDlgItem(IDC_BTN_FOCUS_FAR,this);
	m_btnIrisOpen.SubclassDlgItem(IDC_BTN_IRIS_OPEN,this);
	m_btnIrisClose.SubclassDlgItem(IDC_BTN_IRIS_CLOSE,this);
    m_btnPtzUpleft.SubclassDlgItem(IDC_BTN_PTZ_UPLEFT,this);
    m_btnPtzUpright.SubclassDlgItem(IDC_BTN_PTZ_UPRIGHT,this);
	m_btnPtzDownleft.SubclassDlgItem(IDC_BTN_PTZ_DOWNLEFT,this);
    m_btnPtzDownright.SubclassDlgItem(IDC_BTN_PTZ_DOWNRIGHT,this);

	//抓图combo
	m_coPicType.SetCurSel(0);
	m_coJpgSize.SetCurSel(0);
	m_coJpgQuality.SetCurSel(0);
	GetDlgItem(IDC_STATIC_JPGPARA)->EnableWindow(FALSE);
	m_coJpgSize.EnableWindow(FALSE);
	m_coJpgQuality.EnableWindow(FALSE);

	//camera init
	CRect rect1, rect2;
	GetDlgItem(IDC_CAMERA1_SHOW)->GetClientRect(rect1);
	GetDlgItem(IDC_CAMERA2_SHOW)->GetClientRect(rect2);
	m_arm = new CnComm;
	m_camera1 = new Camera(3, IDC_CAMERA1_SHOW, rect1.Height(), rect1.Width(), m_arm);
	m_camera2 = new Camera(2, IDC_CAMERA2_SHOW, rect2.Height(), rect2.Width(), m_arm);
	p_recive = new thread(&Camera::Recive, m_camera2);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CRealPlayDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CRealPlayDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CRealPlayDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

/*************************************************
函数名:    	OnButtonLogin
函数描述:	注册/注销 按钮
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnButtonLogin() 
{
	
	if(!m_bIsLogin)    //login
	{
		if(!DoLogin())
			return;
		DoGetDeviceResoureCfg();  //获取设备资源信息	
		CreateDeviceTree();        //创建通道树
		GetDecoderCfg();                           //获取云台解码器信息
		InitDecoderReferCtrl();         //初始化解码器相关控件      
		GetDlgItem(IDC_BUTTON_LOGIN)->SetWindowText("Logout");
		m_bIsLogin = TRUE;
		
	}
	else      //logout
	{
		
		
		if(m_bIsPlaying|| m_bIsRecording)
		{
			MessageBox("Stop Play or record first!");
			return;
		}
        NET_DVR_Logout_V30(m_struDeviceInfo.lLoginID); 
		GetDlgItem(IDC_BUTTON_LOGIN)->SetWindowText("Login");
		m_ctrlTreeChan.DeleteAllItems();
		m_struDeviceInfo.lLoginID = -1;
		m_bIsLogin = FALSE;
	}
}

/*************************************************
函数名:    	DoLogin
函数描述:	向设备注册
输入参数:   
输出参数:   			
返回值:		
**************************************************/
BOOL CRealPlayDlg::DoLogin()
{
	UpdateData(TRUE);
	CString DeviceIp;
	BYTE nField0,nField1,nField2,nField3; 
	m_ctrlDevIp.GetAddress(nField0,nField1,nField2,nField3);
	DeviceIp.Format("%d.%d.%d.%d",nField0,nField1,nField2,nField3);
	
	NET_DVR_DEVICEINFO_V30 DeviceInfoTmp;
	memset(&DeviceInfoTmp,0,sizeof(NET_DVR_DEVICEINFO_V30));
	
	LONG lLoginID = NET_DVR_Login_V30(DeviceIp.GetBuffer(DeviceIp.GetLength()),m_nDevPort, \
		m_csUser.GetBuffer(m_csUser.GetLength()),m_csPWD.GetBuffer(m_csPWD.GetLength()),&DeviceInfoTmp);
	if(lLoginID == -1)
	{
		MessageBox("Login to Device failed!\n");
		return FALSE;
	}
    m_struDeviceInfo.lLoginID = lLoginID;
	m_struDeviceInfo.iDeviceChanNum = DeviceInfoTmp.byChanNum;
    m_struDeviceInfo.iIPChanNum = DeviceInfoTmp.byIPChanNum;
    m_struDeviceInfo.iStartChan  = DeviceInfoTmp.byStartChan;
    m_struDeviceInfo.iIPStartChan  = DeviceInfoTmp.byStartDChan;
	return TRUE;
}

/*************************************************
函数名:    	DoGetDeviceResoureCfg
函数描述:	获取设备的通道资源
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::DoGetDeviceResoureCfg()
{
	NET_DVR_IPPARACFG_V40 IpAccessCfg;
	memset(&IpAccessCfg,0,sizeof(IpAccessCfg));	
	DWORD  dwReturned;

	m_struDeviceInfo.bIPRet = NET_DVR_GetDVRConfig(m_struDeviceInfo.lLoginID,NET_DVR_GET_IPPARACFG_V40,0,&IpAccessCfg,sizeof(NET_DVR_IPPARACFG_V40),&dwReturned);

	int i;
    if(!m_struDeviceInfo.bIPRet)   //不支持ip接入,9000以下设备不支持禁用模拟通道
	{
		for(i=0; i<MAX_ANALOG_CHANNUM; i++)
		{
			if (i < m_struDeviceInfo.iDeviceChanNum)
			{
				sprintf(m_struDeviceInfo.struChanInfo[i].chChanName,"camera%d",i+m_struDeviceInfo.iStartChan);
				m_struDeviceInfo.struChanInfo[i].iChanIndex=i+m_struDeviceInfo.iStartChan;  //通道号
				m_struDeviceInfo.struChanInfo[i].bEnable = TRUE;
				
			}
			else
			{
				m_struDeviceInfo.struChanInfo[i].iChanIndex = -1;
				m_struDeviceInfo.struChanInfo[i].bEnable = FALSE;
				sprintf(m_struDeviceInfo.struChanInfo[i].chChanName, "");	
			}
		}
	}
	else        //支持IP接入，9000设备
	{
		for(i=0; i<MAX_ANALOG_CHANNUM; i++)  //模拟通道
		{
			if (i < m_struDeviceInfo.iDeviceChanNum)
			{
				sprintf(m_struDeviceInfo.struChanInfo[i].chChanName,"camera%d",i+m_struDeviceInfo.iStartChan);
				m_struDeviceInfo.struChanInfo[i].iChanIndex=i+m_struDeviceInfo.iStartChan;
				if(IpAccessCfg.byAnalogChanEnable[i])
				{
					m_struDeviceInfo.struChanInfo[i].bEnable = TRUE;
				}
				else
				{
					m_struDeviceInfo.struChanInfo[i].bEnable = FALSE;
				}
				
			}
			else//clear the state of other channel
			{
				m_struDeviceInfo.struChanInfo[i].iChanIndex = -1;
				m_struDeviceInfo.struChanInfo[i].bEnable = FALSE;
				sprintf(m_struDeviceInfo.struChanInfo[i].chChanName, "");	
			}		
		}

		//数字通道
		for(i=0; i<MAX_IP_CHANNEL; i++)
		{
			if(IpAccessCfg.struStreamMode[i].uGetStream.struChanInfo.byEnable)  //ip通道在线
			{
				m_struDeviceInfo.struChanInfo[i+MAX_ANALOG_CHANNUM].bEnable = TRUE;
                m_struDeviceInfo.struChanInfo[i+MAX_ANALOG_CHANNUM].iChanIndex = i+IpAccessCfg.dwStartDChan;
				sprintf(m_struDeviceInfo.struChanInfo[i+MAX_ANALOG_CHANNUM].chChanName,"IP Camera %d",i+1);

			}
			else
			{
               m_struDeviceInfo.struChanInfo[i+MAX_ANALOG_CHANNUM].bEnable = FALSE;
			    m_struDeviceInfo.struChanInfo[i+MAX_ANALOG_CHANNUM].iChanIndex = -1;
			}
		}
		
		
	}

}

/*************************************************
函数名:    	CreateDeviceTree
函数描述:	创建通道树
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::CreateDeviceTree()
{
	m_hDevItem = m_ctrlTreeChan.InsertItem("Dev");
	m_ctrlTreeChan.SetItemData(m_hDevItem,DEVICETYPE*1000);
    for(int i=0; i<MAX_CHANNUM_V30; i++)
	{
		if(m_struDeviceInfo.struChanInfo[i].bEnable)  //通道有效，插入通道树
		{
             HTREEITEM ChanItem = m_ctrlTreeChan.InsertItem(m_struDeviceInfo.struChanInfo[i].chChanName,m_hDevItem);
			 m_ctrlTreeChan.SetItemData(ChanItem,CHANNELTYPE*1000+i);   //Data对应通道在数组中的索引
		}
	}
	m_ctrlTreeChan.Expand(m_hDevItem,TVE_EXPAND);
}

/*************************************************
函数名:    	OnDblclkTreeChan
函数描述:	双击通道数，播放选中通道
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnDblclkTreeChan(NMHDR* pNMHDR, LRESULT* pResult) 
{
	HTREEITEM hSelected = m_ctrlTreeChan.GetSelectedItem();
	//未选中
	if(NULL == hSelected)
		return;
	DWORD itemData = m_ctrlTreeChan.GetItemData(hSelected);
	HTREEITEM hParent = NULL;
	int itype = itemData/1000;    //
	int iIndex = itemData%1000;

	switch(itype)
	{
	case DEVICETYPE:
		m_iCurChanIndex = -1;
		break;
	case CHANNELTYPE:
		m_iCurChanIndex = iIndex;
		TRACE("select chan: %d\n",iIndex);
        DbPlayChannel(iIndex);
		OnSelchangeComboPreset();
		break;
	default:
		break;

	}
	*pResult = 0;
}

/*************************************************
函数名:    	DbPlayChannel
函数描述:	双击播放
输入参数:   ChanIndex-通道号
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::DbPlayChannel(int ChanIndex)
{
 
	if(!m_bIsPlaying)  //Play
	{
		StartPlay(ChanIndex);
	}
	else                //Stop,play
	{
        StopPlay();
		StartPlay(ChanIndex);

	}
}

/*************************************************
函数名:    	StartPlay
函数描述:	开始一路播放
输入参数:   ChanIndex-通道号
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::StartPlay(int iChanIndex)
{
	NET_DVR_CLIENTINFO ClientInfo;
	ClientInfo.hPlayWnd     = GetDlgItem(IDC_STATIC_PLAY)->m_hWnd;
	ClientInfo.lChannel     = m_iCurChanIndex+1;
	ClientInfo.lLinkMode    = 0;
    ClientInfo.sMultiCastIP = NULL;
	TRACE("Channel number:%d\n",ClientInfo.lChannel);
	m_lPlayHandle = NET_DVR_RealPlay_V30(m_struDeviceInfo.lLoginID,&ClientInfo,NULL,NULL,TRUE);
	
	/*
	NET_DVR_PREVIEWINFO PreviewInfo;
	PreviewInfo.hPlayWnd = GetDlgItem(IDC_STATIC_PLAY)->m_hWnd;
	PreviewInfo.lChannel = m_iCurChanIndex+1;
	PreviewInfo.dwLinkMode = 0;
	PreviewInfo.dwStreamType = 0;
	PreviewInfo.bBlocked = true;

	m_lPlayHandle = NET_DVR_RealPlay_V40(m_struDeviceInfo.lLoginID, &PreviewInfo, NULL, NULL);
	*/
	if(-1 == m_lPlayHandle)
	{
		DWORD err=NET_DVR_GetLastError();
		CString m_csErr;
        m_csErr.Format("播放出错，错误代码%d",err);
		MessageBox(m_csErr);
	}
				NET_DVR_SetCapturePictureMode(0);
	m_bIsPlaying = TRUE;
	GetDlgItem(IDC_BUTTON_PLAY)->SetWindowText("停止播放");

}

/*************************************************
函数名:    	StopPlay
函数描述:	停止播放
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::StopPlay()
{
	if(m_lPlayHandle != -1)
	{
		if(m_bIsRecording)  //正在录像，先停止
		{
			StopRecord();
		}
		NET_DVR_StopRealPlay(m_lPlayHandle);
		m_lPlayHandle=-1;
		m_bIsPlaying = FALSE;
		GetDlgItem(IDC_STATIC_PLAY)->Invalidate();
	}
}

/*************************************************
函数名:    	OnSelchangedTreeChan
函数描述:	获取选中的通道号
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnSelchangedTreeChan(NMHDR* pNMHDR, LRESULT* pResult) 
{
	NM_TREEVIEW* pNMTreeView = (NM_TREEVIEW*)pNMHDR;
	// TODO: Add your control notification handler code here
	HTREEITEM hSelected = m_ctrlTreeChan.GetSelectedItem();
	//未选中
	if(NULL == hSelected)
		return;
	DWORD itemData = m_ctrlTreeChan.GetItemData(hSelected);
	HTREEITEM hParent = NULL;
	int itype = itemData/1000;    //
	int iIndex = itemData%1000;
	
	switch(itype)
	{
	case DEVICETYPE:
        m_iCurChanIndex = -1;
		break;
	case CHANNELTYPE:
		m_iCurChanIndex = iIndex;
		OnSelchangeComboPreset();
		TRACE("select chan index: %d\n",iIndex);
		break;
	default:
		break;
		
	}
	*pResult = 0;
}

/*************************************************
函数名:    	GetCurChanIndex
函数描述:	获取当前选中的通道号
输入参数:   
输出参数:   			
返回值:		当前选中的通道号
**************************************************/
int CRealPlayDlg::GetCurChanIndex()
{
    return m_iCurChanIndex;
}

/*************************************************
函数名:    	GetPlayHandle
函数描述:	获取播放句柄
输入参数:   
输出参数:   			
返回值:		播放句柄
**************************************************/
LONG CRealPlayDlg::GetPlayHandle()
{
    return m_lPlayHandle;
}

/*************************************************
函数名:    	GetPTZSpeed
函数描述:	获取云台速度
输入参数:   
输出参数:   			
返回值:		云台速度
**************************************************/
int CRealPlayDlg::GetPTZSpeed()
{
   return (m_comboPTZSpeed.GetCurSel());
}

/*************************************************
函数名:    	OnButtonRecord
函数描述:	开始/停止录像 按钮
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnButtonRecord() 
{
	if(m_lPlayHandle == -1)
	{
        MessageBox("请先选择一个通道播放");
		return;
	}
	if(!m_bIsRecording)
	{
		StartRecord();
	}
	else
	{
        StopRecord();
	}
	
}

/*************************************************
函数名:    	StartRecord
函数描述:	开始录像
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::StartRecord()
{
	char RecName[256] = {0};
	
	CTime CurTime = CTime::GetCurrentTime();;
	sprintf(RecName,"%04d%02d%02d%02d%02d%02d_ch%02d.mp4",CurTime.GetYear(),CurTime.GetMonth(),CurTime.GetDay(), \
		CurTime.GetHour(),CurTime.GetMinute(),CurTime.GetSecond(),m_struDeviceInfo.struChanInfo[GetCurChanIndex()].iChanIndex);

	 if(!NET_DVR_SaveRealData(m_lPlayHandle,RecName))
	 {
		 MessageBox("启动录像失败");
		 return;
	 }
     m_bIsRecording = TRUE;
	 GetDlgItem(IDC_BUTTON_RECORD)->SetWindowText("停止录像");
}

/*************************************************
函数名:    	StopRecord
函数描述:	停止录像
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::StopRecord()
{
    if(!NET_DVR_StopSaveRealData(m_lPlayHandle))
	{
		MessageBox("停止录像失败");
		return;
	}
	m_bIsRecording = FALSE;
	 GetDlgItem(IDC_BUTTON_RECORD)->SetWindowText("录像");
}

/*************************************************
函数名:    	OnButtonCapture
函数描述:	抓图
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnButtonCapture() 
{
	if(m_lPlayHandle == -1)
	{
        MessageBox("请先选择一个通道播放");
		return;
	}
	UpdateData(TRUE);

	char PicName[256];
	
	int iPicType = m_coPicType.GetCurSel();
	if(1 == iPicType)  //bmp
	{
		//CTime CurTime = CTime::GetCurrentTime();;
		//sprintf(PicName,"%04d%02d%02d%02d%02d%02d_ch%02d.bmp",CurTime.GetYear(),CurTime.GetMonth(),CurTime.GetDay(), \
			//CurTime.GetHour(),CurTime.GetMinute(),CurTime.GetSecond(),m_struDeviceInfo.struChanInfo[GetCurChanIndex()].iChanIndex);
    
		sprintf(PicName, "./capture/1-%d.jpg", picNum++);
		if (imwrite(PicName, m_camera2->frame))
		{
			MessageBox("抓图成功!");
		}
	}
	else if(0 == iPicType)  //jpg
	{
		CTime CurTime = CTime::GetCurrentTime();;
		//sprintf(PicName,"%04d%02d%02d%02d%02d%02d_ch%02d.jpg",CurTime.GetYear(),CurTime.GetMonth(),CurTime.GetDay(), \
			CurTime.GetHour(),CurTime.GetMinute(),CurTime.GetSecond(),m_struDeviceInfo.struChanInfo[GetCurChanIndex()].iChanIndex);
		//组建jpg结构
		sprintf(PicName, "./capture/2-%d.jpg", picNum++);
		NET_DVR_JPEGPARA JpgPara = {0};
        JpgPara.wPicSize = (WORD)m_coJpgSize.GetCurSel();
		JpgPara.wPicQuality = (WORD)m_coJpgQuality.GetCurSel();

		LONG iCurChan = m_struDeviceInfo.struChanInfo[GetCurChanIndex()].iChanIndex;

		if(m_camera1->idx == 1 && (m_struDeviceInfo.lLoginID, iCurChan, &JpgPara, PicName))
		{
			MessageBox("抓图成功");
		}
		
		if (m_camera1->idx == 3 && imwrite(PicName, m_camera1->frame))
		{
			MessageBox("抓图成功!");
		}
	}
	
	return;	
}

/*************************************************
函数名:    	OnButtonPlay
函数描述:	播放选中通道
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnButtonPlay() 
{
	if(m_iCurChanIndex == -1)
	{
		MessageBox("选择一个通道");
		return;
	}
	if(!m_bIsPlaying)
	{
         StartPlay(m_iCurChanIndex);
		 m_bIsPlaying = TRUE;
		 GetDlgItem(IDC_BUTTON_PLAY)->SetWindowText("停止播放");

	}
	else
	{
		StopPlay();
		m_bIsPlaying = FALSE;
		GetDlgItem(IDC_BUTTON_PLAY)->SetWindowText("播放");
	}
	
}


/*************************************************
函数名:    	GetDecoderCfg
函数描述:	获取云台解码器信息
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::GetDecoderCfg()
{
     NET_DVR_DECODERCFG_V30 DecoderCfg;
	 DWORD  dwReturned;
	 BOOL bRet;
		
      
	 //获取通道解码器信息
	 for(int i=0; i<MAX_CHANNUM_V30; i++)
	 {
		 if(m_struDeviceInfo.struChanInfo[i].bEnable)
		 {
			 memset(&DecoderCfg,0,sizeof(NET_DVR_DECODERCFG_V30));
			 bRet = NET_DVR_GetDVRConfig(m_struDeviceInfo.lLoginID,NET_DVR_GET_DECODERCFG_V30 , \
				 m_struDeviceInfo.struChanInfo[i].iChanIndex,&DecoderCfg,sizeof(NET_DVR_DECODERCFG_V30),&dwReturned);
			 if(!bRet)
			 {
				 TRACE("Get DecderCfg failed,Chan:%d\n",m_struDeviceInfo.struChanInfo[i].iChanIndex);
				 continue;
			 }

			 memcpy(&m_struDeviceInfo.struChanInfo[i].struDecodercfg,&DecoderCfg,sizeof(NET_DVR_DECODERCFG_V30));
		 }
		 
	 }

}

/*************************************************
函数名:    	InitDecoderReferCtrl
函数描述:	初始化云台控制相关控件
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::InitDecoderReferCtrl()
{
    int i;
	CString tmp;
	//设置预置点
	for(i=0; i<MAX_PRESET_V30; i++)
	{  
		tmp.Format("%d",i+1);     //i+1
		m_comboPreset.AddString(tmp);
	}
	m_comboPreset.SetCurSel(0);

	GetDlgItem(IDC_BUTTON_PRESET_GOTO)->EnableWindow(FALSE);
    //GetDlgItem(IDC_BUTTON_PRESET_SET)->EnableWindow(FALSE);
	GetDlgItem(IDC_BUTTON_PRESET_DEL)->EnableWindow(FALSE);

	//巡航轨迹
    for(i=0; i<MAX_CRUISE_SEQ; i++)
	{
         tmp.Format("%d",i+1);     //i+1
		 m_comboSeq.AddString(tmp);
	}
	m_comboSeq.SetCurSel(0);


}



/*************************************************
函数名:    	OnSelchangeComboPreset
函数描述:	预置点combobox，改变按钮状态
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnSelchangeComboPreset() 
{
	int iIndex = m_comboPreset.GetCurSel();    
	
	if(m_struDeviceInfo.struChanInfo[m_iCurChanIndex].struDecodercfg.bySetPreset[iIndex])
	{
        GetDlgItem(IDC_BUTTON_PRESET_GOTO)->EnableWindow(TRUE);
		GetDlgItem(IDC_BUTTON_PRESET_DEL)->EnableWindow(TRUE);
	}
	else
	{
        GetDlgItem(IDC_BUTTON_PRESET_GOTO)->EnableWindow(FALSE);
		GetDlgItem(IDC_BUTTON_PRESET_DEL)->EnableWindow(FALSE);
	}
}

/*************************************************
函数名:    	OnButtonPresetGoto
函数描述:	调用预置点
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnButtonPresetGoto() 
{
	int iPreset = m_comboPreset.GetCurSel()+1;    //+1
	if(m_lPlayHandle >= 0)
	{
		if(!NET_DVR_PTZPreset(m_lPlayHandle,GOTO_PRESET,iPreset))
		{
			MessageBox("调用预置点失败");
			return;
		}
	}
	else
	{
		if(!NET_DVR_PTZPreset_Other(m_struDeviceInfo.lLoginID,m_struDeviceInfo.struChanInfo[m_iCurChanIndex].iChanIndex, \
			                  GOTO_PRESET,iPreset))
		{
			MessageBox("调用预置点失败");
			return;
		}

	}
}

/*************************************************
函数名:    	OnButtonPresetSet
函数描述:	设置预置点
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnButtonPresetSet() 
{
	int iPreset = m_comboPreset.GetCurSel()+1;    //+1
	if(m_lPlayHandle >= 0)
	{
		if(!NET_DVR_PTZPreset(m_lPlayHandle,SET_PRESET,iPreset))
		{
			MessageBox("设置预置点失败");
			return;
		}
	}
	else
	{
		if(!NET_DVR_PTZPreset_Other(m_struDeviceInfo.lLoginID,m_struDeviceInfo.struChanInfo[m_iCurChanIndex].iChanIndex, \
			SET_PRESET,iPreset))
		{
			MessageBox("设置预置点失败");
			return;
		}
		
	}

	//添加到预置点信息
	m_struDeviceInfo.struChanInfo[m_iCurChanIndex].struDecodercfg.bySetPreset[iPreset-1] = TRUE;
	//更新按钮状态
    OnSelchangeComboPreset();
	
}


/*************************************************
函数名:    	OnButtonPresetDel
函数描述:	删除预置点
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnButtonPresetDel() 
{
	int iPreset = m_comboPreset.GetCurSel()+1;    //+1
	if(m_lPlayHandle >= 0)
	{
		if(!NET_DVR_PTZPreset(m_lPlayHandle,CLE_PRESET,iPreset))
		{
			MessageBox("删除预置点失败");
			return;
		}
	}
	else
	{
		if(!NET_DVR_PTZPreset_Other(m_struDeviceInfo.lLoginID,m_struDeviceInfo.struChanInfo[m_iCurChanIndex].iChanIndex, \
			CLE_PRESET,iPreset))
		{
			MessageBox("删除预置点失败");
			return;
		}
		
	}
	
	//添加到预置点信息
	m_struDeviceInfo.struChanInfo[m_iCurChanIndex].struDecodercfg.bySetPreset[iPreset-1] = FALSE;
	//更新按钮状态
    OnSelchangeComboPreset();
	
}




/*************************************************
函数名:    	OnButtonSeqGoto
函数描述:	调用/停止 巡航
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnButtonSeqGoto() 
{
	int iSeq = m_comboSeq.GetCurSel()+1;    //+1
	if(!m_bIsOnCruise)
	{
		if(m_lPlayHandle >= 0)
		{
	    	if(!NET_DVR_PTZCruise(m_lPlayHandle,RUN_SEQ,iSeq,0,0))
			{
		    	MessageBox("调用巡航失败");
		    	return;
			}
		}
	    else
		{
	    	if(!NET_DVR_PTZCruise_Other(m_struDeviceInfo.lLoginID,m_struDeviceInfo.struChanInfo[m_iCurChanIndex].iChanIndex, \
		                             	RUN_SEQ,iSeq,0,0))
			{
		    	MessageBox("调用巡航失败");
		    	return;
			}
		
		}
        m_bIsOnCruise = TRUE;
		GetDlgItem(IDC_BUTTON_SEQ_GOTO)->SetWindowText("停止");
	}
	else
	{
        if(m_lPlayHandle >= 0)
		{
			if(!NET_DVR_PTZCruise(m_lPlayHandle,STOP_SEQ,iSeq,0,0))
			{
				MessageBox("停止巡航失败");
				return;
			}
		}
		else
		{
			if(!NET_DVR_PTZCruise_Other(m_struDeviceInfo.lLoginID,m_struDeviceInfo.struChanInfo[m_iCurChanIndex].iChanIndex, \
				STOP_SEQ,iSeq,0,0))
			{
				MessageBox("停止巡航失败");
				return;
			}
			
		}
		m_bIsOnCruise = FALSE;
		GetDlgItem(IDC_BUTTON_SEQ_GOTO)->SetWindowText("调用");
	}
	
}

/*************************************************
函数名:    	OnButtonSeqSet
函数描述:	设置巡航路径
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnButtonSeqSet() 
{
	CDlgPTZCruise Dlg;
	Dlg.DoModal();
	
}


/*************************************************
函数名:    	OnButtonTrackRun
函数描述:	开始运行轨迹
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnButtonTrackRun() 
{
	if(!m_bTrackRun)
	{
		if(m_lPlayHandle >= 0)
		{
			if(!NET_DVR_PTZTrack(m_lPlayHandle,RUN_CRUISE))
			{
				MessageBox("运行轨迹失败");
			}
		}
		else
		{
			if(!NET_DVR_PTZTrack(m_struDeviceInfo.lLoginID,m_struDeviceInfo.struChanInfo[m_iCurChanIndex].iChanIndex))
			{
				MessageBox("运行轨迹失败");
			}
		}
		m_bTrackRun = TRUE;
		GetDlgItem(IDC_BUTTON_TRACK_RUN)->SetWindowText("停止");    
	}
	else
	{
        //随便发送一个云台控制命令停止运行
        if(m_lPlayHandle >= 0)
		{
			if(!NET_DVR_PTZControl(m_lPlayHandle,TILT_UP,1))
			{
				MessageBox("停止轨迹失败");
			}
		}
		else
		{
			if(!NET_DVR_PTZControl_Other(m_struDeviceInfo.lLoginID,m_struDeviceInfo.struChanInfo[m_iCurChanIndex].iChanIndex,TILT_UP,1))
			{
				MessageBox("停止轨迹失败");
			}
		}
		m_bTrackRun = FALSE;
		GetDlgItem(IDC_BUTTON_TRACK_RUN)->SetWindowText("运行");   
	}


	
}

/*************************************************
函数名:    	OnButtonTrackStart
函数描述:	开始记录轨迹
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnButtonTrackStart() 
{
	
	if(m_lPlayHandle >= 0)
	{
		if(!NET_DVR_PTZTrack(m_lPlayHandle,STA_MEM_CRUISE))
		{
			MessageBox("开始记录轨迹失败");
			return;
		}

	}
	else
	{
       if(!NET_DVR_PTZTrack_Other(m_struDeviceInfo.lLoginID,m_struDeviceInfo.struChanInfo[m_iCurChanIndex].iChanIndex,STA_MEM_CRUISE))
	   {
		   MessageBox("开始记录轨迹失败");
			return;
	   }
	}

	
}

/*************************************************
函数名:    	OnButtonTrackStop
函数描述:	停止记录轨迹
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnButtonTrackStop() 
{
	if(m_lPlayHandle >= 0)
	{
		if(!NET_DVR_PTZTrack(m_lPlayHandle,STO_MEM_CRUISE))
		{
			MessageBox("停止失败");
			return;
		}
		
	}
	else
	{
		if(!NET_DVR_PTZTrack_Other(m_struDeviceInfo.lLoginID,m_struDeviceInfo.struChanInfo[m_iCurChanIndex].iChanIndex,STO_MEM_CRUISE))
		{
			MessageBox("停止失败");
			return;
		}
	}
	
}

/*************************************************
函数名:    	OnBtnPtzAuto
函数描述:	云台开始/停止左右自动扫描
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnBtnPtzAuto() 
{
	int iSpeed = GetPTZSpeed();
	if(m_lPlayHandle >= 0)
	{
	if(!m_bAutoOn)
	{
       if(iSpeed >= 1)
	   {
		   NET_DVR_PTZControlWithSpeed(m_lPlayHandle,PAN_AUTO,0,iSpeed);
	   }
	   else
	   {
		    NET_DVR_PTZControl(m_lPlayHandle,PAN_AUTO,0);
	   }
        GetDlgItem(IDC_BTN_PTZ_AUTO)->SetWindowText("停止");
		m_bAutoOn = TRUE;
	}
	else
	{
		 if(iSpeed >= 1)
		 {
			 NET_DVR_PTZControlWithSpeed(m_lPlayHandle,PAN_AUTO,1,iSpeed);
		 }
		 else
		 {
             NET_DVR_PTZControl(m_lPlayHandle,PAN_AUTO,1);
		 }
		 GetDlgItem(IDC_BTN_PTZ_AUTO)->SetWindowText("自动");
		 m_bAutoOn = FALSE;
	}
	}
	
}

/*************************************************
函数名:    	OnBtnAuxPwron
函数描述:	开始/停止 辅助设备1
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnBtnAuxPwron() 
{
	if(m_lPlayHandle >= 0)
	{
		if(!m_bAuxOn1)
		{
			NET_DVR_PTZControl(m_lPlayHandle,AUX_PWRON1,0);
			GetDlgItem(IDC_BTN_AUX_PWRON)->SetWindowText("停止1");
			m_bAuxOn1 = TRUE;
		}
		else
		{
			NET_DVR_PTZControl(m_lPlayHandle,AUX_PWRON1,1);
			GetDlgItem(IDC_BTN_AUX_PWRON)->SetWindowText("辅助1");
			m_bAuxOn1 = FALSE;
		}
	}

	
}

/*************************************************
函数名:    	OnBtnAuxPwron2
函数描述:	开始/停止 辅助设备2
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnBtnAuxPwron2() 
{
	if(m_lPlayHandle >= 0)
	{
		if(!m_bAuxOn2)
		{
			NET_DVR_PTZControl(m_lPlayHandle,AUX_PWRON2,0);
			GetDlgItem(IDC_BTN_AUX_PWRON2)->SetWindowText("停止2");
			m_bAuxOn2 = TRUE;
		}
		else
		{
			NET_DVR_PTZControl(m_lPlayHandle,AUX_PWRON2,1);
			GetDlgItem(IDC_BTN_AUX_PWRON2)->SetWindowText("辅助2");
			m_bAuxOn2 = FALSE;
		}
	}
	
}

/*************************************************
函数名:    	OnBtnFanPwron
函数描述:	开始/停止 风扇
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnBtnFanPwron() 
{
	if(m_lPlayHandle >= 0)
	{
		if(!m_bFanOn)
		{
			NET_DVR_PTZControl(m_lPlayHandle,FAN_PWRON,0);
			GetDlgItem(IDC_BTN_FAN_PWRON)->SetWindowText("停风");
			m_bFanOn = TRUE;
		}
		else
		{
			NET_DVR_PTZControl(m_lPlayHandle,FAN_PWRON,1);
			GetDlgItem(IDC_BTN_FAN_PWRON)->SetWindowText("风扇");
			m_bFanOn = FALSE;
		}
	}
	
	
}

/*************************************************
函数名:    	OnBtnHeaterPwron
函数描述:	开始/停止加热
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnBtnHeaterPwron() 
{
	if(m_lPlayHandle >= 0)
	{
		if(!m_bHeaterOn)
		{
			NET_DVR_PTZControl(m_lPlayHandle,HEATER_PWRON,0);
			GetDlgItem(IDC_BTN_HEATER_PWRON)->SetWindowText("停止");
			m_bHeaterOn = TRUE;
		}
		else
		{
			NET_DVR_PTZControl(m_lPlayHandle,HEATER_PWRON,1);
			GetDlgItem(IDC_BTN_HEATER_PWRON)->SetWindowText("加热");
			m_bHeaterOn = FALSE;
		}
	}
	
	
}

/*************************************************
函数名:    	OnBtnLightPwron
函数描述:	打开/关闭灯光
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnBtnLightPwron() 
{
	if(m_lPlayHandle >= 0)
	{
		if(!m_bLightOn)
		{
			NET_DVR_PTZControl(m_lPlayHandle,LIGHT_PWRON,0);
			GetDlgItem(IDC_BTN_LIGHT_PWRON)->SetWindowText("关灯");
			m_bLightOn = TRUE;
		}
		else
		{
			NET_DVR_PTZControl(m_lPlayHandle,LIGHT_PWRON,1);
			GetDlgItem(IDC_BTN_LIGHT_PWRON)->SetWindowText("灯光");
			m_bLightOn = FALSE;
		}
	}
	
}

/*************************************************
函数名:    	OnBtnWiperPwron
函数描述:	开始/停止雨刷
输入参数:   
输出参数:   			
返回值:		
**************************************************/
void CRealPlayDlg::OnBtnWiperPwron() 
{
	if(m_lPlayHandle >= 0)
	{	
		if(!m_bWiperOn)
		{
		    NET_DVR_PTZControl(m_lPlayHandle,WIPER_PWRON,0);
		    GetDlgItem(IDC_BTN_WIPER_PWRON)->SetWindowText("雨停");
		    m_bWiperOn = TRUE;
		}
	    else
		{
            NET_DVR_PTZControl(m_lPlayHandle,WIPER_PWRON,1);
	    	GetDlgItem(IDC_BTN_WIPER_PWRON)->SetWindowText("雨刷");
	    	m_bWiperOn = FALSE;
		}

	}

}


void CRealPlayDlg::OnSelchangeComboPicType() 
{
	int iSel = m_coPicType.GetCurSel();
	if(0 == iSel)  //bmp
	{   
		GetDlgItem(IDC_STATIC_JPGPARA)->EnableWindow(FALSE);
        m_coJpgSize.EnableWindow(FALSE);
		m_coJpgQuality.EnableWindow(FALSE);
	}
	else if(1 == iSel)          //jpg
	{
		GetDlgItem(IDC_STATIC_JPGPARA)->EnableWindow(TRUE);
        m_coJpgSize.EnableWindow(TRUE);
		m_coJpgQuality.EnableWindow(TRUE);
	}
	
}

void CRealPlayDlg::OnClose() 
{
	// TODO: Add your message handler code here and/or call default
	if (m_bIsLogin)
	{
        StopPlay();		
        NET_DVR_Logout_V30(m_struDeviceInfo.lLoginID); 
	}
	CDialog::OnClose();
}





void CRealPlayDlg::OnBnClickedBtnCameraOpen()
{
	// TODO: 在此添加控件通知处理程序代码
	//SetTimer(2, 40, NULL);
	if(m_arm->IsOpen()){
		m_arm->Close();
	}
	if (!m_arm->Open(comId, 115200, NOPARITY, 8, ONESTOPBIT)) {
		MessageBox("fail connect");
	}
	else m_camera2->isopen = true;
	m_camera2->Open();
	m_camera1->Open();

	SetTimer(1, 50, NULL);
	//m_camera->m_com.SetWnd(AfxGetMainWnd()->m_hWnd);
	//m_camera->Reset();

}

afx_msg LRESULT CRealPlayDlg::OnComReceive(WPARAM wParam, LPARAM lParam)
{
   //读取串口上的字符
   char buf[1024];
   m_arm->ReadString(buf, 1024);  //ReadString适合读一般字符串，Read函数适合读取任意数据
   return 0;
}

void CRealPlayDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	switch(nIDEvent)
	{
	case 1:
		{
			Mat frame1, frame2;
			frame1 = m_camera1->GetFrame();
			frame2 = m_camera2->GetFrame();

			Track(m_camera1, frame1);
			Track(m_camera2, frame2);

			if (m_camera1->state == 2 && m_camera2->state == 2) {
				cv::Mat test = triangulation(m_camera1->centerOfBox, m_camera2->centerOfBox, m_camera1, m_camera2);
				cv::Mat real3d(3, 1, CV_64F);
				double dtest = test.at<double>(3);
				for (int i = 0; i < 3; i++) real3d.at<double>(i) = test.at<double>(i) / dtest;
				TRACE("triangulation : %f %f %f \n", real3d.at<double>(0), real3d.at<double>(1), real3d.at<double>(2));
				m_camera1->SetReal3D(real3d);
				m_camera2->SetReal3D(real3d);
			}

			if (m_camera1->state >= 2 || m_camera1->state == 3) m_camera1->CameraMove(2);
			if (m_camera2->state >= 2 || m_camera2->state == 3) m_camera2->CameraMove(2);

			Paint(m_camera1, frame1);
			Paint(m_camera2, frame2);

			/*
			if (m_camera2->state == 1) {

				cv::Mat test = triangulation(m_camera1->centerOfBox, m_camera2->centerOfBox, m_camera1, m_camera2);
				cv::Mat real3d(3, 1, CV_64F);
				double dtest = test.at<double>(3);
				for (int i = 0; i < 3; i++) real3d.at<double>(i) = test.at<double>(i) / dtest;
				TRACE("triangulation : %f %f %f \n", real3d.at<double>(0), real3d.at<double>(1), real3d.at<double>(2));
				m_camera1->SetReal3D(real3d);
				m_camera2->SetReal3D(real3d);
				// test
				m_camera1->state = 1;
				m_camera2->state = 2;

			}
			*/
		}
		break;
	case 2:
		{
			
		}
		break;
	}
}



void CRealPlayDlg::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
	LONG lPlayHandle = g_pMainDlg->GetPlayHandle();
	int iPTZSpeed = g_pMainDlg->GetPTZSpeed();
	bool ret;
	clock_t begin = clock();
	//for (int i = 0; i < 25; i++) {
		ret = NET_DVR_PTZControlWithSpeed(lPlayHandle,PAN_RIGHT,0,iPTZSpeed);
		if(!ret)
		{
			MessageBox("云台控制失败!");
		}
		Sleep(40);
		NET_DVR_PTZControlWithSpeed(lPlayHandle,PAN_RIGHT,1,iPTZSpeed);
	//}
	clock_t end = clock();
	double err = 1.0 * (end - begin) / CLOCKS_PER_SEC;
	haveMove = 1;
}


void CRealPlayDlg::OnBnClickedBtnStartTracking()
{
	// TODO: 在此添加控件通知处理程序代码
	m_camera1->isTracking = true;
	m_camera2->isTracking = true;
	if (m_camera1->state == 1) {
		m_camera1->state = 2;
	}
	if (m_camera2->state == 1) {
		m_camera2->state = 2;
	}

}

void CRealPlayDlg::Track(Camera *cam, Mat &frame) {
	cam->frame = frame;
	if (cam->state == 2 || cam->state == 3) {
		bool isok = cam->TrackFromImage(frame);
		if (!isok) cam->state = 3;
		if (isok) cam->state = 2;
	}
	if (cam->state == 3 && m_camera1->state == 2) {
		cv::Mat pt3d = m_camera1->GetReal3D();
		cam->TrackFromReal3D(pt3d);
	}
}

void CRealPlayDlg::Paint(Camera *cam, Mat &frame) {
	if (haveMove && cam->idx == 2) {
		/*
		cv::Mat R, t;
		haveMove = 0;
		cap >> frame;
		poseCal.computePoseFromImage(cam->frame, frame, cam->K, R, t);
		TRACE("R size: %d %d %d\n", R.cols, R.rows, R.type());
		TRACE("Angles : %lf %lf %lf\n", R.at<double>(0, 0), R.at<double>(1, 1));
		Eigen::Matrix3d rotate;
		Eigen::Vector3d angles;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				rotate(i, j) = R.at<double>(i, j);
			}
		}
		double sy = std::sqrt(rotate(0, 0) * rotate(0, 0) + rotate(1, 0) * rotate(1, 0));

		bool singular = sy < 1e-6; // If

		double x, y, z;
		if (!singular)
		{
			x = std::atan2(rotate(2, 1), rotate(2, 2));
			y = std::atan2(-rotate(2, 0), sy);
			z = std::atan2(rotate(1, 0), rotate(0, 0));
		}
		else
		{
			x = std::atan2(-rotate(1, 2), rotate(1, 1));
			y = std::atan2(-rotate(2, 0), sy);
			z = 0;
		}
		angles = Eigen::Vector3d(x, y, z);
		TRACE("Angles : %lf %lf %lf\n", angles(0), angles(1), angles(2));
		*/
	}
	CDC* pDC;
	HDC hDC;
	CvvImage cimg;
	IplImage img;
	CRect rect;

	pDC = GetDlgItem(cam->IDC_CAMERA_SHOW)->GetDC();     //根据ID获得窗口指针再获取与该窗口关联的上下文指针
	hDC = pDC->GetSafeHdc();      // 获取设备上下文句柄 
	img = cam->frame;
	cimg.CopyOf(&img);    //复制该帧图像
	GetDlgItem(cam->IDC_CAMERA_SHOW)->GetClientRect(&rect);
	double scalex = 1.0 * frame.cols / cam->widthOfScreen, scaley = 1.0 *  frame.rows/ cam->heightOfScreen;
	if (cam->mouseState == 1) {
		cam->boundingBox = cv::Rect(scalex * cam->topCornerOfShowBox.x, scaley * cam->topCornerOfShowBox.y, scalex * (cam->bottomCornerOfShowBox.x - cam->topCornerOfShowBox.x), scaley * (cam->bottomCornerOfShowBox.y - cam->topCornerOfShowBox.y));
		cam->preBoundingBox = cam->boundingBox;
	}
	cimg.DrawToHDC(hDC, &rect);   //显示到设备的矩形框内
	if(cam->mouseState == 1 || cam->state == 1 || cam->state == 2) {
		
		CPen pen;
		pen.CreatePen(PS_SOLID, 3, RGB(255, 0, 0));
		pDC = GetDlgItem(cam->IDC_CAMERA_SHOW)->GetDC();     //根据ID获得窗口指针再获取与该窗口关联的上下文指针
		CRect drawRect = CRect(max(0.0, cam->boundingBox.x / scalex), max(0.0, cam->boundingBox.y / scaley), min(cam->widthOfScreen, (cam->boundingBox.x + cam->boundingBox.width) / scalex), min(cam->heightOfScreen, (cam->boundingBox.y + cam->boundingBox.height) / scaley));
		pDC->SelectObject(pen);
		pDC->SelectStockObject(NULL_BRUSH);
		pDC->Rectangle(&drawRect);
	}	
	if (cam->state == 3) {
		CPen pen;
		pen.CreatePen(PS_SOLID, 3, RGB(0, 255, 255));
		pDC = GetDlgItem(cam->IDC_CAMERA_SHOW)->GetDC();     //根据ID获得窗口指针再获取与该窗口关联的上下文指针
		CRect drawRect = CRect(max(0.0, cam->boundingBox.x / scalex), max(0.0, cam->boundingBox.y / scaley), min(cam->widthOfScreen, (cam->boundingBox.x + cam->boundingBox.width) / scalex), min(cam->heightOfScreen, (cam->boundingBox.y + cam->boundingBox.height) / scaley));
		pDC->SelectObject(pen);
		pDC->SelectStockObject(NULL_BRUSH);
		pDC->Rectangle(&drawRect);
	}
	if (cam->state == 4) {
		CPen pen;
		pen.CreatePen(PS_SOLID, 3, RGB(0, 0, 255));
		pDC = GetDlgItem(cam->IDC_CAMERA_SHOW)->GetDC();     //根据ID获得窗口指针再获取与该窗口关联的上下文指针
		CRect drawRect = CRect(max(0.0, cam->boundingBox.x / scalex), max(0.0, cam->boundingBox.y / scaley), min(cam->widthOfScreen, (cam->boundingBox.x + cam->boundingBox.width) / scalex), min(cam->heightOfScreen, (cam->boundingBox.y + cam->boundingBox.height) / scaley));
		pDC->SelectObject(pen);
		pDC->SelectStockObject(NULL_BRUSH);
		pDC->Rectangle(&drawRect);
	}
	if (cam->state == 2 || cam->state == 3) {
		PathShow(cam);
	}
	ReleaseDC(pDC);
}

void CRealPlayDlg::PathShow(Camera *cam) {
	if (cam->idx == 2 && cam->path.size() >= 2) {
		CDC* pDC = GetDlgItem(IDC_PATH1_SHOW)->GetDC();
		CPen pen;
		pen.CreatePen(PS_SOLID, 3, RGB(255, 0, 0));
		pDC->SelectObject(pen);
		double pretx = cam->path[cam->path.size() - 2].second * cos(cam->path[cam->path.size() - 2].first) * 50 + 30;
		double prety = cam->path[cam->path.size() - 2].second * sin(cam->path[cam->path.size() - 2].first) * 50 + 30;
		double tx = cam->path[cam->path.size() - 1].second * cos(cam->path[cam->path.size() - 1].first) * 50 + 30;
		double ty = cam->path[cam->path.size() - 1].second * sin(cam->path[cam->path.size() - 1].first) * 50 + 30;
		pDC->MoveTo(prety, pretx);
		pDC->LineTo(ty, tx);
		TRACE("point : %f %f\n ", cam->path[cam->path.size() - 1].second, cam->path[cam->path.size() - 1].first / acos(-1) * 180);
	}
}

void CRealPlayDlg::myOnLButtonDown(Camera *cam, CPoint point) {
	CRect rect;//定义一个矩形框，包含左上角和右下角可访问成员
	GetDlgItem(cam->IDC_CAMERA_SHOW)->GetClientRect(rect);//获取Picture控件的位置信息，存入rect中
	GetDlgItem(cam->IDC_CAMERA_SHOW)->ClientToScreen(rect);//转换成屏幕坐标
	if (rect.PtInRect(point))//判断point是否在rect内部
	{
		if(cam->mouseState == 0) {
			cam->topCornerOfShowBox.x = point.x - rect.left; 
			cam->topCornerOfShowBox.y = point.y - rect.top;
			cam->bottomCornerOfShowBox.x = point.x - rect.left; 
			cam->bottomCornerOfShowBox.y = point.y - rect.top;
			cam->mouseState = 1;
		}
	}
}
void CRealPlayDlg::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	ClientToScreen(&point);//将鼠标坐标转换成屏幕坐标
	myOnLButtonDown(m_camera1, point);
	myOnLButtonDown(m_camera2, point);
}

void CRealPlayDlg::myOnMouseMove(Camera *cam, CPoint point) {
	CRect rect;//定义一个矩形框，包含左上角和右下角可访问成员
	GetDlgItem(cam->IDC_CAMERA_SHOW)->GetClientRect(rect);//获取Picture控件的位置信息，存入rect中
	GetDlgItem(cam->IDC_CAMERA_SHOW)->ClientToScreen(rect);//转换成屏幕坐标
	if (rect.PtInRect(point)){//判断point是否在rect内部
		if(cam->mouseState == 1) {
			cam->bottomCornerOfShowBox.x = point.x - rect.left; 
			cam->bottomCornerOfShowBox.y = point.y - rect.top;
		}
	}
	

}
void CRealPlayDlg::OnMouseMove(UINT nFlags, CPoint point)
{
	ClientToScreen(&point);//将鼠标坐标转换成屏幕坐标
	myOnMouseMove(m_camera1, point);
	myOnMouseMove(m_camera2, point);
}

void CRealPlayDlg::myOLButtonUp(Camera *cam, CPoint point) {
	CRect rect;//定义一个矩形框，包含左上角和右下角可访问成员
	GetDlgItem(cam->IDC_CAMERA_SHOW)->GetClientRect(rect);//获取Picture控件的位置信息，存入rect中
	GetDlgItem(cam->IDC_CAMERA_SHOW)->ClientToScreen(rect);//转换成屏幕坐标
	if (rect.PtInRect(point)){//判断point是否在rect内部
		if(cam->mouseState == 1) {
			cam->state = 1;
			cam->bottomCornerOfShowBox.x = point.x - rect.left; 
			cam->bottomCornerOfShowBox.y = point.y - rect.top;
			cam->mouseState = 2;
		}
	}
}


void CRealPlayDlg::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	ClientToScreen(&point);//将鼠标坐标转换成屏幕坐标
	myOLButtonUp(m_camera1, point);
	myOLButtonUp(m_camera2, point);
}



void CRealPlayDlg::OnBnClickedBtnUarmLeft()
{
	// TODO:  在此添加控件通知处理程序代码
	if (m_camera2) {
		m_camera2->MoveToPolarPosition(10, 0, 0, 12000, 2);
	}
}


void CRealPlayDlg::OnBnClickedBtnUarmRight()
{
	// TODO:  在此添加控件通知处理程序代码
	if (m_camera2) {
		m_camera2->MoveToPolarPosition(-10, 0, 0, 12000, 2);
	}
}


void CRealPlayDlg::OnBnClickedBtnUarmStop()
{
	// TODO:  在此添加控件通知处理程序代码
	if (m_camera2) {
		m_camera2->StopMovement();
	}
}


void CRealPlayDlg::OnBnClickedBtnUarmUp()
{
	// TODO:  在此添加控件通知处理程序代码
	if (m_camera2) {
		m_camera2->MoveToPolarPosition(0, 0, 30, 1000, 2);
	}
}


void CRealPlayDlg::OnBnClickedBtnUarmDown()
{
	// TODO:  在此添加控件通知处理程序代码
	if (m_camera2) {
		m_camera2->MoveToPolarPosition(0, 0, -30, 1000, 2);
	}
}


void CRealPlayDlg::OnBnClickedBtnUarmReset()
{
	// TODO:  在此添加控件通知处理程序代码
	if (m_camera2) {
		m_camera2->Reset();
	}
}
