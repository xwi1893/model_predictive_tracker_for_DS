//=============================================================================
/** @file
 *  @brief NetComm�F�ԗ��f�[�^��M�p�\���̒�` (���M�f�[�^�͌Œ蒷)
 *
 *
 *
 *  @author Hiroki TODA
 *
 *  @par ���r�W������� 
 *  @n $Revision$
 *  @n $Date$
 *  @n $Author$
 *  @n Copyright (c) 2008 MITSUBISHI PRECISION Co.,Ltd. All Rights Reserved.
 */
//=============================================================================
#ifndef NCI_CAR_DATA_H
#define NCI_CAR_DATA_H


/// AT�V�t�g���o�[�ʒu
enum eCarShiftPos {
	E_SHIFT_None	= (-99),		// �Ȃ�
	E_SHIFT_P		= (-9),			// P �����W(AT)
	E_SHIFT_R		= (-1),			// R �����W(AT,MT)
	E_SHIFT_N		= (0),			// N �����W(AT,MT)
	E_SHIFT_M		= (10),			// M(S) �����W(AT)
	E_SHIFT_D		= (9),			// D �����W(AT)
	E_SHIFT_8		= (8),			// 8 �����W(AT,MT)
	E_SHIFT_7		= (7),			// 7 �����W(AT,MT)
	E_SHIFT_6		= (6),			// 6 �����W(AT,MT)
	E_SHIFT_5		= (5),			// 5 �����W(AT,MT)
	E_SHIFT_4		= (4),			// 4 �����W(AT,MT)
	E_SHIFT_3		= (3),			// 3 �����W(AT,MT)
	E_SHIFT_2		= (2),			// 2 �����W(AT,MT)
	E_SHIFT_1		= (1)			// 1 �����W(AT,MT)
};


/// ���C�g�X�C�b�`�ʒu (cLightSw)
enum eCarLightSw {
	E_LIGHT_SW_OFF		= (0),		// ���C�g OFF
	E_LIGHT_SW_SMALL	= (1),		// ���C�g SMALL
	E_LIGHT_SW_ON		= (2),		// ���C�g ON
	E_LIGHT_SW_AUTO	    = (9)		// ���C�g AUTO
};


/// �C�O�j�b�V�����L�[�ʒu (cKeyPos)
enum eCarKeyPos {
	E_KEY_LOCK		= (0),			///< �C�O�j�b�V�����L�[ LOCK
	E_KEY_OFF		= (1),			///< �C�O�j�b�V�����L�[ OFF
	E_KEY_ACC		= (2),			///< �C�O�j�b�V�����L�[ ACC
	E_KEY_ON		= (3),			///< �C�O�j�b�V�����L�[ ON
	E_KEY_START		= (4)			///< �C�O�j�b�V�����L�[ START
};


/// ���C�p�[�X�C�b�`�ʒu (cCtrlWiperPos)
enum eCarWiper {
	E_WIPER_OFF = (0),				///< ���C�p�[ OFF
	E_WIPER_INT	= (1),				///< ���C�p�[ INT
	E_WIPER_L   = (2),				///< ���C�p�[ LOW
	E_WIPER_H	= (3)				///< ���C�p�[ HIGH
};

/// ���C�g�ޓ_����� (hLights)
enum eCarLight {
	E_LIGHTS_OFF	= (0x0000),		///< ���C�g�� �SOFF
	E_LIGHTS_POS	= (0x0001),		///< ���C�g�� �|�W�V���������v
	E_LIGHTS_HEAD	= (0x0002),		///< ���C�g�� �w�b�h�����v
	E_LIGHTS_UPPER	= (0x0004),		///< ���C�g�� �w�b�h�����v���s�r�[��
	E_LIGHTS_FOG	= (0x0008),		///< ���C�g�� �t�H�O�����v
	E_LIGHTS_LEFT	= (0x0010),		///< ���C�g�� �^�[���V�O�i����
	E_LIGHTS_RIGHT	= (0x0020),		///< ���C�g�� �^�[���V�O�i���E
	E_LIGHTS_STOP	= (0x0100),		///< ���C�g�� �X�g�b�v�����v
	E_LIGHTS_BACK	= (0x0200),		///< ���C�g�� �o�b�N�A�b�v�����v
	E_LIGHTS_SPR1	= (0x1000),		///< ���C�g�� �\��1
	E_LIGHTS_SPR2	= (0x2000),		///< ���C�g�� �\��2
	E_LIGHTS_SPR3	= (0x4000),		///< ���C�g�� �\��3
	E_LIGHTS_SPR4	= (0x8000)		///< ���C�g�� �\��4
};

/// �g�����X�~�b�V����(cTransmission)
enum eCarTransmission {
	E_TRANSMISSION_4AT,
	E_TRANSMISSION_5AT,
	E_TRANSMISSION_5MT,
	E_TRANSMISSION_6MT,
	E_TRANSMISSION_7MT,
	E_TRANSMISSION_SEMI_AT
};

/// �z��
enum eCarArraySize {
	E_CABO_WRN_SPR			= (16),	// �^�]�ȏo�̓f�[�^: �x����: �\��
	E_CAR_ROAD_SIZE			= (12),	// �H�ʏ�� �E�E 8�֕�(0=�O��,1=�O�E,2=�㍶,3=��E,4=��3�����O,5=��3������,6=��3���E��,7=��3���E�O,8=��4�����O,9=��4������,10=��4���E��,11=��4���E�O)
	E_CAR_TIRE_SIZE			= (12),	// �^�C����� �E�E 8�֕�(0=�O��,1=�O�E,2=�㍶,3=��E,4=��3�����O,5=��3������,6=��3���E��,7=��3���E�O,8=��4�����O,9=��4������,10=��4���E��,11=��4���E�O)
	E_CAR_L_SIZE			= (3),	// �ԗ��`��: ���� �E�E 4����(0=����(��1����),1=��2����,3=��3����)
	E_CAR_B_SIZE			= (6)	// �ԗ��`��: �֋� �E�E 4����(0=��1��,1=��2��,2=��3����,3=��3���O,4=��4����,5=��4���O)
};

/// �z��
enum eCarExArraySize {
    E_CABIN_C_SPR        = (128),   // �ԗ����͊g���p�f�[�^��
    E_CABIN_F_SPR        = (128),   // �ԗ����͊g���p�f�[�^��
    E_CABOUT_C_SPR       = (128),   // �ԗ��o�͊g���p�f�[�^��
    E_CABOUT_F_SPR       = (128),   // �ԗ��o�͊g���p�f�[�^��
    E_CAR_SPR            = (128)    // ���Ԋg���p�f�[�^��
};


//-----------------------------------------------------------------------------
/** @brief  ���ԗ��f�[�^�\����
 *
 */
//-----------------------------------------------------------------------------
typedef struct {

	// �������f�[�^
	struct ST_INIT {
		float fPosX;						///< �� �����ʒu���W X						[m]
		float fPosY;						///< �� �����ʒu���W Y						[m]
		float fPosZ;						///< �� �����ʒu���W Z						[m]
		float fHeading;						///< �� �����ʒu���ʊp H					[deg]
		float fPitch;						///< �� �����ʒu���ʊp P					[deg]
		float fRoll;						///< �� �����ʒu���ʊp R					[deg]
		float fDbHeading;					///< �� �f�[�^�x�[�X���W�n �����ʒu���ʊp H	[deg]
		float fDbPitch;						///< �� �f�[�^�x�[�X���W�n �����ʒu���ʊp P	[deg]
		float fDbRoll;						///< �� �f�[�^�x�[�X���W�n �����ʒu���ʊp R	[deg]
		int   nCPId;						///< �� ����CP ID							[n-d]
		int   nBodyId;						///< �� �����{�f�BID						[n-d]
	} stInit;

	// �Փ˃f�[�^
	struct ST_COLLISION {
		bool  bRdlimXi;						///< �� �͂ݏo�� X ������					[n-d]
		bool  bRdlimXd;						///< �� �͂ݏo�� X �������̔���				[n-d]
		bool  bRdlimYi;						///< �� �͂ݏo�� Y ������					[n-d]
		bool  bRdlimYd;						///< �� �͂ݏo�� Y �������̔���				[n-d]
		bool  bCollision;					///< �� �Փ�								[n-d]
		short hId;                          ///< �� �Փˎԗ�ID(-1:�Փ˂Ȃ�)				[n-d]
	} stCollision;

	/// �^�]�ȓ��̓f�[�^
	struct ST_CABINE_IN {

		// �X�e�A�����O����
		float	fStrAngle;					///< �� ���Ǌp(CCW+)						[deg]
		float	fStrTrq;					///< �� ���ǃg���N(CCW+)					[N�Em]
		
		// �^�]�� I/O ����
		float	fAccel;						///< �� �A�N�Z���X�g���[�N					[n-d]
		float	fBrake;						///< �� �t�b�g�u���[�L�X�g���[�N			[n-d]
		bool	bBrakeSw;					///< �� �t�b�g�u���[�L�X�C�b�` 	    		[n-d]
		float	fClutch;					///< �� �N���b�`�X�g���[�N					[n-d]
		char	cShiftPos;					///< �� �V�t�g�|�W�V����					[n-d]
		char	cShiftPosS;					///< �� �V�t�g�|�W�V����(�V�[�P���V�����p)	[n-d]
		bool	bShiftOdOFF;				///< �� �V�t�g�|�W�V����(OD OFF)			[n-d]
		bool	bShiftCvtPlus;				///< �� �V�t�g�|�W�V����(M+)				[n-d]
		bool	bShiftCvtMinus;				///< �� �V�t�g�|�W�V����(M-)				[n-d]
		char	cLightSw;					///< �� ���C�g�X�C�b�`�ʒu					[n-d]
		bool	bLightUpper;				///< �� ���C�g�X�C�b�`(�A�b�p�[�r�[��)		[n-d]
		bool	bLightPass;					///< �� ���C�g�X�C�b�`(PASS)				[n-d]
		bool	bLightFog;					///< �� ���C�g�X�C�b�`(Fog)					[n-d]
		bool	bWinkerLeft;				///< �� �E�B���J�[ (Left)					[n-d]
		bool	bWinkerRight;				///< �� �E�B���J�[ (Right)					[n-d]
		bool	bHazard;					///< �� Hazard								[n-d]
		char	cKeyPos;					///< �� �C�O�j�b�V�����L�[�ʒu				[n-d]
		float	fPrkBrake;  				///< �� �p�[�L���O�u���[�L�X�g���[�N		[n-d]
		bool	bParkingSw;					///< �� �p�[�L���O�u���[�L�X�C�b�`			[n-d]
		bool	bExhBrakeSw;				///< �� �r�C�u���[�L�X�C�b�`				[n-d]
		bool	bCtrlHornSw;				///< �� ���̑�����n(�z�[���X�C�b�`)		[n-d]
		bool	bCtrlWasherSw;				///< �� ���̑�����n(�E�H�b�V��)			[n-d]
		char	cCtrlWiperPos;				///< �� ���̑�����n(���C�p�X�C�b�`�ʒu)	[n-d]
		char	cCtrlWiperInt;				///< �� ���̑�����n(���C�p�[INT�Ԋu)		[n-d]
		bool	bSafeEmerStop;				///< �� ���S�֘A(���[�V�����ً}��~)		[n-d]
		bool	bSafeDoorLock;				///< �� ���S�֘A(�h�A���b�N)				[n-d]
		bool	bSafeDoorOpen;				///< �� ���S�֘A(�h�A�J��)					[n-d]
		bool	bSafeSheatBelt;				///< �� ���S�֘A(�V�[�g�x���g				[n-d]

	} stInput;

	
	/// �^�]�ȏo�̓f�[�^
	struct ST_CABIN_OUT {

		// �X�e�A�����O�o��
		short	hStrMode;					///< �� ���䃂�[�h							[n-d]
		float	fStrLimitRight;				///< �� ���Ǌp���~�b�g(�E)					[deg]
		float	fStrLimitLeft;				///< �� ���Ǌp���~�b�g(��)					[deg]
		float	fStrIs;						///< �� �I�[�o�[�I�[���E�X�e�A�����O��		[n-d]
		float	fStrBeta;					///< �� �O��������p(��+)					[deg]
		float	fStrWtRate;					///< �� �O���׏d�W��						[%]
		float	fStrMust;					///< �� �O���H�ʖ��C�W��					[%]
		float	fStrCenter;					///< �� ���A�ʒu(CCW+)						[deg]		(not used)
		float	fStrTorqueSlope;			///< �� ���A�g���N�X��						[N�Em/rad]	(not used)
		float	fStrTorque;					///< �� �g���N�E�R�}���h(CCW+)				[N�Em]		(not used)	

		// �^�]�� I/O �o��
		float	fSpeed;						///< �� �X�s�[�h���[�^�[					[km/h]
		float	fRpm;						///< �� �G���W����]��						[rpm]
		float	fTemperature;				///< �� �����v								[n-d]
		float	fFuel;						///< �� �R���v								[n-d]
		bool	bIndWinkerLeft;				///< �� �C���W�P�[�^(�^�[���V�O�i����)		[n-d]
		bool	bIndWinkerRight;			///< �� �C���W�P�[�^(�^�[���V�O�i���E)		[n-d]
		bool	bIndUpperBeam;				///< �� �C���W�P�[�^(�A�b�p�r�[��)			[n-d]
		char	cIndShiftPos;				///< �� �C���W�P�[�^(�V�t�g�|�W�V����)		[n-d]
		char	cIndShiftPosS;				///< �� �C���W�P�[�^(�V�[�P���V�����V�t�g)	[n-d]
		bool	bIndExhBrake;				///< �� �C���W�P�[�^(�G�L�]�[�X�g�u���[�L)	[n-d]
		bool	bWarnAirBag;				///< �� �x����(�G�A�o�b�O)					[n-d]
		bool	bWarnDoor;					///< �� �x����(Door)						[n-d]
		bool	bWarnOdOff;					///< �� �x����(O/D OFF)						[n-d]
		bool	bWarnPrkBrake;				///< �� �x����(PKB)							[n-d]
		bool	bWarnBrake;					///< �� �x����(�u���[�L)					[n-d]
		bool	bWarnSeatBelt;				///< �� �x����(�V�[�g�x���g)				[n-d]
		bool	bWarnAbs;					///< �� �x����(ABS)							[n-d]
		bool	bWarnEngine;				///< �� �x����(�G���W��)					[n-d]
		bool	bWarnOil;					///< �� �x����(OIL)							[n-d]
		bool	bWarnBattery;				///< �� �x����(�o�b�e���[)					[n-d]
		bool	bWarnFuel;					///< �� �x����(�R��)						[n-d]
		bool	bWarnExhTemp;				///< �� �x����(�r�C��)						[n-d]
		bool	bWarnSpare[E_CABO_WRN_SPR]; ///< �� �x����(�\��)						[n-d]
		bool	bIllmInpane;				///< �� �Ɩ�(�v���)						[n-d]
		bool	bSafeDoorLock;				///< �� ���S�֘A(�h�A���b�N)				[n-d]
		bool	bShiftUnlock;				///< �� �V�t�g���o�[���b�N����				[n-d]
		bool	bBackupBuzzer;				///< �� �o�b�N�u�U�[						[n-d]
		bool	bHorn;						///< �� �z�[��								[n-d]
	} stOutput;
	

	/// ���ԃf�[�^
	struct ST_CAR_INFO {
		/// �ԗ����
		short	hId;						///< �� �ԗ�ID								[n-d]
		int		nBodyId;					///< �� BODY ID								[n-d]
		int		nCPId;						///< �� CP ID								[n-d]
		double	dPos[3];					///< �� �ʒu���W(X,Y,Z)						[m]
		double	dAng[3];					///< �� ISO���W�n �p���p(��,��,��)	        [rad]
		double	dDbAng[3];					///< �� �f�[�^�x�[�X���W�n �p���p(��,��,��)	[rad]
		double	dPos_d[3];					///< �� ���x�x�N�g��(u,v,w)					[m/s]
		double	dAng_d[3];					///< �� �p���x�x�N�g��(p,q,r)				[rad/s]
		double	dPos_dd[3];					///< �� �����x�x�N�g��(du/dt,dv/dt,dw/dt)	[m/s2]
		double	dAng_dd[3];					///< �� �p�����x�x�N�g��(dp/dt,dq/dt,dr/dt) [rad/s2]
		float	fTurnRate;					///< �� ����(d����/dt)					[rad/s] (not used)
		float	fSpeed;						///< �� ���x								[m/s]
		float	fEngineN;					///< �� �G���W����]��						[rad/s]
		float	fEngineTorque;				///< �� �G���W���o�̓g���N					[N�Em]
		bool	bEngineState;				///< �� �G���W�����						[n-d]
		short	hGearPos;					///< �� �ϑ��@�M�A�ʒu						[n-d]
		short	hLights;					///< �� ���C�g�ޓ_�����					[n-d]
		char    cTransmission;              ///< �� �g�����X�~�b�V����                  [n-d]   (not used)
		double	dDistanceCovered;			///< �� ���ԑ��s����						[m]    
		bool    bCourseMiss;                ///< �� �R�[�X�~�X                          [n-s] (not used)

		/// �ԗֈʒu�̘H�ʏ��	
		/// 12�֕�(0=�O��,1=�O�E,2=�㍶,3=��E,4=��3�����O,5=��3������,6=��3���E��,7=��3���E�O,8=��4�����O,9=��4������,10=��4���E��,11=��4���E�O)
		struct ST_ROAD_INFO	{				
			float	fPos[2];				///< �� �^�C���ԗ����W(x,y)					[m]
			float	fHeight;				///< �� �H�ʍ���							[m]
			float	fNormal[3];				///< �� �H�ʖ@��(a,b,c)						[n-d]
			int		nMat;					///< �� ���m�|���S������					[n-d]
			int		nObj;					///< �� ���m�I�u�W�F�N�g����				[n-d]	(not used)
			int		nType;					 ///< �� ���m�I�u�W�F�N�g�^�C�v				[n-d]	(not used)
			int		nHit;					///< �� ���m���f���ԍ�						[n-d]	(not used)
			char	cDetect;				///< �� ���m�|���S����			   			[n-d]	(not used)
			double  dDzDx;                  ///< �� �^�C���ڒn�ʂ̌X�ΐ���X����			[n-d]
			double  dDzDy;                  ///< �� �^�C���ڒn�ʂ̌X�ΐ���Y����			[n-d]
		} stRoad[E_CAR_ROAD_SIZE];

		
		/// �^�C�����
		/// 8�֕�(0=�O��,1=�O�E,2=�㍶,3=��E,4=��3�����O,5=��3������,6=��3���E��,7=��3���E�O,8=��4�����O,9=��4������,10=��4���E��,11=��4���E�O)
		struct ST_TIRE_STATE {				
			float	fS;						///< �� �X���b�v��							[n-d]
			float	fAlpha;					///< �� �X���b�v�p							[rad]
			float	fVx;					///< �� �^�C������(X)						[m/s]
			float	fVxTC;					///< �� �H�ʑ��x(X)							[m/s]
			float	fF[3];					///< �� ��(X,Y,Z)							[N]
			float	fFTotal;				///< �� ���C��								[N]	(not used)
			float	fDelta;					///< �� �Ǌp								[rad]
		} stTire[E_CAR_TIRE_SIZE];				
		

		/// �ԗ��`��
		struct ST_SHAPE {
			float	fLength;				///< �� �S��								[m]
			float	fWidth;					///< �� �S��								[m]
			float	fHeight;				///< �� �S��								[m]	(not used)
			float	fl[E_CAR_L_SIZE];		///< �� ����(�z�C�[���x�[�X)				[m] 4����(0=����(��1����),1=��2����,2=��3����)
			float	fb[E_CAR_B_SIZE];		///< �� �֋�(�g���b�h)						[m] 4����(0=��1��,1=��2��,2=��3��,3=��4��)
			float	fRearOverhang;			///< �� ��I�[�o�[�n���O					[m]
		} stShape;

		/// �ԗ��d�S�f�[�^
		struct ST_CENTER_OF_GRAVITY {
			double	dPos[3];				///< �� �ʒu���W(X,Y,Z)						[m]
			double	dAng[3];				///< �� �p���p(��,��,��)					[rad]
			double	dPos_d[3];				///< �� ���x�x�N�g��(u,v,w)					[m/s]
			double	dAng_d[3];				///< �� �p���x�x�N�g��(p,q,r)				[rad/s]
			double	dPos_dd[3];				///< �� �����x�x�N�g��(du/dt,dv/dt,dw/dt)	[m/s2]
			double	dAng_dd[3];				///< �� �p�����x�x�N�g��(dp/dt,dq/dt,dr/dt)	[rad/s2]
			double  dBeta;                  ///< �� ������p                            [rad]
		} stCG;

	} stInfo;

} ST_CAR_DATA;

//-----------------------------------------------------------------------------
/** @brief  ���ԗ��g���f�[�^�\����
 *
 */
//-----------------------------------------------------------------------------
typedef struct {

    /// �ԗ����͊g���p�f�[�^
    struct ST_CABINE_IN {
        char  cInSpare[E_CABIN_C_SPR];   ///< �ԗ����͊g���p�f�[�^(char�^)[n-d]
        float fInSpare[E_CABIN_F_SPR];   ///< �ԗ����͊g���p�f�[�^(float�^)[n-d]
    } stInput;

    /// �ԗ��o�͊g���p�f�[�^
    struct ST_CABIN_OUT {
        char  cOutSpare[E_CABOUT_C_SPR]; ///< �ԗ��o�͊g���p�f�[�^(char�^)[n-d]
        float fOutSpare[E_CABOUT_F_SPR]; ///< �ԗ��o�͊g���p�f�[�^(float�^)[n-d]
    } stOutput;

    /// ���Ԋg���p�f�[�^
    struct ST_CAR_INFO {
        int   nCarSpare[E_CAR_SPR];      ///< ���Ԋg���p�f�[�^(int�^)[n-d]
        float fCarSpare[E_CAR_SPR];      ///< ���Ԋg���p�f�[�^(float�^)[n-d]
    } stInfo;

} ST_CAR_DATA_EX;

//-----------------------------------------------------------------------------
/** @brief  ���ԗ��f�[�^��M�p�\����
 *
 *  @note
 *
 */
//-----------------------------------------------------------------------------
///#pragma pack(push, 1)
typedef struct {

	short sFps;              ///< ��{���s���g��
	short hStrMode;          ///< �X�e�A�����O���[�h(1:�Q�[���X�e�A�����O�Œ�)
	long  lCycle;            ///< ���s�T�C�N����J�E���g
	long  lFrame;            ///< ���s�t���[����J�E���g
	long  lTotalFrame;       ///< �ʎZ�t���[����J�E���g
	double  dTime;           ///< ���s�J�n����̌o�ߎ���
	char  cStatus;           ///< �X�e�[�^�X0:��~ 5:�������� 10:���������� 15:���s�� 20:�ꎞ��~ 25:��~������
	char  cRecPbkMode;       ///< �L�^�Đ����[�h (0:�L�^���Ȃ� 1:�L�^���� 10:�Đ�)
	char  cInitReq;          ///< ���������N�G�X�g
	char  cReposReq;         ///< ���|�W�V�������N�G�X�g(1:���Ԑ��� 2:HAT 3:DSPACE etc)
	char  cTermReq;          ///< �I���������N�G�X�g

	ST_CAR_DATA	stCar;   ///< ���L���������ԃf�[�^

} NCI_ST_CAR_DATA;
///#pragma pack(pop)

//-----------------------------------------------------------------------------
/** @brief  ���ԗ��g���f�[�^��M�p�\����
 *
 *  @note
 *
 */
//-----------------------------------------------------------------------------
///#pragma pack(push, 1)
typedef struct {

	ST_CAR_DATA_EX	stCarEx;   ///< ���L���������Ԋg���f�[�^

} NCI_ST_CAR_DATA_EX; // ST_CAR_DATA_EX;
///#pragma pack(pop)

//-----------------------------------------------------------------------------
/** @brief  MPC control vehicle state data
 *
 *  @note
 *
 */
 //-----------------------------------------------------------------------------
typedef struct {
	long lTotalFrame;
	double dtime;
	double fSpeed;
	double dPos[3]; 
	double dAng[3];
	double yawRate;
	double dBeta;
	float fStrBeta;
}CAR_STATE;

#endif // NCI_CAR_DATA_H

// End Of File

