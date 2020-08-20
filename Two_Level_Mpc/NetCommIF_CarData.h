//=============================================================================
/** @file
 *  @brief NetComm：車両データ受信用構造体定義 (送信データは固定長)
 *
 *
 *
 *  @author Hiroki TODA
 *
 *  @par リビジョン情報 
 *  @n $Revision$
 *  @n $Date$
 *  @n $Author$
 *  @n Copyright (c) 2008 MITSUBISHI PRECISION Co.,Ltd. All Rights Reserved.
 */
//=============================================================================
#ifndef NCI_CAR_DATA_H
#define NCI_CAR_DATA_H


/// ATシフトレバー位置
enum eCarShiftPos {
	E_SHIFT_None	= (-99),		// なし
	E_SHIFT_P		= (-9),			// P レンジ(AT)
	E_SHIFT_R		= (-1),			// R レンジ(AT,MT)
	E_SHIFT_N		= (0),			// N レンジ(AT,MT)
	E_SHIFT_M		= (10),			// M(S) レンジ(AT)
	E_SHIFT_D		= (9),			// D レンジ(AT)
	E_SHIFT_8		= (8),			// 8 レンジ(AT,MT)
	E_SHIFT_7		= (7),			// 7 レンジ(AT,MT)
	E_SHIFT_6		= (6),			// 6 レンジ(AT,MT)
	E_SHIFT_5		= (5),			// 5 レンジ(AT,MT)
	E_SHIFT_4		= (4),			// 4 レンジ(AT,MT)
	E_SHIFT_3		= (3),			// 3 レンジ(AT,MT)
	E_SHIFT_2		= (2),			// 2 レンジ(AT,MT)
	E_SHIFT_1		= (1)			// 1 レンジ(AT,MT)
};


/// ライトスイッチ位置 (cLightSw)
enum eCarLightSw {
	E_LIGHT_SW_OFF		= (0),		// ライト OFF
	E_LIGHT_SW_SMALL	= (1),		// ライト SMALL
	E_LIGHT_SW_ON		= (2),		// ライト ON
	E_LIGHT_SW_AUTO	    = (9)		// ライト AUTO
};


/// イグニッションキー位置 (cKeyPos)
enum eCarKeyPos {
	E_KEY_LOCK		= (0),			///< イグニッションキー LOCK
	E_KEY_OFF		= (1),			///< イグニッションキー OFF
	E_KEY_ACC		= (2),			///< イグニッションキー ACC
	E_KEY_ON		= (3),			///< イグニッションキー ON
	E_KEY_START		= (4)			///< イグニッションキー START
};


/// ワイパースイッチ位置 (cCtrlWiperPos)
enum eCarWiper {
	E_WIPER_OFF = (0),				///< ワイパー OFF
	E_WIPER_INT	= (1),				///< ワイパー INT
	E_WIPER_L   = (2),				///< ワイパー LOW
	E_WIPER_H	= (3)				///< ワイパー HIGH
};

/// ライト類点灯状態 (hLights)
enum eCarLight {
	E_LIGHTS_OFF	= (0x0000),		///< ライト類 全OFF
	E_LIGHTS_POS	= (0x0001),		///< ライト類 ポジションランプ
	E_LIGHTS_HEAD	= (0x0002),		///< ライト類 ヘッドランプ
	E_LIGHTS_UPPER	= (0x0004),		///< ライト類 ヘッドランプ走行ビーム
	E_LIGHTS_FOG	= (0x0008),		///< ライト類 フォグランプ
	E_LIGHTS_LEFT	= (0x0010),		///< ライト類 ターンシグナル左
	E_LIGHTS_RIGHT	= (0x0020),		///< ライト類 ターンシグナル右
	E_LIGHTS_STOP	= (0x0100),		///< ライト類 ストップランプ
	E_LIGHTS_BACK	= (0x0200),		///< ライト類 バックアップランプ
	E_LIGHTS_SPR1	= (0x1000),		///< ライト類 予備1
	E_LIGHTS_SPR2	= (0x2000),		///< ライト類 予備2
	E_LIGHTS_SPR3	= (0x4000),		///< ライト類 予備3
	E_LIGHTS_SPR4	= (0x8000)		///< ライト類 予備4
};

/// トランスミッション(cTransmission)
enum eCarTransmission {
	E_TRANSMISSION_4AT,
	E_TRANSMISSION_5AT,
	E_TRANSMISSION_5MT,
	E_TRANSMISSION_6MT,
	E_TRANSMISSION_7MT,
	E_TRANSMISSION_SEMI_AT
};

/// 配列数
enum eCarArraySize {
	E_CABO_WRN_SPR			= (16),	// 運転席出力データ: 警告灯: 予備
	E_CAR_ROAD_SIZE			= (12),	// 路面情報 ・・ 8輪分(0=前左,1=前右,2=後左,3=後右,4=第3軸左外,5=第3軸左内,6=第3軸右内,7=第3軸右外,8=第4軸左外,9=第4軸左内,10=第4軸右内,11=第4軸右外)
	E_CAR_TIRE_SIZE			= (12),	// タイヤ状態 ・・ 8輪分(0=前左,1=前右,2=後左,3=後右,4=第3軸左外,5=第3軸左内,6=第3軸右内,7=第3軸右外,8=第4軸左外,9=第4軸左内,10=第4軸右内,11=第4軸右外)
	E_CAR_L_SIZE			= (3),	// 車両形状: 軸距 ・・ 4軸間(0=軸距(第1軸距),1=第2軸距,3=第3軸距)
	E_CAR_B_SIZE			= (6)	// 車両形状: 輪距 ・・ 4軸分(0=第1軸,1=第2軸,2=第3軸内,3=第3軸外,4=第4軸内,5=第4軸外)
};

/// 配列数
enum eCarExArraySize {
    E_CABIN_C_SPR        = (128),   // 車両入力拡張用データ数
    E_CABIN_F_SPR        = (128),   // 車両入力拡張用データ数
    E_CABOUT_C_SPR       = (128),   // 車両出力拡張用データ数
    E_CABOUT_F_SPR       = (128),   // 車両出力拡張用データ数
    E_CAR_SPR            = (128)    // 自車拡張用データ数
};


//-----------------------------------------------------------------------------
/** @brief  自車両データ構造体
 *
 */
//-----------------------------------------------------------------------------
typedef struct {

	// 初期化データ
	struct ST_INIT {
		float fPosX;						///< → 初期位置座標 X						[m]
		float fPosY;						///< → 初期位置座標 Y						[m]
		float fPosZ;						///< → 初期位置座標 Z						[m]
		float fHeading;						///< → 初期位置方位角 H					[deg]
		float fPitch;						///< → 初期位置方位角 P					[deg]
		float fRoll;						///< → 初期位置方位角 R					[deg]
		float fDbHeading;					///< → データベース座標系 初期位置方位角 H	[deg]
		float fDbPitch;						///< → データベース座標系 初期位置方位角 P	[deg]
		float fDbRoll;						///< → データベース座標系 初期位置方位角 R	[deg]
		int   nCPId;						///< → 初期CP ID							[n-d]
		int   nBodyId;						///< → 初期ボディID						[n-d]
	} stInit;

	// 衝突データ
	struct ST_COLLISION {
		bool  bRdlimXi;						///< ← はみ出し X 軸方向					[n-d]
		bool  bRdlimXd;						///< ← はみ出し X 軸方向の反対				[n-d]
		bool  bRdlimYi;						///< ← はみ出し Y 軸方向					[n-d]
		bool  bRdlimYd;						///< ← はみ出し Y 軸方向の反対				[n-d]
		bool  bCollision;					///< ← 衝突								[n-d]
		short hId;                          ///< ← 衝突車両ID(-1:衝突なし)				[n-d]
	} stCollision;

	/// 運転席入力データ
	struct ST_CABINE_IN {

		// ステアリング入力
		float	fStrAngle;					///< → 操舵角(CCW+)						[deg]
		float	fStrTrq;					///< → 操舵トルク(CCW+)					[N・m]
		
		// 運転席 I/O 入力
		float	fAccel;						///< → アクセルストローク					[n-d]
		float	fBrake;						///< → フットブレーキストローク			[n-d]
		bool	bBrakeSw;					///< → フットブレーキスイッチ 	    		[n-d]
		float	fClutch;					///< → クラッチストローク					[n-d]
		char	cShiftPos;					///< → シフトポジション					[n-d]
		char	cShiftPosS;					///< → シフトポジション(シーケンシャル用)	[n-d]
		bool	bShiftOdOFF;				///< → シフトポジション(OD OFF)			[n-d]
		bool	bShiftCvtPlus;				///< → シフトポジション(M+)				[n-d]
		bool	bShiftCvtMinus;				///< → シフトポジション(M-)				[n-d]
		char	cLightSw;					///< → ライトスイッチ位置					[n-d]
		bool	bLightUpper;				///< → ライトスイッチ(アッパービーム)		[n-d]
		bool	bLightPass;					///< → ライトスイッチ(PASS)				[n-d]
		bool	bLightFog;					///< → ライトスイッチ(Fog)					[n-d]
		bool	bWinkerLeft;				///< → ウィンカー (Left)					[n-d]
		bool	bWinkerRight;				///< → ウィンカー (Right)					[n-d]
		bool	bHazard;					///< → Hazard								[n-d]
		char	cKeyPos;					///< → イグニッションキー位置				[n-d]
		float	fPrkBrake;  				///< → パーキングブレーキストローク		[n-d]
		bool	bParkingSw;					///< → パーキングブレーキスイッチ			[n-d]
		bool	bExhBrakeSw;				///< → 排気ブレーキスイッチ				[n-d]
		bool	bCtrlHornSw;				///< → その他操作系(ホーンスイッチ)		[n-d]
		bool	bCtrlWasherSw;				///< → その他操作系(ウォッシャ)			[n-d]
		char	cCtrlWiperPos;				///< → その他操作系(ワイパスイッチ位置)	[n-d]
		char	cCtrlWiperInt;				///< → その他操作系(ワイパーINT間隔)		[n-d]
		bool	bSafeEmerStop;				///< → 安全関連(モーション緊急停止)		[n-d]
		bool	bSafeDoorLock;				///< → 安全関連(ドアロック)				[n-d]
		bool	bSafeDoorOpen;				///< → 安全関連(ドア開放)					[n-d]
		bool	bSafeSheatBelt;				///< → 安全関連(シートベルト				[n-d]

	} stInput;

	
	/// 運転席出力データ
	struct ST_CABIN_OUT {

		// ステアリング出力
		short	hStrMode;					///< ← 制御モード							[n-d]
		float	fStrLimitRight;				///< ← 操舵角リミット(右)					[deg]
		float	fStrLimitLeft;				///< ← 操舵角リミット(左)					[deg]
		float	fStrIs;						///< ← オーバーオール・ステアリング比		[n-d]
		float	fStrBeta;					///< ← 前軸横滑り角(左+)					[deg]
		float	fStrWtRate;					///< ← 前軸荷重係数						[%]
		float	fStrMust;					///< ← 前軸路面摩擦係数					[%]
		float	fStrCenter;					///< ← 復帰位置(CCW+)						[deg]		(not used)
		float	fStrTorqueSlope;			///< ← 復帰トルク傾斜						[N・m/rad]	(not used)
		float	fStrTorque;					///< ← トルク・コマンド(CCW+)				[N・m]		(not used)	

		// 運転席 I/O 出力
		float	fSpeed;						///< ← スピードメーター					[km/h]
		float	fRpm;						///< ← エンジン回転数						[rpm]
		float	fTemperature;				///< ← 水温計								[n-d]
		float	fFuel;						///< ← 燃料計								[n-d]
		bool	bIndWinkerLeft;				///< ← インジケータ(ターンシグナル左)		[n-d]
		bool	bIndWinkerRight;			///< ← インジケータ(ターンシグナル右)		[n-d]
		bool	bIndUpperBeam;				///< ← インジケータ(アッパビーム)			[n-d]
		char	cIndShiftPos;				///< ← インジケータ(シフトポジション)		[n-d]
		char	cIndShiftPosS;				///< ← インジケータ(シーケンシャルシフト)	[n-d]
		bool	bIndExhBrake;				///< ← インジケータ(エキゾーストブレーキ)	[n-d]
		bool	bWarnAirBag;				///< ← 警告灯(エアバッグ)					[n-d]
		bool	bWarnDoor;					///< ← 警告灯(Door)						[n-d]
		bool	bWarnOdOff;					///< ← 警告灯(O/D OFF)						[n-d]
		bool	bWarnPrkBrake;				///< ← 警告灯(PKB)							[n-d]
		bool	bWarnBrake;					///< ← 警告灯(ブレーキ)					[n-d]
		bool	bWarnSeatBelt;				///< ← 警告灯(シートベルト)				[n-d]
		bool	bWarnAbs;					///< ← 警告灯(ABS)							[n-d]
		bool	bWarnEngine;				///< ← 警告灯(エンジン)					[n-d]
		bool	bWarnOil;					///< ← 警告灯(OIL)							[n-d]
		bool	bWarnBattery;				///< ← 警告灯(バッテリー)					[n-d]
		bool	bWarnFuel;					///< ← 警告灯(燃料)						[n-d]
		bool	bWarnExhTemp;				///< ← 警告灯(排気温)						[n-d]
		bool	bWarnSpare[E_CABO_WRN_SPR]; ///< ← 警告灯(予備)						[n-d]
		bool	bIllmInpane;				///< ← 照明(計器板)						[n-d]
		bool	bSafeDoorLock;				///< ← 安全関連(ドアロック)				[n-d]
		bool	bShiftUnlock;				///< ← シフトレバーロック解除				[n-d]
		bool	bBackupBuzzer;				///< ← バックブザー						[n-d]
		bool	bHorn;						///< ← ホーン								[n-d]
	} stOutput;
	

	/// 自車データ
	struct ST_CAR_INFO {
		/// 車両状態
		short	hId;						///< → 車両ID								[n-d]
		int		nBodyId;					///< → BODY ID								[n-d]
		int		nCPId;						///< → CP ID								[n-d]
		double	dPos[3];					///< ← 位置座標(X,Y,Z)						[m]
		double	dAng[3];					///< ← ISO座標系 姿勢角(φ,θ,ψ)	        [rad]
		double	dDbAng[3];					///< ← データベース座標系 姿勢角(φ,θ,ψ)	[rad]
		double	dPos_d[3];					///< ← 速度ベクトル(u,v,w)					[m/s]
		double	dAng_d[3];					///< ← 角速度ベクトル(p,q,r)				[rad/s]
		double	dPos_dd[3];					///< ← 加速度ベクトル(du/dt,dv/dt,dw/dt)	[m/s2]
		double	dAng_dd[3];					///< ← 角加速度ベクトル(dp/dt,dq/dt,dr/dt) [rad/s2]
		float	fTurnRate;					///< ← 旋回率(d方位/dt)					[rad/s] (not used)
		float	fSpeed;						///< ← 速度								[m/s]
		float	fEngineN;					///< ← エンジン回転数						[rad/s]
		float	fEngineTorque;				///< ← エンジン出力トルク					[N・m]
		bool	bEngineState;				///< ← エンジン状態						[n-d]
		short	hGearPos;					///< ← 変速機ギア位置						[n-d]
		short	hLights;					///< ← ライト類点灯状態					[n-d]
		char    cTransmission;              ///< ← トランスミッション                  [n-d]   (not used)
		double	dDistanceCovered;			///< → 自車走行距離						[m]    
		bool    bCourseMiss;                ///< → コースミス                          [n-s] (not used)

		/// 車輪位置の路面情報	
		/// 12輪分(0=前左,1=前右,2=後左,3=後右,4=第3軸左外,5=第3軸左内,6=第3軸右内,7=第3軸右外,8=第4軸左外,9=第4軸左内,10=第4軸右内,11=第4軸右外)
		struct ST_ROAD_INFO	{				
			float	fPos[2];				///< ← タイヤ車両座標(x,y)					[m]
			float	fHeight;				///< → 路面高さ							[m]
			float	fNormal[3];				///< → 路面法線(a,b,c)						[n-d]
			int		nMat;					///< → 検知ポリゴン属性					[n-d]
			int		nObj;					///< → 検知オブジェクト属性				[n-d]	(not used)
			int		nType;					 ///< → 検知オブジェクトタイプ				[n-d]	(not used)
			int		nHit;					///< → 検知モデル番号						[n-d]	(not used)
			char	cDetect;				///< → 検知ポリゴン数			   			[n-d]	(not used)
			double  dDzDx;                  ///< → タイヤ接地面の傾斜正接X方向			[n-d]
			double  dDzDy;                  ///< → タイヤ接地面の傾斜正接Y方向			[n-d]
		} stRoad[E_CAR_ROAD_SIZE];

		
		/// タイヤ状態
		/// 8輪分(0=前左,1=前右,2=後左,3=後右,4=第3軸左外,5=第3軸左内,6=第3軸右内,7=第3軸右外,8=第4軸左外,9=第4軸左内,10=第4軸右内,11=第4軸右外)
		struct ST_TIRE_STATE {				
			float	fS;						///< ← スリップ比							[n-d]
			float	fAlpha;					///< ← スリップ角							[rad]
			float	fVx;					///< ← タイヤ周速(X)						[m/s]
			float	fVxTC;					///< ← 路面速度(X)							[m/s]
			float	fF[3];					///< ← 力(X,Y,Z)							[N]
			float	fFTotal;				///< ← 摩擦力								[N]	(not used)
			float	fDelta;					///< ← 舵角								[rad]
		} stTire[E_CAR_TIRE_SIZE];				
		

		/// 車両形状
		struct ST_SHAPE {
			float	fLength;				///< ← 全長								[m]
			float	fWidth;					///< ← 全幅								[m]
			float	fHeight;				///< ← 全高								[m]	(not used)
			float	fl[E_CAR_L_SIZE];		///< ← 軸距(ホイールベース)				[m] 4軸間(0=軸距(第1軸距),1=第2軸距,2=第3軸距)
			float	fb[E_CAR_B_SIZE];		///< ← 輪距(トレッド)						[m] 4軸分(0=第1軸,1=第2軸,2=第3軸,3=第4軸)
			float	fRearOverhang;			///< ← 後オーバーハング					[m]
		} stShape;

		/// 車両重心データ
		struct ST_CENTER_OF_GRAVITY {
			double	dPos[3];				///< ← 位置座標(X,Y,Z)						[m]
			double	dAng[3];				///< ← 姿勢角(φ,θ,ψ)					[rad]
			double	dPos_d[3];				///< ← 速度ベクトル(u,v,w)					[m/s]
			double	dAng_d[3];				///< ← 角速度ベクトル(p,q,r)				[rad/s]
			double	dPos_dd[3];				///< ← 加速度ベクトル(du/dt,dv/dt,dw/dt)	[m/s2]
			double	dAng_dd[3];				///< ← 角加速度ベクトル(dp/dt,dq/dt,dr/dt)	[rad/s2]
			double  dBeta;                  ///< ← 横滑り角                            [rad]
		} stCG;

	} stInfo;

} ST_CAR_DATA;

//-----------------------------------------------------------------------------
/** @brief  自車両拡張データ構造体
 *
 */
//-----------------------------------------------------------------------------
typedef struct {

    /// 車両入力拡張用データ
    struct ST_CABINE_IN {
        char  cInSpare[E_CABIN_C_SPR];   ///< 車両入力拡張用データ(char型)[n-d]
        float fInSpare[E_CABIN_F_SPR];   ///< 車両入力拡張用データ(float型)[n-d]
    } stInput;

    /// 車両出力拡張用データ
    struct ST_CABIN_OUT {
        char  cOutSpare[E_CABOUT_C_SPR]; ///< 車両出力拡張用データ(char型)[n-d]
        float fOutSpare[E_CABOUT_F_SPR]; ///< 車両出力拡張用データ(float型)[n-d]
    } stOutput;

    /// 自車拡張用データ
    struct ST_CAR_INFO {
        int   nCarSpare[E_CAR_SPR];      ///< 自車拡張用データ(int型)[n-d]
        float fCarSpare[E_CAR_SPR];      ///< 自車拡張用データ(float型)[n-d]
    } stInfo;

} ST_CAR_DATA_EX;

//-----------------------------------------------------------------------------
/** @brief  自車両データ受信用構造体
 *
 *  @note
 *
 */
//-----------------------------------------------------------------------------
///#pragma pack(push, 1)
typedef struct {

	short sFps;              ///< 基本実行周波数
	short hStrMode;          ///< ステアリングモード(1:ゲームステアリング固定)
	long  lCycle;            ///< 実行サイクル･カウント
	long  lFrame;            ///< 実行フレーム･カウント
	long  lTotalFrame;       ///< 通算フレーム･カウント
	double  dTime;           ///< 実行開始からの経過時間
	char  cStatus;           ///< ステータス0:停止 5:初期化中 10:初期化完了 15:実行中 20:一時停止 25:停止処理中
	char  cRecPbkMode;       ///< 記録再生モード (0:記録しない 1:記録する 10:再生)
	char  cInitReq;          ///< 初期化リクエスト
	char  cReposReq;         ///< リポジションリクエスト(1:他車制御 2:HAT 3:DSPACE etc)
	char  cTermReq;          ///< 終了処理リクエスト

	ST_CAR_DATA	stCar;   ///< 共有メモリ自車データ

} NCI_ST_CAR_DATA;
///#pragma pack(pop)

//-----------------------------------------------------------------------------
/** @brief  自車両拡張データ受信用構造体
 *
 *  @note
 *
 */
//-----------------------------------------------------------------------------
///#pragma pack(push, 1)
typedef struct {

	ST_CAR_DATA_EX	stCarEx;   ///< 共有メモリ自車拡張データ

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

