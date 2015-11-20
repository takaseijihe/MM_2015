/************************************************************************/
/*  ファイル名  :init.h													*/
/*  H/W			:Pi:Co(SH7125)											*/
/*  作成日      :2012/04/15												*/
/*	作成者		:T.Kusumi												*/
/*	プロジェクト:マイクロマウスで楽しく組込入門							*/
/*	モジュール	:初期処理用ヘッダーファイル								*/
/************************************************************************/
void init_clock(void);				//CPUの動作周波数を設定
void init_io(void);					//IOポート初期化
void init_cmt(void);				//CMT初期化
void init_mtu(void);				//MTU設定
void initAD(void);				//AD変換設定
void initCMT(void);				//CMT初期化２
void initIO(void);
void init_mtu_motor(void);
void init_mtu_speaker(void);
