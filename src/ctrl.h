/************************************************************************/
/*  ファイル名  :ctrl.h													*/
/*  H/W			:Pi:Co(SH7125)											*/
/*  作成日      :2012/04/15												*/
/*	作成者		:T.Kusumi												*/
/*	プロジェクト:マイクロマウスで楽しく組込入門（最終ステップ）			*/
/*	モジュール	:コントローラ用ヘッダーファイル							*/
/************************************************************************/
void ctrl_led_on(void);								//LED全点灯
void ctrl_led_off(void);							//LED全消灯
void ctrl_led_r(int);								//LED処理（右）
void ctrl_led_l(int);								//LED処理（左）
void ctrl_led_fr(int);								//LED処理（前右）
void ctrl_led_fl(int);								//LED処理（前左）
void ctrl_led_f(int);								//前2個LED処理
void ctrl_led_num(int);								//LEDで2進数表示
unsigned char ctrl_sw_b(void);						//スイッチ処理（青）
unsigned char ctrl_sw_y(void);						//スイッチ処理（黄）
unsigned char ctrl_sw_r(void);						//スイッチ処理（赤）
void ctrl_sw_all(unsigned char *,					//スイッチ処理（一括）
	 			 unsigned char *,
				 unsigned char *);
void ctrl_bz(int, int);								//ブザー処理
void ctrl_bz_melody(int, int, int);					//ブザー処理（メロディ）
//外部モジュール
extern void drv_led_r(int);							//LEDドライバ（右）
extern void drv_led_l(int);							//LEDドライバ（左）
extern void drv_led_fr(int);						//LEDドライバ（前右）
extern void drv_led_fl(int);						//LEDドライバ（前左）
extern unsigned char drv_sw_b(void);				//スイッチドライバ（青）
extern unsigned char drv_sw_y(void);				//スイッチドライバ（黄）
extern unsigned char drv_sw_r(void);				//スイッチドライバ（赤）
extern void drv_bz(int, int);						//ブザードライバ
