
/*  H/W			:Pi:Co(SH7125)											*/
/*  作成日      :2012/04/15												*/
/*	作成者		:T.Kusumi												*/
/*	プロジェクト:マイクロマウスで楽しく組込入門（最終ステップ）			*/
/*	モジュール	:ドライバ用ヘッダーファイル								*/
/************************************************************************/
void drv_led_r(int);								//LEDドライバ（右）
void drv_led_l(int);								//LEDドライバ（左）
void drv_led_fr(int);								//LEDドライバ（前右）
void drv_led_fl(int);	//LEDドライバ（前左）
unsigned char drv_sw_b(void);						//スイッチドライバ（青）
unsigned char drv_sw_y(void);						//スイッチドライバ（黄）
unsigned char drv_sw_r(void);						//スイッチドライバ（赤）
void drv_bz(int, int);								//ブザードライバ
void drv_wait_ms(int);								//ms単位で待ち時間を生成する
//外部変数
extern volatile int	G_TimerCount;					//1mSごとにカウントアップされる変数
//LEDポート
#define LED_L				PE.DRL.BIT.B0			//LED　左
#define LED_FL				PE.DRL.BIT.B1			//LED　前左
#define LED_FR				PE.DRL.BIT.B2			//LED　前右
#define LED_R				PE.DRL.BIT.B3			//LED　右
//スイッチポート
#define SW_BLUE				PB.DR.BIT.B1			//青スイッチ
#define SW_YELLOW			PB.DR.BIT.B2			//黄スイッチ
#define SW_RED				PB.DR.BIT.B3			//赤スイッチ
//ウェイト時間
#define CHAT_WAIT			10						//チャタリング回避用待ち時間(ms)
//ブザーMTUポート
#define MTU_BZ				MTU2.TSTR.BIT.CST2		//ブザー用MTU
//ブザーMTU発振数
#define MTU_BZ_CLOCK		24000000				//ブザー用MTUの発振数(24MHz)
//ブザーMTUレジスタ
#define MTU_BZ_CYCLE		MTU22.TGRB				//ブザー周期を設定
#define MTU_BZ_PULSWIDTH	MTU22.TGRA				//ブザーパルス幅を設定
