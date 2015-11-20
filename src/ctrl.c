/************************************************************************/
/*  ファイル名  :ctrl.c													*/
/*  H/W			:Pi:Co(SH7125)											*/
/*  作成日      :2012/04/15												*/
/*	作成者		:T.Kusumi												*/
/*	プロジェクト:マイクロマウスで楽しく組込入門（最終ステップ）			*/
/*	モジュール	:コントローラ											*/
/************************************************************************/
#include	"common.h"
#include	"ctrl.h"
/************************************************/
/*	LED全点灯					(ctrl_led_on)	*/
/************************************************/
/*	すべてのLEDを点灯する。						*/
/************************************************/
void ctrl_led_on(void)
{
	//LED全点灯
	drv_led_r(LED_ON);				//右点灯
	drv_led_fr(LED_ON);				//前右点灯
	drv_led_fl(LED_ON);				//前左点灯
	drv_led_l(LED_ON);				//左
}

/************************************************/
/*	LED全消灯					(ctrl_led_off)	*/
/************************************************/
/*	すべてのLEDを消灯する。						*/
/************************************************/
void ctrl_led_off(void)
{
	//LED全消灯
	drv_led_r(LED_OFF);				//右消灯
	drv_led_fr(LED_OFF);			//前右消灯
	drv_led_fl(LED_OFF);			//前左消灯
	drv_led_l(LED_OFF);				//左消灯
}

/************************************************/
/*	右LED処理					(ctrl_led_r)	*/
/************************************************/
/*	右LEDの点灯/消灯を行う。					*/
/*----------------------------------------------*/
/*	IN :int i_stat … LED_OFF(0):消灯			*/
/*					  LED_ON (1):点灯			*/
/************************************************/
void ctrl_led_r(int i_stat)
{
	//LED処理（右）
	drv_led_r(i_stat);
}

/************************************************/
/*	左LED処理					(ctrl_led_l)	*/
/************************************************/
/*	左LEDの点灯/消灯を行う。					*/
/*----------------------------------------------*/
/*	IN :int i_stat … LED_OFF(0):消灯			*/
/*					  LED_ON (1):点灯			*/
/************************************************/
void ctrl_led_l(int i_stat)
{
	//LED処理（左）
	drv_led_l(i_stat);
}

/************************************************/
/*	前右LED処理					(ctrl_led_fr)	*/
/************************************************/
/*	前右LEDの点灯/消灯を行う。					*/
/*----------------------------------------------*/
/*	IN :int i_stat … LED_OFF(0):消灯			*/
/*					  LED_ON (1):点灯			*/
/************************************************/
void ctrl_led_fr(int i_stat)
{
	//LED処理（前右）
	drv_led_fr(i_stat);
}

/************************************************/
/*	前左LED処理					(ctrl_led_fl)	*/
/************************************************/
/*	前左LEDの点灯/消灯を行う。					*/
/*----------------------------------------------*/
/*	IN :int i_stat … LED_OFF(0):消灯			*/
/*					  LED_ON (1):点灯			*/
/************************************************/
void ctrl_led_fl(int i_stat)
{
	//LED処理（前左）
	drv_led_fl(i_stat);
}

/************************************************/
/*	前2個LED処理					(ctrl_led_f)*/
/************************************************/
/*	前2個LEDの点灯/消灯を行う。					*/
/*----------------------------------------------*/
/*	IN :int i_stat … LED_OFF(0):消灯			*/
/*					  LED_ON (1):点灯			*/
/************************************************/
void ctrl_led_f(int i_stat)
{
	//前2個LED点灯
	drv_led_fr(i_stat);
	drv_led_fl(i_stat);
}

/************************************************/
/*	LED2進数表示				(ctrl_led_num)	*/
/************************************************/
/*	LEDで2進数を表示する。						*/
/*----------------------------------------------*/
/*	IN :int i_number … LEDに表示する数値		*/
/************************************************/
void ctrl_led_num(int i_number)
{
	//LEDで2進数を表示する。
	drv_led_r(i_number & 0x0001);			//1BIT…右
	drv_led_fr((i_number & 0x0002) >> 1);	//2BIT…前右
	drv_led_fl((i_number & 0x0004) >> 2);	//3BIT…前左
	drv_led_l((i_number & 0x0008) >>3);		//4BIT…左
}

/************************************************/
/*	スイッチ処理（一括）		(ctrl_sw_all)	*/
/************************************************/
/*	すべのスイッチの状態を取得して返す。		*/
/*----------------------------------------------*/
/*	OUT:unsigned char *o_sw_b … 青スイッチ		*/
/*					             ON:SW_ON(0)	*/
/*		unsigned char *o_sw_y … 黄スイッチ		*/
/*					             ON:SW_ON(0)	*/
/*		unsigned char *o_sw_r … 赤スイッチ		*/
/*					             ON:SW_ON(0)	*/
/************************************************/
void ctrl_sw_all(unsigned char *o_sw_b,
				 unsigned char *o_sw_y,
				 unsigned char *o_sw_r)
{
	//スイッチ処理（一括）
	*o_sw_b	= drv_sw_b();			//青スイッチ
	*o_sw_y	= drv_sw_y();			//黄スイッチ
	*o_sw_r	= drv_sw_r();			//赤スイッチ
	//どれかのスイッチがONならば、すべてのスイッチがOFFになるまで待つ。
	if (*o_sw_b == SW_ON || *o_sw_y == SW_ON || *o_sw_r == SW_ON)
	{
		while(drv_sw_b() == SW_ON || drv_sw_y() == SW_ON || drv_sw_r() == SW_ON);
	}
}

/************************************************/
/*	ブザー処理						(ctrl_bz)	*/
/************************************************/
/*	周波数､時間を指定してブザーを鳴らす。		*/
/*----------------------------------------------*/
/*	IN :int i_freq   … 周波数					*/
/*		int	i_ms     … 時間(ms)				*/
/************************************************/
void ctrl_bz(int i_freq, int i_ms)
{
	//ブザー処理
	drv_bz(i_freq, i_ms);						//鳴らす
}

/************************************************/
/*	ブザー処理（メロディ）	(ctrl_bz_melody)	*/
/************************************************/
/*	周波数､拍数､テンポを指定してブザーを鳴らす。*/
/*	続けて鳴らす場合に音がつながらないように	*/
/*	短時間の間隔をあける。						*/
/*----------------------------------------------*/
/*	IN :int i_freq  … 周波数					*/
/*		int i_beat  … 拍数(四分音符=2)			*/
/*		int i_tempo … テンポ(1分間の拍数)		*/
/************************************************/
void ctrl_bz_melody(int i_freq, int i_beat, int i_tempo)
{
	//ブザー処理（メロディ）
	int ms = (i_beat * 60 * 1000 / 2) / i_tempo;
	//
	drv_bz(i_freq, (int)(ms * 0.9));				//鳴らす
	drv_bz(BZ_FREQ_REST, (int)(ms * 0.1));			//間隔をあける
}