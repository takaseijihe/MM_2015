/*
 * interrupt.c
 *
 *  Created on: 2015/09/06
 *      Author: 崇伸
 */

#include	"iodefine.h"
#include	"common.h"
#include	"intrpt.h"

/************************************************/
/*	CMT0割込み処理					(int_cmt0)	*/
/************************************************/
/*	タイマー割込み(1ms)							*/
/************************************************/
void int_cmt0(void)
{
	PE.DRL.BIT.B7 =1;
	//タイマー割込み(1ms)
	CMT0_INT_F = 0;			//フラグクリア
	//タイマーカウント
	G_TimerCount++;			//1msごとにカウントアップ
}
