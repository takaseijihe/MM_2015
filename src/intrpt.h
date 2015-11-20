/*
 * interrupt.h
 *
 *  Created on: 2015/09/06
 *      Author: 崇伸
 */
void int_cmt0(void);								//タイマー割込み(1ms)
//外部変数
extern volatile int	G_TimerCount;					//1mSごとにカウントアップされる変数
//割込みフラグ
#define CMT0_INT_F		CMT0.CMCSR.BIT.CMF			//タイマー0割込みフラグ


