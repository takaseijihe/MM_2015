/*
 * init.c
 *
 *  Created on: 2015/09/06
 *      Author: 崇伸
 */


#include	"iodefine.h"
#include	"common.h"
#include	"init.h"

/************************************************/
/*	クロック初期化				(init_clock)	*/
/************************************************/
/*	CPUクロックを設定する。(48MHz)				*/
/************************************************/
void initIO(void)//ポートの入出力設定
{
	PFC.PEIORL.WORD = 0xabff;//abff
	PFC.PAIORL.WORD = 0xffff;
	PFC.PBIORL.WORD = 0xffff;


	PFC.PEIORL.BIT.B7 = 1;//ブザーPE8出力設定
	PFC.PECRL2.BIT.PE7MD = 1;//PE8をMTU端子に設定PECRL3レジスタ
//	PFC.PEIORL.BIT.B10=0;
	PFC.PEIORL.BIT.B12=0;
//	PFC.PEIORL.BIT.B14=0;
}
void initAD(void)
{
    STB.CR4.BIT._AD0     = 0;       //AD0のスタンバイを解除
    STB.CR4.BIT._AD1     = 0;       //AD1のスタンバイを解除
    //
    AD0.ADCR.BIT.ADST    = 0;       //AD停止
    AD0.ADCSR.BIT.ADF    = 0;       //ADFフラグクリア
    AD0.ADCSR.BIT.ADIE   = 0;       //割込み禁止
    AD0.ADCSR.BIT.TRGE   = 0;       //トリガイネーブル無効
    AD0.ADCSR.BIT.CONADF = 0;       //ADFコントロール（独立ADF）
    AD0.ADCSR.BIT.STC    = 0;       //ステートコントロール（50ステート）
    AD0.ADCSR.BIT.CKSL   = 0;       //周辺動作クロックPφ
    AD0.ADCSR.BIT.ADM    = 0;       //シングルスキャン
    AD0.ADCSR.BIT.ADCS   = 0;       //サイクルスキャンしない
    AD0.ADCSR.BIT.CH     = 1;       //AD0-ch0選択
    //
    AD1.ADCR.BIT.ADST    = 0;       //AD停止
    AD1.ADCSR.BIT.ADF    = 0;       //ADFフラグクリア
    AD1.ADCSR.BIT.ADIE   = 0;       //割込み禁止
    AD1.ADCSR.BIT.TRGE   = 0;       //トリガイネーブル無効
    AD1.ADCSR.BIT.CONADF = 0;       //ADFコントロール（独立ADF）
    AD1.ADCSR.BIT.STC    = 0;       //ステートコントロール（50ステート）
    AD1.ADCSR.BIT.CKSL   = 0;       //周辺動作クロックPφ
    AD1.ADCSR.BIT.ADM    = 0;       //シングルスキャン
    AD1.ADCSR.BIT.ADCS   = 0;       //サイクルスキャンしない
    AD1.ADCSR.BIT.CH     = 1;       //AD1-ch0選択
}
void init_clock(void)
{
	//クロック初期化
	int i;
	//
	CPG.FRQCR.BIT.IFC = 1;				//CPUクロックをX'talの周波数×４に設定する(初期値は×２)
	for(i = 0; i < 0xfffff; i++)	;	//PLLの動作が安定するまで待つ
}

/************************************************/
/*	I/Oポート初期化					(init_io)	*/
/************************************************/
/*	ポートを初期化する。						*/
/************************************************/
/*void init_io(void)
{
	//スイッチ
	PFC.PBIORL.BIT.B1		= 0;	//PB1を入力に設定（右）
	PFC.PBIORL.BIT.B2		= 0;	//PB2を入力に設定（中）
	PFC.PBIORL.BIT.B3		= 0;	//PB3を入力に設定（左）
	//LED
	PFC.PEIORL.BIT.B0		= 1;	//PE0を出力に設定（左）
	PFC.PEIORL.BIT.B1		= 1;	//PE1を出力に設定（前左）
	PFC.PEIORL.BIT.B2		= 1;	//PE2を出力に設定（前右）
	PFC.PEIORL.BIT.B3		= 1;	//PE3を出力に設定（右）
	//ブザー
	PFC.PEIORL.BIT.B6		= 1;	//PE6を出力に設定
	//ブザーMTU
	PFC.PECRL2.BIT.PE6MD 	= 1;	//PE6をMTU端子に設定
}
*/
/************************************************/
/*	CMT初期化						(init_cmt)	*/
/************************************************/
/*	CMTを初期設定する。							*/
/************************************************/
/*void init_cmt(void)
{
	//CMTスタンバイモード解除
	STB.CR4.BIT._CMT = 0;
	//
	CMT.CMSTR.BIT.STR0 = 0;			//CMT0カウント停止
	//CMT0は制御用タイマー
	CMT0.CMCSR.BIT.CMIE = 1;		//割り込みを許可
	CMT0.CMCSR.BIT.CKS = 0;			//カウントクロックPφ/8 = 3MHz
	CMT0.CMCSR.BIT.CMF = 0;			//フラグクリア
	//CMT0.CMCOR = 3000-1;			//1msごとに割り込み
	CMT0.CMCOR = 0x02cc-0x0001;
	//
	INTC.IPRJ.BIT._CMT0 = 0x0f;		//割り込み優先度を最高に設定
	//

}*/

void init_mtu_motor(void){//9.4.5p.239を見ながら順番に書く

	STB.CR4.BIT._MTU2 = 0;			//MTUスタンバイ解除
	MTU2.TSTR.BYTE = 0; 			//カウンタ動作停止

	//MTU20.TCNT = 0;
	MTU20.TIER.BIT.TGIEB = 1;//TGRBコンペアマッチによる割り込みを許可 タイマーインタラプトエネイブル
	INTC.IPRD.BIT._MTU20G = 0x0f;	//割り込み優先度を高めに設定15
	MTU20.TCR.BIT.TPSC = 1; 		// プリスケーラ MPφ (MPφ=25MHz):4 1562500/s
	MTU20.TCR.BIT.CKEG = 0;
	MTU20.TCR.BIT.CCLR = 1; 		// TGRAコンペアマッチでTCNTクリアp.170
	MTU20.TIOR.BIT.IOA = 1; 		// 初期状態:0 コンペアマッチ:0,TGRAに達した時に状態が0になる
	MTU20.TIOR.BIT.IOB = 2;			//TGRBに達した時に状態が1になる
	MTU20.TGRA = 5000;				//てきとーな値
	MTU20.TGRB = 99;				//パルス幅4uS
	MTU20.TMDR.BIT.MD = 3; 			// PWM2モードp.165
	PFC.PECRL1.BIT.PE1MD = 1;		//MTU用端子に設定 MTUのタイマ割り込みに設定
	PFC.PEIORL.BIT.B1 = 1;			//設定

	//MTU21.TCNT = 0;
	MTU21.TIER.BIT.TGIEB = 1;
	INTC.IPRD.BIT._MTU21G = 0x0d;	//割り込み優先度を高めに設定14
	MTU21.TCR.BIT.TPSC = 1; 		// カウンタクロックの設定プリスケーラ MPφ (MPφ=25MHz):p.171
	MTU21.TCR.BIT.CKEG = 0;
	MTU21.TCR.BIT.CCLR = 1; 		// TGRAコンペアマッチでTCNTクリア
	MTU21.TIOR.BIT.IOA = 1;			// 初期状態:0 コンペアマッチ:0
	MTU21.TIOR.BIT.IOB = 2;
	MTU21.TGRA = 5000;			//てきとーな値
	MTU21.TGRB = 99;				//パルス幅4uS
	MTU21.TMDR.BIT.MD = 3; 			// PWM2モードp.165
	PFC.PECRL2.BIT.PE5MD = 1;		//MTU用端子に設定
	PFC.PEIORL.BIT.B5 = 1;			//設定一応つけたし　山本さん書いてたから










//	MTU20.TMDR.BIT.MD = 2; 			// PWM1モード

//	MTU21.TMDR.BIT.MD = 2; 			// PWM1モード
}


/*void init_mtu_speaker(void){

	STB.CR4.BIT._MTU2 = 0;

	MTU22.TMDR.BIT.MD = 4;	//位相計数モード１

	MTU21.TCNT = 30000;
	MTU22.TCNT = 30000;

	MTU2.TSTR.BIT.CST2 = 1;	//タイマ2カウントスタート

}
/************************************************/
/*	MTU初期化						(init_mtu)	*/
/************************************************/
/*	MTUを初期設定する。							*/
/*	MTU22:ブザー用								*/
/************************************************/
/*void init_mtu(void)
{
	MTU2.TSTR.BYTE = 0;
	//MTU2スタンバイモード解除
	STB.CR4.BIT._MTU2 = 0;
	//
	MTU2.TSTR.BYTE=0;//タイマ動作ストップ

	//ブザー用MTU
	MTU22.TCR.BIT.TPSC = 0;			//MTU22の動作クロックは24MHz
	MTU22.TCR.BIT.CCLR = 2;			//TGRBコンペアマッチでカウンタクリア
	MTU22.TIOR.BIT.IOB = 1;			//初期出力0コンペアマッチ0出力
	MTU22.TIOR.BIT.IOA = 2;			//初期出力0コンペアマッチ1出力
	MTU22.TGRA = 6000;
	MTU22.TGRB = 12000;				//発振周波数2kHz
	MTU22.TMDR.BIT.MD = 2;			//PWMモード1に設定
	//
	MTU2.TOER.BYTE=0xff;			//MTU出力端子を出力許可する
	//

}*/
void initCMT(void)//CMT割込みの設定
{
	STB.CR4.BIT._CMT = 0;//CMTスタンバイ解除

	//(1)コンペアマッチタイマスタートレジスタ(CMSTR)
	CMT.CMSTR.BIT.STR0=0;//ステータスレジスタ　0:カウント停止,1:カウント開始

	//(2)コンペアマッチタイマコントロール/ステータスレジスタ(CMCSR)
	CMT0.CMCSR.BIT.CMIE = 1; //割込みイネーブル許可
	CMT0.CMCSR.BIT.CKS = 0;//1/8
	CMT0.CMCSR.BIT.CMF = 0; //フラグをクリア
	CMT0.CMCOR = 3125;//割込み周期
	INTC.IPRJ.BIT._CMT0 = 0xc;//割り込み優先度(10)
}

/************************************************/
/*	H/W初期化						(init_all)	*/
/************************************************/
/*	すべてのH/Wの初期化を行う。					*/
/************************************************/
void init_all(void)
{
	init_clock();		//CPUの動作周波数を設定
	//init_io();			//I/Oポートを設定
	//init_cmt();			//CMTを設定
	init_mtu_motor();			//MTUの初期化
	initAD();				//AD変換設定
	initCMT();				//CMT初期化２
	initIO();
}
