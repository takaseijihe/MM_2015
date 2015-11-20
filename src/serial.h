#include<stdio.h>
#include<stdarg.h>

void init_sci1(void){
	long divider;
	unsigned short baud = 38400;
	unsigned char tmp;

	STB.CR3.BIT._SCI1=0;	//STANDBY解除
	PFC.PACRL1.BIT.PA3MD=1; //シリアルポート設定
	PFC.PACRL2.BIT.PA4MD = 1;//シリアルポート設定
	SCI1.SCSCR.BYTE=0x00;//送受信割り込み禁止

	//ビットレート関連の計算
	divider = 32;
	if (baud < 300) {
		tmp = 0x03;
		divider = divider <<6;
	}else
	if(baud<1200){
		tmp = 0x02;
		divider = divider <<4;
	}else
	if (baud <4800){
		tmp = 0x01;
		divider = divider <<2;
	}else{
		tmp = 0x00;
	}

	SCI1.SCSMR.BYTE = tmp;
	tmp = (unsigned char)(25000000/divider/baud)-1;
	SCI1.SCBRR = tmp;
	SCI1.SCSCR.BIT.TE=1;
	SCI1.SCSCR.BIT.RE=1;
}
void put1byte(char c)
{
	while( SCI1.SCSSR.BIT.TDRE == 0);
	SCI1.SCSSR.BIT.TDRE = 0;
	SCI1.SCTDR = c;
}
void putnbyte(char *buf,int len){
	int c;

	for(c = 0;c<len;c++){
		put1byte(buf[c]);
	}
}
int myprintf(const char *fmt, ...)
{//可変長引数・・・データ値が違っても使える
	static char buffer[100];
	int len;

	va_list ap;//
	va_start(ap, fmt);//omajinai

	len = vsprintf(buffer, fmt, ap);//データの長さを識別
	putnbyte(buffer, len);
	va_end(ap);
	return len;
}
