/**
 * @file AvrTimer.h
 * @brief AVRタイマ制御部
 * @author f.aimano
 * @date 2017/10/20
 * @see http://threesons-technicalmemo.blogspot.jp/2014/03/avr16.html
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/common.h>

#include "AvrTimer.h"
#include "MotorManager.h"

extern int getSensorPatternCalledFromTimer(void);

#define F_CPU_DIV1000	(16000L)

volatile static int32_t avr16bitMTimer = 0;	// Avrの16bitタイマの経過時間(msec)

/**
 * AVRタイマ開始
 * @brief AVRタイマ開始
 */
void AvrTimerStart() {
	avr16bitMTimer = 0;
 
	//割り込み禁止
	cli();
 
	//ﾀｲﾏ/ｶｳﾝﾀ1制御ﾚｼﾞｽﾀAの設定
	//（タイマ用ピンとして使わない）
	TCCR1A = 0x00; //タイマ関係のピンは標準ポート動作とする
 
	//ﾀｲﾏ/ｶｳﾝﾀ1制御ﾚｼﾞｽﾀBの設定
	TCCR1B = (1<<CS12)|(0<<CS11)|(1<<CS10);//1024分周でタイマON
 
	//ﾀｲﾏ/ｶｳﾝﾀ1割り込みﾏｽｸ ﾚｼﾞｽﾀ設定
	TIMSK1 = (1<<TOIE1);//タイマ１オーバーフロー割り込み許可
 
	//割り込み許可
	sei();
}
 
/**
 * AVRタイマ取得
 * @brief AVRタイマ取得
 */
int32_t AvrTimerGet() {
	//16ビットレジスタの読み書きの際には、テンポラリレジスタを使用する。
	//このため、割り込み禁止操作が必要。
 
	//ステータスレジスタを一時保存する変数
	uint8_t sreg;
 
	//ステータスレジスタを保存
	sreg = SREG;
 
	//割り込み禁止
	cli();
 
	//現在のタイマ値を取得
	uint16_t t = TCNT1;
 
	//SREGを戻す。これによって割り込み禁止状態が戻る。（SREGのIビットを戻すから）
	SREG = sreg;
 
	int32_t tw = (int32_t)t;
 
	return avr16bitMTimer + (tw * 1024) / F_CPU_DIV1000;
}
 
/**
 * AVRタイマ終了
 * @brief AVRタイマ終了
 */
void AvrTimerEnd() {
	uint8_t sreg;
	sreg = SREG;
	cli();
 
	//タイマを0にする
	TCNT1 = 0;
	SREG = sreg;
 
	//経過時間も0にする
	avr16bitMTimer = 0;
}
 
/**
 * AVRタイマ初期化
 * @brief AVRタイマ初期化
 */
void AvrTimerReset() {
	TCCR1B = 0;//タイマoff 
}

/**
 * TIMER1_OVF_vectのタイマ割り込み
 * @brief TIMER1_OVF_vectのタイマ割り込み
 */
ISR(TIMER1_OVF_vect) {
	avr16bitMTimer += (65536L * 1024L)/ F_CPU_DIV1000;
#ifdef ENABLE_AVRTIMER
	// センサー値のBitパターンを取得
	getSensorPatternCalledFromTimer();
	// 現在の速度(右モータ)取得
	GetCurrentSpeedRCalledFromTimer();
	// 現在の速度(左モータ)取得
	GetCurrentSpeedLCalledFromTimer();
#endif // ENABLE_AVRTIMER
}