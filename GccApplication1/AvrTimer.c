/**
 * @file AvrTimer.h
 * @brief AVRタイマ制御部
 * @author f.aimano
 * @date 2017/10/20
 * @see http://threesons-technicalmemo.blogspot.jp/2014/03/avr16.html
 * @detail CM-700のCPUはAtmelのATMega2561が実装されている。
 * ATMega2561のクロックは16MHz。
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/common.h>

#include "AvrTimer.h"
#include "DebugLog.h"
#include "MovingDistance.h"
#include "MotorManager.h"

extern int getSensorPatternCalledFromTimer(void);

#define F_CPU_DIV1000	(16000L)	// 16MHz
#define DIVIDE			(8L)		// 分周（ﾀｲﾏ/ｶｳﾝﾀ1制御ﾚｼﾞｽﾀBの設定と相関あり）

volatile static uint32_t avr16bitMTimer = 0;	// Avrの16bitタイマの経過時間(msec)

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
	TCCR1B = (0<<CS12)|(1<<CS11)|(0<<CS10);	// 1/8分周でタイマON
 
	//ﾀｲﾏ/ｶｳﾝﾀ1割り込みﾏｽｸ ﾚｼﾞｽﾀ設定
	TIMSK1 = (1<<TOIE1);//タイマ１オーバーフロー割り込み許可
 
	//割り込み許可
	sei();
}
 
/**
 * AVRタイマ取得
 * @brief AVRタイマ取得(ミリ秒)
 * @attention ISR(TIMER1_OVF_vect) 内でのコールは禁止。
 */
uint32_t AvrTimerGet() {
	//16ビットレジスタの読み書きの際には、テンポラリレジスタを使用する。
	//このため、割り込み禁止操作が必要。
 
	return avr16bitMTimer;
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
 * @attention 本メソッド書き処理を行うのは禁止。処理が止まります。
 * ・stdio (printf, putc, getc, etc),
 * ・malloc
 * ・new
 */
ISR(TIMER1_OVF_vect) {
	avr16bitMTimer += (65536L * DIVIDE)/ F_CPU_DIV1000;
}