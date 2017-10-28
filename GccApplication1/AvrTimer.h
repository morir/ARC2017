/**
 * @file AvrTimer.h
 * @brief AVRタイマ制御部
 * @author f.aimano
 * @date 2017/10/20
 */

#ifndef AVRTIMER_H_
#define AVRTIMER_H_

#include "stdint.h"

//#define ENABLE_AVRTIMER	// AVRタイマーを有効にする（お試し版）

/**
 * AVRタイマ開始
 * @brief AVRタイマ開始
 */
void AvrTimerStart();
 
/**
 * AVRタイマ取得
 * @brief AVRタイマ取得(ミリ秒)
 */
uint32_t AvrTimerGet();

/**
 * AVRタイマ終了
 * @brief AVRタイマ終了
 */
void AvrTimerEnd();
 
/**
 * AVRタイマ初期化
 * @brief AVRタイマ初期化
 */
void AvrTimerReset();

#endif /* AVRTIMER_H_ */