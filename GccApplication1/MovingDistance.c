/**
 * @file MovingDistance.c
 * @brief 移動距離
 * @author f.aimano
 * @date 2017/10/28
 */

#include "AvrTimer.h"
#include "DebugLog.h"
#include "MotorManager.h"
#include "MovingDistance.h"

static int32_t movingDistance = 0;	// 移動距離
static int velocity_0 = 0;			// 初速度
static uint32_t lastTime = 0;		// 前回の時間(ミリ秒)
static MEASUREMENT_STATUS measurementStatus = NOT_MEASURING;	// 移動距離の計測中状態

/**
 * 移動距離の計測を開始
 * @brief 移動距離の計測を開始
 */
void StartMeasuringMovingDistance() {
	measurementStatus = MEASURING;
	movingDistance = 0;
}

/**
 * 移動距離の計測を停止
 * @brief 移動距離の計測を停止
 */
void StopMeasuringMovingDistance() {
	measurementStatus = NOT_MEASURING;
}

/**
 * 移動距離を取得
 * @brief 移動距離を取得
 * @return 移動距離
 */
int32_t GetMovingDistance() {
	return movingDistance;
}

/**
 * 移動距離を更新
 * @brief 移動距離を更新
 * @param  currentTime 現在の時間
 */
void UpdateMovingDistance(uint32_t currentTime) {
	if(measurementStatus == NOT_MEASURING) {
		// 未計測中の場合、移動距離の更新を行わない。
		return;
	}
	
	// 現在の速度を計算
	int velocity = (GetCurrentSignedSpeedR() + GetCurrentSignedSpeedL()) / 2;
	
	// 経過時間(msec)を計算
	uint32_t elapsedTime = currentTime - lastTime;
	
	// 加速度を計算
	int acceleration = 0;
	if(0 != elapsedTime) {
		acceleration = (velocity - velocity_0) / elapsedTime;
	}
	
	// 現在の移動距離を計算
	int32_t currentMovingDistance = velocity_0 * (int32_t)elapsedTime + acceleration *(int32_t)(elapsedTime * elapsedTime) / 2;
	
	// 「ｃｍ／ミリ秒」から「ｃｍ／秒」に変換し、移動距離に反映
	movingDistance = movingDistance + currentMovingDistance / 1000;
	
	// 初速度および前回の時間を更新
	velocity_0 = velocity;
	lastTime = currentTime;
}