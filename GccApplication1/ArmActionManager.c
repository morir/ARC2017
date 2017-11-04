/*
 * ArmActionManager.c
 *
 * Created: 2017/10/28 22:38:53
 *  Author: Administrator
 */ 

#include "ArmActionManager.h"

#include "MotorManager.h"
#include "AvrTimer.h"
#include "DebugLog.h"


/************************************************************************/
// Arm用モータの初期設定
// Arm用モータを宝物検索用形態(ライントレース用)の位置に設定
/************************************************************************/
void initDumpMotor(void) {
	MotorControlJoint(WRIST_MOTOR, 200, 512);
	executeRotate(SHOULDER_MOTOR, 100, 680, 680);
	executeRotate(UPPER_ARM_MOTOR, 100, 70, 70);
	executeRotate(FORE_ARM_MOTOR, 100, 300, 300);
}

/************************************************************************/
// 宝物検索用形態
// 宝物検索用の位置に設定する。
/************************************************************************/
void FindFormation(void)
{
	MotorControlJoint(WRIST_MOTOR, 200, 512);
	executeRotate(SHOULDER_MOTOR, 100, 680, 680);
	executeRotate(UPPER_ARM_MOTOR, 100, 70, 70);
	executeRotate(FORE_ARM_MOTOR, 100, 300, 300);
}


/************************************************************************/
// 宝物回収形態(下げてひらいた状態)
// 宝物を回収するための準備形態にする
/************************************************************************/
void ArmOpenFormation(void)
{
	//-- 下げてひらく
	executeRotate(WRIST_MOTOR, 200, 770, 770);
	executeRotate(FORE_ARM_MOTOR, 150, 360, 360);
	executeRotate(UPPER_ARM_MOTOR, 150, 120, 120);
	executeRotate(SHOULDER_MOTOR, 150, 500, 500);
}

/************************************************************************/
// 宝物回収＆搭載用形態
// 宝物を回収し、搭載する
// 段階的に動作させ宝物を落とす
/************************************************************************/
void CatchAndReleaseFormation(void)
{
	//-- 掴む
	executeRotate(WRIST_MOTOR, 200, 512, 512 );

	//-- 持ち上げ開始	
	executeRotate(UPPER_ARM_MOTOR, 100, 250, 250);
	MotorControlJoint(SHOULDER_MOTOR, 100, 500);
	MotorControlJoint(FORE_ARM_MOTOR, 100, 680);

	//-- 持ち上げ途中
	executeRotate(UPPER_ARM_MOTOR, 100, 500, 500);

	//-- 落とす直前
	executeRotate(FORE_ARM_MOTOR, 100, 680, 680);
	MotorControlJoint( UPPER_ARM_MOTOR, 100, 680 );
	executeRotate( SHOULDER_MOTOR, 100, 430, 430 );

	//-- 落とす
	executeRotate( WRIST_MOTOR, 200, 620, 620 );
	
	_delay_ms(700);//1秒待つ⇒動作に合わせて変更してください

	//-- 宝物検索用形態に移行するための準備
	MotorControlJoint(UPPER_ARM_MOTOR, 100, 280);
	executeRotate(SHOULDER_MOTOR, 40, 580, 580);
	MotorControlJoint(WRIST_MOTOR, 200, 512);
	MotorControlJoint(FORE_ARM_MOTOR, 100, 400);	
	MotorControlJoint(SHOULDER_MOTOR, 100, 630);
	executeRotate(UPPER_ARM_MOTOR, 100, 120, 120);
	
	// 宝物検索用ライントレース形態に戻す
	FindFormation();
}


/************************************************************************/
// 現在のモータ角度を表示(Debug用)
// 現在のモータ角度を表示する。
/************************************************************************/
void Debug_AllMotorCurrentAngle(void)
{
	LOG_ERROR("GetCurrentAngle(SHOULDER_MOTOR) %d\r\n", GetCurrentAngle(SHOULDER_MOTOR));
	LOG_ERROR("GetCurrentAngle(UPPER_ARM_MOTOR) %d\r\n", GetCurrentAngle(UPPER_ARM_MOTOR));
	LOG_ERROR("GetCurrentAngle(FORE_ARM_MOTOR) %d\r\n", GetCurrentAngle(FORE_ARM_MOTOR));
	LOG_ERROR("GetCurrentAngle(WRIST_MOTOR) %d\r\n", GetCurrentAngle(WRIST_MOTOR));
}

/**
 * 設定角度が目標角度(次ステップへ進んで良い角度) になるまで動作する 
 * @param motorId     モータID
 * @param speed       設定速度
 * @param angle       設定角度
 * @param targetangle 目標角度(次ステップへ進んで良い角度) 
 */
void executeRotate(int motorId, int speed, int angle, int targetangle){
	//設定角度への動作を実行
	MotorControlJoint( motorId, speed, angle );
	
	if((angle - GetCurrentAngle(motorId)) > 0) 
	{
		// 目標角度に達していない間は動作する
		while( (targetangle - CorrectionValue) > GetCurrentAngle(motorId))
		{
			_delay_ms(50);//適切なウェイト時間を設定
		}
	}
	else
	{
		// 目標角度に達していない間は動作する
		while( GetCurrentAngle(motorId) > (targetangle + CorrectionValue))
		{
			_delay_ms(50);//適切なウェイト時間を設定
		}		
	}
}

/**
 * 現在のモータ角度取得
 * @brief  現在のモータ角度取得
 * @param  motorId モータID
 * @return 現在の角度
 * @detail 上位バイト2bit、下位8bitから現在の角度を取得する。
 *         パケット通信失敗時、前回の速度を返す。
 *         出力軸：0～300°、0～1023　※中央：150° = 512
 *         300～360°の間：不定値
 *
 *         下記のモータIDを入力パラメータとすること。
 *         SHOULDER_MOTOR       12      // Shoulder Motor address(肩モータ)
 *         UPPER_ARM_MOTOR      25      // Upper arm Motor address(上腕モータ)
 *         FORE_ARM_MOTOR       14      // ForeArm Motor address(前腕モータ)
 *         WRIST_MOTOR          23      // Wrist Motor address(手首モータ)
 */
int GetCurrentAngle(int motorId) {
	int readValueHigh = 0;	// 上位バイト
	int readValueLow = 0;	// 下位バイト
	static int angle = 0;	// 現在の位置
	
	// 上位バイト取得
	readValueHigh = dxl_read_byte(motorId, CTRL_TBL_ADDR_PRESENT_POSITION_H) & 0x03;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return angle;
	}
	// 下位バイト取得
	readValueLow  = dxl_read_byte(motorId, CTRL_TBL_ADDR_PRESENT_POSITION_L) & 0xFF;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return angle;
	}
	// 上位バイトと下位バイトから現在の位置を計算
	angle = ((readValueHigh << 8) + readValueLow);
	LOG_DEBUG("GetCurrentAngle(%d) is %d\n", motorId, angle);

	return angle;
}
