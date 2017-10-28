/*
 * MotorManager.c
 *
 * Created: 2014/10/29 14:46:23
 *  Author: Administrator
 */ 
#include "MotorManager.h"
#include "math.h"

#include "AvrTimer.h"
#include "DebugLog.h"

#define DBG 1
//#define _MOTOR_OFF_

#define HIGH_RATE (0.90)
#define SOFT_ROUND_RATE (0.80)
#define MIDDLE_ROUND_RATE (0.70)
#define TIGHT_ROUND_RATE (0.60)
#define TURN_INSIDE_RATE (0.50)
#define HALF_RATE (0.50)

#ifdef ENABLE_AVRTIMER
static int curSpeedR = 0;	// 現在の速度(右モータ)
static int curSpeedL = 0;	// 現在の速度(左モータ)
#endif // ENABLE_AVRTIMER

void MotorInit(void) {
    dxl_initialize( 0, DEFAULT_BAUDNUM ); // Not using device index
    //Wheel Mode
//    dxl_write_word( RIGHT_MOTOR, P_CW_ANGLE_LIMIT_L, 0 );
//    dxl_write_word( RIGHT_MOTOR, P_CW_ANGLE_LIMIT_H, 0 );
//    dxl_write_word( RIGHT_MOTOR, P_CCW_ANGLE_LIMIT_L, 0 );
//    dxl_write_word( RIGHT_MOTOR, P_CCW_ANGLE_LIMIT_H, 0 );
//    dxl_write_word( LEFT_MOTOR, P_CW_ANGLE_LIMIT_L, 0 );
//    dxl_write_word( LEFT_MOTOR, P_CW_ANGLE_LIMIT_H, 0 );
//    dxl_write_word( LEFT_MOTOR, P_CCW_ANGLE_LIMIT_L, 0 );
//    dxl_write_word( LEFT_MOTOR, P_CCW_ANGLE_LIMIT_H, 0 );
    //Set Torque
    dxl_write_word( RIGHT_MOTOR, 24, 1 );
    dxl_write_word( LEFT_MOTOR,  24, 1 );
    //Set EEP Lock
    dxl_write_word( RIGHT_MOTOR, P_EEP_LOCK, 1 );
    dxl_write_word( LEFT_MOTOR, P_EEP_LOCK, 1 );
    // Set goal speed
    dxl_write_word( BROADCAST_ID, P_GOAL_SPEED_L, 0 );
	// AX-S1 赤外線を低感度モードに設定
	
    _delay_ms(500);//1000
	
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

void MotorControl(int id, int power) {
#ifndef _MOTOR_OFF_
    int CommStatus = COMM_RXSUCCESS;
    dxl_write_word( id, P_GOAL_SPEED_L, power );
    CommStatus = dxl_get_result();
    if( CommStatus == COMM_RXSUCCESS )
        PrintErrorCode();
    else
        PrintCommStatus(CommStatus);
#endif // _MOTOR_OFF_
}

/************************************************************************/
// 関節用モーター実行関数
// id モーターID
// speed 速度
// position 位置（0～1023、512はセンター）
/************************************************************************/
void MotorControlJoint(int id, int speed, int position) {
	#ifndef _MOTOR_OFF_
	int CommStatus = COMM_RXSUCCESS;
	dxl_write_word( id, P_GOAL_POSITION_L, position );
	dxl_write_word( id, P_GOAL_SPEED_L, speed );
	CommStatus = dxl_get_result();
	if( CommStatus == COMM_RXSUCCESS )
	PrintErrorCode();
	else
	PrintCommStatus(CommStatus);
	#endif // _MOTOR_OFF_
}

void Execute(int type) {
	//type = 0;//DBG
    switch (type) {
		case TRACE_STRAIGHT:
            LOG_INFO("TRACE_STRAIGHT\r\n");
			StraightMove();
            break;
		case TRACE_L_STRAIGHT:
            LOG_INFO("Left Straight\r\n");
            LeftStraightMove();
            break;
		case TRACE_R_STRAIGHT:
			LOG_INFO("Right Straight\r\n");
			RightStraightMove();
			break;
		case TRACE_L_ROUND_SOFT:
			LOG_INFO("Left Soft Round\r\n");
			LeftSoftRoundMove();
			break;
		case TRACE_L_ROUND_MIDDLE:
			LOG_INFO("Left Middle Round\r\n");
			LeftMiddleRoundMove();
			break;
		case TRACE_L_ROUND_TIGHT:
			LOG_INFO("Left Tight Round\r\n");
			LeftTightRoundMove();
			break;
		case TRACE_R_ROUND_SOFT:
            LOG_INFO("Right Soft Round\r\n");
            RightSoftRoundMove();
            break;
		case TRACE_R_ROUND_MIDDLE:
			LOG_INFO("Right Middle Round\r\n");
			RightMiddleRoundMove();
			break;
		case TRACE_R_ROUND_TIGHT:
			LOG_INFO("Right Tight Round\r\n");
			RightTightRoundMove();
			break;
		case TRACE_L_TURN:
            LOG_INFO("Left Turn\r\n");
            LeftTurnMove();
            break;	
		case TRACE_R_TURN:
            LOG_INFO("Right Turn\r\n");
            RightTurnMove();
            break;
		case TRACE_SLOW_STRAIGHT:
			LOG_INFO("TRACE_SLOW_STRAIGHT\r\n");
			StraightLowMove();
			break;
        default:
            LOG_INFO("Unknown type[%d]\r\n", type);
            break;
    }
}

void setParamMoveAction(int right, int left) {
	int correctionVal = 0;
    MotorControl( RIGHT_MOTOR, right );
    MotorControl( LEFT_MOTOR, left + correctionVal );
    LOG_DEBUG("RIGHT_MORTOR:%d  LEFT_MORTOR:%d \r\n",right, left);
}

void StopMove(void) {
    MotorControl( RIGHT_MOTOR, 1024 );
    MotorControl( LEFT_MOTOR, 0 );
}

void StraightMoveRightShift(void) {
    MotorControl( RIGHT_MOTOR, 1473 );//280 P_CCW_SPEED_TURN    1532
    MotorControl( LEFT_MOTOR, 500 );//300 P_CW_SPEED_NOMAL    609
}

void StraightMoveLeftShift(void) {
    MotorControl( RIGHT_MOTOR, 1523 );//300 P_CCW_SPEED_NOMAL   1632
    MotorControl( LEFT_MOTOR, 450 );//280 P_CW_SPEED_TURN     509
}

void StraightMoveRightShift2(void) {
    MotorControl( RIGHT_MOTOR, 1423 );//250 P_CCW_SPEED_TURN_2 1432
    MotorControl( LEFT_MOTOR, 500 );//300 P_CW_SPEED_NOMAL    609
}

void StraightMoveLeftShift2(void) {
    MotorControl( RIGHT_MOTOR, 1523 ); //300 P_CCW_SPEED_NOMAL 1632
    MotorControl( LEFT_MOTOR, 400 ); //250 P_CW_SPEED_TURN_2 409
}

void TurnMoveRight(void) {
    MotorControl( RIGHT_MOTOR, 400 );//250 P_CW_SPEED_TURN 509
    MotorControl( LEFT_MOTOR, 400 );//250 P_CW_SPEED_TURN 509
}

void TurnMoveLeft(void) {
    MotorControl( RIGHT_MOTOR, 1424 );//250 P_CCW_SPEED_TURN 1532
    MotorControl( LEFT_MOTOR, 1424 );//250 P_CCW_SPEED_TURN 1532
}

void TurnLowMoveRight(void) {
    MotorControl( RIGHT_MOTOR, 300 );//200 P_CW_SPEED_TURN_2   409
    MotorControl( LEFT_MOTOR, 300 );//200 P_CW_SPEED_TURN_2 409
}

void TurnLowMoveLeft(void) {
    MotorControl( RIGHT_MOTOR, 1324 );//200 P_CCW_SPEED_TURN_2 1432
    MotorControl( LEFT_MOTOR, 1324 );//200 P_CCW_SPEED_TURN_2 1432
}

void BackMove(void) {
    MotorControl( RIGHT_MOTOR, 250 ); //250 P_CW_SPEED_TURN 509
    MotorControl( LEFT_MOTOR, 1273 ); //R250 P_CCW_SPEED_TURN 1532
}

void Move(int leftSpeed, int rightSpeed)
{
	//LOG_INFO("left = %3d, Right = %3d\r\n", leftSpeed, rightSpeed);
    MotorControl(LEFT_MOTOR, leftSpeed);
    MotorControl(RIGHT_MOTOR, rightSpeed);
}

void StraightMove(void) {
	int execBaseSpeed = (BaseSpeed < MAX_SPEED) ? BaseSpeed : MAX_SPEED;
	int leftSpeed = execBaseSpeed;
	int rightSpeed = (1024 + execBaseSpeed);

	Move(leftSpeed, rightSpeed);
}

void StraightLowMove(void) {
	int leftSpeed = SLOW_STRAIGHT_VAL;
	int rightSpeed = (1024 + SLOW_STRAIGHT_VAL);

	Move(leftSpeed, rightSpeed);
}

void BackLowMove(void) {
	int leftSpeed = (1024 + SLOW_BACK_VAL);
	int rightSpeed = SLOW_BACK_VAL;

	MotorControl( LEFT_MOTOR, leftSpeed);
	MotorControl( RIGHT_MOTOR, rightSpeed );
}

void LeftStraightMove(void) {
	int execBaseSpeed = (BaseSpeed < MAX_SPEED) ? BaseSpeed : MAX_SPEED;
	int leftSpeed = (int)((float)execBaseSpeed * HIGH_RATE);
	int rightSpeed = (1024 + execBaseSpeed);

	Move(leftSpeed, rightSpeed);
}

void RightStraightMove(void) {
	int execBaseSpeed = (BaseSpeed < MAX_SPEED) ? BaseSpeed : MAX_SPEED;
	int leftSpeed = execBaseSpeed;
	int rightSpeed = (1024 + (int)((float)execBaseSpeed * HIGH_RATE));

	Move(leftSpeed, rightSpeed);
}

void LeftSoftRoundMove(void) {
	int execBaseSpeed = (BaseSpeed < MAX_SPEED) ? BaseSpeed : MAX_SPEED;
	int leftSpeed = (execBaseSpeed - 10) ;
	int rightSpeed = (1024 + execBaseSpeed);

	Move(leftSpeed, rightSpeed);
}

void LeftMiddleRoundMove(void) {
	int execBaseSpeed = (BaseSpeed < MAX_SPEED) ? BaseSpeed : MAX_SPEED;
	int leftSpeed = (execBaseSpeed - 20) ;
	int rightSpeed = (1024 + execBaseSpeed);

	Move(leftSpeed, rightSpeed);
}

void LeftTightRoundMove(void) {
	int execBaseSpeed = (BaseSpeed < MAX_SPEED) ? BaseSpeed : MAX_SPEED;
	int leftSpeed = (execBaseSpeed - 40) ;
	int rightSpeed = (1024 + execBaseSpeed);

	Move(leftSpeed, rightSpeed);
}

void RightSoftRoundMove(void) {
	int execBaseSpeed = (BaseSpeed < MAX_SPEED) ? BaseSpeed : MAX_SPEED;
	int leftSpeed = execBaseSpeed;
	int rightSpeed = (1024 + (execBaseSpeed - 10));

	Move(leftSpeed, rightSpeed);
}

void RightMiddleRoundMove(void) {
	int execBaseSpeed = (BaseSpeed < MAX_SPEED) ? BaseSpeed : MAX_SPEED;
	int leftSpeed = execBaseSpeed;
	int rightSpeed = (1024 + (execBaseSpeed - 20));

	Move(leftSpeed, rightSpeed);
}

void RightTightRoundMove(void) {
	int execBaseSpeed = (BaseSpeed < MAX_SPEED) ? BaseSpeed : MAX_SPEED;
	int leftSpeed = execBaseSpeed;
	int rightSpeed = (1024 + (execBaseSpeed - 40));

	Move(leftSpeed, rightSpeed);
}

void LeftTurnMove(void) {
	int speed = TURN_SPEED_BASE;
	int leftSpeed = (1024 + (speed));
	int rightSpeed = (1024 + speed);

	Move(leftSpeed, rightSpeed);
}

void RightTurnMove(void) {
	int speed = TURN_SPEED_BASE;
	int leftSpeed = speed;
	int rightSpeed = (speed);

	Move(leftSpeed, rightSpeed);
}

void LeftTurnByBaseSpeedAdjust(void) {
	int leftSpeed = TURN_SPEED_BASE;
	int rightSpeed = TURN_SPEED_BASE;
	int turnSpeedJudgeVal = (int)(TURN_SPEED_JUDGE_VAL / 10);
	int baseSpeedVal = (int)((BaseSpeed)/ 10);
	int turnBaseVal = TURN_SPEED_BASE /10;
	int turnAdjustVal = (int)(TURN_SPEED_BASE - (((baseSpeedVal * baseSpeedVal * turnBaseVal) / (turnSpeedJudgeVal * turnSpeedJudgeVal)) * 10) );
	LOG_DEBUG("BaseSpeed:[%d] turnAdjustVal:[%d]\r\n",BaseSpeed, turnAdjustVal);

	if (turnAdjustVal > 0) {
		leftSpeed = TURN_SPEED_BASE - turnAdjustVal;
		rightSpeed = TURN_SPEED_BASE + turnAdjustVal;
	} else if (turnAdjustVal < 0 ) {
		leftSpeed = TURN_SPEED_BASE + (TURN_SPEED_BASE + turnAdjustVal);
		rightSpeed = TURN_SPEED_BASE - (TURN_SPEED_BASE + turnAdjustVal);
	}
    LOG_DEBUG("leftSpeed:[%d] rightSpeed:[%d]\r\n",leftSpeed, rightSpeed);
	
	leftSpeed = (1024 + leftSpeed);
	rightSpeed = (1024 + rightSpeed);

	Move(leftSpeed, rightSpeed);
}

void RightTurnByBaseSpeedAdjust(void) {
	int leftSpeed = TURN_SPEED_BASE;
	int rightSpeed = TURN_SPEED_BASE;
	int turnSpeedJudgeVal = (int)(TURN_SPEED_JUDGE_VAL / 10);
	int baseSpeedVal = (int)(BaseSpeed / 10);
	int turnBaseVal = TURN_SPEED_BASE /10;
	int turnAdjustVal = (int)(TURN_SPEED_BASE - (((baseSpeedVal * baseSpeedVal * turnBaseVal) / (turnSpeedJudgeVal * turnSpeedJudgeVal)) * 10) );
    LOG_DEBUG("BaseSpeed:[%d] turnAdjustVal:[%d]\r\n",BaseSpeed, turnAdjustVal);

	if (turnAdjustVal > 0) {
		leftSpeed = TURN_SPEED_BASE + turnAdjustVal;
		rightSpeed = TURN_SPEED_BASE - turnAdjustVal;
	} else if (turnAdjustVal < 0 ) {
		leftSpeed = TURN_SPEED_BASE - (TURN_SPEED_BASE + turnAdjustVal);
		rightSpeed = TURN_SPEED_BASE + (TURN_SPEED_BASE + turnAdjustVal);
	}
    LOG_DEBUG("leftSpeed:[%d] rightSpeed:[%d]\r\n",leftSpeed, rightSpeed);
	
	leftSpeed = (leftSpeed);
	rightSpeed = (rightSpeed);

	Move(leftSpeed, rightSpeed);
}

void LeftTurnSlowMove(int rate) {
	int speed = (TURN_SPEED_BASE * rate) / 100;
	int leftSpeed = (1024 + speed);
	int rightSpeed = (1024 + speed);

	Move(leftSpeed, rightSpeed);
}

void RightTurnSlowMove(int rate) {
	int speed = (TURN_SPEED_BASE * rate) / 100;
	int leftSpeed = speed;
	int rightSpeed = speed;

	Move(leftSpeed, rightSpeed);
}

void PrintErrorCode() {
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
    LOG_ERROR("Input voltage error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
    LOG_ERROR("Angle limit error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
    LOG_ERROR("Overheat error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
    LOG_ERROR("Out of range error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
    LOG_ERROR("Checksum error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
    LOG_ERROR("Overload error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
    LOG_ERROR("Instruction code error!\n");
}

// Print communication result
void PrintCommStatus(int CommStatus) {
    switch(CommStatus)  {
        case COMM_TXFAIL:
            LOG_INFO("COMM_TXFAIL: Failed transmit instruction packet!\n");
            break;

        case COMM_TXERROR:
            LOG_INFO("COMM_TXERROR: Incorrect instruction packet!\n");
            break;

        case COMM_RXFAIL:
            LOG_INFO("COMM_RXFAIL: Failed get status packet from device!\n");
            break;

        case COMM_RXWAITING:
            LOG_INFO("COMM_RXWAITING: Now receiving status packet!\n");
            break;

        case COMM_RXTIMEOUT:
            LOG_INFO("COMM_RXTIMEOUT: There is no status packet!\n");
            break;

        case COMM_RXCORRUPT:
            LOG_INFO("COMM_RXCORRUPT: Incorrect status packet!\n");
            break;

        default:
            LOG_INFO("This is unknown error code!\n");
            break;
    }
}

#ifdef ENABLE_AVRTIMER
/**
 * 現在の速度(右モータ)取得。
 * @brief 現在の速度(右モータ)取得。
 * @return 現在の速度(右モータ)
 * @detail 現在の速度(右モータ)を取得する。
 *         停止:0,1024
 *         前進:1025～2047(CCW)
 *         後進:0001～1023(CW)
 */
int GetCurrentSpeedR(void) {
	return curSpeedR;
}

/**
 * 現在の速度(左モータ)取得。
 * @brief 現在の速度(左モータ)取得。
 * @return 現在の速度(左モータ)
 * @detail 現在の速度(左モータ)を取得する。
 *         停止:0,1024
 *         前進:0001～1023(CW)
 *         後進:1025～2047(CCW)
 */
int GetCurrentSpeedL(void) {
	return curSpeedL;
}

/**
 * 現在の符号付の速度(右モータ)を取得。
 * @brief 現在の符号付の速度(右モータ)を取得。
 * @return 符号付の現在の速度(右モータ)
 * @detail 現在の符号付の速度(右モータ)を取得する。
 *         停止:0,1024
 *         前進:1025～2047(CCW) ⇒ 1～1023
 *         後進:0001～1023(CW)  ⇒ -1023～-1 
 */
int GetCurrentSignedSpeedR(void) {
	int signed_speed = GetCurrentSpeedR();
	
	if(signed_speed == 0 || signed_speed == 1024)
	{
		signed_speed = 0;
	}
	else if(1025 <= signed_speed && signed_speed <= 2047) {
		signed_speed = signed_speed - 1024;
	}
	else {
		signed_speed = signed_speed - 1024;
	}
	
	return signed_speed;
}

/**
 * 現在の符号付の速度(左モータ)を取得。
 * @brief 現在の符号付の速度(左モータ)を取得。
 * @return 符号付の現在の速度(左モータ)
 * @detail 現在の符号付の速度(左モータ)を取得する。
 *         停止:0,1024
 *         前進:0001～1023(CW)  ⇒ 変換しない
 *         後進:1025～2047(CCW) ⇒ -1023～-1
 */
int GetCurrentSignedSpeedL(void) {
	int signed_speed = GetCurrentSpeedL();
	if(signed_speed == 0 || signed_speed == 1024)
	{
		signed_speed = 0;
	}
	else if(1 <= signed_speed && signed_speed <= 1023) {
		//変換しない
	}
	else {
		signed_speed = signed_speed - 1024;
	}
	
	return signed_speed;
}

/**
 * パケット通信をして、現在の速度(右モータ)取得。
 * @brief パケット通信をして、現在の速度(右モータ)取得。
 * @detail パケット（上位バイト3bit、下位8bit）から現在の速度(右モータ)を取得する。
 *         パケット通信失敗時、前回の速度を返す。
 *         前進:1024～2047(CCW)
 *         後進:0000～1023(CW)
 */
void GetCurrentSpeedRCalledFromTimer(void) {
	int readValueHigh = 0;	// 上位バイト
	int readValueLow = 0;	// 下位バイト
	
	// 上位バイト取得
	readValueHigh = dxl_read_byte(RIGHT_MOTOR, CTRL_TBL_ADDR_PRESENT_SPEED_H) & 0x07;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return ;
	}
	// 下位バイト取得
	readValueLow  = dxl_read_byte(RIGHT_MOTOR, CTRL_TBL_ADDR_PRESENT_SPEED_L) & 0xFF;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return ;
	}
	// 上位バイトと下位バイトから現在の速度を計算
	curSpeedR = ((readValueHigh << 8) + readValueLow);
	LOG_DEBUG("GetCurrentSpeedR() is %d\n", curSpeedR);

	return ;
}

/**
 * パケット通信をして、現在の速度(左モータ)取得。
 * @brief パケット通信をして、現在の速度(左モータ)取得。
 * @detail パケット（上位バイト3bit、下位8bit）から現在の速度(左モータ)を取得する。
 *         パケット通信失敗時、前回の速度を返す。
 *         前進:0000～1023(CW)
 *         後進:1024～2047(CCW)
 */
void GetCurrentSpeedLCalledFromTimer(void) {
	int readValueHigh = 0;	// 上位バイト
	int readValueLow = 0;	// 下位バイト
	
	// 上位バイト取得
	readValueHigh = dxl_read_byte(LEFT_MOTOR, CTRL_TBL_ADDR_PRESENT_SPEED_H) & 0x07;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return ;
	}
	// 下位バイト取得
	readValueLow  = dxl_read_byte(LEFT_MOTOR, CTRL_TBL_ADDR_PRESENT_SPEED_L) & 0xFF;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return ;
	}
	// 上位バイトと下位バイトから現在の速度を計算
	curSpeedL = ((readValueHigh << 8) + readValueLow);
	LOG_DEBUG("GetCurrentSpeedL() is %d\n", curSpeedL);

	return ;
}
#else // ENABLE_AVRTIMER

/**
 * 現在の速度(右モータ)取得。
 * @brief 現在の速度(右モータ)取得。
 * @return 現在の速度(右モータ)
 * @detail 上位バイト3bit、下位8bitから現在の速度(右モータ)を取得する。
 *         パケット通信失敗時、前回の速度を返す。
 *         前進:1024～2047(CCW)
 *         後進:0000～1023(CW)
 */
int GetCurrentSpeedR(void) {
	int readValueHigh = 0;	// 上位バイト
	int readValueLow = 0;	// 下位バイト
	static int speed = 0;	// 現在の速度
	
	// 上位バイト取得
	readValueHigh = dxl_read_byte(RIGHT_MOTOR, CTRL_TBL_ADDR_PRESENT_SPEED_H) & 0x07;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return speed;
	}
	// 下位バイト取得
	readValueLow  = dxl_read_byte(RIGHT_MOTOR, CTRL_TBL_ADDR_PRESENT_SPEED_L) & 0xFF;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return speed;
	}
	// 上位バイトと下位バイトから現在の速度を計算
	speed = ((readValueHigh << 8) + readValueLow);
	LOG_DEBUG("GetCurrentSpeedR() is %d\n", speed);

	return speed;
}

/**
 * 現在の速度(左モータ)取得。
 * @brief 現在の速度(左モータ)取得。
 * @return 現在の速度(左モータ)
 * @detail 上位バイト3bit、下位8bitから現在の速度(左モータ)を取得する。
 *         パケット通信失敗時、前回の速度を返す。
 *         前進:0000～1023(CW)
 *         後進:1024～2047(CCW)
 */
int GetCurrentSpeedL(void) {
	int readValueHigh = 0;	// 上位バイト
	int readValueLow = 0;	// 下位バイト
	static int speed = 0;	// 現在の速度
	
	// 上位バイト取得
	readValueHigh = dxl_read_byte(LEFT_MOTOR, CTRL_TBL_ADDR_PRESENT_SPEED_H) & 0x07;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return speed;
	}
	// 下位バイト取得
	readValueLow  = dxl_read_byte(LEFT_MOTOR, CTRL_TBL_ADDR_PRESENT_SPEED_L) & 0xFF;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return speed;
	}
	// 上位バイトと下位バイトから現在の速度を計算
	speed = ((readValueHigh << 8) + readValueLow);
	LOG_DEBUG("GetCurrentSpeedL() is %d\n", speed);

	return speed;
}

int GetCurrentSignedSpeedR(void) {
	// ビルドエラー対応処理
	return 0;
}
int GetCurrentSignedSpeedL(void) {
	// ビルドエラー対応処理
	return 0;
}
#endif // ENABLE_AVRTIMER

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
	LOG_DEBUG("GetCurrentPositionR() is %d\n", angle);

	return angle;
}

/**
 * AX-S1の赤外線センサ値を取得する。
 * @brief AX-S1の赤外線センサ値を取得する。
 * @param (int *out_fire_data_left) 左側の赤外線センサ値
 * @param (int *out_fire_data_center) 中央の赤外線センサ値
 * @param (int *out_fire_data_right) 右側の赤外線センサ値
 * @return なし
 */
void GetAXS1SensorFireData(int *out_fire_data_left, int *out_fire_data_center, int *out_fire_data_right)
{
	static int fire_data_left = 0;		// 左側の赤外線センサ値
	static int fire_data_center = 0;	// 中央の赤外線センサ値
	static int fire_data_right = 0;		// 右側の赤外線センサ値
	
	// 左側の赤外線センサ値
	fire_data_left = dxl_read_byte(CENTER_AXS1_SENSOR, AXS1_ADDR_IR_LEFT_FIRE_DATA);
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return;
	}
	*out_fire_data_left = fire_data_left;
	
	// 中央の赤外線センサ値
	fire_data_center = dxl_read_byte(CENTER_AXS1_SENSOR, AXS1_ADDR_IR_CENTER_FIRE_DATA);
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return;
	}
	*out_fire_data_center = fire_data_center;
	
	// 右側の赤外線センサ値
	fire_data_right = dxl_read_byte(CENTER_AXS1_SENSOR, AXS1_ADDR_IR_RIGHT_FIRE_DATA);
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return;
	}
	*out_fire_data_right = fire_data_right;
	
	LOG_DEBUG("GetAXS1SensorFireData() [%4d, %4d, %4d]\n", fire_data_left, fire_data_center, fire_data_right);

	return;
}