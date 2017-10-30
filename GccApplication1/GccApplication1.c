//##########################################################
//##                      R O B O T I S                   ##
//## CM-700 (Atmega2561) Example code for Dynamixel.      ##
//##                                           2009.11.10 ##
//##########################################################

#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include "SerialManager.h"
#include "include/dynamixel.h"
#include "SensorManager.h"
#include "MotorManager.h"
#include "DebugLog.h"
#include "AvrTimer.h"
#include "ArmActionManager.h"

// ------------------ Defined ------------------
// Line Sensor
#define LINE_STATE_BLACK    0//センサー値でラインが白判定
#define LINE_STATE_WHITE    1//センサー値でラインが黒判定

#define _LED_ON_

#define STOP_JUDGE_MAX_LIMIT	(10)//停止判定の上限値
#define SLOW_TURN_RATE_BY_BASE	(50)//ベースの20%の速さ
#define HISTORY_MAXSIZE (5)//履歴管理最大数

#define PATTERN_L_ROUND_TIGHT (-3)
#define PATTERN_L_ROUND_MIDDLE (-2)
#define PATTERN_L_ROUND_SOFT (-1)
#define PATTERN_R_ROUND_SOFT (1)
#define PATTERN_R_ROUND_MIDDLE (2)
#define PATTERN_R_ROUND_TIGHT (3)

#ifdef ENABLE_AVRTIMER
// ------------------ Global variables ------------------
static int curSensorPattern = 0;	// 現在のセンサー値
#endif // ENABLE_AVRTIMER

int isSearchingLeftSide = 0;

// ------------------ Method Definition ------------------
void executeTraceProcess(void);
void traceCommon(int counter, int maxSpeed);
void traceForwardArea_01(void);
void traceForwardArea_02(void);
void traceForwardArea_03(void);
void traceForwardArea_04(void);
void traceForwardArea_05(void);
void treasureHunt_01(void);
void traceBackwardArea_01(void);
void traceBackwardArea_02(void);
void traceBackwardArea_03(void);
void traceBackwardArea_04(void);
void treasureHunt_02(void);
void traceBackwardArea_06(void);
void traceBackwardArea_07(void);
void traceBackwardArea_08(void);
void traceBackwardArea_09(void);
void traceBackwardArea_10(void);
void traceBackwardArea_11(void);
void traceBackwardArea_12(void);
void traceBackwardArea_13(void);
void treasureHunt_03(void);
void traceBackwardArea_14(void);
void traceBackwardArea_15(void);
void traceBackwardArea_16(void);
void traceBackwardArea_17(void);
void traceBackwardArea_18(void);

#ifdef ENABLE_AVRTIMER
int getSensorPatternCalledFromTimer(void);
#endif // ENABLE_AVRTIMER
int getSensorPattern(void);
void initSensorHistory(void);
int getActionWithHistory(void);
void initCargoBedMotor(void);
void dumpTreasures(void);
void stopMoveLessThanVal(int val);

void TreasureFindingLineTrace(int isFirst);
void executeLeftTurnFromOnLine(void);
void executeRightTurnFromOnLine(void);

void getSensors(void);

int executeLeftTurn(void);
int executeRightTurn(void);

int initLeftTurnAction(int maxVal);
int initRightTurnAction(int maxVal);
void adjustTurnPosition(void);
void executeFinalAction(void);

void initEmergencyStop(void);

void setLED(void);
void LED_on(int i);
void LED_off(int i);

// ------------------ Global Variables Definition ------------------

// Serial Message Buffer
int serCmd[SERIAL_BUFFER_SIZE] = {0};

// IR Sensor 
unsigned int IR[ADC_PORT_6 + 1] = {0,0,0,0,0,0,0};

// IRの状態(BITパターン)
int IR_BitPattern = 0;
int IR_BitPatternHistory[HISTORY_MAXSIZE] =
 { TRACE_STRAIGHT, TRACE_STRAIGHT, TRACE_STRAIGHT, TRACE_STRAIGHT, TRACE_STRAIGHT };
int currentCount = 0;

// 今回のトレース動作
int currentTraceAction = TRACE_STRAIGHT;

int isSearchingLeft = 0;

// ------------------ Method ------------------

// ------------------ Table ------------------
int ActionTable[] = {
	/* 00:BIT_000000 */	TRACE_STRAIGHT,
	/* 01:BIT_000001 */	TRACE_R_ROUND_TIGHT,
	/* 02:BIT_000010 */	TRACE_R_ROUND_SOFT,
	/* 03:BIT_000011 */	TRACE_R_TURN,
	/* 04:BIT_000100 */	TRACE_R_ROUND_SOFT,
	/* 05:BIT_000101 */	TRACE_R_TURN,
	/* 06:BIT_000110 */	TRACE_R_ROUND_SOFT,
	/* 07:BIT_000111 */	TRACE_R_TURN,
	/* 08:BIT_001000 */	TRACE_L_ROUND_SOFT,
	/* 09:BIT_001001 */	TRACE_UNDEFINED,
	/* 10:BIT_001010 */	TRACE_UNDEFINED,
	/* 11:BIT_001011 */	TRACE_UNDEFINED,
	/* 12:BIT_001100 */	TRACE_STRAIGHT,
	/* 13:BIT_001101 */	TRACE_UNDEFINED,
	/* 14:BIT_001110 */	TRACE_R_ROUND_SOFT,
	/* 15:BIT_001111 */	TRACE_R_TURN,
	/* 16:BIT_010000 */	TRACE_L_ROUND_SOFT,
	/* 17:BIT_010001 */	TRACE_UNDEFINED,
	/* 18:BIT_010010 */	TRACE_UNDEFINED,
	/* 19:BIT_010011 */	TRACE_UNDEFINED,
	/* 20:BIT_010100 */	TRACE_UNDEFINED,
	/* 21:BIT_010101 */	TRACE_UNDEFINED,
	/* 22:BIT_010110 */	TRACE_UNDEFINED,
	/* 23:BIT_010111 */	TRACE_UNDEFINED,
	/* 24:BIT_011000 */	TRACE_L_ROUND_SOFT,
	/* 25:BIT_011001 */	TRACE_UNDEFINED,
	/* 26:BIT_011010 */	TRACE_UNDEFINED,
	/* 27:BIT_011011 */	TRACE_UNDEFINED,
	/* 28:BIT_011100 */	TRACE_L_ROUND_SOFT,
	/* 29:BIT_011101 */	TRACE_UNDEFINED,
	/* 30:BIT_011110 */	TRACE_STRAIGHT,
	/* 31:BIT_011111 */	TRACE_R_TURN,
	/* 32:BIT_100000 */	TRACE_L_ROUND_TIGHT,
	/* 33:BIT_100001 */	TRACE_UNDEFINED,
	/* 34:BIT_100010 */	TRACE_UNDEFINED,
	/* 35:BIT_100011 */	TRACE_UNDEFINED,
	/* 36:BIT_100100 */	TRACE_UNDEFINED,
	/* 37:BIT_100101 */	TRACE_UNDEFINED,
	/* 38:BIT_100110 */	TRACE_UNDEFINED,
	/* 39:BIT_100111 */	TRACE_UNDEFINED,
	/* 40:BIT_101000 */	TRACE_L_TURN,
	/* 41:BIT_101001 */	TRACE_UNDEFINED,
	/* 42:BIT_101010 */	TRACE_UNDEFINED,
	/* 43:BIT_101011 */	TRACE_UNDEFINED,
	/* 44:BIT_101100 */	TRACE_UNDEFINED,
	/* 45:BIT_101101 */	TRACE_UNDEFINED,
	/* 46:BIT_101110 */	TRACE_UNDEFINED,
	/* 47:BIT_101111 */	TRACE_UNDEFINED,
	/* 48:BIT_110000 */	TRACE_L_TURN,
	/* 49:BIT_110001 */	TRACE_UNDEFINED,
	/* 50:BIT_110010 */	TRACE_UNDEFINED,
	/* 51:BIT_110011 */	TRACE_UNDEFINED,
	/* 52:BIT_110100 */	TRACE_UNDEFINED,
	/* 53:BIT_110101 */	TRACE_UNDEFINED,
	/* 54:BIT_110110 */	TRACE_UNDEFINED,
	/* 55:BIT_110111 */	TRACE_UNDEFINED,
	/* 56:BIT_111000 */	TRACE_L_TURN,
	/* 57:BIT_111001 */	TRACE_UNDEFINED,
	/* 58:BIT_111010 */	TRACE_UNDEFINED,
	/* 59:BIT_111011 */	TRACE_UNDEFINED,
	/* 60:BIT_111100 */	TRACE_L_TURN,
	/* 61:BIT_111101 */	TRACE_UNDEFINED,
	/* 62:BIT_111110 */	TRACE_L_TURN,
	/* 63:BIT_111111 */	TRACE_SLOW_STRAIGHT
};

enum patternIndex {
	L_TURN,
	L_ROUND_TIGHT,
	L_ROUND_MIDDLE,
	L_ROUND_SOFT,
	STRAIGHT,
	R_ROUND_SOFT,
	R_ROUND_MIDDLE,
	R_ROUND_TIGHT,
	R_TURN,
	UNDEFINED
};

/**
* エントリーポイント
* @brief エントリーポイント
* @return 0：メイン処理の継続
* @return 1：メイン処理の終了
*/
int main(void) {
    
    initEmergencyStop();
    setLED();
    initIRSensor();
	initSensorHistory();
    MotorInit();
    initSerial();
	initCargoBedMotor();

#if(0)
	// 現在のモータ角度を表示(Debug用)
	Debug_AllMotorCurrentAngle();

	LOG_DEBUG("Call initDumpMotor() %s\r\n", "");
 	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	initDumpMotor();
	
	LOG_DEBUG("Call FindFormation() %s\r\n", "");
//	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
//	FindFormation();
	
	LOG_DEBUG("Call CatchAndReleaseFormation() %s\r\n", ""); 
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	CatchAndReleaseFormation();
	
	LOG_DEBUG("Call FindFormation() %s\r\n", "");
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	FindFormation();
#else
	// 現在のモータ角度を表示(Debug用)
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	Debug_AllMotorCurrentAngle();
#endif

#ifdef ENABLE_AVRTIMER
	getSensorPatternCalledFromTimer();

	// AVRタイマ開始
	AvrTimerStart();

#else // ENABLE_AVRTIMER
	getSensorPattern();
#endif // ENABLE_AVRTIMER

	// トレース動作開始
	executeTraceProcess();

    // ゴール判定後の動作実質ここから開始？
	//executeFinalAction();
	StopMove();

#ifdef ENABLE_AVRTIMER
	// AVRタイマ停止
	AvrTimerEnd();
	AvrTimerReset();
#endif // ENABLE_AVRTIMER
}

/**
* ライントレース動作
* @brief ライントレース動作
* @return なし
* @detail ゴール判定条件を満たすまでライントレース動作を行う。
*/
void executeTraceProcess(void) {
	traceForwardArea_01();
	traceForwardArea_02();
	traceForwardArea_03();
	traceForwardArea_04();
	traceForwardArea_05();
	treasureHunt_01();
	traceBackwardArea_01();
	traceBackwardArea_02();
	traceBackwardArea_03();
	traceBackwardArea_04();
	treasureHunt_02();
	traceBackwardArea_06();
	traceBackwardArea_07();
	traceBackwardArea_08();
	traceBackwardArea_09();
	traceBackwardArea_10();
	traceBackwardArea_11();
	traceBackwardArea_12();
	traceBackwardArea_13();
	treasureHunt_03();
	traceBackwardArea_14();
	traceBackwardArea_15();
	traceBackwardArea_16();
	traceBackwardArea_17();
	traceBackwardArea_18();
}

/*
 * 共通トレース動作
 * @return なし
 * @condition
 *   開始条件：スタートコマンドを受信する。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
 void traceCommon(int counter, int maxSpeed) {
	// センサー値を取得
	getSensors();
	currentTraceAction = getActionWithHistory();
	if (currentTraceAction == TRACE_UNDEFINED) {
		return;
	}

#ifdef LOG_INFO_ON
	if ((counter % 1) == 0) {
		BaseSpeed = BaseSpeed + 1;
		counter = 0;
	}
#else
	if ((counter % 5) == 0) {
		BaseSpeed = BaseSpeed + 2;
		counter = 0;
	}
#endif /* _MODE_SKIP_ */
	if (BaseSpeed > maxSpeed) {
		BaseSpeed = maxSpeed;
	}

	Execute(currentTraceAction);
	_delay_ms(1);// delayTimeの間隔を空ける
}

/*
 * 往路エリア 1 のトレース動作
 * @return なし
 * @condition
 *   開始条件：スタートコマンドを受信する。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
 void traceForwardArea_01(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	//初期動作（少しだけ直進）
	StraightMove();
	_delay_ms(100);	// 10ms 間隔を空ける

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 左旋回実行
	currentTraceAction = executeLeftTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 往路エリア 2 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（往路エリア 1 のトレース動作から継続）。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
 void traceForwardArea_02(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 左旋回実行
	currentTraceAction = executeLeftTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 往路エリア 3 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（往路エリア 2 のトレース動作から継続）。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
 void traceForwardArea_03(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 左旋回実行
	currentTraceAction = executeLeftTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 往路エリア 4 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（往路エリア 3 のトレース動作から継続）。
 *   終了条件：センサで右ターンを検出して直角旋回が完了する。
 */
 void traceForwardArea_04(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 右旋回実行
	currentTraceAction = executeRightTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 往路エリア 5 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（往路エリア 4 のトレース動作から継続）。
 *   終了条件：センサで右ターンを検出して直角旋回が完了する。
 */
 void traceForwardArea_05(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 右旋回実行
	currentTraceAction = executeRightTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 宝物 1 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（往路エリア 5 のトレース動作から継続）。
 *   終了条件：宝物（白）を回収、180度旋回が完了する。
 */
 void treasureHunt_01(void) {
         //int sensorPattern = BIT_000000;
	LOG_INFO("treasureHunt_01() %s\r\n", "1");

         int left = 0, center = 0, right = 0;
		 int isFirst = 0;
         while (center <= 180) {
			 GetAXS1SensorFireData(&left, &center, &right);
             // 宝物検索用ライントレースを実行
             TreasureFindingLineTrace(isFirst);
			 isFirst++;
         }
         // 停止する
         StopMove();
         _delay_ms(100);
         // 前進or後進する（実動作に合わせて設定）。
         // executeXXXX();
	// 手を開く
	MotorControlJoint( WRIST_MOTOR, 100, 665 );//確認用
         
         // 停止する
         StopMove();
         _delay_ms(100);

         // 宝物を掴んで荷台に乗せる
         CatchAndReleaseFormation();

         // ライン上からの旋回を行う
         executeRightTurnFromOnLine();

         // 停止する
         StopMove();
         _delay_ms(100);
}

/*
 * 復路エリア 1 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（折り返し点のトレース動作から継続）。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
 void traceBackwardArea_01(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 左旋回実行
	currentTraceAction = executeLeftTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 復路エリア 2 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 1 のトレース動作から継続）。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
 void traceBackwardArea_02(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 左旋回実行
	currentTraceAction = executeLeftTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 復路エリア 3 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 2 のトレース動作から継続）。
 *   終了条件：センサで右ターンを検出して直角旋回が完了する。
 */
 void traceBackwardArea_03(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 停止実行
	StopMove();
	initSensorHistory();
	currentTraceAction = TRACE_STRAIGHT;
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 復路エリア 4 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 3 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_04(void) {
    int sensorPattern = BIT_000000;
	int findAnySensorCount = 0;

	// センサーのいずれかが白判定するまで、直進継続
	while (1) {
		StraightMove();
		sensorPattern = getSensorPattern();
		// 3回連続して白判定したらループを抜ける
		if (sensorPattern != BIT_000000) {
			findAnySensorCount++;
			if (findAnySensorCount > 2) {
				break;
			}
		}
	}

	// 停止実行
	StopMove();

	// センサー値に応じて旋回を実行
	// TODO:実装

	// 旋回終了後、停止実行
	StopMove();
	initSensorHistory();
	currentTraceAction = TRACE_STRAIGHT;
}

/*
 * 宝物 2 のトレース動作
 * @return なし
 * @condition
 *   開始条件：
 *   終了条件：
 */
 void treasureHunt_02(void) {
}

/*
 * 復路エリア 6 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 5 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_06(void) {
	static int counter = 0;
	BaseSpeed = 50;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(counter, BaseSpeed);
	}

	// 右旋回実行
	currentTraceAction = executeRightTurn();
	BaseSpeed = 50;
}

/*
 * 復路エリア 7 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 6 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_07(void) {
	static int counter = 0;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(counter, BaseSpeed);
	}

	// 右旋回実行
	currentTraceAction = executeRightTurn();
}

/*
 * 復路エリア 8 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 7 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_08(void) {
	static int counter = 0;
	static int maxSpeed = BASE_SPEED_INIT_VAL;
    int sensorPattern = BIT_111111;
	int findAnySensorCount = 0;

	// ラインが途切れるまではトレース動作
	while (sensorPattern != BIT_000000) {
		traceCommon(counter, BaseSpeed);
		sensorPattern = getSensorPattern();
	}

	// 停止実行
	StopMove();

	// センサーのいずれかが白判定するまで、直進継続
	while (1) {
		StraightMove();
		sensorPattern = getSensorPattern();

		// 3回連続して白判定したらループを抜ける
		if (sensorPattern != BIT_000000) {
			findAnySensorCount++;
			if (findAnySensorCount > 2) {
				break;
			}
		}
	}

	// ちょっと進んで停止
	adjustTurnPosition();

	// 右旋回実行
	executeRightTurnFromOnLine();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 復路エリア 9 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 8 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_09(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 右旋回実行
	currentTraceAction = executeRightTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 復路エリア 10 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 9 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_10(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 停止実行
	StopMove();
	initSensorHistory();
	currentTraceAction = TRACE_STRAIGHT;
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 復路エリア 11 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 10 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_11(void) {
	int sensorPattern = BIT_000000;
	int findAnySensorCount = 0;

	// センサーのいずれかが白判定するまで、直進継続
	while (1) {
		StraightMove();
		sensorPattern = getSensorPattern();
		// 3回連続して白判定したらループを抜ける
		if (sensorPattern != BIT_000000) {
			findAnySensorCount++;
			if (findAnySensorCount > 2) {
				break;
			}
		}
	}

	// 停止実行
	StopMove();

	// センサー値に応じて旋回を実行
	// TODO:実装

	// 旋回終了後、停止実行
	StopMove();
	initSensorHistory();
	currentTraceAction = TRACE_STRAIGHT;
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 復路エリア 12 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 11 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_12(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 停止実行
	StopMove();
	initSensorHistory();
	currentTraceAction = TRACE_STRAIGHT;
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 復路エリア 13 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 12 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_13(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	while (counter > 1500) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 停止実行
	StopMove();
	initSensorHistory();
	currentTraceAction = TRACE_STRAIGHT;
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 宝物 3 のトレース動作
 * @return なし
 * @condition
 *   開始条件：
 *   終了条件：
 */
 void treasureHunt_03(void) {
}

/*
 * 復路エリア 14 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 13 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_14(void) {
}

/*
 * 復路エリア 15 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 14 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_15(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 左旋回実行
	currentTraceAction = executeLeftTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 復路エリア 16 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 15 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_16(void) {
	static int counter = 0;
	static int maxSpeed = BASE_SPEED_INIT_VAL;
	int sensorPattern = BIT_111111;
	int findAnySensorCount = 0;

	// ラインが途切れるまではトレース動作
	while (sensorPattern != BIT_000000) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 停止実行
	StopMove();

	// センサーのいずれかが白判定するまで、直進継続
	while (1) {
		StraightMove();
		sensorPattern = getSensorPattern();

		// 3回連続して白判定したらループを抜ける
		if (sensorPattern != BIT_000000) {
			findAnySensorCount++;
			if (findAnySensorCount > 2) {
				break;
			}
		}
	}

	// 停止実行
	StopMove();

	// 左旋回実行
	currentTraceAction = executeLeftTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 復路エリア 17 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 16 のトレース動作から継続）。
 *   終了条件：
 */
void traceBackwardArea_17(void) {
	static int counter = 0;
	static int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(counter, maxSpeed);
		counter++;
	}

	// 右旋回実行
	currentTraceAction = executeRightTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 復路エリア 18 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 17 のトレース動作から継続）。
 *   終了条件：ゴールエリアを検出して終了動作が完了する。
 */
void traceBackwardArea_18(void) {
}

#ifdef ENABLE_AVRTIMER
/**
 * センサー値のBitパターンを取得する。
 * @brief センサー値を参照し、対応するアクションを取得する。
 * @return 戻り値の説明
 */
int getSensorPatternCalledFromTimer(void) {
	// センサー値を取得
	getSensors();
	curSensorPattern = IR_BitPattern;
	return curSensorPattern;
}

/**
 * センサー値のBitパターンを取得する。
 * @brief センサー値を参照し、対応するアクションを取得する。
 * @return 戻り値の説明
 */
int getSensorPattern(void) {
	return curSensorPattern;
}

#else // ENABLE_AVRTIMER

/**
* センサー値のBitパターンを取得する。
* @brief センサー値を参照し、対応するアクションを取得する。
* @return 戻り値の説明
*/
int getSensorPattern(void) {
	// LEDを設定
	//setLED();
	
	// センサー値を取得
	getSensors();
	return IR_BitPattern;
}

#endif // ENABLE_AVRTIMER

/**
* Bitパターンの履歴を初期化する。
* @brief Bitパターンの履歴を直進のBitパターンで初期化する。
* @return 戻り値の説明
*/
void initSensorHistory(void) {
	memset(IR_BitPatternHistory, TRACE_STRAIGHT, sizeof(IR_BitPatternHistory));
}

/**
* 履歴管理を使ったセンサー値のBitパターンを取得する。
* @brief センサー値を参照し、対応するアクションを取得する。
* @return 戻り値の説明
*/
int getActionWithHistory(void) {
	static int ptn = 0;
	int maxIndex = 0;
	int maxCount = -1;
	int actionPattern = 0;
	int patternCounter[] = {
		0,	// [0]: L_TURN
		0,	// [1]: L_ROUND_TIGHT
		0,	// [2]: L_ROUND_MIDDLE
		0,	// [3]: L_ROUND_SOFT
		0,	// [4]: STRAIGHT
		0,	// [5]: R_ROUND_SOFT
		0,	// [6]: R_ROUND_MIDDLE
		0,	// [7]: R_ROUND_TIGHT
		0,	// [8]: R_TURN
		0	// [9]: UNDEFINED
	};

	// 履歴からパターンを集計する
	for (int i = 0; i < HISTORY_MAXSIZE; i++) {
		actionPattern = ActionTable[IR_BitPatternHistory[i]];
		switch (actionPattern) {
			case TRACE_L_TURN:
				patternCounter[L_TURN]++;
				break;
			case TRACE_L_ROUND_TIGHT:
				patternCounter[L_ROUND_TIGHT]++;
				break;
			case TRACE_L_ROUND_MIDDLE:
				patternCounter[L_ROUND_MIDDLE]++;
				break;
			case TRACE_L_ROUND_SOFT:
				patternCounter[L_ROUND_SOFT]++;
				break;
			case TRACE_STRAIGHT:
				patternCounter[STRAIGHT]++;
				break;
			case TRACE_R_ROUND_SOFT:
				patternCounter[R_ROUND_SOFT]++;
				break;
			case TRACE_R_ROUND_MIDDLE:
				patternCounter[R_ROUND_MIDDLE]++;
				break;
			case TRACE_R_ROUND_TIGHT:
				patternCounter[R_ROUND_TIGHT]++;
				break;
			case TRACE_R_TURN:
				patternCounter[R_TURN]++;
				break;
			default:
				patternCounter[UNDEFINED]++;
				break;
		}
	}
	
	maxCount = -1;
	// 一番多かったパターンを採用する
	for (int j = 0; j < 10; j++) {
		if (patternCounter[j] > maxCount) {
			maxIndex = j;
			maxCount = patternCounter[j];
		}
	}

	switch (maxIndex) {
		case L_TURN:
		    LOG_DEBUG("TRACE_L_TURN: maxCount = %d\r\n", maxCount);
			ptn = TRACE_L_TURN;
			break;
		case L_ROUND_TIGHT:
		    LOG_DEBUG("TRACE_L_ROUND_TIGHT: maxCount = %d\r\n", maxCount);
			ptn = TRACE_L_ROUND_TIGHT;
			break;
		case L_ROUND_MIDDLE:
		    LOG_DEBUG("TRACE_L_ROUND_MIDDLE: maxCount = %d\r\n", maxCount);
			ptn = TRACE_L_ROUND_MIDDLE;
			break;
		case L_ROUND_SOFT:
		    LOG_DEBUG("TRACE_L_ROUND_SOFT: maxCount = %d\r\n", maxCount);
			ptn = TRACE_L_ROUND_SOFT;
			break;
		case STRAIGHT:
		    LOG_DEBUG("TRACE_STRAIGHT: maxCount = %d\r\n", maxCount);
			ptn = TRACE_STRAIGHT;
			break;
		case R_ROUND_SOFT:
		    LOG_DEBUG("TRACE_R_ROUND_SOFT: maxCount = %d\r\n", maxCount);
			ptn = TRACE_R_ROUND_SOFT;
			break;
		case R_ROUND_MIDDLE:
		    LOG_DEBUG("TRACE_R_ROUND_MIDDLE: maxCount = %d\r\n", maxCount);
			ptn = TRACE_R_ROUND_MIDDLE;
			break;
		case R_ROUND_TIGHT:
		    LOG_DEBUG("TRACE_R_ROUND_TIGHT: maxCount = %d\r\n", maxCount);
			ptn = TRACE_R_ROUND_TIGHT;
			break;
		case R_TURN:
		    LOG_DEBUG("TRACE_R_TURN: maxCount = %d\r\n", maxCount);
			ptn = TRACE_R_TURN;
			break;
		default: 
		    LOG_DEBUG("currentTraceAction[%d]: maxCount = %d\r\n", currentTraceAction, maxCount);
			ptn = TRACE_UNDEFINED;
			break;
	}

	return ptn;
}

/**
 * センサー値を取得
 * @brief センサー値を取得
 * @return なし
 * @detail センサー値を取得し、IR[]およびIR_BitPatternを更新する。
 */
void getSensors(void) {
	/* 現在のカウンタ値を更新 */
    currentCount = ((currentCount + 1) % HISTORY_MAXSIZE);
	/* センサー値を取得 */
    ReadIRSensors(IR);
	
	/* IR状態をBITパターンに変換 */
	IR_BitPattern = 0;
	if (IR[RIGHT_OUTSIDE] <= COMPARE_VALUE)	IR_BitPattern |= BIT_RIGHT_OUTSIDE_ON;
	if (IR[RIGHT_CENTER]  <= COMPARE_VALUE)	IR_BitPattern |= BIT_RIGHT_CENTER_ON;
	if (IR[RIGHT_INSIDE]  <= COMPARE_VALUE)	IR_BitPattern |= BIT_RIGHT_INSIDE_ON;
	if (IR[LEFT_INSIDE]   <= COMPARE_VALUE)	IR_BitPattern |= BIT_LEFT_INSIDE_ON;
	if (IR[LEFT_CENTER]   <= COMPARE_VALUE)	IR_BitPattern |= BIT_LEFT_CENTER_ON;
	if (IR[LEFT_OUTSIDE]  <= COMPARE_VALUE)	IR_BitPattern |= BIT_LEFT_OUTSIDE_ON;

	IR_BitPatternHistory[currentCount] = IR_BitPattern;
	
    LOG_INFO("sensor %3d: %3d: %3d: %3d: %3d: %3d \r\n",
			 IR[LEFT_OUTSIDE], IR[LEFT_CENTER], IR[LEFT_INSIDE],
			 IR[RIGHT_INSIDE], IR[RIGHT_CENTER], IR[RIGHT_OUTSIDE]);
    LOG_DEBUG("IR[L %1d%1d%1d%1d%1d%1d R]\r\n",
			  ((IR[LEFT_OUTSIDE]  <= COMPARE_VALUE) ? 1 : 0),
			  ((IR[LEFT_CENTER]   <= COMPARE_VALUE) ? 1 : 0),
			  ((IR[LEFT_INSIDE]   <= COMPARE_VALUE) ? 1 : 0),
			  ((IR[RIGHT_INSIDE]  <= COMPARE_VALUE) ? 1 : 0),
			  ((IR[RIGHT_CENTER]  <= COMPARE_VALUE) ? 1 : 0),
			  ((IR[RIGHT_OUTSIDE] <= COMPARE_VALUE) ? 1 : 0));
}

/**
* ゴール到達時の処理
* @brief ゴール到達時の処理
* @return なし
*/
void executeFinalAction(void)
{
	LOG_INFO("executeFinalAction!!\r\n");
	StopMove();
	_delay_ms(1000);

	/* ゆっくり後進 */
	MotorControl(RIGHT_MOTOR, 40);
	MotorControl(LEFT_MOTOR, 1063);
	_delay_ms(500);//！要調整
	StopMove();//停止を実行
	_delay_ms(10);

	/* 200度くらい右回りで回転 */
	MotorControl(RIGHT_MOTOR, 75);
	MotorControl(LEFT_MOTOR, 75);
	_delay_ms(1200);//！要調整
	StopMove();
	_delay_ms(10);

	/* 荷台を傾けて宝物を落とす */
	dumpTreasures();
	_delay_ms(10);
	
	/* ゆっくり前進 */
	MotorControl(RIGHT_MOTOR, 1063);
	MotorControl(LEFT_MOTOR, 40);
	_delay_ms(500);
	StopMove();//停止を実行
}

/************************************************************************/
// 荷台用モータの初期設定
// 荷台用モーターを少し進行方向に傾ける。
/************************************************************************/
void initCargoBedMotor(void) {
	GetCurrentAngle(CARGO_BED_MOTOR);
	//最大速度で、642の位置へ動かす
	MotorControlJoint( CARGO_BED_MOTOR, 0, 550 );//！要調整
}

/************************************************************************/
// 宝物を落とす
/************************************************************************/
void dumpTreasures(void) {
	_delay_ms(1000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( CARGO_BED_MOTOR, 30, 352 );//モーターを後方にゆっくり傾ける！要調整
	_delay_ms(6000);//6秒継続
	MotorControlJoint( CARGO_BED_MOTOR, 100, 512 );//モーターをセンター位置に戻す！要調整
	_delay_ms(3000);//3秒待つ⇒動作に合わせて変更してください

}

/************************************************************************/
// 左右大き目にロボットを揺らしたジグザグなライントレースを実行
/************************************************************************/
void TreasureFindingLineTrace(int isFirst) {
    int sensorPattern = BIT_000000;
	BaseSpeed = 80;

    sensorPattern = getSensorPattern();

	// 初回動作の場合
	if (isFirst == 0) {
		if (sensorPattern == BIT_100000 ||
			sensorPattern == BIT_010000 ||
			sensorPattern == BIT_110000 ||
			sensorPattern == BIT_001000 ||
			sensorPattern == BIT_101000 ||
			sensorPattern == BIT_011000 ||
			sensorPattern == BIT_111000) {
			isSearchingLeftSide = 1;
			Execute(TRACE_L_TRESURE_FIND);
		} else {
			isSearchingLeftSide = 0;
			Execute(TRACE_R_TRESURE_FIND);
		}

		_delay_ms(10);
		return;
	}

	if ((isSearchingLeftSide == 0) &&
		(sensorPattern == BIT_010000 ||
		 sensorPattern == BIT_001000 ||
		 sensorPattern == BIT_011000 ||
		 sensorPattern == BIT_011100)) {
		// ラインの右側サーチ中に左ラインを検出したら、
		// ラインの左側サーチに切り替える
		isSearchingLeftSide = 1;
		Execute(TRACE_L_TRESURE_FIND);
	} else if (
		(isSearchingLeftSide > 0) &&
		(sensorPattern == BIT_000010 ||
		 sensorPattern == BIT_000100 ||
		 sensorPattern == BIT_000110 ||
		 sensorPattern == BIT_001110)) {
		// ラインの左側サーチ中に右ラインを検出したら、
		// ラインの右側サーチに切り替える
		 isSearchingLeftSide = 0;
		 Execute(TRACE_R_TRESURE_FIND);
	}
		
    _delay_ms(10);
}

/************************************************************************/
// トレースライン上からの左旋回を行う。
// 実行前に走行モータを停止しておくこと。
// 旋回後、中央のセンサーがトレースラインを検出したら処理を終了する。
/************************************************************************/
void executeLeftTurnFromOnLine(void) {
    int sensorPattern = BIT_000000;

	// 左旋回実行
	LeftTurnMove();
    
    // 旋回開始時にラインセンサーがラインを読み取る位置に居るはずなので
    // センサーがライン外まで通過していることを確認する。
	while(1) {
    	sensorPattern = getSensorPattern();

    	//右センサーを検出しているか確認する
    	if (sensorPattern == BIT_000001) {
            // 旋回を継続して抜ける（できればここで確実に抜けたい）
            _delay_ms(100);//センサーが全て通過するまで旋回
            break;
        } else if (sensorPattern == BIT_000010) {
            // 旋回を継続して抜ける
            _delay_ms(500);//センサーが全て通過するまで旋回
            break;
        } else if (sensorPattern == BIT_000110) {
            // 旋回を継続して抜ける（予備）
            //_delay_ms(100);
            //break;
    	}
        //_delay_ms(10);//ループの待ち時間を必要に応じて設定
	}
    
    // 旋回動作の復帰動作
    while(1) {
    	sensorPattern = getSensorPattern();

    	//旋回動作を抜けるための条件を判定
    	if (
            sensorPattern == BIT_011100 || sensorPattern == BIT_001110 ||
            sensorPattern == BIT_001000 || sensorPattern == BIT_001100 ||
            sensorPattern == BIT_000100 || sensorPattern == BIT_000110 ||
            sensorPattern == BIT_000010
    	) {
        	LED_on(2);
        	//中央のセンサーが白なら停止を実行
        	stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
        	break;
    	}

    	//左センサーを検出しているか確認する
    	if (sensorPattern == BIT_100000) {
        	//左センサーを検出したら旋回速度を落とす
        	LeftTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
    	}
	}
    
    //旋回停止判定後の止まった位置でセンサーが中央４個のいずれかなら逆旋回終了
    sensorPattern = getSensorPattern();
    if (sensorPattern == BIT_010000) {
        //左センサーなので、左曲りに設定して抜ける
        Execute(TRACE_L_ROUND_MIDDLE);
        _delay_ms(50);
        return;
    } else if (sensorPattern == BIT_011000) {
        //左センサーなので、左曲りに設定して抜ける
        Execute(TRACE_L_ROUND_SOFT);
        _delay_ms(50);
        return;
    } else if (sensorPattern == BIT_001000) {
        //中央センサーなので、直進に設定して抜ける
        Execute(TRACE_STRAIGHT);
        _delay_ms(50);
        return;
    } else if (sensorPattern == BIT_001100) {
        //中央センサーなので、直進に設定して抜ける
        Execute(TRACE_STRAIGHT);
        _delay_ms(50);
        return;
    } else if (sensorPattern == BIT_000100) {
        //中央センサーなので、直進に設定して抜ける
        Execute(TRACE_STRAIGHT);
        _delay_ms(50);
        return;
    } else if (sensorPattern == BIT_000110) {
        //右センサーなので、右曲りに設定して抜ける
        Execute(TRACE_R_ROUND_SOFT);
        _delay_ms(50);
        return;
    } else if (sensorPattern == BIT_000010) {
        //右センサーなので、右曲りに設定して抜ける
        Execute(TRACE_R_ROUND_MIDDLE);
        _delay_ms(50);
        return;
    }
    
    // 旋回停止判定中にセンサーがラインを通り越した想定
    //センサーを中央に戻すため遅い旋回を実行
    RightTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
    	sensorPattern = getSensorPattern();

    	//旋回動作を抜けるための条件を判定
        // 精度は落ちるが、とりあえず旋回を抜ける
	    if (sensorPattern == BIT_000010) {
    	    //右センサーなので、右曲りに設定して抜ける
            Execute(TRACE_R_ROUND_MIDDLE);
            _delay_ms(50);
    	    return;
    	} else if (sensorPattern == BIT_000110) {
    	    //右センサーなので、右曲りに設定して抜ける
            Execute(TRACE_R_ROUND_SOFT);
            _delay_ms(50);
    	    return;
    	} else if (sensorPattern == BIT_000100) {
    	    //中央センサーなので、直進に設定して抜ける
            Execute(TRACE_STRAIGHT);
            _delay_ms(50);
    	    return;
    	} else if (sensorPattern == BIT_001100) {
    	    //中央センサーなので、直進に設定して抜ける
            Execute(TRACE_STRAIGHT);
            _delay_ms(50);
    	    return;
    	} else if (sensorPattern == BIT_001000) {
    	    //中央センサーなので、直進に設定して抜ける
            Execute(TRACE_STRAIGHT);
            _delay_ms(50);
    	    return;
    	} else if (sensorPattern == BIT_011000) {
    	    //左センサーなので、左曲りに設定して抜ける
            Execute(TRACE_L_ROUND_SOFT);
            _delay_ms(50);
    	    return;
    	} else if (sensorPattern == BIT_010000) {
    	    //左センサーなので、左曲りに設定して抜ける
            Execute(TRACE_L_ROUND_MIDDLE);
            _delay_ms(50);
    	    return;
	    }
	}

    // あきらめて直進で抜ける
    Execute(TRACE_STRAIGHT);
    _delay_ms(20);
    return;
}

/************************************************************************/
// トレースライン上からの右旋回を行う。
// 実行前に走行モータを停止しておくこと。
// 旋回後、中央のセンサーがトレースラインを検出したら処理を終了する。
/************************************************************************/
void executeRightTurnFromOnLine(void) {
    int sensorPattern = BIT_000000;

    // 左旋回実行
    RightTurnMove();
    
    // 旋回開始時にラインセンサーがラインを読み取る位置に居るはずなので
    // センサーがライン外まで通過していることを確認する。
    while(1) {
        sensorPattern = getSensorPattern();

        //右センサーを検出しているか確認する
        if (sensorPattern == BIT_100000) {
            // 旋回を継続して抜ける（できればここで確実に抜けたい）
            _delay_ms(100);//センサーが全て通過するまで旋回
            break;
        } else if (sensorPattern == BIT_010000) {
            // 旋回を継続して抜ける
            _delay_ms(500);//センサーが全て通過するまで旋回
            break;
        } else if (sensorPattern == BIT_011000) {
            // 旋回を継続して抜ける（予備）
            //_delay_ms(100);
            //break;
        }
        //_delay_ms(10);//ループの待ち時間を必要に応じて設定
    }
    
    // 旋回動作の復帰動作
    while(1) {
        sensorPattern = getSensorPattern();

        //旋回動作を抜けるための条件を判定
        if (
        sensorPattern == BIT_001110 || sensorPattern == BIT_011100 ||
        sensorPattern == BIT_000010 || sensorPattern == BIT_000110 ||
        sensorPattern == BIT_000100 || sensorPattern == BIT_001100 ||
        sensorPattern == BIT_010000
        ) {
            LED_on(2);
            //中央のセンサーが白なら停止を実行
            stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
            break;
        }

        //右センサーを検出しているか確認する
        if (sensorPattern == BIT_000001) {
            //右センサーを検出したら旋回速度を落とす
            LeftTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
        }
    }
    
    //旋回停止判定後の止まった位置でセンサーが中央４個のいずれかなら逆旋回終了
    sensorPattern = getSensorPattern();
    if (sensorPattern == BIT_000010) {
        //左センサーなので、左曲りに設定して抜ける
        Execute(TRACE_L_ROUND_MIDDLE);
        _delay_ms(50);
        return;
        } else if (sensorPattern == BIT_011000) {
        //左センサーなので、左曲りに設定して抜ける
        Execute(TRACE_L_ROUND_SOFT);
        _delay_ms(50);
        return;
        } else if (sensorPattern == BIT_001000) {
        //中央センサーなので、直進に設定して抜ける
        Execute(TRACE_STRAIGHT);
        _delay_ms(50);
        return;
        } else if (sensorPattern == BIT_001100) {
        //中央センサーなので、直進に設定して抜ける
        Execute(TRACE_STRAIGHT);
        _delay_ms(50);
        return;
        } else if (sensorPattern == BIT_000100) {
        //中央センサーなので、直進に設定して抜ける
        Execute(TRACE_STRAIGHT);
        _delay_ms(50);
        return;
        } else if (sensorPattern == BIT_000110) {
        //右センサーなので、右曲りに設定して抜ける
        Execute(TRACE_R_ROUND_SOFT);
        _delay_ms(50);
        return;
        } else if (sensorPattern == BIT_000010) {
        //右センサーなので、右曲りに設定して抜ける
        Execute(TRACE_R_ROUND_MIDDLE);
        _delay_ms(50);
        return;
    }
    
    // 旋回停止判定中にセンサーがラインを通り越した想定
    //センサーを中央に戻すため遅い旋回を実行
    RightTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
    while(1) {
        sensorPattern = getSensorPattern();

        //旋回動作を抜けるための条件を判定
        // 精度は落ちるが、とりあえず旋回を抜ける
        if (sensorPattern == BIT_000010) {
            //右センサーなので、右曲りに設定して抜ける
            Execute(TRACE_R_ROUND_MIDDLE);
            _delay_ms(50);
            return;
        } else if (sensorPattern == BIT_000110) {
            //右センサーなので、右曲りに設定して抜ける
            Execute(TRACE_R_ROUND_SOFT);
            _delay_ms(50);
            return;
        } else if (sensorPattern == BIT_000100) {
            //中央センサーなので、直進に設定して抜ける
            Execute(TRACE_STRAIGHT);
            _delay_ms(50);
            return;
        } else if (sensorPattern == BIT_001100) {
            //中央センサーなので、直進に設定して抜ける
            Execute(TRACE_STRAIGHT);
            _delay_ms(50);
            return;
        } else if (sensorPattern == BIT_001000) {
            //中央センサーなので、直進に設定して抜ける
            Execute(TRACE_STRAIGHT);
            _delay_ms(50);
            return;
        } else if (sensorPattern == BIT_011000) {
            //左センサーなので、左曲りに設定して抜ける
            Execute(TRACE_L_ROUND_SOFT);
            _delay_ms(50);
            return;
        } else if (sensorPattern == BIT_010000) {
            //左センサーなので、左曲りに設定して抜ける
            Execute(TRACE_L_ROUND_MIDDLE);
            _delay_ms(50);
            return;
        }
    }

    // あきらめて直進で抜ける
    Execute(TRACE_STRAIGHT);
    _delay_ms(20);
    return;
}

/**
 * 速度が0～入力値以下になるまで、停止動作を継続する 
 * @param maxVal 停止判定の上限値
 */
void stopMoveLessThanVal(int maxVal){
	StopMove();//停止を実行
	int judgeSpeed = 0;
	while(1) {
		judgeSpeed = GetCurrentSpeedR();//モーターの速度を取得
		if( (judgeSpeed >= 0 && judgeSpeed <= maxVal) ||
		  (judgeSpeed >= 1024 && judgeSpeed <= (1024 + maxVal)) ) {
			//速度がmaxVal以下ならstop()抜ける
			break;
		}
	}
}

/**
 * 左旋回実行
 * 旋回動作をさせて、センサーが中央になったら直進を指定して抜ける
 */
int executeLeftTurn(void){
	int sensorPattern = BIT_000000;

	//旋回判定されたら停止を実行
	initLeftTurnAction(STOP_JUDGE_MAX_LIMIT);
	LED_on(1);

	//停止が確定したらベース速度に応じて、前進or後進を実行
	adjustTurnPosition();

	// 左旋回実行
    LeftTurnMove();
	while(1) {
		sensorPattern = getSensorPattern();

		//旋回動作を抜けるための条件を判定
		if (
			//sensorPattern == BIT_010000 || sensorPattern == BIT_011000 ||
			sensorPattern == BIT_011100 || sensorPattern == BIT_001110 ||
			sensorPattern == BIT_001000 || sensorPattern == BIT_001100 ||
			sensorPattern == BIT_000100 || sensorPattern == BIT_000110 ||
			sensorPattern == BIT_000010
			) {
			LED_on(2);
			//中央のセンサーが白なら停止を実行
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			break;
		}

   	    //左センサーを検出しているか確認する
   	    if (sensorPattern == BIT_100000) {
       	    //左センサーを検出したら旋回速度を落とす
       	    LeftTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
   	    }
	}

	//旋回停止判定後の止まった位置でセンサーが中央４個のいずれかなら逆旋回終了
	sensorPattern = getSensorPattern();
	if (sensorPattern == BIT_010000) {
	    //左センサーなので、左曲りに設定して抜ける
	    return TRACE_L_ROUND_MIDDLE;
	} else if (sensorPattern == BIT_011000) {
	    //左センサーなので、左曲りに設定して抜ける
	    return TRACE_L_ROUND_SOFT;
	} else if (sensorPattern == BIT_001000) {
		//中央センサーなので、直進に設定して抜ける
	    return TRACE_STRAIGHT;
	} else if (sensorPattern == BIT_001100) {
		//中央センサーなので、直進に設定して抜ける
	    return TRACE_STRAIGHT;
	} else if (sensorPattern == BIT_000100) {
		//中央センサーなので、直進に設定して抜ける
	    return TRACE_STRAIGHT;
	} else if (sensorPattern == BIT_000110) {
	    //右センサーなので、右曲りに設定して抜ける
	    return TRACE_R_ROUND_SOFT;
	} else if (sensorPattern == BIT_000010) {
	    //右センサーなので、右曲りに設定して抜ける
	    return TRACE_R_ROUND_MIDDLE;
	}
	
	LED_on(3);
	//センサーを中央に戻すため遅い旋回を実行
	RightTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
		//逆旋回動作を抜けるための条件を判定
		sensorPattern = getSensorPattern();
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001100 || sensorPattern == BIT_000100) {
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			return TRACE_STRAIGHT;
		} else if ( sensorPattern == BIT_010000 ||	sensorPattern == BIT_011000 ||
					sensorPattern == BIT_100000 ||	sensorPattern == BIT_110000 ) {
			//既に逆側まで旋回していたら（想定よりも早く解除できてしまった場合など）
	        //左センサーなので、左曲りに設定して抜ける
            
	        return TRACE_L_ROUND_MIDDLE;
		}
	}

	//再度センサーを中央に戻すため遅い旋回を実行（ここまでは実行されない想定）
	LED_on(4);
	LeftTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
		//逆旋回動作を抜けるための条件を判定
		sensorPattern = getSensorPattern();
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001100 || sensorPattern == BIT_000100) {
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			return TRACE_STRAIGHT;
		}
	}
	return TRACE_STRAIGHT;
}

/**
 * 右旋回実行
 * 旋回動作をさせて、センサーが中央になったら直進を指定して抜ける
 */
int executeRightTurn(void){
	int sensorPattern = BIT_000000;

	//旋回判定されたら停止を実行
	initRightTurnAction(STOP_JUDGE_MAX_LIMIT);
	LED_on(1);

	//停止が確定したらベース速度に応じて、前進or後進を実行
	adjustTurnPosition();

	// 右旋回実行
	RightTurnMove();
	while(1) {
		sensorPattern = getSensorPattern();

		//旋回動作を抜けるための条件を判定
		if (
			//sensorPattern == BIT_000010 || sensorPattern == BIT_000110 ||
			sensorPattern == BIT_011100 || sensorPattern == BIT_001110 ||
			sensorPattern == BIT_000100 || sensorPattern == BIT_001100 ||
			sensorPattern == BIT_001000 || sensorPattern == BIT_011000 ||
			sensorPattern == BIT_010000
		    ) {
			LED_on(2);
			//中央のセンサーが黒なら停止を実行
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			break;
		}
   	    //右センサーを検出しているか確認する
   	    if (sensorPattern == BIT_000001) {
       	    //右センサーを検出したら旋回速度を落とす
       	    RightTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
   	    }
	}

	//旋回停止判定後の止まった位置でセンサーが中央なら逆旋回終了
    sensorPattern = getSensorPattern();
    if (sensorPattern == BIT_000010) {
        //左センサーなので、左曲りに設定して抜ける
        return TRACE_R_ROUND_MIDDLE;
    } else if (sensorPattern == BIT_000110) {
        //左センサーなので、左曲りに設定して抜ける
        return TRACE_R_ROUND_SOFT;
    } else if (sensorPattern == BIT_000100) {
        //中央センサーなので、直進に設定して抜ける
        return TRACE_STRAIGHT;
    } else if (sensorPattern == BIT_001100) {
        //中央センサーなので、直進に設定して抜ける
        return TRACE_STRAIGHT;
    } else if (sensorPattern == BIT_001000) {
        //中央センサーなので、直進に設定して抜ける
        return TRACE_STRAIGHT;
    } else if (sensorPattern == BIT_011000) {
        //右センサーなので、右曲りに設定して抜ける
        return TRACE_L_ROUND_SOFT;
    } else if (sensorPattern == BIT_010000) {
        //右センサーなので、右曲りに設定して抜ける
        return TRACE_L_ROUND_MIDDLE;
    }
		
	LED_on(3);
	//センサーを中央に戻すため遅い旋回を実行
	LeftTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
		//逆旋回動作を抜けるための条件を判定
		sensorPattern = getSensorPattern();
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001100 || sensorPattern == BIT_000100) {
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			return TRACE_STRAIGHT;
		} else if ( sensorPattern == BIT_000110 || sensorPattern == BIT_000010 ||
					sensorPattern == BIT_000011 || sensorPattern == BIT_000001 ) {
			//既に逆側まで旋回していたら（想定よりも早く解除できてしまった場合）
	        //左センサーなので、左曲りに設定して抜ける
	        return TRACE_R_ROUND_MIDDLE;
		}
	}
		
	//再度センサーを中央に戻すため遅い旋回を実行（ここまでは実行されない想定）
	LED_on(4);
	RightTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
		//逆旋回動作を抜けるための条件を判定
		getSensors();
		sensorPattern = IR_BitPattern;
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001100 || sensorPattern == BIT_000100) {
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			break;
		}
	}
	return TRACE_STRAIGHT;
}

/**
 * 左旋回動作の初期化処理
 * 停止を実行して、
 * 基準以下の速度まで減速できたら、左旋回のステータスを返す
 */
int initLeftTurnAction(int referenceVal) {

	StopMove();//停止を実行
	int judgeSpeed = 0;
	while(1) {
		judgeSpeed = GetCurrentSpeedR();//モーターの速度を取得
		if( (judgeSpeed >= 0 && judgeSpeed <= referenceVal) ||
			(judgeSpeed >= 1024 && judgeSpeed <= (1024 + referenceVal)) ) {
			//速度がmaxVal以下ならstop()抜ける
			return TRACE_L_TURN;
		}
	}
}

/**
 * 右旋回動作の初期化
 * 停止を実行して、
 * 基準以下の速度まで減速できたら、右旋回のステータスを返す
 */
int initRightTurnAction(int referenceVal) {

	StopMove();//停止を実行
	int judgeSpeed = 0;
	while(1) {
		judgeSpeed = GetCurrentSpeedR();//モーターの速度を取得
		if( (judgeSpeed >= 0 && judgeSpeed <= referenceVal) ||
			(judgeSpeed >= 1024 && judgeSpeed <= (1024 + referenceVal)) ) {
			//速度がmaxVal以下ならstop()抜ける
			return TRACE_R_TURN;
		}
	}
}

/**
 * 旋回に入ったベース速度に応じて、位置を調整する。
 * 2017ロボに合わせて調整必要！
 */
void adjustTurnPosition(void) {
	if (BaseSpeed <= 50 ) {
		StraightLowMove();
		_delay_ms(150);	// 150ms 間隔を空ける
	} else if (BaseSpeed <= 100 ) {
		StraightLowMove();
		_delay_ms(100);	// 100ms 間隔を空ける
	} else if (BaseSpeed <= 120 ) {
		StraightLowMove();
		_delay_ms(80);	// 80ms 間隔を空ける
	} else if (BaseSpeed <= 140 ) {
		StraightLowMove();
		_delay_ms(60);	// 60ms 間隔を空ける
	} else if (BaseSpeed <= 160 ) {
		StraightLowMove();
		_delay_ms(40);	// 40ms 間隔を空ける
	} else if (BaseSpeed <= 180 ) {
		StraightLowMove();
		_delay_ms(20);	// 20ms 間隔を空ける
	}
    StopMove();
}

void initEmergencyStop(void) {
    DDRD  = 0x70;
    PORTD = 0x11;
}

/**
 * LEDを設定
 * @brief LEDを設定
 * @param (引数名) 引数の説明
 * @param (引数名) 引数の説明
 * @return 戻り値の説明
 * @sa 参照すべき関数を書けばリンクが貼れる
 * @detail 詳細な説明
 */
void setLED(void) {
#ifdef _LED_ON_
	//マイコンのレジスタ(DDRC)の設定
    DDRC  = 0x7F;
	//マイコンの出力ポート(PORTC)の設定
    PORTC = 0x7F;
#endif // _LED_ON_
}

/**
 * LED点灯
 * @brief LED点灯
 * @param (int i) LEDの番号
 * @return なし
 */
void LED_on(int i) {
#ifdef _LED_ON_
    if (i < 1 || i > 6) return;
	unsigned char c = PORTC;
	//マイコンの出力ポート(PORTC)の設定
    PORTC = c^(1<<i);
#endif // _LED_ON_
}

/**
 * LED消灯
 * @brief LED消灯
 * @param (int i) LEDの番号
 * @return なし
 */
void LED_off(int i) {
#ifdef _LED_ON_
    if (i < 1 || i > 6) return;
	unsigned char c = PORTC;
	//マイコンの出力ポート(PORTC)の設定
    PORTC = ~(c^(1<<i));
#endif // _LED_ON_
}
