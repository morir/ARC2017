//##########################################################
//##                      R O B O T I S                   ##
//## CM-700 (Atmega2561) Example code for Dynamixel.      ##
//##                                           2009.11.10 ##
//##########################################################

#define F_CPU   16000000L   //16MHz
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include "SerialManager.h"
#include "include/dynamixel.h"
#include "SensorManager.h"
#include "MotorManager.h"
#include "DebugLog.h"

#include "pid.h"

// ------------------ Defined ------------------
// Line Sensor
#define LINE_STATE_BLACK    0//センサー値でラインが白判定
#define LINE_STATE_WHITE    1//センサー値でラインが黒判定

#define _LED_ON_
//#define _MODE_SKIP_			// ショートカットモード

#define DELAY_MAX_TIME      (100)//delay時間の最大値(ミリ秒)
#define STOP_JUDGE_MAX_LIMIT	(10)//停止判定の上限値
#define SLOW_TURN_RATE_BY_BASE	(50)//ベースの20%の速さ
#define HISTORY_MAXSIZE (5)//履歴管理最大数

#define PATTERN_L_ROUND_TIGHT (-3)
#define PATTERN_L_ROUND_MIDDLE (-2)
#define PATTERN_L_ROUND_SOFT (-1)
#define PATTERN_R_ROUND_SOFT (1)
#define PATTERN_R_ROUND_MIDDLE (2)
#define PATTERN_R_ROUND_TIGHT (3)

// ------------------ Method Definition ------------------
void executeTraceProcess(void);
void traceForwardArea_01(void);
void traceForwardArea_02(void);
void traceForwardArea_03(void);
void traceForwardArea_04(void);
void traceForwardArea_05(void);
void traceForwardArea_06(void);
void traceTurnAroundPoint(void);
void traceBackwardArea_01(void);
void traceBackwardArea_02(void);
void traceBackwardArea_03(void);
void traceBackwardArea_04(void);
void traceBackwardArea_05(void);
void traceBackwardArea_06(void);

int isRightRound(void);
int isLeftRound(void);
int isStraightDetected(int sensor);
int isLeftInsideDetected(int sensor);
int isRightInsideDetected(int sensor);
int isDetectedNothing(int sensor);
int doesNeedToResetSpeed(void);
int getSensorPattern(void);
void initSensorHistory(void);
int getSensorPatternWithHistory(void);
void initPETbottlesMotor(void);
void placePETbottles(void);
void stopMoveLessThanVal(int val);

void initDumpMotor(void);
void TraceFormation(void);
void FindFormation(void);
void CatchAndReleaseFormation(void);
void executeRotate(int motorId, int speed, int angle, int targetangle);

void getSensors(void);

int executeLeftTurn(void);
int executeRightTurn(void);
void executeRound(void);
int needChangedSmooth(void);
int getSmoothAction(void);

int initLeftTurnAction(int maxVal);
int initRightTurnAction(int maxVal);
void adjustTurnPosition(void);
void executeDelay(void);
void executeFinalAction(void);

void initEmergencyStop(void);
void executeSkipAction(void);

void setLED(void);
void LED_on(int i);
void LED_off(int i);

// ------------------ Global Variables Definition ------------------

// Serial Message Buffer
int serCmd[SERIAL_BUFFER_SIZE] = {0};

// Goal Judgment counter
int goalCounter = 0;

// Move State
//int mCurrentAction = MOVE_SELECTION_TYPE_STRAIGHT;
//
//// Next Move State
//int mBeforeMoveState = MOVE_SELECTION_TYPE_STRAIGHT;

// IR Sensor 
unsigned int IR[ADC_PORT_6 + 1] = {0,0,0,0,0,0,0};

// IRの状態(BITパターン)
int IR_BitPattern = 0;
int IR_BitPatternHistory[HISTORY_MAXSIZE] =
 { TRACE_STRAIGHT, TRACE_STRAIGHT, TRACE_STRAIGHT, TRACE_STRAIGHT, TRACE_STRAIGHT };
int currentCount = 0;


int mMoveCount = 1;

// 前々回のトレース動作
int prePrevTraceAction = TRACE_STRAIGHT;
// 前回のトレース動作
int previousTraceAction = TRACE_STRAIGHT;
// 今回のトレース動作
int currentTraceAction = TRACE_STRAIGHT;

// PID Param
float pGain = 200;   //Proportional Gain
float iGain =  0.2;  //Integral Gain
float dGain =  120;  //Differential Gain
int delay = 10;
int32_t eInteg = 0;  //Integral accumulator
int32_t ePrev  =0;   //Previous Error

int PID_ctlr = 0;	//!< PID制御用変数。中心のセンサからの距離を入力することで、直進時のブレを抑制する制御を行う。

// ------------------ Method ------------------

// ------------------ Table ------------------
int ActionTable[] = {
	/* 00:BIT_000000 */	TRACE_STRAIGHT,
	/* 01:BIT_000001 */	TRACE_R_ROUND_TIGHT,
	/* 02:BIT_000010 */	TRACE_R_ROUND_MIDDLE,
	/* 03:BIT_000011 */	TRACE_R_TURN,
	/* 04:BIT_000100 */	TRACE_R_ROUND_SOFT,
	/* 05:BIT_000101 */	TRACE_R_TURN,
	/* 06:BIT_000110 */	TRACE_R_ROUND_MIDDLE,
	/* 07:BIT_000111 */	TRACE_R_TURN,
	/* 08:BIT_001000 */	TRACE_L_ROUND_SOFT,
	/* 09:BIT_001001 */	TRACE_UNDEFINED,
	/* 10:BIT_001010 */	TRACE_UNDEFINED,
	/* 11:BIT_001011 */	TRACE_UNDEFINED,
	/* 12:BIT_001100 */	TRACE_STRAIGHT,
	/* 13:BIT_001101 */	TRACE_UNDEFINED,
	/* 14:BIT_001110 */	TRACE_R_ROUND_MIDDLE,
	/* 15:BIT_001111 */	TRACE_R_TURN,
	/* 16:BIT_010000 */	TRACE_L_ROUND_MIDDLE,
	/* 17:BIT_010001 */	TRACE_UNDEFINED,
	/* 18:BIT_010010 */	TRACE_UNDEFINED,
	/* 19:BIT_010011 */	TRACE_UNDEFINED,
	/* 20:BIT_010100 */	TRACE_UNDEFINED,
	/* 21:BIT_010101 */	TRACE_UNDEFINED,
	/* 22:BIT_010110 */	TRACE_UNDEFINED,
	/* 23:BIT_010111 */	TRACE_UNDEFINED,
	/* 24:BIT_011000 */	TRACE_L_ROUND_MIDDLE,
	/* 25:BIT_011001 */	TRACE_UNDEFINED,
	/* 26:BIT_011010 */	TRACE_UNDEFINED,
	/* 27:BIT_011011 */	TRACE_UNDEFINED,
	/* 28:BIT_011100 */	TRACE_L_ROUND_MIDDLE,
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
	initPETbottlesMotor();
#if(0)
	LOG_DEBUG("Call initDumpMotor() %s\r\n", "");
 	_delay_ms(5000);//1秒待つ⇒動作に合わせて変更してください
	initDumpMotor();
	
	LOG_DEBUG("Call FindFormation() %s\r\n", "");
 	_delay_ms(5000);//1秒待つ⇒動作に合わせて変更してください
	FindFormation();
	
	LOG_DEBUG("Call CatchAndReleaseFormation() %s\r\n", ""); 
	_delay_ms(5000);//1秒待つ⇒動作に合わせて変更してください
	CatchAndReleaseFormation();
	
//  LOG_DEBUG("Call TraceFormation() %s\r\n", "");
//	_delay_ms(5000);//1秒待つ⇒動作に合わせて変更してください
//	TraceFormation();
#endif

	getSensorPattern();

	// ロボ動作開始

    // ショートカットモードを作る場合はここに入れる。
#ifdef _MODE_SKIP_
	executeSkipAction();
#endif /* _MODE_SKIP_ */

	// トレース動作開始
	executeTraceProcess();

    // ゴール判定後の動作実質ここから開始？
	executeFinalAction();
}

/**
* ライントレース動作
* @brief ライントレース動作
* @return なし
* @detail ゴール判定条件を満たすまでライントレース動作を行う。
*/
void executeTraceProcess(void) {
	static int sensorPattern = BIT_000000;
    static int counter = 0;
	
#ifdef _MODE_SKIP_
	//ショートカットモードの場合は、初期動作不要
#else
	//初期動作（少しだけ直進）
	StraightMove();
	_delay_ms(100);	// 10ms 間隔を空ける
#endif /* _MODE_SKIP_ */

	while (1) {

		// センサ値のビットパターンを取得する。
		sensorPattern = getSensorPattern();

		// センサ値のパターンが最終動作であればループを抜ける。
		if (sensorPattern == TRACE_FINALACTION) {
			LED_on(1);
			LED_on(2);
			LED_on(3);
			LED_on(4);
			LED_on(5);
			break;
		}

		// 前回の動作とセンサ値のパターンの組み合わせから今回の動作を仮決定する。
		currentTraceAction = ActionTable[(sensorPattern / 2)];
//		LOG_INFO("(sensorPattern / 2) %3d\r\n", (sensorPattern / 2));
//		LOG_INFO("previousTraceAction %3d: sensorPattern %3d: currentTraceAction: %3d \r\n",
//		         previousTraceAction, sensorPattern, currentTraceAction);

		// LEDを設定
		setLED();

		if(currentTraceAction == TRACE_L_TURN)
		{
			//旋回実行
			currentTraceAction = executeLeftTurn();
			if (currentTraceAction == TRACE_SLOW_STRAIGHT) {
				StraightMove();
				executeDelay();
				sensorPattern = getSensorPattern();
				if(sensorPattern != BIT_000000 ) {
					currentTraceAction = TRACE_STRAIGHT;
				}
			}

			BaseSpeed = BASE_SPEED_INIT_VAL;
			//sensorPattern = getSensorPattern();
			//if(sensorPattern == BIT_111110 ) {
				//StraightMove();
				//LED_on(1);
				//LED_on(2);
				//LED_on(3);
				//LED_on(4);
				//LED_on(5);
				//_delay_ms(250);	// 200ms 間隔を空ける
			//}
		}
		else if (currentTraceAction == TRACE_R_TURN)
		{
			//旋回実行
			currentTraceAction = executeRightTurn();
			if (currentTraceAction == TRACE_SLOW_STRAIGHT) {
				StraightMove();
				executeDelay();
				sensorPattern = getSensorPattern();
				if(sensorPattern != BIT_000000 ) {
					currentTraceAction = TRACE_STRAIGHT;
				}
			}
			BaseSpeed = BASE_SPEED_INIT_VAL;
			sensorPattern = getSensorPattern();
			//if(sensorPattern == BIT_111110 ) {
				//StraightMove();
				//LED_on(1);
				//LED_on(2);
				//LED_on(3);
				//LED_on(4);
				//LED_on(5);
				//_delay_ms(250);	// 200ms 間隔を空ける
			//}
		}
		else if (isLeftRound() || isRightRound()) {
			executeRound();
		}
		else if (previousTraceAction == TRACE_SLOW_STRAIGHT && sensorPattern == BIT_000000) {
			//前回ゆっくり直進でセンサーが白だったら、ゆっくりを継続
			currentTraceAction = TRACE_SLOW_STRAIGHT;
		}
			
		if (doesNeedToResetSpeed()) {
			BaseSpeed = BASE_SPEED_INIT_VAL;
		}
			
		counter++;
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

		Execute(currentTraceAction);

		_delay_ms(1);// delayTimeの間隔を空ける

		// 今回の動作を前回の動作に退避する。
		prePrevTraceAction = previousTraceAction;
		previousTraceAction = currentTraceAction;
	}
}

/*
 * 往路エリア 1 のトレース動作
 * @return なし
 * @condition
 *   開始条件：スタートコマンドを受信する。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
 void traceForwardArea_01(void) {
}

/*
 * 往路エリア 2 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（往路エリア 1 のトレース動作から継続）。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
 void traceForwardArea_02(void) {
}

/*
 * 往路エリア 3 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（往路エリア 2 のトレース動作から継続）。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
 void traceForwardArea_03(void) {
}

/*
 * 往路エリア 4 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（往路エリア 3 のトレース動作から継続）。
 *   終了条件：センサで右ターンを検出して直角旋回が完了する。
 */
 void traceForwardArea_04(void) {
}

/*
 * 往路エリア 5 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（往路エリア 4 のトレース動作から継続）。
 *   終了条件：センサで右ターンを検出して直角旋回が完了する。
 */
 void traceForwardArea_05(void) {
}

/*
 * 往路エリア 6 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（往路エリア 5 のトレース動作から継続）。
 *   終了条件：宝物（白）を検出して回収が完了する。
 */
 void traceForwardArea_06(void) {
}

/*
 * 折り返し点のトレース動作
 * @return なし
 * @condition
 *   開始条件：宝物（白）の回収が完了する。
 *   終了条件：折り返し動作を終了する。
 */
 void traceTurnAroundPoint(void) {
}

/*
 * 復路エリア 1 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（折り返し点のトレース動作から継続）。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
 void traceBackwardArea_01(void) {
}

/*
 * 復路エリア 2 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 1 のトレース動作から継続）。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
 void traceBackwardArea_02(void) {
}

/*
 * 復路エリア 3 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 2 のトレース動作から継続）。
 *   終了条件：センサで右ターンを検出して直角旋回が完了する。
 */
 void traceBackwardArea_03(void) {
}

/*
 * 復路エリア 4 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 3 のトレース動作から継続）。
 *   終了条件：センサで右ターンを検出して直角旋回が完了する。
 */
 void traceBackwardArea_04(void) {
}

/*
 * 復路エリア 5 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 4 のトレース動作から継続）。
 *   終了条件：センサで右ターンを検出して直角旋回が完了する。
 */
 void traceBackwardArea_05(void) {
}

/*
 * 復路エリア 6 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 5 のトレース動作から継続）。
 *   終了条件：ゴールエリアを検出して終了動作が完了する。
 */
 void traceBackwardArea_06(void) {
}

/**
 * 右カーブ動作中か判定する 
 * @return 戻り値
 */
int isRightRound(void) {
	return ((previousTraceAction == TRACE_R_STRAIGHT) ||
			(previousTraceAction == TRACE_R_ROUND_SOFT) ||
			(previousTraceAction == TRACE_R_ROUND_MIDDLE) ||
			(previousTraceAction == TRACE_R_ROUND_TIGHT));
}

/**
 * 左カーブ動作中か判定する 
 * @return 戻り値
 */
int isLeftRound(void) {
	return ((previousTraceAction == TRACE_L_STRAIGHT) ||
			(previousTraceAction == TRACE_L_ROUND_SOFT) ||
			(previousTraceAction == TRACE_L_ROUND_MIDDLE) ||
			(previousTraceAction == TRACE_L_ROUND_TIGHT));
}

/**
 * 中央のセンサでラインを検出したか判定する 
 * @param sensor センサの検出パターン
 * @return 戻り値
 */
int isStraightDetected(int sensor) {
	return ((sensor == BIT_001000) || (sensor == BIT_001001));
}

/**
 * 左内側のセンサでラインを検出したか判定する 
 * @param sensor センサの検出パターン
 * @return 戻り値
 */
int isLeftInsideDetected(int sensor) {
	return ((sensor == BIT_010000) || (sensor == BIT_010001));
}

/**
 * 右内側のセンサでラインを検出したか判定する 
 * @param sensor センサの検出パターン
 * @return 戻り値
 */
int isRightInsideDetected(int sensor) {
	return ((sensor == BIT_000100) || (sensor == BIT_000101));
}

/**
 * センサでラインが未検出か判定する 
 * @param sensor センサの検出パターン
 * @return 戻り値
 */
int isDetectedNothing(int sensor) {
	return ((sensor == BIT_000000) || (sensor == BIT_000001));
}

/**
 * 速度をリセットするか判定する 
 * @return 戻り値
 */
int doesNeedToResetSpeed(void) {
	return ((currentTraceAction == TRACE_L_TURN) || (currentTraceAction == TRACE_R_TURN));
}

/**
* センサー値のBitパターンを取得する。
* @brief センサー値を参照し、対応するアクションを取得する。
* @return 戻り値の説明
*/
int getSensorPattern(void) {
    static int ptn = 0;
	
	// LEDを設定
	//setLED();
	
	// センサー値を取得
	getSensors();
	
	// 判定条件数を減らすためゴール判定用センサ値をフィルタリングする。
	ptn = ((IR_BitPattern >> 1) << 1);

	if (goalCounter >= 50 &&
		((IR_BitPattern == BIT_000011 ) ||
		 (IR_BitPattern == BIT_000111 ) ||
		 (IR_BitPattern == BIT_001111 ) ||
		 (IR_BitPattern == BIT_011111 ) ||
		 (IR_BitPattern == BIT_111111 )
		)){
		ptn = TRACE_FINALACTION;
	}

	return ptn;
}

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
int getSensorPatternWithHistory(void) {
	static int ptn = 0;
	int maxIndex = 0;
	int maxCount = -1;
	static int patternCounter[] = {
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
		switch (IR_BitPatternHistory[i]) {
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
			ptn = TRACE_L_TURN;
			break;
		case L_ROUND_TIGHT:
			ptn = TRACE_L_ROUND_TIGHT;
			break;
		case L_ROUND_MIDDLE:
			ptn = TRACE_L_ROUND_MIDDLE;
			break;
		case L_ROUND_SOFT:
			ptn = TRACE_L_ROUND_SOFT;
			break;
		case STRAIGHT:
			ptn = TRACE_STRAIGHT;
			break;
		case R_ROUND_SOFT:
			ptn = TRACE_R_ROUND_SOFT;
			break;
		case R_ROUND_MIDDLE:
			ptn = TRACE_R_ROUND_MIDDLE;
			break;
		case R_ROUND_TIGHT:
			ptn = TRACE_R_ROUND_TIGHT;
			break;
		case R_TURN:
			ptn = TRACE_R_TURN;
			break;
		default: 
			ptn = currentTraceAction;
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
	_delay_ms(5000);

	/* 200度くらい右回りで回転 */
	MotorControl(RIGHT_MOTOR, 75);
	MotorControl(LEFT_MOTOR, 75);
	_delay_ms(1200);
	StopMove();
	_delay_ms(10);

	/* ペットボトル設置を実行 */
	placePETbottles();
	_delay_ms(10);

	/* ゆっくり後進 */
	MotorControl(RIGHT_MOTOR, 40);
	MotorControl(LEFT_MOTOR, 1063);
	_delay_ms(500);
	StopMove();//停止を実行
	_delay_ms(10);
	
	/* ゆっくり前進 */
	MotorControl(RIGHT_MOTOR, 1063);
	MotorControl(LEFT_MOTOR, 40);
	_delay_ms(500);
	StopMove();//停止を実行
}

/************************************************************************/
// ペットボトル用モータの初期設定
// ペットボトル設置用モーターを少し前方に傾ける。
/************************************************************************/
void initPETbottlesMotor(void) {
	//最大速度で、642の位置へ動かす
	MotorControlJoint( PETBOTTOLE_MOTOR, 0, 642 );
}

/************************************************************************/
// ペットボトル掴む用モーターの初期設定
// ペットボトル掴む用モーターをライントレース用の位置に設定
/************************************************************************/
void initDumpMotor(void) {
//	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
//	MotorControlJoint( WRIST_MOTOR, 100, 512 );
//	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
//	MotorControlJoint( SHOULDER_MOTOR, 100, 410 );
//	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
//	MotorControlJoint( UPPER_ARM_MOTOR, 100, 820 );
//	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
//	MotorControlJoint( FORE_ARM_MOTOR, 100, 615 );

	int correctionValue = 20; // 補正値(目標角度を設定角度の-20に暫定)

	executeRotate(WRIST_MOTOR, 100, 512, 512 - correctionValue);
	executeRotate(SHOULDER_MOTOR, 100, 410, 410 - correctionValue);
	executeRotate(UPPER_ARM_MOTOR, 100, 820, 820 - correctionValue);
	executeRotate(FORE_ARM_MOTOR, 100, 615, 615 - correctionValue);	
}

/************************************************************************/
// ライントレース用形態
// ライントレース用の位置に設定する。
/************************************************************************/
void TraceFormation(void)
{
	initDumpMotor();
}

/************************************************************************/
// お宝検索用形態
// お宝検索用の位置に設定する。
/************************************************************************/
void FindFormation(void)
{
/*
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( FORE_ARM_MOTOR, 100, 205 );
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( SHOULDER_MOTOR, 100, 512 );	
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( UPPER_ARM_MOTOR, 100, 666 );
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( SHOULDER_MOTOR, 100, 615 );
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( UPPER_ARM_MOTOR, 100, 137 );
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( WRIST_MOTOR, 100, 665 );
*/
	int correctionValue = 20; // 補正値(目標角度を設定角度の-20に暫定)

	executeRotate(FORE_ARM_MOTOR, 100, 512, 512 - correctionValue);
	executeRotate(SHOULDER_MOTOR, 100, 410, 410 - correctionValue);
	executeRotate(UPPER_ARM_MOTOR, 100, 820, 820 - correctionValue);
	executeRotate(SHOULDER_MOTOR, 100, 615, 615 - correctionValue);	
	executeRotate(UPPER_ARM_MOTOR, 100, 615, 615 - correctionValue);
	executeRotate(WRIST_MOTOR, 100, 615, 615 - correctionValue);
}

/************************************************************************/
// お宝回収＆搭載用形態
// お宝回収＆搭載用の位置に設定する。
/************************************************************************/
void CatchAndReleaseFormation(void)
{
/*
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( WRIST_MOTOR, 100, 768 );
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( UPPER_ARM_MOTOR, 100, 205 );
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( SHOULDER_MOTOR, 100, 478 );
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( UPPER_ARM_MOTOR, 100, 222 );	
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( SHOULDER_MOTOR, 100, 444 );
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( WRIST_MOTOR, 100, 512 );

	// 持ち上げる
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( SHOULDER_MOTOR, 100, 546 );
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( UPPER_ARM_MOTOR, 100, 768 );
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( FORE_ARM_MOTOR, 100, 768 );
	_delay_ms(2000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( WRIST_MOTOR, 100, 768 );
*/
	int correctionValue = 20; // 補正値(目標角度を設定角度の-20に暫定)

	executeRotate(WRIST_MOTOR, 100, 768, 768 - correctionValue);
	executeRotate(UPPER_ARM_MOTOR, 100, 205, 205 - correctionValue);
	executeRotate(SHOULDER_MOTOR, 100, 478, 478 - correctionValue);
	executeRotate(UPPER_ARM_MOTOR, 100, 222, 222 - correctionValue);
	executeRotate(SHOULDER_MOTOR, 100, 444, 444 - correctionValue);
	executeRotate(WRIST_MOTOR, 100, 512, 512 - correctionValue);

	// 持ち上げる
	executeRotate(SHOULDER_MOTOR, 100, 546, 546 - correctionValue);
	executeRotate(UPPER_ARM_MOTOR, 100, 768, 768 - correctionValue);
	executeRotate(FORE_ARM_MOTOR, 100, 768, 768 - correctionValue);
	executeRotate(WRIST_MOTOR, 100, 768, 768 - correctionValue);
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

	// 目標角度に達していない間は動作する
	while( targetangle < GetCurrentAngle(motorId) )
	{
		// 設定角度への動作を再実行
		MotorControlJoint( motorId, speed, angle );
		_delay_ms(50);//適切なウェイト時間を設定
	}
}

/************************************************************************/
// ペットボトル設置
/************************************************************************/
void placePETbottles(void) {
	_delay_ms(1000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( PETBOTTOLE_MOTOR, 30, 352 );//モーターを後方にゆっくり傾ける
	_delay_ms(6000);//6秒継続
	MotorControlJoint( PETBOTTOLE_MOTOR, 100, 512 );//モーターをセンター位置に戻す
	_delay_ms(3000);//3秒待つ⇒動作に合わせて変更してください

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
	int initResult = initLeftTurnAction(STOP_JUDGE_MAX_LIMIT);
	if (initResult == TRACE_SLOW_STRAIGHT) {
		return TRACE_SLOW_STRAIGHT;
	}
	LED_on(1);

	//停止が確定したらベース速度に応じて、前進or後進を実行
	adjustTurnPosition();

//	_delay_ms(5000);	// 10ms 間隔を空ける
//	LeftTurnByBaseSpeedAdjust();
	LeftTurnMove();
	while(1) {
		sensorPattern = getSensorPattern();
		//旋回動作を抜けるための条件を判定
		if (
			sensorPattern == BIT_010000 || sensorPattern == BIT_010001 ||
			sensorPattern == BIT_011000 || sensorPattern == BIT_011001 ||
			sensorPattern == BIT_001000 || sensorPattern == BIT_001001 ||
			sensorPattern == BIT_001100 || sensorPattern == BIT_001101 ||
			sensorPattern == BIT_000100 || sensorPattern == BIT_000101
			) {
			LED_on(2);
			//中央のセンサーが黒なら停止を実行
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			break;
		}
	}

	//旋回停止判定後の止まった位置でセンサーが中央なら逆旋回終了
	getSensors();
	sensorPattern = IR_BitPattern;
	if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
		//中央センサーなので、直進に設定して抜ける
		return TRACE_STRAIGHT;
	} else if (sensorPattern == BIT_010000 || sensorPattern == BIT_010001) {
		//左センサーなので、左曲りに設定して抜ける
		return TRACE_L_ROUND_MIDDLE;
	}
	
	LED_on(3);
	//センサーを中央に戻すため遅い旋回を実行
	RightTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
		//逆旋回動作を抜けるための条件を判定
		getSensors();
		sensorPattern = IR_BitPattern;
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			return TRACE_STRAIGHT;
		} else if ( sensorPattern == BIT_010000 ||	sensorPattern == BIT_010001 ||
					sensorPattern == BIT_100000 ||	sensorPattern == BIT_100001 ) {
			//既に逆側まで旋回していたら（想定よりも早く解除できてしまった場合など）
			break;
		}
	}

	//再度センサーを中央に戻すため遅い旋回を実行（ここまでは実行されない想定）
	LED_on(4);
	LeftTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
		//逆旋回動作を抜けるための条件を判定
		getSensors();
		sensorPattern = IR_BitPattern;
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
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

	int initResult = initRightTurnAction(STOP_JUDGE_MAX_LIMIT);
	if (initResult == TRACE_SLOW_STRAIGHT) {
		return TRACE_SLOW_STRAIGHT;
	}

	LED_on(1);

	//停止が確定したらベース速度に応じて、前進or後進を実行
	adjustTurnPosition();

//	_delay_ms(5000);	// 10ms 間隔を空ける

//	RightTurnByBaseSpeedAdjust();
	RightTurnMove();
	while(1) {
		sensorPattern = getSensorPattern();
		//旋回動作を抜けるための条件を判定(＊＊＊＊＊この判定で不足している。旋回抜ける)
		if (
		sensorPattern == BIT_000100 || sensorPattern == BIT_000101 ||
		sensorPattern == BIT_001100 || sensorPattern == BIT_001101 ||
		sensorPattern == BIT_001000 || sensorPattern == BIT_001001 ||
		sensorPattern == BIT_011000 || sensorPattern == BIT_011001 ||
		sensorPattern == BIT_010000 || sensorPattern == BIT_010001
		) {
			LED_on(2);
			//中央のセンサーが黒なら停止を実行
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			break;
		}
	}

	//旋回停止判定後の止まった位置でセンサーが中央なら逆旋回終了
	getSensors();
	sensorPattern = IR_BitPattern;
	if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
		//中央センサーなので、直進に設定して抜ける
		return TRACE_STRAIGHT;
	} else if (sensorPattern == BIT_000100 || sensorPattern == BIT_000101) {
		//右センサーなので、右曲りに設定して抜ける
		return TRACE_R_ROUND_MIDDLE;
	}
		
	LED_on(3);
	//センサーを中央に戻すため遅い旋回を実行
	LeftTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
		//逆旋回動作を抜けるための条件を判定
		getSensors();
		sensorPattern = IR_BitPattern;
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			return TRACE_STRAIGHT;
		} else if ( sensorPattern == BIT_010000 ||
					sensorPattern == BIT_010001 ||
					sensorPattern == BIT_100000 ||
					sensorPattern == BIT_100001 ) {
			//既に逆側まで旋回していたら（想定よりも早く解除できてしまった場合）
			break;

		}
	}
		
	//再度センサーを中央に戻すため遅い旋回を実行（ここまでは実行されない想定）
	LED_on(4);
	RightTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
		//逆旋回動作を抜けるための条件を判定
		getSensors();
		sensorPattern = IR_BitPattern;
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			return TRACE_STRAIGHT;
		}
	}
	return TRACE_STRAIGHT;
}

/**
 * カーブ実行
 */
void executeRound(void){
	static int sensorPattern = BIT_000000;
	
	while (1) {
		// 直進または旋回検知時は処理終了
		if ((currentTraceAction == TRACE_STRAIGHT) ||
			(currentTraceAction == TRACE_L_TURN) ||
			(currentTraceAction == TRACE_R_TURN)) {
				break;
		}
		
		// センサ値のビットパターンを取得する。
		sensorPattern = getSensorPattern();
		if (isDetectedNothing(sensorPattern)) {
			// センサ未検知の場合はラインがセンサの狭間にある場合なので、
			// トレースがなるべく直進に収束するようにトレース動作を調整する。
			if (needChangedSmooth()) {
				currentTraceAction = getSmoothAction();
			} 
			
			Execute(currentTraceAction);
			prePrevTraceAction = previousTraceAction;
			previousTraceAction = currentTraceAction;
		}
		else
		{
			break;
		}
	}
}

int needChangedSmooth(void) {
	if (prePrevTraceAction == TRACE_L_ROUND_SOFT) {
		return ((previousTraceAction == TRACE_STRAIGHT) ||
				(previousTraceAction == TRACE_L_STRAIGHT));
	}
	else if (prePrevTraceAction == TRACE_L_ROUND_MIDDLE) {
		return ((previousTraceAction == TRACE_STRAIGHT) ||
				(previousTraceAction == TRACE_L_STRAIGHT) ||
				(previousTraceAction == TRACE_L_ROUND_SOFT));
	}
	else if (prePrevTraceAction == TRACE_L_ROUND_TIGHT) {
		return ((previousTraceAction == TRACE_STRAIGHT) ||
				(previousTraceAction == TRACE_L_STRAIGHT) ||
				(previousTraceAction == TRACE_L_ROUND_SOFT) ||
				(previousTraceAction == TRACE_L_ROUND_MIDDLE));
	}
	else if (prePrevTraceAction == TRACE_R_ROUND_SOFT) {
		return ((previousTraceAction == TRACE_STRAIGHT) ||
				(previousTraceAction == TRACE_R_STRAIGHT));
	}
	else if (prePrevTraceAction == TRACE_R_ROUND_MIDDLE) {
		return ((previousTraceAction == TRACE_STRAIGHT) ||
				(previousTraceAction == TRACE_R_STRAIGHT) ||
				(previousTraceAction == TRACE_R_ROUND_SOFT));
	}
	else if (prePrevTraceAction == TRACE_R_ROUND_TIGHT) {
		return ((previousTraceAction == TRACE_STRAIGHT) ||
				(previousTraceAction == TRACE_R_STRAIGHT) ||
				(previousTraceAction == TRACE_R_ROUND_SOFT) ||
				(previousTraceAction == TRACE_R_ROUND_MIDDLE));
	}
	else {
		return FALSE;
	}
}

int getSmoothAction() {
	
	if (previousTraceAction == TRACE_L_STRAIGHT) {
		return TRACE_R_STRAIGHT;
	}
	else if (previousTraceAction == TRACE_L_ROUND_SOFT) {
		//return TRACE_R_STRAIGHT;
		return TRACE_L_STRAIGHT;
	}
	else if (previousTraceAction == TRACE_L_ROUND_MIDDLE) {
		//return TRACE_R_STRAIGHT;
		return TRACE_L_ROUND_SOFT;
	}
	else if (previousTraceAction == TRACE_L_ROUND_TIGHT) {
		//return TRACE_R_STRAIGHT;
		return TRACE_L_ROUND_MIDDLE;
	}
	else if (previousTraceAction == TRACE_R_STRAIGHT) {
		return TRACE_L_STRAIGHT;
	}
	else if (previousTraceAction == TRACE_R_ROUND_SOFT) {
		//return TRACE_L_STRAIGHT;
		return TRACE_R_STRAIGHT;
	}
	else if (previousTraceAction == TRACE_R_ROUND_MIDDLE) {
		//return TRACE_L_STRAIGHT;
		return TRACE_R_ROUND_SOFT;
	}
	else if (previousTraceAction == TRACE_R_ROUND_TIGHT) {
		//return TRACE_L_STRAIGHT;
		return TRACE_R_ROUND_MIDDLE;
	}
	else {
		return TRACE_STRAIGHT;
	}
}

/**
 * 右カーブ実行
 */
void executeRightRound(void){
	while (1) {
		// ターン検出時は処理終了
		if ((currentTraceAction == TRACE_L_TURN) ||
			(currentTraceAction == TRACE_R_TURN)) {
				break;
		}
	}
}

/**
 * 左旋回動作の初期化
 * 停止を実行して、途中で全て黒になったら直進モードにする
 * 基準以下の速度まで減速できたら、旋回を継続する
 */
int initLeftTurnAction(int maxVal) {
	int sensorPattern = BIT_000000;

	StopMove();//停止を実行
	int judgeSpeed = 0;
	while(1) {
		sensorPattern = getSensorPattern();//センサー値を取得
		if(sensorPattern == BIT_111110 || sensorPattern == BIT_111111 ||
		sensorPattern == BIT_011110 || sensorPattern == BIT_011111 ||
		sensorPattern == BIT_001110 || sensorPattern == BIT_001111 ||
		sensorPattern == BIT_000110 || sensorPattern == BIT_000111 ||
		sensorPattern == BIT_000010 || sensorPattern == BIT_000011
		) {
			//旋回判定後の停止中に黒ラインになったら旋回を止めて、直進する
			//旋回を止める条件は、センサー値がBIT_XXXX1Xでも良いかな。。。
			return TRACE_SLOW_STRAIGHT;
		}

		judgeSpeed = GetCurrentSpeedR();//モーターの速度を取得
		if( (judgeSpeed >= 0 && judgeSpeed <= maxVal) ||
			(judgeSpeed >= 1024 && judgeSpeed <= (1024 + maxVal)) ) {
			//速度がmaxVal以下ならstop()抜ける
			return TRACE_L_TURN;
		}
	}
}

/**
 * 右旋回動作の初期化
 * 停止を実行して、途中で全て黒になったら直進モードにする
 * 基準以下の速度まで減速できたら、旋回を継続する
 */
int initRightTurnAction(int maxVal) {
	int sensorPattern = BIT_000000;

	StopMove();//停止を実行
	int judgeSpeed = 0;
	while(1) {
		sensorPattern = getSensorPattern();//センサー値を取得
		if(sensorPattern == BIT_111110 || sensorPattern == BIT_111111 ||
		sensorPattern == BIT_111100 || sensorPattern == BIT_111101 ||
		sensorPattern == BIT_111000 || sensorPattern == BIT_111001 ||
		sensorPattern == BIT_110000 || sensorPattern == BIT_110001 ||
		sensorPattern == BIT_100000 || sensorPattern == BIT_100001
		) {
			//旋回判定後の停止中に黒ラインになったら旋回を止めて、直進する
			//旋回を止める条件は、センサー値がBIT_1XXXXXでも良いかな。。。
			return TRACE_SLOW_STRAIGHT;
		}

		judgeSpeed = GetCurrentSpeedR();//モーターの速度を取得
		if( (judgeSpeed >= 0 && judgeSpeed <= maxVal) ||
			(judgeSpeed >= 1024 && judgeSpeed <= (1024 + maxVal)) ) {
			//速度がmaxVal以下ならstop()抜ける
			return TRACE_R_TURN;
		}
	}
}

/**
 * 旋回に入ったベース速度に応じて、位置を調整する。
 */
void adjustTurnPosition(void) {
	if (BaseSpeed <= 200 ) {
		StraightLowMove();
		_delay_ms(200);	// 10ms 間隔を空ける
	} else if (BaseSpeed > 200 && BaseSpeed <= 300 ) {
		StraightLowMove();
		_delay_ms(150);	// 10ms 間隔を空ける
	} else if (BaseSpeed > 300 && BaseSpeed <= 330 ) {
		StraightLowMove();
		_delay_ms(60);	// 10ms 間隔を空ける
	} else if (BaseSpeed > 330 && BaseSpeed <= 350 ) {
		//StraightLowMove();
		//_delay_ms(50);	// 10ms 間隔を空ける
	} else if (BaseSpeed > 350 && BaseSpeed <= 450 ) {
		BackLowMove();
		_delay_ms(250);	// 10ms 間隔を空ける
	} else if (BaseSpeed > 450 && BaseSpeed <= 480 ) {
		BackLowMove();
		_delay_ms(300);	// 10ms 間隔を空ける
	} else if (BaseSpeed > 480 && BaseSpeed <= 500 ) {
		BackLowMove();
		_delay_ms(350);	// 10ms 間隔を空ける
	} else if (BaseSpeed > 500 && BaseSpeed <= 530 ) {
		BackLowMove();
		_delay_ms(400);	// 10ms 間隔を空ける
	}
	StopMove();
}

/**
 * 速度に応じた旋回復帰後のディレイ時間を取得する。
 */
void executeDelay(void) {
	if (BaseSpeed <= 200 ) {
		_delay_ms(400);
	} else if (BaseSpeed > 200 && BaseSpeed <= 250 ) {
		_delay_ms(350);
	} else if (BaseSpeed > 250 && BaseSpeed <= 300 ) {
		_delay_ms(300);
	} else if (BaseSpeed > 300 && BaseSpeed <= 350 ) {
		_delay_ms(250);
	} else if (BaseSpeed > 350 && BaseSpeed <= 400 ) {
		_delay_ms(200);
	} else {
		_delay_ms(400);
	}
}

/**
* ショートカットの処理
* @brief ショートカットの処理
* @return なし
*/
void executeSkipAction(void) {
	LOG_INFO("***** executeSkipAction START!! *****\r\n");

	static int sensorPattern = BIT_000000;
    static int counter = 0;

	// 初期動作（直進）
	StraightMove();
	_delay_ms(100);	// 10ms 間隔を空ける

	while (1) {
		StraightMove();

		// センサ値のビットパターンを取得する。
		getSensors();
		sensorPattern = IR_BitPattern;

		// センサ値のパターンが全黒 or 直角ライン判定であればループを抜ける。
		if (sensorPattern == BIT_111111 || sensorPattern == BIT_111110 ||
			sensorPattern == BIT_011111 || sensorPattern == BIT_011110 ||
			sensorPattern == BIT_001111 || sensorPattern == BIT_001110 ||
			sensorPattern == BIT_000111 || sensorPattern == BIT_000110 ||
			sensorPattern == BIT_111101 || sensorPattern == BIT_111100 ||
			sensorPattern == BIT_111001 || sensorPattern == BIT_111000 ||
			sensorPattern == BIT_110001 || sensorPattern == BIT_110000
			) {
			break;
		}
#ifdef LOG_INFO_ON
		if ((counter % 1) == 0) {
			BaseSpeed = BaseSpeed + 1;
			counter = 0;
		}
#else
		if ((counter % 5) == 0) {
			BaseSpeed = BaseSpeed + 3;
			counter = 0;
		}
#endif /* _MODE_SKIP_ */
	}

	//旋回判定されたら停止を実行
	stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);

	//停止が確定したらベース速度に応じて、前進or後進を実行
	adjustTurnPosition();

	//ベース速度を初期化
	BaseSpeed = BASE_SPEED_INIT_VAL;

	// 左旋回
	LeftTurnMove();
	while(1) {
		getSensors();
		sensorPattern = IR_BitPattern;
		//旋回動作を抜けるための条件を判定
		if (
			sensorPattern == BIT_010000 || sensorPattern == BIT_010001 ||
			sensorPattern == BIT_011000 || sensorPattern == BIT_011001 ||
			sensorPattern == BIT_001000 || sensorPattern == BIT_001001 ||
			sensorPattern == BIT_001100 || sensorPattern == BIT_001101 ||
			sensorPattern == BIT_000100 || sensorPattern == BIT_000101
			) {
			LED_on(2);
			//中央のセンサーが黒なら停止を実行
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			break;
		}
	}
	
	//旋回停止判定後の止まった位置でセンサーが中央なら逆旋回終了
	getSensors();
	sensorPattern = IR_BitPattern;
	if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
		//中央センサーなので、直進に設定して抜ける
		StraightMove();
		return;
		} else if (sensorPattern == BIT_010000 || sensorPattern == BIT_010001) {
		//左センサーなので、左曲りに設定して抜ける
		LeftSoftRoundMove();
		return;
	}

	//センサーを中央に戻すため遅い旋回を実行
	RightTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
		//逆旋回動作を抜けるための条件を判定
		getSensors();
		sensorPattern = IR_BitPattern;
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			StraightMove();
			return;
		} else if ( sensorPattern == BIT_010000 ||	sensorPattern == BIT_010001 ||
			sensorPattern == BIT_100000 ||	sensorPattern == BIT_100001 ) {
			//既に逆側まで旋回していたら（想定よりも早く解除できてしまった場合など）
			LeftMiddleRoundMove();
			return;
		}
	}

	LOG_INFO("***** executeSkipAction END!! *****\r\n");

	// 通常のライントレースに復帰
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
