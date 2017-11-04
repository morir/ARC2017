/*
 * TraceManager.c
 *
 * Created: 2017/10/28 22:37:52
 *  Author: Administrator
 */ 
 
#include "SensorManager.h"
#include "MotorManager.h"
#include "MovingDistance.h"
#include "TraceManager.h"
#include "TracePatternManager.h"
#include "TurnManager.h"

extern void adjustTurnPosition(void);
extern void LED_on(int i);

void initTraceAction() {
	currentTraceAction = TRACE_STRAIGHT;
	isSearchingLeftSide = 0;
}

/*
 * 共通トレース動作
 * @return なし
 * @condition
 *   開始条件：スタートコマンドを受信する。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
void traceCommon(int *counter, int *maxSpeed) {
	// センサー値を取得
	getSensors();
	currentTraceAction = getActionWithHistory();
	if (currentTraceAction == TRACE_UNDEFINED) {
		_delay_ms(1);// delayTimeの間隔を空ける
		return;
	}

#ifdef LOG_INFO_ON
	if ((*counter % 1) == 0) {
		BaseSpeed = BaseSpeed + 1;
		*counter = 0;
	}
#else
	if ((*counter % 5) == 0) {
		BaseSpeed = BaseSpeed + 2;
		*counter = 0;
	}
#endif /* _MODE_SKIP_ */
	if (BaseSpeed > *maxSpeed) {
		BaseSpeed = *maxSpeed;
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
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	//初期動作（少しだけ直進）
	StraightMove();
	_delay_ms(100);	// 10ms 間隔を空ける

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(&counter, &maxSpeed);
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
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(&counter, &maxSpeed);
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
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(&counter, &maxSpeed);
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
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(&counter, &maxSpeed);
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
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(&counter, &maxSpeed);
		counter++;
	}

	// 右旋回実行
	currentTraceAction = executeRightTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 復路エリア 1 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（折り返し点のトレース動作から継続）。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
 void traceBackwardArea_01(void) {
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(&counter, &maxSpeed);
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
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(&counter, &maxSpeed);
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
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	
	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(&counter, &maxSpeed);
		counter++;
	}

	// 停止実行
	stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
	LED_on(1);
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

	StraightMove();
	_delay_ms(10);

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
		_delay_ms(1);
	}

	// 停止実行
	stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);

	// センサー値に応じて旋回を実行
	// TODO:実装

	// 旋回終了後、停止実行
	StopMove();
	initSensorHistory();
	currentTraceAction = TRACE_STRAIGHT;
}

/*
 * 復路エリア 6 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 5 のトレース動作から継続）。
 *   終了条件：
 */
 //void traceBackwardArea_06(void) {
	//int counter = 0;
	//int maxSpeed = BASE_SPEED_BY_TURF_AREA;
//
	//while (currentTraceAction != TRACE_R_TURN) {
		//traceCommon(&counter, &maxSpeed);
		//// 加速しない
		//maxSpeed = BASE_SPEED_BY_TURF_AREA;
	//}
//
	//// 右旋回実行
	//executeRightTurn();
	//BaseSpeed = BASE_SPEED_BY_TURF_AREA;
//}

/*
 * 復路エリア 6 のトレース動作（宝物２からのバック）
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 5 のトレース動作から継続）。
 *   終了条件：
 */
void traceBackwardArea_06(void) {
	 int counter = 0;
	 int maxSpeed = BASE_SPEED_BY_TURF_AREA;

	 while (currentTraceAction != TRACE_R_TURN) {
		//バックする。
		// 未実装
	 }

	 //必要に応じて位置を調整

	 // 右旋回実行
	 executeRightTurn();
	 BaseSpeed = BASE_SPEED_BY_TURF_AREA;
}

/*
 * 復路エリア 7 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 6 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_07(void) {
	int counter = 0;
	int maxSpeed = BASE_SPEED_BY_TURF_AREA;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(&counter, &maxSpeed);
		// 加速しない
		maxSpeed = BASE_SPEED_BY_TURF_AREA;
		_delay_ms(1);
	}

	// 右旋回実行
	executeRightTurn();
}

/*
 * 復路エリア 8 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 7 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_08(void) {
	int counter = 0;
	int maxSpeed = BASE_SPEED_BY_TURF_AREA;
    int sensorPattern = BIT_111111;
	int findAnySensorCount = 0;

	// ラインが途切れるまではトレース動作
	while (sensorPattern != BIT_000000) {
		traceCommon(&counter, &maxSpeed);
		sensorPattern = getSensorPattern();
		// 加速しない
		maxSpeed = BASE_SPEED_BY_TURF_AREA;
		_delay_ms(1);
	}

	// 停止実行
	//stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);

	StraightMove();
    _delay_ms(10);

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
		_delay_ms(1);
	}

	// ちょっと進んで停止
	adjustTurnPosition();

	// 右旋回実行
	executeRightTurnFromOnLine();
	BaseSpeed = BASE_SPEED_INIT_VAL;
	initSensorHistory();
	currentTraceAction = TRACE_STRAIGHT;
}

/*
 * 復路エリア 9 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 8 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_09(void) {
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(&counter, &maxSpeed);
		counter++;
		_delay_ms(1);
	}

	// 右旋回実行
	executeRightTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
	initSensorHistory();
	currentTraceAction = TRACE_STRAIGHT;
	StopMove();//DBG
}

/*
 * 復路エリア 10 のトレース動作
 * @return なし
 * @condition
 *   開始条件：なし（復路エリア 9 のトレース動作から継続）。
 *   終了条件：
 */
 void traceBackwardArea_10(void) {
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(&counter, &maxSpeed);
		counter++;
	}

	// 停止実行
	stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
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
		_delay_ms(1);
	}

	// 停止実行
	StopMove();

	// センサー値に応じて旋回を実行
	// TODO:実装

	// 旋回終了後、停止実行
	stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
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
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(&counter, &maxSpeed);
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
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (counter > 1500) {
		traceCommon(&counter, &maxSpeed);
		counter++;
	}

	// 停止実行
	StopMove();
	initSensorHistory();
	currentTraceAction = TRACE_STRAIGHT;
	BaseSpeed = BASE_SPEED_INIT_VAL;
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
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(&counter, &maxSpeed);
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
	int counter = 0;
	int maxSpeed = BASE_SPEED_INIT_VAL;
	int sensorPattern = BIT_111111;
	int findAnySensorCount = 0;

	// ラインが途切れるまではトレース動作
	while (sensorPattern != BIT_000000) {
		traceCommon(&counter, &maxSpeed);
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
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(&counter, &maxSpeed);
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
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	// 直進
	currentTraceAction = TRACE_STRAIGHT;
	BaseSpeed = BASE_SPEED_INIT_VAL;

	// AVRタイマ開始
	AvrTimerStart();

	// 移動距離の計測を開始
	StartMeasuringMovingDistance();

	// 目標の距離まで前進
	//currentTraceAction = TRACE_STRAIGHT;
	int32_t targetDistance = 130;	//目標距離 = 130cm
	while (targetDistance > GetMovingDistance()) {
		// 移動距離を更新
		UpdateMovingDistance();

		// 共通トレース動作
		traceCommon(&counter, &maxSpeed);
		counter++;
	}

	// 移動距離の計測を停止
	StopMeasuringMovingDistance();
	_delay_ms(10);

	// AVRタイマ停止
	AvrTimerEnd();
	AvrTimerReset();

	// 停止実行
	StopMove();
}

/*
 * 左右大き目にロボットを揺らしたジグザグなライントレースを実行
 */
void TreasureFindingLineTrace(int isFirst) {
	int sensorPattern = BIT_000000;
	BaseSpeed = 50;

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
	    sensorPattern == BIT_011100 ||
	    sensorPattern == BIT_001100
	    )) {
		// ラインの右側サーチ中に左ラインを検出したら、
		// ラインの左側サーチに切り替える
		isSearchingLeftSide = 1;
		Execute(TRACE_L_TRESURE_FIND);
	} else if (
	    (isSearchingLeftSide > 0) &&
	    (sensorPattern == BIT_000010 ||
	    sensorPattern == BIT_000100 ||
	    sensorPattern == BIT_000110 ||
	    sensorPattern == BIT_001110 ||
	    sensorPattern == BIT_001100
	    )) {
		// ラインの左側サーチ中に右ラインを検出したら、
		// ラインの右側サーチに切り替える
		isSearchingLeftSide = 0;
		Execute(TRACE_R_TRESURE_FIND);
	}
	
	_delay_ms(10);
}

/*
 * 宝物1個の場合の左旋回検出用トレース動作
 * @return なし
 * @condition
 *   開始条件：前回のトレース動作から継続。
 *   終了条件：センサで左ターンを検出して直角旋回が完了する。
 */
 void shortTraceToLeftTurn(void) {
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_L_TURN) {
		traceCommon(&counter, &maxSpeed);
		counter++;
	}

	// 左旋回実行
	currentTraceAction = executeLeftTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}

/*
 * 宝物1個の場合の右旋回検出用トレース動作
 * @return なし
 * @condition
 *   開始条件：前回のトレース動作から継続。
 *   終了条件：センサで右ターンを検出して直角旋回が完了する。
 */
 void shortTraceToRightTurn(void) {
	int counter = 0;
	int maxSpeed = MAX_SPEED;

	while (currentTraceAction != TRACE_R_TURN) {
		traceCommon(&counter, &maxSpeed);
		counter++;
	}

	// 右旋回実行
	currentTraceAction = executeRightTurn();
	BaseSpeed = BASE_SPEED_INIT_VAL;
}
