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
#include "TraceManager.h"
#include "TracePatternManager.h"
#include "TurnManager.h"

// ------------------ Defined ------------------
// Line Sensor
#define LINE_STATE_BLACK    0//センサー値でラインが白判定
#define LINE_STATE_WHITE    1//センサー値でラインが黒判定

#define _LED_ON_

#ifdef ENABLE_AVRTIMER
// ------------------ Global variables ------------------
static int curSensorPattern = 0;	// 現在のセンサー値
#endif // ENABLE_AVRTIMER

// ------------------ Method Definition ------------------
void executeTraceProcess(void);
void treasureHunt_01(void);
void treasureHunt_02(void);
void treasureHunt_03(void);

void initCargoBedMotor(void);
void dumpTreasures(void);
void stopMoveLessThanVal(int val);

void executeLeftTurnFromOnLine(void);
void executeRightTurnFromOnLine(void);
void adjustTurnPosition(void);
void executeFinalAction(void);

void initEmergencyStop(void);

void setLED(void);
void LED_on(int i);
void LED_off(int i);

// ------------------ Global Variables Definition ------------------

// Serial Message Buffer
int serCmd[SERIAL_BUFFER_SIZE] = {0};

int isSearchingLeft = 0;

// ------------------ Method ------------------

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
	initTraceAction();
	initSensorHistory();
	initActionTable();
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
	//traceForwardArea_01();
	//traceForwardArea_02();
	//traceForwardArea_03();
	//traceForwardArea_04();
	//traceForwardArea_05();
	treasureHunt_01();
	//traceBackwardArea_01();
	//traceBackwardArea_02();
	//traceBackwardArea_03();
	//traceBackwardArea_04();
	//treasureHunt_02();
	//traceBackwardArea_06();
	//traceBackwardArea_07();
	//traceBackwardArea_08();
	//traceBackwardArea_09();
	//traceBackwardArea_10();
	//traceBackwardArea_11();
	//traceBackwardArea_12();
	//traceBackwardArea_13();
	//treasureHunt_03();
	//traceBackwardArea_14();
	//traceBackwardArea_15();
	//traceBackwardArea_16();
	//traceBackwardArea_17();
	//traceBackwardArea_18();
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
    _delay_ms(500);
    // 前進or後進する（実動作に合わせて設定）。
    // executeXXXX();
    // 手を開く
    MotorControlJoint( WRIST_MOTOR, 100, 665 );//確認用
         
    // 停止する
    StopMove();
    _delay_ms(1000);

    // 宝物を掴んで荷台に乗せる
    //CatchAndReleaseFormation();

    // ライン上からの旋回を行う
    executeRightTurnFromOnLine();

    // 停止する
    StopMove();
    _delay_ms(100);
}

/*
 * 宝物 2 のトレース動作
 * @return なし
 * @condition
 *   開始条件：
 *   終了条件：
 */
 void treasureHunt_02(void) {
    LOG_INFO("treasureHunt_02() %s\r\n", "1");

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

	int counter = 0;
	int maxSpeed = 50;

    // 右直角ライン検出までライントレース実行
	while (currentTraceAction != TRACE_R_TURN) {

    	traceCommon(&counter, &maxSpeed);
		// 加速しない
		maxSpeed = 50;
	}

	// 右旋回実行
	currentTraceAction = executeRightTurn();
	BaseSpeed = 50;

    // 停止する
    StopMove();
    _delay_ms(100);
}

/*
 * 宝物 3 のトレース動作
 * @return なし
 * @condition
 *   開始条件：
 *   終了条件：
 */
 void treasureHunt_03(void) {
    LOG_INFO("treasureHunt_03() %s\r\n", "1");

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
            _delay_ms(300);//センサーが全て通過するまで旋回
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
            sensorPattern == BIT_000100 || sensorPattern == BIT_000110
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
            _delay_ms(300);//センサーが全て通過するまで旋回
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
        sensorPattern == BIT_000100 || sensorPattern == BIT_001100 ||
        sensorPattern == BIT_001000 || sensorPattern == BIT_011000
        ) {
            LED_on(2);
            //中央のセンサーが白なら停止を実行
            stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
            break;
        }

        //右センサーを検出しているか確認する
        if (sensorPattern == BIT_000001) {
            //右センサーを検出したら旋回速度を落とす
            RightTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
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
