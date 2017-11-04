/*
 * TurnManager.c
 *
 * Created: 2017/10/28 22:37:52
 *  Author: Administrator
 */
 
#include "SensorManager.h"
#include "MotorManager.h"
#include "TurnManager.h"
#include "TracePatternManager.h"

extern void adjustTurnPosition(void);
extern void LED_on(int i);
extern void stopMoveLessThanVal(int val);

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
    } else if (sensorPattern == BIT_010000
               || sensorPattern == BIT_011100
               || sensorPattern == BIT_001110) {
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
		sensorPattern = getSensorPattern();
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001100 || sensorPattern == BIT_000100) {
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			break;
		}
	}
	return TRACE_STRAIGHT;
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
 * 旋回に入ったベース速度に応じて、位置を調整する。
 * 2017ロボに合わせて調整必要！
 */
void adjustTurnPosition(void) {
	if (BaseSpeed <= 80 ) {
		StraightLowMove();
		_delay_ms(250);	// 300ms 間隔を空ける
	} else if (BaseSpeed <= 100 ) {
		StraightLowMove();
		_delay_ms(220);	// 270ms 間隔を空ける
	} else if (BaseSpeed <= 120 ) {
		StraightLowMove();
		_delay_ms(190);	// 240ms 間隔を空ける
	} else if (BaseSpeed <= 140 ) {
		StraightLowMove();
		_delay_ms(170);	// 190ms 間隔を空ける
	} else if (BaseSpeed <= 160 ) {
		StraightLowMove();
		_delay_ms(160);	// 160ms 間隔を空ける
	} else if (BaseSpeed <= 180 ) {
		StraightLowMove();
		_delay_ms(140);	// 140ms 間隔を空ける
	} else {
		StraightLowMove();
		_delay_ms(120);	// 120ms 間隔を空ける
	}

    StopMove();
}


