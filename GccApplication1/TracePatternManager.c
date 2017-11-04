/*
 * TracePatternManager.c
 *
 * Created: 2017/10/28 22:38:20
 *  Author: Administrator
 */ 

#include <string.h>
#include "TracePatternManager.h"
#include "SensorManager.h"
#include "MotorManager.h"
#include "DebugLog.h"

void initActionTable() {
	ActionTable[0]  = TRACE_STRAIGHT;		/* 00:BIT_000000 */
	ActionTable[1]  = TRACE_R_ROUND_TIGHT;	/* 01:BIT_000001 */
	ActionTable[2]  = TRACE_R_ROUND_MIDDLE;	/* 02:BIT_000010 */
	ActionTable[3]  = TRACE_R_TURN;			/* 03:BIT_000011 */
	ActionTable[4]  = TRACE_R_ROUND_SOFT;	/* 04:BIT_000100 */
	ActionTable[5]  = TRACE_R_TURN;			/* 05:BIT_000101 */
	ActionTable[6]  = TRACE_R_ROUND_SOFT;	/* 06:BIT_000110 */
	ActionTable[7]  = TRACE_R_TURN;			/* 07:BIT_000111 */
	ActionTable[8]  = TRACE_L_ROUND_SOFT;	/* 08:BIT_001000 */
	ActionTable[9]  = TRACE_UNDEFINED;		/* 09:BIT_001001 */
	ActionTable[10] = TRACE_UNDEFINED;		/* 10:BIT_001010 */
	ActionTable[11] = TRACE_UNDEFINED;		/* 11:BIT_001011 */
	ActionTable[12] = TRACE_STRAIGHT;		/* 12:BIT_001100 */
	ActionTable[13] = TRACE_UNDEFINED;		/* 13:BIT_001101 */
	ActionTable[14] = TRACE_R_ROUND_SOFT;	/* 14:BIT_001110 */
	ActionTable[15] = TRACE_R_TURN;			/* 15:BIT_001111 */
	ActionTable[16] = TRACE_L_ROUND_MIDDLE;	/* 16:BIT_010000 */
	ActionTable[17] = TRACE_UNDEFINED;		/* 17:BIT_010001 */
	ActionTable[18] = TRACE_UNDEFINED;		/* 18:BIT_010010 */
	ActionTable[19] = TRACE_UNDEFINED;		/* 19:BIT_010011 */
	ActionTable[20] = TRACE_UNDEFINED;		/* 20:BIT_010100 */
	ActionTable[21] = TRACE_UNDEFINED;		/* 21:BIT_010101 */
	ActionTable[22] = TRACE_UNDEFINED;		/* 22:BIT_010110 */
	ActionTable[23] = TRACE_UNDEFINED;		/* 23:BIT_010111 */
	ActionTable[24] = TRACE_L_ROUND_SOFT;	/* 24:BIT_011000 */
	ActionTable[25] = TRACE_UNDEFINED;		/* 25:BIT_011001 */
	ActionTable[26] = TRACE_UNDEFINED;		/* 26:BIT_011010 */
	ActionTable[27] = TRACE_UNDEFINED;		/* 27:BIT_011011 */
	ActionTable[28] = TRACE_L_ROUND_SOFT;	/* 28:BIT_011100 */
	ActionTable[29] = TRACE_UNDEFINED;		/* 29:BIT_011101 */
	ActionTable[30] = TRACE_STRAIGHT;		/* 30:BIT_011110 */
	ActionTable[31] = TRACE_R_TURN;			/* 31:BIT_011111 */
	ActionTable[32] = TRACE_L_ROUND_TIGHT;	/* 32:BIT_100000 */
	ActionTable[33] = TRACE_UNDEFINED;		/* 33:BIT_100001 */
	ActionTable[34] = TRACE_UNDEFINED;		/* 34:BIT_100010 */
	ActionTable[35] = TRACE_UNDEFINED;		/* 35:BIT_100011 */
	ActionTable[36] = TRACE_UNDEFINED;		/* 36:BIT_100100 */
	ActionTable[37] = TRACE_UNDEFINED;		/* 37:BIT_100101 */
	ActionTable[38] = TRACE_UNDEFINED;		/* 38:BIT_100110 */
	ActionTable[39] = TRACE_UNDEFINED;		/* 39:BIT_100111 */
	ActionTable[40] = TRACE_L_TURN;			/* 40:BIT_101000 */
	ActionTable[41] = TRACE_UNDEFINED;		/* 41:BIT_101001 */
	ActionTable[42] = TRACE_UNDEFINED;		/* 42:BIT_101010 */
	ActionTable[43] = TRACE_UNDEFINED;		/* 43:BIT_101011 */
	ActionTable[44] = TRACE_UNDEFINED;		/* 44:BIT_101100 */
	ActionTable[45] = TRACE_UNDEFINED;		/* 45:BIT_101101 */
	ActionTable[46] = TRACE_UNDEFINED;		/* 46:BIT_101110 */
	ActionTable[47] = TRACE_UNDEFINED;		/* 47:BIT_101111 */
	ActionTable[48] = TRACE_L_TURN;			/* 48:BIT_110000 */
	ActionTable[49] = TRACE_UNDEFINED;		/* 49:BIT_110001 */
	ActionTable[50] = TRACE_UNDEFINED;		/* 50:BIT_110010 */
	ActionTable[51] = TRACE_UNDEFINED;		/* 51:BIT_110011 */
	ActionTable[52] = TRACE_UNDEFINED;		/* 52:BIT_110100 */
	ActionTable[53] = TRACE_UNDEFINED;		/* 53:BIT_110101 */
	ActionTable[54] = TRACE_UNDEFINED;		/* 54:BIT_110110 */
	ActionTable[55] = TRACE_UNDEFINED;		/* 55:BIT_110111 */
	ActionTable[56] = TRACE_L_TURN;			/* 56:BIT_111000 */
	ActionTable[57] = TRACE_UNDEFINED;		/* 57:BIT_111001 */
	ActionTable[58] = TRACE_UNDEFINED;		/* 58:BIT_111010 */
	ActionTable[59] = TRACE_UNDEFINED;		/* 59:BIT_111011 */
	ActionTable[60] = TRACE_L_TURN;			/* 60:BIT_111100 */
	ActionTable[61] = TRACE_UNDEFINED;		/* 61:BIT_111101 */
	ActionTable[62] = TRACE_L_TURN;			/* 62:BIT_111110 */
	ActionTable[63] = TRACE_SLOW_STRAIGHT;	/* 63:BIT_111111 */
}

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

/**
* Bitパターンの履歴を初期化する。
* @brief Bitパターンの履歴を直進のBitパターンで初期化する。
* @return 戻り値の説明
*/
void initSensorHistory(void) {
	memset(IR, 0, sizeof(IR));
	IR_BitPattern = 0;
	currentCount = 0;
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
		    LOG_DEBUG("TRACE_UNDEFINED: maxCount = %d\r\n", maxCount);
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

