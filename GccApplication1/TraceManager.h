/*
 * TraceManager.h
 *
 * Created: 2017/10/28 22:55:59
 *  Author: Administrator
 */ 


#ifndef TRACEMANAGER_H_
#define TRACEMANAGER_H_


// ------------------ Defined ------------------


// ------------------ Global variables ------------------

// ç°âÒÇÃÉgÉåÅ[ÉXìÆçÏ
int currentTraceAction;

int isSearchingLeftSide;

// ------------------ Method Definition ------------------
void initTraceAction(void);

void traceCommon(int *counter, int *maxSpeed);

void traceForwardArea_01(void);
void traceForwardArea_02(void);
void traceForwardArea_03(void);
void traceForwardArea_04(void);
void traceForwardArea_05(void);

void traceBackwardArea_01(void);
void traceBackwardArea_02(void);
void traceBackwardArea_03(void);
void traceBackwardArea_04(void);

void traceBackwardArea_06(void);
void traceBackwardArea_07(void);
void traceBackwardArea_08(void);
void traceBackwardArea_09(void);
void traceBackwardArea_10(void);
void traceBackwardArea_11(void);
void traceBackwardArea_12(void);
void traceBackwardArea_13(void);

void traceBackwardArea_14(void);
void traceBackwardArea_15(void);
void traceBackwardArea_16(void);
void traceBackwardArea_17(void);
void traceBackwardArea_18(void);

void TreasureFindingLineTrace(int isFirst);

#endif /* TRACEMANAGER_H_ */