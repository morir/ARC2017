/*
 * ArmActionManager.h
 *
 * Created: 2017/10/28 22:42:34
 *  Author: Administrator
 */ 


#ifndef ARMACTIONMANAGER_H_
#define ARMACTIONMANAGER_H_

#include <util/delay.h>
#include "include/dynamixel.h"

// ------------------ Defined ------------------
#define SHOULDER_MOTOR       12      // Shoulder Motor address
#define UPPER_ARM_MOTOR      25      // Upper arm Motor address
#define FORE_ARM_MOTOR       14      // ForeArm Motor address
#define WRIST_MOTOR          23      // Wrist Motor address

// MX-12W Control Table Address
#define CTRL_TBL_ADDR_PRESENT_POSITION_L	(36)	// Lowest byte of Current Position
#define CTRL_TBL_ADDR_PRESENT_POSITION_H	(37)	// Highest byte of Current Position


#define CorrectionValue		(50) // ï‚ê≥íl(ñ⁄ïWäpìxÇê›íËäpìxÇÃ-50Ç…ébíË)

// ------------------ Method Definition ------------------
void initDumpMotor(void);
void FindFormation(void);
void CatchAndReleaseFormation(void);

void Debug_AllMotorCurrentAngle(void);
void executeRotate(int motorId, int speed, int angle, int targetangle);
int GetCurrentAngle(int motorId);

#endif /* ARMACTIONMANAGER_H_ */