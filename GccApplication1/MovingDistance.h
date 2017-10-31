/**
 * @file MovingDistance.h
 * @brief ˆÚ“®‹——£
 * @author f.aimano
 * @date 2017/10/28
 */

#ifndef MOVINGDISTANCE_H_
#define MOVINGDISTANCE_H_

typedef enum _MEASUREMENT_STATUS {
	NOT_MEASURING = 0,	// –¢Œv‘ª’†
	MEASURING,			// Œv‘ª’†
} MEASUREMENT_STATUS;

void StartMeasuringMovingDistance();
void StopMeasuringMovingDistance();
int32_t GetMovingDistance();
void UpdateMovingDistance(uint32_t currentTime);

#endif /* MOVINGDISTANCE_H_ */