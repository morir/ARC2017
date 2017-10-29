/**
 * @file MovingDistance.c
 * @brief �ړ�����
 * @author f.aimano
 * @date 2017/10/28
 */

#include "AvrTimer.h"
#include "DebugLog.h"
#include "MotorManager.h"
#include "MovingDistance.h"

static int32_t movingDistance = 0;	// �ړ�����
static int velocity_0 = 0;			// �����x
static uint32_t lastTime = 0;		// �O��̎���(�~���b)

/**
 * �ړ��������擾
 * @brief �ړ��������擾
 * @return �ړ�����
 */
int32_t GetMovingDistance() {
	return movingDistance;
}

/**
 * �ړ��������擾
 * @brief �ړ�������ݒ�
 * @param  distance ����
 */
void SetMovingDistance(int32_t distance) {
	movingDistance = distance;
}

/**
 * �ړ��������X�V
 * @brief �ړ��������X�V
 * @param  currentTime ���݂̎���
 */
void UpdateMovingDistance(uint32_t currentTime) {
	// ���݂̑��x���v�Z
	int velocity = (GetCurrentSignedSpeedR() + GetCurrentSignedSpeedL()) / 2;
	
	// �o�ߎ���(msec)���v�Z
	uint32_t elapsedTime = currentTime - lastTime;
	
	// �����x���v�Z
	int acceleration = 0;
	if(0 != elapsedTime) {
		acceleration = (velocity - velocity_0) / elapsedTime;
	}
	
	// ���݂̈ړ��������v�Z
	int32_t currentMovingDistance = velocity_0 * (int32_t)elapsedTime + acceleration *(int32_t)(elapsedTime * elapsedTime) / 2;
	
	// �u�����^�~���b�v����u�����^�b�v�ɕϊ����A�ړ������ɔ��f
	movingDistance = movingDistance + currentMovingDistance / 1000;
	
	// �����x����ёO��̎��Ԃ��X�V
	velocity_0 = velocity;
	lastTime = currentTime;
}