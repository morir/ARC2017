/*
 * ArmActionManager.c
 *
 * Created: 2017/10/28 22:38:53
 *  Author: Administrator
 */ 

#include "ArmActionManager.h"

#include "MotorManager.h"
#include "AvrTimer.h"
#include "DebugLog.h"


/************************************************************************/
// Arm�p���[�^�̏����ݒ�
// Arm�p���[�^��󕨌����p�`��(���C���g���[�X�p)�̈ʒu�ɐݒ�
/************************************************************************/
void initDumpMotor(void) {
	FindFormation();
}

/************************************************************************/
// �󕨌����p�`��
// �󕨌����p�̈ʒu�ɐݒ肷��B
/************************************************************************/
void FindFormation(void)
{
	executeRotate(UPPER_ARM_MOTOR, 100, 80, 80 - CorrectionValue);
	executeRotate(FORE_ARM_MOTOR, 100, 310, 310 - CorrectionValue);
	executeRotate(WRIST_MOTOR, 200, 512, 512 - CorrectionValue);
	executeRotate(SHOULDER_MOTOR, 100, 680, 680 - CorrectionValue);

	executeRotate(UPPER_ARM_MOTOR, 100, 70, 70 - CorrectionValue);
	executeRotate(FORE_ARM_MOTOR, 100, 290, 290 - CorrectionValue);
}

/************************************************************************/
// �󕨉�������ڗp�`��
// �󕨂�������A���ڂ���
// �i�K�I�ɓ��삳���󕨂𗎂Ƃ�
/************************************************************************/
void CatchAndReleaseFormation(void)
{
	//-- �Ђ炭
	//executeRotate( WRIST_MOTOR, 100, 768, 768 - CorrectionValue );

	//-- �͂�
	executeRotate(WRIST_MOTOR, 200, 512, 512 - CorrectionValue );

	//-- �����グ�J�n
//	executeRotate(FORE_ARM_MOTOR, 100, 355, 355 - CorrectionValue);
	executeRotate(SHOULDER_MOTOR, 100, 650, 650 - CorrectionValue);
	
	//-- �����グ�r��	
	executeRotate(SHOULDER_MOTOR, 100,	512, 512 - CorrectionValue);
	executeRotate(UPPER_ARM_MOTOR, 100, 512, 512 - CorrectionValue);

	executeRotate(UPPER_ARM_MOTOR, 100, 705, 705 - CorrectionValue);
	executeRotate(SHOULDER_MOTOR, 100, 345, 345 - CorrectionValue);

	//-- ���Ƃ����O
	executeRotate(SHOULDER_MOTOR, 100, 345, 345 - CorrectionValue);
	executeRotate(UPPER_ARM_MOTOR, 100, 705, 705 - CorrectionValue);
	executeRotate(FORE_ARM_MOTOR, 100, 730, 730 - CorrectionValue);

	//-- ���Ƃ�
	executeRotate( WRIST_MOTOR, 200, 640, 640 - CorrectionValue );
	
	_delay_ms(1000);//1�b�҂˓���ɍ��킹�ĕύX���Ă�������

	//-- �󕨌����p�`�ԂɈڍs���邽�߂̏���
	executeRotate(FORE_ARM_MOTOR, 100, 155, 155 - CorrectionValue);
	executeRotate(UPPER_ARM_MOTOR, 100, 200, 200 - CorrectionValue);
	executeRotate(SHOULDER_MOTOR, 100, 512, 512 - CorrectionValue);
	_delay_ms(1000);//1�b�҂˓���ɍ��킹�ĕύX���Ă�������
}




/************************************************************************/
// ���݂̃��[�^�p�x��\��(Debug�p)
// ���݂̃��[�^�p�x��\������B
/************************************************************************/
void Debug_AllMotorCurrentAngle(void)
{
	LOG_DEBUG("GetCurrentAngle(SHOULDER_MOTOR) %d\r\n", GetCurrentAngle(SHOULDER_MOTOR));
	LOG_DEBUG("GetCurrentAngle(UPPER_ARM_MOTOR) %d\r\n", GetCurrentAngle(UPPER_ARM_MOTOR));
	LOG_DEBUG("GetCurrentAngle(FORE_ARM_MOTOR) %d\r\n", GetCurrentAngle(FORE_ARM_MOTOR));
	LOG_DEBUG("GetCurrentAngle(WRIST_MOTOR) %d\r\n", GetCurrentAngle(WRIST_MOTOR));
}

/**
 * �ݒ�p�x���ڕW�p�x(���X�e�b�v�֐i��ŗǂ��p�x) �ɂȂ�܂œ��삷�� 
 * @param motorId     ���[�^ID
 * @param speed       �ݒ葬�x
 * @param angle       �ݒ�p�x
 * @param targetangle �ڕW�p�x(���X�e�b�v�֐i��ŗǂ��p�x) 
 */
void executeRotate(int motorId, int speed, int angle, int targetangle){
	//�ݒ�p�x�ւ̓�������s
	MotorControlJoint( motorId, speed, angle );

	// �ڕW�p�x�ɒB���Ă��Ȃ��Ԃ͓��삷��
	while( targetangle > GetCurrentAngle(motorId) )
	{
		// �ݒ�p�x�ւ̓�����Ď��s
//		MotorControlJoint( motorId, speed, angle );
		_delay_ms(100);//�K�؂ȃE�F�C�g���Ԃ�ݒ�
	}
}

/**
 * ���݂̃��[�^�p�x�擾
 * @brief  ���݂̃��[�^�p�x�擾
 * @param  motorId ���[�^ID
 * @return ���݂̊p�x
 * @detail ��ʃo�C�g2bit�A����8bit���猻�݂̊p�x���擾����B
 *         �p�P�b�g�ʐM���s���A�O��̑��x��Ԃ��B
 *         �o�͎��F0�`300���A0�`1023�@�������F150�� = 512
 *         300�`360���̊ԁF�s��l
 *
 *         ���L�̃��[�^ID����̓p�����[�^�Ƃ��邱�ƁB
 *         SHOULDER_MOTOR       12      // Shoulder Motor address(�����[�^)
 *         UPPER_ARM_MOTOR      25      // Upper arm Motor address(��r���[�^)
 *         FORE_ARM_MOTOR       14      // ForeArm Motor address(�O�r���[�^)
 *         WRIST_MOTOR          23      // Wrist Motor address(��񃂁[�^)
 */
int GetCurrentAngle(int motorId) {
	int readValueHigh = 0;	// ��ʃo�C�g
	int readValueLow = 0;	// ���ʃo�C�g
	static int angle = 0;	// ���݂̈ʒu
	
	// ��ʃo�C�g�擾
	readValueHigh = dxl_read_byte(motorId, CTRL_TBL_ADDR_PRESENT_POSITION_H) & 0x03;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// �p�P�b�g�ʐM���s���A�O��l��Ԃ��B
		return angle;
	}
	// ���ʃo�C�g�擾
	readValueLow  = dxl_read_byte(motorId, CTRL_TBL_ADDR_PRESENT_POSITION_L) & 0xFF;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// �p�P�b�g�ʐM���s���A�O��l��Ԃ��B
		return angle;
	}
	// ��ʃo�C�g�Ɖ��ʃo�C�g���猻�݂̈ʒu���v�Z
	angle = ((readValueHigh << 8) + readValueLow);
	LOG_DEBUG("GetCurrentAngle() is %d\n", angle);

	return angle;
}
