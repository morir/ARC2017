/**
 * @file AvrTimer.h
 * @brief AVR�^�C�}���䕔
 * @author f.aimano
 * @date 2017/10/20
 */

#ifndef AVRTIMER_H_
#define AVRTIMER_H_

#include "stdint.h"

//#define ENABLE_AVRTIMER	// AVR�^�C�}�[��L���ɂ���i�������Łj

/**
 * AVR�^�C�}�J�n
 * @brief AVR�^�C�}�J�n
 */
void AvrTimerStart();
 
/**
 * AVR�^�C�}�擾
 * @brief AVR�^�C�}�擾
 */
int32_t AvrTimeGet();
 
/**
 * AVR�^�C�}�I��
 * @brief AVR�^�C�}�I��
 */
void AvrTimerEnd();
 
/**
 * AVR�^�C�}������
 * @brief AVR�^�C�}������
 */
void AvrTimerReset();

#endif /* AVRTIMER_H_ */