/**
 * @file AvrTimer.h
 * @brief AVR�^�C�}���䕔
 * @author f.aimano
 * @date 2017/10/20
 * @see http://threesons-technicalmemo.blogspot.jp/2014/03/avr16.html
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/common.h>

#include "AvrTimer.h"
#include "MotorManager.h"

extern int getSensorPatternCalledFromTimer(void);

#define F_CPU_DIV1000	(16000L)

volatile static int32_t avr16bitMTimer = 0;	// Avr��16bit�^�C�}�̌o�ߎ���(msec)

/**
 * AVR�^�C�}�J�n
 * @brief AVR�^�C�}�J�n
 */
void AvrTimerStart() {
	avr16bitMTimer = 0;
 
	//���荞�݋֎~
	cli();
 
	//���/����1����ڼ޽�A�̐ݒ�
	//�i�^�C�}�p�s���Ƃ��Ďg��Ȃ��j
	TCCR1A = 0x00; //�^�C�}�֌W�̃s���͕W���|�[�g����Ƃ���
 
	//���/����1����ڼ޽�B�̐ݒ�
	TCCR1B = (1<<CS12)|(0<<CS11)|(1<<CS10);//1024�����Ń^�C�}ON
 
	//���/����1���荞��Ͻ� ڼ޽��ݒ�
	TIMSK1 = (1<<TOIE1);//�^�C�}�P�I�[�o�[�t���[���荞�݋���
 
	//���荞�݋���
	sei();
}
 
/**
 * AVR�^�C�}�擾
 * @brief AVR�^�C�}�擾
 */
int32_t AvrTimerGet() {
	//16�r�b�g���W�X�^�̓ǂݏ����̍ۂɂ́A�e���|�������W�X�^���g�p����B
	//���̂��߁A���荞�݋֎~���삪�K�v�B
 
	//�X�e�[�^�X���W�X�^���ꎞ�ۑ�����ϐ�
	uint8_t sreg;
 
	//�X�e�[�^�X���W�X�^��ۑ�
	sreg = SREG;
 
	//���荞�݋֎~
	cli();
 
	//���݂̃^�C�}�l���擾
	uint16_t t = TCNT1;
 
	//SREG��߂��B����ɂ���Ċ��荞�݋֎~��Ԃ��߂�B�iSREG��I�r�b�g��߂�����j
	SREG = sreg;
 
	int32_t tw = (int32_t)t;
 
	return avr16bitMTimer + (tw * 1024) / F_CPU_DIV1000;
}
 
/**
 * AVR�^�C�}�I��
 * @brief AVR�^�C�}�I��
 */
void AvrTimerEnd() {
	uint8_t sreg;
	sreg = SREG;
	cli();
 
	//�^�C�}��0�ɂ���
	TCNT1 = 0;
	SREG = sreg;
 
	//�o�ߎ��Ԃ�0�ɂ���
	avr16bitMTimer = 0;
}
 
/**
 * AVR�^�C�}������
 * @brief AVR�^�C�}������
 */
void AvrTimerReset() {
	TCCR1B = 0;//�^�C�}off 
}

/**
 * TIMER1_OVF_vect�̃^�C�}���荞��
 * @brief TIMER1_OVF_vect�̃^�C�}���荞��
 */
ISR(TIMER1_OVF_vect) {
	avr16bitMTimer += (65536L * 1024L)/ F_CPU_DIV1000;
#ifdef ENABLE_AVRTIMER
	// �Z���T�[�l��Bit�p�^�[�����擾
	getSensorPatternCalledFromTimer();
	// ���݂̑��x(�E���[�^)�擾
	GetCurrentSpeedRCalledFromTimer();
	// ���݂̑��x(�����[�^)�擾
	GetCurrentSpeedLCalledFromTimer();
#endif // ENABLE_AVRTIMER
}