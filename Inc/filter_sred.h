
#ifndef FILTER_SRED_H_
#define FILTER_SRED_H_

#define COUNT_FILTER	256

typedef union
{
    unsigned int Val;
    struct
    {
    	unsigned Flag:1;		//���� ���������� �����
    	unsigned Index:8;		//�������� ������� ������
    	unsigned Filter_sum:23;	//������� ����� ��� ����������
    } Reg;
} FILTER_REG;

unsigned short int	filter_sred(unsigned short int ADC_val, unsigned short int* buf, FILTER_REG* filter_reg);

#endif /* SW_TIMER_H_ */
