/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��hmc5883.c
 * ����     ��hmc5883����
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
*********************************************************************************/
#include "include.h"

//��ǰ�ų������ֵ����Сֵ
int16_t  HMC58X3_limit[6]={0};
int16_t  *mag_limt = HMC58X3_limit;
int16_t magdata[6]={0};

/*====================================================================================================*/
/*====================================================================================================*
**���� : Init_HMC5883L
**���� : ָ�����ʼ��
**���� : None
**ݔ�� : ״̬
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
u8 Init_HMC5883L(void)
{
	u8 ack; 
	
	ack = Single_Read(MAG_ADDRESS, 0x0A);
	
	if (!ack)
			return FALSE;

	// leave test mode
	Single_Write(MAG_ADDRESS, HMC58X3_R_CONFA, 0x78);   // 0x70Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 75Hz ; normal measurement mode
	Single_Write(MAG_ADDRESS, HMC58X3_R_CONFB, 0x20);   //0x20 Configuration Register B  -- 001 00000    configuration gain 1.33Ga
	Single_Write(MAG_ADDRESS, HMC58X3_R_MODE, 0x00);    // Mode register             -- 000000 00    continuous Conversion Mode
	delay(100);

	return TRUE;	 
}
	
/*====================================================================================================*/
/*====================================================================================================*
**���� : hmc5883lRead
**���� : ��ȡ�ش�����
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void HMC5883lRead(int16_t *magData)
{
	u8 buf[6],cy,con=0;
	int16_t mag[3];
	static u8 onc=1;
	static int32_t An[3] = {0,0,0};
	
	// ��ȡ�Ĵ�������
	I2C_Read(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);

	// ʮλ����˲�
	An[0] -= An[0]/10;
	An[0] += (int16_t)(buf[0] << 8 | buf[1]);
	mag[0] = An[0]/10;

	An[1] -= An[1]/10;
	An[1] += (int16_t)(buf[4] << 8 | buf[5]);
	mag[1] = An[1]/10;

	An[2] -= An[2]/10;
	An[2] += (int16_t)(buf[2] << 8 | buf[3]);
	mag[2] = An[2]/10;
  magdata[0]=mag[2];
  magdata[1]=buf[2];
	magdata[2]=buf[3];

	//��ҪУ׼
	if(flag.calibratingM) {
		onc=1;
		flag.MagIssue = 0;
		Mag_Calibration(mag);
	}
 
	if(onc){
		onc=0;
		
		// ���������ֵ��ƫС ˵���ش������⣬ͣ�õش�  
		for(cy=0;cy<6;cy++)	{
			if(absu16(*(mag_limt+cy))<20)	con++;
		}
		if(con>=2) flag.MagIssue = 1;
  }
	
	// ����
	for(cy=0;cy<3;cy++)
		*(magData+cy) = (fp32)(mag[cy] -(*(mag_limt+cy+3) + *(mag_limt+cy))/2);
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : Mag_Calibration
**���� : �ش�У׼
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Mag_Calibration(int16_t *array)
{
	u8 cy;
	static u8  clen_flag=1; 
	static fp32 x,y,z; 
	
	//У׼֮ǰ�Ȱ�֮ǰ��������
	if(clen_flag){
		clen_flag = 0;
		x=y=z=0;
		for(cy=0;cy<6;cy++)
			*(mag_limt+cy)=0;
	}
  
	// ��ʼ�ɼ� Ѱ���������������Сֵ
	for(cy=0;cy<3;cy++){
		if(*(mag_limt+cy)> *(array+cy)) *(mag_limt+cy) = *(array+cy);  //����С

		else if(*(mag_limt+cy+3)<*(array+cy)) *(mag_limt+cy+3) = *(array+cy);  //�����
	}
	//��������жϽ��еش�У׼�Ķ������ü��ٶȼ��ж��Ƿ�ֱ�������������ж��Ƿ�ת����360��
	if(flag.calibratingM == 1 && (absu16(sensor.acc.averag.z) > 5000))   {
	  z += sensor.gyro.radian.z * Gyro_G * 0.002f;
		if(absFloat(z)>360)  flag.calibratingM = 2;
	}
	
	if(flag.calibratingM == 2 && (absu16(sensor.acc.averag.x) > 5000))   {
	  x += sensor.gyro.radian.x * Gyro_G * 0.002f;
		if(absFloat(x)>360)  flag.calibratingM = 3;
	}
	
	if(flag.calibratingM == 3 && (absu16(sensor.acc.averag.y) > 5000))   {
	  y += sensor.gyro.radian.y * Gyro_G * 0.002f;
		if(absFloat(y)>360)  {
			clen_flag = 1;
			flag.calibratingM = 0;
			EE_SAVE_MAG_OFFSET();
		}
	}	
}
