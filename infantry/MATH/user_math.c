#include <user_math.h>



/****
	*@note :������ƣ��޷���
	*@param[in] : �޷�����
	*/
void amplitude_limit(float *object, float limit)
{
	if((*object) > limit) *object = limit;
	if((*object) < -limit) *object  = -limit;
}

/****
    *@brief ����
    *@param[in] object      ������
    *@param[in] dead_lim	����ֵ
    */
float add_dead_limit(float *object, float dead_lim)
{
	if(ABS(*object) < dead_lim)  *object = 0;
		else *object = *object;
	return (*object);
}

/****
    *@brief ����
    *@param[in] object      ������
    *@param[in] dead_lim	����ֵ
    */
short add_rc_dead_limit(short *object, float dead_lim)
{
	if(ABS(*object) < dead_lim)  *object = 0;
		else *object = *object;
	return (*object);
}

void float_TO_Hex(unsigned char * pchMessage,float fdata)
{
	HexToFloat_t FH;
	FH.float_data=fdata;
    *(pchMessage+0)= FH.hex_data[0];
	*(pchMessage+1)= FH.hex_data[1];
	*(pchMessage+2)= FH.hex_data[2];
	*(pchMessage+3)= FH.hex_data[3];
}
