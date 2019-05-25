#include <user_math.h>



/****
	*@note :振幅限制（限幅）
	*@param[in] : 限幅对象
	*/
void amplitude_limit(float *object, float limit)
{
	if((*object) > limit) *object = limit;
	if((*object) < -limit) *object  = -limit;
}

/****
    *@brief 死区
    *@param[in] object      对象（误差）
    *@param[in] dead_lim	死区值
    */
float add_dead_limit(float *object, float dead_lim)
{
	if(ABS(*object) < dead_lim)  *object = 0;
		else *object = *object;
	return (*object);
}

/****
    *@brief 死区
    *@param[in] object      对象（误差）
    *@param[in] dead_lim	死区值
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
