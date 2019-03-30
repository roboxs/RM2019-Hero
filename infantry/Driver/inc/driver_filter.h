#ifndef _DRIVER_FILTER_H
#define _DRIVER_FILTER_H

typedef struct{
	double raw_value;
	double xbuf[18];
	double ybuf[18];
	double filtered_value;
}Filter_t;


double Chebyshev50HzLPF(Filter_t *F);


#endif

