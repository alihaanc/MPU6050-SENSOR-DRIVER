/*
 * IIR_filter.c
 *
 *  Created on: Jul 22, 2022
 *      Author: Alihan
 */

/* Implementation a discrete-time first-order IIR filter.
 alpha =0 -> no filtering, alpha = 1 ->max filtring.*/

#include "IIR_filter.h"
#include "MPU_V2.1.h"


void IIRFirstOrder_Init(IIRFirstOrder *filter, float alpha) {
	/*IIR filter Alpha is 0-1 it depends your Sampling time */
	filter->alpha = alpha;
	filter->beta= (1-filter->alpha);
	filter->out   = 0.0f;
}

float IIRFirstOrder_Filter(IIRFirstOrder *filter, float input) {
	filter->out = filter->alpha * filter->out + filter->beta * input;
	return filter->out;
}

/*Resources
 https://www.youtube.com/c/PhilS94
 https://www.youtube.com/watch?v=whSw42XddsU&t=2s*/

