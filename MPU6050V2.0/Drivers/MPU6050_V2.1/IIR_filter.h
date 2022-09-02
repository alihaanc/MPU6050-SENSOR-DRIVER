/*
 * IIR_filter.h
 *
 *  Created on: Jul 22, 2022
 *      Author: hp
 */
#include "stdio.h"
#ifndef FILTER_IIR_FILTER_H_
#define FILTER_IIR_FILTER_H_

typedef struct {
	float alpha;
	float beta;
	float out;

} IIRFirstOrder;



void IIRFirstOrder_Init(IIRFirstOrder *filter, float alpha);
float IIRFirstOrder_Filter(IIRFirstOrder *filter, float input);

#endif /* FILTER_IIR_FILTER_H_ */
