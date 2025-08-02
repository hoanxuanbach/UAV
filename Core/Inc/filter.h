

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#define IIR_ACC_ALPHA	0.9391
#define IIR_ACC_BETA	0.0305
#define IIR_GYR_ALPHA	0.8816
#define IIR_GYR_BETA	0.0592
#define COMP_ALPHA		0.95

typedef struct{
	float alpha;
	float beta;
	float prev_output; //y[n-1]
	float prev_input;  //x[n-1]
} IIR_Filter_1D;

typedef struct{
	IIR_Filter_1D x;
	IIR_Filter_1D y;
	IIR_Filter_1D z;
} IIR_Filter_3D;

void IIR_Filter_1D_Init(IIR_Filter_1D *f, float alpha, float beta);
float IIR_Filter_1D_Update(IIR_Filter_1D *f, float input);
void IIR_Filter_3D_Init(IIR_Filter_3D *f, float alpha, float beta);
void IIR_Filter_3D_Update(IIR_Filter_3D *f, float x_in, float y_in, float z_in, float *x_out, float *y_out, float *z_out);

typedef struct{

};

#endif /* INC_IIR_LOW_PASS_FILTER_H_ */
