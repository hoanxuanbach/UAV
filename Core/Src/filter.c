#include "filter.h"
void IIR_Filter_1D_Init(IIR_Filter_1D *f, float alpha, float beta){
    f->alpha = alpha;
    f->beta = beta;
    f->prev_input = 0.0f;
    f->prev_output = 0.0f;
}
float IIR_Filter_1D_Update(IIR_Filter_1D *f, float input){
    float output = f->alpha * f->prev_output + f->beta * input + f->beta * f->prev_input;
    f->prev_input = input;
    f->prev_output = output;
    return output;
}
void IIR_Filter_3D_Init(IIR_Filter_3D *f, float alpha, float beta){
	IIR_Filter_1D_Init(&f->x, alpha, beta);
	IIR_Filter_1D_Init(&f->y, alpha, beta);
	IIR_Filter_1D_Init(&f->z, alpha, beta);
}
void IIR_Filter_3D_Update(IIR_Filter_3D *f, float x_in, float y_in, float z_in, float *x_out, float *y_out, float *z_out){
    *x_out = IIR_Filter_1D_Update(&f->x, x_in);
    *y_out = IIR_Filter_1D_Update(&f->y, y_in);
    *z_out = IIR_Filter_1D_Update(&f->z, z_in);
}
