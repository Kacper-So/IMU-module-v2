#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(float *a, float *b){
    _b[0] = b[0];
    _b[1] = b[1];
    _b[2] = b[2];
    _a[0] = a[0];
    _a[1] = a[1];
}
LowPassFilter::~LowPassFilter(){

}

void LowPassFilter::calculate_output(float input, float *output){
    _x[0] = input;
    _y[0] = _a[0]*_y[1] + _a[1]*_y[2] + _b[0]*_x[0] + _b[1]*_x[1] + _b[2]*_x[2];


    *output = _y[0];

    for(int i = 1; i >= 0; i--){

        _x[i+1] = _x[i]; // store xi
        _y[i+1] = _y[i]; // store yi

    }
}
void LowPassFilter::calculate_output(double input, double *output){
    _x[0] = input;
    _y[0] = _a[0]*_y[1] + _a[1]*_y[2] + _b[0]*_x[0] + _b[1]*_x[1] + _b[2]*_x[2];


    *output = _y[0];

    for(int i = 1; i >= 0; i--){

        _x[i+1] = _x[i]; // store xi
        _y[i+1] = _y[i]; // store yi

    }
}

void LowPassFilter::calculate_output(int input, int *output){
    _x[0] = input;
    _y[0] = _a[0]*_y[1] + _a[1]*_y[2] + _b[0]*_x[0] + _b[1]*_x[1] + _b[2]*_x[2];


    *output = _y[0];

    for(int i = 1; i >= 0; i--){

        _x[i+1] = _x[i]; // store xi
        _y[i+1] = _y[i]; // store yi

    }
}
void LowPassFilter::calculate_output(uint32_t input, uint32_t *output){
    _x[0] = input;
    _y[0] = _a[0]*_y[1] + _a[1]*_y[2] + _b[0]*_x[0] + _b[1]*_x[1] + _b[2]*_x[2];


    *output = _y[0];

    for(int i = 1; i >= 0; i--){

        _x[i+1] = _x[i]; // store xi
        _y[i+1] = _y[i]; // store yi

    }
}