#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H
#include "Arduino.h"
class LowPassFilter{
// https://www.sciencedirect.com/science/article/pii/S221201731400471X
// https://create.arduino.cc/projecthub/curiores/design-implement-a-digital-low-pass-filter-on-an-arduino-b6f45e

  public:
    LowPassFilter(float *a, float *b);
    ~LowPassFilter();
    void calculate_output(float input, float *output);
    void calculate_output(double input, double *output);
    void calculate_output(int input, int *output);
    void calculate_output(uint32_t input, uint32_t *output);

  private:
    float _b[3] = {0.0, 0.0, 0.0};
    float _a[2] = {0.0, 0.0};
    float _y[3] = {0.0, 0.0, 0.0};
    float _x[3] = {0.0, 0.0, 0.0};

};
#endif // LOW_PASS_FILTER_H