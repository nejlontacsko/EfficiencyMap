#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "math_constants.h"

#include <stdio.h>

#include "coeff.h"
#include "range.h"

using namespace std;

typedef enum
{
    GetTorqueAngvelPair,
    GetOneEfficiencyValue,
    GenerateMap
} operation;

typedef struct
{
    int m;
    float theta;
} dynamics;

typedef struct
{
    float Mdemand, Omega;
} motor;

//TODO: new func for dynamics and motor

typedef coeff_Rslt* coeff;

__device__ float calculate_Mdemand(dynamics* d, coeff c, float v, float i)
{
    const float h0 = ((((c->c[4] * d->theta + c->c[3]) * d->theta + c->c[2]) * d->theta + c->c[1]) * d->theta + c->c[0]);
    return (d->m * h0 + c->cF * v * v) / i;
}

__device__ float calculate_Omega(float v, coeff c)
{
    return v * c->cOmega;
}


/*__global__ void calcValues(float* p_values, float* m_values, const float* c_values, const float m, const float c_F, const float r_dk, const float i_gbx) {
    const int xIndex = blockIdx.x * blockDim.x + threadIdx.x;
    const int yIndex = blockIdx.y * blockDim.y + threadIdx.y;
    const int zIndex = blockIdx.z * blockDim.z + threadIdx.z;

    const float theta = xIndex * 0.1f;
    const float v = yIndex * 0.1f;
    const float i = zIndex * 0.1f;

    

    const int index = xIndex + yIndex * blockDim.x + zIndex * blockDim.x * blockDim.y;
    p_values[index] = h0;
    m_values[index] = Omega;
}*/

__global__ void calculate_values(float* output_mdemand, float* output_omega, float* output_color, float theta_step, float v_step, float igbx, float mdemand_coeff, float cf_coeff, float rd_k)
{
    // Calculate the thread's x, y, and z indices
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    int idz = blockIdx.z * blockDim.z + threadIdx.z;

    // Calculate the corresponding values of theta and v based on the indices
    float theta = idx * theta_step;
    float v = idy * v_step;


}

coeff_Rslt* creatCoeffs()
{
    coeff_Road* cRoad = newCoeff(9.8067, 0.01);
    coeff_Aero* cAero = newCoeff(1.2041, 1.05, 4);
    coeff_Trac* cTrac = newCoeff(4, 0.9, 0.98, 0.3);

    coeff_Rslt* cRslt = calcCoeffs(cRoad, cAero, cTrac);

    free(cRoad);
    free(cAero);
    free(cTrac);
}

int main() {
    //Prepare
    coeff_Rslt* coeffs = creatCoeffs();

    range
        * iRange = newRange(0, 33, 0.1f),
        * vRange = newRange(0, 33, 0.1f);
    
    float* Mdemand, d_Mdemand;
    float* Omega, d_Omega;

    operation op = GetTorqueAngvelPair;

    switch (op)
    {
    case GetTorqueAngvelPair:

        //Device alloc
        const int size = sizeFromRange(iRange);
        const int numValues = size * size * size;

        cudaMallocManaged((void**)&d_Mdemand, numValues * sizeof(float));
        cudaMallocManaged((void**)&d_Omega, numValues * sizeof(float));

        //Input upload
        a = 2;
        b = 7;

        cudaMemcpy(d_a, &a, size, cudaMemcpyHostToDevice);
        cudaMemcpy(d_b, &b, size, cudaMemcpyHostToDevice);

        //Perform calculations on device
        dim3 threadsPerBlock(8, 8, 8);
        dim3 numBlocks(size / threadsPerBlock.x, size / threadsPerBlock.y, size / threadsPerBlock.z);

        calcValues << <numBlocks, threadsPerBlock >> > (p_values, m_values, m, c_F, r_dk, i_gbx);

        cudaDeviceSynchronize();

        //Output download
        cudaMemcpy(&c, d_c, size, cudaMemcpyDeviceToHost);

        //Cleanup on device
        cudaFree(p_values);
        cudaFree(m_values);
        break;
    case GetOneEfficiencyValue:
        break;
    case GenerateMap:
        break;
    }

    //Cleanup on host
    free(pointers);
    return 0;
}
