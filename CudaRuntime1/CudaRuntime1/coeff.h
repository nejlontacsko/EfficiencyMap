#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>

typedef struct
{
	double g, alpha;
} coeff_Road;

typedef struct
{
	double rho, beta, area;
} coeff_Aero;

typedef struct
{
	double iDiff, eta, delta, rSk;
} coeff_Trac;

typedef struct
{
	float c[5], cF, cOmega;
} coeff_Rslt;

coeff_Road* newCoeff(double, double);
coeff_Aero* newCoeff(double, double, double);
coeff_Trac* newCoeff(double, double, double, double);

coeff_Rslt* calcCoeffs(coeff_Road*, coeff_Aero*, coeff_Trac*);