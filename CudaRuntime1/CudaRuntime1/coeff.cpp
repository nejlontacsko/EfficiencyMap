#include "coeff.h"

coeff_Road* newCoeff(double g, double alpha)
{
    coeff_Road* c = (coeff_Road*)malloc(sizeof(coeff_Road));
    c->g = g; c->alpha = alpha;
    return c;
}

coeff_Aero* newCoeff(double rho, double beta, double area)
{
    coeff_Aero* c = (coeff_Aero*)malloc(sizeof(coeff_Aero));
    c->rho = rho; c->beta = beta; c->area = area;
    return c;
}

coeff_Trac* newCoeff(double iDiff, double eta, double delta, double rSk)
{
    coeff_Trac* c = (coeff_Trac*)malloc(sizeof(coeff_Trac));
    c->iDiff = iDiff; c->eta = eta; c->delta = delta; c->rSk = rSk;
    return c;
}

coeff_Rslt* calcCoeffs(coeff_Road* r, coeff_Aero* a, coeff_Trac* t)
{
    coeff_Rslt* c = (coeff_Rslt*)malloc(sizeof(coeff_Rslt));

    const double
        rDk = t->delta * t->rSk,
        cTrac = t->iDiff * t->eta / rDk,
        cv2 = r->g / cTrac,
        cAero = a->rho * a->beta * a->area / 2;

    c->c[4] = cv2 * r->alpha / 24;
    c->c[3] = -cv2 / 6;
    c->c[2] = -cv2 * r->alpha / 2;
    c->c[1] = cv2;
    c->c[0] = cv2 * r->alpha;

    c->cF = cAero * cv2;
    c->cOmega = 30 * M_1_PI / rDk;

    return c;
}