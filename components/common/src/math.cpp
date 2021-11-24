/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "math.hpp"

#include <cmath>

/* Local functions */
static float NormalQuantile(float p);
static float NormalCdf(float u);

/* External functions */
namespace measurements::radar
{
    float InverseChiSquareDistribution(const float x, const float df) {
        if (x == 0.0f)
            return 0.0f;
        else
            return df * std::pow(1.0f - 2.0f / (9.0f * df) + std::sqrt(2.0f / (9.0f * df)) * NormalQuantile(x), 3.0f);
    }
}   //  namespace measurements::radar

/* Helpers */
static float NormalQuantile(float p)
{
	float q, t, u;

	static const float a[6] = {-3.969683028665376e+01f,  2.209460984245205e+02f, -2.759285104469687e+02f,  1.383577518672690e+02f, -3.066479806614716e+01f, 2.506628277459239e+00f};
	static const float b[5] = {-5.447609879822406e+01f,  1.615858368580409e+02f, -1.556989798598866e+02f,  6.680131188771972e+01f, -1.328068155288572e+01};
	static const float c[6] = {-7.784894002430293e-03f, -3.223964580411365e-01f, -2.400758277161838e+00f, -2.549732539343734e+00f,  4.374664141464968e+00f,	2.938163982698783e+00f};
	static const float d[4] = { 7.784695709041462e-03f,  3.224671290700398e-01f,  2.445134137142996e+00f,  3.754408661907416e+00f};

	q = std::min(p, 1.0f - p);

	if (q > 0.02425f) {
		/* Rational approximation for central region. */
		u = q - 0.5f;
		t = u * u;
		u = u * (((((a[0] * t + a[1]) * t + a[2]) * t + a[3]) * t + a[4]) * t + a[5]) / (((((b[0] * t + b[1]) * t + b[2]) * t + b[3]) * t + b[4]) * t + 1.0f);
	} else {
		/* Rational approximation for tail region. */
		t = std::sqrt(-2.0f * std::log(q));
		u = (((((c[0] * t + c[1]) * t + c[2]) * t + c[3]) * t + c[4]) * t + c[5]) / ((((d[0] * t + d[1]) * t + d[2]) * t + d[3]) * t + 1.0f);
	}

	/* The relative error of the approximation has absolute value less
	than 1.15e-9.  One iteration of Halley's rational method (third
	order) gives full machine precision... */
	t = NormalCdf(u) - q; /* error */
	t = t * 2.506628274631000502415765284811f * std::exp(0.5f * u * u); /* f(u)/df(u) */
	u = u - t / (1.0f + u * t / 2.0f); /* Halley's method */

	return (p > 0.5f ? -u : u);
}

static float NormalCdf(float u)
{
	static const float a[5] = {1.161110663653770e-002f, 3.951404679838207e-001f, 2.846603853776254e+001f, 1.887426188426510e+002f, 3.209377589138469e+003f};
	static const float b[5] = {1.767766952966369e-001f, 8.344316438579620e+000f, 1.725514762600375e+002f, 1.813893686502485e+003f, 8.044716608901563e+003f};
	static const float c[9] = {2.15311535474403846e-8f, 5.64188496988670089e-1f, 8.88314979438837594e00f, 6.61191906371416295e01f, 2.98635138197400131e02f, 8.81952221241769090e02f, 1.71204761263407058e03f, 2.05107837782607147e03f, 1.23033935479799725e03f};
	static const float d[9] = {1.00000000000000000e00f, 1.57449261107098347e01f, 1.17693950891312499e02f, 5.37181101862009858e02f, 1.62138957456669019e03f, 3.29079923573345963e03f, 4.36261909014324716e03f, 3.43936767414372164e03f, 1.23033935480374942e03f};
	static const float p[6] = {1.63153871373020978e-2f, 3.05326634961232344e-1f, 3.60344899949804439e-1f, 1.25781726111229246e-1f, 1.60837851487422766e-2f, 6.58749161529837803e-4f};
	static const float q[6] = {1.00000000000000000e00f, 2.56852019228982242e00f, 1.87295284992346047e00f, 5.27905102951428412e-1f, 6.05183413124413191e-2f, 2.33520497626869185e-3f};
	float y, z;

	y = std::abs(u);
	if (y <= 0.46875f * 1.4142135623730950488016887242097f) {
		/* evaluate erf() for |u| <= sqrt(2)*0.46875 */
		z = y * y;
		y = u * ((((a[0] * z + a[1]) * z + a[2]) * z + a[3]) * z + a[4]) / ((((b[0] * z + b[1]) * z + b[2]) * z + b[3]) * z + b[4]);
		return 0.5f + y;
	}

	z = 0.5f * std::exp(-0.5f * y * y);
	if (y <= 4.0f) {
		/* evaluate erfc() for sqrt(2)*0.46875 <= |u| <= sqrt(2)*4.0 */
		y = y / 1.4142135623730950488016887242097f;
		y = ((((((((c[0] * y + c[1]) * y + c[2]) * y + c[3]) * y + c[4]) * y + c[5]) * y + c[6]) * y + c[7]) * y + c[8]) 
            / ((((((((d[0] * y + d[1]) * y + d[2]) * y + d[3]) * y + d[4]) * y + d[5]) * y + d[6]) * y + d[7]) * y + d[8]);
		y = z * y;
	} else {
		/* evaluate erfc() for |u| > sqrt(2)*4.0 */
		z = z * 1.4142135623730950488016887242097f / y;
		y = 2.0f / (y * y);
		y = y * (((((p[0] * y + p[1]) * y + p[2]) * y + p[3]) * y + p[4]) * y + p[5]) / (((((q[0] * y + q[1]) * y + q[2]) * y + q[3]) * y + q[4]) * y + q[5]);
		y = z * (0.564189583547756286948f - y);
	}
	return (u < 0.0f ? y : 1.0f - y);
}
