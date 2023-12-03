
#include "../geometry/spline.h"

template<typename T> T Spline<T>::at(float time) const {

	// A4T1b: Evaluate a Catumull-Rom spline

	// Given a time, find the nearest positions & tangent values
	// defined by the control point map.

	// Transform them for use with cubic_unit_spline

	// Be wary of edge cases! What if time is before the first knot,
	// before the second knot, etc...

	if(knots.empty()) return T();

	if(knots.size() == 1) return knots.begin()->second;

	if(time <= knots.begin()->first) return knots.begin()->second;

	if(time >= std::prev(knots.end())->first) return std::prev(knots.end())->second;

	auto k2 = knots.upper_bound(time);
	auto k1 = std::prev(k2);
	T tangent0, tangent1;

	if(k1 == knots.begin()){
		tangent0 = (k2->second - k1->second) / (k2->first - k1->first);
	}else{
		tangent0 = (k2->second - std::prev(k1)->second) / (k2->first - std::prev(k1)->first);
	}

	if(k2 == std::prev(knots.end())){
		tangent1 = (k2->second - k1->second) / (k2->first - k1->first);
	}else{
		tangent1 = (std::next(k2)->second - k1->second) / (std::next(k2)->first - k1->first);
	}

	tangent0 *= (k2->first - k1->first);
	tangent1 *= (k2->first - k1->first);

	float time_normalized = (time - k1->first) / (k2->first - k1->first);

	return cubic_unit_spline(time_normalized, k1->second, k2->second, tangent0, tangent1);
}

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

	// A4T1a: Hermite Curve over the unit interval

	// Given time in [0,1] compute the cubic spline coefficients and use them to compute
	// the interpolated value at time 'time' based on the positions & tangents

	// Note that Spline is parameterized on type T, which allows us to create splines over
	// any type that supports the * and + operators.

	float time_square = time * time;
	float time_cube = time * time * time;

	float h00 = 2.0f * time_cube - 3.0f * time_square + 1.0f;
	float h10 = time_cube - 2.0f * time_square + time;
	float h01 = -2.0f * time_cube + 3.0f * time_square;
	float h11 = time_cube - time_square;

	return h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;
}

template class Spline<float>;
template class Spline<double>;
template class Spline<Vec4>;
template class Spline<Vec3>;
template class Spline<Vec2>;
template class Spline<Mat4>;
template class Spline<Spectrum>;
