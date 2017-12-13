#ifndef HERMITECURVE_H
#define HERMITECURVE_H

#include "Vec3.h"

class HermiteCurve {
	public:
		Vec3f ps;
		Vec3f pe;
		Vec3f ts;
		Vec3f te;

		HermiteCurve(Vec3f _ps, Vec3f _pe, Vec3f _ts, Vec3f _te){
			ps = _ps;
			pe = _pe;
			ts = _ts;
			te = _te;
		}

		// s in [0;1]
		Vec3f interpolate(float s){
			Vec3f _value = (2.f*s*s*s - 3.f*s*s +1.f)*ps +
				(s*s*s - 2.f*s*s +s)*ts +
				(-2.f*s*s*s + 3.f*s*s)*pe +
				(s*s*s -s*s)*te;

			return _value;
		}

		Vec3f get_tangent(float s){
			Vec3f _value = (6.f*s*s - 6.f*s)*ps +
				(3*s*s - 4.f*s +1.f)*ts +
				(-6.f*s*s + 6.f*s)*pe +
				(3.f*s*s -2.f*s)*te;

			return _value;
		}

		inline HermiteCurve& operator= (const HermiteCurve & _axis) {
                ps = _axis.ps;
                pe = _axis.pe;
                ts = _axis.ts;
                te = _axis.te;
                return (*this);
        };
};
#endif // HERMITECURVE_H