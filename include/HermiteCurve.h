#ifndef HERMITECURVE_H
#define HERMITECURVE_H

#include "Vec3.h"

class HermiteCurve {
	public:
		Vec3d ps;
		Vec3d pe;
		Vec3d ts;
		Vec3d te;

		HermiteCurve(Vec3d _ps, Vec3d _pe, Vec3d _ts, Vec3d _te){
			ps = _ps;
			pe = _pe;
			ts = _ts;
			te = _te;
		}

		// s in [0;1]
		Vec3d interpolate(double s){
			Vec3d _value = (2.0*s*s*s - 3.0*s*s +1.0)*ps +
				(s*s*s - 2.0*s*s +s)*ts +
				(-2.0*s*s*s + 3.0*s*s)*pe +
				(s*s*s -s*s)*te;

			return _value;
		}

		Vec3d get_tangent(double s){
			Vec3d _value = (6.0*s*s - 6.0*s)*ps +
				(3.0*s*s - 4.0*s +1.0)*ts +
				(-6.0*s*s + 6.0*s)*pe +
				(3.0*s*s -2.0*s)*te;

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