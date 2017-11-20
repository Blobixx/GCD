#ifndef GC_H
#define GC_H

#include "intersect.h"
#include "HermiteCurve.h"
#include <vector>

struct controlPoint_t {
	float parameter;
	float dist;
	int axis_part; // component of the axis the point belongs to
}

class GC {

	public:

		Vec3f p; //Point p for which the GC has been constructed at
		std::vector<HermiteCurve> axis; //Axis = list of control points
		Vec3f ps; //Starting point of this axis
		Vec3f pe; //End point of the axis
		const char* shape = "gecko.off";
		std::vector<Polylines> profiles; //Set of profiles curves of GC

		GC(){}
		GC(std::vector<HermiteCurve> _axis, Vec3f _ps, Vec3f _pe){
			GC.axis = _axis;
			ps = _ps;
			pe = _pe;
		}
		GC(std::vector<HermiteCurve> _axis, Polylines _profiles){
			GC.axis = _axis;
			GC.profiles = _profiles;
		}

		void generate_profiles(float step);
		float cylindricity(float alpha);
		float straightness(float C);
		float profile_variation();
		controlPoint_t findMaxDistToLine(Point_3 start_point, Point_3 end_point);
		GC merge(GC b);
};

#endif // GC_H