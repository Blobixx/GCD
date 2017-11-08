#ifndef GC_H
#define GC_H 

#include "intersect.h"
#include "HermiteCurve.h"


class GC {
	
	public:

		Vec3f p; //Point p for which the GC has been constructed at
		HermiteCurve axis; //Axis = list of control points
		Vec3f ps; //Starting point of this axis
		Vec3f pe; //End point of the axis
		const char* shape = "gecko.off";
		std::vector<Polylines> profiles; //Set of profiles curves of GC

		GC(){}
		GC(Vec3f _axis, Polylines _profiles){
			GC.axis = _axis;
			GC.profiles = _profiles;
		}

		void generate_profiles(float step);
		float cylindricity(float alpha);
		float straightness(float C);
		float profile_variation();
		GC merge(GC b);
};

#endif // GC_H