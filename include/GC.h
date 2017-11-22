#ifndef GC_H
#define GC_H

#include "Utils.h"
#include "HermiteCurve.h"
#include <vector>
#include <algorithm>

class GC {

	public:

		Vec3f p; //Point p for which the GC has been constructed at
		std::vector<HermiteCurve> axis; //Axis = list of control points
		Vec3f ps; //Starting point of this axis
		Vec3f pe; //End point of the axis
		const char* shape = "gecko.off";
		// Set of profiles curves of GC
		std::vector<Polylines> profiles;
		std::vector<CGAL_Mesh> approximatedShapes;
		std::vector<Polylines> aligned_profiles;
		// Centroids of each profiles curves aligned on a straigt line
		std::vector<Point_3> aligned_centroids;
		int nb_profile_samples = 6;

		GC(std::vector<HermiteCurve> _axis, Vec3f _ps, Vec3f _pe){
			axis = _axis;
			ps = _ps;
			pe = _pe;
		};

		GC(std::vector<HermiteCurve> _axis, std::vector<Polylines> _profiles){
			axis = _axis;

			profiles.resize(0);
			for(int i = 0; i < _profiles.size(); i++){
				Polylines _polylines = _profiles[i];
				Polylines copy_polylines;

				for(int j = 0; j < _polylines.size(); j++){
					Polyline_type _polytype = _polylines[j];
					Polyline_type copy_polytype;

					for(int k = 0; k < _polytype.size(); k++){
						copy_polytype.push_back(_polytype[k]);
					}
					copy_polylines.push_back(copy_polytype);
				}
				profiles.push_back(copy_polylines);
			}
		};

		float cylindricity(float alpha, float C);
		float straightness(float C);
		float profileVariation();
		controlPoint_t findMaxDistToLine(Point_3 start_point, Point_3 end_point);
		std::vector<Point_3> sampleProfileCurve(Polylines profile, int nb_samples);
		GC merge(GC b);
		float generateApproximatedProfileCurves(Vector_vector_points profiles_samples);
};

#endif // GC_H