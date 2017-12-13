#ifndef GC_H
#define GC_H

#include "Utils.h"
#include "HermiteCurve.h"
#include <vector>
#include <algorithm>

class GC {

	public:

		Vec3f p; // Point p for which the local GC has been constructed at
		Vec3f rosa_normal; // Normal of the original local GC
		std::vector<HermiteCurve> axis; //Axis = list of control points
		Vec3f ps; // Starting point of this axis
		Vec3f pe; // End point of the axis
		const char* shape; // Mesh model
		// Set of profiles curves of GC
		std::vector<Polylines> profiles;
		std::vector<CGAL_Mesh> approximatedShapes;
		std::vector<Polylines> aligned_profiles;
		// Centroids of each profiles curves aligned on a straigt line
		std::vector<Point_3> aligned_centroids;
		int nb_profile_samples = 6;

		GC(Vec3f _p, Vec3f _normal){
			p = _p;
			rosa_normal = _normal;
			shape = "../hand_mesh.off";
		}

		GC(std::vector<HermiteCurve> _axis, Vec3f _ps, Vec3f _pe){
			axis = _axis;
			ps = _ps;
			pe = _pe;
			shape = "../hand_mesh.off";
		}
		GC(std::vector<HermiteCurve> _axis, Vec3f _ps, Vec3f _pe, std::vector<Polylines> _profiles){
			axis = _axis;
			ps = _ps;
			pe = _pe;
			shape = "../hand_mesh.off";
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
		}

		GC(std::vector<HermiteCurve> _axis, std::vector<Polylines> _profiles){
			axis = _axis;
			shape = "../hand_mesh.off";
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
		}

		float cylindricity(float C, float alpha);
		float straightness(float C);
		float profileVariation();
		// Finds point with the maximun distant to the line between start_point and end_point
		controlPoint_t findMaxDistToLine(Point_3 start_point, Point_3 end_point);
		// Samples the profile curve with nb_samples samples points.
		std::vector<Point_3> sampleProfileCurve(Polylines profile, int nb_samples);
		GC merge(GC b);
		float generateApproximatedProfileCurves(Vector_vector_points profiles_samples);
		// Returns a vector with all the points belonging to the GC
		std::vector<Point_3> getAllPoints();
};

#endif // GC_H