#ifndef GC_H
#define GC_H

#include "Utils.h"
#include "HermiteCurve.h"
#include <vector>
#include <algorithm>

class GC {

	public:

		Vec3d p; // Point p for which the local GC has been constructed at
		Vec3d rosa_normal; // Normal of the original local GC
		std::vector<HermiteCurve> axis; //Axis = list of control points
		Vec3d ps; // Starting point of this axis
		Vec3d pe; // End point of the axis
		const char* shape; // Mesh model

		Vector_vector_point_3 profiles;
		std::vector<Point_3> profilesCentroids;
		std::vector<Vec3d> profilesNormals;

		Vector_vector_point_3 alignedProfilesSamples;
		Vector_vector_point_3 alignedProfiles;
		Vector_vector_point_3 alignedAndParallelProfiles;
		std::vector<Point_3> alignedCentroids;

		int nb_profile_samples;
		double cylindricity;
		double checkStraightness;

		GC(Vec3d _p, Vec3d _normal){
			p = _p;
			rosa_normal = _normal;
		}

		GC(std::vector<HermiteCurve> _axis, Vec3d _ps, Vec3d _pe, bool computeProfile = true){
			axis = _axis;
			ps = _ps;
			pe = _pe;
			checkStraightness = 0.0;
			if(computeProfile)
				computeProfiles();
		}

		GC(std::vector<HermiteCurve> _axis, Vec3d _ps, Vec3d _pe, const char* filename, bool computeProfile = true){
			axis = _axis;
			ps = _ps;
			pe = _pe;
			shape = filename;
			checkStraightness = 0.0;
			if(computeProfile)
				computeProfiles();
		}

		GC(std::vector<HermiteCurve> _axis, Vec3d _ps, Vec3d _pe, Vector_vector_point_3 _profiles){
			axis = _axis;
			ps = _ps;
			pe = _pe;
			for(int i = 0; i < _profiles.size(); i++){

				std::vector<Point_3> points = _profiles[i];
				profiles.push_back(points);

			}
		}
		GC(std::vector<HermiteCurve> _axis, Vector_vector_point_3 _profiles){
			axis = _axis;
			profiles.resize(0);
			for(int i = 0; i < _profiles.size(); i++){

				std::vector<Point_3> points = _profiles[i];
				profiles.push_back(points);

			}
		}

		inline GC& operator= (const GC& _gc) {
                ps = _gc.ps;
                pe = _gc.pe;
                axis = _gc.axis;
                cylindricity = _gc.cylindricity;
                profilesNormals = _gc.profilesNormals;
                profilesCentroids = _gc.profilesCentroids;
                for(int i = 0; i < profiles.size(); i++){
                	profiles[i].clear();
                	profiles[i].resize(0);
                }
                profiles.clear();
                profiles.resize(_gc.profiles.size());

                for(int i = 0; i < _gc.profiles.size(); i++){
                	std::vector<Point_3> pts = _gc.profiles[i];
                	for(int j = 0; j < pts.size(); j++){
                        profiles[i].push_back(pts[j]);
                	}
                }
                shape  = _gc.shape;
                return (*this);
        };

		double computeCylindricity(double C, double alpha, double epsilon);
		double straightness(double C, double epsilon);
		double profileVariation(double epsilon);
        void computeProfiles();
		GC merge(GC b);
        double generateApproximatedProfileCurves(Vector_vector_point_3 profiles_samples, double epsilon, Vec3d n);
		// Returns a vector with all the points belonging to the GC
		std::vector<Point_3> getAllPoints();
        void DP(controlPoint_t controlPtA, controlPoint_t controlPtB, double& _straightness, double _epsilon);
        void DP(controlPoint controlPtA, controlPoint controlPtB, double& _straightness, int step, double _epsilon);
		~GC();
};

#endif // GC_H
