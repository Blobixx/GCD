#ifndef SHAPE_H
#define SHAPE_H

#include <iostream>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>

#include "GC.h"
#include "Vec3.h"
#include "Mesh.h"

class Shape {

	public:

		CGAL_Mesh mesh;
		std::vector<GC> localGCs;
		std::vector<GC> nonLocalGCs;
		double C;
		double alpha;
		double tau;
		// std::vector<Vec3f> loadPointCloud();
		// // Find rosa plane at point p. Returns plane's normal.
		// std::vector<Vec3f> findRosaPlane(Vec3f p);
		void initLocalGCs(const char* pointsFile, const char* normalsFile, double espilon);
		void mergeLocalGCs();
		// void constructLocalGCs(float epsilon, int nbProfiles);
		Shape(){
			C = 0.1;
			alpha = 1.0;
            tau = 0.8;
		}
		Shape(double _C, double _alpha, double _tau){
			C = _C;
			alpha = _alpha;
			tau = _tau;
		}

};

#endif // SHAPE_H
