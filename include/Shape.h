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
		Shape(){}
		std::vector<GC> localGCs;
		// std::vector<Vec3f> loadPointCloud();
		// // Find rosa plane at point p. Returns plane's normal.
		// std::vector<Vec3f> findRosaPlane(Vec3f p);
		void initLocalGCs(const char* pointsFile, const char* normalsFile, double espilon, int nbProfiles);
		// void constructLocalGCs(float epsilon, int nbProfiles);

};

#endif // SHAPE_H