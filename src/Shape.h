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
#include "tiny_obj_loader.h"

class Shape {

	public:

		Mesh mesh;
		std::vector<Vec3f> loadPointCloud();
		std::vector<Vec3f> findRosaPlane(Vec3f p); //find rosa plane at point p. Returns plane's normal.
};

#endif // SHAPE_H