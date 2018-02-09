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
        double dn;
        double epsilon;
        const char* filename;

		void initLocalGCs(const char* pointsFile, const char* normalsFile);
		void mergeLocalGCs();

		Shape(){
			C = 0.1;
			alpha = 1.0;
            tau = 0.12;
		}
		Shape(double _C, double _alpha, double _tau, double _espilon, double _dn, const char* _filename){
			C = _C;
			alpha = _alpha;
			tau = _tau;
			epsilon = _espilon;
			dn = _dn;
			filename = _filename;
		}

};

#endif // SHAPE_H
