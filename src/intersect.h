// Adapted from CGAL examples

#ifndef INTERSECT_H
#define INTERSECT_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <fstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> CGAL_Mesh;
typedef std::vector<K::Point_3> Polyline_type;
//typedef std::list< Polyline_type > Polylines;
typedef std::vector<Polyline_type > Polylines;
typedef K::Point_3 Point_3;

Polylines cross_section(Vec3f normal, Vec3f p, const char* filename){

    Polylines polylines;
    std::ifstream input(filename);
    CGAL_Mesh mesh;
    if (!input || !(input >> mesh) || mesh.is_empty()
                         || !CGAL::is_triangle_mesh(mesh)) {
        std::cerr << "Not a valid input file." << std::endl;
        return polylines;
    }
    // Slicer constructor from the mesh
    CGAL::Polygon_mesh_slicer<CGAL_Mesh, K> slicer(mesh);

    float a = normal[0];
    float b = normal[1];
    float c = normal[2];
    float d = -a*p[0] - b*p[1] - c*p[2];

    //Equation of plane is a*x + b*y + c*z + d = 0
    slicer(K::Plane_3(a, b, c, d), std::back_inserter(polylines));

    return polylines;
}

#endif //INTERSECT_H