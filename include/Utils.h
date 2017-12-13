#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>
#include <algorithm>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/array.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <CGAL/aff_transformation_tags.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/squared_distance_3.h>
#include <CGAL/Line_3.h>
#include <CGAL/Vector_3.h>
#include "Vec3.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3  Point_3;
typedef CGAL::Surface_mesh<Point_3> CGAL_Mesh;
typedef std::vector<K::Point_3> Polyline_type;
typedef std::vector<Polyline_type > Polylines;
typedef CGAL::cpp11::array<std::size_t,3> Facet;

typedef CGAL::Aff_transformation_3<K> Aff_transformation_3; //K kernel defined in intersect.h
typedef K::Vector_3  Vector_3;
typedef K::Line_3  Line_3;
typedef std::vector<std::vector<Point_3>> Vector_vector_points;

struct controlPoint_t {
	float parameter;
	float dist;
	int axis_part; // component of the axis (Polylines) the point belongs to
};

struct Construct{
	CGAL_Mesh& mesh;
	template < typename PointIterator>
	Construct(CGAL_Mesh& mesh,PointIterator b, PointIterator e)
	: mesh(mesh){
		for(; b!=e; ++b){
		  boost::graph_traits<CGAL_Mesh>::vertex_descriptor v;
		  v = add_vertex(mesh);
		  mesh.point(v) = *b;
		}
	}

	Construct& operator=(const Facet f){
		typedef boost::graph_traits<CGAL_Mesh>::vertex_descriptor vertex_descriptor;
		typedef boost::graph_traits<CGAL_Mesh>::vertices_size_type size_type;

		mesh.add_face(vertex_descriptor(static_cast<size_type>(f[0])),
		              vertex_descriptor(static_cast<size_type>(f[1])),
		              vertex_descriptor(static_cast<size_type>(f[2])));
		return *this;
	}
	Construct& operator*() { return *this; }
	Construct& operator++() { return *this; }
	Construct operator++(int) { return *this; }
};

class Utils{

	public:
		static CGAL_Mesh generateMesh(std::vector<Point_3> points){

		  //std::ifstream in((argc>1)?argv[1]:"data/half.xyz");
		  //std::vector<Point_3> points;
		  //std::vector<Facet> facets;
		  CGAL_Mesh m;
		  /*std::copy(std::istream_iterator<Point_3>(in),
		            std::istream_iterator<Point_3>(),
		            std::back_inserter(points));*/
		  Construct construct(m,points.begin(),points.end());
		  CGAL::advancing_front_surface_reconstruction(points.begin(),
		                                               points.end(),
		                                               construct);
		  return m;
		}

		// Filename need to be .off format
		static Polylines cross_section(Vec3f normal, Vec3f p, const char* filename){

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

		static Polylines cross_section(Vec3f normal, Vec3f p, CGAL_Mesh mesh){

		    Polylines polylines;
		    CGAL::Polygon_mesh_slicer<CGAL_Mesh, K> slicer(mesh);

		    float a = normal[0];
		    float b = normal[1];
		    float c = normal[2];
		    float d = -a*p[0] - b*p[1] - c*p[2];

		    //Equation of plane is a*x + b*y + c*z + d = 0
		    slicer(K::Plane_3(a, b, c, d), std::back_inserter(polylines));

		    return polylines;
		}

		static float Haussdorf(std::vector<Point_3> ci_samples, std::vector<Point_3> cj_samples){

			//The sample points are 2D points but I consider them as 3D points with y = 0 (so I can use distance method
			//from CGAL)

			int size_i = ci_samples.size();
			int size_j = cj_samples.size();

			// computes h(ci, cj)
			float max_ij = 0.f;
			for(int k = 0; k < size_i; k++){
				float shortest = std::sqrt(CGAL::squared_distance(ci_samples[k], cj_samples[0]));;
				for(int h = 1; h < size_j; h++){
					float d = std::sqrt(CGAL::squared_distance(ci_samples[k], cj_samples[h]));
					if (d < shortest){
						shortest = d;
					}
				}
				if(shortest > max_ij){
					max_ij = shortest;
				}
			}

			// computes h(cj, ci)
			float max_ji = 0.f;
			for(int h = 0; h < size_j; h++){
				float shortest = std::sqrt(CGAL::squared_distance(cj_samples[h], ci_samples[0]));
				for(int k = 1; k < size_i; k++){
					float d = std::sqrt(CGAL::squared_distance(cj_samples[h], ci_samples[k]));
					if (d < shortest){
						shortest = d;
					}
				}
				if(shortest > max_ji){
					max_ji = shortest;
				}
			}

			// returns max(h(ci, cj), h(cj, ci))
			return std::max(max_ij, max_ji);

		}

};

#endif //UTILS_H