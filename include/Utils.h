#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
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

#include <CGAL/Simple_cartesian.h>
#include <CGAL/wlop_simplify_and_regularize_point_set.h>
#include <CGAL/number_utils.h> // for CGAL::sqrt

typedef CGAL::Surface_mesh<Point_3> CGAL_Mesh;
typedef std::vector<K::Point_3> Polyline_type;
typedef std::vector<Polyline_type > Polylines;
typedef CGAL::cpp11::array<std::size_t,3> Facet;

typedef CGAL::Aff_transformation_3<K> Aff_transformation_3; //K kernel defined in Vec3.h
typedef K::Vector_3  Vector_3;
typedef K::Line_3  Line_3;
typedef std::vector<std::vector<Point_3>> Vector_vector_point_3;

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

struct controlPoint_t {

	int axis_part; // to which hermite curve of the axis the point belongs to
	double parameter;
	double dist;

	controlPoint_t(): axis_part(0), parameter(0.0), dist(0.0) {}
	controlPoint_t(int axis_part, double parameter, double dist): axis_part(axis_part),
																parameter(parameter),
																dist(dist) {}
    bool isSameAs(const controlPoint_t _control) {
        if(axis_part == _control.axis_part && parameter == _control.parameter && dist == _control.dist){
            return true;
        }
        return false;
    };

    inline controlPoint_t& operator= (const controlPoint_t & _cp) {
            axis_part = _cp.axis_part;
            parameter = _cp.parameter;
            dist = _cp.dist;
            return (*this);
    };
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
		  // std::cout << "step 01" << std::endl;
		  Construct construct(m,points.begin(),points.end());
		  // std::cout << "step 02" << std::endl;
		  CGAL::advancing_front_surface_reconstruction(points.begin(),
		                                               points.end(),
		                                               construct);
		  // std::cout << "step 03" << std::endl;
		  return m;
		}

		static CGAL_Mesh generateMesh(Vector_vector_point_3 alignedProfilesSamples, std::vector<bool> control_profiles_indices){

			std::vector<Point_3> points;
			for(int i = 0; i < control_profiles_indices.size(); i++){
				if(control_profiles_indices[i]){
					points.insert(points.end(), alignedProfilesSamples[i].begin(), alignedProfilesSamples[i].end()-1);
				}
			}

			CGAL_Mesh m;
			/*std::copy(std::istream_iterator<Point_3>(in),
			        std::istream_iterator<Point_3>(),
			        std::back_inserter(points));*/
			// std::cout << "step 01" << std::endl;
			Construct construct(m,points.begin(),points.end());
			// std::cout << "step 02" << std::endl;
			CGAL::advancing_front_surface_reconstruction(points.begin(),
			                                           points.end(),
			                                           construct);
			// std::cout << "step 03" << std::endl;
			return m;
		}

		// Filename need to be .off format
		static std::vector<Point_3> cross_section(Vec3d normal, Vec3d p, const char* filename){

		    Polylines polylines;
		    std::ifstream input(filename);
		    CGAL_Mesh mesh;
		    if (!input || !(input >> mesh) || mesh.is_empty()
		                         || !CGAL::is_triangle_mesh(mesh)) {
		        std::cerr << "Not a valid input file." << std::endl;
		        return getAllPoints(polylines);
		    }
		    // Slicer constructor from the mesh
		    CGAL::Polygon_mesh_slicer<CGAL_Mesh, K> slicer(mesh);

		    double a = normal[0];
		    double b = normal[1];
		    double c = normal[2];
		    double d = -a*p[0] - b*p[1] - c*p[2];

		    //Equation of plane is a*x + b*y + c*z + d = 0
		    slicer(K::Plane_3(a, b, c, d), std::back_inserter(polylines));

		    return getAllPoints(polylines);
		}

		/*static Polylines cross_section(Vec3f normal, Vec3f p, CGAL_Mesh mesh){

		    Polylines polylines;
		    CGAL::Polygon_mesh_slicer<CGAL_Mesh, K> slicer(mesh);

		    float a = normal[0];
		    float b = normal[1];
		    float c = normal[2];
		    float d = -a*p[0] - b*p[1] - c*p[2];

		    //Equation of plane is a*x + b*y + c*z + d = 0
		    slicer(K::Plane_3(a, b, c, d), std::back_inserter(polylines));

		    return polylines;
		}*/
		static std::vector<Point_3> cross_section(Vec3d normal, Vec3d p, CGAL_Mesh mesh){

		    Polylines polylines;
		    CGAL::Polygon_mesh_slicer<CGAL_Mesh, K> slicer(mesh);

		    double a = normal[0];
		    double b = normal[1];
		    double c = normal[2];
		    double d = -a*p[0] - b*p[1] - c*p[2];

		    //Equation of plane is a*x + b*y + c*z + d = 0
		    slicer(K::Plane_3(a, b, c, d), std::back_inserter(polylines));

		    return getAllPoints(polylines);
		}

		static double Haussdorf(std::vector<Point_3> ci_samples, std::vector<Point_3> cj_samples){

			//The sample points are 2D points but I consider them as 3D points with y = 0 (so I can use distance method
			//from CGAL)

			int size_i = ci_samples.size();
			int size_j = cj_samples.size();

			if(size_i == 0 || size_j == 0){
				if(size_i == 0){
					std::cout << "ci_samples is empty" << std::endl;
				}
				if(size_j == 0){
					std::cout << "cj_samples is empty" << std::endl;
				}
				return 0.0;
			}

			// computes h(ci, cj)
			double max_ij = 0.0;
			for(int k = 0; k < size_i; k++){

				// Initialization of shortest
				double shortest = CGAL::sqrt(CGAL::to_double(CGAL::squared_distance(ci_samples[k], cj_samples[0])));
				for(int h = 0; h < size_j; h++){

					double d = CGAL::sqrt(CGAL::to_double(CGAL::squared_distance(ci_samples[k], cj_samples[h])));
					if (d < shortest){
						shortest = d;
					}
				}
				if(shortest > max_ij){
					max_ij = shortest;
				}
			}

			// computes h(cj, ci)
			double max_ji = 0.0;
			for(int h = 0; h < size_j; h++){
				double shortest = CGAL::sqrt(CGAL::to_double(CGAL::squared_distance(cj_samples[h], ci_samples[0])));
				for(int k = 0; k < size_i; k++){
					double d = CGAL::sqrt(CGAL::to_double(CGAL::squared_distance(cj_samples[h], ci_samples[k])));
					if (d < shortest){
						shortest = d;
					}
				}
				if(shortest > max_ji){
					max_ji = shortest;
				}
			}

			return std::max(max_ij, max_ji);

		}

		static std::vector<Point_3> getAllPoints(Polylines polylines){

			std::vector<Point_3> allPoints;
			for(int i = 0; i < polylines.size(); i++){

				Polyline_type poly_type = polylines[i];
				for(int j = 0; j < poly_type.size(); j++){
					allPoints.push_back(poly_type[j]);
				}
			}
			return allPoints;
		}

		static std::vector<Point_3> sampleProfileCurve(std::vector<Point_3> points, int nb_samples){

			assert(nb_samples > 0);

			int nbPts = points.size();
			// assert(nbPts > 0);

			if( nbPts < nb_samples){
				return points;
			}

		  	std::vector<Point_3> output;

		  	int step = std::floor(nbPts/nb_samples);
		  	for(int i = 0; i < nbPts; i+= step){
		  		output.push_back(points[i]);
		  	}
		  	/*//parameters
		  	// double percent = (double)nb_samples/points.size();
		  	const double retain_percentage = 2;   // percentage of points to retain.
		 	const double neighbor_radius = 0.5;   // neighbors size.
		  	// std::cout << "before samplePoints"<<std::endl;
		  	CGAL::wlop_simplify_and_regularize_point_set
		                          <Concurrency_tag>
		                          (points.begin(),
		                           points.end(),
		                           std::back_inserter(output),
		                           retain_percentage,
		                           neighbor_radius
		                           );
		  	std::cout <<"nbSamples " << retain_percentage*points.size() << ", output size: " << output.size() << std::endl;*/
		  	return output;
		}

		static double perpendicularDistance(Vec3d pt, Vec3d lineStart, Vec3d lineEnd){

            const Vec3d lineDir = lineEnd - lineStart;
            const Vec3d m = lineStart - pt;

            Vec3d tmp = cross(m, lineDir);
            double l = tmp.length();
            double dirL = lineDir.length();
            double d = l/dirL;

			return d;
		}

};

#endif //UTILS_H
