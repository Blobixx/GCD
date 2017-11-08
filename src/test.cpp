#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <fstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef std::vector<K::Point_3> Polyline_type;
typedef std::list< Polyline_type > Polylines;

int main(int argc, char* argv[]){

    const char* filename = (argc > 1) ? argv[1] : "monkey.off";
    std::ifstream input(filename);
    Mesh mesh;
    if (!input || !(input >> mesh) || mesh.is_empty()
                         || !CGAL::is_triangle_mesh(mesh)) {
        std::cerr << "Not a valid input file." << std::endl;
        return 1;
    }
    // Slicer constructor from the mesh
    CGAL::Polygon_mesh_slicer<Mesh, K> slicer(mesh); 
    Polylines polylines;
    slicer(K::Plane_3(0, 0, 1, -0.4), std::back_inserter(polylines));
    std::cout << "At z = 0.4, the slicer intersects "
                        << polylines.size() << " polylines" << std::endl;
    polylines.clear();
    slicer(K::Plane_3(0, 0, 1, 0.2), std::back_inserter(polylines));
    std::cout << "At z = -0.2, the slicer intersects "
                        << polylines.size() << " polylines" << std::endl;
    
    Polyline_type intersection = polylines.back();
    std::cout << "Nb points = " << intersection.size() << std::endl;
    for(int i=0; i<intersection.size(); i++){
        std::cout << intersection[i] << std::endl;
    }
    polylines.clear();

    return 0;
}