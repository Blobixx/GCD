#include "Shape.h"

// Initialization of local GC from a point and the rosa plane normal
void Shape::initLocalGCs(const char* pointsFile, const char* normalsFile, float epsilon, int nbProfiles){

	std::ifstream pFile(pointsFile);
	std::ifstream nFile(normalsFile);
    if (!pFile || !nFile) {
    	std::cerr << "Problems when opening files" << std::endl;
        exit (EXIT_FAILURE);
    }
    localGCs.clear();
    int nb_GC;
    float x, y, z;
    float u, v, w;
    pFile >> nb_GC;
    nFile >> nb_GC;
    for(int index = 0; index < nb_GC; index++){
    	std::cout << "Iteration " << index << std::endl;
    	pFile >> x >> y >> z;
    	nFile >> u >> v >> w;
		Vec3f point(x, y, z);
		Vec3f normal(u, v, w);
    	Vec3f ps = point - epsilon*normal;
    	Vec3f pe = point + epsilon*normal;
    	HermiteCurve axis_curve(ps, pe, normal, normal);
    	std::vector<HermiteCurve> axis;
    	axis.push_back(axis_curve);
    	std::vector<Polylines> profiles;

    	for(int i = 0; i < nbProfiles; i++){

			Vec3f s = axis_curve.interpolate(i/nbProfiles);
			Vec3f tangent_at_s = axis_curve.get_tangent(i/nbProfiles);
			Vec3f orth_plane_at_s = normalize(tangent_at_s); // The normal of the plane perpendicular to axis at p is the tangeant at p
			profiles.push_back(Utils::cross_section(orth_plane_at_s, s, "../hand_mesh.off"));
		}
		GC localGC(axis, ps, pe, profiles);
		// localGCs.push_back(localGC);
		// float cylindricity = localGC.cylindricity(0.1f, 1.0f);
		// std::cout << "Profile variation = " << cylindricity << std::endl;
		if(index%2 == 0){
			localGCs.push_back(localGC);
		}
		/*std::cout << "localGC n°" << index <<": axis[" << axis[0].ps<<","<<axis[0].pe <<"], ps["<< ps <<"], pe[" << pe <<"]." << std::endl;
		std::cout << "On affiche les points du premier profiles" << std::endl;
		for(int u = 0; u < 1; u++){
			std::cout << " Profile n°"<<u;
			Polylines polylines = localGC.profiles[u];
			for(int v = 0; v < polylines.size(); v++){
				Polyline_type polytype = polylines[v];
				for(int w = 0; w < polytype.size(); w++){
					std::cout << ", Point n°"<<w<<" " <<polytype[w] << std::endl;
				}
			}
		}*/
    }
    std::cout << "End of initLocalGCs. localGCs size is " << localGCs.size() << std::endl;
}