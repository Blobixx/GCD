#include "Shape.h"

// Initialization of local GC from a point and the rosa plane normal
// TODO: remove the nbProfiles argument, not needed. Just the axis are needed for locgal Gcs.
void Shape::initLocalGCs(const char* pointsFile, const char* normalsFile, double epsilon, int nbProfiles){

	std::ifstream pFile(pointsFile);
	std::ifstream nFile(normalsFile);
    if (!pFile || !nFile) {
    	std::cerr << "Problems when opening files" << std::endl;
        exit (EXIT_FAILURE);
    }
    localGCs.clear();
    int nb_GC;
    double x, y, z;
    double u, v, w;
    pFile >> nb_GC;
    nFile >> nb_GC;
    int nb1 = 0;
    int nb2 = 0;
    int nb3 = 0;
    int nb4 = 0;
    int nb5 = 0;
    int nb6 = 0;
    for(int index = 0; index < nb_GC; index++){
    	std::cout << "Iteration " << index << std::endl;
    	pFile >> x >> y >> z;
    	nFile >> u >> v >> w;
    	if( index == 64 || index == 65 || index == 217 || index == 276 || index == 281 || index == 304 || index == 317 
    		|| index == 341 )
    		continue;
		Vec3d point(x, y, z);
		Vec3d normal(u, v, w);
    	Vec3d ps = point - epsilon*normal;
    	Vec3d pe = point + epsilon*normal;
    	HermiteCurve axis_curve(ps, pe, normal, normal);
    	std::vector<HermiteCurve> axis;
    	axis.push_back(axis_curve);

    	// Ai-je besoin de calculer les profiles ????

  		// Vector_vector_point_3 profiles;

  		// for(int i = 0; i < nbProfiles; i++){

		// 	Vec3d s = axis_curve.interpolate(i/nbProfiles);
		// 	Vec3d tangent_at_s = axis_curve.get_tangent(i/nbProfiles);
		// 	Vec3d orth_plane_at_s = normalize(tangent_at_s); // The normal of the plane perpendicular to axis at p is the tangeant at p
		// 	profiles.push_back(Utils::cross_section(orth_plane_at_s, s, "../hand_mesh.off"));
		// }
		// GC localGC(axis, ps, pe, profiles);
		GC localGC(axis, ps, pe);
		// localGCs.push_back(localGC);
		double cylindricity = localGC.cylindricity(0.1, 1.0);
		if( cylindricity < 0.1){
			nb1++;
		}else if (cylindricity < 0.2){
			nb2++;
		}else if (cylindricity < 0.3){
			nb3++;
		}else if (cylindricity < 0.4){
			nb4++;
		}else if (cylindricity < 0.5){
			nb5++;
		}else{
			nb6++;
		}
		// std::cout << "Profile variation = " << cylindricity << std::endl;
		std::cout << "Cylindricity = " << cylindricity << std::endl;
		// if(index%2 == 0){
			// localGCs.push_back(localGC);
		// }
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
    std::cout <<"[0, 0.1[: " << nb1 << " ,[0.1, 0.2[= " << nb2 << " ,[0.2, 0.3[= " << nb3 << " ,[0.3, 0.4[= " << nb4 << " ,[0.4, 0.5[= " << nb5 << " ,[0.5, ...[= " << nb6 << std::endl;
}