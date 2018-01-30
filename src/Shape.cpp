#include "Shape.h"

// Initialization of local GC from a point and the rosa plane normal
// TODO: remove the nbProfiles argument, not needed. Just the axis are needed for locgal Gcs.
void Shape::initLocalGCs(const char* pointsFile, const char* normalsFile, double epsilon){

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
    // int nb1 = 0;
    // int nb2 = 0;
    // int nb3 = 0;
    // int nb4 = 0;
    // int nb5 = 0;
    // int nb6 = 0;
    std::cout << "There are " << nb_GC <<" potential local GCs" << std::endl;
    for(int index = 0; index < nb_GC; index++){

    	std::cout << "Iteration " << index << std::endl;
    	pFile >> x >> y >> z;
    	nFile >> u >> v >> w;
        if(index == 47 || index == 64 || index == 65|| index == 67|| index == 71 || index == 207 || index == 249 || index == 256 || index == 276 || index == 281 || index == 304 || index == 317
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
		localGC.computeCylindricity(C, alpha);
		// if(cylindricity < 0.1){
		// 	nb1++;
		// }else if (cylindricity < 0.2){
		// 	nb2++;
		// }else if (cylindricity < 0.3){
		// 	nb3++;
		// }else if (cylindricity < 0.4){
		// 	nb4++;
		// }else if (cylindricity < 0.5){
		// 	nb5++;
		// }else{
		// 	nb6++;
		// }
        std::cout << "Cylindricity = " << localGC.cylindricity << std::endl;
		if(localGC.cylindricity < tau){
			localGCs.push_back(localGC);
		}
		// if(index%2 == 0){
		// 	localGCs.push_back(localGC);
		// }

    }
    std::cout << "End of initLocalGCs. localGCs size is " << localGCs.size() << std::endl;
    // std::cout <<"[0, 0.1[: " << nb1 << " ,[0.1, 0.2[= " << nb2 << " ,[0.2, 0.3[= " << nb3 << " ,[0.3, 0.4[= " << nb4 << " ,[0.4, 0.5[= " << nb5 << " ,[0.5, ...[= " << nb6 << std::endl;
}

void Shape::mergeLocalGCs(){

	std::cout << "Starting merging local GCs ..."<<std::endl;

	int nbLocalGCs = localGCs.size();
	std::vector<bool> isMerged(nbLocalGCs, false);
	std::vector<GC> tmp; // Stores current state of non local GCs.
	// tmp.push_back(localGCs[0]);
	// isMerged[0] = true;
	bool done = false;
	int generatorIndex = 0;

	while(!done){

		GC gcA = localGCs[generatorIndex];
		isMerged[generatorIndex] = true;
		// tmp.push_back(gcA);
		int size = tmp.size();
        std::cout << "Currently " << nonLocalGCs.size() <<" non local GCs." << std::endl;
        GC nonLocalGC = gcA;
		int index = 0;
		while(index < nbLocalGCs){

			// If localGCs[index] has not been merged yet
			if(!isMerged[index]){

				// Get GC to be merge
				GC gcB = localGCs[index];
				GC merge = nonLocalGC.merge(gcB);
                double threshold =  nonLocalGC.cylindricity + gcB.cylindricity;
				double cylindricityMerge = merge.computeCylindricity(C, alpha);
				if(cylindricityMerge < threshold){ // Merge is valid

					std::cout << "Another merged for generator " << generatorIndex << std::endl;
					// Set added gc as merged
					isMerged[index] = true;
					// Replace current nonlocal gc with the new merge
					// tmp[size-1] = merge;
					nonLocalGC = merge;
				}
			}
			index++;
		}
		nonLocalGCs.push_back(nonLocalGC);

		assert(isMerged.size() == localGCs.size());
		int notmerged = 0;
		for(int j = 0; j < isMerged.size(); j++){
			if(!isMerged[j]){
				notmerged++;
			}
		}
		std::cout <<"Currently " << notmerged <<" unmerged local GCs." << std::endl;
		// Search for next generator
		bool foundNewGenerator = false;
		for(int i = 0; i < nbLocalGCs; i++){

			if(!isMerged[i]){
				foundNewGenerator = true;
				generatorIndex = i;
				break;
			}
		}
		if(!foundNewGenerator){

			done = true;
		}

	}

	// nonLocalGCs = tmp;
	std::cout << "End of mergeLocalGCs. nonLocalGCs size is " << nonLocalGCs.size() << std::endl;
}
