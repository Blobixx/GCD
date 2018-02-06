#include "Shape.h"

// Initialization of local GC from a point and the rosa plane normal
void Shape::initLocalGCs(const char* pointsFile, const char* normalsFile){

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
    std::cout << "There are " << nb_GC <<" potential local GCs" << std::endl;

    for(int index = 0; index < nb_GC; index++){

    	std::cout << "Iteration " << index << std::endl;
    	pFile >> x >> y >> z;
    	nFile >> u >> v >> w;
        if(index == 32 | index == 67 || index == 199)// || index  == 102 || index  == 178 || index  == 202)
            continue;
      //   if(index == 47 || index == 64 || index == 65|| index == 67|| index == 71 || index == 207 || index == 249 || index == 256 || index == 276 || index == 281 || index == 304 || index == 317
    		// || index == 341 || index == 359 || index == 448 || index == 450  || index == 534   )
    		// continue;

		Vec3d point(x, y, z);
		Vec3d normal(u, v, w);
        Vec3d ps = point - normal*dn;
        Vec3d pe = point + normal*dn;
    	HermiteCurve axis_curve(ps, pe, normal, normal);
    	std::vector<HermiteCurve> axis;
    	axis.push_back(axis_curve);


		GC localGC(axis, ps, pe);

		localGC.computeCylindricity(C, alpha, epsilon);
		// Debug, check straightness of local GCs = C
		assert(fabs(localGC.checkStraightness - C) < 0.0000001);
        std::cout << "Cylindricity = " << localGC.cylindricity << std::endl;
		if(localGC.cylindricity < tau){
			localGCs.push_back(localGC);
		}

    }
    std::cout << "End of initLocalGCs. localGCs size is " << localGCs.size() << std::endl;
}

// void Shape::mergeLocalGCs(){

// 	std::cout << "Starting merging local GCs ..."<<std::endl;

// 	int nbLocalGCs = localGCs.size();
// 	std::vector<bool> isMerged(nbLocalGCs, false);
// 	std::vector<GC> tmp; // Stores current state of non local GCs.
// 	// tmp.push_back(localGCs[0]);
// 	// isMerged[0] = true;
// 	bool done = false;
// 	int generatorIndex = 0;
// 	int notmerged = nbLocalGCs;
// 	while(!done){

// 		GC gcA = localGCs[generatorIndex];
// 		isMerged[generatorIndex] = true;
// 		notmerged --;
// 		// tmp.push_back(gcA);
// 		int size = tmp.size();
//         std::cout << "Currently " << nonLocalGCs.size() <<" non local GCs." << std::endl;
//         GC nonLocalGC = gcA;
// 		int index = 0;
// 		while(index < nbLocalGCs){

// 			// If localGCs[index] has not been merged yet
// 			if(!isMerged[index]){

// 				// Get GC to be merge
// 				GC gcB = localGCs[index];
// 				GC merge = nonLocalGC.merge(gcB);
//                 double threshold =  nonLocalGC.cylindricity + gcB.cylindricity;
//                 double cylindricityMerge = merge.computeCylindricity(C, alpha, epsilon);
//                 std::cout << "cylindricityMerge: " << cylindricityMerge << std::endl;
// 				if(cylindricityMerge < threshold){ // Merge is valid

// 					std::cout << "Another merged for generator " << generatorIndex << std::endl;
// 					// Set added gc as merged
// 					isMerged[index] = true;
// 					notmerged--;
// 					// Replace current nonlocal gc with the new merge
// 					// tmp[size-1] = merge;
// 					nonLocalGC = merge;
// 					std::cout << "nonLocalGC: " << nonLocalGC.cylindricity << std::endl;
// 				}
// 			}
// 			index++;
// 		}
// 		std::cout << "Still " << notmerged << " local GCs" << std::endl;
// 		nonLocalGCs.push_back(nonLocalGC);

// 		// Search for next generator
// 		bool foundNewGenerator = false;
// 		for(int i = 0; i < nbLocalGCs; i++){

// 			if(!isMerged[i]){
// 				foundNewGenerator = true;
// 				generatorIndex = i;
// 				std::cout << "New generator: " << generatorIndex << std::endl;
// 				break;
// 			}
// 		}
// 		if(!foundNewGenerator){

// 			mergeNonLocalGCs();
// 			done = true;
// 		}

// 	}

// 	// nonLocalGCs = tmp;
// 	std::cout << "End of mergeLocalGCs. nonLocalGCs size is " << nonLocalGCs.size() << std::endl;
// }

void Shape::mergeLocalGCs(){

	std::cout << "Starting merging local GCs ..."<<std::endl;

	int nbLocalGCs = localGCs.size();
	// std::vector<bool> isMerged(nbLocalGCs, false);
	std::vector<GC> tmp = localGCs; // Stores current state of non local GCs.
	// tmp.push_back(localGCs[0]);
	// isMerged[0] = true;
	bool done = false;
	int generatorIndex = 0;
	int notmerged = nbLocalGCs;
	while(generatorIndex < tmp.size()){

		GC gcA = tmp[generatorIndex];
		// isMerged[generatorIndex] = true;
		// notmerged --;
		// tmp.push_back(gcA);
		int size = tmp.size();
        // std::cout << "Currently " << nonLocalGCs.size() <<" non local GCs." << std::endl;
        // GC nonLocalGC = gcA;
		int index = 0;
        std::cout << "generator " << generatorIndex << std::endl;
        while(index < tmp.size()){

			// If localGCs[index] has not been merged yet
            if(index != generatorIndex){
                std::cout << "checking " << index << std::endl;
				// Get GC to be merge
				GC gcB = tmp[index];
				GC merge = gcA.merge(gcB);
                double threshold =  gcA.cylindricity + gcB.cylindricity;
                double cylindricityMerge = merge.computeCylindricity(C, alpha, epsilon);
                // std::cout << "cylindricityMerge: " << cylindricityMerge << std::endl;
				if(cylindricityMerge < threshold){ // Merge is valid

					std::cout << "Another merged for generator " << generatorIndex << std::endl;
					// Set added gc as merged
					// isMerged[index] = true;
					// notmerged--;
					// Replace current nonlocal gc with the new merge
					// tmp[size-1] = merge;
					tmp[generatorIndex] = merge;
                    tmp.erase(tmp.begin() + index);
					if(generatorIndex > index){
						index --;
						generatorIndex--;
					}
					// std::cout << "nonLocalGC: " << nonLocalGC.cylindricity << std::endl;
				}
			}
			index++;
		}
        generatorIndex++;
        std::cout << "Currently " << tmp.size() << " non localGCs" << std::endl;
	}
	// nonLocalGCs = tmp;
	std::cout << "End of mergeLocalGCs. nonLocalGCs size is " << tmp.size() << std::endl;
	nonLocalGCs = tmp;
}
