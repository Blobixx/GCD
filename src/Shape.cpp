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
        if(index == 32 | index == 67 || index == 199)
            continue;

		Vec3d point(x, y, z);
		Vec3d normal(u, v, w);
        Vec3d ps = point - normal*dn;
        Vec3d pe = point + normal*dn;
    	HermiteCurve axis_curve(ps, pe, normal, normal);
    	std::vector<HermiteCurve> axis;
    	axis.push_back(axis_curve);


		GC localGC(axis, ps, pe, filename);

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

void Shape::mergeLocalGCs(){

	std::cout << "Starting merging local GCs ..."<<std::endl;

	int nbLocalGCs = localGCs.size();
	std::vector<GC> tmp = localGCs; // Stores current state of non local GCs.
	int generatorIndex = 0;
	while(generatorIndex < tmp.size()){

		GC gcA = tmp[generatorIndex];
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
				if(cylindricityMerge < threshold){ // Merge is valid

					std::cout << "Another merged for generator " << generatorIndex << std::endl;
					tmp[generatorIndex] = merge;
                    tmp.erase(tmp.begin() + index);
					if(generatorIndex > index){
						index --;
						generatorIndex--;
					}
				}
			}
			index++;
		}
        generatorIndex++;
        std::cout << "Currently " << tmp.size() << " non localGCs" << std::endl;
	}
	std::cout << "End of mergeLocalGCs. nonLocalGCs size is " << tmp.size() << std::endl;
	nonLocalGCs = tmp;
}
