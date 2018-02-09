#include "GC.h"
#include <cmath>
#include <utility>
#include <cassert>


// Douglas Peucker algorithm
void GC::DP(controlPoint controlPtA, controlPoint controlPtB, double& _straightness, int step, double epsilon){

	if(controlPtA.axis_part == controlPtB.axis_part && controlPtA.dist == controlPtB.dist &&
		controlPtA.parameter == controlPtB.parameter){
        return;
	}

    Vec3d lineStart = axis[controlPtA.axis_part].interpolate((double)controlPtA.parameter/step);
    Vec3d lineEnd = axis[controlPtB.axis_part].interpolate((double)controlPtB.parameter/step);

	double d_step = step;

	double maxDist = 0.0;
    int maxAxisPart;
    int maxParam = -1;

	// check on controlPtA axis part
	for(int t = (controlPtA.parameter+1); t <= step; t++){        double param = (double) t/d_step;
        Vec3d curPt = axis[controlPtA.axis_part].interpolate(param);
        double d = Utils::perpendicularDistance(curPt, lineStart, lineEnd);
        if(d > maxDist){
			maxDist = d;
            maxAxisPart = controlPtA.axis_part;
            maxParam = t;
		}
	}

	// check on controlPtB axis part
	for(int t = 0; t < controlPtB.parameter; t++){

        double param = (double) t/d_step;
        Vec3d curPt = axis[controlPtB.axis_part].interpolate(param);
        double d = Utils::perpendicularDistance(curPt, lineStart, lineEnd);
        if(d > maxDist){
			maxDist = d;
            maxAxisPart = controlPtB.axis_part;
            maxParam = t;
		}
	}

	// check on axis part in between
	for(int part = (controlPtA.axis_part + 1); part < controlPtB.axis_part; part++){
        for(int t = 0; t <= step; t++){

            double param = (double) t/d_step;
            Vec3d curPt = axis[part].interpolate(param);
            double d = Utils::perpendicularDistance(curPt, lineStart, lineEnd);
            if(d > maxDist){
				maxDist = d;
				maxAxisPart = part;
                maxParam = t;
			}
		}
	}
    //std::cout << "maxDist: " << maxDist << std::endl;
    if(maxDist > epsilon){

        _straightness += maxDist;
        controlPoint maxControlPt(maxAxisPart, maxParam, 0.0);
        DP(controlPtA, maxControlPt, _straightness, step, epsilon);
        DP(maxControlPt, controlPtB, _straightness, step, epsilon);
	}
}

double GC::straightness(double C, double epsilon){

	double straightness = C;
	int step = 50;
    int axisSize = axis.size();

    controlPoint axisStart_t(0, 0, 0.0);
    controlPoint axisEnd_t(axisSize-1, step, 0.0);

	DP(axisStart_t, axisEnd_t, straightness, step, epsilon);
	checkStraightness = straightness;
	return straightness;
}

// Fills up profiles, profilesCentroids and profilesNormas vectors.
void GC::computeProfiles(){

	/* ---------- SAMPLING OF AXIS --------- */

	// Sample the axis with a dense set of points.
	// Computes profile curves as cross sections with the shape perpendicular to the axis at point
	int nbCentroidsPerPart = 3; // Actually 4 centroids as the end point of the axis is not counted here
	int nbAxisParts = axis.size();

	for(int j = 0; j < nbAxisParts; j++){

		HermiteCurve hermiteCurve = axis[j];
		for(int i = 0; i < nbCentroidsPerPart; i++){

			double t  = (double)i/nbCentroidsPerPart;
			Vec3d s = hermiteCurve.interpolate(t);
			Vec3d tangentAtS = hermiteCurve.get_tangent(t);
			Vec3d orthPlaneAtS = normalize(tangentAtS); //The normal of the plane perpendicular to axis at p is the tangeant at p
			std::vector<Point_3> out = Utils::cross_section(orthPlaneAtS, s, shape);
			if(out.size() != 0){
				profiles.push_back(out);
				profilesCentroids.push_back(Point_3(s[0], s[1], s[2]));
				profilesNormals.push_back(orthPlaneAtS);
			}
		}
	}
	// Adds pe which is not added with the previous computation
	Vec3d tangentAtPe = axis[nbAxisParts-1].get_tangent(1.0);
	Vec3d orthPlaneAtPe = normalize(tangentAtPe); //The normal of the plane perpendicular to axis at p is the tangeant at p
	std::vector<Point_3> out_pe = Utils::cross_section(orthPlaneAtPe, pe, shape);
	if(out_pe.size() != 0){
		profiles.push_back(out_pe);
		profilesCentroids.push_back(Point_3(pe[0], pe[1], pe[2]));
		profilesNormals.push_back(orthPlaneAtPe);
	}

	/* ---------- END SAMPLING OF AXIS --------- */
}

double GC::profileVariation(double epsilon){

	/* ---------- TRANSFORMATION SO THAT CENTROIDS REST ON A STRAIGHT LINE --------- */

	int nbCentroids = profilesCentroids.size();
    if(nbCentroids < 2)
        return 10.0;
	Point_3 psPt = profilesCentroids[0];
	Point_3 pePt = profilesCentroids[nbCentroids-1];
	Line_3 line(psPt, pePt);
	alignedCentroids.push_back(profilesCentroids[0]);
	alignedProfiles.push_back(profiles[0]);

	// For each profile (except start and end), find the orthogonal projection
	// of the centroid on the line. Then translates the profile and its centroid
	for( int index = 1; index < profilesCentroids.size()-1; index++){

		Point_3 centroid = profilesCentroids[index];
		// Orthogonal projection of centroid on the line [ps,pe]
		Point_3 centroidProjection = line.projection(centroid);
		Vector_3 translationVector(centroid, centroidProjection); // Vector_3(a, b) return b-a vector

		// Apply translation on centroid
		Aff_transformation_3 translation(CGAL::TRANSLATION, translationVector);
		Point_3 alignedCentroid = translation.transform(centroid);
		// Stores aligned centroid
		alignedCentroids.push_back(alignedCentroid);

		// Apply translation on all the points of the profile
		std::vector<Point_3> profilePts(profiles[index]);
		std::vector<Point_3> alignedProfilePts;
		// Now apply transformations of the profile correspoding to centroid
		for(int pt = 0; pt < profilePts.size(); pt++){

			Point_3 tmp = profilePts[pt];
			Point_3 alignedPt = translation.transform(tmp);
			alignedProfilePts.push_back(alignedPt);
		}
		alignedProfiles.push_back(alignedProfilePts);
	}

	alignedCentroids.push_back(profilesCentroids[nbCentroids-1]);
	alignedProfiles.push_back(profiles[nbCentroids-1]);

	/* ---------- END OF TRANSFORMATION SO THAT CENTROIDS REST ON A STRAIGHT LINE --------- */

	/* ---------- TRANSFORMATION OF THE PROFILES SO THEY ARE PARALLEL --------- */

	// Profiles parallel to plane with normal vector(ps-pe)
	Vec3d n = ps - pe;
	n.normalize();

	for(int index = 0; index < alignedProfiles.size(); index++){

		Vec3d centroid = alignedCentroids[index];
		Point_3 centroid_3(centroid[0], centroid[1], centroid[2]); //Vec3d to Point_3

		Vec3d orth_plane_at_centroid = profilesNormals[index];
		orth_plane_at_centroid.normalize();
		double cos = dot(orth_plane_at_centroid, n);
		Vec3d v = cross(orth_plane_at_centroid, n);
		double v1 = v[0];
		double v2 = v[1];
		double v3 = v[2];

		// cases cos = 1.0 && cos = -1.0
		if( std::fabs(cos + 1.0) < 0.00001 || std::fabs(cos - 1.0) < 0.00001){

			// Profile is already orthogonal to n
			// Add the profile itself in rotated_profiles and centroid
			alignedAndParallelProfiles.push_back(alignedProfiles[index]);
			// rotatedCentroids.push_back(centroid_3);
		}else{

			double inv = 1.0/(1.0+cos);
			// Rotation to orient the plane so that its normal is n
			// This rotatation consits of aligning the plane normal with the vector n
			Aff_transformation_3 rotation(1.0-(v2*v2+v3*v3)*inv, -1.0*v3+(v1*v2)*inv, v2+(v1*v3)*inv, 0,
											v3+(v1*v2)*inv, 1.0-(v1*v1+v3*v3)*inv, -1.0*v1+(v2*v3)*inv, 0,
											-1.0*v2+(v1*v3)*inv, v1+(v2*v3)*inv, 1.0-(v1*v1+v2*v2)*inv, 0, 1.0
										);

			// Apply rotation on the profile
            std::vector<Point_3> profilePts = alignedProfiles[index];
			std::vector<Point_3> parallelProfilePts;
			for(int pt = 0; pt < profilePts.size(); pt++){

				Point_3 tmp = rotation.transform(profilePts[pt]);
				parallelProfilePts.push_back(tmp);
			}
			alignedAndParallelProfiles.push_back(parallelProfilePts);
		}
	}
	/* ---------- END OF TRANSFORMATION OF THE PROFILES SO THEY ARE PARALLEL --------- */

	/* ---- DEBUG ---- */
    assert(alignedAndParallelProfiles.size() == nbCentroids);
	// assert(originalCentroids.size() == rotatedCentroids.size());
	for(int idx = 0; idx < alignedAndParallelProfiles.size(); idx++){
		assert(alignedAndParallelProfiles[idx].size() != 0);
	}
	/* ---- END DEBUG ---- */


	nb_profile_samples = 5;
	// Vector containing the sample points for each curve
	alignedProfilesSamples.resize(alignedAndParallelProfiles.size());
	for(int i = 0; i < alignedAndParallelProfiles.size(); i++){

		assert(alignedAndParallelProfiles[i].size() != 0);
		alignedProfilesSamples[i] = Utils::sampleProfileCurve(alignedAndParallelProfiles[i], nb_profile_samples);
		assert(alignedProfilesSamples[i].size() != 0);
	}

	double profileVariation = generateApproximatedProfileCurves(alignedProfilesSamples, epsilon, n);
	profileVariation += Utils::Haussdorf(alignedProfilesSamples[0], alignedProfilesSamples[nbCentroids-1]);

	return profileVariation;
}

double GC::generateApproximatedProfileCurves(Vector_vector_point_3 profiles_samples, double epsilon, Vec3d n){

	// std::cout << "Starting generateApproximatedProfileCurves" << std::endl;
	int nb_profiles = alignedAndParallelProfiles.size();
	if(nb_profiles < 2){
		std::cout << "Weird, not enough profiles" << std::endl;
		return 0.0;
	}

	// Vector to check if a profile has already been added to the approximated shape
	std::vector<bool> control_profiles_indices(nb_profiles, false);

	control_profiles_indices[0] = true;
	control_profiles_indices[nb_profiles-1] = true;

	bool done = false;
	double sumDist = 0.0;

	while( !done ){

        bool allAtTrue = true;
		for(int idx = 0; idx < control_profiles_indices.size(); idx++){
            if(!control_profiles_indices[idx]){
				allAtTrue = false;
				break;
			}
		}
		if(allAtTrue){
			break;
		}

		double max_dist = 0.0;
		int max_index = -1;
		for(int i = 0; i < nb_profiles; i++){

			if(!control_profiles_indices[i]){

				// min and max indices to reduce the number of points to generate mesh
				int minCtrlPrflIdx = 0;
				int maxCtrlPrflIdx = nb_profiles-1;
				int u = 0; int w = nb_profiles-1;
				while( u < (i-1)){
					if(control_profiles_indices[u] && !control_profiles_indices[u+1]){
						minCtrlPrflIdx = u;
					}
					u++;
				}
				while( w > (i+1)){
					if(control_profiles_indices[w] && !control_profiles_indices[w-1]){
						maxCtrlPrflIdx = w;
					}
					w--;
				}

				CGAL_Mesh approximated_shape = Utils::generateMesh(alignedProfilesSamples, control_profiles_indices, minCtrlPrflIdx, maxCtrlPrflIdx);
                Point_3 profile_centroid = alignedCentroids[i];
				Vec3d v_profile_centroid(profile_centroid);

                std::vector<Point_3> approxProfile = Utils::cross_section(n, v_profile_centroid, approximated_shape);
				if(approxProfile.size() == 0){
					break;
				}
				std::vector<Point_3> approxProfileSamples(Utils::sampleProfileCurve(approxProfile, nb_profile_samples));
                double dist = Utils::Haussdorf(alignedProfilesSamples[i], approxProfileSamples);
				if( dist > max_dist){

					max_dist = dist;
					max_index = i;
				}
			}
		}

        if((max_dist > 0.01) || (max_index != -1)){
			sumDist += max_dist;
			control_profiles_indices[max_index] = true;
		}else{
			done = true;
		}
	}

	// std::cout << "End of generateApproximatedProfileCurves" << std::endl;
	return sumDist;
	// return 0.0;
}

double GC::computeCylindricity(double  C, double alpha, double epsilon){

    cylindricity = straightness(C, epsilon) + alpha*profileVariation(epsilon);
	return cylindricity;
}


GC GC::merge(GC b){

	int n = axis.size() - 1;
	// Constructions of an Hermite Curve from end point of this to start point of b
	HermiteCurve unionOfAxis(axis[n].pe, b.axis[0].ps, axis[n].te, b.axis[0].ts);
	std::vector<HermiteCurve> middleAxis;
	middleAxis.push_back(unionOfAxis);
	GC middleGC(middleAxis, axis[n].pe, b.axis[0].ps, shape, false);

	// The new axis is the union of all the axis
	std::vector<HermiteCurve> newAxis(axis);
	newAxis.insert(newAxis.end(), middleGC.axis.begin(), middleGC.axis.end());
	newAxis.insert(newAxis.end(), b.axis.begin(), b.axis.end());

	GC mergedGC(newAxis, ps, b.pe, shape);
	for(int i = 0; i < mergedGC.axis.size()-1; i++){
		HermiteCurve hCurveA = mergedGC.axis[i];
		HermiteCurve hCurveB = mergedGC.axis[i+1];
		assert(hCurveA.pe == hCurveB.ps);
	}
	return mergedGC;
}

std::vector<Point_3> GC::getAllPoints(){

	std::vector<Point_3> allPoints;
	for(int i = 0; i < profiles.size(); i++){

		std::vector<Point_3> prfl = profiles[i];
		for(int j = 0; j < prfl.size(); j++){

			allPoints.push_back(prfl[j]);
		}
	}

	return allPoints;
}

GC::~GC(){

}
