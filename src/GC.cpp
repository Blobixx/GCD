#include "GC.h"
#include <cmath>
#include <utility>
#include <cassert>


void GC::DP(controlPoint_t controlPtA, controlPoint_t controlPtB, double& _straightness){

	double epsilon = 0.16;

	int axisSize = axis.size();
	// J'ai pas besoin de la dist en fait dans les controlPoint ???
	// controlPoint_t axisStart_t(0, 0.0);
	// controlPoint_t axisEnd_t(axisSize-1, 1.0);
	Vec3d lineStart = axis[controlPtA.axis_part].interpolate(controlPtA.parameter);
	Vec3d lineEnd = axis[controlPtB.axis_part].interpolate(controlPtB.parameter);
	Point_3 lineStart_3(lineStart[0], lineStart[1], lineStart[2]);
	Point_3 lineEnd_3(lineEnd[0], lineEnd[1], lineEnd[2]);


	int step = 50;
	double d_step = step;

	double maxDist = 0.0;
	int maxAxisPart = -1;
	double maxParam = -1.0;

	for(int part = 0; part < axisSize; part++){
		for(int t = 0; t < step; t++){

			double param = t/d_step;
            Vec3d curPt = axis[part].interpolate(param);
			Point_3 curPt_3(curPt[0], curPt[1], curPt[2]);
            double d = std::sqrt(CGAL::to_double(squared_distance(curPt_3, Line_3(lineStart_3, lineEnd_3))));

			if( d > maxDist){
				maxDist = d;
				maxAxisPart = part;
				maxParam = param;
			}
		}
	}

	if(maxDist > epsilon){

        _straightness += maxDist;
		controlPoint_t maxControlPt(maxAxisPart, maxParam, 0.0);
        DP(controlPtA, maxControlPt, _straightness);
        DP(maxControlPt, controlPtB, _straightness);
	}
	// return maxDist;
}
double GC::straightness(double C){

	double straightness = C;

    int axisSize = axis.size();
    controlPoint_t axisStart_t(0, 0.0, 0.0);
    controlPoint_t axisEnd_t(axisSize-1, 1.0, 0.0);

	DP(axisStart_t, axisEnd_t, straightness);

	return straightness;
}
/*double GC::straightness(double C){
	// std::cout << "Starting computing straightness ..." << std::endl;
	double straightness = C;
    double epsilon = 0.18;//0.16;

	// Contains all the control_points ordered by their parameter's value
	std::vector<controlPoint_t> list;
	// Adds ps to list. ps belongs to axis_part n°0
	controlPoint_t psControlPoint;
	list.push_back(psControlPoint);
	// Adds pe to list
	controlPoint_t peControlPoint(axis.size()-1, 1.0, 0.0);
	list.push_back(peControlPoint);
	bool done = false;

	while(!done){

		controlPoint_t maxControlPt;
		assert(maxControlPt.dist == 0);
		bool hasChanged = false;

		for(int index = 0; index < list.size()-1; index++){

			// Construction of the line between two consecutives control points
			controlPoint_t controlPtA = list[index];
			controlPoint_t controlPtB = list[index+1];

			// Get furthest point from line [controlPtA, controlPtB]
            if( !controlPtA.isSameAs(controlPtB)){
            	hasChanged = true;
				controlPoint_t controlPoint = findMaxDistToLine(controlPtA, controlPtB);
				if(controlPoint.dist > maxControlPt.dist){
					maxControlPt = controlPoint;
				}
			}

		}
		if(!hasChanged){
			break;
		}

		if(maxControlPt.dist > epsilon){

			// update straightness
			straightness += maxControlPt.dist;

			// add control point to list
			int idx_part = 0;
			int idx_param = 0;
			int listSize = list.size();
			while(idx_part < listSize && maxControlPt.axis_part > list[idx_part].axis_part){
				idx_part++;
			}
			while( (idx_part+idx_param < listSize) && maxControlPt.parameter > list[idx_part+idx_param].parameter){
				idx_param++;
			}

			assert(idx_part+idx_param < listSize);

			list.insert(list.begin() + idx_part+idx_param, maxControlPt);
		}else{
			done = true;
		}
	}

	// std::cout << "End of straightness." << std::endl;
	return straightness;
}*/

controlPoint_t GC::findMaxDistToLine(controlPoint_t controlPtA, controlPoint_t controlPtB){

	// Get control points coordinates
	Vec3d v_controlPtA = axis[controlPtA.axis_part].interpolate(controlPtA.parameter);
	Vec3d v_controlPtB = axis[controlPtB.axis_part].interpolate(controlPtB.parameter);
	// Conversion from Vec3d to Point_3
	Point_3 start_point(v_controlPtA[0], v_controlPtA[1], v_controlPtA[2]);
	Point_3 end_point(v_controlPtB[0], v_controlPtB[1], v_controlPtB[2]);

	Line_3 line(start_point, end_point);
	controlPoint_t maxControlPt;
	int step = 50;
	double d_step = step;

	// for(int j = 0; j < axis.size(); j++){
	for(int j = 0; j <= controlPtB.axis_part; j++){ // Je peux m'areter à l'axis part du B ?

		// Looking for points every 0.02
		for(int t = 1; t < step; t++){ // Start at 1 so that the control point itself is not considered

			double param = (double)t/d_step;

			// if point is a control point
			if( ( fabs(t/d_step - controlPtA.parameter) < 0.000001 && (j == controlPtA.axis_part)) ||
				( fabs(t/d_step - controlPtB.parameter) < 0.000001 && (j == controlPtB.axis_part))
			  ){

			  	continue;
			}
			// Consider point on axis curve
			Vec3d point = axis[j].interpolate(param);
			Point_3 point_3(point[0], point[1], point[2]);

			// Computes distance from point to line
			double d = std::sqrt(squared_distance(point_3, line));
			if( d > maxControlPt.dist){
				maxControlPt.dist = d;
				maxControlPt.parameter = param;
				maxControlPt.axis_part = j;
			}
		}
	}

	return maxControlPt;
}

double GC::debugProfileVariation(){

	// std::cout << "Starting computing profile variation ..." << std::endl;

	std::vector<Vec3d> originalCentroids; // centroids points on axis
	// Vector_vector_point_3 originalProfiles;
	std::vector<Vec3d> orthPlanesNormals; // Vector of normals of the plane perpendicular the axis at each sample points
	std::vector<Point_3> rotatedCentroids;
	// Vector_vector_point_3 rotatedProfiles;

	/* ---------- SAMPLING OF AXIS --------- */

	// Sample the axis with a dense set of points.
	// Computes profile curves as cross sections with the shape perpendicular to the axis at point
	int nbCentroidsPerPart = 3;
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
				originalProfiles.push_back(out);
				originalCentroids.push_back(s);
				orthPlanesNormals.push_back(orthPlaneAtS);
			}
		}
	}
	// Adds pe which is not added with the previous computation
	Vec3d tangentAtPe = axis[nbAxisParts-1].get_tangent(1.0);
	Vec3d orthPlaneAtPe = normalize(tangentAtPe); //The normal of the plane perpendicular to axis at p is the tangeant at p
	std::vector<Point_3> out_pe = Utils::cross_section(orthPlaneAtPe, pe, shape);
	if(out_pe.size() != 0){
		originalProfiles.push_back(out_pe);
		originalCentroids.push_back(pe);
		orthPlanesNormals.push_back(orthPlaneAtPe);
	}
	int nbCentroids = originalCentroids.size();

	/* ---------- END SAMPLING OF AXIS --------- */

	/* ---- DEBUG ---- */
	assert(nbCentroids != 0);
	assert(originalProfiles.size() == originalCentroids.size());
	for(int idx = 0; idx < originalProfiles.size(); idx ++){
		assert(originalProfiles[idx].size() != 0);
	}
	/* ---- END DEBUG ---- */

	/* ---------- TRANSFORMATION OF THE PROFILES SO THEY ARE PARALLEL --------- */
	// Profiles parallel to plane with normal vector(0,1,0)
	Vec3d n(0.0,1.0,0.0);
	for(int index = 0; index < originalProfiles.size(); index++){

		Vec3d centroid = originalCentroids[index];
		Point_3 centroid_3(centroid[0], centroid[1], centroid[2]); //Vec3d to Point_3

		Vec3d orth_plane_at_centroid = orthPlanesNormals[index];
		orth_plane_at_centroid.normalize();
		assert(fabs(orth_plane_at_centroid.length() - 1.0) < 0.00000000001); // Check if normalized
		double cos = dot(orth_plane_at_centroid, n);
 		Vec3d v = cross(orth_plane_at_centroid, n);
		double v1 = v[0];
		double v2 = v[1];
		double v3 = v[2];

		// cases cos = 1.0 && cos = -1.0
		if( std::fabs(cos + 1.0) < 0.00001 || std::fabs(cos - 1.0) < 0.00001){

			// Profile is already orthogonal to n
			// Add the profile itself in rotated_profiles and centroid
			rotatedProfiles.push_back(originalProfiles[index]);
			// rotatedCentroids.push_back(centroid_3);

		}else{

			double inv = 1.0/(1.0+cos);
			// Rotation to orient the plane so that its normal is n(0,1,0)
			// This rotatation consits of aligning the plane normal with the vector n
			Aff_transformation_3 rotation(1.0-(v2*v2+v3*v3)*inv, -1.0*v3+(v1*v2)*inv, v2+(v1*v3)*inv, 0,
											v3+(v1*v2)*inv, 1.0-(v1*v1+v3*v3)*inv, -1.0*v1+(v2*v3)*inv, 0,
											-1.0*v2+(v1*v3)*inv, v1+(v2*v3)*inv, 1.0-(v1*v1+v2*v2)*inv, 0, 1.0
										);
			// Apply rotation on centroid
			// Point_3 rotated_centroid = rotation.transform(centroid_3);
			// rotatedCentroids.push_back(rotated_centroid);

			// Apply rotation on the profile
			std::vector<Point_3> profilePts = originalProfiles[index];
			std::vector<Point_3> rotatedProfilePts;
			for(int pt = 0; pt < profilePts.size(); pt++){

				Point_3 tmp = rotation.transform(profilePts[pt]);
				rotatedProfilePts.push_back(tmp);
			}
			rotatedProfiles.push_back(rotatedProfilePts);
		}
	}
	/* ---------- END OF TRANSFORMATION OF THE PROFILES SO THEY ARE PARALLEL --------- */

	/* ---- DEBUG ---- */
	assert(originalProfiles.size() == rotatedProfiles.size());
	// assert(originalCentroids.size() == rotatedCentroids.size());
	for(int idx = 0; idx < rotatedProfiles.size(); idx++){
		assert(rotatedProfiles[idx].size() != 0);
	}
	/* ---- END DEBUG ---- */

	return 0.0;
}
double GC::profileVariation(){

	// std::cout << "Starting computing profile variation ..." << std::endl;

	std::vector<Vec3d> originalCentroids; // centroids points on axis
	// Vector_vector_point_3 originalProfiles;
	std::vector<Vec3d> orthPlanesNormals; // Vector of normals of the plane perpendicular the axis at each sample points
	std::vector<Point_3> rotatedCentroids;
	// Vector_vector_point_3 rotatedProfiles;

	/* ---------- SAMPLING OF AXIS --------- */

	// Sample the axis with a dense set of points.
	// Computes profile curves as cross sections with the shape perpendicular to the axis at point
	int nbCentroidsPerPart = 3;
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
				originalProfiles.push_back(out);
				originalCentroids.push_back(s);
				orthPlanesNormals.push_back(orthPlaneAtS);
			}
		}
	}
	// Adds pe which is not added with the previous computation
	Vec3d tangentAtPe = axis[nbAxisParts-1].get_tangent(1.0);
	Vec3d orthPlaneAtPe = normalize(tangentAtPe); //The normal of the plane perpendicular to axis at p is the tangeant at p
	std::vector<Point_3> out_pe = Utils::cross_section(orthPlaneAtPe, pe, shape);
	if(out_pe.size() != 0){
		originalProfiles.push_back(out_pe);
		originalCentroids.push_back(pe);
		orthPlanesNormals.push_back(orthPlaneAtPe);
	}
	int nbCentroids = originalCentroids.size();

	/* ---------- END SAMPLING OF AXIS --------- */

	/* ---- DEBUG ---- */
	assert(nbCentroids != 0);
	assert(originalProfiles.size() == originalCentroids.size());
	for(int idx = 0; idx < originalProfiles.size(); idx ++){
		assert(originalProfiles[idx].size() != 0);
	}
	/* ---- END DEBUG ---- */

	/* ---------- TRANSFORMATION OF THE PROFILES SO THEY ARE PARALLEL --------- */
	// Profiles parallel to plane with normal vector(0,1,0)
	Vec3d n(0.0,1.0,0.0);
	for(int index = 0; index < originalProfiles.size(); index++){

		Vec3d centroid = originalCentroids[index];
		Point_3 centroid_3(centroid[0], centroid[1], centroid[2]); //Vec3d to Point_3

		Vec3d orth_plane_at_centroid = orthPlanesNormals[index];
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
			rotatedProfiles.push_back(originalProfiles[index]);
			rotatedCentroids.push_back(centroid_3);

		}else{

			double inv = 1.0/(1.0+cos);
			// Rotation to orient the plane so that its normal is n(0,1,0)
			// This rotatation consits of aligning the plane normal with the vector n
			Aff_transformation_3 rotation(1.0-(v2*v2+v3*v3)*inv, -1.0*v3+(v1*v2)*inv, v2+(v1*v3)*inv, 0,
											v3+(v1*v2)*inv, 1.0-(v1*v1+v3*v3)*inv, -1.0*v1+(v2*v3)*inv, 0,
											-1.0*v2+(v1*v3)*inv, v1+(v2*v3)*inv, 1.0-(v1*v1+v2*v2)*inv, 0, 1.0
										);
			// Apply rotation on centroid
			Point_3 rotated_centroid = rotation.transform(centroid_3);
			rotatedCentroids.push_back(rotated_centroid);

			// Apply rotation on the profile
			std::vector<Point_3> profilePts = originalProfiles[index];
			std::vector<Point_3> rotatedProfilePts;
			for(int pt = 0; pt < profilePts.size(); pt++){

				Point_3 tmp = rotation.transform(profilePts[pt]);
				rotatedProfilePts.push_back(tmp);
			}
			rotatedProfiles.push_back(rotatedProfilePts);
		}
	}
	/* ---------- END OF TRANSFORMATION OF THE PROFILES SO THEY ARE PARALLEL --------- */

	/* ---- DEBUG ---- */
	assert(originalProfiles.size() == rotatedProfiles.size());
	assert(originalCentroids.size() == rotatedCentroids.size());
	for(int idx = 0; idx < rotatedProfiles.size(); idx++){
		assert(rotatedProfiles[idx].size() != 0);
	}
	/* ---- END DEBUG ---- */

	// Printing the dot product between n(0,1,0) and the vector (centroid, pointOnCurve)
	// Should be equal to 0
	// Vector_3 _n(0.0, 1.0, 0.0);
	// for(int idx = 0; idx < rotatedProfiles.size(); idx++){

	// 	std::vector<Point_3> profile = rotatedProfiles[idx];
	// 	Point_3 c = rotatedCentroids[idx];
	// 	for(int pt = 0; pt < profile.size(); pt++){

	// 		Point_3 p = profile[pt];
	// 		Vector_3 _v(p, c);
	// 		double test = CGAL::to_double(_v*_n);
	// 		assert( std::fabs(test) < 0.000001);
	// 		// std::cout << _v*_n << std::endl;
	// 	}
	// }

	/* ---------- TRANSFORMATION SO THAT CENTROIDS REST ON A STRAIGHT LINE --------- */

	Point_3 psPt = rotatedCentroids[0];
	Point_3 pePt = rotatedCentroids[nbCentroids-1];
	Line_3 line(psPt, pePt);
	alignedCentroids.push_back(rotatedCentroids[0]);
	alignedCentroids.push_back(rotatedCentroids[nbCentroids-1]);
	alignedProfiles.push_back(rotatedProfiles[0]);
	alignedProfiles.push_back(rotatedProfiles[nbCentroids-1]);

	// For each profile (except start and end), find the orthogonal projection
	// of the centroid on the line. Then translates the profile and its centroid
	for( int index = 1; index < rotatedCentroids.size()-1; index++){

		Point_3 rotatedCentroid = rotatedCentroids[index];
		// Orthogonal projection of centroid on the line [ps,pe]
		Point_3 centroidProjection = line.projection(rotatedCentroid);
		Vector_3 translationVector(rotatedCentroid, centroidProjection); // Vector_3(a, b) return b-a vector

		// Apply translation on centroid
		Aff_transformation_3 translation(CGAL::TRANSLATION, translationVector);
		Point_3 alignedCentroid = translation.transform(rotatedCentroid);
		// Stores aligned centroid
		alignedCentroids.push_back(alignedCentroid);

		// Apply translation on all the points of the profile
		std::vector<Point_3> rotatedProfilePts(rotatedProfiles[index]);
		std::vector<Point_3> alignedProfilePts;
		// Now apply transformations of the profile correspoding to centroid
		for(int pt = 0; pt < rotatedProfilePts.size(); pt++){

			Point_3 tmp = rotatedProfilePts[pt];
			Point_3 alignedPt = translation.transform(tmp);
			alignedProfilePts.push_back(alignedPt);
		}
		alignedProfiles.push_back(alignedProfilePts);
	}

	/* ---------- END OF TRANSFORMATION SO THAT CENTROIDS REST ON A STRAIGHT LINE --------- */
	// // Print the norm of the croos product betwen vector(ps, pe) and (ps, centroid)
	// // Should be 0
	// Point_3 _ps(ps[0], ps[1], ps[2]);
	// Point_3 _pe(pe[0], pe[1], pe[2]);
	// Vector_3 _v(_ps, _pe);
	// for(int idx = 0; idx < alignedProfiles.size(); idx++){

	// 	std::vector<Point_3> profile = alignedProfiles[idx];
	// 	for(int pt = 0; pt < alignedCentroids.size(); pt++){

	// 		Point_3 c = alignedCentroids[idx];
	// 		Vector_3 _w(_ps, c);
	// 		double norm = CGAL::to_double(CGAL::cross_product(_v, _w).squared_length());
	// 		std::cout << norm << std::endl;
	// 	}
	// }

	nb_profile_samples = 6;
	// Vector containing the sample points for each curve
	Vector_vector_point_3 profiles_samples;
	for(int i = 0; i < alignedProfiles.size(); i++){

		assert(alignedProfiles[i].size() != 0);
		std::vector<Point_3> s(Utils::sampleProfileCurve(alignedProfiles[i], nb_profile_samples));
		assert(s.size() != 0);
		profiles_samples.push_back(s);
	}

	double profileVariation = generateApproximatedProfileCurves(profiles_samples);
	std::vector<Point_3> cs_samples(Utils::sampleProfileCurve(alignedProfiles[0], nb_profile_samples));
	std::vector<Point_3> ce_samples(Utils::sampleProfileCurve(alignedProfiles[alignedProfiles.size()-1], nb_profile_samples));
	profileVariation += Utils::Haussdorf(cs_samples, ce_samples);

	// std::cout << "End of profileVariation" << std::endl;
	return profileVariation;
	// return 0.0;

}

double GC::generateApproximatedProfileCurves(Vector_vector_point_3 profiles_samples){

	// std::cout << "Starting generateApproximatedProfileCurves" << std::endl;
	double epsilon = 0.01;
	int nb_profiles = alignedProfiles.size();
	// std::cout << "There is " << nb_profiles << " aligned profiles" << std::endl;
	if(nb_profiles < 2){
		std::cout << "Weird, not enough profiles" << std::endl;
		return 0.0;
	}

	// std::vector<Point_3> control_profiles;
	// Vector to check if a profile has already been added to the approximated shape
	std::vector<bool> control_profiles_indices(nb_profiles, false);

	// Initialization of the approximated shape with start and end profiles
	std::vector<Point_3> cs_points = alignedProfiles[0];
	std::vector<Point_3> control_profiles(cs_points);
	// control_profiles.insert(control_profiles.end(), cs_points.begin(), cs_points.end());
	control_profiles_indices[0] = true;
	std::vector<Point_3> ce_points = alignedProfiles[nb_profiles-1];
	control_profiles.insert(control_profiles.end(), ce_points.begin(), ce_points.end()-1);
	control_profiles_indices[nb_profiles-1] = true;

	bool done = false;
	double sumDist = 0.0;

	while( !done ){

		CGAL_Mesh approximated_shape = Utils::generateMesh(control_profiles);

		double max_dist = 0.0;
		int max_index = -1;
		// std::cout << "step 1" << std::endl;
		for(int i = 1; i < nb_profiles-1; i++){

			if( !control_profiles_indices[i]){
				Point_3 profile_centroid = alignedCentroids[i];
				Vec3d v_profile_centroid(profile_centroid);

				// std::cout << "step 2" << std::endl;
				std::vector<Point_3> approxProfile = Utils::cross_section(Vec3d(0.0,1.0,0.0), v_profile_centroid, approximated_shape);
				// std::cout << "step 3" << std::endl;
				if(approxProfile.size() == 0){
					break;
				}
				std::vector<Point_3> approxProfileSamples(Utils::sampleProfileCurve(approxProfile, nb_profile_samples));
				// std::cout << "step 4" << std::endl;
				double dist = Utils::Haussdorf(profiles_samples[i], approxProfileSamples);
				// std::cout << "step 5" << std::endl;
				if( dist > max_dist){

					max_dist = dist;
					max_index = i;
				}
			}
		}

		if((max_dist > epsilon) && (max_index != -1)){
			// std::cout << "step 6" << std::endl;
			// std::cout << "max_dist: "<< max_dist <<std::endl;
			sumDist += max_dist;
			control_profiles_indices[max_index] = true;
			std::vector<Point_3> control_prfl_points(alignedProfiles[max_index]);
			// std::cout << "step 7" << std::endl;
			control_profiles.insert(control_profiles.end(), control_prfl_points.begin(),
													control_prfl_points.end())-1;
			approximated_shape.clear();
		}else{
			// std::cout << "step 8" << std::endl;
			done = true;
		}
	}

	// std::cout << "End of generateApproximatedProfileCurves" << std::endl;
	return sumDist;
	// return 0.0;
}

double GC::computeCylindricity(double  C, double alpha){

	// cylindricity = straightness(C) + alpha*profileVariation();
	cylindricity = alpha*debugProfileVariation();
	return cylindricity;
}


GC GC::merge(GC b){

	int n = axis.size() - 1;
	// Constructions of an Hermite Curve from end point of this to start point of b
	HermiteCurve unionOfaxis(axis[n].pe, axis[n].te, b.axis[0].ps, b.axis[0].ts);
	std::vector<HermiteCurve> newAxis(axis);
	// The new axis is the union of both axis + the union
	// newAxis.insert(newAxis.begin(), axis.begin(), axis.end());
	newAxis.push_back(unionOfaxis);
	newAxis.insert(newAxis.end(), b.axis.begin(), b.axis.end()-1);
	GC mergedGC(newAxis, ps, b.pe);

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
