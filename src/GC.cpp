#include "GC.h"
#include <cmath>
#include <utility>
#include <cassert>


void GC::DP(controlPoint controlPtA, controlPoint controlPtB, double& _straightness, int step, double epsilon){

	if(controlPtA.axis_part == controlPtB.axis_part && controlPtA.dist == controlPtB.dist &&
		controlPtA.parameter == controlPtB.parameter){
        return;// _straightness;
	}
    // double epsilon = 0.1;

	int axisSize = axis.size();

    Vec3d lineStart = axis[controlPtA.axis_part].interpolate((double)controlPtA.parameter/step);
    Vec3d lineEnd = axis[controlPtB.axis_part].interpolate((double)controlPtB.parameter/step);

	double d_step = step;

	double maxDist = 0.0;
    int maxAxisPart;
//	double maxParam = -1.0;
    int maxParam = -1;

	// check on controlPtA axis part
	for(int t = (controlPtA.parameter+1); t <= step; t++){

        double param = (double) t/d_step;
        Vec3d curPt = axis[controlPtA.axis_part].interpolate(param);
		// Point_3 curPt_3(curPt[0], curPt[1], curPt[2]);
        // double d = std::sqrt(CGAL::to_double(squared_distance(curPt_3, Line_3(lineStart_3, lineEnd_3))));
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
		// Point_3 curPt_3(curPt[0], curPt[1], curPt[2]);
        // double d = std::sqrt(CGAL::to_double(squared_distance(curPt_3, Line_3(lineStart_3, lineEnd_3))));
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
			// Point_3 curPt_3(curPt[0], curPt[1], curPt[2]);
            // double d = std::sqrt(CGAL::to_double(squared_distance(curPt_3, Line_3(lineStart_3, lineEnd_3))));
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
//        controlPoint_t maxControlPt(_part, maxParam, 0.0);
        controlPoint maxControlPt(maxAxisPart, maxParam, 0.0);
        DP(controlPtA, maxControlPt, _straightness, step, epsilon);
        DP(maxControlPt, controlPtB, _straightness, step, epsilon);
//        DP(controlPtA, maxControlPt, _straightness, epsilon);
//        DP(maxControlPt, controlPtB, _straightness, epsilon);
	}
}

double GC::straightness(double C, double epsilon){

	double straightness = C;
	int step = 50;
    int axisSize = axis.size();
    // controlPoint_t axisStart_t(0, 0.0, 0.0);
    // controlPoint_t axisEnd_t(axisSize-1, 1.0, 0.0);

    controlPoint axisStart_t(0, 0, 0.0);
    controlPoint axisEnd_t(axisSize-1, step, 0.0);



	DP(axisStart_t, axisEnd_t, straightness, step, epsilon);
	checkStraightness = straightness;
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
// controlPoint_t GC::findMaxDistToLine(controlPoint_t controlPtA, controlPoint_t controlPtB){

// 	// Get control points coordinates
// 	Vec3d v_controlPtA = axis[controlPtA.axis_part].interpolate(controlPtA.parameter);
// 	Vec3d v_controlPtB = axis[controlPtB.axis_part].interpolate(controlPtB.parameter);
// 	// Conversion from Vec3d to Point_3
// 	Point_3 start_point(v_controlPtA[0], v_controlPtA[1], v_controlPtA[2]);
// 	Point_3 end_point(v_controlPtB[0], v_controlPtB[1], v_controlPtB[2]);

// 	Line_3 line(start_point, end_point);
// 	controlPoint_t maxControlPt;
// 	int step = 50;
// 	double d_step = step;

// 	// for(int j = 0; j < axis.size(); j++){
// 	for(int j = 0; j <= controlPtB.axis_part; j++){ // Je peux m'areter à l'axis part du B ?

// 		// Looking for points every 0.02
// 		for(int t = 1; t < step; t++){ // Start at 1 so that the control point itself is not considered

// 			double param = (double)t/d_step;

// 			// if point is a control point
// 			if( ( fabs(t/d_step - controlPtA.parameter) < 0.000001 && (j == controlPtA.axis_part)) ||
// 				( fabs(t/d_step - controlPtB.parameter) < 0.000001 && (j == controlPtB.axis_part))
// 			  ){

// 			  	continue;
// 			}
// 			// Consider point on axis curve
// 			Vec3d point = axis[j].interpolate(param);
// 			Point_3 point_3(point[0], point[1], point[2]);

// 			// Computes distance from point to line
// 			double d = std::sqrt(squared_distance(point_3, line));
// 			if( d > maxControlPt.dist){
// 				maxControlPt.dist = d;
// 				maxControlPt.parameter = param;
// 				maxControlPt.axis_part = j;
// 			}
// 		}
// 	}

// 	return maxControlPt;
// }

// /*double GC::profileVariation(double epsilon){

// 	// std::cout << "Starting computing profile variation ..." << std::endl;

// 	// std::vector<Vec3d> originalCentroids; // centroids points on axis
// 	// Vector_vector_point_3 originalProfiles;
// 	// std::vector<Vec3d> orthPlanesNormals; // Vector of normals of the plane perpendicular the axis at each sample points
// 	// std::vector<Point_3> rotatedCentroids;
// 	// Vector_vector_point_3 rotatedProfiles;

// 	/* ---------- TRANSFORMATION OF THE PROFILES SO THEY ARE PARALLEL --------- */
// 	// Profiles parallel to plane with normal vector(0,1,0)
// 	Vec3d n(0.0,1.0,0.0);
// 	for(int index = 0; index < profiles.size(); index++){

// 		Vec3d centroid = profilesCentroids[index];
// 		Point_3 centroid_3(centroid[0], centroid[1], centroid[2]); //Vec3d to Point_3

// 		Vec3d orth_plane_at_centroid = profilesNormals[index];
// 		orth_plane_at_centroid.normalize();
// 		double cos = dot(orth_plane_at_centroid, n);
// 		Vec3d v = cross(orth_plane_at_centroid, n);
// 		double v1 = v[0];
// 		double v2 = v[1];
// 		double v3 = v[2];

// 		// cases cos = 1.0 && cos = -1.0
// 		if( std::fabs(cos + 1.0) < 0.00001 || std::fabs(cos - 1.0) < 0.00001){

// 			// Profile is already orthogonal to n
// 			// Add the profile itself in rotated_profiles and centroid
// 			rotatedProfiles.push_back(profiles[index]);
// 			// rotatedCentroids.push_back(centroid_3);

// 		}else{

// 			double inv = 1.0/(1.0+cos);
// 			// Rotation to orient the plane so that its normal is n(0,1,0)
// 			// This rotatation consits of aligning the plane normal with the vector n
// 			Aff_transformation_3 rotation(1.0-(v2*v2+v3*v3)*inv, -1.0*v3+(v1*v2)*inv, v2+(v1*v3)*inv, 0,
// 											v3+(v1*v2)*inv, 1.0-(v1*v1+v3*v3)*inv, -1.0*v1+(v2*v3)*inv, 0,
// 											-1.0*v2+(v1*v3)*inv, v1+(v2*v3)*inv, 1.0-(v1*v1+v2*v2)*inv, 0, 1.0
// 										);
// 			// Apply rotation on centroid
// 			// Point_3 rotated_centroid = rotation.transform(centroid_3);
// 			// rotatedCentroids.push_back(rotated_centroid);

// 			// Apply rotation on the profile
//             std::vector<Point_3> profilePts = profiles[index];
// 			std::vector<Point_3> rotatedProfilePts;
// 			for(int pt = 0; pt < profilePts.size(); pt++){

// 				Point_3 tmp = rotation.transform(profilePts[pt]);
// 				rotatedProfilePts.push_back(tmp);
// 			}
// 			rotatedProfiles.push_back(rotatedProfilePts);
// 		}
// 	}
// 	/* ---------- END OF TRANSFORMATION OF THE PROFILES SO THEY ARE PARALLEL --------- */

// 	/* ---- DEBUG ---- */
//     assert(profiles.size() == rotatedProfiles.size());
// 	// assert(originalCentroids.size() == rotatedCentroids.size());
// 	for(int idx = 0; idx < rotatedProfiles.size(); idx++){
// 		assert(rotatedProfiles[idx].size() != 0);
// 	}
// 	/* ---- END DEBUG ---- */

// 	// Printing the dot product between n(0,1,0) and the vector (centroid, pointOnCurve)
// 	// Should be equal to 0
// 	// Vector_3 _n(0.0, 1.0, 0.0);
// 	// for(int idx = 0; idx < rotatedProfiles.size(); idx++){

// 	// 	std::vector<Point_3> profile = rotatedProfiles[idx];
// 	// 	Point_3 c = rotatedCentroids[idx];
// 	// 	for(int pt = 0; pt < profile.size(); pt++){

// 	// 		Point_3 p = profile[pt];
// 	// 		Vector_3 _v(p, c);
// 	// 		double test = CGAL::to_double(_v*_n);
// 	// 		assert( std::fabs(test) < 0.000001);
// 	// 		// std::cout << _v*_n << std::endl;
// 	// 	}
// 	// }

// 	 ---------- TRANSFORMATION SO THAT CENTROIDS REST ON A STRAIGHT LINE --------- 

// 	int nbCentroids = profilesCentroids.size();
// 	Point_3 psPt = profilesCentroids[0];
// 	Point_3 pePt = profilesCentroids[nbCentroids-1];
// 	Line_3 line(psPt, pePt);
// 	alignedCentroids.push_back(profilesCentroids[0]);
// 	alignedProfiles.push_back(rotatedProfiles[0]);

// 	// For each profile (except start and end), find the orthogonal projection
// 	// of the centroid on the line. Then translates the profile and its centroid
// 	for( int index = 1; index < profilesCentroids.size()-1; index++){

// 		Point_3 centroid = profilesCentroids[index];
// 		// Orthogonal projection of centroid on the line [ps,pe]
// 		Point_3 centroidProjection = line.projection(centroid);
// 		Vector_3 translationVector(centroid, centroidProjection); // Vector_3(a, b) return b-a vector

// 		// Apply translation on centroid
// 		Aff_transformation_3 translation(CGAL::TRANSLATION, translationVector);
// 		Point_3 alignedCentroid = translation.transform(centroid);
// 		// Stores aligned centroid
// 		alignedCentroids.push_back(alignedCentroid);

// 		// Apply translation on all the points of the profile
// 		std::vector<Point_3> rotatedProfilePts(rotatedProfiles[index]);
// 		std::vector<Point_3> alignedProfilePts;
// 		// Now apply transformations of the profile correspoding to centroid
// 		for(int pt = 0; pt < rotatedProfilePts.size(); pt++){

// 			Point_3 tmp = rotatedProfilePts[pt];
// 			Point_3 alignedPt = translation.transform(tmp);
// 			alignedProfilePts.push_back(alignedPt);
// 		}
// 		alignedProfiles.push_back(alignedProfilePts);
// 	}

// 	alignedCentroids.push_back(profilesCentroids[nbCentroids-1]);
// 	alignedProfiles.push_back(rotatedProfiles[nbCentroids-1]);

// 	/* ---------- END OF TRANSFORMATION SO THAT CENTROIDS REST ON A STRAIGHT LINE --------- */
// 	// // Print the norm of the croos product betwen vector(ps, pe) and (ps, centroid)
// 	// // Should be 0
// 	// Point_3 _ps(ps[0], ps[1], ps[2]);
// 	// Point_3 _pe(pe[0], pe[1], pe[2]);
// 	// Vector_3 _v(_ps, _pe);
// 	// for(int idx = 0; idx < alignedProfiles.size(); idx++){

// 	// 	std::vector<Point_3> profile = alignedProfiles[idx];
// 	// 	for(int pt = 0; pt < alignedCentroids.size(); pt++){

// 	// 		Point_3 c = alignedCentroids[idx];
// 	// 		Vector_3 _w(_ps, c);
// 	// 		double norm = CGAL::to_double(CGAL::cross_product(_v, _w).squared_length());
// 	// 		std::cout << norm << std::endl;
// 	// 	}
// 	// }

// 	nb_profile_samples = 6;
// 	// Vector containing the sample points for each curve
// 	alignedProfilesSamples.resize(alignedProfiles.size());
// 	for(int i = 0; i < alignedProfiles.size(); i++){

// 		assert(alignedProfiles[i].size() != 0);
// 		alignedProfilesSamples[i] = Utils::sampleProfileCurve(alignedProfiles[i], nb_profile_samples);
// 		assert(alignedProfilesSamples[i].size() != 0);
// 	}

// 	double profileVariation = generateApproximatedProfileCurves(alignedProfilesSamples, epsilon);
// 	std::vector<Point_3> csSamples(Utils::sampleProfileCurve(alignedProfiles[0], nb_profile_samples));
// 	std::vector<Point_3> ceSamples(Utils::sampleProfileCurve(alignedProfiles[alignedProfiles.size()-1], nb_profile_samples));
// 	profileVariation += Utils::Haussdorf(csSamples, ceSamples);

// 	// std::cout << "End of profileVariation" << std::endl;
// 	return profileVariation;
// 	// return 0.0;

// }*/
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
	// Profiles parallel to plane with normal vector(0,1,0)
	// Vec3d n(0.0,1.0,0.0);
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

	// Printing the dot product between n(0,1,0) and the vector (centroid, pointOnCurve)
	// Should be equal to 0
	// Vector_3 _n(n[0], n[1], n[2]);
	// for(int idx = 0; idx < alignedAndParallelProfiles.size(); idx++){

	// 	std::vector<Point_3> profile = alignedAndParallelProfiles[idx];
	// 	Point_3 c = alignedCentroids[idx];
	// 	for(int pt = 0; pt < profile.size(); pt++){

	// 		Point_3 p = profile[pt];
	// 		Vector_3 _v(p, c);
	// 		double test = CGAL::to_double(_v*_n);
	// 		assert( std::fabs(test) < 0.000001);
	// 		// std::cout << _v*_n << std::endl;
	// 	}
	// }

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

	nb_profile_samples = 5;
	// Vector containing the sample points for each curve
	alignedProfilesSamples.resize(alignedAndParallelProfiles.size());
	for(int i = 0; i < alignedAndParallelProfiles.size(); i++){

		assert(alignedAndParallelProfiles[i].size() != 0);
		alignedProfilesSamples[i] = Utils::sampleProfileCurve(alignedAndParallelProfiles[i], nb_profile_samples);
		assert(alignedProfilesSamples[i].size() != 0);
	}

	double profileVariation = generateApproximatedProfileCurves(alignedProfilesSamples, epsilon, n);
	// std::vector<Point_3> csSamples(Utils::sampleProfileCurve(alignedProfiles[0], nb_profile_samples));
	// std::vector<Point_3> ceSamples(Utils::sampleProfileCurve(alignedProfiles[alignedProfiles.size()-1], nb_profile_samples));
	profileVariation += Utils::Haussdorf(alignedProfilesSamples[0], alignedProfilesSamples[nbCentroids-1]);

	// std::cout << "End of profileVariation" << std::endl;
	return profileVariation;
	// return 0.0;

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
		// CGAL_Mesh approximated_shape = Utils::generateMesh(alignedProfilesSamples, control_profiles_indices);

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
// double GC::generateApproximatedProfileCurves(Vector_vector_point_3 profiles_samples, double epsilon){

// 	// std::cout << "Starting generateApproximatedProfileCurves" << std::endl;
// 	// double epsilon = 0.01;
// 	int nb_profiles = alignedProfiles.size();
// 	// std::cout << "There is " << nb_profiles << " aligned profiles" << std::endl;
// 	if(nb_profiles < 2){
// 		std::cout << "Weird, not enough profiles" << std::endl;
// 		return 0.0;
// 	}

// 	// std::vector<Point_3> control_profiles;
// 	// Vector to check if a profile has already been added to the approximated shape
// 	std::vector<bool> control_profiles_indices(nb_profiles, false);

// 	// Initialization of the approximated shape with start and end profiles
// //	std::vector<Point_3> cs_points = alignedProfiles[0];
// //	std::vector<Point_3> control_profiles(cs_points);
// 	// control_profiles.insert(control_profiles.end(), cs_points.begin(), cs_points.end());
// 	control_profiles_indices[0] = true;
// //	std::vector<Point_3> ce_points = alignedProfiles[nb_profiles-1];
// //	control_profiles.insert(control_profiles.end(), ce_points.begin(), ce_points.end());
// 	control_profiles_indices[nb_profiles-1] = true;

// 	bool done = false;
// 	double sumDist = 0.0;

// 	while( !done ){

// 		int allAtTrue = true;
// 		for(int idx = 0; idx < control_profiles_indices.size(); idx++){
//             if(!control_profiles_indices[idx]){
// 				allAtTrue = false;
// 				break;
// 			}
// 		}
// 		if(allAtTrue){
// 			break;
// 		}
// 		// CGAL_Mesh approximated_shape = Utils::generateMesh(alignedProfilesSamples, control_profiles_indices);

// 		double max_dist = 0.0;
// 		int max_index = -1;
// 		// std::cout << "step 1" << std::endl;
// 		for(int i = 0; i < nb_profiles; i++){

// 			if( !control_profiles_indices[i]){

// 				int minCtrlPrflIdx = 0;
// 				int maxCtrlPrflIdx = nb_profiles-1;
// 				int u = 0; int w = nb_profiles-1;
// 				while( u < (i-1)){
// 					if(control_profiles_indices[u] && !control_profiles_indices[u+1]){
// 						minCtrlPrflIdx = u;
// 					}
// 					u++;
// 				}
// 				while( w > (i+1)){
// 					if(control_profiles_indices[w] && !control_profiles_indices[w-1]){
// 						maxCtrlPrflIdx = w;
// 					}
// 					w--;
// 				}

// 				CGAL_Mesh approximated_shape = Utils::generateMesh(alignedProfilesSamples, control_profiles_indices, minCtrlPrflIdx, maxCtrlPrflIdx);
//                 Point_3 profile_centroid = alignedCentroids[i];
// 				Vec3d v_profile_centroid(profile_centroid);

// 				// std::cout << "step 2" << std::endl;
// 				std::vector<Point_3> approxProfile = Utils::cross_section(Vec3d(0.0,1.0,0.0), v_profile_centroid, approximated_shape);
// 				// std::cout << "step 3" << std::endl;
// 				if(approxProfile.size() == 0){
// 					break;
// 				}
// 				std::vector<Point_3> approxProfileSamples(Utils::sampleProfileCurve(approxProfile, nb_profile_samples));
// 				// std::cout << "step 4" << std::endl;
//                 double dist = Utils::Haussdorf(alignedProfilesSamples[i], approxProfileSamples);
// 				// std::cout << "step 5" << std::endl;
// 				if( dist > max_dist){

// 					max_dist = dist;
// 					max_index = i;
// 				}
// 			}
// 		}

//         if((max_dist > 0.01) || (max_index != -1)){
// 			// std::cout << "step 6" << std::endl;
// 			// std::cout << "max_dist: "<< max_dist <<std::endl;
// 			sumDist += max_dist;
// 			control_profiles_indices[max_index] = true;
// //			std::vector<Point_3> control_prfl_points(alignedProfiles[max_index]);
// 			// std::cout << "step 7" << std::endl;
// //			control_profiles.insert(control_profiles.end(), control_prfl_points.begin(),
// //													control_prfl_points.end());
// //			approximated_shape.clear();
// 		}else{
// 			// std::cout << "step 8" << std::endl;
// 			done = true;
// 		}
// 	}

// 	// std::cout << "End of generateApproximatedProfileCurves" << std::endl;
// 	return sumDist;
// 	// return 0.0;
// }

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
	GC middleGC(middleAxis, axis[n].pe, b.axis[0].ps, false);
    // middleGC.profileVariation();

	// The new axis is the union of all the axis
	std::vector<HermiteCurve> newAxis(axis);
	newAxis.insert(newAxis.end(), middleGC.axis.begin(), middleGC.axis.end());
	newAxis.insert(newAxis.end(), b.axis.begin(), b.axis.end());

	GC mergedGC(newAxis, ps, b.pe);
	for(int i = 0; i < mergedGC.axis.size()-1; i++){
		HermiteCurve hCurveA = mergedGC.axis[i];
		HermiteCurve hCurveB = mergedGC.axis[i+1];
		assert(hCurveA.pe == hCurveB.ps);
	}

	/*// Filling up mergedGC profiles, centroids normals, samples with those of gc A, middleGC and gc B
	int sizeA = profiles.size();
	int sizeMiddle = middleGC.profiles.size();
	int sizeB = b.profiles.size();
	int sizeMerge = sizeA + sizeMiddle + sizeB;

	// Vector_vector_point_3 tmpProfiles(sizeMerge);
	mergedGC.profiles.resize(sizeMerge);
	for(int i = 0; i < sizeA; i++){
		assert(mergedGC.profiles[i].size() == 0);
		std::vector<Point_3> points = profiles[i];
		for(int j = 0; j < points.size(); j++){
			mergedGC.profiles[i].push_back(points[j]);
		}
	}
	for(int i = 0; i < sizeMiddle; i++){
		std::vector<Point_3> points = middleGC.profiles[i];
		for(int j = 0; j < points.size(); j++){
			mergedGC.profiles[i+sizeA].push_back(points[j]);
		}
	}
	for(int i = 0; i < sizeB; i++){
		std::vector<Point_3> points = b.profiles[i];
		for(int j = 0; j < points.size(); j++){
			mergedGC.profiles[i+sizeA+sizeMiddle].push_back(points[j]);
		}
	}
	assert(mergedGC.profilesCentroids.size() == 0);
	// std::vector<Point_3> tmpProfilesCentroids;
	for(int i = 0; i < sizeA; i++){
		mergedGC.profilesCentroids.push_back(profilesCentroids[i]);
	}
	for(int i = 0; i < sizeMiddle; i++){
		mergedGC.profilesCentroids.push_back(middleGC.profilesCentroids[i]);
	}
	for(int i = 0; i < sizeB; i++){
		mergedGC.profilesCentroids.push_back(b.profilesCentroids[i]);
	}
	// mergedGC.profilesCentroids.insert(mergedGC.profilesCentroids.end(), profilesCentroids.begin(), profilesCentroids.end() );
	// mergedGC.profilesCentroids.insert(mergedGC.profilesCentroids.end(), middleGC.profilesCentroids.begin(), middleGC.profilesCentroids.end() );
	// mergedGC.profilesCentroids.insert(mergedGC.profilesCentroids.end(), b.profilesCentroids.begin(), b.profilesCentroids.end() );
	// mergedGC.profilesCentroids = tmpProfilesCentroids;

	// Vector_vector_point_3 tmpAlignedProfilesSamples; //(sizeMerge);
	// mergedGC.alignedProfilesSamples.resize(sizeMerge);
	// for(int i = 0; i < sizeA; i++){
	// 	assert(mergedGC.alignedProfilesSamples[i].size() == 0);
	// 	std::vector<Point_3> samples = alignedProfilesSamples[i];
	// 	for(int j = 0; j < samples.size(); j++){
	// 		mergedGC.alignedProfilesSamples[i].push_back(samples[j]);
	// 	}
	// }
 //    for(int i = 0; i < sizeMiddle; i++){
	// 	std::vector<Point_3> samples = middleGC.alignedProfilesSamples[i];
	// 	for(int j = 0; j < samples.size(); j++){
 //            mergedGC.alignedProfilesSamples[i+sizeA].push_back(samples[j]);
	// 	}
	// }
 //    for(int i = 0; i < sizeB; i++){
	// 	std::vector<Point_3> samples = b.alignedProfilesSamples[i];
	// 	for(int j = 0; j < samples.size(); j++){
	// 		mergedGC.alignedProfilesSamples[i+sizeA+sizeMiddle].push_back(samples[j]);
	// 	}
	// }

	assert(mergedGC.profilesNormals.size() == 0);
	for(int i = 0; i < sizeA; i++){
		mergedGC.profilesNormals.push_back(profilesNormals[i]);
	}
	for(int i = 0; i < sizeMiddle; i++){
		mergedGC.profilesNormals.push_back(middleGC.profilesNormals[i]);
	}
	for(int i = 0; i < sizeB; i++){
		mergedGC.profilesNormals.push_back(b.profilesNormals[i]);
	}
	// std::vector<Vec3d> tmpProfilesNormals;
	// mergedGC.profilesNormals.insert(mergedGC.profilesNormals.end(), middleGC.profilesNormals.begin(), middleGC.profilesNormals.end() );
	// mergedGC.profilesNormals.insert(mergedGC.profilesNormals.end(), b.profilesNormals.begin(), b.profilesNormals.end() );
	// mergedGC.profilesNormals = tmpProfilesNormals;*/


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
