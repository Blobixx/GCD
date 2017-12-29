#include "GC.h"
#include <cmath>
#include <utility>


float GC::straightness(float C){
	std::cout << "Starting computing straightness ..." << std::endl;
	float straightness = C;
	float epsilon = 0.0000003;

	// Conversion from Vec3f to  CGAL Point_3
	// Useful to be able to use the CGAL function dist to line.
	Point_3 _ps(ps[0], ps[1], ps[2]);
	Point_3 _pe(pe[0], pe[1], pe[2]);

	// Constains all the control_points ordered by their parameter's value
	std::vector<controlPoint_t> list;
	// Adds ps to list. ps belongs to axis_part n°0
	controlPoint_t psControlPoint;
	psControlPoint.dist = 0.0f;
	psControlPoint.parameter = 0.0f;
	psControlPoint.axis_part = 0;
	list.push_back(psControlPoint);
	// Adds pe to list. ps belongs to axis_part n°0
	controlPoint_t peControlPoint;
	peControlPoint.dist = 0.0f;
	peControlPoint.parameter = 1.0f;
	peControlPoint.axis_part = axis.size()-1;
	list.push_back(peControlPoint);
	// TODO: add a Point_3 to the controlPoint_t struct so I dont
	// have to recompute it each time
	bool done = false;
	while( !done ){

		controlPoint_t maxControlPt;
		maxControlPt.dist = 0;

		for(int index = 0; index < list.size()-1; index++){
			controlPoint_t controlPtA = list[index];
			controlPoint_t controlPtB = list[index+1];
			Vec3f v_controlPtA = axis[controlPtA.axis_part].interpolate(controlPtA.parameter);
			Vec3f v_controlPtB = axis[controlPtB.axis_part].interpolate(controlPtB.parameter);
			Point_3 p_controlPtA((double)v_controlPtA[0], (double)v_controlPtA[1], (double)v_controlPtA[2]);
			Point_3 p_controlPtB((double)v_controlPtB[0], (double)v_controlPtB[1], (double)v_controlPtB[2]);

			controlPoint_t controlPoint = findMaxDistToLine(p_controlPtA, p_controlPtB);
			if(controlPoint.dist > maxControlPt.dist){
				maxControlPt.dist = controlPoint.dist;
				maxControlPt.parameter = controlPoint.parameter;
				maxControlPt.axis_part = controlPoint.axis_part;
			}

		}
		// std::cout <<"dist = " << control_point.dist << std::endl;
		if(maxControlPt.dist > epsilon){
			for(int i = 0; i < list.size(); i++){

				// Insertion of the control in the list at the right place
				if( maxControlPt.axis_part == list[i].axis_part){
					if(maxControlPt.parameter < list[i].parameter){
						list.insert(list.begin() + i, maxControlPt);
						break;
					}
				}
			}
		}else{
			done = true;
		}
	}

	for(int i = 0; i < list.size(); i++){
		straightness += list[i].dist;
	}

	std::cout << "End of straightness." << std::endl;
	return straightness;
}

// TODO: Rename this to findMaxDistToAxisLine
controlPoint_t GC::findMaxDistToLine(Point_3 start_point, Point_3 end_point){

	Line_3 line(start_point, end_point);
	float max_dist = 0.f;
	float max_value = 0.f; //control point paramater
	int max_axis_part = 0;

	for(int j = 0; j < axis.size(); j++){
		//Increasing step is 0.02
		for(int t = 1; t < 50; t++){

			// Consider point on axis curve
			Vec3f point = axis[j].interpolate(t/50.0f);
			Point_3 _point(point[0], point[1], point[2]);

			//Computes distance from point to line
			float dist = std::sqrt(squared_distance(_point, line));
			if( dist > max_dist){
				max_dist = dist;
				max_value = t/50.0f;
				max_axis_part = j;
			}
		}
	}

	controlPoint_t control_point;
	control_point.parameter = max_value;
	control_point.dist = max_dist;
	control_point.axis_part = max_axis_part;
	// std::cout <<"control point: " << max_value <<", "<<max_dist <<std::endl;

	return control_point;
}

float GC::profileVariation(){

	std::cout << "Starting computing profile variation ..." << std::endl;

	std::vector<Vec3f> original_centroids; // centroids points on axis
	int nbCentroidsPerPart = 3;

	Vector_vector_point_3 original_profiles;
	// vector of normals of the plane perpendicular the axis at each sample points
	std::vector<Vec3f> orth_planes;
	std::vector<Point_3> rotated_centroids;
	Vector_vector_point_3 rotated_profiles;
	// Sample the axis with a dense set of points.
	// Computes profile curves as cross sections with the shape perpendicular to the axis at point
	// TODO: what follows is wrong. Gives more centroids than wanted !!
	int nbAxisParts = axis.size();
	int nbCentroids = nbCentroidsPerPart*nbAxisParts;
	for(int j = 0; j < nbAxisParts; j++){

		HermiteCurve hermite_curve = axis[j];
		for(int i = 0; i < nbCentroidsPerPart; i++){

			float t  = (float)i/nbCentroidsPerPart;
			Vec3f s = hermite_curve.interpolate(t);
			original_centroids.push_back(s);
			Vec3f tangent_at_s = hermite_curve.get_tangent(t);
			Vec3f orth_plane_at_s = normalize(tangent_at_s); //The normal of the plane perpendicular to axis at p is the tangeant at p
			original_profiles.push_back(Utils::cross_section(orth_plane_at_s, s, shape));
			orth_planes.push_back(orth_plane_at_s);
		}
	}
	// Adds pe which is not added with the previous computation
	original_centroids.push_back(pe);
	Vec3f tangent_at_pe = axis[nbAxisParts-1].get_tangent(1.0f);
	Vec3f orth_plane_at_pe = normalize(tangent_at_pe); //The normal of the plane perpendicular to axis at p is the tangeant at p
	original_profiles.push_back(Utils::cross_section(orth_plane_at_pe, pe, shape));
	orth_planes.push_back(orth_plane_at_pe);
	nbCentroids++; // +1 because pe has been added

	// Align profile curves so they reside on parallel planes (normal_vector(0,1,0))
	Vec3f n(0,1,0);
	for(int index = 0; index < original_profiles.size(); index++){

		Vec3f centroid = original_centroids[index];
		Point_3 centroid_3(centroid[0], centroid[1], centroid[2]); //Vec3f to Point_3

		Vec3f orth_plane_at_centroid = orth_planes[index];
		float cos = dot(orth_plane_at_centroid, n);
		Vec3f v = cross(orth_plane_at_centroid, n);
		float v1 = v[0];
		float v2 = v[1];
		float v3 = v[2];

		if( cos != -1.0f){

			// Rotation to orient the plane so that its normal is n(0,1,0)
			// This rotatation consits of aligning the plane normal with the vector n
			Aff_transformation_3 rotation(1.f-(v2*v2+v3*v3)/(1.f+cos), -1.f*v3+(v1*v2)/(1.f+cos), v2+(v1*v3)/(1.f+cos),
										v3+(v1*v2)/(1.f+cos), 1.f-(v1*v1+v3*v3)/(1.f+cos), -1.f*v1+(v2*v3)/(1.f+cos),
										-1.f*v2+(v1*v3)/(1.f+cos), v1+(v2*v3)/(1.f+cos), 1.f-(v1*v1+v2*v2)/(1.f+cos)
									);
			// Apply rotation on centroid
			Point_3 rotated_centroid = rotation.transform(centroid_3);
			rotated_centroids.push_back(rotated_centroid);

			// Apply rotatin on the profile
			std::vector<Point_3> profilePts = original_profiles[index];
			std::vector<Point_3> rotatedProfilePts;
			for(int pt = 0; pt < profilePts.size(); pt++){

				Point_3 tmp = rotation.transform(profilePts[pt]);
				rotatedProfilePts.push_back(tmp);
			}
			rotated_profiles.push_back(rotatedProfilePts);
		}else{

			// TODO: case cos = -1
			std::cout << "ici" << std::endl;
		}
	}

	Point_3 psPt = rotated_centroids[0];
	Point_3 pePt = rotated_centroids[nbCentroids-1];
	Line_3 line(psPt, pePt);
	aligned_centroids.push_back(rotated_centroids[0]);
	aligned_centroids.push_back(rotated_centroids[nbCentroids-1]);
	aligned_profiles.push_back(rotated_profiles[0]);
	aligned_profiles.push_back(rotated_profiles[nbCentroids-1]);

	// For each profile (except start and end), find the orthogonal projection
	// of the centroid on the line. Then translates the profile and its centroid
	// so that the centroid rests on the line
	for( int index = 1; index < rotated_centroids.size()-1; index++){

		Point_3 rotatedCentroid = rotated_centroids[index];
		Point_3 centroidProjection = line.projection(rotatedCentroid);
		Vector_3 translationVector(rotatedCentroid, centroidProjection); // Vector_3(a, b) return b-a vector

		// Apply translation on centroid
		Aff_transformation_3 translation(CGAL::TRANSLATION, translationVector);
		Point_3 alignedCentroid = translation.transform(rotatedCentroid);
		// Stores aligned centroid
		aligned_centroids.push_back(alignedCentroid);

		// Apply translation on all the points of the profile
		std::vector<Point_3> rotatedProfilePts(rotated_profiles[index]);
		std::vector<Point_3> alignedProfilePts;
		// Now apply transformations of the profile correspoding to centroid
		for(int pt = 0; pt < rotatedProfilePts.size(); pt++){

			Point_3 tmp = rotatedProfilePts[pt];
			Point_3 alignedPt = translation.transform(tmp);
			alignedProfilePts.push_back(alignedPt);
		}
		aligned_profiles.push_back(alignedProfilePts);
	}

	for(int idx = 0; idx < aligned_profiles.size(); idx++){

		std::vector<Point_3> profile = aligned_profiles[idx];
		for(int pt = 0; pt < profile.size(); pt++){

			Point_3 p = profile[pt];
			Point_3 c = aligned_centroids[idx];
			Vector_3 _v(p, c);
			Vector_3 _n(0.0, 1.0, 0.0);
			std::cout << _v*_n << std::endl;
		}
	}

	nb_profile_samples = 6;

	// Vector containing the sample points for each curve
	/*Vector_vector_point_3 profiles_samples;
	for(int i = 0; i < aligned_profiles.size(); i++){

		std::vector<Point_3> s(sampleProfileCurve(aligned_profiles[i]));
		profiles_samples.push_back(s); // Is it the same as the line above ? why use insert at end ?
	}*/

	/*float profileVariation = generateApproximatedProfileCurves(profiles_samples);
	std::vector<Point_3> cs_samples(sampleProfileCurve(aligned_profiles[0]));
	std::vector<Point_3> ce_samples(sampleProfileCurve(aligned_profiles[aligned_profiles.size()-1]));
	profileVariation += Utils::Haussdorf(cs_samples, ce_samples);

	return profileVariation;*/
	return 0.0f;

}

std::vector<Point_3> GC::sampleProfileCurve(std::vector<Point_3> profile){

	/*int n = profile.size();
	if( n <= nb_profile_samples){

		return profile;
	}else{

		std::vector<Point_3> samples;
		int index = 0;
		while( samples.size() < nb_profile_samples){

			samples.push_back(profile[index]);
			index = (index+nb_profile_samples)%n;
		}
		return samples;
	}*/
	int n = profile.size();
	if( n <= nb_profile_samples){

		return profile;
	}else{
		return Utils::samplePoints(profile, nb_profile_samples);
	}
}

float GC::generateApproximatedProfileCurves(Vector_vector_point_3 profiles_samples){

	float epsilon = 0.01f;
	int nb_profiles = aligned_profiles.size();

	if( nb_profiles < 1){
		return 0.0f;
	}

	std::vector<Point_3> control_profiles;
	// Vector to check if a profile has already been added to the approximated shape
	std::vector<bool> control_profiles_indices(nb_profiles, false);

	// Initialization of the approximated shape with start and end profiles
	std::vector<Point_3> cs_points(aligned_profiles[0]);
	control_profiles.insert(control_profiles.end(), cs_points.begin(), cs_points.end());
	control_profiles_indices[0] = true;
	std::vector<Point_3> ce_points(aligned_profiles[nb_profiles-1]);
	control_profiles.insert(control_profiles.end(), ce_points.begin(), ce_points.end());
	control_profiles_indices[nb_profiles-1] = true;

	bool done = false;
	float sumDist = 0.0f;

	while( !done ){

		CGAL_Mesh approximated_shape = Utils::generateMesh(control_profiles);

		float max_dist = 0.0f;
		int max_index = -1;
		// std::cout << "step 1" << std::endl;
		for(int i = 1; i < nb_profiles-1; i++){

			if( !control_profiles_indices[i]){
				Point_3 profile_centroid = aligned_centroids[i];
				Vec3f v_profile_centroid(profile_centroid);

				// std::cout << "step 2" << std::endl;
				std::vector<Point_3> approx_profile(Utils::cross_section(Vec3f(0,1,0), v_profile_centroid, approximated_shape));
				// std::cout << "step 3" << std::endl;
				// std::cout << "approx_profile size: " << approx_profile.size() <<std::endl;
				std::vector<Point_3> approx_profile_samples(sampleProfileCurve(approx_profile));
				// std::cout << "step 4" << std::endl;
				float dist = Utils::Haussdorf(profiles_samples[i], approx_profile_samples);
				// std::cout << "step 5" << std::endl;
				if( dist > max_dist){

					max_dist = dist;
					max_index = i;
				}
			}
		}

		if((max_dist > epsilon) && (max_index != -1)){
			// std::cout << "step 6" << std::endl;
			sumDist += max_dist;
			control_profiles_indices[max_index] = true;
			std::vector<Point_3> control_prfl_points(aligned_profiles[max_index]);
			// std::cout << "step 7" << std::endl;
			control_profiles.insert(control_profiles.end(), control_prfl_points.begin(),
													control_prfl_points.end());
			approximated_shape.clear();
		}else{
			// std::cout << "step 8" << std::endl;
			done = true;
		}
	}

	return sumDist;
}

float GC::cylindricity(float  C, float alpha){
	float cylindricity = straightness(C) + alpha*profileVariation();
	// float cylindricity = profileVariation();
	return cylindricity;
}


GC GC::merge(GC b){

	int n = axis.size() - 1;
	int m = b.axis.size() - 1;
	HermiteCurve unionOfaxis(axis[n].pe, axis[n].te, b.axis[m].ps, b.axis[m].ts);
	std::vector<HermiteCurve> newAxis;
	newAxis.insert(newAxis.begin(), axis.begin(), axis.end());
	newAxis.push_back(unionOfaxis);
	newAxis.insert(newAxis.end(), b.axis.begin(), b.axis.end());
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
	// std::cout << "getAllPoints(). nb points: " << allPoints.size() << std::endl;
	return allPoints;
}