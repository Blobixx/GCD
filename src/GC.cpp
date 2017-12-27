#include "GC.h"
#include <cmath>
#include <utility>


float GC::straightness(float C){
	std::cout << "Starting computing straightness ..." << std::endl;
	float straightness = C;
	float epsilon = 0.0000003f;

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
	int nb_points = 5; // nb of centroids
	std::vector<Polylines> original_profiles;
	// vector of normals of the plane perpendicular the axis at each sample points
	std::vector<Vec3f> orth_planes;

	// Sample the axis with a dense set of points.
	// for loop gives actually 11 points containing ps and pe
	// Computes profile curves as cross sections with the shape perpendicular to the axis at point
	for(int j = 0; j < axis.size(); j++){

		HermiteCurve hermite_curve = axis[j];
		for(int i = 0; i <= nb_points; i++){

			Vec3f s = hermite_curve.interpolate(i/nb_points);
			original_centroids.push_back(s);
			Vec3f tangent_at_s = hermite_curve.get_tangent(i/nb_points);
			Vec3f orth_plane_at_s = normalize(tangent_at_s); //The normal of the plane perpendicular to axis at p is the tangeant at p
			original_profiles.push_back(Utils::cross_section(orth_plane_at_s, s, shape));
			orth_planes.push_back(orth_plane_at_s);
		}
	}

	/* Align profile curves so they reside on parallel planes (normal_vector(0,1,0))
	   Align centroids so they lie on the line passing through ps with direction vector (0,1,0)*/
	Vec3f n(0,1,0);

	for(int index = 0; index < original_centroids.size(); index++){

		Vec3f centroid = original_centroids[index];
		Point_3 centroid_3(centroid[0], centroid[1], centroid[2]); //Vec3f to Point_3

		Vec3f orth_plane_at_centroid = orth_planes[index];
		float cos = dot(orth_plane_at_centroid, n);
		Vec3f v = cross(orth_plane_at_centroid, n);
		float v1 = v[0];
		float v2 = v[1];
		float v3 = v[2];

		if( cos != -1){
			// rotation to orient the plane so that its normal is n(0,1,0)
			Aff_transformation_3 rotation(1-(v2*v2+v3*v3)/(1+cos), -1.f*v3+(v1*v2)/(1+cos), v2+(v1*v3)/(1+cos),
										v3+(v1*v2)/(1+cos), 1-(v1*v1+v3*v3)/(1+cos), -1.f*v1+(v2*v3)/(1+cos),
										-1.f*v2+(v1*v3)/(1+cos), v1+(v2*v3)/(1+cos), 1-(v1*v1+v2*v2)/(1+cos)
									);
			// Apply rotation on centroid
			Point_3 rotated_centroid = rotation.transform(centroid_3);
			//Translation to position all centroid on the straight line going throu ps and pe
			Vector_3 translation_vector(ps[0] - rotated_centroid.x(), 0, ps[2] - rotated_centroid.z());
			// Apply translation on centroid
			Aff_transformation_3 translation(CGAL::TRANSLATION, translation_vector);
			Point_3 aligned_centroid = translation.transform(rotated_centroid);
			aligned_centroids.push_back(aligned_centroid);

			// Now apply transformations of the profile correspoding to centroid
			Polylines profile(original_profiles[index]);
			Polylines aligned_profile;

			for(int polyline_type_index = 0; polyline_type_index < profile.size(); polyline_type_index++){
				Polyline_type profile_component(profile[polyline_type_index]);
				Polyline_type aligned_profile_component;

				for(int point_index = 0; point_index < profile_component.size(); point_index++){
					Point_3 current_point = profile_component[point_index];

					// Apply rotation and translation on every point of each profiles curve;
					Point_3 rotated_point = rotation.transform(current_point);
					Point_3 aligned_point = translation.transform(rotated_point);
					aligned_profile_component.push_back(aligned_point);
				}
				aligned_profile.push_back(aligned_profile_component);
			}
			aligned_profiles.push_back(aligned_profile);
		}else{
	// TODO: case cos = -1
			std::cout << "ici" << std::endl;
		}
	}

	/* Now each aligned profile is sampled with the same nb of points
	   The following assures that this nb of points is at least the nb of points of each curve */
	/*Polylines poly = aligned_profiles[0];
	Polyline_type poly_type = poly[0];
	// Number of points in the first component of the first curve
	int min_polytype_size = poly_type.size();
	for(int i = 0; i < aligned_profiles.size(); i++){
		Polylines _poly = aligned_profiles[i];
		for(int j = 0; j < _poly.size(); j++){
			Polyline_type _poly_type = _poly[j];
			int s = _poly_type.size();
			if( s < min_polytype_size){
				min_polytype_size = s;
			}
		}
	}
	int nb_profile_samples = 6;
	int nb_samples = std::min(nb_profile_samples, min_polytype_size);*/
	nb_profile_samples = 6;
	for(int i = 0; i < aligned_profiles.size(); i++){
		int nbPts = Utils::getAllPoints(aligned_profiles[i]).size();
		if( nbPts < nb_profile_samples){
			nb_profile_samples = nbPts;
		}
	}

	// Vector containing the sample points for each curve
	Vector_vector_points profiles_samples;
	for(int i = 0; i < aligned_profiles.size(); i++){
		std::vector<Point_3> s(sampleProfileCurve(aligned_profiles[i], nb_profile_samples));
		// profiles_samples[i].insert(profiles_samples[i].end(), s.begin(), s.end());
		profiles_samples.push_back(s); // Is it the same as the line above ? why use insert at end ?
	}


	// std::cout << "merde 1" << std::endl;
	float profileVariation = generateApproximatedProfileCurves(profiles_samples);
	// std::cout << "merde 2" << std::endl;
	std::vector<Point_3> cs_samples(sampleProfileCurve(aligned_profiles[0], nb_profile_samples));
	// std::cout << "merde 3" << std::endl;
	std::vector<Point_3> ce_samples(sampleProfileCurve(aligned_profiles[aligned_profiles.size()-1], nb_profile_samples));
	// std::cout << "merde 4" << std::endl;
	profileVariation += Utils::Haussdorf(cs_samples, ce_samples);

	std::cout << "End of profile variation." << std::endl;
	return profileVariation;
}

// TODO: Can be moved to Utils.h ?
std::vector<Point_3> GC::sampleProfileCurve(Polylines profile, int nb_samples){

	// int n = profile.size(); //nb components of the polylines
	// // Select x points per component so that x*nb_component = nb_samples
	// int nb_points_per_component = (int)std::floor(nb_samples/n);
	// std::vector<Point_3> samples;
	// for(int c = 0; c < n; c++){

	// 	Polyline_type polytype = profile[c];
	// 	for(int x = 0; x < nb_points_per_component; x++){

	// 		samples.push_back(polytype[x]);
	// 	}
	// }
	std::vector<Point_3> profile_points(Utils::getAllPoints(profile));
	int n = profile_points.size();
	if( n <= nb_samples){
		return profile_points;
	}else{
		std::vector<Point_3> samples;
		int index = 0;
		while( samples.size() < nb_samples){
			samples.push_back(profile_points[index]);
			index = (index+nb_samples)%n;
		}
		return samples;
	}
	// return samples;
}

float GC::generateApproximatedProfileCurves(Vector_vector_points profiles_samples){

	float epsilon = 0.01f;
	int nb_profiles = aligned_profiles.size();

	if( nb_profiles < 1){
		return 0.0f;
	}

	std::vector<Point_3> control_profiles;
	// Vector to check if a profile has already been added to the approximated shape
	std::vector<bool> control_profiles_indices(nb_profiles, false);

	// Initialization of the approximated shape with start and end profiles
	std::vector<Point_3> cs_points(Utils::getAllPoints(aligned_profiles[0]));
	control_profiles.insert(control_profiles.end(), cs_points.begin(), cs_points.end());
	control_profiles_indices[0] = true;
	std::vector<Point_3> ce_points(Utils::getAllPoints(aligned_profiles[nb_profiles-1]));
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
				Vec3f v_profile_centroid(profile_centroid.x(), profile_centroid.y(), profile_centroid.z());

				// std::cout << "step 2" << std::endl;
				Polylines approx_profile(Utils::cross_section(Vec3f(0,1,0), v_profile_centroid, approximated_shape));
				// std::cout << "step 3" << std::endl;
				std::vector<Point_3> approx_profile_samples(sampleProfileCurve(approx_profile, nb_profile_samples));
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
			std::vector<Point_3> control_prfl_points(Utils::getAllPoints(aligned_profiles[max_index]));
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

		Polylines poly = profiles[i];
		for(int j = 0; j < poly.size(); j++){

			Polyline_type poly_type = poly[j];
			for(int k = 0; k < poly_type.size(); k++){

				allPoints.push_back(poly_type[k]);
			}
		}
	}
	// std::cout << "getAllPoints(). nb points: " << allPoints.size() << std::endl;
	return allPoints;
}