#include "GC.h"
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/squared_distance_3.h>
#include <CGAL/Line_3.h>

typedef CGAL::Aff_transformation_3<K> Aff_transformation_3; //K kernel defined in intersect.h

float GC::straightness(float C){

	float straightness = C;
	float epsilon = 0.005f;

	//Create CGAL format for ps, pe, and axis as a line
	Point_3 _ps(GC.axis.ps[0], GC.axis.ps[1], GC.axis.ps[2]);
	Point_3 _pe(GC.axis.pe[0], GC.axis.pe[1], GC.axis.pe[2]);

	//constains all the control_points ordered by their parameter's value
	std::vector<controlPoint_t> list;

	bool done = false;
	while( !done ){

		controlPoint_t control_point = findMaxDistToLine(_ps, _pe);
		if(control_point.dist > epsilon){
			for(int i = 0; i < list.size(); i++){
				if( control_point.parameter > list[i].parameter){
					list.insert(list.begin() + i, control_point);
					break;
				}
			}
		}else{
			done = true;
		}
	}

	for(int i = 0; i < list.size(); i++){
		straightness += list[i].dist;
	}

	return straightness;

}

controlPoint_t GC::findMaxDistToLine(Point_3 start_point, Point_3 end_point){

	Line_3 line(start_point, end_point);
	float max_dist = 0.f;
	float value = 0.f; //control point paramater

	//Increasing step is 0.01
	for(int t = 0; t < 100; t++){
		//Consider point on axis curve
		Vec3f point = axis.interpolate(t/100.0f);
		Point_3 _point(point[0], point[1], point[2]);

		//Computes distance from point to line
		float dist = squared_distance_3(_point, line);
		if( dist > max_dist){
			max_dist = dist;
			control_point = t/100.0f;
		}
	}

	controlPoint_t control_point;
	control_point.parameter = parameter;
	control_point.dist = max_dist;

	return control_point;
}

float GC::profile_variation(){

	int nb_points = 10;
	std::vector<Vec3f> original_points_sample; //sampling points on axis
	std::vector<Polylines> original_profile_curves;

	//Sample the axis with a dense set of points.
	//for loop gives actually 11 points containing p_s and p_e
	//Calcule profile curves as cross sections with the shape perpendicular to the axis at p
	for(int i = 0; i <= nb_points; i++){
		Vec3f p = axis.interpolate(i/nb_points);
		original_points_sample.push_back(p);
		Vec3f tangent_at_p = axis.get_tangent(i/nb_points);
		Vec3f plane_normal_at_p = normalize(tangent_at_p); //The normal of the plane perpendicular to axis at p is the tangeant at p
		original_profile_curves.push_back(cross_section(plane_normal_at_p, p, shape));
	}

	//Align profile curves so they reside on parallel planes (normal_vector(0,1,0))
	//Align centroids so they lie on the line passing through ps with direction vector (0,1,0)
	Vec3f n(0,1,0);
	float cos = dot(plane_normal_at_p, n);
	Vec3f v = cross(plane_normal_at_p, n);
	float v1 = v[0];
	float v2 = v[1];
	float v3 = v[2];
	std::vector<Polylines> aligned_profile_curves;
	std::vector<K::Point_3> aligned_centroids_sample;

	if( cos != -1){
		Aff_transformation_3 rotation(1-(v2*v2+v3*v3)/(1+cos), -1.f*v3+(v1*v2)/(1+cos), v2+(v1*v3)/(1+cos),
										v3+(v1v2)/(1+cos), 1-(v1*v1+v3*v3)/(1+cos), -1.f*v1+(v2v3)/(1+cos),
										-1.f*v2+(v1v3)/(1+cos), v1+(v2*v3)/(1+cos), 1-(v1*v1+v2*v2)/(1+cos)
									);

		for(int cross_section_index = 0; cross_section_index < original_profile_curves.size(); cross_section_index++){
			Vec3f centroid = original_points_sample[centroid_index];
			Point_3 centroid_point(centroid[0], centroid[1], centroid[2]); //Vec3f to Point_3
			Point_3 rotated_centroid = centroid_point.transform(rotation);

			Vector_3 translation_vector(ps[0] - rotated_centroid.x(), 0, ps[2] - rotated_centroid.z());
			Point_3 aligned_centroid = rotated_centroid.transform(translation_vector);

			aligned_centroids_sample.push_back(aligned_centroid);

			Polylines curve_polylines = original_profile_curves[cross_section_index];
			Polylines aligned_curve_polylines;

			for(int cross_section_polyline_type_index = 0; cross_section_polyline_type_index < curve_polylines.size(); cross_section_polyline_type_index++){
				Polyline_type curve = curve_polylines[cross_section_polyline_type_index];
				Polyline_type aligned_curve;

				for(int point_index = 0; point_index < curve.size(); point_index++){
					Point_3 current_point = curve[point_index];
						Point_3 rotated_point = current_point.transform(rotation);
						Point_3 aligned_point = rotated_point.transform(translation_vector);
						aligned_curve.push_back(rotated_point);
				}
				align_curve_polylines.push_back(aligned_curve);
			}
			aligned_profile_curves.push_back(aligned_curve_polylines);
		}

	}else{
		// do something
	}

}

float GC::cylindricity(float alpha, float C){
	float cylindricity = straightness(C) + alpha*profile_variation();
	return cylindricity;
}

float Haussdorf(Polylines c_i, Polylines c_j){

	int nb_samples = 6;
	//The sample points are 2D points but I consider them as 3D points with y = 0 (so I can use distance method
	//from CGAL)

	// How to sample when I have no idea about how the points are positioned ?
	std::vector<Point_3> c_i_samples;
	std::vector<Point_3> c_j_samples;

	//Let's consider there is only 1 component per polyline
	int size_i = c_i.size();
	int size_j = c_j.size();
	float min_ij = MAX_VALUE; // how to express MAX_VALUE in C++ ?
	float min_ji = MAX_VALUE;

	for(int k = 0; k < size_i; k++){
		for(int h = 0; h < size_j, h++){
			float d = CGAL::squared_distance_3(c_i_samples[k], c_j_samples[h]);
			if (d < min_ij){
				min_ij = d;
			}
		}
	}

	for(int h = 0; h < size_j; h++){
		for(int k = 0; k < size_i, k++){
			float d = CGAL::squared_distance_3(c_j_samples[h], c_i_samples[k]);
			if (d < min_ji){
				min_ji = d;
			}
		}
	}

	return std::max(min_ij, min_ji);

}


//nb_profiles "before" p, nb_profiles after p
void GC::generate_profiles(int nb_profiles, float step) {

	// for(int j = 0; j < nb_profiles+1; j++ ){
	// 	Vec3f p_step = ((nb_profiles - j)/nb_profiles)*p_e + (j/nb_profiles)*ps;
	// 	profiles.push_back(intersect(axis, p_step, shape));
	// }

}

GC GC::merge(GC b){

}