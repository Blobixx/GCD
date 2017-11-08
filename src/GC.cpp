#include "GC.h"
#include <CGAL/Aff_transformation_3.h>

typedef CGAL::Aff_transformation_3<K> Aff_transformation_3; //K kernel defined in intersect.h


float GC::straightness(float C){

}

float GC::profile_variation(){

	//Sample the axis with a dense set of points.
	int nb_points = 10;
	std::vector<Vec3f> original_points_sample; //sampling points on axis
	std::vector<Polylines> original_profile_curves;

	//for loop gives actually 11 points containing ps and pe
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
				align_curve_polylines.push_back(aligne_curve);
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

//nb_profiles "before" p, nb_profiles after p
void GC::generate_profiles(int nb_profiles, float step) {

	// for(int j = 0; j < nb_profiles+1; j++ ){
	// 	Vec3f p_step = ((nb_profiles - j)/nb_profiles)*pe + (j/nb_profiles)*ps; 
	// 	profiles.push_back(intersect(axis, p_step, shape));
	// }

}

GC GC::merge(GC b){

}