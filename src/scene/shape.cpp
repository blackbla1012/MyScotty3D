
#include "shape.h"
#include "../geometry/util.h"

namespace Shapes {

Vec2 Sphere::uv(Vec3 dir) {
	float u = std::atan2(dir.z, dir.x) / (2.0f * PI_F);
	if (u < 0.0f) u += 1.0f;
	float v = std::acos(-1.0f * std::clamp(dir.y, -1.0f, 1.0f)) / PI_F;
	return Vec2{u, v};
}

BBox Sphere::bbox() const {
	BBox box;
	box.enclose(Vec3(-radius));
	box.enclose(Vec3(radius));
	return box;
}

PT::Trace Sphere::hit(Ray ray) const {
	//A3T2 - sphere hit

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!
	float discriminant = 4 * dot(ray.point, ray.dir) * dot(ray.point, ray.dir) - 4 * ray.dir.norm_squared() * (ray.point.norm_squared() - radius * radius);
	
	PT::Trace ret;
	ret.origin = ray.point;
	//no solution
	if(discriminant < 0){
		ret.hit = false;       // was there an intersection?
		ret.distance = 0.0f;   // at what distance did the intersection occur?
		ret.position = Vec3{}; // where was the intersection?
		ret.normal = Vec3{};   // what was the surface normal at the intersection?
		ret.uv = Vec2{}; 	   // what was the uv coordinates at the intersection? (you may find Sphere::uv to be useful)
		
	}
	else{
		float t1 = (-2 * dot(ray.point, ray.dir) + std::sqrt(discriminant)) / 2 * ray.dir.norm_squared();
		float t2 = (-2 * dot(ray.point, ray.dir) - std::sqrt(discriminant)) / 2 * ray.dir.norm_squared();
		if(t1 >= 0 && t2 >= 0){
			if(t2 >= ray.dist_bounds.x && t2 <= ray.dist_bounds.y){
				ret.hit = true;
				ret.distance = t2;
				ret.position = ray.at(t2);
				ret.normal = ray.at(t2).unit();
				ret.uv = uv(ret.position);
			}else if(t1 >= ray.dist_bounds.x && t1 <= ray.dist_bounds.y){
				ret.hit = true;
				ret.distance = t1;
				ret.position = ray.at(t1);
				ret.normal = ray.at(t1).unit();
				ret.uv = uv(ret.position);
			}else{
				ret.hit = false;   
				ret.distance = FLT_MAX;
				ret.position = Vec3{};
				ret.normal = Vec3{};
				ret.uv = Vec2{};
			}
		}else if(t1 >= 0 && t2 < 0){
			if(t1 >= ray.dist_bounds.x && t1 <= ray.dist_bounds.y){
				ret.hit = true;
				ret.distance = t1;
				ret.position = ray.at(t1);
				ret.normal = ray.at(t1).unit();
				ret.uv = uv(ret.position);
			}else{
				ret.hit = false;   
				ret.distance = FLT_MAX;
				ret.position = Vec3{};
				ret.normal = Vec3{};
				ret.uv = Vec2{};
			}
		}else{
			ret.hit = false;   
			ret.distance = FLT_MAX;
			ret.position = Vec3{};
			ret.normal = Vec3{};
			ret.uv = Vec2{};
		}
	}

	return ret;
    
}

Vec3 Sphere::sample(RNG &rng, Vec3 from) const {
	die("Sampling sphere area lights is not implemented yet.");
}

float Sphere::pdf(Ray ray, Mat4 pdf_T, Mat4 pdf_iT) const {
	die("Sampling sphere area lights is not implemented yet.");
}

Indexed_Mesh Sphere::to_mesh() const {
	return Util::sphere_mesh(radius, 2);
}

} // namespace Shapes

bool operator!=(const Shapes::Sphere& a, const Shapes::Sphere& b) {
	return a.radius != b.radius;
}

bool operator!=(const Shape& a, const Shape& b) {
	if (a.shape.index() != b.shape.index()) return false;
	return std::visit(
		[&](const auto& shape) {
			return shape != std::get<std::decay_t<decltype(shape)>>(b.shape);
		},
		a.shape);
}
