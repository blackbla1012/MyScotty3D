
#include "particles.h"

bool Particles::Particle::update(const PT::Aggregate &scene, Vec3 const &gravity, const float radius, const float dt) {

	//A4T4: particle update

	// Compute the trajectory of this particle for the next dt seconds.

	// (1) Build a ray representing the particle's path as if it travelled at constant velocity.

	// (2) Intersect the ray with the scene and account for collisions. Be careful when placing
	// collision points using the particle radius. Move the particle to its next position.

	// (3) Account for acceleration due to gravity after updating position.

	// (4) Repeat until the entire time step has been consumed.

	// (5) Decrease the particle's age and return 'false' if it should be removed.

	float time_left = dt;

	if(velocity.norm() < EPS_F){
		velocity += gravity * dt;
		age -= dt;
		return (age > 0);	
	}

	while(time_left > EPS_F){
		Ray ray = Ray(position, velocity);

		PT::Trace hit = scene.hit(ray);

		if(!hit.hit){
			position += velocity * dt;
			velocity += gravity * dt;
			break;
		}

		float cosTheta = dot(hit.normal, velocity) / (hit.normal.norm() * velocity.norm());
		float t = std::max(0.0f, (hit.distance - std::fabs(radius / cosTheta)) / velocity.norm());
		
		if(t > time_left){
			position += velocity * time_left;
			velocity += gravity * time_left;
			printf("\nposition %f, %f, %f", position.x, position.y, position.z);
			printf("\nvelocity %f, %f, %f", velocity.x, velocity.y, velocity.z);
			break;
		}//edge case

		position += velocity * t;
		velocity = velocity - 2.0f * dot(velocity, hit.normal.unit()) * hit.normal.unit();
		velocity += gravity * t;
		
		time_left = time_left - t;

	}
	age -= dt;
	return (age > 0);

}

void Particles::advance(const PT::Aggregate& scene, const Mat4& to_world, float dt) {

	if(step_size < EPS_F) return;

	step_accum += dt;

	while(step_accum > step_size) {
		step(scene, to_world);
		step_accum -= step_size;
	}
}

void Particles::step(const PT::Aggregate& scene, const Mat4& to_world) {

	std::vector<Particle> next;
	next.reserve(particles.size());

	for(Particle& p : particles) {
		if(p.update(scene, gravity, radius, step_size)) {
			next.emplace_back(p);
		}
	}

	if(rate > 0.0f) {

		//helpful when emitting particles:
		float cos = std::cos(Radians(spread_angle) / 2.0f);

		//will emit particle i when i == time * rate
		//(i.e., will emit particle when time * rate hits an integer value.)
		//so need to figure out all integers in [current_step, current_step+1) * step_size * rate
		//compute the range:
		double begin_t = current_step * double(step_size) * double(rate);
		double end_t = (current_step + 1) * double(step_size) * double(rate);

		uint64_t begin_i = uint64_t(std::max(0.0, std::ceil(begin_t)));
		uint64_t end_i = uint64_t(std::max(0.0, std::ceil(end_t)));

		//iterate all integers in [begin, end):
		for (uint64_t i = begin_i; i < end_i; ++i) {
			//spawn particle 'i':

			float y = lerp(cos, 1.0f, rng.unit());
			float t = 2 * PI_F * rng.unit();
			float d = std::sqrt(1.0f - y * y);
			Vec3 dir = initial_velocity * Vec3(d * std::cos(t), y, d * std::sin(t));

			Particle p;
			p.position = to_world * Vec3(0.0f, 0.0f, 0.0f);
			p.velocity = to_world.rotate(dir);
			p.age = lifetime; //NOTE: could adjust lifetime based on index
			next.push_back(p);
		}
	}

	particles = std::move(next);
	current_step += 1;
}

void Particles::reset() {
	particles.clear();
	step_accum = 0.0f;
	current_step = 0;
	rng.seed(seed);
}

bool operator!=(const Particles& a, const Particles& b) {
	return a.gravity != b.gravity
	|| a.radius != b.radius
	|| a.initial_velocity != b.initial_velocity
	|| a.spread_angle != b.spread_angle
	|| a.lifetime != b.lifetime
	|| a.rate != b.rate
	|| a.step_size != b.step_size
	|| a.seed != b.seed;
}
