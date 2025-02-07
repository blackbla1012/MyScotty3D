
#include "bvh.h"
#include "aggregate.h"
#include "instance.h"
#include "tri_mesh.h"

#include <stack>

namespace PT {

struct BVHBuildData {
	BVHBuildData(size_t start, size_t range, size_t dst) : start(start), range(range), node(dst) {
	}
	size_t start; ///< start index into the primitive array
	size_t range; ///< range of index into the primitive array
	size_t node;  ///< address to update
};

struct SAHBucketData {
	BBox bb;          ///< bbox of all primitives
	size_t num_prims; ///< number of primitives in the bucket
};

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	//A3T3 - build a bvh

	// Keep these
    nodes.clear();
    primitives = std::move(prims);

    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.

	std::function<void(size_t)> SAH_helper;
	
	SAH_helper = [&](size_t node){
		
		if (nodes[node].size <= max_leaf_size)
		{
			return;
		}

		//Node PNode = nodes[node_index];
		float SAH_min = FLT_MAX;
		size_t index_min = size_t(-1);
		int best_axis = -1;
		for(int axis = 0; axis < 3; axis++)
		{

			std::sort(primitives.begin() + nodes[node].start, primitives.begin() + nodes[node].start + nodes[node].size,
									[axis](const Primitive &a, const Primitive &b)
									{ return a.bbox().center()[axis] < b.bbox().center()[axis];});

			for(size_t i = nodes[node].start + 1; i < nodes[node].start + nodes[node].size; i++)
			{
				BBox left = BBox();
				BBox right = BBox();

				for(size_t j = nodes[node].start; j < nodes[node].start + nodes[node].size; j++)
				{
					if(j < i) left.enclose(primitives[j].bbox());
					else right.enclose(primitives[j].bbox());
				}

				float SAH = (left.surface_area() * (float)(i -nodes[node].start) + right.surface_area() * (float)(nodes[node].start + nodes[node].size - i)) / nodes[node].bbox.surface_area();
				
				if(SAH < SAH_min){
					SAH_min = SAH;
					index_min = i;
					best_axis = axis;
				}
			}

		}

		std::sort(primitives.begin() + nodes[node].start, primitives.begin() + nodes[node].start + nodes[node].size,
								[best_axis](const Primitive &a, const Primitive &b)
								{ return a.bbox().center()[best_axis] < b.bbox().center()[best_axis]; });

		BBox left_best;
		BBox right_best;

		for(size_t i = nodes[node].start; i < nodes[node].start + nodes[node].size; i++){
			if(i < index_min) left_best.enclose(primitives[i].bbox());
			else right_best.enclose(primitives[i].bbox());
		}

		size_t left_index = new_node(left_best, nodes[node].start, (index_min - nodes[node].start));
		size_t right_index = new_node(right_best, index_min, (nodes[node].size + nodes[node].start - index_min));
		nodes[node].l = left_index;
		nodes[node].r = right_index;
		SAH_helper(left_index);
		SAH_helper(right_index);
	};

	BBox root_bb;
	for(size_t i = root_idx; i < primitives.size(); i++){
		root_bb.enclose(primitives[i].bbox());
	}

	size_t root_index = new_node(root_bb, root_idx, primitives.size());

	SAH_helper(root_index);

}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {
	//A3T3 - traverse your BVH

    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

	//TODO: replace this code with a more efficient traversal:
	std::function<Trace(const Ray&, Node)> find_closest_hit = [&](const Ray& ray, Node node)
	{
		if(node.is_leaf())
		{
			Trace ret;
			for(size_t i = node.start; i < node.start + node.size; i++)
			{
				Trace hit = primitives[i].hit(ray);
				ret = Trace::min(ret, hit);
			}
			return ret;
		}
		Node left_child = nodes[node.l];
		Node right_child = nodes[node.r];

		Vec2 times_left = ray.dist_bounds;
		Vec2 times_right = ray.dist_bounds;

		bool hit_left = left_child.bbox.hit(ray, times_left);
		bool hit_right = right_child.bbox.hit(ray, times_right);

		if(hit_left && hit_right)
		{
			if(times_left.x < times_right.x)
			{
				Trace left_trace = find_closest_hit(ray, left_child);
				if(left_trace.hit && left_trace.distance < times_right.x) return left_trace;
				else if(left_trace.hit){
					Trace right_trace = find_closest_hit(ray, right_child);
					if(right_trace.hit && right_trace.distance < left_trace.distance) return right_trace;
					else return left_trace;
				}
				else return find_closest_hit(ray, right_child);
			}
			else{
				Trace right_trace = find_closest_hit(ray, right_child);
				if(right_trace.hit && right_trace.distance < times_left.x) return right_trace;
				else if(right_trace.hit){
					Trace left_trace = find_closest_hit(ray, left_child);
					if(left_trace.hit && left_trace.distance < right_trace.distance) return left_trace;
					else return right_trace;
				}
				else return find_closest_hit(ray, left_child);
			}
		}
		else if(hit_left) return find_closest_hit(ray, left_child);
		else if(hit_right) return find_closest_hit(ray, right_child);
		else{
			Trace ret;
			ret.origin = ray.point;
			ret.hit = false;
			ret.distance = FLT_MAX;  
			ret.position = Vec3{}; 
			ret.normal = Vec3{}; 
			ret.uv = Vec2{};

			return ret;
		}
	};

	Vec2 times = ray.dist_bounds;

	if(nodes.empty()){
		Trace ret;
		ret.origin = ray.point;
		ret.hit = false;
		ret.distance = FLT_MAX;  
		ret.position = Vec3{}; 
		ret.normal = Vec3{}; 
		ret.uv = Vec2{};

		return ret;
	}
	else{
		if(nodes[root_idx].bbox.hit(ray, times))
		{
			Trace hit = find_closest_hit(ray, nodes[root_idx]);
			return hit;
		}
		else{
			Trace ret;
			ret.origin = ray.point;
			ret.hit = false;
			ret.distance = FLT_MAX;  
			ret.position = Vec3{}; 
			ret.normal = Vec3{}; 
			ret.uv = Vec2{};

			return ret;
		}
	}
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	build(std::move(prims), max_leaf_size);
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
	nodes.clear();
	return std::move(primitives);
}

template<typename Primitive>
template<typename P>
typename std::enable_if<std::is_copy_assignable_v<P>, BVH<P>>::type BVH<Primitive>::copy() const {
	BVH<Primitive> ret;
	ret.nodes = nodes;
	ret.primitives = primitives;
	ret.root_idx = root_idx;
	return ret;
}

template<typename Primitive> Vec3 BVH<Primitive>::sample(RNG &rng, Vec3 from) const {
	if (primitives.empty()) return {};
	int32_t n = rng.integer(0, static_cast<int32_t>(primitives.size()));
	return primitives[n].sample(rng, from);
}

template<typename Primitive>
float BVH<Primitive>::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
	if (primitives.empty()) return 0.0f;
	float ret = 0.0f;
	for (auto& prim : primitives) ret += prim.pdf(ray, T, iT);
	return ret / primitives.size();
}

template<typename Primitive> void BVH<Primitive>::clear() {
	nodes.clear();
	primitives.clear();
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
	// A node is a leaf if l == r, since all interior nodes must have distinct children
	return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
	Node n;
	n.bbox = box;
	n.start = start;
	n.size = size;
	n.l = l;
	n.r = r;
	nodes.push_back(n);
	return nodes.size() - 1;
}
 
template<typename Primitive> BBox BVH<Primitive>::bbox() const {
	if(nodes.empty()) return BBox{Vec3{0.0f}, Vec3{0.0f}};
	return nodes[root_idx].bbox;
}

template<typename Primitive> size_t BVH<Primitive>::n_primitives() const {
	return primitives.size();
}

template<typename Primitive>
uint32_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, uint32_t level,
                                   const Mat4& trans) const {

	std::stack<std::pair<size_t, uint32_t>> tstack;
	tstack.push({root_idx, 0u});
	uint32_t max_level = 0u;

	if (nodes.empty()) return max_level;

	while (!tstack.empty()) {

		auto [idx, lvl] = tstack.top();
		max_level = std::max(max_level, lvl);
		const Node& node = nodes[idx];
		tstack.pop();

		Spectrum color = lvl == level ? Spectrum(1.0f, 0.0f, 0.0f) : Spectrum(1.0f);
		GL::Lines& add = lvl == level ? active : lines;

		BBox box = node.bbox;
		box.transform(trans);
		Vec3 min = box.min, max = box.max;

		auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

		edge(min, Vec3{max.x, min.y, min.z});
		edge(min, Vec3{min.x, max.y, min.z});
		edge(min, Vec3{min.x, min.y, max.z});
		edge(max, Vec3{min.x, max.y, max.z});
		edge(max, Vec3{max.x, min.y, max.z});
		edge(max, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

		if (!node.is_leaf()) {
			tstack.push({node.l, lvl + 1});
			tstack.push({node.r, lvl + 1});
		} else {
			for (size_t i = node.start; i < node.start + node.size; i++) {
				uint32_t c = primitives[i].visualize(lines, active, level - lvl, trans);
				max_level = std::max(c + lvl, max_level);
			}
		}
	}
	return max_level;
}

template class BVH<Triangle>;
template class BVH<Instance>;
template class BVH<Aggregate>;
template BVH<Triangle> BVH<Triangle>::copy<Triangle>() const;

} // namespace PT
