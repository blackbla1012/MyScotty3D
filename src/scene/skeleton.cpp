#include <unordered_set>
#include "skeleton.h"
#include "test.h"
#include <iostream>

void Skeleton::Bone::compute_rotation_axes(Vec3 *x_, Vec3 *y_, Vec3 *z_) const {
	assert(x_ && y_ && z_);
	auto &x = *x_;
	auto &y = *y_;
	auto &z = *z_;

	//y axis points in the direction of extent:
	y = extent.unit();
	//if extent is too short to normalize nicely, point along the skeleton's 'y' axis:
	if (!y.valid()) {
		y = Vec3{0.0f, 1.0f, 0.0f};
	}

	//x gets skeleton's 'x' axis projected to be orthogonal to 'y':
	x = Vec3{1.0f, 0.0f, 0.0f};
	x = (x - dot(x,y) * y).unit();
	if (!x.valid()) {
		//if y perfectly aligns with skeleton's 'x' axis, x, gets skeleton's z axis:
		x = Vec3{0.0f, 0.0f, 1.0f};
		x = (x - dot(x,y) * y).unit(); //(this should do nothing)
	}

	//z computed from x,y:
	z = cross(x,y);

	//x,z rotated by roll:
	float cr = std::cos(roll / 180.0f * PI_F);
	float sr = std::sin(roll / 180.0f * PI_F);
	// x = cr * x + sr * -z;
	// z = cross(x,y);
	std::tie(x, z) = std::make_pair(cr * x + sr * -z, cr * z + sr * x);
}

std::vector< Mat4 > Skeleton::bind_pose() const {
	//A4T2a: bone-to-skeleton transformations in the bind pose
	//(the bind pose does not rotate by Bone::pose)

	std::vector< Mat4 > bind;
	bind.reserve(bones.size());

	//NOTE: bones is guaranteed to be ordered such that parents appear before child bones.
	
	for (auto const &bone : bones) {
		(void)bone; //avoid complaints about unused bone
		//placeholder -- your code should actually compute the correct transform:
		if(bone.parent == -1U) {
			bind.emplace_back(Mat4::translate(base));
		}
		else{
			BoneIndex parent_index = bone.parent;
			bind.emplace_back(bind[parent_index] * Mat4::translate(bones[parent_index].extent));
		}
	}

	assert(bind.size() == bones.size()); //should have a transform for every bone.
	return bind;
}

std::vector< Mat4 > Skeleton::current_pose() const {
    //A4T2a: bone-to-skeleton transformations in the current pose

	//Similar to bind_pose(), but takes rotation from Bone::pose into account.
	// (and translation from Skeleton::base_offset!)

	//You'll probably want to write a loop similar to bind_pose().

	//Useful functions:
	//Bone::compute_rotation_axes() will tell you what axes (in local bone space) Bone::pose should rotate around.
	//Mat4::angle_axis(angle, axis) will produce a matrix that rotates angle (in degrees) around a given axis.
	std::vector< Mat4 > current;
	current.reserve(bones.size());
	
	for (auto const &bone : bones) {
		Vec3 x, y, z;
		bone.compute_rotation_axes(&x, &y, &z);
		if(bone.parent == -1U) {
			current.emplace_back(Mat4::translate(base + base_offset) * Mat4::angle_axis(bone.pose.z, z) * Mat4::angle_axis(bone.pose.y, y) * Mat4::angle_axis(bone.pose.x, x));
		}
		else{
			BoneIndex parent_index = bone.parent;
			current.emplace_back(current[parent_index] * Mat4::translate(bones[parent_index].extent) * Mat4::angle_axis(bone.pose.z, z) * Mat4::angle_axis(bone.pose.y, y) * Mat4::angle_axis(bone.pose.x, x));
		}
	}

	assert(current.size() == bones.size()); //should have a transform for every bone.
	return current;
}

std::vector< Vec3 > Skeleton::gradient_in_current_pose() const {
    //A4T2b: IK gradient

    // Computes the gradient (partial derivative) of IK energy relative to each bone's Bone::pose, in the current pose.

	//The IK energy is the sum over all *enabled* handles of the squared distance from the tip of Handle::bone to Handle::target
	std::vector< Vec3 > gradient(bones.size(), Vec3{0.0f, 0.0f, 0.0f});

	//TODO: loop over handles and over bones in the chain leading to the handle, accumulating gradient contributions.
	//remember bone.compute_rotation_axes() -- should be useful here, too!

	std::vector<Mat4> current_poses = current_pose();
	for(auto const &handle : handles){

		if(handle.bone == -1U || !handle.enabled) continue;

		BoneIndex bone_index = handle.bone;
		Vec3 p = current_poses[bone_index] * bones[bone_index].extent;
		
		while(bones[bone_index].parent != -1U){

			Bone bone = bones[bone_index];
			Bone parent_bone = bones[bone.parent];

			Vec3 x, y, z;
			bones[bone_index].compute_rotation_axes(&x, &y, &z);

			Mat4 xform_x = current_poses[bone.parent] * Mat4::translate(parent_bone.extent) * Mat4::angle_axis(bone.pose.z, z) * Mat4::angle_axis(bone.pose.y, y);
			Mat4 xform_y = current_poses[bone.parent] * Mat4::translate(parent_bone.extent) * Mat4::angle_axis(bone.pose.z, z);
			Mat4 xform_z = current_poses[bone.parent] * Mat4::translate(parent_bone.extent);

			Vec3 r = current_poses[bone_index] * Vec3(0.0f, 0.0f, 0.0f);
			gradient[bone_index].x += dot((p - handle.target), cross(xform_x.rotate(x), (p-r)));
			gradient[bone_index].y += dot((p - handle.target), cross(xform_y.rotate(y), (p-r)));
			gradient[bone_index].z += dot((p - handle.target), cross(xform_z.rotate(z), (p-r)));

			bone_index = bones[bone_index].parent;
		}

		if(bones[bone_index].parent == -1U){

			Bone bone = bones[bone_index];
			
			Vec3 x, y, z;
			bones[bone_index].compute_rotation_axes(&x, &y, &z);

			Mat4 xform_x = Mat4::translate(base + base_offset) * Mat4::angle_axis(bone.pose.z, z) * Mat4::angle_axis(bone.pose.y, y);
			Mat4 xform_y = Mat4::translate(base + base_offset) * Mat4::angle_axis(bone.pose.z, z);
			Mat4 xform_z =  Mat4::translate(base + base_offset);

			Vec3 r = current_poses[bone_index] * Vec3(0.0f, 0.0f, 0.0f);
			gradient[bone_index].x += dot((p - handle.target), cross(xform_x.rotate(x), (p-r)));
			gradient[bone_index].y += dot((p - handle.target), cross(xform_y.rotate(y), (p-r)));
			gradient[bone_index].z += dot((p - handle.target), cross(xform_z.rotate(z), (p-r)));
		}
	}

	assert(gradient.size() == bones.size());
	return gradient;
}

bool Skeleton::solve_ik(uint32_t steps) {
	//A4T2b - gradient descent
	//check which handles are enabled
	//run `steps` iterations
	
	//call gradient_in_current_pose() to compute d loss / d pose
	//add ...

	//if at a local minimum (e.g., gradient is near-zero), return 'true'.
	//if run through all steps, return `false`.

	float timestep = 0.1f;
	float tolerance = 0.000001f;
	for(uint32_t i = 0; i < steps; i++){
		std::vector<Vec3> gradient = gradient_in_current_pose();
		float grad_norm = 0.0f;
		for(int j = 0; j < gradient.size(); j++)
			grad_norm += gradient[j].norm_squared();
		if (grad_norm <= tolerance) return true;
		for(size_t k = 0; k < bones.size(); k++)
			bones[k].pose -= timestep * gradient[k];

	}
	return false;
}

Vec3 Skeleton::closest_point_on_line_segment(Vec3 const &a, Vec3 const &b, Vec3 const &p) {
	//A4T3: bone weight computation (closest point helper)

    // Return the closest point to 'p' on the line segment from a to b

	//Efficiency note: you can do this without any sqrt's! (no .unit() or .norm() is needed!)
	Vec3 AP = p - a;
	Vec3 AB = b - a;
	float ratio = dot(AP, AB) / AB.norm_squared();
	if(ratio > 1.0f) return b;
	else if(ratio < 0.0f) return a;
	else return (a + ratio * AB);
	
}

void Skeleton::assign_bone_weights(Halfedge_Mesh *mesh_) const {
	assert(mesh_);
	auto &mesh = *mesh_;
	(void)mesh; //avoid complaints about unused mesh

	//A4T3: bone weight computation

	//visit every vertex and **set new values** in Vertex::bone_weights (don't append to old values)

	//be sure to use bone positions in the bind pose (not the current pose!)

	//you should fill in the helper closest_point_on_line_segment() before working on this function
	auto bpose = bind_pose();

	for(auto v:mesh.vertices){
		v.bone_weights.clear();
		std::vector<Halfedge_Mesh::Vertex::Bone_Weight> bone_weights;
		float weightSum = 0.0f;
	
		for(BoneIndex i = 0U; i < bones.size(); i++){
			Vec3 Closest_Point = closest_point_on_line_segment(bpose[i] * Vec3(0.0f), bpose[i] * bones[i].extent, v.position);
			Vec3 wijVec = v.position - Closest_Point;
			float wijNorm = wijVec.norm();
			float weight = std::max(0.0f, bones[i].radius - wijNorm) / bones[i].radius;
			bone_weights.emplace_back(Halfedge_Mesh::Vertex::Bone_Weight{i, weight});
			weightSum += weight;
		}

		if(weightSum > 0.0f){
			for(auto &weight : bone_weights){
				weight.weight /= weightSum;
			}
			v.bone_weights = bone_weights;
		}

	}

}

Indexed_Mesh Skeleton::skin(Halfedge_Mesh const &mesh, std::vector< Mat4 > const &bind, std::vector< Mat4 > const &current) {
	assert(bind.size() == current.size());


	//A4T3: linear blend skinning

	//one approach you might take is to first compute the skinned positions (at every vertex) and normals (at every corner)
	// then generate faces in the style of Indexed_Mesh::from_halfedge_mesh

	//---- step 1: figure out skinned positions ---

	std::unordered_map< Halfedge_Mesh::VertexCRef, Vec3 > skinned_positions;
	std::unordered_map< Halfedge_Mesh::HalfedgeCRef, Vec3 > skinned_normals;
	//reserve hash table space to (one hopes) avoid re-hashing:
	skinned_positions.reserve(mesh.vertices.size());
	skinned_normals.reserve(mesh.halfedges.size());

	//(you will probably want to precompute some bind-to-current transformation matrices here)
	std::vector<Mat4> bind_to_curr;
	bind_to_curr.reserve(bind.size());
	for(int i = 0; i < bind_to_curr.size(); i++){
		Mat4 bToCBone =bind[i].inverse() * current[i];
		bind_to_curr.emplace_back(bToCBone);
	}

	for (auto vi = mesh.vertices.begin(); vi != mesh.vertices.end(); ++vi) {


		//skinned_positions.emplace(vi, vi->position); //PLACEHOLDER! Replace with code that computes the position of the vertex according to vi->position and vi->bone_weights.
		//NOTE: vertices with empty bone_weights should remain in place.

		Mat4 weightedb_to_C = Mat4::Zero;
		if(vi->bone_weights.size() == 0) skinned_positions.emplace(vi, vi->position);
		else{
			for(BoneIndex i = 0U; i < vi->bone_weights.size(); i++){
				weightedb_to_C += bind_to_curr[vi->bone_weights[i].bone] * vi->bone_weights[i].weight;
			}
			skinned_positions.emplace(vi, weightedb_to_C * vi->position); 
		}

		if(weightedb_to_C == Mat4::Zero) weightedb_to_C = Mat4::I;
		

		//circulate corners at this vertex:
		auto h = vi->halfedge;
		do {
			//NOTE: could skip if h->face->boundary, since such corners don't get emitted
			if(h->face->boundary) continue;

			Mat4 b_to_cNormalWeighted =  Mat4::transpose(Mat4::inverse(weightedb_to_C));
			skinned_normals.emplace(h, (b_to_cNormalWeighted * h->corner_normal).unit()); //PLACEHOLDER! Replace with code that properly transforms the normal vector! Make sure that you normalize correctly.

			h = h->twin->next;
		} while (h != vi->halfedge);
	}

	//---- step 2: transform into an indexed mesh ---

	//Hint: you should be able to use the code from Indexed_Mesh::from_halfedge_mesh (SplitEdges version) pretty much verbatim, you'll just need to fill in the positions and normals.

	std::vector<Indexed_Mesh::Vert> verts;
	std::vector<Indexed_Mesh::Index> idxs;

	for (Halfedge_Mesh::FaceCRef f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
		if (f->boundary) continue;

		//every corner gets its own copy of a vertex:
		uint32_t corners_begin = static_cast<uint32_t>(verts.size());
		Halfedge_Mesh::HalfedgeCRef h = f->halfedge;
		do {
			Indexed_Mesh::Vert vert;
			vert.pos = skinned_positions.at(h->vertex);
			vert.norm = skinned_normals.at(h);
			vert.uv = h->corner_uv;
			vert.id = f->id;
			verts.emplace_back(vert);
			h = h->next;
		} while (h != f->halfedge);
		uint32_t corners_end = static_cast<uint32_t>(verts.size());

		//divide face into a triangle fan:
		for (size_t i = corners_begin + 1; i + 1 < corners_end; i++) {
			idxs.emplace_back(corners_begin);
			idxs.emplace_back(static_cast<uint32_t>(i));
			idxs.emplace_back(static_cast<uint32_t>(i+1));
		}
	}

	Indexed_Mesh result = Indexed_Mesh(std::move(verts), std::move(idxs));//PLACEHOLDER! you'll probably want to copy the SplitEdges case from this function o'er here and modify it to use skinned_positions and skinned_normals.

	return result;
}

void Skeleton::for_bones(const std::function<void(Bone&)>& f) {
	for (auto& bone : bones) {
		f(bone);
	}
}


void Skeleton::erase_bone(BoneIndex bone) {
	assert(bone < bones.size());
	//update indices in bones:
	for (uint32_t b = 0; b < bones.size(); ++b) {
		if (bones[b].parent == -1U) continue;
		if (bones[b].parent == bone) {
			assert(b > bone); //topological sort!
			//keep bone tips in the same place when deleting parent bone:
			bones[b].extent += bones[bone].extent;
			bones[b].parent = bones[bone].parent;
		} else if (bones[b].parent > bone) {
			assert(b > bones[b].parent); //topological sort!
			bones[b].parent -= 1;
		}
	}
	// erase the bone
	bones.erase(bones.begin() + bone);
	//update indices in handles (and erase any handles on this bone):
	for (uint32_t h = 0; h < handles.size(); /* later */) {
		if (handles[h].bone == bone) {
			erase_handle(h);
		} else if (handles[h].bone > bone) {
			handles[h].bone -= 1;
			++h;
		} else {
			++h;
		}
	}
}

void Skeleton::erase_handle(HandleIndex handle) {
	assert(handle < handles.size());

	//nothing internally refers to handles by index so can just delete:
	handles.erase(handles.begin() + handle);
}


Skeleton::BoneIndex Skeleton::add_bone(BoneIndex parent, Vec3 extent) {
	assert(parent == -1U || parent < bones.size());
	Bone bone;
	bone.extent = extent;
	bone.parent = parent;
	//all other parameters left as default.

	//slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set< uint32_t > used;
	for (auto const &b : bones) {
		used.emplace(b.channel_id);
	}
	while (used.count(next_bone_channel_id)) ++next_bone_channel_id;
	bone.channel_id = next_bone_channel_id++;

	//all other parameters left as default.

	BoneIndex index = BoneIndex(bones.size());
	bones.emplace_back(bone);

	return index;
}

Skeleton::HandleIndex Skeleton::add_handle(BoneIndex bone, Vec3 target) {
	assert(bone < bones.size());
	Handle handle;
	handle.bone = bone;
	handle.target = target;
	//all other parameters left as default.

	//slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set< uint32_t > used;
	for (auto const &h : handles) {
		used.emplace(h.channel_id);
	}
	while (used.count(next_handle_channel_id)) ++next_handle_channel_id;
	handle.channel_id = next_handle_channel_id++;

	HandleIndex index = HandleIndex(handles.size());
	handles.emplace_back(handle);

	return index;
}


Skeleton Skeleton::copy() {
	//turns out that there aren't any fancy pointer data structures to fix up here.
	return *this;
}

void Skeleton::make_valid() {
	for (uint32_t b = 0; b < bones.size(); ++b) {
		if (!(bones[b].parent == -1U || bones[b].parent < b)) {
			warn("bones[%u].parent is %u, which is not < %u; setting to -1.", b, bones[b].parent, b);
			bones[b].parent = -1U;
		}
	}
	if (bones.empty() && !handles.empty()) {
		warn("Have %u handles but no bones. Deleting handles.", uint32_t(handles.size()));
		handles.clear();
	}
	for (uint32_t h = 0; h < handles.size(); ++h) {
		if (handles[h].bone >= HandleIndex(bones.size())) {
			warn("handles[%u].bone is %u, which is not < bones.size(); setting to 0.", h, handles[h].bone);
			handles[h].bone = 0;
		}
	}
}

//-------------------------------------------------

Indexed_Mesh Skinned_Mesh::bind_mesh() const {
	return Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges);
}

Indexed_Mesh Skinned_Mesh::posed_mesh() const {
	return Skeleton::skin(mesh, skeleton.bind_pose(), skeleton.current_pose());
}

Skinned_Mesh Skinned_Mesh::copy() {
	return Skinned_Mesh{mesh.copy(), skeleton.copy()};
}
