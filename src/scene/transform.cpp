
#include "transform.h"

Mat4 Transform::local_to_parent() const {
	return Mat4::translate(translation) * rotation.to_mat() * Mat4::scale(scale);
}

Mat4 Transform::parent_to_local() const {
	return Mat4::scale(1.0f / scale) * rotation.inverse().to_mat() * Mat4::translate(-translation);
}

Mat4 Transform::local_to_world() const {

	Mat4 result = Mat4::translate(translation) * rotation.to_mat() * Mat4::scale(scale);

    if (std::shared_ptr<Transform> parent_ = parent.lock()) {
        result = parent_->local_to_world() * result;
    }

    return result;
	
}

Mat4 Transform::world_to_local() const {

	Mat4 result = Mat4::scale(1.0f / scale) * rotation.inverse().to_mat() * Mat4::translate(-translation);

	if (std::shared_ptr<Transform> parent_ = parent.lock()) {
        result = result * parent_->world_to_local();
    }

	return result; 
}

bool operator!=(const Transform& a, const Transform& b) {
	return a.parent.lock() != b.parent.lock() || a.translation != b.translation ||
	       a.rotation != b.rotation || a.scale != b.scale;
}


