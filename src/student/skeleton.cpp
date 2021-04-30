
#include "../scene/skeleton.h"

Vec3 closest_on_line_segment(Vec3 start, Vec3 end, Vec3 point) {

    // TODO(Animation): Task 3

    // Return the closest point to 'point' on the line segment from start to end
    return Vec3{};
}

Mat4 Joint::joint_to_bind() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space in bind position.

    // Bind position implies that all joints have pose = Vec3{0.0f}

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos
    
    
    Mat4 to_bind_transform;
    
    if(parent == nullptr){
        // at root, no translation
        to_bind_transform = Mat4::I;
    }else{
        // apply parent transform on the right, parent first, then local
        to_bind_transform = parent->joint_to_bind() * Mat4::translate(parent->extent);
    }
    
    // return pure translation matrix
    return to_bind_transform;
}

Mat4 Joint::joint_to_posed() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space, taking into account joint poses.

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos
    
    
    Mat4 to_pose_transform;
    
    if(parent == nullptr){
        // at root, no translation
        to_pose_transform = Mat4::euler(pose);
    }else{
        // apply parent transform on the right, parent first, then local
        to_pose_transform = parent->joint_to_posed() * Mat4::translate(parent->extent) * Mat4::euler(pose);
    }
    
    // return both translation and rotation
    return to_pose_transform;
}

Vec3 Skeleton::end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the bind position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.
    
    
    // apply base position on th right, base first
    Mat4 to_bind_transform = joint_to_bind(j) * Mat4::translate(base_pos);
    Vec3 end_bind = to_bind_transform * (j->extent);    // TODO: check if this is correct
    
    // pure translation
    return end_bind;
}

Vec3 Skeleton::posed_end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the posed position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.
    
    
    // apply base position on th right, base first
    Mat4 to_pose_transform = joint_to_posed(j) * Mat4::translate(base_pos);
    Vec3 end_pose = to_pose_transform * (j->extent);
    
    
    // translation and rotation
    return end_pose;
}

Mat4 Skeleton::joint_to_bind(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space in
    // bind position. This should take into account Skeleton::base_pos.
    
    
    Mat4 to_bind_transform = j->joint_to_bind() * Mat4::translate(base_pos);
    
    // return pure translation matrix
    return to_bind_transform;
}

Mat4 Skeleton::joint_to_posed(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space with
    // poses. This should take into account Skeleton::base_pos.
    
    
    Mat4 to_pose_transform = j->joint_to_posed() * Mat4::translate(base_pos);
    
    // return both translation and rotation
    return to_pose_transform;
    
}

void Skeleton::find_joints(const GL::Mesh& mesh,
                           std::unordered_map<unsigned int, std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Construct a mapping from vertex indices to lists of joints in this skeleton
    // that should effect the vertex at that index. A joint should effect a vertex
    // if it is within Joint::radius distance of the bone's line segment in bind position.

    const std::vector<GL::Mesh::Vert>& verts = mesh.verts();
    (void)verts;

    // For each i in [0, verts.size()), map[i] should contain the list of joints that
    // effect vertex i. Note that i is NOT Vert::id! i is the index in verts.

    for_joints([&](Joint* j) {
        // What vertices does joint j effect?
    });
}

void Skeleton::skin(const GL::Mesh& input, GL::Mesh& output,
                    const std::unordered_map<unsigned int, std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Apply bone poses & weights to the vertices of the input (bind position) mesh
    // and store the result in the output mesh. See the task description for details.
    // map was computed by find_joints, hence gives a mapping from vertex index to
    // the list of bones the vertex should be effected by.

    // Currently, this just copies the input to the output without modification.

    std::vector<GL::Mesh::Vert> verts = input.verts();
    for(size_t i = 0; i < verts.size(); i++) {

        // Skin vertex i. Note that its position is given in object bind space.
    }

    std::vector<GL::Mesh::Index> idxs = input.indices();
    output.recreate(std::move(verts), std::move(idxs));
}

void Joint::compute_gradient(Vec3 target, Vec3 current) {

    // TODO(Animation): Task 2

    // Computes the gradient of IK energy for this joint and, should be called
    // recursively upward in the heirarchy. Each call should storing the result
    // in the angle_gradient for this joint.

    // Target is the position of the IK handle in skeleton space.
    // Current is the end position of the IK'd joint in skeleton space.
}

void Skeleton::step_ik(std::vector<IK_Handle*> active_handles) {

    // TODO(Animation): Task 2

    // Do several iterations of Jacobian Transpose gradient descent for IK
}
