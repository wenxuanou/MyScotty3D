#include "../scene/skeleton.h"

Vec3 closest_on_line_segment(Vec3 start, Vec3 end, Vec3 point) {

    // TODO(Animation): Task 3

    // Return the closest point to 'point' on the line segment from start to end
    // return location of the point on line
    
    Vec3 v1 = end - start;
    Vec3 v2 = point - start;
    // project of point on line
    Vec3 proj = dot(v2, v1.unit()) * v1.unit();
    
    Vec3 point = proj + proj;   // TODO: check result
    
    return point;
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
    
    
    Vec3 bind_end;
    bind_end = Mat4::translate(base_pos) * j->joint_to_bind() * (j->extent);
    return bind_end;    // pure translation
    
}

Vec3 Skeleton::posed_end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the posed position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.

    
    Vec3 posed_end;
    posed_end = Mat4::translate(base_pos) * j->joint_to_posed() * (j->extent);
    return posed_end;   // translation and rotation
    
}

Mat4 Skeleton::joint_to_bind(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space in
    // bind position. This should take into account Skeleton::base_pos.
    
    
    Mat4 to_bind_transform = j->joint_to_bind() * Mat4::translate(base_pos);
    
    // return pure translation matrix
    return to_bind_transform;
    
    
//    return Mat4::I;
}

Mat4 Skeleton::joint_to_posed(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space with
    // poses. This should take into account Skeleton::base_pos.
    
    
    Mat4 to_pose_transform = j->joint_to_posed() * Mat4::translate(base_pos);
    
    // return both translation and rotation
    return to_pose_transform;
    
    
//    return Mat4::I;
}

void Skeleton::find_joints(const GL::Mesh& mesh,
                           std::unordered_map<unsigned int, std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Construct a mapping from vertex indices to lists of joints in this skeleton
    // that should effect the vertex at that index. A joint should effect a vertex
    // if it is within Joint::radius distance of the bone's line segment in bind position.

    const std::vector<GL::Mesh::Vert>& verts = mesh.verts();    // vector of vertices

    // For each i in [0, verts.size()), map[i] should contain the list of joints that
    // effect vertex i. Note that i is NOT Vert::id! i is the index in verts.

    for(int i = 0; i < verts.size(); i ++){
        
        // TODO: check if coordinace space is correct
        Vec3 v_pos = verts[i].pos;      // vertex position, assume in bind space,
        
        // TODO: check if this will be recycled
        std::vector<Joint*> J_related;  // joint related to the vertex
        
        
        // TODO: check if this working
        // iterate all joints in skeleton
        for_joints([&](Joint* j) {
            // What vertices does joint j effect?
           
            // take vertex point back to joint space
            Mat4 to_joint = joint_to_bind(j);           // only translation // TODO: check here
            Vec3 v_posJoint = Mat4::inverse(to_joint) * v_pos;
            // find distance to the cloest point on bone
            Vec3 dist = v_pos - closest_on_line_segment(Vec3(0.0f), j->extent, v_posJoint);
            
            if(dist.norm() <= j->radius){
                // include joint if in radius
                J_related.push_back(j);
            }
        });
        
        
        // create and insert element to map
        map.emplace(i, J_related);  // TODO: check if this work
//        map.insert((std::make_pair<unsigned int, std::vector<Joint*> >(i, J_related));
    }
    
    
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
        // need joint_to_bind
        
        Vec3 v_pos = verts[i].pos;              // current position
        Vec3 v_norm = verts[i].norm;            // current norm
        Vec3 v_posNew, v_normNew;               // new position and norm
        
        std::vector<Joint*> J_related = map[i]; // all related joint of this vertex
        std::vector<float> weights;             // weigth buffer for each joint
        float weight_sum = 0.0f;
        
        // compute weights
        for(size_t countJ = 0; countJ < J_related.size(); countJ++){
            
            Joint* j = J_related[countJ];
            
            // map vertex position to joint position
            Mat4 to_joint = joint_to_posed(j);   // include translation and rotation
            Vec3 v_posJoint = Mat4::inverse(to_joint) * v_pos;
            
            // find distance to the cloest point on bone
            Vec3 dist = v_pos - closest_on_line_segment(Vec3(0.0f), j->extent, v_posJoint);
            
            // weight of joint on vertex
            float w = 1.0f / dist.norm();
            weights.push_back(w);
            weight_sum += w;
        }
        
        // compute new vertex position
        for(size_t countJ = 0; countJ < J_related.size(); countJ++){
            
            Joint* j = J_related[countJ];
            
            // map vertex position to joint position
            Mat4 to_joint = joint_to_posed(j);   // include translation and rotation
            Vec3 v_posJoint = Mat4::inverse(to_joint) * v_pos;
            
            // TODO: how to get vji?
            float weight = weights[countJ] / weight_sum;
            v_posNew = v_posNew + weight * to_joint * v_posJoint;
            
        }
        
        verts[i].pos = v_posNew;
        
    }

    std::vector<GL::Mesh::Index> idxs = input.indices();    // input vertex index, stay the same
    output.recreate(std::move(verts), std::move(idxs));
}

void Joint::compute_gradient(Vec3 target, Vec3 current) {

    // TODO(Animation): Task 2

    // Computes the gradient of IK energy for this joint and, should be called
    // recursively upward in the heirarchy. Each call should storing the result
    // in the angle_gradient for this joint.

    // Target is the position of the IK handle in skeleton space.
    // Current is the end position of the IK'd joint in skeleton space.
    
    
    // initialize values
    Vec3 p_theta = current;
    Vec3 q = target;
    
    // difference between target and posed in skeleton space, (0,0,0) in joint space, vector
    Vec3 p = target - joint_to_posed() * Vec3(0.0f);    // point to target
    

    Vec3 parent_angle_grad(0.0f);
    
    // check if have parent
    // compute on parent first
    if(parent != nullptr){
        parent -> compute_gradient(target, current);
        parent_angle_grad = parent -> angle_gradient;
    }
    
    
    
    // accumulate parent gradient
    angle_gradient = parent_angle_grad;
    
    // compute gradient on current
    // do xyz separately
    Vec3 r_x(1.0f, 0.0f, 0.0f),
         r_y(0.0f, 1.0f, 0.0f),
         r_z(0.0f, 0.0f, 1.0f);
    
    Vec3 J_theta_x = cross(r_x, p);     // rotate around x
    Vec3 J_theta_y = cross(r_y, p);     // rotate around y
    Vec3 J_theta_z = cross(r_z, p);     // rotate around z
    
    // add up by axis
    angle_gradient.x = dot(J_theta_x, (p_theta - q)) - angle_gradient.x;    // subtract the gradient of parent
    angle_gradient.y = dot(J_theta_y, (p_theta - q)) - angle_gradient.y;
    angle_gradient.z = dot(J_theta_z, (p_theta - q)) - angle_gradient.z;
    
    
}

void Skeleton::step_ik(std::vector<IK_Handle*> active_handles) {

    // TODO(Animation): Task 2

    // Do several iterations of Jacobian Transpose gradient descent for IK
    
    // main loop
    int maxIter = 50;
    float alpht_timestep = 0.005f;
    for(int iter = 0; iter < maxIter; iter++){
        // iterate each handle
        for(size_t id = 0; id < active_handles.size(); id++){
            
            Joint* j = active_handles[id]->joint;
            Vec3 target = active_handles[id]->target;            // target position of joint, in skeleton space
            Vec3 current = (j->joint_to_posed()) * (j->extent);   // current position of joint, in skeleton space
            
            j->compute_gradient(target, current);
            
            // update pose
            j->pose -= alpht_timestep * j->angle_gradient;
            j->angle_gradient = Vec3(0.0f); // reset gradient to 0
            
        }
        
        
    }
    
}
