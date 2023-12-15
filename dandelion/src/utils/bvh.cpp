#include "bvh.h"

#include <cassert>
#include <iostream>
#include <optional>

#include <Eigen/Geometry>
#include "formatter.hpp"
#include <spdlog/spdlog.h>

#include "math.hpp"

using Eigen::Vector3f;
using std::optional;
using std::vector;

BVHNode::BVHNode() : left(nullptr), right(nullptr), face_idx(0)
{
}

BVH::BVH(const GL::Mesh& mesh) : root(nullptr), mesh(mesh)
{
}

// 建立bvh，将需要建立BVH的图元索引初始化
void BVH::build()
{
    if (mesh.faces.count() == 0) {
        root = nullptr;
        return;
    }

    primitives.resize(mesh.faces.count());
    for (size_t i = 0; i < mesh.faces.count(); i++) primitives[i] = i;

    root = recursively_build(primitives);
    return;
}
// 删除bvh
void BVH::recursively_delete(BVHNode* node)
{
    if (node == nullptr)
        return;
    recursively_delete(node->left);
    recursively_delete(node->right);
    delete node;
    node = nullptr;
}
// 统计BVH树建立的节点个数
size_t BVH::count_nodes(BVHNode* node)
{
    if (node == nullptr)
        return 0;
    else
        return count_nodes(node->left) + count_nodes(node->right) + 1;
}
// 递归建立BVH
BVHNode* BVH::recursively_build(vector<size_t> faces_idx)
{
    
    BVHNode* node = new BVHNode();
    node->face_idx = 0;
    vector<SortNode> face_sort;
    AABB aabb;
    for (size_t i = 0; i < faces_idx.size(); i++) {
        AABB tp = get_aabb(mesh, faces_idx[i]);
        SortNode tpa = {faces_idx[i], tp.centroid()};
        face_sort.push_back(tpa);
        aabb = union_AABB(aabb, tp);
    }
    node->aabb = aabb;
    if(faces_idx.size() == 1) 
    {   
        node->left = NULL;
        node->right = NULL;
        node->face_idx = faces_idx[0];
        return node;
    }
    else if(faces_idx.size() == 2)
    {
        vector<size_t> face_left = {faces_idx[0]};
        vector<size_t> face_right = {faces_idx[1]};
        node->left = recursively_build(face_left);
        node->right = recursively_build(face_right);
        //node->aabb = union_AABB(node->left->aabb, node->right->aabb);
    }
    else
    {
        int lst_dimension;
        lst_dimension = aabb.max_extent();
        
        vector<size_t> leftfaces, rightfaces;
        if(lst_dimension == 0)
        {
            SortNode MinNode = face_sort[0];
            int min_id;
            for(int i = 0; i < faces_idx.size() - 1; i++)
            {
                MinNode = face_sort[i];
                min_id = i;
                for(int j = i + 1; j < faces_idx.size(); j++)
                {
                    if(face_sort[j].centroid.x() < MinNode.centroid.x())
                    {
                        MinNode = face_sort[j];
                        min_id = j;
                    }
                }
                SortNode t = face_sort[i];
                face_sort[i] = MinNode;
                face_sort[min_id] = t;
            }
            for(int i = 0; i < face_sort.size(); i++)
            {
                if(i <= face_sort.size() / 2) leftfaces.push_back(face_sort[i].index);
                else rightfaces.push_back(face_sort[i].index);
            }
        }
        else if(lst_dimension == 1)
        {
            SortNode MinNode = face_sort[0];
            int min_id;
            for(int i = 0; i < faces_idx.size() - 1; i++)
            {
                MinNode = face_sort[i];
                min_id = i;
                for(int j = i + 1; j < faces_idx.size(); j++)
                {
                    if(face_sort[j].centroid.y() < MinNode.centroid.y())
                    {
                        MinNode = face_sort[j];
                        min_id = j;
                    }
                }
                SortNode t = face_sort[i];
                face_sort[i] = MinNode;
                face_sort[min_id] = t;
            }
            for(int i = 0; i < face_sort.size(); i++)
            {
                if(i <= face_sort.size() / 2) leftfaces.push_back(face_sort[i].index);
                else rightfaces.push_back(face_sort[i].index);
            }
        }
        else
        {
            SortNode MinNode = face_sort[0];
            int min_id;
            for(int i = 0; i < faces_idx.size() - 1; i++)
            {
                MinNode = face_sort[i];
                min_id = i;
                for(int j = i + 1; j < faces_idx.size(); j++)
                {
                    if(face_sort[j].centroid.z() < MinNode.centroid.z())
                    {
                        MinNode = face_sort[j];
                        min_id = j;
                    }
                }
                
                SortNode t = face_sort[i];
                face_sort[i] = MinNode;
                face_sort[min_id] = t;
            }
            for(int i = 0; i < face_sort.size(); i++)
            {
                if(i <= face_sort.size() / 2) leftfaces.push_back(face_sort[i].index);
                else rightfaces.push_back(face_sort[i].index);
            }
        }
        node->left = recursively_build(leftfaces);
        node->right = recursively_build(rightfaces);
    }
    // if faces_idx.size()==1: return node;
    // if faces_idx.size()==2: recursively_build() & union_AABB(node->left->aabb,
    // node->right->aabb); else:
    // choose the longest dimension among x,y,z
    // devide the primitives into two along the longest dimension
    // recursively_build() & union_AABB(node->left->aabb, node->right->aabb)
    return node;
}
// 使用BVH求交
optional<Intersection> BVH::intersect(const Ray& ray, [[maybe_unused]] const GL::Mesh& mesh,
                                      const Eigen::Matrix4f obj_model)
{
    model = obj_model;
    Eigen::Matrix4f inv_model = this->model.inverse();
    optional<Intersection> isect;
    if (!root) {
        isect = std::nullopt;
        return isect;
    }
    Ray ray_model;
    ray_model.direction = ((model.inverse() * to_vec4(ray.direction)).head<3>()).normalized();
    ray_model.origin = ((model.inverse() * ray.origin.homogeneous())).head<3>();
    isect = ray_node_intersect(root, ray_model);
    if(isect.has_value())
    {
        isect->normal = ((model.inverse().transpose()) * (to_vec4(isect->normal))).head<3>();
        //std::cout << isect->normal.x()<< " "<<isect->normal.y()<<" "<< isect->normal.z() << std::endl;
    }
    return isect;
}
// 发射的射线与当前节点求交，并递归获取最终的求交结果
optional<Intersection> BVH::ray_node_intersect(BVHNode* node, const Ray& ray) const
{
    if(!node) return std::nullopt;
    optional<Intersection> isect;
    // these lines below are just for compiling and can be deleted
    Eigen::Matrix4f inv_model = this->model.inverse();
    Eigen::Vector3f inv_dir(1.0f / ray.direction.x(), 1.0f / ray.direction.y(), 1.0f / ray.direction.z());
    std::array<int, 3> dir_neg;
    dir_neg[0] = ray.direction.x() > 0 ? 0 : 1;
    dir_neg[1] = ray.direction.y() > 0 ? 0 : 1;
    dir_neg[2] = ray.direction.z() > 0 ? 0 : 1;
    bool isIn = node->aabb.intersect(ray, inv_dir, dir_neg);
    if(!isIn)
    {
        return std::nullopt;
    }
    else
    {
        if(node->face_idx > 0)
        {
            isect = ray_triangle_intersect(ray, mesh, node->face_idx);
        }
        else
        {
            optional<Intersection> left_result, right_result;
            if(node->left)left_result = ray_node_intersect(node->left, ray);
            if(node->right)right_result = ray_node_intersect(node->right, ray);
            if(left_result.has_value() && !right_result.has_value()) isect = left_result;
            else if(right_result.has_value() && !left_result.has_value()) isect = right_result;
            else if(right_result.has_value() && left_result.has_value())
            {
                if(right_result->t <= left_result->t) isect = right_result;
                else isect = left_result;
            }
            else
            {
                return std::nullopt;
            }
        }
    }


    // The node intersection is performed in the model coordinate system.
    // Therefore, the ray needs to be transformed into the model coordinate system.
    // The intersection attributes returned are all in the model coordinate system.
    // Therefore, They are need to be converted to the world coordinate system.    
    // If the model shrinks, the value of t will also change.
    // The change of t can be solved by intersection point changing simultaneously
            
    return isect;
}
