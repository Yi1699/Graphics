#include "ray.h"
#include <iostream>
#include <cmath>
#include <array>

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include "../utils/math.hpp"

using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
using std::numeric_limits;
using std::optional;
using std::size_t;

constexpr float infinity = 1e5f;
constexpr float eps      = 1e-5f;

Intersection::Intersection() : t(numeric_limits<float>::infinity()), face_index(0)
{
}

Ray generate_ray(int width, int height, int x, int y, Camera& camera, float depth)
{
    Vector2f pos((float)x+0.5f, (float)y+0.5f);
    Vector2f center((float)width/2.0f, (float)height/2.0f);
    Matrix4f inv_view = camera.view().inverse();
    Vector4f view_pos_specified(pos.x()-center.x(), -(pos.y()-center.y()), -depth, 1.0f);
    //从成像平面生成射线
    float fov_y = radians(camera.fov_y_degrees);
    float ratio = (2.0f * tan(fov_y / 2.0f)) / (float)height;
    //根据比例计算归一化的相机坐标系坐标，其深度为-1
    Vector4f view_pos = ratio * view_pos_specified;
    view_pos[3] = 1.0f;
    view_pos[2] = -1.0f;
    //转换到世界坐标系
    Vector4f word_pos_specified = (inv_view * view_pos);
    // The ratio between the specified plane (width x height)'s depth and the image plane's depth.
    
    // Transfer the view-space position to world space.
    Vector3f world_pos(word_pos_specified.x(), word_pos_specified.y(), word_pos_specified.z());
    Vector3f rayDir = (world_pos - camera.position).normalized();
    //返回射线的出发点坐标与射线方向
    return {camera.position, rayDir};
}

optional<Intersection> ray_triangle_intersect(const Ray& ray, const GL::Mesh& mesh, size_t index)
{
    // these lines below are just for compiling and can be deleted
    (void)ray;
    (void)mesh;
    (void)index;
    // these lines above are just for compiling and can be deleted
    Intersection result;
    
    if (result.t - infinity < -eps) {
        return result;
    } else {
        return std::nullopt;
    }
}

optional<Intersection> naive_intersect(const Ray& ray, const GL::Mesh& mesh, const Matrix4f model)
{//求射线与面片的交点
    Intersection result;
    result.t = infinity;
    for (size_t i = 0; i < mesh.faces.count(); ++i) {
        //获取三角形的三个顶点坐标
        Vector3f P0 = mesh.vertex(mesh.face(i)[0]);
        Vector3f P1 = mesh.vertex(mesh.face(i)[1]);
        Vector3f P2 = mesh.vertex(mesh.face(i)[2]);
        // Vector3f N0 = mesh.normal(mesh.face(i)[0]);
        // Vector3f N1 = mesh.normal(mesh.face(i)[1]);
        // Vector3f N2 = mesh.normal(mesh.face(i)[2]);
        Vector4f P04(P0.x(), P0.y(), P0.z(), 1.f);
        Vector4f P14(P1.x(), P1.y(), P1.z(), 1.f);
        Vector4f P24(P2.x(), P2.y(), P2.z(), 1.f);
        P04 = model * P04;
        P14 = model * P14;
        P24 = model * P24;
        P0 = (P04).head<3>();
        P1 = (P14).head<3>();
        P2 = (P24).head<3>();
        //用MT算法求交点
        Vector3f E1 = P1 - P0;
        Vector3f E2 = P2 - P0;
        Vector3f S = ray.origin - P0;
        Vector3f S1 = (ray.direction).cross(E2);
        Vector3f S2 = S.cross(E1);
        Vector3f P10 = P1 - P0;
        Vector3f P21 = P2 - P1;
        Vector3f Norm = (P10.cross(P21)).normalized();
        float t = S2.dot(E2) / (E1.dot(S1));
        float b1 = S1.dot(S) / (E1.dot(S1));
        float b2 = S2.dot(ray.direction) / (E1.dot(S1));
        float b0 = 1- b1 - b2;
        float judge = Norm.dot(ray.direction);
        if(t > eps && b1 > 0 && b2 > 0 && b0 > 0 && judge < -eps && t - eps < result.t)
        {//当求出的t大于0，且重心坐标均大于零（说明交点在三角形内），光线不与三角形平行且从正面穿过，求得的交点比原最近交点更近。
            Vector3f Vec = {b0, b1, b2};
            result.t = t - eps;
            //Vector3f InsertNorm = (b0 * N0 + b1 * N1 + b2 * N2).normalized();
            result.normal = Norm;//或 = InsertNorm;
            result.barycentric_coord = Vec;
            result.face_index = i;
        }
        // Vertex a, b and c are assumed to be in counterclockwise order.
        // Construct matrix A = [d, a - b, a - c] and solve Ax = (a - origin)
        // Matrix A is not invertible, indicating the ray is parallel with the triangle.
        // Test if alpha, beta and gamma are all between 0 and 1.
    }
    // Ensure result.t is strictly less than the constant `infinity`.
    if (result.t - infinity < -eps) {
        return result;
    }
    return std::nullopt;
}
