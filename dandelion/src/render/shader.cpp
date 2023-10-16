#include "shader.h"
#include "../utils/math.hpp"
#include<iostream>
#ifdef _WIN32
#undef min
#undef max
#endif

using Eigen::Vector3f;
using Eigen::Vector4f;

// vertex shader & fragement shader can visit
// all the static variables below from Uniforms structure
Eigen::Matrix4f Uniforms::MVP;
Eigen::Matrix4f Uniforms::inv_trans_M;
int Uniforms::width;
int Uniforms::height;

// vertex shader
VertexShaderPayload vertex_shader(const VertexShaderPayload& payload)
{
    VertexShaderPayload output_payload = payload;
    // Vertex position transformation顶点位置变换到视口坐标系

    output_payload.position =Uniforms::MVP * output_payload.position;

    Eigen::Matrix4f viewport = Eigen::Matrix4f::Zero();
    viewport(0, 0) = Uniforms::width/2.f/output_payload.position.w();
    viewport(1, 1) = Uniforms::height/2.f/output_payload.position.w();
    viewport(2, 2) = 1;
    viewport(3, 3) = 1;
    
    Vector4f vv = {Uniforms::width/2.f, Uniforms::height/2.f, 0, 0};
    output_payload.position =viewport * output_payload.position + vv;
    Vector4f tempnormal = {output_payload.normal[0], output_payload.normal[1], output_payload.normal[2], 0};
    tempnormal = Uniforms::inv_trans_M * tempnormal;
    output_payload.normal = {tempnormal[0], tempnormal[1], tempnormal[2]};
    output_payload.normal.normalize();

    // Vertex normal transformation法线变换到世界坐标系

    return output_payload;
}

Vector3f phong_fragment_shader(const FragmentShaderPayload& payload, GL::Material material,
                               const std::list<Light>& lights, Camera camera)
{
    Vector3f result = {0, 0, 0};
    Vector3f vi = camera.position - payload.world_pos;
 
    vi.normalize();
    float Iam = 0.1f;
    float p = material.shininess;
    // ka,kd,ks can be got from material.ambient,material.diffuse,material.specular
    Vector3f ka = material.ambient, kd = material.diffuse, ks = material.specular;
//    std::cout<<ks[0]<<" "<<ks[1]<<" "<<ks[2]<<std::endl;
    Vector3f Ls = {0, 0, 0};
    Vector3f Ld = {0, 0, 0};
    for (std::list<Light>::const_iterator it = lights.begin(); it != lights.end(); ++it) {
        Vector3f li = it->position - payload.world_pos;
 
        float Light_length = li.norm();
        li.normalize();
        Vector3f ha = (vi + li) / (vi + li).norm();
        float attenuated_light = it->intensity / std::pow(Light_length, 2.0f);
        Ld = Ld + kd * (attenuated_light) * std::max(0.0f, payload.world_normal.dot(li));
        Ls = Ls + ks * (attenuated_light) * std::pow(std::max(0.0f, payload.world_normal.dot(ha)), p);
    }

    result = (Ls + Ld + ka * Iam) * 255.f;
    for(int i = 0; i < 3; i++)
    {
        if(result[i] > 255) result[i] = 255.f;
    }

    return result;
}
