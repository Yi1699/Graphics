#include "shader.h"
#include "../utils/math.hpp"

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
    Eigen::Matrix4f viewport = Eigen::Matrix4f::Zero();
    viewport(0, 0) = Uniforms::width/2;
    viewport(0, 3) = Uniforms::width/2;
    viewport(1, 1) = Uniforms::height/2;
    viewport(1, 3) = Uniforms::height/2;
    viewport(2, 2) = 1;
    viewport(3, 3) = 1;

    output_payload.position = viewport * Uniforms::MVP * output_payload.position;
    // Vertex normal transformation法线变换到世界坐标系
    output_payload.normal = Uniforms::inv_trans_M * output_payload.normal;

    return output_payload;
}

Vector3f phong_fragment_shader(const FragmentShaderPayload& payload, GL::Material material,
                               const std::list<Light>& lights, Camera camera)
{

    Vector3f result = {0, 0, 0};
    
    // ka,kd,ks can be got from material.ambient,material.diffuse,material.specular
    Vector3f ka = material.ambient, kd = material.diffuse, ks = material.shininess;

    for (std::list<Light>::iterator it = values.begin(); it != values.end(); ++it) {
        
    }

    // set ambient light intensity

    // Light Direction
    
    // View Direction
        
    // Half Vector
        
    // Light Attenuation
        
    // Ambient
        
    // Diffuse
        
    // Specular
        
    // set rendering result max threshold to 255
    
    return result * 255.f;
}
