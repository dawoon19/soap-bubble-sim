#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_3;
uniform vec2 u_texture_3_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
    return length(texture(u_texture_3, uv));
}

void main() {
  // YOUR CODE HERE
    vec3 b = cross(v_normal.xyz,v_tangent.xyz);
    
    mat3 TBN = mat3(v_tangent.xyz,b,v_normal.xyz);
    
    float du = ( h( vec2(v_uv.x + 1.0/u_texture_3_size.x,v_uv.y)) -  h(v_uv)) * u_normal_scaling * u_height_scaling;
    float dv = ( h( vec2(v_uv.x,v_uv.y + 1.0/u_texture_3_size.y)) -  h(v_uv)) * u_normal_scaling * u_height_scaling;
    
    vec3 n0 = vec3(-du,-dv,1);
    vec3 nd = TBN * n0;
    
    
    // phong
    float k_a = 0.1;
    float k_d = 1;
    float k_s = 0.5;
    vec3 I_a = vec3(1,1,1);
    float p = 10;
    
    float r = length(u_light_pos - vec3(v_position));
    vec3 v = normalize(u_cam_pos - vec3(v_position));
    vec3 l = normalize(u_light_pos - vec3(v_position));
    vec3 h = normalize(v + l);
    
//  out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
    out_color.xyz = k_a * I_a + k_d * u_light_intensity / pow(r,2) * max(0, dot(nd,l)) + k_s * u_light_intensity / pow(r,2) * pow( max(0, dot(nd, h)),p);
  out_color.a = 1;
}

