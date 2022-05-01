#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  
  // (Placeholder code. You will want to replace it.)
    float k_a = 0.5;
//    float k_a = 1;
    float k_d = 1;
//    float k_d = 0;
    float k_s = 0.5;
//    float k_s = 0;
    vec3 I_a = vec3(1,1,1);
    float p = 10;
    
    float r = length(u_light_pos - vec3(v_position));
    vec3 v = normalize(u_cam_pos - vec3(v_position));
    vec3 l = normalize(u_light_pos - vec3(v_position));
    vec3 h = normalize(v + l);
    
//  out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
    out_color.xyz = k_a * I_a + k_d * u_light_intensity / pow(r,2) * max(0, dot(v_normal.xyz,l)) + k_s * u_light_intensity / pow(r,2) * pow( max(0, dot(v_normal.xyz, h)),p);
    out_color.a = 1;
}

