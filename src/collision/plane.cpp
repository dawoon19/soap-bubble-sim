#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Plane::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with planes.
    double prev_distance = dot(normal, pm.last_position - point);
    double current_distance = dot(normal, pm.position - point);
    if ((current_distance <= 0 && prev_distance >= 0) || (current_distance >= 0 && prev_distance <= 0)) {
//        Vector3D d = -normal;
//        double t;
//        if (d.norm() != 0)
//            t = dot((point - pm.position),normal) / dot(d, normal);
//        Vector3D tangent_point = pm.position + t * d;
        
        Vector3D tangent_point = pm.position - normal * current_distance;
        float dir_of_plane = prev_distance < 0 ? -1 : 1;
        
        Vector3D dist = (tangent_point - pm.last_position);
        Vector3D correction_vector = dist + normal * (SURFACE_OFFSET * dir_of_plane);
//        Vector3D correction_vector = tangent_point - pm.last_position + normal * SURFACE_OFFSET * dir_of_plane;
                
        
        pm.position = pm.last_position + correction_vector * (1 - friction);
    }

}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  positions.col(0) << sPoint + 2 * (sCross + sParallel);
  positions.col(1) << sPoint + 2 * (sCross - sParallel);
  positions.col(2) << sPoint + 2 * (-sCross + sParallel);
  positions.col(3) << sPoint + 2 * (-sCross - sParallel);

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
