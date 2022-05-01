#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
    // add point masses
    for (double h = 0; h < num_height_points; h++) {
        for (double w = 0; w < num_width_points; w++) {
            double x = w * ((double)width / (double)num_width_points);
            double y = h * ((double)height / (double)num_height_points);
            Vector3D position;
            bool pin = false;
            if (orientation == 0) {
                position = Vector3D(x, 1.0, y);
            }
            else {
                double r_offset = (1.0 * rand() / RAND_MAX - 1.0) / 1000.0;
                position = Vector3D(x, y, r_offset);
            }
            point_masses.emplace_back(PointMass(position, pin));
            
        }
    }
    //if pinned
    for (int i = 0; i < pinned.size(); i++) {
        point_masses[pinned[i][1] * num_width_points + pinned[i][0]].pinned = true;
    }
    //springs
    for (int h = 0; h < num_height_points; h++) {
        for (int w = 0; w < num_width_points; w++) {
            //structural
            if (w > 0) {
                springs.emplace_back(Spring(&point_masses[h * num_width_points + w], &point_masses[h * num_width_points + (w - 1)], STRUCTURAL));
            }
            if (h > 0) {
                springs.emplace_back(Spring(&point_masses[h * num_width_points + w], &point_masses[(h - 1) * num_width_points + w], STRUCTURAL));
            }
            //shearing
            if (h > 0 && w > 0) {
                springs.emplace_back(Spring(&point_masses[h * num_width_points + w], &point_masses[(h - 1) * num_width_points + (w - 1)], SHEARING));
            }
            if (h > 0 && num_width_points > (w + 1)) {
                springs.emplace_back(Spring(&point_masses[h * num_width_points + w], &point_masses[(h - 1) * num_width_points + (w + 1)], SHEARING));
            }
            //bending
            if (w > 1) {
                springs.emplace_back(Spring(&point_masses[h * num_width_points + w], &point_masses[h * num_width_points + (w - 2)], BENDING));
            }
            if (h > 1) {
                springs.emplace_back(Spring(&point_masses[h * num_width_points + w], &point_masses[(h - 2) * num_width_points + w], BENDING));
            }
        }
    }

}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
    Vector3D F_ext = Vector3D(0,0,0);
    for (int i = 0; i < external_accelerations.size(); i++) {
        F_ext += mass * external_accelerations[i];
    }

    for (vector<PointMass>::iterator ptr = point_masses.begin(); ptr < point_masses.end(); ptr++) {
        ptr->forces = F_ext;
    }

    for (vector<Spring>::iterator spr = springs.begin(); spr < springs.end(); spr++) {
        if ((spr->spring_type == BENDING && cp->enable_bending_constraints)
            || (spr->spring_type == SHEARING && cp->enable_shearing_constraints)
            || (spr->spring_type == STRUCTURAL && cp->enable_structural_constraints)) {

            Vector3D F_s = spr->pm_b->position - spr->pm_a->position;
            double F_s_abs = cp->ks * 0.2 * ( F_s.norm() - spr->rest_length );
            F_s.normalize();
            F_s *= F_s_abs;

            spr->pm_a->forces += F_s;
            spr->pm_b->forces -= F_s;
        }
    }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
    for (vector<PointMass>::iterator ptr = point_masses.begin(); ptr < point_masses.end(); ptr++) {
        if (!ptr->pinned && mass != 0) {
            Vector3D a_t = ptr->forces / mass;
            Vector3D new_pos = ptr->position + (1 - cp->damping / 100.0) * (ptr->position - ptr->last_position) + a_t * pow(delta_t, 2);
            ptr->last_position = ptr->position;
            ptr->position = new_pos;
        }
    }

  // TODO (Part 4): Handle self-collisions.
    build_spatial_map();
    for (PointMass &pm : point_masses) {
        self_collide(pm, simulation_steps);
    }

  // TODO (Part 3): Handle collisions with other primitives.
    for (PointMass &pm : point_masses) {
        for (CollisionObject *co : *collision_objects) {
            co->collide(pm);
        }
    }

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
    for (vector<Spring>::iterator spr = springs.begin(); spr < springs.end(); spr++) {
        if ((spr->spring_type == BENDING && cp->enable_bending_constraints)
            || (spr->spring_type == SHEARING && cp->enable_shearing_constraints)
            || (spr->spring_type == STRUCTURAL && cp->enable_structural_constraints)) {
            Vector3D vec = spr->pm_b->position - spr->pm_a->position;
            Vector3D correction = vec;
            if (vec.norm() > spr->rest_length * 1.1) {
                vec.normalize();
                vec *= spr->rest_length * 1.1;
            }
            correction = correction - vec;

            double num_unpinned = 0;
            num_unpinned += spr->pm_a->pinned ? 0 : 1;
            num_unpinned += spr->pm_b->pinned ? 0 : 1;

            if (num_unpinned > 0) {
                if (!spr->pm_a->pinned) {
                    spr->pm_a->position += correction / num_unpinned;
                }
                if (!spr->pm_b->pinned) {
                    spr->pm_b->position -= correction / num_unpinned;
                }
            }
        }
    }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
    int i = 0;
    for (PointMass &pm : point_masses) {
//        if (pm.position.x != NAN && pm.position.y != NAN && pm.position.z != NAN) {
        float pos = hash_position(pm.position);
        if (map.count(pos)==0) {
//            vector<PointMass *> *new_hash_pms = new vector<PointMass *>();
            map[pos] = new vector<PointMass *>();
        }
//        i = i + 1;
//        cout << i << "\n";
        map[pos]->push_back(&pm);
//        }
    }
    
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
    float pos = hash_position(pm.position);
    
    if (map.count(pos) == 0) {
        return;
    }
    
    Vector3D correction_vector;
    float num = 0;
    for (PointMass *point : *(map[pos])) {
        Vector3D direction = pm.position - point->position;
        if (abs(direction.norm()) < 2 * thickness) {
            if (point != &pm) {
                correction_vector += direction.unit() * (2 * thickness - (direction.norm()));
                num++;
            }
        }
    }
    if (num != 0 && simulation_steps != 0) {
        Vector3D avg_vector = correction_vector / (num);
        pm.position += avg_vector / simulation_steps;
    }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
    
    float w, h, t;
    
    if (num_width_points != 0) {
        w = 3.0 * width / (float)num_width_points;
    }
    if (num_height_points != 0) {
        h = 3.0 * height / (float)num_height_points;
    }
    t = max(w, h);
    float hash, x, y, z;
    
    if (orientation == 1) {
//        x = w != 0 ? ((int) (pos.x / w)) - fmod(pos.x,w) : 0;
//        y = h != 0 ? ((int) (pos.y / h)) - fmod(pos.y,h) : 0;
//        z = t != 0 ? ((int) (pos.z / t)) - fmod(pos.z,t) : 0;
        x = floor(pos.x / w);
        y = floor(pos.y / h);
        z = floor(pos.z / t);
        hash = y * width + x + z * width * height;
    } else if (orientation == 0) {
//        x = w != 0 ? ((int) (pos.x / w)) - fmod(pos.x,w) : 0;
//        y = t != 0 ? ((int) (pos.y / t)) - fmod(pos.y,t) : 0;
//        z = h != 0 ? ((int) (pos.z / h)) - fmod(pos.z,h) : 0;
        x = floor(pos.x / w);
        y = floor(pos.z / h);
        z = floor(pos.y / t);
        hash = y * width + x + z * width * height;
    }
//    cout << "dim (w, h, t):" << w << ", " << h << ", "<< t <<"\n";
//    cout << "coords:" << pos.x << ", " << pos.y << ", "<< pos.z <<"\n";

//    float x = (pos.x - fmod(pos.x, w)) / w;
//    float y = (pos.y - fmod(pos.y, h)) / h;
//    float z = (pos.z - fmod(pos.z, t)) / t;


//    float hash = (fmod(pos.y,h) * 3.0 * width + fmod(pos.x, w)) + fmod(pos.z,t) * 9.0 * width * height;
//
//    return pow(x, 5) + pow(y, 7) + pow(z, 11);
    return hash;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
