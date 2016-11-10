#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CMU462 {

  bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

    // TODO:
    // Implement ray - bounding box intersection test
    // If the ray intersected the bouding box within the range given by
    // t0, t1, update t0 and t1 with the new intersection times.

    Vector3D bounds[2] = {min, max};

    double x_min = (bounds[r.sign[0]].x - r.o.x) * r.inv_d.x;
    double x_max = (bounds[1 - r.sign[0]].x - r.o.x) * r.inv_d.x;
    double y_min = (bounds[r.sign[1]].y - r.o.y) * r.inv_d.y;
    double y_max = (bounds[1 - r.sign[1]].y - r.o.y) * r.inv_d.y;
    double z_min = (bounds[r.sign[2]].z - r.o.z) * r.inv_d.z;
    double z_max = (bounds[1 - r.sign[2]].z - r.o.z) * r.inv_d.z;

    double mins[4] = {t0, x_min, y_min, z_min};
    double maxs[4] = {t1, x_max, y_max, z_max};

    double t_min = *std::max_element(mins, mins + 4);
    double t_max = *std::min_element(maxs, maxs + 4);

    // std::cout << "t_min = " << t_min << ", t_max = " << t_max << "\n";

    if(t_min <= t_max) {
      t0 = t_min;
      t1 = t_max;
      return true;
    }

    return false;

  }

  void BBox::draw(Color c) const {

    glColor4f(c.r, c.g, c.b, c.a);

    // top
    glBegin(GL_LINE_STRIP);
    glVertex3d(max.x, max.y, max.z);
    glVertex3d(max.x, max.y, min.z);
    glVertex3d(min.x, max.y, min.z);
    glVertex3d(min.x, max.y, max.z);
    glVertex3d(max.x, max.y, max.z);
    glEnd();

    // bottom
    glBegin(GL_LINE_STRIP);
    glVertex3d(min.x, min.y, min.z);
    glVertex3d(min.x, min.y, max.z);
    glVertex3d(max.x, min.y, max.z);
    glVertex3d(max.x, min.y, min.z);
    glVertex3d(min.x, min.y, min.z);
    glEnd();

    // side
    glBegin(GL_LINES);
    glVertex3d(max.x, max.y, max.z);
    glVertex3d(max.x, min.y, max.z);
    glVertex3d(max.x, max.y, min.z);
    glVertex3d(max.x, min.y, min.z);
    glVertex3d(min.x, max.y, min.z);
    glVertex3d(min.x, min.y, min.z);
    glVertex3d(min.x, max.y, max.z);
    glVertex3d(min.x, min.y, max.z);
    glEnd();

  }

  std::ostream& operator<<(std::ostream& os, const BBox& b) {
    return os << "BBOX(" << b.min << ", " << b.max << ")";
  }

} // namespace CMU462
