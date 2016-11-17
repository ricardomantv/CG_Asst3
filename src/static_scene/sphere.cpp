#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CMU462 { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // TODO:
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  double od = dot(r.o - o, r.d);
  double o_len = (r.o - o).norm();

  double d = pow(od, 2) - pow(o_len, 2) + r2;
  if(d < 0) {
    // Discriminant < 0, no intersection
    return false;
  }

  double root = sqrt(d);
  double t_minus = -1 * od - root;
  double t_plus = -1 * od + root;

  if((t_minus < r.min_t && t_plus < r.min_t) || (r.max_t < t_minus && r.max_t < t_plus)) {
    // Intersection points outside of valid t range
    return false;
  }

  if(t_minus < t_plus) {
    t1 = t_minus;
    t2 = t_plus;
  } else {
    t1 = t_plus;
    t2 = t_minus;
  }

  return true;

}

bool Sphere::intersect(const Ray& r) const {

  // TODO:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  Intersection isect;
  return intersect(r, &isect);

}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // TODO:
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t1;
  double t2;

  bool intersect = test(r, t1, t2);
  r.max_t = t2;
  if(intersect && t1 < i->t) {
    Vector3D n = normal(r.at_time(t1));
    if(dot(n, r.d) > 0) {
      n *= -1;
    }

    i->t = t1;
    i->primitive = this;
    i->n = n;
    i->bsdf = get_bsdf();
  }

  return intersect;

}

void Sphere::draw(const Color& c) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CMU462
