#include "triangle.h"

#include "CMU462/CMU462.h"
#include "GL/glew.h"

namespace CMU462 { namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, vector<size_t>& v) :
    mesh(mesh), v(v) { }
Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) :
    mesh(mesh), v1(v1), v2(v2), v3(v3) { }

BBox Triangle::get_bbox() const {

  // TODO:
  // compute the bounding box of the triangle
  Vector3D p0 = mesh->positions[v1];
  Vector3D p1 = mesh->positions[v2];
  Vector3D p2 = mesh->positions[v3];

  double min_x = (p0.x < p1.x) ? ((p0.x < p2.x) ? p0.x : p2.x) : ((p1.x < p2.x) ? p1.x : p2.x);
  double min_y = (p0.y < p1.y) ? ((p0.y < p2.y) ? p0.y : p2.y) : ((p1.y < p2.y) ? p1.y : p2.y);
  double min_z = (p0.z < p1.z) ? ((p0.z < p2.z) ? p0.z : p2.z) : ((p1.z < p2.z) ? p1.z : p2.z);

  double max_x = (p0.x > p1.x) ? ((p0.x > p2.x) ? p0.x : p2.x) : ((p1.x > p2.x) ? p1.x : p2.x);
  double max_y = (p0.y > p1.y) ? ((p0.y > p2.y) ? p0.y : p2.y) : ((p1.y > p2.y) ? p1.y : p2.y);
  double max_z = (p0.z > p1.z) ? ((p0.z > p2.z) ? p0.z : p2.z) : ((p1.z > p2.z) ? p1.z : p2.z);

  Vector3D min_V = Vector3D(min_x, min_y, min_z);
  Vector3D max_V = Vector3D(max_x, max_y, max_z);

  return BBox(min_V, max_V);
}

bool Triangle::intersect(const Ray& r) const {

  // TODO: implement ray-triangle intersection
  Intersection isect;
  return intersect(r, &isect);
}

bool Triangle::intersect(const Ray& r, Intersection *isect) const {

  // TODO:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

  // Calculate if ray intersects triangle
  Vector3D p0 = mesh->positions[v1];
  Vector3D p1 = mesh->positions[v2];
  Vector3D p2 = mesh->positions[v3];

  Vector3D e1 = p1 - p0;
  Vector3D e2 = p2 - p0;
  Vector3D s = r.o - p0;

  Vector3D e1d = cross(e1, r.d);
  Vector3D se2 = -1 * cross(s, e2);
  double coef = 1 / (dot(e1d, e2));
  Vector3D cramer = Vector3D(dot(se2, r.d), dot(e1d, s), dot(se2, e1));

  Vector3D uvt = coef * cramer;
  double u = uvt.x;
  double v = uvt.y;
  double t = uvt.z;

  if(t < r.min_t || r.max_t < t) {
    // Don't bother caclulating barycentric if not within t bounds
    return false;
  }
  if(u < 0 || 1 < u) {
    return false;
  }
  if(v < 0 || 1 < v) {
    return false;
  }
  if((u + v) < 0 || 1 < (u + v)) {
    return false;
  }

  r.max_t = t;

  // Get normals of 3 points and find overall normal
  Vector3D n0 = mesh->normals[v1];
  Vector3D n1 = mesh->normals[v2];
  Vector3D n2 = mesh->normals[v3];
  Vector3D n = (1 - u - v) * n0 + u * n1 + v * n2;
  if(dot(n, r.d) > 0) {
    n *= -1;
  }

  // Assign Intersection struct values
  if(t < isect->t) {
    isect->t = t;
    isect->primitive = this;
    isect->n = n;
    isect->bsdf = mesh->get_bsdf();
  }

  return true;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}



} // namespace StaticScene
} // namespace CMU462
