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

  return BBox();
}

bool Triangle::intersect(const Ray& r) const {

  // TODO: implement ray-triangle intersection
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

  Vector2D A = Vector2D(0, 0);
  Vector2D B = Vector2D(0, 1);
  Vector2D C = Vector2D(1, 0);
  Vector2D P = Vector2D(u, v);

  Vector2D ba = B - A;
  Vector2D ca = C - A;
  Vector2D bp = B - P;
  Vector2D cp = C - P;
  Vector2D ap = A - P;

  double areaABC = cross(ba, ca) / 2;
  double areaPAB = cross(ap, bp) / 2;
  double areaPCA = cross(cp, ap) / 2;
  double areaPBC = cross(bp, cp) / 2;

  Vector3D bary = Vector3D(areaPBC, areaPCA, areaPAB) / areaABC;

  return bary.x > 0 && bary.y > 0 && bary.z > 0;
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
  
  Vector2D A = Vector2D(0, 0);
  Vector2D B = Vector2D(1, 0);
  Vector2D C = Vector2D(0, 1);
  Vector2D P = Vector2D(u, v);

  Vector2D ba = B - A;
  Vector2D ca = C - A;
  Vector2D bp = B - P;
  Vector2D cp = C - P;
  Vector2D ap = A - P;

  double areaABC = cross(ba, ca) / 2;
  double areaPAB = cross(ap, bp) / 2;
  double areaPCA = cross(cp, ap) / 2;
  double areaPBC = cross(bp, cp) / 2;

  Vector3D bary = Vector3D(areaPBC, areaPCA, areaPAB) / areaABC;

  bool intersect = bary.x > 0 && bary.y > 0 && bary.z > 0;

  // bool intersect = 0 <= (u + v) && (u + v) <= 1;

  if(intersect) {
    // Get normals of 3 points
    Vector3D n0 = mesh->normals[v1];
    Vector3D n1 = mesh->normals[v2];
    Vector3D n2 = mesh->normals[v3];

    // Assign Intersection struct values
    isect->t = t;
    isect->primitive = this;
    isect->n = (1 - u - v) * n0 + u * n1 + v * n2;
    isect->bsdf = mesh->get_bsdf();
  }


  // cout << "u = " << u << ", v = " << v << ", intersect = " << intersect << "\n";
  // cout << "t = " << t << ", min = " << r.min_t << ", max = " << r.max_t << "\n";

  return intersect;
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
