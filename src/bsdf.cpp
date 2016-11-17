#include "bsdf.h"

#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CMU462 {

  void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {

    Vector3D z = Vector3D(n.x, n.y, n.z);
    Vector3D h = z;
    if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z)) h.x = 1.0;
    else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z)) h.y = 1.0;
    else h.z = 1.0;

    z.normalize();
    Vector3D y = cross(h, z);
    y.normalize();
    Vector3D x = cross(z, y);
    x.normalize();

    o2w[0] = x;
    o2w[1] = y;
    o2w[2] = z;
  }

  // Diffuse BSDF //

  Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    return albedo * (1.0 / PI);
  }

  Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
    *wi = sampler.get_sample(pdf);
    return f(wo, *wi);
  }

  // Mirror BSDF //

  Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    if(wo.x == wi.x * -1 && wo.y == wi.y * -1 && wo.z == wi.z) {
      return (1.0 / cos_theta(wo)) * reflectance;
    }

    return Spectrum();
  }

  Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

    // TODO:
    // Implement MirrorBSDF
    *pdf = 1.0f;
    reflect(wo, wi);
    return f(wo, *wi);
  }

  // Glossy BSDF //

  /*
     Spectrum GlossyBSDF::f(const Vector3D& wo, const Vector3D& wi) {
     return Spectrum();
     }

     Spectrum GlossyBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
   *pdf = 1.0f;
   return reflect(wo, wi, reflectance);
   }
   */

  // Refraction BSDF //

  Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    return Spectrum();
  }

  Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

    // TODO:
    // Implement RefractionBSDF
    *pdf = 1.0f;

    double cosi = cos_theta(wo);
    double ni;
    double nt;
    if(cosi > 0) {
      ni = 1.0;
      nt = ior;
    } else {
      ni = ior;
      nt = 1.0;
    }

    double ntni = nt / ni;

    if(refract(wo, wi, ior)) {
      return (pow(ntni, 2) / std::abs(cosi)) * transmittance;
    } else {
      return Spectrum();
    }
  }

  // Glass BSDF //

  Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    return Spectrum();
  }

  Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

    // TODO:
    // Compute Fresnel coefficient and either reflect or refract based on it.

    // Total internal reflection
    if(!refract(wo, wi, ior)) {
      *pdf = 1.0f;
      reflect(wo, wi);
      return reflectance;
    }

    double cosi = cos_theta(wo);
    double cost = cos_theta(*wi); // wi already calculated

    double ni;
    double nt;
    if(cosi > 0) {
      ni = 1.0;
      nt = ior;
    } else {
      ni = ior;
      nt = 1.0;
    }

    double nint = ni / nt;

    double niCosi = ni * cosi;
    double niCost = ni * cost;
    double ntCosi = nt * cosi;
    double ntCost = nt * cost;
    double r_par = (ntCosi - niCost) / (ntCosi + niCost);
    double r_perp = (niCosi - ntCost) / (niCosi + ntCost);

    double fresnel = (0.5) * (pow(r_par, 2) + pow(r_perp, 2));

    double random = (double)(std::rand()) / RAND_MAX;
    if(random < fresnel) {
      *pdf = fresnel;
      reflect(wo, wi);
      return fresnel * (1.0 / cosi) * reflectance;
    } else {
      *pdf = 1.0 - fresnel;
      return (1.0 - fresnel) * (pow(1.0 / nint, 2) / std::abs(cosi)) * transmittance;
    }

  }

  void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {

    // TODO:
    // Implement reflection of wo about normal (0,0,1) and store result in wi.
    *wi = Vector3D(-1 * wo.x, -1 * wo.y, wo.z);
  }

  bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {

    // TODO:
    // Use Snell's Law to refract wo surface and store result ray in wi.
    // Return false if refraction does not occur due to total internal reflection
    // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
    // ray entering the surface through vacuum.

    double cosi = cos_theta(wo);
    double ni;
    double nt;
    if(cosi > 0) {
      ni = 1.0f;
      nt = ior;
    } else {
      ni = ior;
      nt = 1.0f;
    }

    double sin2i = sin_theta2(wo);
    double nint = ni / nt;
    double sin2t = pow(nint, 2) * sin2i;

    if(1.0 - sin2t < 0.0) {
      // Internal reflection
      return false;
    }

    double cost = sqrt(1.0 - sin2t);
    *wi = Vector3D(-1 * nint * wo.x, -1 * nint * wo.y, cost);

    return true;

  }

  // Emission BSDF //

  Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    return Spectrum();
  }

  Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
    *wi  = sampler.get_sample(pdf);
    return Spectrum();
  }

} // namespace CMU462
