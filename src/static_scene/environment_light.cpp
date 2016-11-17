#include "environment_light.h"
#include <iostream>

namespace CMU462 { namespace StaticScene {

EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
    : envMap(envMap) {
  // TODO: initialize things here as needed
  w2o[0] = Vector3D(0, 0, 1);
  w2o[1] = Vector3D(0, 1, 0);
  w2o[2] = Vector3D(-1, 0, 0);
  o2w = w2o.T();
}

Spectrum EnvironmentLight::sample_L(const Vector3D& p, Vector3D* wi,
                                    float* distToLight,
                                    float* pdf) const {
  // TODO: Implement

  // Generate random theta and pi with uniform sphere sampling
  double Xi1 = (((double)(std::rand()) / RAND_MAX) * 2) - 1; // random cos_theta from -1 to 1
  double Xi2 = (double)(std::rand()) / RAND_MAX; // random double from 0 to 1 for phi

  double theta = acos(Xi1);
  double phi = 2.0 * PI * Xi2;

  double xs = sinf(theta) * cosf(phi);
  double ys = sinf(theta) * sinf(phi);
  double zs = cosf(theta);

  Vector3D rand_dir = Vector3D(xs, ys, zs);

  // Get color from environment map
  size_t w = envMap->w;
  size_t h = envMap->h;

  size_t row_o = (size_t)((theta / PI) * h);
  size_t col_o = (size_t)((phi / (2 * PI)) * w);

  size_t row_i = (row_o + 1) % h;
  size_t col_i = (col_o + 1) % w;

  Spectrum color = Spectrum();
  color += envMap->data[row_o * w + col_o];
  color += envMap->data[row_o * w + col_i];
  color += envMap->data[row_i * w + col_o];
  color += envMap->data[row_i * w + col_i];
  color *= (1.0 / 4.0);


  *wi = o2w * rand_dir;
  *distToLight = INF_F;
  *pdf = 1.0f / (4.0f * PI);

  return color;
}

Spectrum EnvironmentLight::sample_dir(const Ray& r) const {
  // TODO: Implement

  Vector3D world_ray = (w2o * r.d).unit();

  size_t w = envMap->w;
  size_t h = envMap->h;

  double u = atan2(world_ray.z, world_ray.x) / (2 * PI) + 0.5;
  double v = acos(world_ray.y) / PI;

  size_t row_o = floor(v * h);
  size_t col_o = floor(u * w);

  size_t row_i = (row_o + 1) % h;
  size_t col_i = (col_o + 1) % w;


  Spectrum color = Spectrum();
  color += envMap->data[row_o * w + col_o];
  color += envMap->data[row_o * w + col_i];
  color += envMap->data[row_i * w + col_o];
  color += envMap->data[row_i * w + col_i];

  color *= (1.0 / 4.0);

  return color;
}

} // namespace StaticScene
} // namespace CMU462
