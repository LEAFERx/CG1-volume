#pragma once
#include "volumeData.h"
#include "Eigen/Dense"



//ref https://en.wikipedia.org/wiki/Trilinear_interpolation
class Interpolator {
public:
  virtual volumeData interpolate(Eigen::Vector3f& p, const voxel& voxel) = 0;
};

#define NN_COMPARE(cxxx) \
  if ((p - currPos).norm() > (p - voxel.cxxx.position).norm()) { \
    currPos = voxel.cxxx.position;                               \
    v.density = voxel.cxxx.density;                              \
    v.gradient = voxel.cxxx.gradient;                            \
  }

class NearestNeighbourInterpolator : public Interpolator {
public:
  inline volumeData interpolate(Eigen::Vector3f& p, const voxel& voxel) {
    //Here your NNInterpolator code
    volumeData v(p.x(), p.y(), p.z(), voxel.c000.density, voxel.c000.gradient);
    Eigen::Vector3f currPos(voxel.c000.position);
    NN_COMPARE(c001);
    NN_COMPARE(c010);
    NN_COMPARE(c011);
    NN_COMPARE(c100);
    NN_COMPARE(c101);
    NN_COMPARE(c110);
    NN_COMPARE(c111);

    return v;
  };
};

class TrilinearInterpolator : public Interpolator {
public:
  inline volumeData interpolate(Eigen::Vector3f& p, const voxel& voxel) {
    //Here your TrilinearInterpolator code
    float xd = (p.x() - voxel.c000.position.x()) / (voxel.c100.position.x() - voxel.c000.position.x());
    float yd = (p.y() - voxel.c000.position.y()) / (voxel.c010.position.y() - voxel.c000.position.y());
    float zd = (p.z() - voxel.c000.position.z()) / (voxel.c001.position.z() - voxel.c000.position.z());

    Eigen::Vector3f g00 = voxel.c000.gradient * (1 - xd) + voxel.c100.gradient * xd;
    Eigen::Vector3f g01 = voxel.c001.gradient * (1 - xd) + voxel.c101.gradient * xd;
    Eigen::Vector3f g10 = voxel.c010.gradient * (1 - xd) + voxel.c110.gradient * xd;
    Eigen::Vector3f g11 = voxel.c011.gradient * (1 - xd) + voxel.c111.gradient * xd;

    Eigen::Vector3f g0 = g00 * (1 - yd) + g10 * yd;
    Eigen::Vector3f g1 = g01 * (1 - yd) + g11 * yd;

    Eigen::Vector3f g = g0 * (1 - zd) + g1 * zd;

    float d00 = voxel.c000.density * (1 - xd) + voxel.c100.density * xd;
    float d01 = voxel.c001.density * (1 - xd) + voxel.c101.density * xd;
    float d10 = voxel.c010.density * (1 - xd) + voxel.c110.density * xd;
    float d11 = voxel.c011.density * (1 - xd) + voxel.c111.density * xd;
    
    float d0 = d00 * (1 - yd) + d10 * yd;
    float d1 = d01 * (1 - yd) + d11 * yd;
    
    float d = d0 * (1 - zd) + d1 * zd;

    return volumeData(p.x(), p.y(), p.z(), d, g);
  };
};