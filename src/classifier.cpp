#include "classifier.h"
#include <cmath>

opticsData myClassifier::transfer(volumeData v_data, float dt, Camera* cam, std::vector<Light*> lights, float grad_max_norm) {
  opticsData optics;
	// Write your own classifier, make use of input args(may not all)
	// should set transparency and color of optics
  tinycolormap::Color mapped_color = tinycolormap::GetJetColor(v_data.density);
  Eigen::Vector3f color(mapped_color.r(), mapped_color.g(), mapped_color.b());

  // Phong lighting
  Eigen::Vector3f normal = v_data.gradient.normalized();
  if (normal.norm() < 0.01f) {
    normal = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
  }
  // Ambient
  Eigen::Vector3f ambient = Eigen::Vector3f::Ones() * 0.05f;
  // Diffuse & Specular
  Eigen::Vector3f viewDirection = (cam->m_Pos - v_data.position).normalized();
  Eigen::Vector3f specColor = Eigen::Vector3f::Ones();
  Eigen::Vector3f diffuse = Eigen::Vector3f::Zero();
  Eigen::Vector3f specular = Eigen::Vector3f::Zero();
  for (auto light : lights) {
    Eigen::Vector3f lightDirection = (light->m_Pos - v_data.position).normalized();
    float diff = std::max(lightDirection.dot(normal), 0.0f);
    diffuse += diff * color.cwiseProduct(light->m_Color);
    // diffuse += ((normal + Eigen::Vector3f::Ones()) / 2);
    //if (v_data.gradient.norm() > 1.0f) {
    //  diffuse += Eigen::Vector3f::Ones();
    //}

    Eigen::Vector3f reflectDirection = (2 * normal.dot(lightDirection) * normal - lightDirection).normalized();
    float spec = std::pow(std::max(reflectDirection.dot(viewDirection), 0.0f), 32.0f);
    specular += spec * specColor;
  }

  float relative_gradient_norm = v_data.gradient.norm() / grad_max_norm;
  optics.color = 5 * (ambient + diffuse + specular) * dt * v_data.density * (1.0f + 0.05f * relative_gradient_norm);
  optics.transparency = Eigen::Vector3f::Ones() * std::exp(-v_data.density * dt) * (1.0f + 0.05f * relative_gradient_norm);
  return optics;
}