#include "renderer.h"

void Renderer::setCamera(Camera* cam) { this->camera = cam; };
void Renderer::addLight(Light* light) { this->lights.push_back(light); };
void Renderer::setVolume(Volume* vol) { this->volume = vol; };
void Renderer::setClassifier(Classifier* classifier) { this->classifier = classifier; };
void Renderer::setInterpolator(Interpolator* interpolator) { this->interpolator = interpolator; };

void Renderer::renderFrontToBack(std::string imgPath) {
#pragma omp parallel for
  for (int i = 0; i < camera->m_Film.m_Res.x() * camera->m_Film.m_Res.y(); i++) {
    int dy = i / camera->m_Film.m_Res.x();
    int dx = i - dy * camera->m_Film.m_Res.x();
    Eigen::Vector3f color(0, 0, 0); Eigen::Vector3f alpha(0, 0, 0);
    Ray ray = camera->generateRay((float)dx, (float)dy);
    float t_start; float t_end;
    // Intersection calculate
    if (this->volume->getRayStartEnd(ray, t_start, t_end)) {
      // Here your render code
      // const float dt = (t_end - t_start) / 1000;
      const float dt = 0.005f;

      float t = t_start;

      while (t < t_end) {
        Eigen::Vector3f p = ray.getPoint(t);
        volumeData v_data = this->interpolator->interpolate(p, this->volume->getVoxel(p));
        opticsData o_data = this->classifier->transfer(v_data, dt, this->camera, this->lights, this->volume->grad_maxnorm);

        //Volumetric Shadow:
        //float transimittance = 1.0f;
        //const float l_dt = 0.05f;
        //Ray lightRay(p, (this->lights[0]->m_Pos - p).normalized()); // Assume only 1 light for simplicity
        //float l_t_start, l_t_end;
        //if (this->volume->getRayStartEnd(lightRay, l_t_start, l_t_end)) {
        //  l_t_end = std::min(l_t_end, (this->lights[0]->m_Pos - p).norm()); // Avoid go further than light
        //  float l_t = l_t_start;
        //  while (l_t < l_t_end) {
        //    Eigen::Vector3f l_p = lightRay.getPoint(l_t);
        //    volumeData l_v_data = this->interpolator->interpolate(l_p, this->volume->getVoxel(l_p));
        //    transimittance *= std::exp(- 5 * l_v_data.density * l_dt);
        //    l_t += l_dt;
        //  }
        //}
        //o_data.color *= transimittance;

        Compositor::compositeFrontToBack(color, alpha, o_data.getColor(), o_data.getOpacity());

        t += dt;
      }
    }
    camera->setPixel(dx, dy, color);
  }
  camera->m_Film.write(imgPath);
};

void Renderer::renderBackToFront(std::string imgPath) {
#pragma omp parallel for
  for (int i = 0; i < camera->m_Film.m_Res.x() * camera->m_Film.m_Res.y(); i++) {
    int dy = i / camera->m_Film.m_Res.x();
    int dx = i - dy * camera->m_Film.m_Res.x();
    Eigen::Vector3f color(0, 0, 0);
    Ray ray = camera->generateRay((float)dx, (float)dy);
    float t_start; float t_end;
    // Intersection calculate
    if (this->volume->getRayStartEnd(ray, t_start, t_end)) {
      const float dt = 0.005f;

      float t = t_end;

      while (t > t_start) {
        Eigen::Vector3f p = ray.getPoint(t);
        volumeData v_data = this->interpolator->interpolate(p, this->volume->getVoxel(p));
        opticsData o_data = this->classifier->transfer(v_data, dt, this->camera, this->lights, this->volume->grad_maxnorm);

        Compositor::compositeBackToFront(color, o_data.getColor(), o_data.getOpacity());

        t -= dt;
      }
    }
    camera->setPixel(dx, dy, color);
  }
  camera->m_Film.write(imgPath);
};