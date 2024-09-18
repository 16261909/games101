//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <cassert>

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;//[0~1]*13650    
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){//random get the first area > p light,return                
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}



bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject) const
{
  printf("!");
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }
    printf("@");

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Intersection inter = intersect(ray);
    if (!inter.happened)
    {
        return Vector3f(0);
    }
    if (inter.m->hasEmission())
    {
        return inter.m->getEmission();
    }
    Vector3f L_dir = 0.f, L_indir = 0.f;
    float pdf;
    Vector3f p = inter.coords;
    Vector3f n = inter.normal;
    Vector3f wo = -ray.direction;
    Material* m = inter.m;
    sampleLight(inter, pdf);
    Vector3f x = inter.coords;
    Vector3f nn = inter.normal;
    Vector3f emit = inter.emit;
    Vector3f ws = normalize(p - x);
    Intersection inter2 = intersect(Ray(x, ws));
    float eps = 1e-2;
    if (fabs(inter2.distance - (x - p).norm()) < eps)
    {
        L_dir = emit * m->eval(-ws, wo, n) * std::max(dotProduct(nn, ws), 0.f) * std::max(dotProduct(-ws, n), 0.f) / ((p - x).norm() * (p - x).norm()) / pdf;
    }
    else
    {
        if (get_random_float() < RussianRoulette)
        {
            Vector3f wi = m->sample(wo, n).normalized();
            Ray ray2 = Ray(p, wi);
            Intersection inter3 = intersect(ray2);
            float pdf1 = m->pdf(wi, wo, n);
            if (inter3.happened && !inter3.m->hasEmission())
            {
                L_indir = castRay(ray2, depth + 1) * m->eval(wo, wi, n) * std::max(dotProduct(n, wi), 0.f) / pdf1 / RussianRoulette;
            }
        }
    }

    Vector3f L = L_dir + L_indir;
    L.x = clamp(L.x, 0.f, 1.f);
    L.y = clamp(L.y, 0.f, 1.f);
    L.z = clamp(L.z, 0.f, 1.f);

    return L;
}