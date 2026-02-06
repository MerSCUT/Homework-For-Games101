//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


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
    float p = get_random_float() * emit_area_sum;       // ?? p 是发光光源面积的随机比例
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
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


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    // ray : 视线
    Vector3f L_dir = 0.0f;
    // direct

    Intersection inter = intersect(ray);        // ray intersection
    if (inter.happened == false)
    {
        return Vector3f(0.f); // No color
    }

    //------------------------------------z
    if (inter.m->hasEmission())
    {
        if (depth == 0) return inter.emit;
        //L_dir = inter.emit;
    }
    //------------------------------------
    
    //------------------------------------
    
    // Direct Light
    auto p = inter.coords;
    auto w_o = normalize(ray.origin - p);
    float pdf_light;
    Intersection light_sample;
    sampleLight(light_sample, pdf_light);
    
    auto x = light_sample.coords;
    auto nn = light_sample.normal;
    auto emit = light_sample.emit;
    // light position and normal

    auto w_s = normalize(x - p);        // !! outwards
    // Then, use render equation
    auto tem = intersect(Ray(p + EPSILON * inter.normal, w_s));
    //auto tem = intersect(Ray(p, w_s));
    if (tem.distance + 0.01f  >= (x - p).norm())
        L_dir += emit * inter.m->eval(w_s, w_o, inter.normal)
        * dotProduct(w_s, inter.normal) * dotProduct(-w_s, nn) / dotProduct(x - p, x - p) / std::max(pdf_light, 0.0000001f);
    // one sample light.

    //----------------------------------------
    //----------------------------------------

    // Indirect Light

    Vector3f L_indir = 0.f;

    //if (depth >= 5) return L_dir + L_indir;
    // Russian Roulette
    if (get_random_float() < RussianRoulette)
    {
        // sampling a direction :
        auto w_i = inter.m->sample(w_o, inter.normal);
        if (dotProduct(w_i, inter.normal) < 0) w_i = -w_i;      // keep outwards
        auto ind = intersect(Ray(p + EPSILON * inter.normal, w_i));                  // indirect intersection
        
        auto q = ind.coords;
        if (ind.happened && !ind.m->hasEmission())
        {
            // No emission material
            // take w_i as output
            L_indir = castRay(Ray(p + EPSILON * inter.normal, w_i), depth + 1) * inter.m->eval(w_i, w_o, inter.normal)
                * dotProduct(w_i, inter.normal) / std::max(inter.m->pdf(w_o, w_i, inter.normal), 0.0000001f) / RussianRoulette;
        }
    }
    return L_dir + L_indir;
}