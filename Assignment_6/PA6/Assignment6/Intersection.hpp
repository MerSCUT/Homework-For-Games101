//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

struct Intersection
{
    Intersection(){
        happened=false;         // 相交发生
        coords=Vector3f();      // 相交点坐标
        normal=Vector3f();      // 相交点法向量
        distance= std::numeric_limits<double>::max();   // 相交点距离光线的距离
        obj =nullptr;           // 相交物体
        m=nullptr;              // 相交物体的材质
    }
    bool happened;
    Vector3f coords;
    Vector3f normal;
    double distance;
    Object* obj;
    Material* m;
};
#endif //RAYTRACING_INTERSECTION_H
