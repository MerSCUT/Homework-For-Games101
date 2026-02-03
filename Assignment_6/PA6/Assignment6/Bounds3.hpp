//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();      // 取double的最小值
        double maxNum = std::numeric_limits<double>::max();         // 取double的最大值
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}                 // 常数化初始化
    Bounds3(const Vector3f p1, const Vector3f p2)                   // (最小点, 最大点)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }               // 对角向量 (最小指最大)
    int maxExtent() const                               // ?
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)     // 对角向量的 哪个坐标最大. 返回索引
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const                              // 包围盒表面面积?
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);     // 
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }     // 包围盒中心
    Bounds3 Intersect(const Bounds3& b)                         // a. Intersect(b) 计算 a 与 b 相交区域 (也是个包围盒)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)             // b1 与 b2 是否相交
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);     // p 是否在包围盒 b 中
    }
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;      // bvh[0] 指最小点, 其他都是最大点
    }

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};



inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects

    // 已知是 Bounds3 的成员函数, 传参都是光线的属性.
    
    
    auto tmin = (pMin - ray.origin) * invDir;
    auto tmax = (pMax - ray.origin) * invDir;
    
    if (!dirIsNeg[0]) std::swap(tmin.x, tmax.x);
    if (!dirIsNeg[1]) std::swap(tmin.y, tmax.y);
    if (!dirIsNeg[2]) std::swap(tmin.z, tmax.z);
    // 取tmin 中最大的和tmax 中最小的
    auto t_in = std::max(tmin.x, std::max(tmin.y, tmin.z));

    auto t_out = std::min(tmax.x, std::min(tmax.y, tmax.z));
    
    if (t_out > t_in && t_out > 0) return true;
    return false;

}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
