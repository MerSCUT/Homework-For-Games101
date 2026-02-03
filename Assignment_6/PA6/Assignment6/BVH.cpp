#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());

    switch (splitMethod)
    {
    case SplitMethod::NAIVE:
        if (objects.size() == 1) {
            // Create leaf _BVHBuildNode_
            node->bounds = objects[0]->getBounds();
            node->object = objects[0];
            node->left = nullptr;
            node->right = nullptr;
            return node;
        }
        else if (objects.size() == 2) {
            // 两个物体
            node->left = recursiveBuild(std::vector{ objects[0] });
            node->right = recursiveBuild(std::vector{ objects[1] });

            node->bounds = Union(node->left->bounds, node->right->bounds);
            return node;
        }
        else {
            Bounds3 centroidBounds;
            for (int i = 0; i < objects.size(); ++i)
                centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
            int dim = centroidBounds.maxExtent();
            switch (dim) {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                        f2->getBounds().Centroid().x;
                    });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                        f2->getBounds().Centroid().y;
                    });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                        f2->getBounds().Centroid().z;
                    });
                break;
            }

            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);     // 中点划分
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        break;
    case SplitMethod::SAH:
        struct Bucket
        {
            int tri_num; // 桶中的图元个数
            Bounds3 bound;  // 桶中图元的包围盒 (包围盒并运算有交换结合律)
            std::vector<Object*> object_list;

            Bucket() : tri_num(0) {}
        };
        if (objects.size() == 1) {
            // Create leaf _BVHBuildNode_
            node->bounds = objects[0]->getBounds();
            node->object = objects[0];
            node->left = nullptr;
            node->right = nullptr;
            return node;
        }
        const int n = 12;
        const float isect = 1;
        std::array<Bucket, n> bucks;
        float Vector3f::* p;
        switch (bounds.maxExtent())
        {
        case 0: p = &Vector3f::x; break;
        case 1: p = &Vector3f::y; break;
        case 2: p = &Vector3f::z; break;
        default:
            p = &Vector3f::x; break;
        }
        float Ileft = bounds.pMin.*p, Iright = bounds.pMax.*p;
        for (auto& obj : objects)
        {
            auto Imid = obj->getBounds().Centroid().*p;
            // 比较
            int i = std::floor((Imid - Ileft) / (Iright - Ileft) * n );
            if (!(bucks[i].tri_num++)) { bucks[i].bound = obj->getBounds(); }
            else { bucks[i].bound = Union(bucks[i].bound, obj->getBounds()); }
            bucks[i].object_list.emplace_back(obj);
        }
        // 此时每个桶里的代价都是可以计算的.
        float C = std::numeric_limits<float>::max();
        int s_best = 0;
        for (int s = 1; s < n; s++)
        {
            auto split = Ileft + s * (Iright - Ileft);
            // 第 s 个桶内的代价记为右侧
            auto cL = 0;
            auto cR = 0;

            int nL = 0, nR = 0;
            Bounds3 bL = bucks[0].bound;
            Bounds3 bR = bucks[n - 1].bound;
            for (int j = 0; j < s; j++)
            {
                nL += bucks[j].tri_num;
                bL = Union(bL, bucks[j].bound);
            }
            if (nL == 0) continue;
            cL +=  nL * isect * bL.SurfaceArea() / bounds.SurfaceArea();
            for (int j = s; j < n; j++)
            {
                nR += bucks[j].tri_num;
                bR = Union(bR, bucks[j].bound);
            }
            cR +=  nR * isect * bR.SurfaceArea() / bounds.SurfaceArea();
            if (nR == 0) continue;
            
            assert(nL != 0 && nR != 0);

            auto temp = 0.125 + cL + cR;
            if (temp < C)
            {
                C = temp; s_best = s;
            }
        }
        
        std::vector<Object*> left_list, right_list;
        // 确定以 s 为分划
        for (int i = 0; i < n; i++)
        {
            if (i < s_best)
                left_list.insert(left_list.end(), bucks[i].object_list.begin(), bucks[i].object_list.end());
            else
                right_list.insert(right_list.end(), bucks[i].object_list.begin(), bucks[i].object_list.end());
        }
        assert(left_list.size() + right_list.size() == objects.size());
        
        if (left_list.size() == 0 )
        {
            node->left = node->right = nullptr;
            node->object_list = right_list;
            node->bounds = bounds;
            return node;
        }
        if (right_list.size() == 0)
        {
            node->left = node->right = nullptr;
            node->object_list = left_list;
            node->bounds = bounds;
            return node;
        }
        assert(left_list.size() != 0 && right_list.size() != 0);
        node->left = recursiveBuild(left_list);
        node->right = recursiveBuild(right_list);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        
        
    }
    

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection

    // 一条 ray 打进来, 与 node 这个 BVH 树 根结点操作, 获取ray与物体相交的信息. 
    Intersection inter;

    if (node == nullptr) return inter;

    if (!node->bounds.IntersectP(ray, ray.direction_inv,
        std::array<int, 3>{int(ray.direction.x > 0), int(ray.direction.y > 0), int(ray.direction.z > 0)}))
    {
        // 若当前结点与 bounds 不相交 : 没这个结点的事情了
        return inter;
    }
    // 若相交 :

    // 是叶子结点 -> 一个叶子结点只对应一个object
    if (node->left == nullptr && node->right == nullptr)
    {
        assert(node->object != nullptr || node->object_list.size() != 0);
        if (node->object != nullptr)
        return node->object->getIntersection(ray);
        
        Intersection isec;
        for (auto& obj : node->object_list)
        {
            auto temp= obj->getIntersection(ray);
            if (temp.distance < isec.distance)
            {
                isec = temp;
            }
        }
        return isec;
    }

    // 不是叶子结点
    auto interL = getIntersection(node->left, ray);   // 与左包围盒有交点信息
    auto interR = getIntersection(node->right, ray);

    if (interL.distance < interR.distance)
    {
        inter = interL;
    }
    else
    {
        inter = interR;
    }
    

    return inter;
}