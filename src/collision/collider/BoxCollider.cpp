/*
 * BoxCollider.cpp
 */

#include "BoxCollider.h"
#include "GroundCollider.h"
#include "SphereCollider.h"

#include <algorithm>

namespace BulletPhysics {
namespace collision {
namespace collider {

static constexpr double CONTACT_MARGIN = 0.01;   // corners just above surface still count, else the box rocks
static constexpr double REST_TOLERANCE = 0.02;   // share of the box a resting face may be tilted by

static constexpr int MAX_CLIP_POINTS = 8;        // a face cut by four planes cannot give more
static constexpr int FEATURE_CLAMPED = 16;       // clamped point, no corner behind it

BoxCollider::BoxCollider(const math::Vec3& size) : m_size(size) {}

void BoxCollider::setAxes(const math::Vec3& axisX, const math::Vec3& axisY, const math::Vec3& axisZ)
{
    m_axes[0] = axisX;
    m_axes[1] = axisY;
    m_axes[2] = axisZ;
}

void BoxCollider::setOrientation(const math::Quat& orientation)
{
    m_axes[0] = orientation.rotate({1.0, 0.0, 0.0});
    m_axes[1] = orientation.rotate({0.0, 1.0, 0.0});
    m_axes[2] = orientation.rotate({0.0, 0.0, 1.0});
}

bool BoxCollider::testCollision(const Collider& other, CollisionInfo& outInfo) const
{
    switch (other.getShape()) {
        case CollisionShape::Box: {
            return testCollisionWithBox(static_cast<const BoxCollider&>(other), outInfo);
        }
        case CollisionShape::Sphere: {
            // sphere does the test, flip normal to point away from us
            if (!static_cast<const SphereCollider&>(other).testCollisionWithBox(*this, outInfo))
            {
                return false;
            }

            outInfo.normal = outInfo.normal * -1.0;
            return true;
        }
        case CollisionShape::Ground: {
            return testCollisionWithGround(static_cast<const GroundCollider&>(other), outInfo);
        }
        default:
            return false;
    }
}

bool BoxCollider::raycast(const Ray& ray, double& outDistance) const
{
    const math::Vec3 half = m_size * 0.5;
    const math::Vec3 toCentre = m_position - ray.origin;

    // slab method run in the box own axes, so a turned box needs no special case
    double entry = -1e30;
    double exit = 1e30;

    for (int i = 0; i < 3; i++)
    {
        const double extent = (i == 0) ? half.x : (i == 1) ? half.y : half.z;

        const double along = ray.direction.dot(m_axes[i]);
        const double centre = toCentre.dot(m_axes[i]);

        if (std::abs(along) < 1e-9)
        {
            // ray runs parallel to this pair of faces, it must start between them
            if (std::abs(centre) > extent)
            {
                return false;
            }

            continue;
        }

        double near = (centre - extent) / along;
        double far = (centre + extent) / along;

        if (near > far)
        {
            std::swap(near, far);
        }

        entry = std::max(entry, near);
        exit = std::min(exit, far);

        if (entry > exit)
        {
            return false;
        }
    }

    // a ray starting inside leaves through the far face
    const double distance = (entry >= 0.0) ? entry : exit;

    if (distance < 0.0 || distance > ray.maxDistance)
    {
        return false;
    }

    outDistance = distance;
    return true;
}

math::Vec3 BoxCollider::normalAt(const math::Vec3& point) const
{
    const math::Vec3 half = m_size * 0.5;
    const math::Vec3 local = point - m_position;

    // the face the point sits closest to owns the normal
    int nearest = 0;
    double smallestGap = 1e30;

    for (int i = 0; i < 3; i++)
    {
        const double extent = (i == 0) ? half.x : (i == 1) ? half.y : half.z;
        const double gap = extent - std::abs(local.dot(m_axes[i]));

        if (gap < smallestGap)
        {
            smallestGap = gap;
            nearest = i;
        }
    }

    return m_axes[nearest] * (local.dot(m_axes[nearest]) >= 0.0 ? 1.0 : -1.0);
}

// half width of the box measured along an arbitrary direction
static double projectedRadius(const math::Vec3& half, const math::Vec3* axes, const math::Vec3& direction)
{
    return std::abs(direction.dot(axes[0])) * half.x
         + std::abs(direction.dot(axes[1])) * half.y
         + std::abs(direction.dot(axes[2])) * half.z;
}

bool BoxCollider::testCollisionWithBox(const BoxCollider& other, CollisionInfo& outInfo) const
{
    const math::Vec3 half = m_size * 0.5;
    const math::Vec3 otherHalf = other.m_size * 0.5;

    const math::Vec3 diff = other.m_position - m_position;

    // separating axis theorem, a gap on any axis means no contact
    math::Vec3 axes[15];
    int axisCount = 0;

    // face normals of both boxes
    for (int i = 0; i < 3; i++)
    {
        axes[axisCount++] = m_axes[i];
        axes[axisCount++] = other.m_axes[i];
    }

    // and every pair of edge directions
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            // parallel edges give a degenerate axis, their face normals cover it
            const math::Vec3 axis = m_axes[i].cross(other.m_axes[j]).normalized();

            if (axis.length() > 0.5)
            {
                axes[axisCount++] = axis;
            }
        }
    }

    double leastOverlap = 1e30;
    math::Vec3 leastAxis{};

    for (int i = 0; i < axisCount; i++)
    {
        const math::Vec3& axis = axes[i];

        const double reach = projectedRadius(half, m_axes, axis) + projectedRadius(otherHalf, other.m_axes, axis);
        const double distance = std::abs(diff.dot(axis));

        if (distance > reach)
        {
            return false;
        }

        const double overlap = reach - distance;
        if (overlap < leastOverlap)
        {
            leastOverlap = overlap;
            leastAxis = axis;
        }
    }

    // point the axis from this box towards the other one
    outInfo.normal = (diff.dot(leastAxis) < 0.0) ? leastAxis * -1.0 : leastAxis;
    outInfo.penetration = leastOverlap;

    // face against face gives a whole polygon, which is what holds a stack up
    outInfo.pointCount = 0;

    clipFace(other, outInfo.normal, outInfo);

    // edge crossing an edge leaves no face to clip, fall back to the deepest corner
    if (outInfo.pointCount == 0)
    {
        math::Vec3 deepest = other.m_position;

        for (int i = 0; i < 3; i++)
        {
            const double extent = (i == 0) ? otherHalf.x : (i == 1) ? otherHalf.y : otherHalf.z;
            const double side = (other.m_axes[i].dot(outInfo.normal) > 0.0) ? -extent : extent;

            deepest += other.m_axes[i] * side;
        }

        const math::Vec3 local = deepest - m_position;
        math::Vec3 point = m_position;

        for (int i = 0; i < 3; i++)
        {
            const double extent = (i == 0) ? half.x : (i == 1) ? half.y : half.z;

            point += m_axes[i] * std::clamp(local.dot(m_axes[i]), -extent, extent);
        }

        outInfo.addPoint(point, FEATURE_CLAMPED);
    }

    return true;
}

// the face most square to the normal, the one that actually rests on the contact
int BoxCollider::faceAxis(const math::Vec3& normal, double& outSide) const
{
    int best = 0;
    double bestDot = 0.0;

    for (int i = 0; i < 3; i++)
    {
        const double dot = m_axes[i].dot(normal);

        if (std::abs(dot) > std::abs(bestDot))
        {
            bestDot = dot;
            best = i;
        }
    }

    outSide = (bestDot > 0.0) ? 1.0 : -1.0;

    return best;
}

// corners of one face, wound so clipping walks its edges in order
void BoxCollider::faceCorners(int axis, double side, math::Vec3 outCorners[4]) const
{
    const math::Vec3 half = m_size * 0.5;

    const double extent = (axis == 0) ? half.x : (axis == 1) ? half.y : half.z;
    const math::Vec3 centre = m_position + m_axes[axis] * (extent * side);

    const int u = (axis + 1) % 3;
    const int v = (axis + 2) % 3;

    const double uExtent = (u == 0) ? half.x : (u == 1) ? half.y : half.z;
    const double vExtent = (v == 0) ? half.x : (v == 1) ? half.y : half.z;

    const math::Vec3 uEdge = m_axes[u] * uExtent;
    const math::Vec3 vEdge = m_axes[v] * vExtent;

    outCorners[0] = centre - uEdge - vEdge;
    outCorners[1] = centre + uEdge - vEdge;
    outCorners[2] = centre + uEdge + vEdge;
    outCorners[3] = centre - uEdge + vEdge;
}

// sutherland-hodgman, keeps what lies inside the plane and cuts the edges crossing it
static int clipAgainstPlane(const math::Vec3* input, int count, const math::Vec3& planeNormal, double planeOffset,
                            math::Vec3* output)
{
    int result = 0;

    for (int i = 0; i < count; i++)
    {
        const math::Vec3& current = input[i];
        const math::Vec3& next = input[(i + 1) % count];

        const double currentDistance = current.dot(planeNormal) - planeOffset;
        const double nextDistance = next.dot(planeNormal) - planeOffset;

        if (currentDistance <= 0.0)
        {
            output[result++] = current;
        }

        if ((currentDistance > 0.0) != (nextDistance > 0.0))
        {
            const double span = currentDistance - nextDistance;

            if (std::abs(span) > 1e-12)
            {
                output[result++] = current + (next - current) * (currentDistance / span);
            }
        }
    }

    return result;
}

// two faces overlap in a polygon, corners alone miss it when neither box has one inside
void BoxCollider::clipFace(const BoxCollider& other, const math::Vec3& normal, CollisionInfo& outInfo) const
{
    double referenceSide = 0.0;
    const int referenceAxis = faceAxis(normal, referenceSide);

    // the incident face looks back at the contact, hence the flipped normal
    double incidentSide = 0.0;
    const int incidentAxis = other.faceAxis(normal * -1.0, incidentSide);

    math::Vec3 polygon[MAX_CLIP_POINTS];
    math::Vec3 clipped[MAX_CLIP_POINTS];

    other.faceCorners(incidentAxis, incidentSide, polygon);
    int count = 4;

    const math::Vec3 half = m_size * 0.5;

    // cut against the four sides of the reference face
    for (int i = 0; i < 3 && count > 0; i++)
    {
        if (i == referenceAxis)
        {
            continue;
        }

        const double extent = (i == 0) ? half.x : (i == 1) ? half.y : half.z;
        const double centre = m_position.dot(m_axes[i]);

        count = clipAgainstPlane(polygon, count, m_axes[i], centre + extent, clipped);
        count = clipAgainstPlane(clipped, count, m_axes[i] * -1.0, -(centre - extent), polygon);
    }

    // keep what sits at the contact plane, the rest belongs to the far side
    const double referenceExtent = (referenceAxis == 0) ? half.x : (referenceAxis == 1) ? half.y : half.z;
    const double surface = m_position.dot(normal) + referenceExtent * referenceSide * m_axes[referenceAxis].dot(normal);

    math::Vec3 touching[MAX_CLIP_POINTS];
    int touchingCount = 0;

    for (int i = 0; i < count; i++)
    {
        if (surface - polygon[i].dot(normal) >= -CONTACT_MARGIN)
        {
            touching[touchingCount++] = polygon[i];
        }
    }

    reducePoints(touching, touchingCount, outInfo);
}

// a manifold holds four, keep the widest spread or the patch stops resisting tipping
void BoxCollider::reducePoints(const math::Vec3* points, int count, CollisionInfo& outInfo)
{
    if (count <= MAX_CONTACT_POINTS)
    {
        for (int i = 0; i < count; i++)
        {
            outInfo.addPoint(points[i], i);
        }

        return;
    }

    math::Vec3 centre{};
    for (int i = 0; i < count; i++)
    {
        centre += points[i];
    }
    centre = centre / static_cast<double>(count);

    bool taken[MAX_CLIP_POINTS]{};

    // first the corner furthest out, then the one furthest from those already kept
    for (int picked = 0; picked < MAX_CONTACT_POINTS; picked++)
    {
        int best = -1;
        double bestDistance = -1.0;

        for (int i = 0; i < count; i++)
        {
            if (taken[i])
            {
                continue;
            }

            double distance = (points[i] - centre).length();

            for (int j = 0; j < count; j++)
            {
                if (taken[j])
                {
                    distance = std::min(distance, (points[i] - points[j]).length());
                }
            }

            if (distance > bestDistance)
            {
                bestDistance = distance;
                best = i;
            }
        }

        if (best < 0)
        {
            break;
        }

        taken[best] = true;
        outInfo.addPoint(points[best], picked);
    }
}

bool BoxCollider::testCollisionWithGround(const GroundCollider& ground, CollisionInfo& outInfo) const
{
    const double groundY = ground.getGroundY();
    const math::Vec3 half = m_size * 0.5;

    math::Vec3 corners[8];
    double lowestY = m_position.y;

    for (int i = 0; i < 8; i++)
    {
        const double sx = (i & 1) ? half.x : -half.x;
        const double sy = (i & 2) ? half.y : -half.y;
        const double sz = (i & 4) ? half.z : -half.z;

        corners[i] = m_position + m_axes[0] * sx + m_axes[1] * sy + m_axes[2] * sz;

        lowestY = std::min(lowestY, corners[i].y);
    }

    if (lowestY >= groundY + CONTACT_MARGIN)
    {
        return false;
    }

    // a tilt of one degree already lifts the far corners past a fixed margin
    const double reach = std::max(CONTACT_MARGIN, m_size.length() * REST_TOLERANCE);

    // every corner near the plane, flat face gives four and stops rocking
    outInfo.pointCount = 0;

    math::Vec3 touching[8];
    int touchingCount = 0;

    for (int i = 0; i < 8; i++)
    {
        if (corners[i].y < lowestY + reach)
        {
            touching[touchingCount++] = {corners[i].x, groundY, corners[i].z};
        }
    }

    reducePoints(touching, touchingCount, outInfo);

    if (outInfo.pointCount > 0)
    {
        // box is first collider, normal points down into ground
        outInfo.normal = math::Vec3{0.0, -1.0, 0.0};
        outInfo.penetration = groundY - lowestY;

        return true;
    }

    return false;
}

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
