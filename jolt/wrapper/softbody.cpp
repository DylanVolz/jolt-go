/*
 * Jolt Physics C Wrapper - Soft Body Operations Implementation (T-0129)
 */

#include "softbody.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodyVertex.h>

#include <memory>

using namespace JPH;

namespace {

inline SoftBodySharedSettings* ToSettings(JoltSoftBodySharedSettings s)
{
    return static_cast<SoftBodySharedSettings*>(s);
}

SoftBodySharedSettings::EBendType ToJoltBendType(JoltBendType t)
{
    switch (t)
    {
    case JoltBendTypeDistance: return SoftBodySharedSettings::EBendType::Distance;
    case JoltBendTypeDihedral: return SoftBodySharedSettings::EBendType::Dihedral;
    case JoltBendTypeNone:
    default:                   return SoftBodySharedSettings::EBendType::None;
    }
}

} // namespace

// --- Shared settings construction -------------------------------------------

JoltSoftBodySharedSettings JoltCreateSoftBodySharedSettings(void)
{
    SoftBodySharedSettings *settings = new SoftBodySharedSettings();
    // Take an initial ref on behalf of the caller. Released by
    // JoltDestroySoftBodySharedSettings.
    settings->AddRef();
    return static_cast<JoltSoftBodySharedSettings>(settings);
}

int JoltSoftBodySharedSettingsAddVertex(JoltSoftBodySharedSettings settings,
                                        float x, float y, float z,
                                        float invMass)
{
    SoftBodySharedSettings *s = ToSettings(settings);
    if (!s) return -1;

    SoftBodySharedSettings::Vertex v;
    v.mPosition = Float3(x, y, z);
    v.mInvMass = invMass;
    int index = static_cast<int>(s->mVertices.size());
    s->mVertices.push_back(v);
    return index;
}

int JoltSoftBodySharedSettingsAddFace(JoltSoftBodySharedSettings settings,
                                      unsigned int v0,
                                      unsigned int v1,
                                      unsigned int v2)
{
    SoftBodySharedSettings *s = ToSettings(settings);
    if (!s) return -1;

    SoftBodySharedSettings::Face f(v0, v1, v2);
    if (f.IsDegenerate()) return -1;
    s->AddFace(f);
    return 0;
}

void JoltSoftBodySharedSettingsAddEdge(JoltSoftBodySharedSettings settings,
                                       unsigned int v0,
                                       unsigned int v1,
                                       float compliance)
{
    SoftBodySharedSettings *s = ToSettings(settings);
    if (!s) return;
    s->mEdgeConstraints.push_back(SoftBodySharedSettings::Edge(v0, v1, compliance));
}

void JoltSoftBodySharedSettingsCreateConstraints(JoltSoftBodySharedSettings settings,
                                                 float compliance,
                                                 float shearCompliance,
                                                 float bendCompliance,
                                                 JoltBendType bendType)
{
    SoftBodySharedSettings *s = ToSettings(settings);
    if (!s) return;

    SoftBodySharedSettings::VertexAttributes attr;
    attr.mCompliance = compliance;
    attr.mShearCompliance = shearCompliance;
    attr.mBendCompliance = bendCompliance;

    s->CreateConstraints(&attr, 1, ToJoltBendType(bendType));
}

void JoltSoftBodySharedSettingsOptimize(JoltSoftBodySharedSettings settings)
{
    SoftBodySharedSettings *s = ToSettings(settings);
    if (!s) return;
    s->Optimize();
}

void JoltDestroySoftBodySharedSettings(JoltSoftBodySharedSettings settings)
{
    SoftBodySharedSettings *s = ToSettings(settings);
    if (!s) return;
    // Release the caller's ref. If any soft body still holds a ref, the
    // object survives until those bodies are destroyed.
    s->Release();
}

JoltSoftBodySharedSettings JoltCreateClothGridSettings(int gridSizeX,
                                                       int gridSizeZ,
                                                       float spacing,
                                                       const float* invMasses,
                                                       int numInvMasses,
                                                       float compliance,
                                                       JoltBendType bendType)
{
    if (gridSizeX < 2 || gridSizeZ < 2 || spacing <= 0.0f) return nullptr;

    SoftBodySharedSettings *settings = new SoftBodySharedSettings();
    settings->AddRef();

    const float offsetX = -0.5f * spacing * (gridSizeX - 1);
    const float offsetZ = -0.5f * spacing * (gridSizeZ - 1);

    // Build vertex grid (row-major, Z outer).
    for (int z = 0; z < gridSizeZ; ++z)
        for (int x = 0; x < gridSizeX; ++x)
        {
            SoftBodySharedSettings::Vertex v;
            v.mPosition = Float3(offsetX + x * spacing, 0.0f, offsetZ + z * spacing);
            int idx = x + z * gridSizeX;
            if (invMasses != nullptr && idx < numInvMasses)
                v.mInvMass = invMasses[idx];
            else
                v.mInvMass = 1.0f;
            settings->mVertices.push_back(v);
        }

    auto vertex_index = [gridSizeX](int x, int z) -> uint32 {
        return static_cast<uint32>(x + z * gridSizeX);
    };

    // Build face list (two triangles per grid cell).
    for (int z = 0; z < gridSizeZ - 1; ++z)
        for (int x = 0; x < gridSizeX - 1; ++x)
        {
            SoftBodySharedSettings::Face f;
            f.mVertex[0] = vertex_index(x, z);
            f.mVertex[1] = vertex_index(x, z + 1);
            f.mVertex[2] = vertex_index(x + 1, z + 1);
            settings->AddFace(f);

            f.mVertex[1] = vertex_index(x + 1, z + 1);
            f.mVertex[2] = vertex_index(x + 1, z);
            settings->AddFace(f);
        }

    SoftBodySharedSettings::VertexAttributes attr;
    attr.mCompliance = compliance;
    attr.mShearCompliance = compliance;
    attr.mBendCompliance = compliance;

    settings->CreateConstraints(&attr, 1, ToJoltBendType(bendType));
    settings->Optimize();

    return static_cast<JoltSoftBodySharedSettings>(settings);
}

// --- Soft body creation -----------------------------------------------------

JoltBodyID JoltCreateSoftBody(JoltBodyInterface bodyInterface,
                              JoltSoftBodySharedSettings settings,
                              float x, float y, float z,
                              float rotX, float rotY, float rotZ, float rotW,
                              int layer,
                              float pressure,
                              int numIterations,
                              float linearDamping,
                              float gravityFactor)
{
    BodyInterface *bi = static_cast<BodyInterface *>(bodyInterface);
    SoftBodySharedSettings *s = ToSettings(settings);
    if (!bi || !s) return nullptr;

    SoftBodyCreationSettings sbcs(s,
                                  RVec3(x, y, z),
                                  Quat(rotX, rotY, rotZ, rotW),
                                  static_cast<ObjectLayer>(layer));
    if (numIterations > 0) sbcs.mNumIterations = static_cast<uint32>(numIterations);
    sbcs.mPressure = pressure;
    sbcs.mLinearDamping = linearDamping;
    sbcs.mGravityFactor = gravityFactor;

    BodyID bodyID = bi->CreateAndAddSoftBody(sbcs, EActivation::Activate);
    if (bodyID.IsInvalid()) return nullptr;

    auto bodyIDPtr = std::make_unique<BodyID>(bodyID);
    return static_cast<JoltBodyID>(bodyIDPtr.release());
}

// --- Soft body runtime queries ----------------------------------------------

int JoltGetSoftBodyVertexCount(JoltPhysicsSystem system, JoltBodyID bodyID)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid = static_cast<const BodyID *>(bodyID);
    if (!ps || !bid) return -1;

    BodyLockRead lock(ps->GetBodyLockInterface(), *bid);
    if (!lock.Succeeded()) return -1;

    const Body &body = lock.GetBody();
    if (!body.IsSoftBody()) return -1;

    const SoftBodyMotionProperties *mp =
        static_cast<const SoftBodyMotionProperties *>(body.GetMotionPropertiesUnchecked());
    if (!mp) return -1;

    return static_cast<int>(mp->GetVertices().size());
}

int JoltGetSoftBodyVertexPositions(JoltPhysicsSystem system,
                                   JoltBodyID bodyID,
                                   float* outXYZ,
                                   int bufLen)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid = static_cast<const BodyID *>(bodyID);
    if (!ps || !bid || !outXYZ || bufLen <= 0) return -1;

    BodyLockRead lock(ps->GetBodyLockInterface(), *bid);
    if (!lock.Succeeded()) return -1;

    const Body &body = lock.GetBody();
    if (!body.IsSoftBody()) return -1;

    const SoftBodyMotionProperties *mp =
        static_cast<const SoftBodyMotionProperties *>(body.GetMotionPropertiesUnchecked());
    if (!mp) return -1;

    RMat44 com = body.GetCenterOfMassTransform();
    const auto &vertices = mp->GetVertices();
    int count = static_cast<int>(vertices.size());
    int maxFromBuf = bufLen / 3;
    if (count > maxFromBuf) count = maxFromBuf;

    for (int i = 0; i < count; ++i)
    {
        RVec3 world = com * vertices[i].mPosition;
        outXYZ[i * 3 + 0] = static_cast<float>(world.GetX());
        outXYZ[i * 3 + 1] = static_cast<float>(world.GetY());
        outXYZ[i * 3 + 2] = static_cast<float>(world.GetZ());
    }
    return count;
}

float JoltGetSoftBodyVertexInvMass(JoltPhysicsSystem system,
                                   JoltBodyID bodyID,
                                   int index)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid = static_cast<const BodyID *>(bodyID);
    if (!ps || !bid || index < 0) return -1.0f;

    BodyLockRead lock(ps->GetBodyLockInterface(), *bid);
    if (!lock.Succeeded()) return -1.0f;

    const Body &body = lock.GetBody();
    if (!body.IsSoftBody()) return -1.0f;

    const SoftBodyMotionProperties *mp =
        static_cast<const SoftBodyMotionProperties *>(body.GetMotionPropertiesUnchecked());
    if (!mp || index >= static_cast<int>(mp->GetVertices().size())) return -1.0f;

    return mp->GetVertex(static_cast<uint>(index)).mInvMass;
}

int JoltSetSoftBodyVertexInvMass(JoltPhysicsSystem system,
                                 JoltBodyID bodyID,
                                 int index,
                                 float invMass)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid = static_cast<const BodyID *>(bodyID);
    if (!ps || !bid || index < 0) return -1;

    BodyLockWrite lock(ps->GetBodyLockInterface(), *bid);
    if (!lock.Succeeded()) return -1;

    Body &body = lock.GetBody();
    if (!body.IsSoftBody()) return -1;

    SoftBodyMotionProperties *mp =
        static_cast<SoftBodyMotionProperties *>(body.GetMotionPropertiesUnchecked());
    if (!mp || index >= static_cast<int>(mp->GetVertices().size())) return -1;

    mp->GetVertex(static_cast<uint>(index)).mInvMass = invMass;
    return 0;
}

void JoltAddSoftBodyForce(JoltBodyInterface bodyInterface,
                          JoltBodyID bodyID,
                          float fx, float fy, float fz)
{
    BodyInterface *bi = static_cast<BodyInterface *>(bodyInterface);
    const BodyID *bid = static_cast<const BodyID *>(bodyID);
    if (!bi || !bid) return;
    bi->AddForce(*bid, Vec3(fx, fy, fz));
}
