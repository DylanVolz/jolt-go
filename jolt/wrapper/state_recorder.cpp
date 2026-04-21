/*
 * Jolt Physics C Wrapper - State Recorder Implementation (T-0130)
 */

#include "state_recorder.h"
#include "physics.h"
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <algorithm>
#include <cstring>
#include <string>

using namespace JPH;

JoltStateRecorder JoltCreateStateRecorder()
{
    return static_cast<JoltStateRecorder>(new StateRecorderImpl());
}

void JoltDestroyStateRecorder(JoltStateRecorder rec)
{
    delete static_cast<StateRecorderImpl *>(rec);
}

void JoltStateRecorderClear(JoltStateRecorder rec)
{
    static_cast<StateRecorderImpl *>(rec)->Clear();
}

void JoltStateRecorderRewind(JoltStateRecorder rec)
{
    static_cast<StateRecorderImpl *>(rec)->Rewind();
}

size_t JoltStateRecorderGetDataSize(JoltStateRecorder rec)
{
    return static_cast<StateRecorderImpl *>(rec)->GetDataSize();
}

size_t JoltStateRecorderCopyData(JoltStateRecorder rec, void *buf, size_t bufLen)
{
    if (buf == nullptr || bufLen == 0)
        return 0;

    StateRecorderImpl *impl = static_cast<StateRecorderImpl *>(rec);
    std::string data = impl->GetData();
    size_t n = std::min(bufLen, data.size());
    std::memcpy(buf, data.data(), n);
    return n;
}

void JoltStateRecorderWriteBytes(JoltStateRecorder rec, const void *data, size_t numBytes)
{
    if (data == nullptr || numBytes == 0)
        return;
    static_cast<StateRecorderImpl *>(rec)->WriteBytes(data, numBytes);
}

void JoltStateRecorderSetValidating(JoltStateRecorder rec, int validating)
{
    static_cast<StateRecorderImpl *>(rec)->SetValidating(validating != 0);
}

int JoltStateRecorderIsValidating(JoltStateRecorder rec)
{
    return static_cast<StateRecorderImpl *>(rec)->IsValidating() ? 1 : 0;
}

int JoltStateRecorderIsEOF(JoltStateRecorder rec)
{
    return static_cast<StateRecorderImpl *>(rec)->IsEOF() ? 1 : 0;
}

int JoltStateRecorderIsFailed(JoltStateRecorder rec)
{
    return static_cast<StateRecorderImpl *>(rec)->IsFailed() ? 1 : 0;
}

int JoltStateRecorderIsEqual(JoltStateRecorder rec, JoltStateRecorder reference)
{
    StateRecorderImpl *a = static_cast<StateRecorderImpl *>(rec);
    StateRecorderImpl *b = static_cast<StateRecorderImpl *>(reference);
    return a->IsEqual(*b) ? 1 : 0;
}

void JoltPhysicsSystemSaveState(JoltPhysicsSystem system, JoltStateRecorder rec, int state)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    StateRecorderImpl *impl = static_cast<StateRecorderImpl *>(rec);
    ps->SaveState(*impl, static_cast<EStateRecorderState>(state), nullptr);
}

int JoltPhysicsSystemRestoreState(JoltPhysicsSystem system, JoltStateRecorder rec)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    StateRecorderImpl *impl = static_cast<StateRecorderImpl *>(rec);

    if (!ps->RestoreState(*impl))
        return -1;
    if (impl->IsFailed())
        return -1;
    return 0;
}
