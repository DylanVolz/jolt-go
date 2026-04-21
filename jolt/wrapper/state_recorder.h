/*
 * Jolt Physics C Wrapper - State Recorder (T-0130)
 *
 * Binds JPH::StateRecorderImpl for serializing and restoring an entire
 * PhysicsSystem (body positions, velocities, constraint state, contact cache).
 *
 * Usage:
 *   JoltStateRecorder rec = JoltCreateStateRecorder();
 *   JoltPhysicsSystemSaveState(system, rec, JoltStateRecorderAll);
 *   size_t n = JoltStateRecorderGetDataSize(rec);
 *   // ... store n bytes via JoltStateRecorderCopyData ...
 *   JoltStateRecorderRewind(rec);
 *   JoltPhysicsSystemRestoreState(system, rec);
 *   JoltDestroyStateRecorder(rec);
 */

#ifndef JOLT_WRAPPER_STATE_RECORDER_H
#define JOLT_WRAPPER_STATE_RECORDER_H

#include "physics.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to a JPH::StateRecorderImpl
typedef void* JoltStateRecorder;

// Matches JPH::EStateRecorderState (uint8 bit field)
typedef enum {
    JoltStateRecorderNone        = 0,
    JoltStateRecorderGlobal      = 1,
    JoltStateRecorderBodies      = 2,
    JoltStateRecorderContacts    = 4,
    JoltStateRecorderConstraints = 8,
    JoltStateRecorderAll         = 1 | 2 | 4 | 8
} JoltStateRecorderState;

// Create a new in-memory state recorder.
JoltStateRecorder JoltCreateStateRecorder();

// Destroy a state recorder and free its backing buffer.
void JoltDestroyStateRecorder(JoltStateRecorder rec);

// Remove all bytes from the recorder and reset read/write positions.
void JoltStateRecorderClear(JoltStateRecorder rec);

// Reset the read position to the start of the stream (required before RestoreState).
void JoltStateRecorderRewind(JoltStateRecorder rec);

// Size in bytes of the data currently buffered in the recorder.
size_t JoltStateRecorderGetDataSize(JoltStateRecorder rec);

// Copy up to bufLen bytes of the recorder's data into buf.
// Returns the number of bytes copied (min(bufLen, GetDataSize)).
size_t JoltStateRecorderCopyData(JoltStateRecorder rec, void *buf, size_t bufLen);

// Append raw bytes into the recorder's buffer (for loading serialized state from disk/network).
void JoltStateRecorderWriteBytes(JoltStateRecorder rec, const void *data, size_t numBytes);

// Validation mode: when restoring, verify each read byte matches the current state.
void JoltStateRecorderSetValidating(JoltStateRecorder rec, int validating);
int  JoltStateRecorderIsValidating(JoltStateRecorder rec);

// Stream health checks.
int JoltStateRecorderIsEOF(JoltStateRecorder rec);
int JoltStateRecorderIsFailed(JoltStateRecorder rec);

// Byte-compare two recorders' buffers. Returns 1 if equal, 0 otherwise.
// Note: consumes/rewinds both stream read positions (matches JPH behavior).
int JoltStateRecorderIsEqual(JoltStateRecorder rec, JoltStateRecorder reference);

// Save the given PhysicsSystem state into the recorder.
// state is a bitmask of JoltStateRecorderState values.
void JoltPhysicsSystemSaveState(JoltPhysicsSystem system, JoltStateRecorder rec, int state);

// Restore the PhysicsSystem state from the recorder.
// The recorder must be rewound first (call JoltStateRecorderRewind).
// Returns 0 on success, -1 on failure (underlying stream error or mismatch).
int JoltPhysicsSystemRestoreState(JoltPhysicsSystem system, JoltStateRecorder rec);

#ifdef __cplusplus
}
#endif

#endif // JOLT_WRAPPER_STATE_RECORDER_H
