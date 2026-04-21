package jolt

// #include <stdlib.h>
// #include "wrapper/state_recorder.h"
import "C"

import (
	"errors"
	"unsafe"
)

// StateRecorderState is a bitmask selecting which aspects of the simulation
// to save. Matches JPH::EStateRecorderState.
type StateRecorderState int

const (
	StateRecorderNone        StateRecorderState = C.JoltStateRecorderNone
	StateRecorderGlobal      StateRecorderState = C.JoltStateRecorderGlobal      // delta time, gravity, etc.
	StateRecorderBodies      StateRecorderState = C.JoltStateRecorderBodies      // body positions, velocities
	StateRecorderContacts    StateRecorderState = C.JoltStateRecorderContacts    // contact cache
	StateRecorderConstraints StateRecorderState = C.JoltStateRecorderConstraints // constraint state
	StateRecorderAll         StateRecorderState = C.JoltStateRecorderAll
)

// StateRecorder is an in-memory buffer used to serialize and restore a
// PhysicsSystem's deterministic state. Each instance must be freed with Destroy.
type StateRecorder struct {
	handle C.JoltStateRecorder
}

// NewStateRecorder creates an empty state recorder.
func NewStateRecorder() *StateRecorder {
	return &StateRecorder{handle: C.JoltCreateStateRecorder()}
}

// Destroy frees the underlying buffer.
func (r *StateRecorder) Destroy() {
	if r.handle != nil {
		C.JoltDestroyStateRecorder(r.handle)
		r.handle = nil
	}
}

// Clear removes all data and resets read/write positions.
func (r *StateRecorder) Clear() {
	C.JoltStateRecorderClear(r.handle)
}

// Rewind resets the read cursor to the start of the stream.
// Call before RestoreState when reusing a recorder that was just written to.
func (r *StateRecorder) Rewind() {
	C.JoltStateRecorderRewind(r.handle)
}

// Size returns the number of bytes currently buffered.
func (r *StateRecorder) Size() int {
	return int(C.JoltStateRecorderGetDataSize(r.handle))
}

// Bytes copies the buffered data into a new byte slice.
// Useful for persisting state to disk or sending over the network.
func (r *StateRecorder) Bytes() []byte {
	n := r.Size()
	if n == 0 {
		return nil
	}
	out := make([]byte, n)
	C.JoltStateRecorderCopyData(r.handle, unsafe.Pointer(&out[0]), C.size_t(n))
	return out
}

// WriteBytes appends raw bytes to the recorder's buffer. Use this to load a
// previously serialized state before calling Rewind + RestoreState.
func (r *StateRecorder) WriteBytes(data []byte) {
	if len(data) == 0 {
		return
	}
	C.JoltStateRecorderWriteBytes(r.handle, unsafe.Pointer(&data[0]), C.size_t(len(data)))
}

// SetValidating enables validation mode. When restoring, each byte read is
// compared against the current state; mismatches are reported by Jolt.
// Only valid for full-state saves (StateRecorderAll) with no filter.
func (r *StateRecorder) SetValidating(v bool) {
	val := C.int(0)
	if v {
		val = 1
	}
	C.JoltStateRecorderSetValidating(r.handle, val)
}

// IsValidating reports whether validation mode is enabled.
func (r *StateRecorder) IsValidating() bool {
	return C.JoltStateRecorderIsValidating(r.handle) != 0
}

// IsEOF reports whether the read cursor has reached the end of the buffer.
func (r *StateRecorder) IsEOF() bool {
	return C.JoltStateRecorderIsEOF(r.handle) != 0
}

// IsFailed reports whether the underlying stream has encountered an error.
func (r *StateRecorder) IsFailed() bool {
	return C.JoltStateRecorderIsFailed(r.handle) != 0
}

// Equal returns true if r and other contain byte-identical buffers.
// Use this to detect determinism drift between runs.
func (r *StateRecorder) Equal(other *StateRecorder) bool {
	return C.JoltStateRecorderIsEqual(r.handle, other.handle) != 0
}

// SaveState serializes the PhysicsSystem's simulation-relevant state into rec.
// state is a bitmask (StateRecorderGlobal | StateRecorderBodies | ...).
// Pass StateRecorderAll to save everything required for deterministic replay.
//
// Note: configuration settings (friction, restitution, motion quality, etc.)
// are NOT saved — Jolt only serializes mutable simulation state.
func (ps *PhysicsSystem) SaveState(rec *StateRecorder, state StateRecorderState) {
	C.JoltPhysicsSystemSaveState(ps.handle, rec.handle, C.int(state))
}

// RestoreState deserializes state from rec into the PhysicsSystem, overwriting
// current simulation state. The recorder must be rewound (call rec.Rewind())
// after writing before it can be read back.
//
// Returns an error if the stream is malformed or Jolt rejects the restore.
func (ps *PhysicsSystem) RestoreState(rec *StateRecorder) error {
	if C.JoltPhysicsSystemRestoreState(ps.handle, rec.handle) != 0 {
		return errors.New("jolt: RestoreState failed")
	}
	return nil
}
