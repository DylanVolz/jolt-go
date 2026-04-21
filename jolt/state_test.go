package jolt

import (
	"bytes"
	"math"
	"testing"
)

// setupFallingSphereWorld creates a tiny deterministic scene: a static ground
// box at Y=0 and a dynamic sphere dropped from Y=10 with a small sideways nudge.
// The sphere bounces, giving us non-trivial trajectory samples.
func setupFallingSphereWorld(t *testing.T) (*PhysicsSystem, *BodyInterface, *BodyID, *BodyID, func()) {
	t.Helper()

	if err := Init(); err != nil {
		t.Fatalf("Init failed: %v", err)
	}

	ps := NewPhysicsSystem()
	bi := ps.GetBodyInterface()

	ground := CreateBox(Vec3{X: 50, Y: 0.5, Z: 50})
	groundBody := bi.CreateBody(ground, Vec3{X: 0, Y: 0, Z: 0}, MotionTypeStatic, false)
	bi.SetRestitution(groundBody, 0.8)

	sphere := CreateSphere(0.5)
	sphereBody := bi.CreateBody(sphere, Vec3{X: 0, Y: 10, Z: 0}, MotionTypeDynamic, false)
	bi.SetRestitution(sphereBody, 0.8)
	bi.SetLinearVelocity(sphereBody, Vec3{X: 1.5, Y: 0, Z: 0.7})
	bi.ActivateBody(sphereBody)

	cleanup := func() {
		bi.RemoveAndDestroyBody(sphereBody)
		bi.RemoveAndDestroyBody(groundBody)
		ground.Destroy()
		sphere.Destroy()
		ps.Destroy()
		// Note: Shutdown is intentionally NOT called — Init/Shutdown is global and
		// other tests in this package rely on Jolt staying initialized.
	}
	return ps, bi, groundBody, sphereBody, cleanup
}

// sample returns a trajectory snapshot: position and velocity of the sphere.
type sample struct {
	pos, vel Vec3
}

func recordTrajectory(t *testing.T, bi *BodyInterface, body *BodyID, ps *PhysicsSystem, steps int, dt float32) []sample {
	t.Helper()
	out := make([]sample, steps)
	for i := range steps {
		ps.Update(dt)
		out[i] = sample{
			pos: bi.GetPosition(body),
			vel: bi.GetLinearVelocity(body),
		}
	}
	return out
}

func samplesEqual(a, b sample) bool {
	const eps = 1e-5
	diff := func(x, y float32) bool { return math.Abs(float64(x-y)) > eps }
	if diff(a.pos.X, b.pos.X) || diff(a.pos.Y, b.pos.Y) || diff(a.pos.Z, b.pos.Z) {
		return false
	}
	if diff(a.vel.X, b.vel.X) || diff(a.vel.Y, b.vel.Y) || diff(a.vel.Z, b.vel.Z) {
		return false
	}
	return true
}

func TestStateRecorderSaveRestore300Ticks(t *testing.T) {
	ps, bi, _, sphere, cleanup := setupFallingSphereWorld(t)
	defer cleanup()

	dt := float32(1.0 / 60.0)

	// Phase 1: simulate 150 ticks, then save state.
	for range 150 {
		ps.Update(dt)
	}

	rec := NewStateRecorder()
	defer rec.Destroy()

	ps.SaveState(rec, StateRecorderAll)
	if rec.Size() == 0 {
		t.Fatal("SaveState produced empty buffer")
	}

	// Phase 2: continue for another 150 ticks, recording trajectory (the "reference").
	reference := recordTrajectory(t, bi, sphere, ps, 150, dt)

	// Restore state back to the tick-150 checkpoint and re-run the same 150 ticks.
	rec.Rewind()
	if err := ps.RestoreState(rec); err != nil {
		t.Fatalf("RestoreState failed: %v", err)
	}
	replay := recordTrajectory(t, bi, sphere, ps, 150, dt)

	// Both 150-tick continuations must produce identical trajectories.
	for i := range reference {
		if !samplesEqual(reference[i], replay[i]) {
			t.Fatalf("trajectory diverged at tick %d: ref=%+v replay=%+v",
				i, reference[i], replay[i])
		}
	}
}

func TestStateRecorderBytesRoundTrip(t *testing.T) {
	ps, bi, _, sphere, cleanup := setupFallingSphereWorld(t)
	defer cleanup()

	dt := float32(1.0 / 60.0)
	for range 60 {
		ps.Update(dt)
	}

	rec := NewStateRecorder()
	defer rec.Destroy()
	ps.SaveState(rec, StateRecorderAll)

	data := rec.Bytes()
	if len(data) == 0 {
		t.Fatal("Bytes returned empty slice")
	}
	if len(data) != rec.Size() {
		t.Fatalf("Bytes length %d != Size %d", len(data), rec.Size())
	}

	// Advance the sim so current state diverges from the snapshot.
	posBefore := bi.GetPosition(sphere)
	for range 60 {
		ps.Update(dt)
	}
	posAfter := bi.GetPosition(sphere)
	if samplesEqual(sample{pos: posBefore}, sample{pos: posAfter}) {
		t.Fatal("sim didn't advance between snapshots — test is degenerate")
	}

	// Load snapshot into a fresh recorder and restore.
	rec2 := NewStateRecorder()
	defer rec2.Destroy()
	rec2.WriteBytes(data)
	if rec2.Size() != len(data) {
		t.Fatalf("WriteBytes size mismatch: got %d want %d", rec2.Size(), len(data))
	}
	rec2.Rewind()
	if err := ps.RestoreState(rec2); err != nil {
		t.Fatalf("RestoreState after roundtrip failed: %v", err)
	}

	// Position should now match the one we captured before advancing.
	got := bi.GetPosition(sphere)
	if !samplesEqual(sample{pos: got}, sample{pos: posBefore}) {
		t.Fatalf("restored position %v != snapshot position %v", got, posBefore)
	}
}

func TestStateRecorderEqual(t *testing.T) {
	ps, _, _, _, cleanup := setupFallingSphereWorld(t)
	defer cleanup()

	dt := float32(1.0 / 60.0)
	for range 30 {
		ps.Update(dt)
	}

	a := NewStateRecorder()
	defer a.Destroy()
	b := NewStateRecorder()
	defer b.Destroy()

	ps.SaveState(a, StateRecorderAll)
	ps.SaveState(b, StateRecorderAll)

	if !a.Equal(b) {
		t.Error("two saves of the same PhysicsSystem state should be byte-equal")
	}
	if !bytes.Equal(a.Bytes(), b.Bytes()) {
		t.Error("Bytes() output should match when Equal() reports true")
	}
}
