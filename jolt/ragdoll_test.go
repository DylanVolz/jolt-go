package jolt

import (
	"math"
	"testing"
)

// MOVING is the dynamic-body object layer baked into the default
// JoltCreatePhysicsSystem (matches Layers::MOVING in physics.cpp).
const ragdollObjectLayerMoving uint16 = 1

// humanoidJoint describes one bone in the test ragdoll.
type humanoidJoint struct {
	name        string
	parent      int     // -1 for root
	pos         Vec3    // world-space body position (T-pose)
	halfHeight  float32 // capsule half-height of the cylindrical part
	radius      float32 // capsule radius
	orient      Quat    // capsule orientation (identity = vertical)
	swingNormal float32 // swing limit (rad), normal half-cone
	swingPlane  float32 // swing limit (rad), plane half-cone
	twistMin    float32 // twist limit (rad)
	twistMax    float32 // twist limit (rad)
	pivotOffset Vec3    // joint pivot relative to this body's position
}

// quatAxisAngle is a tiny helper so the test stays self-contained.
func quatAxisAngle(axis Vec3, angle float32) Quat {
	half := angle * 0.5
	s := float32(math.Sin(float64(half)))
	c := float32(math.Cos(float64(half)))
	a := axis.Normalize()
	return Quat{X: a.X * s, Y: a.Y * s, Z: a.Z * s, W: c}
}

// build15BoneHumanoid spawns a ragdoll: pelvis, spine, chest, neck, head,
// L+R upper arm/forearm/hand, L+R thigh/shin. 15 bones total.
func build15BoneHumanoid() (joints []humanoidJoint, shapes []*Shape) {
	// Permissive default limits so a rest-pose ragdoll just settles.
	defSwing := float32(math.Pi / 2) // 90 deg
	defTwist := float32(math.Pi / 4) // 45 deg

	// Y-up world. Pelvis sits at Y=4 so it has time to fall before settling.
	const baseY = 4.0

	joints = []humanoidJoint{
		// 0: pelvis
		{name: "pelvis", parent: -1, pos: Vec3{0, baseY, 0},
			halfHeight: 0.10, radius: 0.18, orient: QuatIdentity(),
			swingNormal: 0, swingPlane: 0, twistMin: 0, twistMax: 0},
		// 1: spine
		{name: "spine", parent: 0, pos: Vec3{0, baseY + 0.30, 0},
			halfHeight: 0.10, radius: 0.16, orient: QuatIdentity(),
			swingNormal: defSwing * 0.5, swingPlane: defSwing * 0.5,
			twistMin: -defTwist, twistMax: defTwist,
			pivotOffset: Vec3{0, -0.15, 0}},
		// 2: chest
		{name: "chest", parent: 1, pos: Vec3{0, baseY + 0.60, 0},
			halfHeight: 0.12, radius: 0.20, orient: QuatIdentity(),
			swingNormal: defSwing * 0.5, swingPlane: defSwing * 0.5,
			twistMin: -defTwist, twistMax: defTwist,
			pivotOffset: Vec3{0, -0.15, 0}},
		// 3: neck
		{name: "neck", parent: 2, pos: Vec3{0, baseY + 0.85, 0},
			halfHeight: 0.05, radius: 0.06, orient: QuatIdentity(),
			swingNormal: defSwing * 0.6, swingPlane: defSwing * 0.6,
			twistMin: -defTwist, twistMax: defTwist,
			pivotOffset: Vec3{0, -0.10, 0}},
		// 4: head
		{name: "head", parent: 3, pos: Vec3{0, baseY + 1.05, 0},
			halfHeight: 0.05, radius: 0.12, orient: QuatIdentity(),
			swingNormal: defSwing * 0.6, swingPlane: defSwing * 0.6,
			twistMin: -defTwist, twistMax: defTwist,
			pivotOffset: Vec3{0, -0.10, 0}},

		// 5: l_upper_arm (out along +X)
		{name: "l_upper_arm", parent: 2, pos: Vec3{0.40, baseY + 0.70, 0},
			halfHeight: 0.12, radius: 0.06,
			orient:      quatAxisAngle(Vec3{X: 0, Y: 0, Z: 1}, float32(math.Pi/2)),
			swingNormal: defSwing, swingPlane: defSwing,
			twistMin: -defTwist, twistMax: defTwist,
			pivotOffset: Vec3{-0.18, 0, 0}},
		// 6: l_forearm
		{name: "l_forearm", parent: 5, pos: Vec3{0.75, baseY + 0.70, 0},
			halfHeight: 0.12, radius: 0.05,
			orient:      quatAxisAngle(Vec3{X: 0, Y: 0, Z: 1}, float32(math.Pi/2)),
			swingNormal: defSwing * 0.6, swingPlane: defSwing * 0.1,
			twistMin: -defTwist * 0.5, twistMax: defTwist * 0.5,
			pivotOffset: Vec3{-0.17, 0, 0}},
		// 7: l_hand
		{name: "l_hand", parent: 6, pos: Vec3{1.05, baseY + 0.70, 0},
			halfHeight: 0.04, radius: 0.04,
			orient:      quatAxisAngle(Vec3{X: 0, Y: 0, Z: 1}, float32(math.Pi/2)),
			swingNormal: defSwing * 0.5, swingPlane: defSwing * 0.5,
			twistMin: -defTwist, twistMax: defTwist,
			pivotOffset: Vec3{-0.10, 0, 0}},

		// 8: r_upper_arm (out along -X)
		{name: "r_upper_arm", parent: 2, pos: Vec3{-0.40, baseY + 0.70, 0},
			halfHeight: 0.12, radius: 0.06,
			orient:      quatAxisAngle(Vec3{X: 0, Y: 0, Z: 1}, float32(math.Pi/2)),
			swingNormal: defSwing, swingPlane: defSwing,
			twistMin: -defTwist, twistMax: defTwist,
			pivotOffset: Vec3{0.18, 0, 0}},
		// 9: r_forearm
		{name: "r_forearm", parent: 8, pos: Vec3{-0.75, baseY + 0.70, 0},
			halfHeight: 0.12, radius: 0.05,
			orient:      quatAxisAngle(Vec3{X: 0, Y: 0, Z: 1}, float32(math.Pi/2)),
			swingNormal: defSwing * 0.6, swingPlane: defSwing * 0.1,
			twistMin: -defTwist * 0.5, twistMax: defTwist * 0.5,
			pivotOffset: Vec3{0.17, 0, 0}},
		// 10: r_hand
		{name: "r_hand", parent: 9, pos: Vec3{-1.05, baseY + 0.70, 0},
			halfHeight: 0.04, radius: 0.04,
			orient:      quatAxisAngle(Vec3{X: 0, Y: 0, Z: 1}, float32(math.Pi/2)),
			swingNormal: defSwing * 0.5, swingPlane: defSwing * 0.5,
			twistMin: -defTwist, twistMax: defTwist,
			pivotOffset: Vec3{0.10, 0, 0}},

		// 11: l_thigh (down along -Y)
		{name: "l_thigh", parent: 0, pos: Vec3{0.15, baseY - 0.35, 0},
			halfHeight: 0.18, radius: 0.08, orient: QuatIdentity(),
			swingNormal: defSwing, swingPlane: defSwing,
			twistMin: -defTwist, twistMax: defTwist,
			pivotOffset: Vec3{0, 0.22, 0}},
		// 12: l_shin
		{name: "l_shin", parent: 11, pos: Vec3{0.15, baseY - 0.85, 0},
			halfHeight: 0.18, radius: 0.07, orient: QuatIdentity(),
			swingNormal: defSwing * 0.5, swingPlane: defSwing * 0.05,
			twistMin: -defTwist * 0.5, twistMax: defTwist * 0.5,
			pivotOffset: Vec3{0, 0.22, 0}},
		// 13: r_thigh
		{name: "r_thigh", parent: 0, pos: Vec3{-0.15, baseY - 0.35, 0},
			halfHeight: 0.18, radius: 0.08, orient: QuatIdentity(),
			swingNormal: defSwing, swingPlane: defSwing,
			twistMin: -defTwist, twistMax: defTwist,
			pivotOffset: Vec3{0, 0.22, 0}},
		// 14: r_shin
		{name: "r_shin", parent: 13, pos: Vec3{-0.15, baseY - 0.85, 0},
			halfHeight: 0.18, radius: 0.07, orient: QuatIdentity(),
			swingNormal: defSwing * 0.5, swingPlane: defSwing * 0.05,
			twistMin: -defTwist * 0.5, twistMax: defTwist * 0.5,
			pivotOffset: Vec3{0, 0.22, 0}},
	}

	shapes = make([]*Shape, len(joints))
	for i, j := range joints {
		shapes[i] = CreateCapsule(j.halfHeight, j.radius)
	}
	return joints, shapes
}

// destroyShapes is a small helper for test cleanup.
func destroyShapes(shapes []*Shape) {
	for _, s := range shapes {
		if s != nil {
			s.Destroy()
		}
	}
}

// TestRagdollSkeletonBasics covers the Skeleton API in isolation.
func TestRagdollSkeletonBasics(t *testing.T) {
	skel := NewSkeleton()
	defer skel.Destroy()

	if got := skel.JointCount(); got != 0 {
		t.Fatalf("empty skeleton joint count = %d, want 0", got)
	}

	root := skel.AddJoint("root", -1)
	if root != 0 {
		t.Fatalf("root joint index = %d, want 0", root)
	}
	child := skel.AddJoint("child", root)
	if child != 1 {
		t.Fatalf("child joint index = %d, want 1", child)
	}
	if skel.JointCount() != 2 {
		t.Fatalf("joint count = %d, want 2", skel.JointCount())
	}
	if skel.ParentIndex(0) != -1 {
		t.Errorf("root parent = %d, want -1", skel.ParentIndex(0))
	}
	if skel.ParentIndex(1) != 0 {
		t.Errorf("child parent = %d, want 0", skel.ParentIndex(1))
	}

	// Forward references must be rejected.
	if got := skel.AddJoint("forward", 99); got != -1 {
		t.Errorf("AddJoint with bogus parent returned %d, want -1", got)
	}
}

// TestRagdoll15BoneHumanoidSettles spawns a 15-bone ragdoll, drops it from
// height, and checks that everything stays finite and bounded — the canonical
// "no constraint explosion" smoke test.
func TestRagdoll15BoneHumanoidSettles(t *testing.T) {
	if err := Init(); err != nil {
		t.Fatalf("Init: %v", err)
	}
	defer Shutdown()

	ps := NewPhysicsSystem()
	defer ps.Destroy()
	bi := ps.GetBodyInterface()

	// Static ground far below so the ragdoll can fall and accumulate motion
	// before potentially landing.
	groundShape := CreateBox(Vec3{X: 50, Y: 0.5, Z: 50})
	defer groundShape.Destroy()
	ground := bi.CreateBody(groundShape, Vec3{X: 0, Y: -2, Z: 0}, MotionTypeStatic, false)
	defer bi.RemoveAndDestroyBody(ground)

	// Build skeleton.
	skel := NewSkeleton()
	defer skel.Destroy()
	joints, shapes := build15BoneHumanoid()
	defer destroyShapes(shapes)

	if len(joints) != 15 {
		t.Fatalf("test setup: humanoid has %d bones, want 15", len(joints))
	}

	for _, j := range joints {
		if idx := skel.AddJoint(j.name, j.parent); idx < 0 {
			t.Fatalf("AddJoint(%q, parent=%d) failed", j.name, j.parent)
		}
	}
	if skel.JointCount() != 15 {
		t.Fatalf("skeleton has %d joints, want 15", skel.JointCount())
	}

	// Build settings.
	settings := NewRagdollSettings(skel)
	if settings == nil {
		t.Fatal("NewRagdollSettings returned nil")
	}
	defer settings.Destroy()

	for i, j := range joints {
		err := settings.SetPart(i, shapes[i], j.pos, j.orient,
			MotionTypeDynamic, ragdollObjectLayerMoving, 0 /* default mass */)
		if err != nil {
			t.Fatalf("SetPart[%d] (%s): %v", i, j.name, err)
		}
	}

	// Pivot is at this body's position offset by pivotOffset (so the joint
	// sits between the two bone segments).
	for i, j := range joints {
		if j.parent < 0 {
			continue
		}
		// Twist axis: from this body toward its parent's body.
		parentPos := joints[j.parent].pos
		dir := parentPos.Sub(j.pos).Normalize()
		// Plane axis: any axis perpendicular to the twist axis. Try Y, fall
		// back to X if the limb is already aligned with Y.
		plane := Vec3{X: 0, Y: 1, Z: 0}
		if math.Abs(float64(plane.Dot(dir))) > 0.9 {
			plane = Vec3{X: 1, Y: 0, Z: 0}
		}
		// Gram-Schmidt
		plane = plane.Sub(dir.Mul(plane.Dot(dir))).Normalize()

		err := settings.SetSwingTwistConstraint(i, SwingTwistLimits{
			Pivot:               j.pos.Add(j.pivotOffset),
			TwistAxis:           dir,
			PlaneAxis:           plane,
			NormalHalfConeAngle: j.swingNormal,
			PlaneHalfConeAngle:  j.swingPlane,
			TwistMin:            j.twistMin,
			TwistMax:            j.twistMax,
		})
		if err != nil {
			t.Fatalf("SetSwingTwistConstraint[%d] (%s): %v", i, j.name, err)
		}
	}

	if err := settings.Finalize(); err != nil {
		t.Fatalf("Finalize: %v", err)
	}

	rd := NewRagdoll(settings, ps, 1 /* groupID */)
	if rd == nil {
		t.Fatal("NewRagdoll returned nil")
	}
	defer func() {
		rd.RemoveFromPhysicsSystem()
		rd.Destroy()
	}()

	if got := rd.BodyCount(); got != 15 {
		t.Fatalf("ragdoll body count = %d, want 15", got)
	}

	rd.AddToPhysicsSystem(true)
	if !rd.IsActive() {
		t.Fatal("ragdoll should be active immediately after Add+activate")
	}

	// Simulate 2 seconds at 60 Hz.
	const dt = 1.0 / 60.0
	const steps = 120

	// Capture the IDs once so we don't allocate inside the inner loop.
	bodyIDs := make([]*BodyID, rd.BodyCount())
	for i := range bodyIDs {
		bodyIDs[i] = rd.BodyID(i)
	}
	defer func() {
		for _, id := range bodyIDs {
			if id != nil {
				id.Destroy()
			}
		}
	}()

	const explosionSpeed = 200.0 // m/s
	const positionBound = 50.0   // m from origin

	for step := range steps {
		ps.Update(dt)

		for i, id := range bodyIDs {
			pos := bi.GetPosition(id)
			vel := bi.GetLinearVelocity(id)

			if math.IsNaN(float64(pos.X)) || math.IsNaN(float64(pos.Y)) || math.IsNaN(float64(pos.Z)) {
				t.Fatalf("step %d body %d (%s): NaN position", step, i, joints[i].name)
			}
			if math.IsInf(float64(pos.X), 0) || math.IsInf(float64(pos.Y), 0) || math.IsInf(float64(pos.Z), 0) {
				t.Fatalf("step %d body %d (%s): Inf position", step, i, joints[i].name)
			}
			d := math.Sqrt(float64(pos.X*pos.X + pos.Y*pos.Y + pos.Z*pos.Z))
			if d > positionBound {
				t.Fatalf("step %d body %d (%s): position %v drifted to distance %.1f (>%g) from origin",
					step, i, joints[i].name, pos, d, positionBound)
			}
			speed := math.Sqrt(float64(vel.X*vel.X + vel.Y*vel.Y + vel.Z*vel.Z))
			if speed > explosionSpeed {
				t.Fatalf("step %d body %d (%s): linear speed %.1f m/s > %.0f (constraint explosion)",
					step, i, joints[i].name, speed, explosionSpeed)
			}
		}
	}

	// Joint-limit sanity check: every connected child body should still be
	// within roughly the expected distance of its parent. The SwingTwist
	// limits are angular, so we just verify the constraint kept bones from
	// flying apart (which would mean the constraint failed).
	for i, j := range joints {
		if j.parent < 0 {
			continue
		}
		childPos := bi.GetPosition(bodyIDs[i])
		parentPos := bi.GetPosition(bodyIDs[j.parent])
		dx := childPos.X - parentPos.X
		dy := childPos.Y - parentPos.Y
		dz := childPos.Z - parentPos.Z
		dist := float32(math.Sqrt(float64(dx*dx + dy*dy + dz*dz)))
		// At rest, child sits ~|pos - parent.pos| away; allow generous slack
		// since the body has been settling under gravity.
		initialDx := j.pos.X - joints[j.parent].pos.X
		initialDy := j.pos.Y - joints[j.parent].pos.Y
		initialDz := j.pos.Z - joints[j.parent].pos.Z
		initialDist := float32(math.Sqrt(float64(initialDx*initialDx + initialDy*initialDy + initialDz*initialDz)))
		if dist > initialDist+0.5 {
			t.Errorf("body %d (%s) drifted from parent %d (%s): %.2f m (initial %.2f m + 0.5 slack)",
				i, j.name, j.parent, joints[j.parent].name, dist, initialDist)
		}
	}
}

// TestRagdollSetSwingTwistOnRootRejected verifies that attempting to add a
// swing/twist constraint to the root joint (no parent) returns an error.
func TestRagdollSetSwingTwistOnRootRejected(t *testing.T) {
	skel := NewSkeleton()
	defer skel.Destroy()
	skel.AddJoint("root", -1)
	skel.AddJoint("child", 0)

	settings := NewRagdollSettings(skel)
	defer settings.Destroy()

	err := settings.SetSwingTwistConstraint(0, SwingTwistLimits{
		TwistAxis: Vec3{X: 0, Y: 1, Z: 0},
		PlaneAxis: Vec3{X: 1, Y: 0, Z: 0},
	})
	if err == nil {
		t.Fatal("SetSwingTwistConstraint on root joint should fail")
	}
}
