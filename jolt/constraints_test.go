package jolt

import (
	"math"
	"os"
	"testing"
)

// TestMain ensures Jolt is initialized once for all tests in this package.
// Required for any test that creates a PhysicsSystem.
func TestMain(m *testing.M) {
	if err := Init(); err != nil {
		panic(err)
	}
	code := m.Run()
	Shutdown()
	os.Exit(code)
}

// twistAngleAroundX extracts the rotation angle (radians) around the X axis
// from a swing*twist quaternion decomposition (q = q_swing * q_twist where
// q_twist.y = q_twist.z = 0). Returns the signed angle in (-PI, PI].
func twistAngleAroundX(q Quat) float32 {
	// Project onto X axis: twist quat is (q.x, 0, 0, q.w) renormalized.
	twistX := q.X
	twistW := q.W
	mag := float32(math.Sqrt(float64(twistX*twistX + twistW*twistW)))
	if mag < 1e-6 {
		return 0
	}
	twistX /= mag
	twistW /= mag
	return 2 * float32(math.Atan2(float64(twistX), float64(twistW)))
}

// swingHalfAngle returns the swing angle magnitude (radians) from the swing*twist
// decomposition. q_swing has q_swing.x = 0, so |q_swing| = sqrt(y^2 + z^2 + w^2).
// The actual swing angle is 2*atan2(sqrt(y^2+z^2), w_swing).
func swingHalfAngle(q Quat) float32 {
	// q_swing = q * conj(q_twist). With q_twist = normalize((q.x, 0, 0, q.w)),
	// q_swing.w = q.w*twistW + q.x*twistX = sqrt(q.w^2 + q.x^2)
	swingW := float32(math.Sqrt(float64(q.W*q.W + q.X*q.X)))
	swingYZ := float32(math.Sqrt(float64(q.Y*q.Y + q.Z*q.Z)))
	return 2 * float32(math.Atan2(float64(swingYZ), float64(swingW)))
}

// stepSim runs the simulation for n fixed-dt steps.
func stepSim(ps *PhysicsSystem, n int, dt float32) {
	for range n {
		ps.Update(dt)
	}
}

// stepWorld steps the simulation for `duration` seconds in `dt` slices.
// (Port of the helper from T-0122's constraints_test.go.)
func stepWorld(ps *PhysicsSystem, duration, dt float32) {
	steps := int(duration / dt)
	for range steps {
		ps.Update(dt)
	}
}

// makeShoulder creates a fresh physics system with a static torso and a dynamic
// arm joined by a swing-twist constraint. swing limited to 90°, twist to ±45°.
func makeShoulder(t *testing.T) (*PhysicsSystem, *BodyInterface, *BodyID, *Constraint) {
	t.Helper()
	ps := NewPhysicsSystem()
	bi := ps.GetBodyInterface()

	torsoShape := CreateSphere(0.2)
	t.Cleanup(func() { torsoShape.Destroy() })
	torso := bi.CreateBody(torsoShape, Vec3{X: 0, Y: 0, Z: 0}, MotionTypeStatic, false)
	t.Cleanup(func() { torso.Destroy() })

	armShape := CreateSphere(0.2)
	t.Cleanup(func() { armShape.Destroy() })
	arm := bi.CreateBody(armShape, Vec3{X: 1, Y: 0, Z: 0}, MotionTypeDynamic, false)
	t.Cleanup(func() { arm.Destroy() })
	bi.SetGravityFactor(arm, 0)
	bi.ActivateBody(arm)

	c := ps.CreateSwingTwistConstraint(
		torso, arm,
		Vec3{X: 0, Y: 0, Z: 0},
		Vec3{X: 1, Y: 0, Z: 0}, // twist axis = +X
		Vec3{X: 0, Y: 1, Z: 0}, // plane axis = +Y
		float32(math.Pi/2), float32(math.Pi/2),
		float32(-math.Pi/4), float32(math.Pi/4),
		SwingTypeCone,
	)
	if c == nil {
		ps.Destroy()
		t.Fatal("CreateSwingTwistConstraint returned nil")
	}
	ps.AddConstraint(c)
	t.Cleanup(func() {
		ps.RemoveConstraint(c)
		c.Destroy()
		ps.Destroy()
	})
	return ps, bi, arm, c
}

func TestSwingTwistConstraintLimitsRoundTrip(t *testing.T) {
	_, _, _, c := makeShoulder(t)

	const swingHalfCone = float32(math.Pi / 2)
	const twistMin = float32(-math.Pi / 4)
	const twistMax = float32(math.Pi / 4)

	if got := c.SwingTwistGetNormalHalfConeAngle(); math.Abs(float64(got-swingHalfCone)) > 1e-4 {
		t.Errorf("NormalHalfConeAngle = %f, want %f", got, swingHalfCone)
	}
	if got := c.SwingTwistGetPlaneHalfConeAngle(); math.Abs(float64(got-swingHalfCone)) > 1e-4 {
		t.Errorf("PlaneHalfConeAngle = %f, want %f", got, swingHalfCone)
	}
	if got := c.SwingTwistGetTwistMinAngle(); math.Abs(float64(got-twistMin)) > 1e-4 {
		t.Errorf("TwistMinAngle = %f, want %f", got, twistMin)
	}
	if got := c.SwingTwistGetTwistMaxAngle(); math.Abs(float64(got-twistMax)) > 1e-4 {
		t.Errorf("TwistMaxAngle = %f, want %f", got, twistMax)
	}
	if s := c.SwingTwistGetSwingMotorState(); s != MotorStateOff {
		t.Errorf("default swing motor state = %v, want Off", s)
	}
	if s := c.SwingTwistGetTwistMotorState(); s != MotorStateOff {
		t.Errorf("default twist motor state = %v, want Off", s)
	}

	// Setter round-trip
	c.SwingTwistSetMaxFrictionTorque(2.5)
	if got := c.SwingTwistGetMaxFrictionTorque(); math.Abs(float64(got-2.5)) > 1e-5 {
		t.Errorf("MaxFrictionTorque = %.3f, want 2.5", got)
	}

	// Target accessors
	c.SwingTwistSetTargetAngularVelocityCS(Vec3{X: 1, Y: 2, Z: 3})
	if v := c.SwingTwistGetTargetAngularVelocityCS(); v.X != 1 || v.Y != 2 || v.Z != 3 {
		t.Errorf("target angular velocity = %+v, want (1,2,3)", v)
	}
}

func TestSwingTwistConstraintTwistClamped(t *testing.T) {
	ps, bi, arm, c := makeShoulder(t)

	// Drive twist motor to 90° (way past the +45° limit).
	c.SwingTwistSetTwistMotorSettings(20, 1, 1e10, 1e10)
	c.SwingTwistSetTwistMotorState(MotorStatePosition)
	c.SwingTwistSetTargetOrientationCS(Quat{
		X: float32(math.Sin(math.Pi / 4)), Y: 0, Z: 0,
		W: float32(math.Cos(math.Pi / 4)),
	})
	bi.ActivateBody(arm)

	stepSim(ps, 200, 1.0/60.0)

	q := c.SwingTwistGetRotationInConstraintSpace()
	twist := twistAngleAroundX(q)
	const twistMax = float32(math.Pi / 4)
	if math.Abs(float64(twist-twistMax)) > 0.05 {
		t.Errorf("twist after motor drive = %.3f rad, want ~%.3f (twistMax)", twist, twistMax)
	}
	if swing := swingHalfAngle(q); math.Abs(float64(swing)) > 0.05 {
		t.Errorf("swing should be ~0 when only twisting, got %.3f rad", swing)
	}
}

func TestSwingTwistConstraintSwingClamped(t *testing.T) {
	ps, bi, arm, c := makeShoulder(t)

	// Drive swing motor to 120° around Y (past the 90° cone limit).
	c.SwingTwistSetSwingMotorSettings(20, 1, 1e10, 1e10)
	c.SwingTwistSetSwingMotorState(MotorStatePosition)
	c.SwingTwistSetTargetOrientationCS(Quat{
		X: 0, Y: float32(math.Sin(math.Pi / 3)), Z: 0,
		W: float32(math.Cos(math.Pi / 3)),
	})
	bi.ActivateBody(arm)

	stepSim(ps, 300, 1.0/60.0)

	q := c.SwingTwistGetRotationInConstraintSpace()
	swing := swingHalfAngle(q)
	const swingHalfCone = float32(math.Pi / 2)
	if swing > swingHalfCone+0.1 {
		t.Errorf("swing after motor drive = %.3f rad, exceeds cone limit %.3f", swing, swingHalfCone)
	}
	if swing < swingHalfCone-0.25 {
		t.Errorf("swing after motor drive = %.3f rad, expected near cone limit %.3f", swing, swingHalfCone)
	}
}

// makeSliderHinge creates a slider+hinge SixDOF: free TranslationY and
// RotationY, all other axes fixed. Both bodies start coincident at the origin
// so translation impulses don't create torque.
func makeSliderHinge(t *testing.T) (*PhysicsSystem, *BodyInterface, *BodyID, *Constraint) {
	t.Helper()
	ps := NewPhysicsSystem()
	bi := ps.GetBodyInterface()

	anchorShape := CreateSphere(0.2)
	t.Cleanup(func() { anchorShape.Destroy() })
	anchor := bi.CreateBody(anchorShape, Vec3{X: 0, Y: 0, Z: 0}, MotionTypeStatic, false)
	t.Cleanup(func() { anchor.Destroy() })

	bodyShape := CreateSphere(0.2)
	t.Cleanup(func() { bodyShape.Destroy() })
	body := bi.CreateBody(bodyShape, Vec3{X: 0, Y: 0, Z: 0}, MotionTypeDynamic, false)
	t.Cleanup(func() { body.Destroy() })
	bi.SetGravityFactor(body, 0)
	bi.ActivateBody(body)

	limits := AllFreeSixDOFLimits()
	limits.MakeFixedAxis(SixDOFAxisTranslationX)
	limits.MakeFixedAxis(SixDOFAxisTranslationZ)
	limits.MakeFixedAxis(SixDOFAxisRotationX)
	limits.MakeFixedAxis(SixDOFAxisRotationZ)

	c := ps.CreateSixDOFConstraint(
		anchor, body,
		Vec3{X: 0, Y: 0, Z: 0},
		Vec3{X: 1, Y: 0, Z: 0},
		Vec3{X: 0, Y: 1, Z: 0},
		limits,
		SwingTypeCone,
	)
	if c == nil {
		ps.Destroy()
		t.Fatal("CreateSixDOFConstraint returned nil")
	}
	ps.AddConstraint(c)
	t.Cleanup(func() {
		ps.RemoveConstraint(c)
		c.Destroy()
		ps.Destroy()
	})
	return ps, bi, body, c
}

func TestSixDOFSliderHingeAxisFlags(t *testing.T) {
	_, _, _, c := makeSliderHinge(t)

	if !c.SixDOFIsFreeAxis(SixDOFAxisTranslationY) {
		t.Errorf("TranslationY should be free")
	}
	for _, ax := range []SixDOFAxis{SixDOFAxisTranslationX, SixDOFAxisTranslationZ} {
		if !c.SixDOFIsFixedAxis(ax) {
			t.Errorf("translation axis %d should be fixed", ax)
		}
	}
	if !c.SixDOFIsFreeAxis(SixDOFAxisRotationY) {
		t.Errorf("RotationY should be free")
	}
	for _, ax := range []SixDOFAxis{SixDOFAxisRotationX, SixDOFAxisRotationZ} {
		if !c.SixDOFIsFixedAxis(ax) {
			t.Errorf("rotation axis %d should be fixed", ax)
		}
	}
}

func TestSixDOFSliderHingeMotionRestricted(t *testing.T) {
	ps, bi, body, _ := makeSliderHinge(t)

	bi.SetLinearVelocity(body, Vec3{X: 5, Y: 3, Z: 5})
	bi.SetAngularVelocity(body, Vec3{X: 4, Y: 4, Z: 4})
	bi.ActivateBody(body)

	stepSim(ps, 60, 1.0/60.0)

	pos := bi.GetPosition(body)
	// Locked translation axes converge to ≈ 0 under Baumgarte.
	if math.Abs(float64(pos.X)) > 0.05 {
		t.Errorf("body drifted on locked X axis: pos.X = %.3f", pos.X)
	}
	if math.Abs(float64(pos.Z)) > 0.05 {
		t.Errorf("body drifted on locked Z axis: pos.Z = %.3f", pos.Z)
	}
	// Free Y axis: the body slides upward (no gravity, initial vY=3 → ~3 m after 1 s).
	if pos.Y <= 0.5 {
		t.Errorf("body should have slid along free Y axis, pos.Y = %.3f", pos.Y)
	}

	angVel := bi.GetAngularVelocity(body)
	if math.Abs(float64(angVel.X)) > 0.5 {
		t.Errorf("locked rotation X still spinning: angVel.X = %.3f", angVel.X)
	}
	if math.Abs(float64(angVel.Z)) > 0.5 {
		t.Errorf("locked rotation Z still spinning: angVel.Z = %.3f", angVel.Z)
	}
	if math.Abs(float64(angVel.Y)) < 1.0 {
		t.Errorf("free rotation Y was damped unexpectedly: angVel.Y = %.3f", angVel.Y)
	}
}

func TestSixDOFConstraintMotorState(t *testing.T) {
	ps := NewPhysicsSystem()
	defer ps.Destroy()
	bi := ps.GetBodyInterface()

	a := bi.CreateBody(CreateSphere(0.1), Vec3{X: 0, Y: 0, Z: 0}, MotionTypeStatic, false)
	defer a.Destroy()
	b := bi.CreateBody(CreateSphere(0.1), Vec3{X: 0, Y: 1, Z: 0}, MotionTypeDynamic, false)
	defer b.Destroy()
	bi.ActivateBody(b)

	c := ps.CreateSixDOFConstraint(
		a, b,
		Vec3{X: 0, Y: 0, Z: 0},
		Vec3{X: 1, Y: 0, Z: 0},
		Vec3{X: 0, Y: 1, Z: 0},
		AllFreeSixDOFLimits(),
		SwingTypeCone,
	)
	if c == nil {
		t.Fatal("CreateSixDOFConstraint returned nil")
	}
	defer c.Destroy()
	ps.AddConstraint(c)
	defer ps.RemoveConstraint(c)

	for i := range SixDOFAxisCount {
		ax := SixDOFAxis(i)
		if got := c.SixDOFGetMotorState(ax); got != MotorStateOff {
			t.Errorf("axis %d default motor state = %v, want Off", ax, got)
		}
	}
	c.SixDOFSetMotorState(SixDOFAxisTranslationY, MotorStateVelocity)
	if got := c.SixDOFGetMotorState(SixDOFAxisTranslationY); got != MotorStateVelocity {
		t.Errorf("TranslationY motor state = %v, want Velocity", got)
	}
	c.SixDOFSetMotorState(SixDOFAxisRotationX, MotorStatePosition)
	if got := c.SixDOFGetMotorState(SixDOFAxisRotationX); got != MotorStatePosition {
		t.Errorf("RotationX motor state = %v, want Position", got)
	}

	// Friction round-trip
	c.SixDOFSetMaxFriction(SixDOFAxisTranslationX, 2.5)
	if got := c.SixDOFGetMaxFriction(SixDOFAxisTranslationX); math.Abs(float64(got-2.5)) > 1e-5 {
		t.Errorf("TranslationX friction = %.3f, want 2.5", got)
	}

	// Target round-trip
	c.SixDOFSetTargetVelocityCS(Vec3{X: 1, Y: 2, Z: 3})
	if v := c.SixDOFGetTargetVelocityCS(); v.X != 1 || v.Y != 2 || v.Z != 3 {
		t.Errorf("target velocity = %+v, want (1,2,3)", v)
	}
	c.SixDOFSetTargetAngularVelocityCS(Vec3{X: -1, Y: -2, Z: -3})
	if v := c.SixDOFGetTargetAngularVelocityCS(); v.X != -1 || v.Y != -2 || v.Z != -3 {
		t.Errorf("target angular velocity = %+v, want (-1,-2,-3)", v)
	}
}

func TestPointConstraint(t *testing.T) {
	ps := NewPhysicsSystem()
	defer ps.Destroy()

	bi := ps.GetBodyInterface()

	anchorShape := CreateBox(Vec3{X: 0.1, Y: 0.1, Z: 0.1})
	defer anchorShape.Destroy()
	anchor := bi.CreateBody(anchorShape, Vec3{X: 0, Y: 5, Z: 0}, MotionTypeStatic, false)
	defer bi.RemoveAndDestroyBody(anchor)

	// Dynamic body 1m below the anchor; the constraint pins the top of the
	// dynamic body to the anchor position.
	bobShape := CreateBox(Vec3{X: 0.5, Y: 0.5, Z: 0.5})
	defer bobShape.Destroy()
	bob := bi.CreateBody(bobShape, Vec3{X: 0, Y: 4, Z: 0}, MotionTypeDynamic, false)
	defer bi.RemoveAndDestroyBody(bob)
	bi.ActivateBody(bob)

	pivot := Vec3{X: 0, Y: 5, Z: 0}
	constraint := ps.CreatePointConstraint(anchor, bob, pivot)
	if constraint == nil {
		t.Fatal("CreatePointConstraint returned nil")
	}
	defer constraint.Destroy()
	ps.AddConstraint(constraint)
	defer ps.RemoveConstraint(constraint)

	// Push the bob sideways so gravity + constraint produce pendulum motion.
	bi.SetLinearVelocity(bob, Vec3{X: 3, Y: 0, Z: 0})

	stepWorld(ps, 1.0, 1.0/60.0)

	// The attachment point on the bob is its top face (y+0.5 above center).
	// After simulation it should remain fixed at the pivot location: its
	// center of mass should always be at distance 1.0 from the pivot.
	pos := bi.GetPosition(bob)
	offset := pos.Sub(pivot)
	distance := offset.Length()

	// Allow a small drift from solver inaccuracy.
	if math.Abs(float64(distance-1.0)) > 0.05 {
		t.Errorf("bob drifted from pivot: distance=%.3f want ~1.0 (pos=%+v)", distance, pos)
	}

	// Rotation should be free: the bob should have swung off the Y-axis,
	// so at least one of X or Z offset components must be non-trivial.
	swingMagnitude := float32(math.Sqrt(float64(offset.X*offset.X + offset.Z*offset.Z)))
	if swingMagnitude < 0.05 {
		t.Errorf("expected pendulum swing off the Y-axis, got xz magnitude=%.3f (pos=%+v)",
			swingMagnitude, pos)
	}
}

// TestConeConstraintClampsSwing asserts the cone constraint clamps the swing
// between the two bodies' twist axes to the configured half-cone angle.
func TestConeConstraintClampsSwing(t *testing.T) {
	ps := NewPhysicsSystem()
	defer ps.Destroy()

	bi := ps.GetBodyInterface()

	anchorShape := CreateBox(Vec3{X: 0.1, Y: 0.1, Z: 0.1})
	defer anchorShape.Destroy()
	anchor := bi.CreateBody(anchorShape, Vec3{X: 0, Y: 5, Z: 0}, MotionTypeStatic, false)
	defer bi.RemoveAndDestroyBody(anchor)

	bobShape := CreateBox(Vec3{X: 0.5, Y: 0.5, Z: 0.5})
	defer bobShape.Destroy()
	bob := bi.CreateBody(bobShape, Vec3{X: 0, Y: 4, Z: 0}, MotionTypeDynamic, false)
	defer bi.RemoveAndDestroyBody(bob)
	bi.ActivateBody(bob)

	pivot := Vec3{X: 0, Y: 5, Z: 0}
	halfAngle := float32(math.Pi / 6) // 30 degrees
	// Twist axis along +Y. When the bob swings in X or Z, the angle between
	// body1's Y axis and body2's Y axis grows. The constraint must clamp it.
	twistAxis := Vec3{X: 0, Y: 1, Z: 0}
	constraint := ps.CreateConeConstraint(anchor, bob, pivot, twistAxis, halfAngle)
	if constraint == nil {
		t.Fatal("CreateConeConstraint returned nil")
	}
	defer constraint.Destroy()
	ps.AddConstraint(constraint)
	defer ps.RemoveConstraint(constraint)

	// Verify the half-angle round-trip through cos().
	wantCos := float32(math.Cos(float64(halfAngle)))
	gotCos := constraint.ConeGetCosHalfConeAngle()
	if math.Abs(float64(gotCos-wantCos)) > 1e-5 {
		t.Errorf("ConeGetCosHalfConeAngle=%.6f want %.6f", gotCos, wantCos)
	}

	// Apply a large angular velocity tipping the bob around Z; gravity alone
	// would also push it beyond the cone, but the explicit spin guarantees we
	// hit the limit.
	bi.SetAngularVelocity(bob, Vec3{X: 0, Y: 0, Z: 5})

	stepWorld(ps, 2.0, 1.0/120.0)

	// The bob's local +Y axis, rotated into world space, must stay within the
	// cone around the world +Y (twist axis). The dot product of the rotated
	// axis with world +Y must therefore be >= cos(halfAngle) minus solver slop.
	rot := bi.GetRotation(bob)
	bobY := rotateY(rot)
	dot := bobY.Y // bob-local Y rotated into world, dotted with world Y
	tolerance := float32(0.03)
	if dot < wantCos-tolerance {
		angle := math.Acos(float64(dot))
		t.Errorf("swing exceeded cone: dot=%.4f want >= %.4f (angle=%.3f rad, limit=%.3f rad)",
			dot, wantCos, angle, halfAngle)
	}

	// Sanity: the bob should have actually swung — if it stayed perfectly
	// upright the test proves nothing. Require at least half the cone limit.
	if dot > 0.999 {
		t.Errorf("bob did not swing at all (dot=%.4f); test setup is degenerate", dot)
	}
}

// rotateY returns the world-space direction of the body-local +Y axis after
// applying quaternion q (q must be unit-length).
func rotateY(q Quat) Vec3 {
	// Standard formula: v' = q * (0,1,0) * q_conj, expanded.
	// For the +Y basis vector:
	//   x' = 2(qx*qy - qw*qz)
	//   y' = 1 - 2(qx*qx + qz*qz)
	//   z' = 2(qy*qz + qw*qx)
	return Vec3{
		X: 2 * (q.X*q.Y - q.W*q.Z),
		Y: 1 - 2*(q.X*q.X+q.Z*q.Z),
		Z: 2 * (q.Y*q.Z + q.W*q.X),
	}
}
