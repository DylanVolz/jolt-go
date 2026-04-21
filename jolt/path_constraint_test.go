package jolt

import (
	"math"
	"testing"
)

// buildCircularLoop builds a 10-segment closed loop (circle in the XZ plane) of
// Hermite control points. Each point has tangent pointing along the direction of
// travel and normal pointing upward.
func buildCircularLoop(radius float32, n int) []HermitePathPoint {
	points := make([]HermitePathPoint, n)
	// For a unit circle param theta, d(cos theta)/dtheta = -sin theta,
	// d(sin theta)/dtheta = cos theta. Tangent length scales the cubic-Hermite
	// segment; pick a length that roughly matches arc length per segment so the
	// interpolated curve approximates a true circle.
	segmentAngle := float32(2 * math.Pi / float64(n))
	tangentScale := radius * segmentAngle
	for i := range n {
		theta := float64(i) * float64(segmentAngle)
		cos, sin := float32(math.Cos(theta)), float32(math.Sin(theta))
		points[i] = HermitePathPoint{
			Position: Vec3{X: radius * cos, Y: 0, Z: radius * sin},
			Tangent:  Vec3{X: -tangentScale * sin, Y: 0, Z: tangentScale * cos},
			Normal:   Vec3{X: 0, Y: 1, Z: 0},
		}
	}
	return points
}

func TestHermitePathBasics(t *testing.T) {
	points := buildCircularLoop(5.0, 10)
	path := CreateHermitePath(points, true)
	if path == nil {
		t.Fatal("CreateHermitePath returned nil")
	}
	defer path.Destroy()

	// Looping 10-point path → max fraction == 10
	if got := path.MaxFraction(); math.Abs(float64(got-10.0)) > 1e-4 {
		t.Errorf("MaxFraction = %.4f, want 10.0", got)
	}

	// Sampled positions should lie on the circle of radius 5 in the XZ plane.
	for _, frac := range []float32{0, 2.5, 5, 7.5, 9.9} {
		pos, tan := path.PointOnPath(frac)
		r := float32(math.Sqrt(float64(pos.X*pos.X + pos.Z*pos.Z)))
		if math.Abs(float64(r-5.0)) > 0.25 {
			t.Errorf("fraction %.2f: radius = %.4f, want ~5.0", frac, r)
		}
		if math.Abs(float64(pos.Y)) > 1e-3 {
			t.Errorf("fraction %.2f: Y = %.4f, want 0", frac, pos.Y)
		}
		if tan.Length() == 0 {
			t.Errorf("fraction %.2f: zero tangent", frac)
		}
	}
}

func TestHermitePathRejectsTooFewPoints(t *testing.T) {
	if p := CreateHermitePath([]HermitePathPoint{}, false); p != nil {
		t.Error("expected nil for 0 points")
	}
	if p := CreateHermitePath([]HermitePathPoint{{}}, false); p != nil {
		t.Error("expected nil for 1 point")
	}
}

// TestPathConstraintCartOnLoop verifies the acceptance criterion:
// a dynamic cart attached to a 10-segment closed-loop Hermite path stays on
// the rail under motor drive, and the motor's velocity clamp is respected on
// curved segments.
func TestPathConstraintCartOnLoop(t *testing.T) {
	if err := Init(); err != nil {
		t.Fatal(err)
	}
	defer Shutdown()

	ps := NewPhysicsSystem()
	defer ps.Destroy()
	bi := ps.GetBodyInterface()

	// Static anchor at origin — the path is defined in its local space.
	anchorShape := CreateBox(Vec3{X: 0.1, Y: 0.1, Z: 0.1})
	defer anchorShape.Destroy()
	anchor := bi.CreateBody(anchorShape, Vec3{X: 0, Y: 0, Z: 0}, MotionTypeStatic, false)
	defer bi.RemoveAndDestroyBody(anchor)

	// Dynamic cart that will ride the rail. Start it at the first path point.
	radius := float32(5.0)
	cartShape := CreateBox(Vec3{X: 0.2, Y: 0.2, Z: 0.2})
	defer cartShape.Destroy()
	cart := bi.CreateBody(cartShape, Vec3{X: radius, Y: 0, Z: 0}, MotionTypeDynamic, false)
	defer bi.RemoveAndDestroyBody(cart)
	// Disable gravity so the only force on the cart is the path motor — makes
	// the "stays on rail" assertion unambiguous.
	bi.SetGravityFactor(cart, 0)
	bi.ActivateBody(cart)

	// 10-point looping circle.
	points := buildCircularLoop(radius, 10)
	path := CreateHermitePath(points, true)
	if path == nil {
		t.Fatal("CreateHermitePath returned nil")
	}
	defer path.Destroy()

	constraint := ps.CreatePathConstraint(
		anchor, cart,
		path,
		Vec3{X: 0, Y: 0, Z: 0}, // path origin == anchor origin
		QuatIdentity(),
		0.0, // start at fraction 0 (= first point: (radius,0,0))
		PathRotationFree,
		0.0, // no friction
	)
	if constraint == nil {
		t.Fatal("CreatePathConstraint returned nil")
	}
	defer constraint.Destroy()
	ps.AddConstraint(constraint)
	defer ps.RemoveConstraint(constraint)

	// Target linear speed along the path tangent, in m/s. Jolt normalizes the
	// tangent when solving the motor constraint, so target velocity is body
	// speed, not fraction/sec.
	const targetVel float32 = 3.0
	const maxForce = 1e6
	const maxTorque = 1e6
	constraint.PathSetPositionMotorSettings(2.0, 1.0, maxForce, maxTorque)
	constraint.PathSetPositionMotorState(MotorStateVelocity)
	constraint.PathSetTargetVelocity(targetVel)

	if got := constraint.PathGetTargetVelocity(); math.Abs(float64(got-targetVel)) > 1e-4 {
		t.Errorf("PathGetTargetVelocity = %.3f, want %.3f", got, targetVel)
	}
	if got := constraint.PathGetPositionMotorState(); got != MotorStateVelocity {
		t.Errorf("PathGetPositionMotorState = %v, want %v", got, MotorStateVelocity)
	}

	dt := float32(1.0 / 60.0)
	const steps = 600 // 10 seconds

	maxOffRail := float32(0)
	maxSpeed := float32(0)
	speedSum := float32(0)
	speedSamples := 0
	for i := range steps {
		ps.Update(dt)

		// Skip the first handful of steps — the motor ramps up over its spring
		// period before the cart settles to the target speed.
		if i < 60 {
			continue
		}

		pos := bi.GetPosition(cart)
		// On-rail check: cart must stay close to the circle of radius `radius`
		// in the XZ plane. Hermite interpolation of a 10-point polygon is not a
		// perfect circle, but stays within a narrow envelope.
		r := float32(math.Sqrt(float64(pos.X*pos.X + pos.Z*pos.Z)))
		off := float32(math.Abs(float64(r - radius)))
		if off > maxOffRail {
			maxOffRail = off
		}
		if math.Abs(float64(pos.Y)) > 0.25 {
			t.Fatalf("step %d: cart drifted off rail in Y (%.3f)", i, pos.Y)
		}

		vel := bi.GetLinearVelocity(cart)
		speed := vel.Length()
		if speed > maxSpeed {
			maxSpeed = speed
		}
		speedSum += speed
		speedSamples++
	}

	if maxOffRail > 0.5 {
		t.Errorf("cart drifted off rail: max radial offset = %.3f m", maxOffRail)
	}

	// Motor should clamp linear speed near targetVel. Allow ±25% headroom for
	// spring/damper ringing and corner geometry.
	if maxSpeed > targetVel*1.25 {
		t.Errorf("cart exceeded max-speed envelope: got %.3f, expected ≤ %.3f", maxSpeed, targetVel*1.25)
	}
	avgSpeed := speedSum / float32(speedSamples)
	if avgSpeed < targetVel*0.75 {
		t.Errorf("cart average speed below target: got %.3f, expected ≥ %.3f", avgSpeed, targetVel*0.75)
	}

	// Path fraction must have advanced well past the starting point — the cart
	// should have traversed multiple loops in ~10 s. MaxFraction is 10.
	finalFraction := constraint.PathGetPathFraction()
	if finalFraction <= 1.0 {
		t.Errorf("cart did not progress along path: final fraction = %.3f", finalFraction)
	}

	finalPos := bi.GetPosition(cart)
	finalR := float32(math.Sqrt(float64(finalPos.X*finalPos.X + finalPos.Z*finalPos.Z)))
	if math.Abs(float64(finalR-radius)) > 0.5 {
		t.Errorf("final cart position off rail: radius = %.3f, want ~%.3f", finalR, radius)
	}
}
