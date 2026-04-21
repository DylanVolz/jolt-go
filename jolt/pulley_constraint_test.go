package jolt

import (
	"math"
	"testing"
)

// TestPulleyConstraintAccessors verifies the pulley length accessors report
// the values passed to CreatePulleyConstraint and survive round-tripping
// through SetLength.
func TestPulleyConstraintAccessors(t *testing.T) {
	if err := Init(); err != nil {
		t.Fatal(err)
	}
	defer Shutdown()

	ps := NewPhysicsSystem()
	defer ps.Destroy()
	bi := ps.GetBodyInterface()

	shape := CreateBox(Vec3{X: 0.5, Y: 0.5, Z: 0.5})
	defer shape.Destroy()

	body1 := bi.CreateBody(shape, Vec3{X: -1, Y: 5, Z: 0}, MotionTypeDynamic, false)
	body2 := bi.CreateBody(shape, Vec3{X: 1, Y: 5, Z: 0}, MotionTypeDynamic, false)
	defer bi.RemoveAndDestroyBody(body1)
	defer bi.RemoveAndDestroyBody(body2)

	// Fixed pulley point 5m above each body. Both bodies attached at their center.
	fixed1 := Vec3{X: -1, Y: 10, Z: 0}
	fixed2 := Vec3{X: 1, Y: 10, Z: 0}

	c := ps.CreatePulleyConstraint(
		body1, body2,
		Vec3{X: -1, Y: 5, Z: 0}, Vec3{X: 1, Y: 5, Z: 0},
		fixed1, fixed2,
		1.0,
		3.0, 7.0,
	)
	if c == nil {
		t.Fatal("CreatePulleyConstraint returned nil")
	}
	defer c.Destroy()
	ps.AddConstraint(c)
	defer ps.RemoveConstraint(c)

	if got := c.PulleyGetMinLength(); math.Abs(float64(got-3.0)) > 1e-4 {
		t.Errorf("MinLength = %f, want 3.0", got)
	}
	if got := c.PulleyGetMaxLength(); math.Abs(float64(got-7.0)) > 1e-4 {
		t.Errorf("MaxLength = %f, want 7.0", got)
	}

	// Each body is 5m below its fixed point; segment length 5 + 1*5 = 10.
	if got := c.PulleyGetCurrentLength(); math.Abs(float64(got-10.0)) > 1e-3 {
		t.Errorf("CurrentLength = %f, want ~10.0", got)
	}

	c.PulleySetLength(2.0, 8.0)
	if got := c.PulleyGetMinLength(); math.Abs(float64(got-2.0)) > 1e-4 {
		t.Errorf("after SetLength: MinLength = %f, want 2.0", got)
	}
	if got := c.PulleyGetMaxLength(); math.Abs(float64(got-8.0)) > 1e-4 {
		t.Errorf("after SetLength: MaxLength = %f, want 8.0", got)
	}
}

// TestPulleyConstraintCouplesBodies verifies the rope coupling: with a rigid
// rope (min == max) and unequal masses under gravity, the heavier body falls
// and pulls the lighter one up by a matching amount.
func TestPulleyConstraintCouplesBodies(t *testing.T) {
	if err := Init(); err != nil {
		t.Fatal(err)
	}
	defer Shutdown()

	ps := NewPhysicsSystem()
	defer ps.Destroy()
	bi := ps.GetBodyInterface()

	// Unequal masses via unequal box volumes (Jolt defaults mass = density*volume).
	// heavy: 2x2x2 (volume 8), light: 1x1x1 (volume 1) — ~8:1 mass ratio.
	heavyShape := CreateBox(Vec3{X: 1, Y: 1, Z: 1})
	defer heavyShape.Destroy()
	lightShape := CreateBox(Vec3{X: 0.5, Y: 0.5, Z: 0.5})
	defer lightShape.Destroy()

	startY := float32(5.0)
	heavy := bi.CreateBody(heavyShape, Vec3{X: -2, Y: startY, Z: 0}, MotionTypeDynamic, false)
	light := bi.CreateBody(lightShape, Vec3{X: 2, Y: startY, Z: 0}, MotionTypeDynamic, false)
	defer bi.RemoveAndDestroyBody(heavy)
	defer bi.RemoveAndDestroyBody(light)
	bi.ActivateBody(heavy)
	bi.ActivateBody(light)

	// Pulley anchors directly above each body at Y=10.
	fixedHeavy := Vec3{X: -2, Y: 10, Z: 0}
	fixedLight := Vec3{X: 2, Y: 10, Z: 0}

	// Rigid rope: min == max == initial combined length (auto-detect via -1).
	c := ps.CreatePulleyConstraint(
		heavy, light,
		Vec3{X: -2, Y: startY, Z: 0}, Vec3{X: 2, Y: startY, Z: 0},
		fixedHeavy, fixedLight,
		1.0,
		-1, -1,
	)
	if c == nil {
		t.Fatal("CreatePulleyConstraint returned nil")
	}
	defer c.Destroy()
	ps.AddConstraint(c)
	defer ps.RemoveConstraint(c)

	initialLength := c.PulleyGetCurrentLength()

	// Step simulation: 30 frames @ 60Hz = 0.5 seconds. Short enough that
	// neither body reaches its pulley anchor at Y=10 (~1m of travel expected).
	dt := float32(1.0 / 60.0)
	for range 30 {
		ps.Update(dt)
	}

	heavyPos := bi.GetPosition(heavy)
	lightPos := bi.GetPosition(light)

	// Heavy body must have fallen from its start height.
	if heavyPos.Y >= startY-0.05 {
		t.Errorf("heavy body did not fall: Y=%.3f, start=%.3f", heavyPos.Y, startY)
	}

	// Light body must have risen (pulled up by the rope).
	if lightPos.Y <= startY+0.05 {
		t.Errorf("light body was not lifted: Y=%.3f, start=%.3f", lightPos.Y, startY)
	}

	// Rigid-rope invariant: with ratio=1 the heavy fall distance should equal
	// the light rise distance within solver tolerance.
	heavyDrop := startY - heavyPos.Y
	lightRise := lightPos.Y - startY
	if math.Abs(float64(heavyDrop-lightRise)) > 0.1 {
		t.Errorf("rope coupling mismatch: heavy dropped %.3f, light rose %.3f", heavyDrop, lightRise)
	}

	// Combined length should stay close to initial (rigid rope).
	finalLength := c.PulleyGetCurrentLength()
	if math.Abs(float64(finalLength-initialLength)) > 0.1 {
		t.Errorf("rope length drifted: initial=%.3f final=%.3f", initialLength, finalLength)
	}
}
