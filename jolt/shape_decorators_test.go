package jolt

import (
	"math"
	"testing"
)

// --- Decorator shape tests ---

func TestScaledShape(t *testing.T) {
	inner := CreateSphere(1.0)
	defer inner.Destroy()

	// Scale 2x on all axes → ray hits at distance 10 - 2 = 8 instead of 10 - 1 = 9.
	scaled := CreateScaledShape(inner, Vec3{X: 2, Y: 2, Z: 2})
	if scaled == nil {
		t.Fatal("CreateScaledShape returned nil")
	}
	defer scaled.Destroy()

	ray := RRayCast{
		Origin:    Vec3{X: 0, Y: 10, Z: 0},
		Direction: Vec3{X: 0, Y: -20, Z: 0},
	}
	var result RayCastResult
	if !scaled.CastRay(ray, DefaultRayCastSettings(), &result) {
		t.Fatal("ray should have hit the scaled sphere")
	}
	hit := result.Fraction * 20.0
	if math.Abs(float64(hit-8.0)) > 0.1 {
		t.Errorf("hit distance %.2f, expected ~8.0", hit)
	}
}

func TestScaledShapeZeroScaleRejected(t *testing.T) {
	inner := CreateSphere(1.0)
	defer inner.Destroy()

	// Zero scale is rejected by Jolt; wrapper returns nil rather than asserting.
	if got := CreateScaledShape(inner, Vec3{X: 0, Y: 0, Z: 0}); got != nil {
		got.Destroy()
		t.Fatal("expected nil for zero-scale shape")
	}
}

func TestRotatedTranslatedShape(t *testing.T) {
	inner := CreateSphere(1.0)
	defer inner.Destroy()

	// The decorator's COM shifts to the translation, so in COM-local coords
	// the sphere sits at origin. A ray from (-5, 0, 0) going +X hits at ~4.
	rt := CreateRotatedTranslatedShape(inner, Vec3{X: 5, Y: 0, Z: 0}, QuatIdentity())
	if rt == nil {
		t.Fatal("CreateRotatedTranslatedShape returned nil")
	}
	defer rt.Destroy()

	ray := RRayCast{
		Origin:    Vec3{X: -5, Y: 0, Z: 0},
		Direction: Vec3{X: 10, Y: 0, Z: 0},
	}
	var result RayCastResult
	if !rt.CastRay(ray, DefaultRayCastSettings(), &result) {
		t.Fatal("ray should have hit translated sphere")
	}
	hit := result.Fraction * 10.0
	if math.Abs(float64(hit-4.0)) > 0.1 {
		t.Errorf("hit distance %.2f, expected ~4.0", hit)
	}
}

func TestOffsetCenterOfMassShape(t *testing.T) {
	// Offset COM shifts the shape's COM by the given offset without changing
	// geometry. In COM-local coords, a box with half-extent 1 and offset
	// (0,-0.5,0) has its top at Y=1.5 (inner top Y=1 minus COM Y=-0.5).
	inner := CreateBox(Vec3{X: 1, Y: 1, Z: 1})
	defer inner.Destroy()

	offset := CreateOffsetCenterOfMassShape(inner, Vec3{X: 0, Y: -0.5, Z: 0})
	if offset == nil {
		t.Fatal("CreateOffsetCenterOfMassShape returned nil")
	}
	defer offset.Destroy()

	ray := RRayCast{
		Origin:    Vec3{X: 0, Y: 5, Z: 0},
		Direction: Vec3{X: 0, Y: -10, Z: 0},
	}
	var result RayCastResult
	if !offset.CastRay(ray, DefaultRayCastSettings(), &result) {
		t.Fatal("ray should have hit the offset-COM box")
	}
	hit := result.Fraction * 10.0
	if math.Abs(float64(hit-3.5)) > 0.1 {
		t.Errorf("hit distance %.2f, expected ~3.5", hit)
	}
}

// --- MutableCompound tests ---

// TestMutableCompoundAcceptance covers the T-0128 acceptance criterion:
// assemble a 10-piece compound, then mutate one sub-shape without re-baking
// the whole shape.
func TestMutableCompoundAcceptance(t *testing.T) {
	const n = 10

	// Build 10 unit spheres along +X at symmetric positions so the compound's
	// center of mass sits at the origin and COM-local coords equal the
	// positions we pass in. Sub-shape i is at X = i - 4.5.
	subShapes := make([]*Shape, n)
	positions := make([]Vec3, n)
	rotations := make([]Quat, n)
	for i := range n {
		subShapes[i] = CreateSphere(0.25)
		positions[i] = Vec3{X: float32(i) - 4.5, Y: 0, Z: 0}
		rotations[i] = QuatIdentity()
	}
	defer func() {
		for _, s := range subShapes {
			s.Destroy()
		}
	}()

	compound := CreateMutableCompound(subShapes, positions, rotations)
	if compound == nil {
		t.Fatal("CreateMutableCompound returned nil")
	}
	defer compound.Destroy()

	if got := compound.NumSubShapes(); got != n {
		t.Fatalf("NumSubShapes = %d, expected %d", got, n)
	}

	// Sub-shape 5 is at X=0.5. A ray straight down at X=0.5 should hit it.
	const subX = float32(0.5)
	rayAtIdx5 := RRayCast{
		Origin:    Vec3{X: subX, Y: 10, Z: 0},
		Direction: Vec3{X: 0, Y: -20, Z: 0},
	}
	var before RayCastResult
	if !compound.CastRay(rayAtIdx5, DefaultRayCastSettings(), &before) {
		t.Fatal("ray should have hit original sub-shape 5")
	}
	beforeDist := before.Fraction * 20.0
	if math.Abs(float64(beforeDist-9.75)) > 0.2 {
		t.Errorf("pre-mutation hit distance %.2f, expected ~9.75", beforeDist)
	}

	// Move sub-shape 5 far out of the ray path — only that sub-shape. This is
	// the core acceptance: mutate one sub-shape without rebuilding the tree.
	if err := compound.ModifySubShape(5, Vec3{X: subX, Y: 100, Z: 0}, QuatIdentity()); err != nil {
		t.Fatalf("ModifySubShape: %v", err)
	}

	var after RayCastResult
	if compound.CastRay(rayAtIdx5, DefaultRayCastSettings(), &after) {
		t.Errorf("ray should have missed compound after moving sub-shape 5 away (fraction %.2f)", after.Fraction)
	}

	// Rest of the compound must be intact: ray at X = positions[3].X should
	// still hit sub-shape 3.
	rayAtIdx3 := RRayCast{
		Origin:    Vec3{X: positions[3].X, Y: 10, Z: 0},
		Direction: Vec3{X: 0, Y: -20, Z: 0},
	}
	var other RayCastResult
	if !compound.CastRay(rayAtIdx3, DefaultRayCastSettings(), &other) {
		t.Error("ray at sub-shape 3's X should still hit after mutation")
	}
}

func TestMutableCompoundAddRemove(t *testing.T) {
	empty := CreateMutableCompound(nil, nil, nil)
	if empty == nil {
		t.Fatal("CreateMutableCompound with empty input returned nil")
	}
	defer empty.Destroy()

	if got := empty.NumSubShapes(); got != 0 {
		t.Fatalf("empty compound NumSubShapes = %d, expected 0", got)
	}

	sphere := CreateSphere(0.5)
	defer sphere.Destroy()

	idx, err := empty.AddSubShape(Vec3{X: 1, Y: 0, Z: 0}, QuatIdentity(), sphere)
	if err != nil {
		t.Fatalf("AddSubShape: %v", err)
	}
	if idx != 0 {
		t.Errorf("first AddSubShape index = %d, expected 0", idx)
	}
	if got := empty.NumSubShapes(); got != 1 {
		t.Fatalf("NumSubShapes after add = %d, expected 1", got)
	}

	idx2, err := empty.AddSubShape(Vec3{X: -1, Y: 0, Z: 0}, QuatIdentity(), sphere)
	if err != nil {
		t.Fatalf("second AddSubShape: %v", err)
	}
	if idx2 != 1 {
		t.Errorf("second AddSubShape index = %d, expected 1", idx2)
	}

	if err := empty.RemoveSubShape(0); err != nil {
		t.Fatalf("RemoveSubShape: %v", err)
	}
	if got := empty.NumSubShapes(); got != 1 {
		t.Fatalf("NumSubShapes after remove = %d, expected 1", got)
	}
}

func TestMutableCompoundErrorsOnWrongShape(t *testing.T) {
	// A plain sphere is NOT a MutableCompound — mutation calls must fail.
	sphere := CreateSphere(1.0)
	defer sphere.Destroy()

	if got := sphere.NumSubShapes(); got != -1 {
		t.Errorf("NumSubShapes on non-compound = %d, expected -1", got)
	}

	sub := CreateBox(Vec3{X: 0.5, Y: 0.5, Z: 0.5})
	defer sub.Destroy()

	if _, err := sphere.AddSubShape(Vec3{}, QuatIdentity(), sub); err == nil {
		t.Error("AddSubShape on non-compound should have returned an error")
	}
	if err := sphere.RemoveSubShape(0); err == nil {
		t.Error("RemoveSubShape on non-compound should have returned an error")
	}
	if err := sphere.ModifySubShape(0, Vec3{}, QuatIdentity()); err == nil {
		t.Error("ModifySubShape on non-compound should have returned an error")
	}

	// Also: StaticCompound is a compound but NOT a MutableCompound; mutations
	// must fail, but NumSubShapes should still work.
	static := CreateStaticCompound(
		[]*Shape{sub, sub},
		[]Vec3{{X: 0}, {X: 2}},
		[]Quat{QuatIdentity(), QuatIdentity()},
	)
	if static == nil {
		t.Fatal("CreateStaticCompound returned nil")
	}
	defer static.Destroy()

	if got := static.NumSubShapes(); got != 2 {
		t.Errorf("static compound NumSubShapes = %d, expected 2", got)
	}
	if err := static.ModifySubShape(0, Vec3{}, QuatIdentity()); err == nil {
		t.Error("ModifySubShape on StaticCompound should have returned an error")
	}
}

func TestMutableCompoundModifyShapeWithShape(t *testing.T) {
	original := CreateSphere(0.25)
	defer original.Destroy()
	replacement := CreateSphere(2.0)
	defer replacement.Destroy()

	compound := CreateMutableCompound(
		[]*Shape{original},
		[]Vec3{{X: 0, Y: 0, Z: 0}},
		[]Quat{QuatIdentity()},
	)
	if compound == nil {
		t.Fatal("CreateMutableCompound returned nil")
	}
	defer compound.Destroy()

	// Swap the sub-shape for a much larger sphere. A ray that would have
	// missed the 0.25-radius original at X=1.5 should now hit the 2.0-radius
	// replacement.
	if err := compound.ModifySubShapeWithShape(0, Vec3{}, QuatIdentity(), replacement); err != nil {
		t.Fatalf("ModifySubShapeWithShape: %v", err)
	}
	if err := compound.AdjustCenterOfMass(); err != nil {
		t.Fatalf("AdjustCenterOfMass: %v", err)
	}

	ray := RRayCast{
		Origin:    Vec3{X: 1.5, Y: 10, Z: 0},
		Direction: Vec3{X: 0, Y: -20, Z: 0},
	}
	var result RayCastResult
	if !compound.CastRay(ray, DefaultRayCastSettings(), &result) {
		t.Error("ray should have hit the replaced (larger) sub-shape")
	}
}

func TestMutableCompoundModifyOutOfRange(t *testing.T) {
	sub := CreateSphere(0.5)
	defer sub.Destroy()

	compound := CreateMutableCompound(
		[]*Shape{sub},
		[]Vec3{{}},
		[]Quat{QuatIdentity()},
	)
	if compound == nil {
		t.Fatal("CreateMutableCompound returned nil")
	}
	defer compound.Destroy()

	if err := compound.ModifySubShape(99, Vec3{}, QuatIdentity()); err == nil {
		t.Error("ModifySubShape with out-of-range index should have returned an error")
	}
	if err := compound.RemoveSubShape(99); err == nil {
		t.Error("RemoveSubShape with out-of-range index should have returned an error")
	}
}
