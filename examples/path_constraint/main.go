package main

import (
	"fmt"
	"math"

	"github.com/bbitechnologies/jolt-go/jolt"
)

func buildCircularLoop(radius float32, n int) []jolt.HermitePathPoint {
	points := make([]jolt.HermitePathPoint, n)
	segmentAngle := float32(2 * math.Pi / float64(n))
	tangentScale := radius * segmentAngle
	for i := range n {
		theta := float64(i) * float64(segmentAngle)
		cos, sin := float32(math.Cos(theta)), float32(math.Sin(theta))
		points[i] = jolt.HermitePathPoint{
			Position: jolt.Vec3{X: radius * cos, Y: 0, Z: radius * sin},
			Tangent:  jolt.Vec3{X: -tangentScale * sin, Y: 0, Z: tangentScale * cos},
			Normal:   jolt.Vec3{X: 0, Y: 1, Z: 0},
		}
	}
	return points
}

func main() {
	if err := jolt.Init(); err != nil {
		panic(err)
	}
	defer jolt.Shutdown()

	ps := jolt.NewPhysicsSystem()
	defer ps.Destroy()
	bi := ps.GetBodyInterface()

	anchorShape := jolt.CreateBox(jolt.Vec3{X: 0.1, Y: 0.1, Z: 0.1})
	defer anchorShape.Destroy()
	anchor := bi.CreateBody(anchorShape, jolt.Vec3{X: 0, Y: 0, Z: 0}, jolt.MotionTypeStatic, false)
	defer bi.RemoveAndDestroyBody(anchor)

	cartShape := jolt.CreateBox(jolt.Vec3{X: 0.2, Y: 0.2, Z: 0.2})
	defer cartShape.Destroy()
	cart := bi.CreateBody(cartShape, jolt.Vec3{X: 5, Y: 0, Z: 0}, jolt.MotionTypeDynamic, false)
	defer bi.RemoveAndDestroyBody(cart)
	bi.SetGravityFactor(cart, 0)
	bi.ActivateBody(cart)

	path := jolt.CreateHermitePath(buildCircularLoop(5, 10), true)
	if path == nil {
		panic("CreateHermitePath returned nil")
	}
	defer path.Destroy()

	constraint := ps.CreatePathConstraint(
		anchor, cart,
		path,
		jolt.Vec3{},
		jolt.QuatIdentity(),
		0,
		jolt.PathRotationFree,
		0,
	)
	if constraint == nil {
		panic("CreatePathConstraint returned nil")
	}
	defer constraint.Destroy()
	ps.AddConstraint(constraint)
	defer ps.RemoveConstraint(constraint)

	targetVelocity := float32(3.0)
	constraint.PathSetPositionMotorSettings(2, 1, 1e6, 1e6)
	constraint.PathSetPositionMotorState(jolt.MotorStateVelocity)
	constraint.PathSetTargetVelocity(targetVelocity)

	fmt.Println("Path constraint example")
	fmt.Println("=======================")
	fmt.Printf("path max fraction: %.1f\n", path.MaxFraction())

	dt := float32(1.0 / 60.0)
	for step := range 360 {
		ps.Update(dt)
		if step%60 == 0 {
			pos := bi.GetPosition(cart)
			speed := bi.GetLinearVelocity(cart).Length()
			fmt.Printf(
				"[%.1fs] fraction=%5.2f pos=(%6.2f,%5.2f,%6.2f) speed=%4.2f target=%4.2f\n",
				float32(step)*dt,
				constraint.PathGetPathFraction(),
				pos.X, pos.Y, pos.Z,
				speed,
				targetVelocity,
			)
		}
	}
}
