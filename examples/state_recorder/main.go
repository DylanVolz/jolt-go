package main

import (
	"fmt"
	"math"

	"github.com/bbitechnologies/jolt-go/jolt"
)

type sample struct {
	pos jolt.Vec3
	vel jolt.Vec3
}

func makeWorld() (*jolt.PhysicsSystem, *jolt.BodyInterface, *jolt.BodyID, func()) {
	ps := jolt.NewPhysicsSystem()
	bi := ps.GetBodyInterface()

	groundShape := jolt.CreateBox(jolt.Vec3{X: 50, Y: 0.5, Z: 50})
	ground := bi.CreateBody(groundShape, jolt.Vec3{X: 0, Y: 0, Z: 0}, jolt.MotionTypeStatic, false)
	bi.SetRestitution(ground, 0.8)

	sphereShape := jolt.CreateSphere(0.5)
	sphere := bi.CreateBody(sphereShape, jolt.Vec3{X: 0, Y: 10, Z: 0}, jolt.MotionTypeDynamic, false)
	bi.SetRestitution(sphere, 0.8)
	bi.SetLinearVelocity(sphere, jolt.Vec3{X: 1.5, Y: 0, Z: 0.7})
	bi.ActivateBody(sphere)

	cleanup := func() {
		bi.RemoveAndDestroyBody(sphere)
		bi.RemoveAndDestroyBody(ground)
		sphereShape.Destroy()
		groundShape.Destroy()
		ps.Destroy()
	}

	return ps, bi, sphere, cleanup
}

func record(bi *jolt.BodyInterface, sphere *jolt.BodyID, ps *jolt.PhysicsSystem, steps int, dt float32) []sample {
	out := make([]sample, steps)
	for i := range steps {
		ps.Update(dt)
		out[i] = sample{
			pos: bi.GetPosition(sphere),
			vel: bi.GetLinearVelocity(sphere),
		}
	}
	return out
}

func sameSample(a, b sample) bool {
	const eps = 1e-5
	diff := func(x, y float32) bool { return math.Abs(float64(x-y)) > eps }
	return !(diff(a.pos.X, b.pos.X) || diff(a.pos.Y, b.pos.Y) || diff(a.pos.Z, b.pos.Z) ||
		diff(a.vel.X, b.vel.X) || diff(a.vel.Y, b.vel.Y) || diff(a.vel.Z, b.vel.Z))
}

func main() {
	if err := jolt.Init(); err != nil {
		panic(err)
	}
	defer jolt.Shutdown()

	ps, bi, sphere, cleanup := makeWorld()
	defer cleanup()

	dt := float32(1.0 / 60.0)
	for range 90 {
		ps.Update(dt)
	}

	rec := jolt.NewStateRecorder()
	defer rec.Destroy()
	ps.SaveState(rec, jolt.StateRecorderAll)

	reference := record(bi, sphere, ps, 90, dt)

	rec.Rewind()
	if err := ps.RestoreState(rec); err != nil {
		panic(err)
	}
	replay := record(bi, sphere, ps, 90, dt)

	matched := true
	for i := range reference {
		if !sameSample(reference[i], replay[i]) {
			matched = false
			fmt.Printf("mismatch at step %d: ref=%+v replay=%+v\n", i, reference[i], replay[i])
			break
		}
	}

	fmt.Println("State recorder example")
	fmt.Println("======================")
	fmt.Printf("snapshot size: %d bytes\n", rec.Size())
	fmt.Printf("replay matched reference: %v\n", matched)

	for _, idx := range []int{0, 29, 59, 89} {
		s := reference[idx]
		fmt.Printf(
			"[step %02d] pos=(%6.2f,%6.2f,%6.2f) vel=(%5.2f,%5.2f,%5.2f)\n",
			idx+1,
			s.pos.X, s.pos.Y, s.pos.Z,
			s.vel.X, s.vel.Y, s.vel.Z,
		)
	}
}
