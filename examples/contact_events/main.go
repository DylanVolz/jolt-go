package main

import (
	"fmt"

	"github.com/bbitechnologies/jolt-go/jolt"
)

const (
	layerGround = 0
	layerFaller = 1
	layerGhost  = 2
)

func main() {
	if err := jolt.Init(); err != nil {
		panic(err)
	}
	defer jolt.Shutdown()

	cm := jolt.NewCollisionMatrix(3)
	defer cm.Destroy()
	cm.EnableCollision(layerGround, layerFaller)
	cm.EnableCollision(layerFaller, layerFaller)

	ps := jolt.NewPhysicsSystemWithLayers(cm)
	defer ps.Destroy()
	ps.EnableContactEvents()

	bi := ps.GetBodyInterface()

	groundShape := jolt.CreateBox(jolt.Vec3{X: 5, Y: 0.5, Z: 5})
	defer groundShape.Destroy()
	ground := bi.CreateBodyOnLayer(groundShape, jolt.Vec3{X: 0, Y: 0, Z: 0}, jolt.MotionTypeStatic, false, layerGround)
	defer bi.RemoveAndDestroyBody(ground)

	fallingShape := jolt.CreateSphere(0.5)
	defer fallingShape.Destroy()
	falling := bi.CreateBodyOnLayer(fallingShape, jolt.Vec3{X: 0, Y: 3, Z: 0}, jolt.MotionTypeDynamic, false, layerFaller)
	defer bi.RemoveAndDestroyBody(falling)
	bi.ActivateBody(falling)

	ghostShape := jolt.CreateSphere(0.5)
	defer ghostShape.Destroy()
	ghost := bi.CreateBodyOnLayer(ghostShape, jolt.Vec3{X: 2, Y: 1, Z: 0}, jolt.MotionTypeStatic, false, layerGhost)
	defer bi.RemoveAndDestroyBody(ghost)

	fmt.Println("Contact events + collision layers example")
	fmt.Println("=========================================")
	fmt.Printf("ground layer=%d faller layer=%d ghost layer=%d\n", ps.GetBodyLayer(ground), ps.GetBodyLayer(falling), ps.GetBodyLayer(ghost))

	if hit, ok := ps.CastRayFiltered(
		jolt.Vec3{X: 0, Y: 4, Z: 0},
		jolt.Vec3{X: 0, Y: -8, Z: 0},
		jolt.LayerMask(layerFaller),
	); ok {
		fmt.Printf("filtered ray (layerFaller) hit body=%d at y=%.2f\n", hit.BodyID.IndexAndSequenceNumber(), hit.HitPoint.Y)
	}

	if hit, ok := ps.CastRayFiltered(
		jolt.Vec3{X: 2, Y: 3, Z: 0},
		jolt.Vec3{X: 0, Y: -4, Z: 0},
		jolt.LayerMask(layerGhost),
	); ok {
		fmt.Printf("filtered ray (layerGhost) hit body=%d at y=%.2f\n", hit.BodyID.IndexAndSequenceNumber(), hit.HitPoint.Y)
	}

	if _, ok := ps.CastRayFiltered(
		jolt.Vec3{X: 2, Y: 3, Z: 0},
		jolt.Vec3{X: 0, Y: -4, Z: 0},
		jolt.LayerMask(layerGround, layerFaller),
	); !ok {
		fmt.Println("ghost layer is excluded from ground/faller filtered ray as expected")
	}

	counts := map[jolt.ContactEventType]int{}
	dt := float32(1.0 / 60.0)
	for step := range 180 {
		ps.Update(dt)
		events := ps.DrainContactEvents(64)
		for _, ev := range events {
			counts[ev.Type]++
			if ev.Type == jolt.ContactAdded {
				fmt.Printf(
					"[step %03d] contact added body1=%d body2=%d point=(%.2f, %.2f, %.2f)\n",
					step,
					ev.BodyID1,
					ev.BodyID2,
					ev.ContactPoint.X,
					ev.ContactPoint.Y,
					ev.ContactPoint.Z,
				)
			}
		}
		if counts[jolt.ContactAdded] > 0 && step > 90 {
			break
		}
	}

	fmt.Printf(
		"event counts: added=%d persisted=%d removed=%d\n",
		counts[jolt.ContactAdded],
		counts[jolt.ContactPersisted],
		counts[jolt.ContactRemoved],
	)
}
