package main

import (
	"fmt"

	"github.com/bbitechnologies/jolt-go/jolt"
)

func main() {
	if err := jolt.Init(); err != nil {
		panic(err)
	}
	defer jolt.Shutdown()

	const (
		gridSize = 10
		spacing  = float32(0.3)
	)

	ps := jolt.NewPhysicsSystem()
	defer ps.Destroy()
	bi := ps.GetBodyInterface()

	groundShape := jolt.CreateBox(jolt.Vec3{X: 20, Y: 0.25, Z: 20})
	defer groundShape.Destroy()
	ground := bi.CreateBody(groundShape, jolt.Vec3{X: 0, Y: -5, Z: 0}, jolt.MotionTypeStatic, false)
	defer bi.RemoveAndDestroyBody(ground)

	invMasses := make([]float32, gridSize*gridSize)
	for i := range invMasses {
		invMasses[i] = 1
	}
	invMasses[0] = 0
	invMasses[gridSize-1] = 0

	settings := jolt.NewClothGridSettings(gridSize, gridSize, spacing, invMasses, 1e-4, jolt.BendTypeDistance)
	if settings == nil {
		panic("NewClothGridSettings returned nil")
	}
	defer settings.Destroy()

	params := jolt.DefaultSoftBodyParams()
	params.Position = jolt.Vec3{X: 0, Y: 4, Z: 0}
	cloth := bi.NewSoftBody(settings, params)
	if cloth == nil {
		panic("NewSoftBody returned nil")
	}
	defer bi.RemoveAndDestroyBody(cloth)

	vertexCount := ps.GetSoftBodyVertexCount(cloth)
	positions := make([]float32, vertexCount*3)
	centerIndex := (gridSize/2)*gridSize + gridSize/2

	fmt.Println("Soft-body cloth example")
	fmt.Println("=======================")
	fmt.Printf("vertex count: %d (center vertex index %d)\n", vertexCount, centerIndex)

	dt := float32(1.0 / 60.0)
	wind := jolt.Vec3{X: 0, Y: 0, Z: 4 * float32(vertexCount)}

	for step := range 300 {
		if step < 180 {
			bi.AddSoftBodyForce(cloth, wind)
		}
		ps.Update(dt)

		if step%60 == 0 {
			ps.GetSoftBodyVertexPositions(cloth, positions)
			center := jolt.Vec3{
				X: positions[centerIndex*3+0],
				Y: positions[centerIndex*3+1],
				Z: positions[centerIndex*3+2],
			}
			fmt.Printf(
				"[%.1fs] center=(%6.2f,%6.2f,%6.2f) pinnedInvMass=(%.1f, %.1f)\n",
				float32(step)*dt,
				center.X, center.Y, center.Z,
				ps.GetSoftBodyVertexInvMass(cloth, 0),
				ps.GetSoftBodyVertexInvMass(cloth, gridSize-1),
			)
		}
	}
}
