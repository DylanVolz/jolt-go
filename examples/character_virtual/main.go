package main

import (
	"fmt"
	"math"

	"github.com/bbitechnologies/jolt-go/jolt"
)

const (
	moveSpeed = float32(3.0)
	jumpSpeed = float32(7.0)
	gravityY  = float32(-9.81)
)

type inputState struct {
	forward bool
	back    bool
	left    bool
	right   bool
	jump    bool
}

func (in inputState) String() string {
	out := ""
	if in.forward {
		out += "W"
	}
	if in.back {
		out += "S"
	}
	if in.left {
		out += "A"
	}
	if in.right {
		out += "D"
	}
	if in.jump {
		out += " [JUMP]"
	}
	if out == "" {
		return "none"
	}
	return out
}

func normalizeXZ(v jolt.Vec3) jolt.Vec3 {
	mag := float32(math.Sqrt(float64(v.X*v.X + v.Z*v.Z)))
	if mag == 0 {
		return v
	}
	return jolt.Vec3{X: v.X / mag, Y: v.Y, Z: v.Z / mag}
}

func main() {
	if err := jolt.Init(); err != nil {
		panic(err)
	}
	defer jolt.Shutdown()

	ps := jolt.NewPhysicsSystem()
	defer ps.Destroy()
	bi := ps.GetBodyInterface()

	groundShape := jolt.CreateBox(jolt.Vec3{X: 10, Y: 0.5, Z: 10})
	defer groundShape.Destroy()
	ground := bi.CreateBody(groundShape, jolt.Vec3{X: 0, Y: 0, Z: 0}, jolt.MotionTypeStatic, false)
	defer bi.RemoveAndDestroyBody(ground)

	capsule := jolt.CreateCapsule(0.9, 0.5)
	defer capsule.Destroy()

	settings := jolt.NewCharacterVirtualSettings(capsule)
	settings.MaxStrength = 250
	character := ps.CreateCharacterVirtual(settings, jolt.Vec3{X: 0, Y: 4, Z: 0})
	defer character.Destroy()

	dt := float32(1.0 / 60.0)
	gravity := jolt.Vec3{X: 0, Y: gravityY, Z: 0}

	fmt.Println("CharacterVirtual example")
	fmt.Println("========================")
	fmt.Println("Phase 1: free fall onto the platform")
	fmt.Println("Phase 2: walk forward")
	fmt.Println("Phase 3: strafe right")
	fmt.Println("Phase 4: jump, then drift backward-left")
	fmt.Println()

	for step := range 360 {
		timeSec := float32(step) * dt
		input := inputState{}

		switch {
		case timeSec >= 1 && timeSec < 2.5:
			input.forward = true
		case timeSec >= 2.5 && timeSec < 4:
			input.right = true
		case timeSec >= 4 && timeSec < 4.1:
			input.jump = true
		case timeSec >= 4.1 && timeSec < 5.5:
			input.back = true
			input.left = true
		}

		move := jolt.Vec3{}
		if input.forward {
			move.Z += 1
		}
		if input.back {
			move.Z -= 1
		}
		if input.right {
			move.X += 1
		}
		if input.left {
			move.X -= 1
		}
		move = normalizeXZ(move)

		vel := character.GetLinearVelocity()
		if character.IsSupported() {
			vel = character.GetGroundVelocity()
		}
		vel.X = move.X * moveSpeed
		vel.Z = move.Z * moveSpeed
		if input.jump && character.IsSupported() {
			vel.Y = jumpSpeed
		}
		vel.Y += gravityY * dt

		character.SetLinearVelocity(vel)
		character.ExtendedUpdate(dt, gravity)

		if step%30 == 0 {
			pos := character.GetPosition()
			contacts := character.GetActiveContacts(8)
			fmt.Printf(
				"[%.1fs] pos=(%6.2f,%6.2f,%6.2f) vel=(%5.2f,%5.2f,%5.2f) ground=%-13s supported=%-5v contacts=%d input=%s\n",
				timeSec,
				pos.X, pos.Y, pos.Z,
				character.GetLinearVelocity().X, character.GetLinearVelocity().Y, character.GetLinearVelocity().Z,
				character.GetGroundState(),
				character.IsSupported(),
				len(contacts),
				input.String(),
			)
		}
	}
}
