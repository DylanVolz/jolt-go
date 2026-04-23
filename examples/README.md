# Example Programs

The legacy quick-start demo still lives at [`example/main.go`](../example/main.go).
This directory adds smaller focused examples for newer API surface areas.

Run any example directly:

```bash
go run ./examples/character_virtual
go run ./examples/contact_events
go run ./examples/path_constraint
go run ./examples/soft_body_cloth
go run ./examples/state_recorder
```

Or run the full smoke suite:

```bash
./scripts/run-examples.sh
```

Included examples:

- `character_virtual` — kinematic character controller movement, jump, and ground-state reporting.
- `contact_events` — collision layers, filtered raycasts, and contact-event draining.
- `path_constraint` — Hermite spline rail with a motor-driven cart.
- `soft_body_cloth` — pinned cloth grid with wind and runtime vertex inspection.
- `state_recorder` — save/restore deterministic replay with `StateRecorder`.
