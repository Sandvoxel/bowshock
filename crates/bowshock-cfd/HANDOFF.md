# CFD handoff

The 2D-solver handoff that was here (the `cfd-2d` round, 2026-11-15) is superseded. The single entry point is **[`HANDOFF3D.md`](HANDOFF3D.md)** (the 3D viscous
solver on `cfd-3d`: state, how to run it, branches, the gate table, performance, the flip and deletion steps, open problems); the detailed record is
[`DESIGN3D.md`](DESIGN3D.md). The old text stays in git history (`git log -- crates/bowshock-cfd/HANDOFF.md`); the 2D solver itself is still in the tree until the deletion
commit described in `HANDOFF3D.md` section 6.
