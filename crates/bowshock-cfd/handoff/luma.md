# luma-perf section: not delivered, branch NOT merged

Section not delivered. Branch `cfd-3d-lumaperf` at sha 8d5fd93 has 6 commits not in `cfd-3d` (compiled loop per chunk, force read-backs as compiled functions, launch_trace); merging it conflicts in `crates/bowshock-cfd/src/gpu/body3.rs` and `gpu/rans3.rs` (cfd-3d already has gpu3d's compiled loop on Luma `5a592c4`), so it was left unmerged for the owner to resolve.
