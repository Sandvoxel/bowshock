# The bowshock Guide

The user guide of bowshock, an [mdBook](https://rust-lang.github.io/mdBook/) (pinned to
0.5.4 in `.github/workflows/guide.yml`). Published at <https://sandvoxel.github.io/bowshock/>, one
directory per version: `/main/` is the tip of `main`, `/vX.Y.Z/` is the guide as of each release.
The API reference (rustdoc) is published with each version, under `api/` of its directory
(`/main/api/bowshock_core/index.html`).

`main` holds only the Markdown source (`guide/src/`), the theme (`guide/theme/`) and the workflow;
the built site lives on the `gh-pages` branch.

## Writing

Every Rust code block in `guide/src/*.md` is compiled by `cargo test --doc -p bowshock-core`:
`crates/bowshock-core/src/lib.rs` has a `#[cfg(doctest)]` module that includes each chapter. **A new chapter
needs a line there** (and an entry in `guide/src/SUMMARY.md`). Hide setup with `# ` lines as in
rustdoc; use `text` for blocks that are not Rust. A block that calls `Analysis::run` is
`rust,no_run` (it solves the flow, minutes on the CPU); blocks about specs, atmosphere and
validation errors run on CI. Import from the crate root only (`use bowshock_core::{..}`), never from
`bowshock_core::engine`.

```sh
cargo test --release -p bowshock-core --doc     # compile (and run) the guide's examples
```

## Preview

```sh
cargo install mdbook --version 0.5.4 --locked
mdbook serve guide --open            # http://localhost:3000, rebuilds on save
mdbook build guide                   # writes guide/book/
```

The version switcher in the menu bar only appears on the published site (it reads
`/bowshock/versions.json`). The relative link to the API reference (`api/bowshock_core/index.html`)
only resolves on the published site, where the workflow copies `cargo doc` output next to the guide.

## How versions are published

`.github/workflows/guide.yml` builds the guide and the API reference and commits them to `gh-pages`:

| Trigger | Result |
| --- | --- |
| push to `main` touching `guide/**`, `crates/bowshock-core/src/**` or the workflow | publishes `/main/` |
| a GitHub release is published | builds the release tag, publishes `/<tag>/` (for example `/v0.1.0/`) |
| manual run (Actions -> Guide -> Run workflow) with inputs `ref` and `version` | builds `ref`, publishes it as `version` |

After every run the workflow regenerates `/versions.json` (releases newest first, then `main`;
the version switcher reads it) and `/index.html`, which redirects to the newest release (to
`/main/` while there is none). Existing version directories are replaced only when the same
version is published again and are never deleted. The workflow creates `gh-pages` as an orphan
branch on its first run.

## One-time GitHub setting

After the first run has created the `gh-pages` branch: Settings -> Pages -> Source:
**Deploy from a branch** -> branch `gh-pages`, folder `/ (root)`. The branch contains a
`.nojekyll` file, so Pages serves the files as they are.
