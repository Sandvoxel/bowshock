default:
    @just --list

fmt:
    cargo fmt --all

lint:
    cargo fmt --all --check
    cargo clippy --workspace --all-targets -- -D warnings

test:
    cargo test -p bowshock-model -p bowshock-motors -p bowshock-sim -p bowshock

test-cfd:
    cargo test --release -p bowshock-cfd -p bowshock-reference -p bowshock-cases -p bowshock-core

ui-smoke:
    ICED_TEST_BACKEND=tiny-skia cargo run -q -p bowshock -- drive --example crates/bowshock/scripts/shell.drive target/drive-smoke/shell
    ICED_TEST_BACKEND=tiny-skia cargo run -q -p bowshock -- drive --example crates/bowshock/scripts/simulate.drive target/drive-smoke/simulate
    ICED_TEST_BACKEND=tiny-skia cargo run -q -p bowshock -- drive --example crates/bowshock/scripts/unsaved-modal.drive target/drive-smoke/unsaved

ci: lint test test-cfd ui-smoke

run:
    cargo run --release -p bowshock -- --example
