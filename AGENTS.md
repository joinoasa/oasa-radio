# AGENTS.md: working in oasa-radio

Read by AI coding agents (Claude Code, Codex, Cursor and others) and useful for people too. This repo holds **radio adapters, radio settings and experimental waveforms** (tech tree nodes R-COM1, R-COM2 radio side, later T-RF1).

## Rules

Follow the [OASA software standard](https://joinoasa.org/docs/software-standard/). In this repo especially:

- **Encryption is never done here.** Adapters move bytes that [oasa-sec](https://github.com/joinoasa/oasa-sec) has already encrypted. A radio's own encryption is never counted as the OASA link.
- **Rust** for adapters and waveforms; Python for scripts and prototypes.
- **Every adapter and waveform ships with lab tests** in [oasa-lab](https://github.com/joinoasa/oasa-lab), starting with the simulated channel.
- **Radios over cable or in the shielded box** unless a test plan says otherwise.
- Record every outside radio project used in the [software register](https://joinoasa.org/docs/software-register/).

## Commands

| Command | What it does |
|---|---|
| `cargo build --locked --bins --examples` | Build `oasa-radio` and the test relay |
| `cargo fmt --check` and `cargo clippy --locked --all-targets -- -D warnings` | Format and lint. CI fails on warnings |
| `cargo test --locked` | Unit tests, pseudo-terminal bridge tests, CLI tests (Linux) |
| `cargo deny --locked check` and `cargo audit` | License and advisory gates |
| `OASA_SEC_BIN_DIR=<oasa-sec>/target/debug scripts/serial-e2e.sh` | Two real `oasa-link` processes through two serial bridges. Build oasa-sec first (`cargo build --locked --bins`) |
| `oasa-radio bridge --radio serial --device DEV --udp-listen H:P --udp-send H:P` | Run a bridge. `oasa-radio info` lists adapters |

Run all of these before every pull request. More: [README](README.md), [serial framing](docs/serial-framing.md).

## Adding an adapter

Implement `Radio` in `src/adapters/`, register it in `src/cli.rs`, add tests and a README row. Never inspect, alter or log packet bytes (sizes and counts only). Any new crate goes in the README "Built on" table (permissive licenses only).

## Pull requests

- Branch, then open a pull request into `main`. CI must pass.
- One topic per pull request. Say what changed and what you checked.
- Commit with your GitHub no-reply email.

Writing style: short, clear, not wordy.
