# AGENTS.md: working in oasa-radio

Read by AI coding agents (Claude Code, Codex, Cursor and others) and useful for people too. This repo holds **radio adapters, radio settings and experimental waveforms** (tech tree nodes R-COM1, R-COM2 radio side, later T-RF1).

## Rules

Follow the [OASA software standard](https://joinoasa.org/docs/software-standard/). In this repo especially:

- **Encryption is never done here.** Adapters move bytes that [oasa-sec](https://github.com/joinoasa/oasa-sec) has already encrypted. A radio's own encryption is never counted as the OASA link.
- **Rust** for adapters and waveforms; Python for scripts and prototypes.
- **Every adapter and waveform ships with lab tests** in [oasa-lab](https://github.com/joinoasa/oasa-lab), starting with the simulated channel.
- **Radios over cable or in the shielded box** unless a test plan says otherwise.
- Record every outside radio project used in the [software register](https://joinoasa.org/docs/software-register/).

## Pull requests

- Branch, then open a pull request into `main`. CI must pass.
- One topic per pull request. Say what changed and what you checked.
- Commit with your GitHub no-reply email.

Writing style: short, clear, not wordy.
