#!/usr/bin/env bash
# End-to-end check, no hardware: two real oasa-link processes talk through two
# `oasa-radio bridge --radio serial` processes over a pseudo-terminal relay
# limited to 57600 baud. Marked packets go both ways; the bytes on the serial
# line are captured and must never hold the marker. Test tool only.
#
#   cargo build --locked --bins --examples
#   OASA_SEC_BIN_DIR=/path/to/oasa-sec/target/debug scripts/serial-e2e.sh
set -euo pipefail

R=target/debug
S=${OASA_SEC_BIN_DIR:-oasa-sec/target/debug}
PYTHON=${PYTHON:-python3}
CHECK="$PYTHON -I scripts/serial_e2e_check.py"
for f in "$R/oasa-radio" "$R/examples/ptyrelay" "$S/oasa-link" "$S/oasa-pair"; do
  [ -x "$f" ] || { echo "missing $f (build oasa-radio with --bins --examples, and oasa-sec with --bins)"; exit 2; }
done
T=$(mktemp -d)
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done; rm -rf "$T"; }
trap cleanup EXIT
pass() { echo "PASS  $1"; }
fail() {
  echo "FAIL  $1"
  local f
  for f in "$T"/*.log; do [ -e "$f" ] && { echo "--- $f"; tail -n 30 "$f"; }; done
  if [ -n "${GITHUB_ACTIONS:-}" ]; then echo "::error title=serial-e2e::$1"; fi
  exit 1
}
wait_for() { # secs pattern file
  local i
  for i in $(seq $(($1 * 10))); do grep -q -- "$2" "$3" 2>/dev/null && return 0; sleep 0.1; done
  return 1
}
pubkey() { $S/oasa-pair show --dir "$1" | sed -E 's/.*"public_key":"([0-9a-f]{64})".*/\1/'; }
stat() { $CHECK stat "$@"; }

MTU=252
# oasa-link ports (radio side, plaintext in, plaintext out) and bridge listen ports.
G_ENC=7801; G_IN=7811; G_OUT=7812; G_BR=7821   # ground station
D_ENC=7802; D_IN=7813; D_OUT=7814; D_BR=7822   # drone

# --- identities and trust ---------------------------------------------------
for d in gcs drone; do $S/oasa-pair init --dir "$T/$d" --name "$d" >/dev/null; done
$S/oasa-pair trust --dir "$T/gcs" --name drone --key "$(pubkey "$T/drone")" >/dev/null
$S/oasa-pair trust --dir "$T/drone" --name gcs --key "$(pubkey "$T/gcs")" >/dev/null
pass "identities made and trusted with oasa-pair init / show / trust"

# --- the serial line: a pty pair with a 57600 baud limit, capturing both ways --
$R/examples/ptyrelay --link-a "$T/lineA" --link-b "$T/lineB" --baud 57600 \
  --capture-a "$T/capA.bin" --capture-b "$T/capB.bin" > "$T/relay.log" 2>&1 & PIDS+=($!)
wait_for 10 ready "$T/relay.log" || fail "relay did not start"
pass "serial relay up (57600 baud limit, capturing both directions)"

# --- two bridges, two links ---------------------------------------------------
$R/oasa-radio bridge --radio serial --device "$T/lineA" --baud 57600 --mtu $MTU \
  --udp-listen 127.0.0.1:$G_BR --udp-send 127.0.0.1:$G_ENC > "$T/brA.log" 2>&1 & BRA=$!; PIDS+=($BRA)
$R/oasa-radio bridge --radio serial --device "$T/lineB" --baud 57600 --mtu $MTU \
  --udp-listen 127.0.0.1:$D_BR --udp-send 127.0.0.1:$D_ENC > "$T/brB.log" 2>&1 & BRB=$!; PIDS+=($BRB)
wait_for 10 '"event":"start"' "$T/brA.log" && wait_for 10 '"event":"start"' "$T/brB.log" || fail "bridges did not start"

$S/oasa-link --dir "$T/drone" --peer gcs --bind 127.0.0.1:$D_ENC --remote 127.0.0.1:$D_BR --mtu $MTU \
  --plain-listen 127.0.0.1:$D_IN --plain-send 127.0.0.1:$D_OUT --duration 120 > "$T/drone.log" 2>&1 & DRONE=$!; PIDS+=($DRONE)
sleep 0.5
$S/oasa-link --dir "$T/gcs" --peer drone --bind 127.0.0.1:$G_ENC --remote 127.0.0.1:$G_BR --initiate --mtu $MTU \
  --plain-listen 127.0.0.1:$G_IN --plain-send 127.0.0.1:$G_OUT --duration 120 > "$T/gcs.log" 2>&1 & GCS=$!; PIDS+=($GCS)
wait_for 20 session_established "$T/gcs.log" || fail "ground station never got a session"
wait_for 20 session_established "$T/drone.log" || fail "drone never got a session"
pass "handshake crossed the serial line: both oasa-link processes log session_established"

# --- marked packets both ways --------------------------------------------------
$CHECK traffic 40 0.15 $G_IN $G_OUT $D_IN $D_OUT > "$T/traffic.out" || { cat "$T/traffic.out"; fail "marked packets were not delivered exactly once"; }
cat "$T/traffic.out"
pass "40 marked packets each way delivered exactly once, unchanged"

# --- the bytes on the line never hold the marker -------------------------------
$CHECK scan "$T/capA.bin" > "$T/scanA.out" && { cat "$T/scanA.out"; fail "marker seen on the serial line (ground station side)"; } || [ $? -eq 1 ] || { cat "$T/scanA.out"; fail "capture A too small or unreadable"; }
$CHECK scan "$T/capB.bin" > "$T/scanB.out" && { cat "$T/scanB.out"; fail "marker seen on the serial line (drone side)"; } || [ $? -eq 1 ] || { cat "$T/scanB.out"; fail "capture B too small or unreadable"; }
cat "$T/scanA.out" "$T/scanB.out"
pass "captured serial bytes (both directions) contain no marker"

# --- plaintext control: the scanner must see plaintext when there is some ------
# A packet sent straight into the bridge, bypassing oasa-link, goes out as is.
$CHECK inject $G_BR "PLAIN-CONTROL $(printf 'x%.0s' $(seq 80))" || fail "inject failed"
sleep 1
$CHECK scan "$T/capA.bin" PLAIN-CONTROL > "$T/ctl.out" || { cat "$T/ctl.out"; fail "plaintext control not found: the scanner is blind"; }
cat "$T/ctl.out"
pass "plaintext control: the same scanner finds a packet that skipped oasa-link"

# --- stats, exit codes -----------------------------------------------------------
kill -TERM "$BRA" "$BRB"
wait "$BRA" || fail "bridge A did not exit 0 on SIGTERM"
wait "$BRB" || fail "bridge B did not exit 0 on SIGTERM"
kill -TERM "$GCS" "$DRONE"
wait "$GCS" || fail "ground station did not exit 0"
wait "$DRONE" || fail "drone did not exit 0"
for f in brA brB gcs drone; do $CHECK count "$T/$f.log" stats | grep -qx 1 || fail "$f: not exactly one stats line"; done
pass "SIGTERM: every process exits 0 with a final stats line; every log line is JSON"

[ "$(stat "$T/gcs.log" delivered)" = 40 ] && [ "$(stat "$T/drone.log" delivered)" = 40 ] || fail "oasa-link delivered counts are not 40/40"
[ "$(stat "$T/brA.log" drops.bad_frame)" = 0 ] && [ "$(stat "$T/brB.log" drops.bad_frame)" = 0 ] || fail "bad frames on a clean line"
[ "$(stat "$T/brA.log" to_radio)" -ge 41 ] && [ "$(stat "$T/brB.log" from_radio)" -ge 41 ] || fail "bridge counts too low"
[ "$(stat "$T/drone.log" drops.bad_header)" -ge 1 ] || fail "drone did not drop the control packet"
pass "stats agree: 40/40 delivered, no bad frames, the control packet was dropped by oasa-link"

KEY=$(od -An -tx1 -v "$T/gcs/identity.key" | tr -d ' \n')
grep -q "$KEY" "$T"/*.log && fail "a private key is in a log"
grep -q "OASA-E2E-MARKER" "$T/brA.log" "$T/brB.log" && fail "a bridge log holds packet contents"
pass "no private key in any log; bridge logs hold no packet contents"

echo "All serial end-to-end checks passed."
