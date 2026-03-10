#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# RPi Car — Grafana telemetry stack setup script
# Run this ONCE on 192.168.29.105 (the Grafana server).
#
# What it does:
#   1. Checks if InfluxDB + Telegraf containers are already running (idempotent)
#   2. Starts InfluxDB 2.7 + Telegraf via docker compose (if needed)
#   3. Detects the existing Grafana container and its Docker network
#   4. Connects InfluxDB + Telegraf to the same network so Grafana can reach them
#   5. Copies provisioning files into the Grafana container
#   6. Reloads Grafana config (or imports dashboard via API as fallback)
#
# Usage:
#   chmod +x server_setup.sh
#   ./server_setup.sh
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

INFLUX_TOKEN="rpicar-secret-token-change-me"
INFLUX_ORG="rpicar"
INFLUX_BUCKET="car_telemetry"
INFLUX_URL_INTERNAL="http://rpicar_influxdb:8086"

GRAFANA_PORT="${GRAFANA_PORT:-2000}"    # external port Grafana is bound to

info()    { echo -e "  \033[32m✓\033[0m  $*"; }
warn()    { echo -e "  \033[33m⚠\033[0m  $*"; }
section() { echo -e "\n\033[1;34m══  $*  ══\033[0m"; }
die()     { echo -e "  \033[31m✗\033[0m  $*" >&2; exit 1; }

# ─────────────────────── 0. Pre-flight checks ─────────────────────────────────
section "Pre-flight checks"
command -v docker >/dev/null 2>&1 || die "docker not found — install Docker first"
docker info >/dev/null 2>&1       || die "Docker daemon not running"
info "Docker is available"

# ─────────────────────── 1. Find existing Grafana container ──────────────────
section "Detecting existing Grafana container"

GRAFANA_CONTAINER=""
# Try common Grafana container names
for name in grafana grafana-server rpicar_grafana; do
  if docker inspect "$name" >/dev/null 2>&1; then
    GRAFANA_CONTAINER="$name"
    break
  fi
done

# Fallback: find any container running grafana image
if [[ -z "$GRAFANA_CONTAINER" ]]; then
  GRAFANA_CONTAINER=$(docker ps --format '{{.Names}}' --filter "ancestor=grafana/grafana" | head -1 || true)
fi
if [[ -z "$GRAFANA_CONTAINER" ]]; then
  GRAFANA_CONTAINER=$(docker ps --format '{{.Names}}' | xargs -I{} docker inspect {} --format '{{.Name}} {{.Config.Image}}' 2>/dev/null | grep -i grafana | awk '{print $1}' | tr -d / | head -1 || true)
fi

if [[ -z "$GRAFANA_CONTAINER" ]]; then
  warn "Could not auto-detect a running Grafana container."
  warn "Please enter the container name (or press Enter to skip Grafana integration):"
  read -r GRAFANA_CONTAINER || true
fi

if [[ -n "$GRAFANA_CONTAINER" ]]; then
  info "Found Grafana container: $GRAFANA_CONTAINER"
  GRAFANA_NETWORK=$(docker inspect "$GRAFANA_CONTAINER" \
    --format '{{range $k,$v := .NetworkSettings.Networks}}{{$k}} {{end}}' \
    | awk '{print $1}' || true)
  info "Grafana is on network: ${GRAFANA_NETWORK:-<default bridge>}"
else
  warn "Skipping Grafana container integration (no container found)"
  GRAFANA_NETWORK=""
fi

# ─────────────────────── 2. Check / start InfluxDB & Telegraf ─────────────────
section "Starting InfluxDB + Telegraf"

cd "$SCRIPT_DIR"

if docker ps --format '{{.Names}}' | grep -q "^rpicar_influxdb$"; then
  info "rpicar_influxdb already running — skipping"
else
  info "Starting InfluxDB 2.7..."
  docker compose up -d influxdb
fi

if docker ps --format '{{.Names}}' | grep -q "^rpicar_telegraf$"; then
  info "rpicar_telegraf already running — skipping"
else
  info "Starting Telegraf..."
  docker compose up -d telegraf
fi

# ─────────────────────── 3. Wait for InfluxDB to be healthy ───────────────────
section "Waiting for InfluxDB health check"
MAX_WAIT=60
ELAPSED=0
until curl -sf "http://localhost:8086/health" | grep -q '"status":"pass"'; do
  if (( ELAPSED >= MAX_WAIT )); then
    die "InfluxDB did not become healthy within ${MAX_WAIT}s"
  fi
  echo -n "."
  sleep 2
  (( ELAPSED += 2 ))
done
echo ""
info "InfluxDB is healthy"

# ─────────────────────── 4. Connect containers to Grafana's network ──────────
section "Connecting containers to Grafana's Docker network"
if [[ -n "$GRAFANA_NETWORK" && "$GRAFANA_NETWORK" != "bridge" ]]; then
  for svc in rpicar_influxdb rpicar_telegraf; do
    ALREADY=$(docker network inspect "$GRAFANA_NETWORK" \
      --format '{{range .Containers}}{{.Name}} {{end}}' 2>/dev/null || true)
    if echo "$ALREADY" | grep -qw "$svc"; then
      info "$svc is already on network $GRAFANA_NETWORK"
    else
      docker network connect "$GRAFANA_NETWORK" "$svc" \
        && info "Connected $svc → $GRAFANA_NETWORK" \
        || warn "Could not connect $svc to $GRAFANA_NETWORK (may not be needed)"
    fi
  done
  INFLUX_URL_INTERNAL="http://rpicar_influxdb:8086"
else
  # If Grafana is on plain 'bridge' or unknown, use host IP instead
  INFLUX_URL_INTERNAL="http://host.docker.internal:8086"
  warn "Grafana network is 'bridge' or unknown; using host.docker.internal for InfluxDB URL"
  warn "You may need to manually update the datasource URL in Grafana if this doesn't work"
fi

# ─────────────────────── 5. Copy provisioning files into Grafana ─────────────
section "Provisioning Grafana datasource + dashboard"
if [[ -n "$GRAFANA_CONTAINER" ]]; then
  # Ensure provisioning directories exist
  docker exec "$GRAFANA_CONTAINER" mkdir -p \
    /etc/grafana/provisioning/datasources \
    /etc/grafana/provisioning/dashboards

  # Update datasource URL to match actual connectivity
  TMP_DS=$(mktemp)
  sed "s|http://rpicar_influxdb:8086|${INFLUX_URL_INTERNAL}|g" \
    "$SCRIPT_DIR/provisioning/datasources/influxdb.yml" > "$TMP_DS"

  docker cp "$TMP_DS" \
    "$GRAFANA_CONTAINER:/etc/grafana/provisioning/datasources/influxdb.yml"
  docker cp "$SCRIPT_DIR/provisioning/dashboards/dashboard.yml" \
    "$GRAFANA_CONTAINER:/etc/grafana/provisioning/dashboards/dashboard.yml"
  docker cp "$SCRIPT_DIR/provisioning/dashboards/rpicar_dashboard.json" \
    "$GRAFANA_CONTAINER:/etc/grafana/provisioning/dashboards/rpicar_dashboard.json"
  rm -f "$TMP_DS"
  info "Provisioning files copied into $GRAFANA_CONTAINER"

  # Attempt a graceful Grafana reload via HTTP API (no container restart needed)
  GRAFANA_ADMIN="${GRAFANA_ADMIN_USER:-admin}"
  GRAFANA_PASS="${GRAFANA_ADMIN_PASS:-admin}"
  GRAFANA_API="http://localhost:${GRAFANA_PORT}"

  info "Attempting Grafana provisioning reload via API..."
  if curl -sf -u "${GRAFANA_ADMIN}:${GRAFANA_PASS}" \
       -X POST "${GRAFANA_API}/api/admin/provisioning/datasources/reload" \
       -o /dev/null; then
    info "Datasource provisioning reloaded"
  else
    warn "API reload failed (wrong admin password?). Restarting Grafana container..."
    docker restart "$GRAFANA_CONTAINER"
    info "Grafana restarted — wait ~10s then open http://localhost:${GRAFANA_PORT}"
  fi

  # Also import dashboard via API as a belt-and-suspenders approach
  info "Importing dashboard via Grafana API..."
  DASH_PAYLOAD=$(python3 -c "
import json, sys
with open('$SCRIPT_DIR/provisioning/dashboards/rpicar_dashboard.json') as f:
    dash = json.load(f)
dash['id'] = None
payload = {'dashboard': dash, 'overwrite': True, 'folderId': 0}
print(json.dumps(payload))
")
  HTTP_CODE=$(curl -sf -u "${GRAFANA_ADMIN}:${GRAFANA_PASS}" \
    -X POST "${GRAFANA_API}/api/dashboards/db" \
    -H "Content-Type: application/json" \
    -d "$DASH_PAYLOAD" \
    -o /tmp/grafana_import.log \
    -w "%{http_code}" || true)

  if [[ "$HTTP_CODE" == "200" ]]; then
    DASH_URL=$(python3 -c "import json; d=json.load(open('/tmp/grafana_import.log')); print(d.get('url',''))" 2>/dev/null || true)
    info "Dashboard imported! Open: http://localhost:${GRAFANA_PORT}${DASH_URL}"
  else
    warn "Dashboard API import returned HTTP $HTTP_CODE (see /tmp/grafana_import.log)"
    warn "Dashboard will load automatically from file provisioning after Grafana restarts"
  fi
else
  warn "No Grafana container — skipping file provisioning"
  warn "Manually copy provisioning/ into your Grafana container and restart it"
fi

# ─────────────────────── 6. Quick smoke test ─────────────────────────────────
section "Smoke test"
info "Testing Telegraf listener..."
TEST_RESULT=$(curl -sf -o /dev/null -w "%{http_code}" \
  -X POST "http://localhost:8186/write?db=car_telemetry" \
  --data-binary "smoke_test,host=server value=1.0" || true)
if [[ "$TEST_RESULT" == "204" || "$TEST_RESULT" == "200" ]]; then
  info "Telegraf listener is accepting data (HTTP $TEST_RESULT)"
else
  warn "Telegraf returned HTTP $TEST_RESULT for smoke test POST"
fi

# ─────────────────────── 7. Summary ──────────────────────────────────────────
section "Setup Complete"
echo ""
echo "  InfluxDB 2.7  →  http://localhost:8086  (org: rpicar, bucket: car_telemetry)"
echo "  Telegraf      →  http://localhost:8186  (RPi car POSTs here)"
echo "  Grafana       →  http://localhost:${GRAFANA_PORT}  (dashboard: 🚗 RPi Car — Real-Time Telemetry)"
echo ""
echo "  On the RPi car:"
echo "    python3 scripts/main.py   ← postman starts automatically in background"
echo ""
echo "  Verify data is flowing:"
echo "    curl -G 'http://localhost:8086/api/v2/query' \\"
echo "         -H 'Authorization: Token ${INFLUX_TOKEN}' \\"
echo "         -H 'Content-Type: application/vnd.flux' \\"
echo "         --data-urlencode 'org=${INFLUX_ORG}' \\"
echo "         --data-urlencode 'q=from(bucket:\"${INFLUX_BUCKET}\") |> range(start:-1m) |> last()'"
echo ""
echo "  ⚠️  Change default credentials in docker-compose.yml before exposing externally!"
