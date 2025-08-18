function api_request() {
  local method="${1:-GET}"
  local path="$2"
  local body="$3"
  local server="${4:-192.168.1.159}"

  if [[ "$method" = "GET" ]] || [[ "$method" = "DELETE" ]]; then
    curl \
      -s \
      -H "Content-Type: application/json" \
      -H "Accept: application/json" \
      http://$server/$path
  else
    curl -X$method \
      -s \
      -H "Content-Type: application/json" \
      -H "Accept: application/json" \
      --data-raw "$body" \
      http://$server/$path
  fi
}
