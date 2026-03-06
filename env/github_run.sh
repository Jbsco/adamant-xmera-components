#!/bin/sh

this_dir=`dirname "$0"`

# Install cmake and compile fp32-fsw-xmera algorithms
sudo apt-get -y install cmake
cd $this_dir/../../fp32-fsw-xmera
./build.sh linux-gcc-debug
cd - >/dev/null

# Set the environment for the github command:
. $this_dir/activate

# Run the command passed to the script:
echo "$ $@"
eval "$@"
status=$?

# On test failure, print detailed error output from coverage logs so that
# the diagnosis is visible directly in the GitHub Actions log.
if [ $status -ne 0 ]; then
  case "$*" in
    *coverage_all*|*test_all*)
      project_dir=$(cd "$this_dir/.." && pwd)
      log_dir="$project_dir/build/coverage_logs"
      summary="$project_dir/build/coverage_all_summary.txt"

      if [ ! -d "$log_dir" ]; then
        echo "No coverage_logs directory found at $log_dir"
        exit $status
      fi

      echo ""
      echo "=========================================="
      echo "  Test failure detected — detailed output"
      echo "=========================================="

      # Parse the summary to find only the failed tests. Strip ANSI color
      # codes so the FAILED token is matched reliably.
      failed_logs=""
      if [ -f "$summary" ]; then
        stripped=$(sed 's/\x1b\[[0-9;]*m//g' "$summary")
        failed_logs=$(echo "$stripped" | awk '/FAILED$/ {print $2}' |
          while read -r rel_path; do
            log_name=$(echo "$rel_path" | sed 's#/#_#g')
            echo "$log_dir/${log_name}.log"
          done)
      fi

      # Fallback: if no summary or no FAILED lines found, show all logs.
      if [ -z "$failed_logs" ]; then
        failed_logs=$(find "$log_dir" -name '*.log' -type f)
      fi

      for f in $failed_logs; do
        [ -f "$f" ] || continue
        echo ""
        echo "--- $(basename "$f" .log) ---"
        tail -n 100 "$f"
      done
      ;;
  esac
fi

exit $status
