#!/usr/bin/env python3
"""Sample child RSS and total memory while running a command."""
import json, pathlib, subprocess, sys, time
if len(sys.argv) < 4:
    raise SystemExit("uso: medir_recursos.py SALIDA_LOG SALIDA_JSON COMANDO [ARGS...]")
log_path, metrics_path = map(pathlib.Path, sys.argv[1:3])
cmd = sys.argv[3:]
samples = []
start = time.time()
log_path.parent.mkdir(parents=True, exist_ok=True)
with log_path.open("w", buffering=1) as log:
    child = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
    while child.poll() is None:
        sample = {"elapsed_s": round(time.time() - start, 2)}
        try:
            status = pathlib.Path(f"/proc/{child.pid}/status").read_text()
            for line in status.splitlines():
                if line.startswith(("VmRSS:", "VmHWM:", "VmSize:")):
                    key, value = line.split(":", 1)
                    sample[key] = int(value.strip().split()[0])
            for line in pathlib.Path("/proc/meminfo").read_text().splitlines():
                if line.startswith(("MemAvailable:", "MemFree:", "SwapFree:")):
                    key, value = line.split(":", 1)
                    sample[key] = int(value.strip().split()[0])
        except (OSError, ValueError):
            pass
        samples.append(sample)
        time.sleep(1)
    rc = child.wait()
metrics = {
    "command": cmd, "pid": child.pid, "exit_code": rc,
    "duration_s": round(time.time() - start, 2), "sample_count": len(samples),
    "peak_rss_kb": max((s.get("VmRSS", 0) for s in samples), default=0),
    "peak_hwm_kb": max((s.get("VmHWM", 0) for s in samples), default=0),
    "min_mem_available_kb": min((s.get("MemAvailable", 0) for s in samples), default=0),
    "min_swap_free_kb": min((s.get("SwapFree", 0) for s in samples), default=0),
    "samples": samples,
}
metrics_path.parent.mkdir(parents=True, exist_ok=True)
metrics_path.write_text(json.dumps(metrics, indent=2))
print(json.dumps({k: metrics[k] for k in ("exit_code", "duration_s", "sample_count", "peak_rss_kb", "peak_hwm_kb", "min_mem_available_kb", "min_swap_free_kb")}, indent=2))
raise SystemExit(rc)

