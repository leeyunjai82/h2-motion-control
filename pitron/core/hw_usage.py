"""
hw_usage.py — 이 PC 의 CPU / GPU(Intel 내장) / NPU(Intel) 사용률과 OpenVINO 실제 실행 장치 (root 없이 /proc·sysfs 만 읽음)

  CPU : /proc/stat 첫 줄 — 1 − Δ(idle+iowait) / Δtotal
  GPU : Intel 내장 GPU 가 쉰(RC6, 저전력) 시간 비율로 계산 — 1 − Δidle / Δt
          xe   : /sys/class/drm/cardN/device/tile*/gt*/gtidle/idle_residency_ms  (렌더·연산 GT 'gtN-rc' 우선)
          i915 : /sys/class/drm/cardN/gt/gt0/rc6_residency_ms  (옛 커널 cardN/power/rc6_residency_ms)
        ※ 근사값: 짧은 작업이 잦으면 RC6 에 못 들어가 실제보다 높게 나옴
  NPU : intel_vpu  /sys/class/accel/accelN/device/npu_busy_time_us  — Δbusy / Δt
        이 파일이 없는 커널이면 power/runtime_active_time (전원 켜진 시간 비율 — 실제 연산보다 높게 나옴)
  못 읽으면 pct None + why (제어 화면엔 '-', 마우스를 올리면 이유)

  hw = HwUsage(period=1.0).start()
  hw.latest()  → {"cpu": {"pct": 23.1, "src": "/proc/stat"}, "gpu": {"pct": 41.0, "src": "xe …", "note": "…"},
                  "npu": {"pct": None, "why": "…"}, "t": 1760000000.0}

  OpenVINO 가 실제로 쓰는 장치 (AUTO·HETERO 를 풀어서):
  exec_devices(compiled)  → ["GPU"] / ["NPU", "CPU"]
  yolo_devices(yolo)      → ultralytics YOLO(OpenVINO export) 의 실행 장치 (첫 추론 뒤에만, 그 전·PyTorch 모델은 [])

  확인 (이 PC 에서 어느 파일을 읽는지 + 1초마다 값):
  python pitron/core/hw_usage.py --sec 10
"""
import glob
import os
import threading
import time


def _read_num(path):
    with open(path) as f:
        return float(f.read().split()[0])


def _driver(dev_dir):
    """sysfs device 디렉터리의 커널 드라이버 이름 (xe / i915 / intel_vpu …). 없으면 ''."""
    p = os.path.join(dev_dir, "driver")
    return os.path.basename(os.path.realpath(p)) if os.path.exists(p) else ""


def _readable(path):
    """읽히면 None, 아니면 이유 문자열."""
    try:
        _read_num(path)
        return None
    except FileNotFoundError:
        return "없음"
    except PermissionError:
        return "권한 없음"
    except (OSError, ValueError, IndexError) as e:
        return str(e)


class _Counter:
    """누적 카운터 하나 → 구간 사용률 [%]. kind 'busy' = Δbusy/Δt, 'idle' = 1 − Δidle/Δt. scale = 카운터 1 의 초."""

    def __init__(self, path, kind, scale, src, note=""):
        self.path, self.kind, self.scale, self.src, self.note = path, kind, scale, src, note
        self.prev = None

    def sample(self, now):
        v = _read_num(self.path) * self.scale
        p, self.prev = self.prev, (now, v)
        if p is None:
            return None
        dt, dv = now - p[0], v - p[1]
        if dt <= 0 or dv < 0:                   # 첫 값·카운터 리셋
            return None
        frac = dv / dt
        if self.kind == "idle":
            frac = 1.0 - frac
        return 100.0 * min(1.0, max(0.0, frac))


def find_gpu(sys_root="/sys"):
    """Intel GPU 쉬는 시간 카운터 → (_Counter, '') 또는 (None, 이유)."""
    seen, errs = [], []
    for card in sorted(glob.glob(os.path.join(sys_root, "class/drm/card[0-9]*"))):
        if "-" in os.path.basename(card):       # card0-HDMI-A-1 같은 출력 단자
            continue
        dev = os.path.join(card, "device")
        drv = _driver(dev)
        seen.append(f"{os.path.basename(card)}:{drv or '?'}")
        if drv == "xe":
            files = glob.glob(os.path.join(dev, "tile*/gt*/gtidle/idle_residency_ms"))

            def rank(f):
                try:
                    with open(os.path.join(os.path.dirname(f), "name")) as fh:
                        name = fh.read().strip()
                except OSError:
                    name = ""
                return (0 if name.endswith("-rc") else 1, f)     # 렌더·연산 GT 먼저, 미디어 GT('-mc') 뒤
            files.sort(key=rank)
        elif drv == "i915":
            files = glob.glob(os.path.join(card, "gt/gt0/rc6_residency_ms")) + \
                    glob.glob(os.path.join(card, "power/rc6_residency_ms"))
        else:
            continue
        for f in files:
            why = _readable(f)
            rel = os.path.relpath(f, sys_root)
            if why is None:
                return _Counter(f, "idle", 1e-3, f"{drv} {rel}",
                                "GPU 가 저전력(RC6)에 못 들어간 시간 비율 — 짧은 작업이 잦으면 실제보다 높게 나옴"), ""
            errs.append(f"{rel} {why}")
        if not files:
            errs.append(f"{os.path.basename(card)} ({drv}) 쉬는 시간 파일 없음")
    return None, "Intel GPU 사용률 못 읽음: " + "; ".join(errs or [f"DRM 장치 {', '.join(seen) or '없음'}"])


def find_npu(sys_root="/sys"):
    """Intel NPU 사용 시간 카운터 → (_Counter, '') 또는 (None, 이유)."""
    seen, errs = [], []
    for acc in sorted(glob.glob(os.path.join(sys_root, "class/accel/accel[0-9]*"))):
        dev = os.path.join(acc, "device")
        drv = _driver(dev)
        seen.append(f"{os.path.basename(acc)}:{drv or '?'}")
        if drv != "intel_vpu":
            continue
        f = os.path.join(dev, "npu_busy_time_us")
        why = _readable(f)
        if why is None:
            return _Counter(f, "busy", 1e-6, f"intel_vpu {os.path.relpath(f, sys_root)}"), ""
        errs.append(f"npu_busy_time_us {why}")
        f = os.path.join(dev, "power/runtime_active_time")
        why = _readable(f)
        if why is None:
            return _Counter(f, "busy", 1e-3, f"intel_vpu {os.path.relpath(f, sys_root)}",
                            "npu_busy_time_us 없는 커널 → 전원 켜진 시간 비율 (실제 연산보다 높게 나옴)"), ""
        errs.append(f"runtime_active_time {why}")
    return None, "Intel NPU 사용률 못 읽음: " + "; ".join(errs or [f"accel 장치 {', '.join(seen) or '없음'} (intel_vpu 드라이버 확인)"])


class HwUsage:
    """period 초마다 CPU·GPU·NPU 사용률을 재서 latest() 로 돌려줌 (백그라운드 스레드 하나)."""

    def __init__(self, period=1.0, sys_root="/sys", proc_stat="/proc/stat"):
        self.period = float(period)
        self.proc_stat = proc_stat
        self.cpu_prev = None
        self.gpu, gwhy = find_gpu(sys_root)
        self.npu, nwhy = find_npu(sys_root)
        self.why = {"gpu": gwhy, "npu": nwhy}
        self.lock = threading.Lock()
        self.state = {k: {"pct": None, "why": "재는 중"} for k in ("cpu", "gpu", "npu")}
        self.state["t"] = None
        self._th = None

    def _cpu(self):
        with open(self.proc_stat) as f:
            v = [int(x) for x in f.readline().split()[1:9]]   # user nice system idle iowait irq softirq steal
        total, idle = sum(v), v[3] + v[4]
        p, self.cpu_prev = self.cpu_prev, (total, idle)
        if p is None or total <= p[0]:
            return None
        return 100.0 * min(1.0, max(0.0, 1.0 - (idle - p[1]) / (total - p[0])))

    def sample(self):
        now = time.monotonic()
        r = (lambda v: None if v is None else round(v, 1))
        out = {}
        try:
            out["cpu"] = {"pct": r(self._cpu()), "src": "/proc/stat"}
        except (OSError, ValueError) as e:
            out["cpu"] = {"pct": None, "why": f"/proc/stat: {e}"}
        for k in ("gpu", "npu"):
            c = getattr(self, k)
            if c is None:
                out[k] = {"pct": None, "why": self.why[k]}
                continue
            try:
                out[k] = {"pct": r(c.sample(now)), "src": c.src, "note": c.note}
            except (OSError, ValueError, IndexError) as e:
                out[k] = {"pct": None, "why": f"{c.src}: {e}"}
        out["t"] = time.time()
        with self.lock:
            self.state = out
        return out

    def _loop(self):
        while True:
            time.sleep(self.period)
            try:
                self.sample()
            except Exception as e:      # noqa: BLE001 — 사용률 표시 때문에 서버가 죽으면 안 됨
                print(f"[hw_usage] 측정 실패: {e}")

    def start(self):
        if self._th is None:
            self.sample()               # 기준값
            self._th = threading.Thread(target=self._loop, daemon=True, name="hw_usage")
            self._th.start()
        return self

    def latest(self):
        with self.lock:
            return dict(self.state)


def exec_devices(compiled):
    """OpenVINO CompiledModel 이 실제로 쓰는 장치 ['GPU'] / ['NPU', 'CPU'] (AUTO·HETERO 결과, 'GPU.0' → 'GPU'). 못 읽으면 []."""
    if compiled is None:
        return []
    try:
        devs = compiled.get_property("EXECUTION_DEVICES")
    except Exception:                   # noqa: BLE001 — 옛 OpenVINO·플러그인마다 다름
        return []
    if isinstance(devs, str):
        devs = devs.replace(",", " ").split()
    return list(dict.fromkeys(str(d).strip("() ").split(".")[0].upper() for d in devs if str(d).strip("() ")))


def yolo_devices(yolo):
    """ultralytics YOLO(OpenVINO export) 의 실행 장치. 첫 추론 전·PyTorch 모델이면 []."""
    backend = getattr(getattr(yolo, "predictor", None), "model", None)
    return exec_devices(getattr(backend, "ov_compiled_model", None))


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="CPU / GPU / NPU 사용률 — 어느 파일을 읽는지와 1초마다 값")
    ap.add_argument("--sec", type=float, default=10.0)
    ap.add_argument("--sys-root", default="/sys", help="시험용 가짜 sysfs 경로")
    ap.add_argument("--proc-stat", default="/proc/stat")
    a = ap.parse_args()
    hw = HwUsage(sys_root=a.sys_root, proc_stat=a.proc_stat)
    for k in ("gpu", "npu"):
        c = getattr(hw, k)
        print(f"{k.upper()}: {c.src}" + (f"  ({c.note})" if c.note else "") if c else f"{k.upper()}: {hw.why[k]}")
    hw.sample()
    t_end = time.time() + a.sec
    while time.time() < t_end:
        time.sleep(1.0)
        s = hw.sample()
        cols = []
        for k in ("cpu", "gpu", "npu"):
            p = s[k]["pct"]
            cols.append(f"{k.upper()} " + ("    -" if p is None else f"{p:5.1f}%"))
        print("  ".join(cols), flush=True)
