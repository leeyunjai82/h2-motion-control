#!/usr/bin/env bash
# 시스템 상태를 1 초마다 파일에 남김 (멈춰도 마지막 순간까지 디스크에 남게 매번 sync) — XPU 학습 중 멈춤 원인 찾기용
#   ./utils/sys_watch.sh                → logs/sys_watch_<날짜시각>.log   (Ctrl+C 로 끝)
# 한 줄: 시각 · 사용 가능 메모리 · 커널 slab (회수 불가) · 페이지 캐시 · 스왑 사용 · load · CPU 온도 최고 · 와이파이 상태 · 학습 프로세스 메모리
#   Panther Lake 보고: 와이파이(iwlwifi) 송신 중 커널 skbuff slab 폭주 → 메모리 바닥 (omarchy #5827),
#   xe 드라이버 TTM 풀이 MemAvailable 에서 수십 GB 빠져 보임 (Strata #2412) → 메모리 · slab 을 같이 기록
HERE="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$HERE/logs"
OUT="$HERE/logs/sys_watch_$(date +%Y%m%d_%H%M%S).log"
echo "[watch] → $OUT  (Ctrl+C 로 끝)"
echo "# 시각 avail_MB sunreclaim_MB cached_MB swap_used_MB load1 temp_max_C wifi train_rss_MB" | tee "$OUT"
while true; do
  m=$(awk '/^MemAvailable:/{a=$2} /^SUnreclaim:/{s=$2} /^Cached:/{c=$2} /^SwapTotal:/{st=$2} /^SwapFree:/{sf=$2}
           END{printf "%d %d %d %d", a/1024, s/1024, c/1024, (st-sf)/1024}' /proc/meminfo)
  l=$(cut -d' ' -f1 /proc/loadavg)
  t=$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1); t=$(( ${t:-0} / 1000 ))
  w=$(cat /sys/class/net/wl*/operstate 2>/dev/null | head -1); w=${w:-none}
  r=$(ps -C python -o rss=,args= 2>/dev/null | awk '/train_act/{s+=$1} END{printf "%d", s/1024}')
  echo "$(date +%H:%M:%S) $m $l $t $w $r" >> "$OUT"
  sync "$OUT" 2>/dev/null || sync
  sleep 1
done
