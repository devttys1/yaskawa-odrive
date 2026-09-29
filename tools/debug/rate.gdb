# Measures the decode success rate directly, bypassing the UART entirely.
# Useful when console output is unreliable, or to get a clean pass rate.
#
#   ./tools/debug/gdb.sh -f tools/debug/rate.gdb
#
monitor halt
set $ok = 0
set $bad = 0
set $i = 0
set $f = (yask_frame_t *) malloc(sizeof(yask_frame_t))
while $i < 60
  set $r = (int) yask_read($f)
  if ($r == 0)
    set $ok = $ok + 1
  else
    set $bad = $bad + 1
  end
  set $i = $i + 1
end
printf "=== %d reads: %d OK, %d failed (%d%% good) ===\n", $i, $ok, $bad, ($ok*100)/$i
printf "last frame: st=%u mt=%u rot3=%u hall=%u accel=%d ts_fast=%u crc=0x%04x %s\n", $f->single_turn, $f->multi_turn, $f->rot3, $f->hall, $f->accel, $f->ts_fast, $f->crc_rx, $f->crc_ok ? "OK" : "BAD"
detach
