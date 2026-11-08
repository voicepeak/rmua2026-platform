set pagination off
set confirm off
set breakpoint pending on
set print thread-events off
set logging overwrite on
python import time
break *0x2fcc89a
commands
silent
python print("RMUA_TRACE_CLOCK wall_unix=%.9f monotonic=%.9f" % (time.time(), time.monotonic()), flush=True)
printf "RMUA_RANDOM_INIT singleton=%p configured=%u initial=%u current=%u\n", $r12, *(unsigned int*)($r12+0x198), *(unsigned int*)($r12+0x19c), *(unsigned int*)($r12+0x1a0)
continue
end
break *0x2fba970
commands
silent
python print("RMUA_TRACE_CLOCK wall_unix=%.9f monotonic=%.9f" % (time.time(), time.monotonic()), flush=True)
set $singleton = *(unsigned long*)0x71a1c18
if $singleton != 0
printf "RMUA_GAME_RESET state=%p singleton=%p initial=%u current=%u\n", $rdi, $singleton, *(unsigned int*)($singleton+0x19c), *(unsigned int*)($singleton+0x1a0)
else
printf "RMUA_GAME_RESET state=%p singleton=NULL\n", $rdi
end
continue
end
break *0x2fc1360
commands
silent
python print("RMUA_TRACE_CLOCK wall_unix=%.9f monotonic=%.9f" % (time.time(), time.monotonic()), flush=True)
set $actor = $rdi
set $root = *(unsigned long*)($actor+0x130)
if $root != 0
printf "RMUA_GATE actor=%p fname_index=%u fname_number=%u root=%p xyz_cm=%.6f,%.6f,%.6f quat=%.8f,%.8f,%.8f,%.8f scale=%.6f,%.6f,%.6f\n", $actor, *(unsigned int*)($actor+0x18), *(unsigned int*)($actor+0x1c), $root, *(float*)($root+0x1d0), *(float*)($root+0x1d4), *(float*)($root+0x1d8), *(float*)($root+0x1c0), *(float*)($root+0x1c4), *(float*)($root+0x1c8), *(float*)($root+0x1cc), *(float*)($root+0x1e0), *(float*)($root+0x1e4), *(float*)($root+0x1e8)
else
printf "RMUA_GATE actor=%p root=NULL\n", $actor
end
continue
end
break *0x2fb2040
commands
silent
python print("RMUA_TRACE_CLOCK wall_unix=%.9f monotonic=%.9f" % (time.time(), time.monotonic()), flush=True)
printf "RMUA_SRAND context=%p seed=%u\n", $rdi, $esi
continue
end
break *0x2fb23b0
commands
silent
python print("RMUA_TRACE_CLOCK wall_unix=%.9f monotonic=%.9f" % (time.time(), time.monotonic()), flush=True)
printf "RMUA_WIND_GENERATE context=%p xmm0=%.8f xmm1=%.8f xmm2=%.8f xmm3=%.8f\n", $rdi, $xmm0.v4_float[0], $xmm1.v4_float[0], $xmm2.v4_float[0], $xmm3.v4_float[0]
continue
end
break *0x2fc77b0
commands
silent
python print("RMUA_TRACE_CLOCK wall_unix=%.9f monotonic=%.9f" % (time.time(), time.monotonic()), flush=True)
printf "RMUA_LINE object=%p rsi=%p rdx=%p rcx=%p r8=%p r9=%p xmm0=%.8f xmm1=%.8f xmm2=%.8f xmm3=%.8f\n", $rdi, $rsi, $rdx, $rcx, $r8, $r9, $xmm0.v4_float[0], $xmm1.v4_float[0], $xmm2.v4_float[0], $xmm3.v4_float[0]
continue
end
break *0x2fc8900
commands
silent
python print("RMUA_TRACE_CLOCK wall_unix=%.9f monotonic=%.9f" % (time.time(), time.monotonic()), flush=True)
printf "RMUA_VEHICLE_BRANCH object=%p xmm0=%.8f\n", $rdi, $xmm0.v4_float[0]
continue
end
break *0x2fb7110
commands
silent
python print("RMUA_COLLISION_CALLBACK wall_unix=%.9f monotonic=%.9f" % (time.time(), time.monotonic()), flush=True)
printf "RMUA_COLLISION state=%d object=%p\n", *(int*)($rdi+0xd0), $rdi
bt 6
continue
end
break *0x2fb9960
commands
silent
python print("RMUA_TRACE_CLOCK wall_unix=%.9f monotonic=%.9f" % (time.time(), time.monotonic()), flush=True)
printf "RMUA_STATE_ENTER object=%p old_state=%d requested_state=%d flag=%d timer_1a8=%.6f timer_1ac=%.6f\n", $rdi, *(int*)($rdi+0xd0), $esi, $edx, *(float*)($rdi+0x1a8), *(float*)($rdi+0x1ac)
if $esi == 12
 bt 8
end
continue
end
break *0x2fbac30
commands
silent
python print("RMUA_TRACE_CLOCK wall_unix=%.9f monotonic=%.9f" % (time.time(), time.monotonic()), flush=True)
printf "RMUA_STAGE_INIT object=%p arg1=%u arg2=%u stage=%u timer_1a8=%.6f timer_1ac=%.6f\n", $rdi, $esi, $edx, $ecx, *(float*)($rdi+0x1a8), *(float*)($rdi+0x1ac)
continue
end
source /analysis/scripts/predicate_pose_probe.gdb
source /analysis/scripts/charge_state_probe.py
run
