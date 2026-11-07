set pagination off
set confirm off
set print thread-events off
break *0x2fb7110
commands
silent
python import time; print('COLLISION monotonic=%.6f'%time.monotonic(),flush=True)
printf "state=%d object=%p\n",*(int*)($rdi+0xd0),$rdi
continue
end
break *0x2fb9960
commands
silent
python import time; print('STATE monotonic=%.6f'%time.monotonic(),flush=True)
printf "old=%d requested=%d flag=%d\n",*(int*)($rdi+0xd0),$esi,$edx
if $esi == 12
bt 6
end
continue
end
continue
