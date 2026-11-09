set pagination off
set confirm off
set print thread-events off
set $cfg=*(unsigned long*)0x719f540
printf "CONFIG %p initial_score=%d\n",$cfg,*(int*)($cfg+0x2fc)
watch *(int*)($cfg+0x2fc)
commands
silent
printf "SCORE value=%d pc=%p\n",*(int*)($cfg+0x2fc),$pc
bt 5
continue
end
break *0x2fc8d80
commands
silent
printf "SEG actor=%p name=%u,%u idx=%d enter=%d event=%d components=%d score=%d\n",$rdi,*(unsigned int*)($rdi+0x18),*(unsigned int*)($rdi+0x1c),$esi,$edx,$ecx,*(int*)($rdi+0x248),*(int*)($cfg+0x2fc)
continue
end
break *0x2fb9960
commands
silent
printf "STATE old=%d new=%d flag=%d score=%d\n",*(int*)($rdi+0xd0),$esi,$edx,*(int*)($cfg+0x2fc)
continue
end
continue
