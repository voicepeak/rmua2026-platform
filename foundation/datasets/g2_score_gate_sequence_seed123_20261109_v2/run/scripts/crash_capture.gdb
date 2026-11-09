set pagination off
set confirm off
set print thread-events off
break *0x2fc8d80
commands
silent
set $cfg=*(unsigned long*)0x719f540
if $cfg != 0
printf "SEG actor=%p name=%u,%u idx=%d enter=%d event=%d components=%d score=%d\n",$rdi,*(unsigned int*)($rdi+0x18),*(unsigned int*)($rdi+0x1c),$esi,$edx,$ecx,*(int*)($rdi+0x248),*(int*)($cfg+0x2fc)
else
printf "SEG_CONFIG_NULL actor=%p idx=%d\n",$rdi,$esi
end
continue
end
break *0x2fb9960
commands
silent
set $cfg=*(unsigned long*)0x719f540
if $cfg != 0
printf "STATE old=%d new=%d flag=%d score=%d\n",*(int*)($rdi+0xd0),$esi,$edx,*(int*)($cfg+0x2fc)
else
printf "STATE_CONFIG_NULL old=%d new=%d\n",*(int*)($rdi+0xd0),$esi
end
continue
end
continue
