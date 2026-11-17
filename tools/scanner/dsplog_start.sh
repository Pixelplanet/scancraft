# Starts a background filter on the scanner: meaningful DSP/kernel lines go to /tmp/dsp.log.
# The control CGI passes quotes and ';' through raw, decodes %20 %3E %5C, but not %7C or %27, so
# the pipe is produced on the device by printf (\174). Long commands are rejected silently, so the
# script is written in short pieces.
IP=${1:-192.168.178.34}
run(){ q=$(python -I -c "import urllib.parse,sys;print(urllib.parse.quote(sys.argv[1], safe=\"';/=&-_.()\"))" "$1"); curl -s -m 15 "http://$IP/cgi-bin/zx_cmd.cgi?system_cmd=$q" >/dev/null; }
run "rm -f /tmp/dsplog.sh /tmp/dsp.log"
run "printf 'while true; do dmesg -c \174 grep -v ' > /tmp/dsplog.sh"
run "printf ' -e algo_task -e trigger.end ' >> /tmp/dsplog.sh"
run "printf ' -e hold.status -e rk1608_dev_ ' >> /tmp/dsplog.sh"
run "printf ' -e send.msg.len >> /tmp/dsp.log; ' >> /tmp/dsplog.sh"
run "printf 'sleep 0.2; done\n' >> /tmp/dsplog.sh"
# '&' would split the CGI query, so the backgrounding lives in a second script (\046 = '&').
run "printf 'sh /tmp/dsplog.sh > /dev/null 2> /dev/null \046\n' > /tmp/dsplog_bg.sh"
run "sh /tmp/dsplog_bg.sh"
