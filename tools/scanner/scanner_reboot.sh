# Soft-reboots the POP 4 through its control CGI and waits until it answers again (~35 s).
IP=${1:-192.168.178.34}
curl -s -m 5 "http://$IP/cgi-bin/zx_cmd.cgi?system_cmd=sync" >/dev/null
curl -s -m 5 "http://$IP/cgi-bin/zx_cmd.cgi?system_cmd=reboot" >/dev/null
ping -n 12 127.0.0.1 >/dev/null
for i in $(seq 1 40); do
  r=$(curl -s -m 2 "http://$IP/cgi-bin/zx_cmd.cgi?cam_type=mipi&get_depth_output_fmt")
  case "$r" in *result*) echo "scanner up (poll $i)"; exit 0;; esac
  ping -n 3 127.0.0.1 >/dev/null
done
echo "scanner did not come back"; exit 1
