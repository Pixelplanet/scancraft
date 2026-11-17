# Prints the scanner's filtered DSP log (see dsplog_start.sh) and empties it.
IP=${1:-192.168.178.34}
curl -s -m 15 "http://$IP/cgi-bin/zx_cmd.cgi?download=/tmp/dsp.log" | sed -E 's/^\[ *[0-9.]+\] //; s/rk1608 spi1.0: DSP\(0\): *[0-9.]+: //'
curl -s -m 5 "http://$IP/cgi-bin/zx_cmd.cgi?system_cmd=cp%20/dev/null%20/tmp/dsp.log" >/dev/null
