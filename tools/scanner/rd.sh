# usage: rd.sh "<shell command without pipes>"  -> runs it on the scanner, prints its output
q=$(python -I -c "import urllib.parse,sys;print(urllib.parse.quote(sys.argv[1]))" "$1 > /tmp/sc_out.txt 2>&1")
curl -s -m 15 "http://192.168.178.34/cgi-bin/zx_cmd.cgi?system_cmd=$q" >/dev/null
curl -s -m 15 "http://192.168.178.34/cgi-bin/zx_cmd.cgi?download=/tmp/sc_out.txt"
echo
