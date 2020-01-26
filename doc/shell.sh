cp test.service /etc/systemd/system
cp test.timer /etc/systemd/system
systemctl daemon-reload
systemctl enable test.timer
reboot