#!/usr/bin/env bash

echo Setting up dexi.service...

sudo cp "$(dirname "$0")/dexi.service" /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable dexi.service
sudo systemctl restart dexi.service

echo "Dexi service has been installed and started."
echo "Use 'sudo systemctl status dexi.service' to check status."