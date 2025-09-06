#!/usr/bin/env bash

echo Setting up dexi.service...

sudo cp "$(dirname "$0")/dexi.service" /etc/systemd/system/

# Copy default config if it doesn't exist
if [ ! -f /home/dexi/.dexi-config.yaml ]; then
    echo "Copying default configuration..."
    sudo cp "$(dirname "$0")/../config/dexi_config.yaml" /home/dexi/.dexi-config.yaml
    sudo chown dexi:dexi /home/dexi/.dexi-config.yaml
    echo "Configuration copied to /home/dexi/.dexi-config.yaml"
else
    echo "Configuration file already exists at /home/dexi/.dexi-config.yaml"
fi

sudo systemctl daemon-reload
sudo systemctl enable dexi.service
sudo systemctl restart dexi.service

echo "Dexi service has been installed and started."
echo "Use 'sudo systemctl status dexi.service' to check status."