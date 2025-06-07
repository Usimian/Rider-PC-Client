#!/usr/bin/env python3
# coding=utf-8

import configparser
import os

class ConfigManager:
    def __init__(self, config_file='rider_config.ini'):
        self.config_file = config_file
        self.config = configparser.ConfigParser()
        self.load_config()
    
    def load_config(self):
        """Load configuration from file or create default"""
        if os.path.exists(self.config_file):
            self.config.read(self.config_file)
        else:
            # Create default config
            self.config['mqtt'] = {
                'broker_host': '192.168.1.173',
                'broker_port': '1883'
            }
            self.save_config()
    
    def save_config(self):
        """Save current configuration to file"""
        with open(self.config_file, 'w') as f:
            self.config.write(f)
    
    def get_broker_host(self):
        """Get MQTT broker host"""
        return self.config.get('mqtt', 'broker_host', fallback='192.168.1.173')
    
    def get_broker_port(self):
        """Get MQTT broker port"""
        return self.config.getint('mqtt', 'broker_port', fallback=1883)
    
    def set_broker_host(self, host):
        """Set MQTT broker host and save"""
        self.config.set('mqtt', 'broker_host', host)
        self.save_config()
    
    def set_broker_port(self, port):
        """Set MQTT broker port and save"""
        self.config.set('mqtt', 'broker_port', str(port))
        self.save_config() 