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
                'broker_host': '192.168.1.130',
                'broker_port': '1883'
            }
            self.config['llm'] = {
                'ollama_url': 'http://localhost:11434',
                'default_model': 'llava:7b',
                'temperature': '0.7',
                'max_tokens': '500',
                'enabled': 'true'
            }
            self.save_config()
    
    def save_config(self):
        """Save current configuration to file"""
        with open(self.config_file, 'w') as f:
            self.config.write(f)
    
    def get_broker_host(self):
        """Get MQTT broker host"""
        return self.config.get('mqtt', 'broker_host', fallback='192.168.1.130')
    
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
    
    # LLM Configuration Methods
    def get_ollama_url(self):
        """Get ollama server URL"""
        return self.config.get('llm', 'ollama_url', fallback='http://localhost:11434')
    
    def get_llm_default_model(self):
        """Get default LLM model"""
        return self.config.get('llm', 'default_model', fallback='llava:7b')
    
    def get_llm_temperature(self):
        """Get LLM temperature setting"""
        return self.config.getfloat('llm', 'temperature', fallback=0.7)
    
    def get_llm_max_tokens(self):
        """Get LLM max tokens setting"""
        return self.config.getint('llm', 'max_tokens', fallback=500)
    
    def is_llm_enabled(self):
        """Check if LLM features are enabled"""
        return self.config.getboolean('llm', 'enabled', fallback=True)
    
    def set_ollama_url(self, url):
        """Set ollama server URL and save"""
        if 'llm' not in self.config:
            self.config.add_section('llm')
        self.config.set('llm', 'ollama_url', url)
        self.save_config()
    
    def set_llm_default_model(self, model):
        """Set default LLM model and save"""
        if 'llm' not in self.config:
            self.config.add_section('llm')
        self.config.set('llm', 'default_model', model)
        self.save_config()
    
    def set_llm_temperature(self, temperature):
        """Set LLM temperature and save"""
        if 'llm' not in self.config:
            self.config.add_section('llm')
        self.config.set('llm', 'temperature', str(temperature))
        self.save_config()
    
    def set_llm_max_tokens(self, max_tokens):
        """Set LLM max tokens and save"""
        if 'llm' not in self.config:
            self.config.add_section('llm')
        self.config.set('llm', 'max_tokens', str(max_tokens))
        self.save_config()
    
    def set_llm_enabled(self, enabled):
        """Set LLM enabled status and save"""
        if 'llm' not in self.config:
            self.config.add_section('llm')
        self.config.set('llm', 'enabled', str(enabled).lower())
        self.save_config() 