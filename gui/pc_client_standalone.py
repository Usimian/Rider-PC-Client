#!/usr/bin/env python3
# coding=utf-8

# Rider Robot PC Client - Standalone Version
# MQTT-based remote control and monitoring client for the XGO Rider Robot
# Marc Wester

import argparse
import signal
import sys
from core.app_controller import ApplicationController

def main():
    parser = argparse.ArgumentParser(description='Rider Robot PC Client - Standalone Version')
    parser.add_argument('-d', '--debug', action='store_true', 
                       help='Enable debug messages showing all MQTT traffic')
    args = parser.parse_args()
    
    print("Rider Robot PC Client - Standalone Version (Refactored)")
    print("========================================================")
    if args.debug:
        print("🐛 Debug mode enabled - showing all MQTT traffic")
    else:
        print("ℹ️ Use -d or --debug flag to see detailed MQTT traffic")
    print("🛡️ Enhanced with graceful disconnect to prevent robot corruption")
    
    app_controller = None
    try:
        app_controller = ApplicationController(debug=args.debug)
        app_controller.run()
    except KeyboardInterrupt:
        print("\n⌨️ Keyboard interrupt in main...")
    except Exception as e:
        print(f"❌ Main error: {e}")
    finally:
        if app_controller:
            try:
                # Immediate cleanup without timeout - timeout handling is now in window close handler
                app_controller.cleanup()
            except Exception as e:
                print(f"⚠️ Final cleanup error: {e}")
                print("🚪 Forcing exit...")
                import os
                os._exit(0)
        
        print("👋 Application terminated")

if __name__ == "__main__":
    main() 