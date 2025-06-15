#!/usr/bin/env python3
# coding=utf-8

import requests
import json
import time
from typing import Dict, Any, List, Optional, Callable

class OllamaClient:
    def __init__(self, base_url: str = "http://localhost:11434", debug: bool = False):
        self.base_url = base_url.rstrip('/')
        self.debug_mode = debug
        self.session = requests.Session()
        self.session.timeout = 30
        
        if debug:
            print(f"🤖 Ollama client initialized with base URL: {base_url}")
    
    def is_server_available(self) -> bool:
        """Check if ollama server is running and accessible"""
        try:
            response = self.session.get(f"{self.base_url}/api/tags", timeout=5)
            available = response.status_code == 200
            if self.debug_mode:
                print(f"🔍 Ollama server availability check: {'✅ Available' if available else '❌ Unavailable'}")
            return available
        except Exception as e:
            if self.debug_mode:
                print(f"🔍 Ollama server check failed: {e}")
            return False
    
    def get_available_models(self) -> List[str]:
        """Get list of available models from ollama server"""
        try:
            response = self.session.get(f"{self.base_url}/api/tags")
            response.raise_for_status()
            
            data = response.json()
            models = []
            
            for model_info in data.get('models', []):
                model_name = model_info.get('name', '')
                if model_name:
                    models.append(model_name)
            
            if self.debug_mode:
                print(f"🤖 Available models: {models}")
            
            return models
        except Exception as e:
            if self.debug_mode:
                print(f"❌ Failed to get models: {e}")
            return []
    
    def generate_with_image(self, model: str, prompt: str, image_data: str, 
                           temperature: float = 0.7, max_tokens: int = 500,
                           response_callback: Optional[Callable[[str], None]] = None) -> Dict[str, Any]:
        """
        Generate response from vision-language model with image
        
        Args:
            model: Model name (e.g., 'llava:7b')
            prompt: Text prompt for the model
            image_data: Base64 encoded image data
            temperature: Sampling temperature (0.0 to 2.0)
            max_tokens: Maximum tokens to generate
            response_callback: Optional callback for streaming responses
        
        Returns:
            Dict with response data or error information
        """
        try:
            payload = {
                "model": model,
                "prompt": prompt,
                "images": [image_data],
                "stream": response_callback is not None,
                "options": {
                    "temperature": temperature,
                    "num_predict": max_tokens
                }
            }
            
            if self.debug_mode:
                print(f"🤖 Sending request to {model} with prompt: '{prompt[:50]}...'")
            
            response = self.session.post(
                f"{self.base_url}/api/generate",
                json=payload,
                stream=response_callback is not None,
                timeout=120  # Longer timeout for generation
            )
            
            response.raise_for_status()
            
            if response_callback:
                # Handle streaming response
                return self._handle_streaming_response(response, response_callback)
            else:
                # Handle single response
                data = response.json()
                return {
                    "success": True,
                    "response": data.get("response", ""),
                    "model": model,
                    "done": data.get("done", True)
                }
                
        except requests.exceptions.Timeout:
            error_msg = "Request timed out - ollama server may be overloaded"
            if self.debug_mode:
                print(f"⏰ {error_msg}")
            return {"success": False, "error": error_msg}
            
        except requests.exceptions.ConnectionError:
            error_msg = "Cannot connect to ollama server - is it running?"
            if self.debug_mode:
                print(f"🔌 {error_msg}")
            return {"success": False, "error": error_msg}
            
        except Exception as e:
            error_msg = f"Ollama request failed: {str(e)}"
            if self.debug_mode:
                print(f"❌ {error_msg}")
            return {"success": False, "error": error_msg}
    
    def _handle_streaming_response(self, response, callback: Callable[[str], None]) -> Dict[str, Any]:
        """Handle streaming response from ollama"""
        try:
            full_response = ""
            
            for line in response.iter_lines():
                if line:
                    try:
                        data = json.loads(line.decode('utf-8'))
                        chunk = data.get("response", "")
                        
                        if chunk:
                            full_response += chunk
                            callback(chunk)
                        
                        if data.get("done", False):
                            break
                            
                    except json.JSONDecodeError:
                        continue
            
            return {
                "success": True,
                "response": full_response,
                "done": True
            }
            
        except Exception as e:
            error_msg = f"Error processing streaming response: {str(e)}"
            if self.debug_mode:
                print(f"❌ {error_msg}")
            return {"success": False, "error": error_msg}
    
    def test_model(self, model: str, test_prompt: str = "Hello! Can you see this message?") -> bool:
        """Test if a specific model is working"""
        try:
            result = self.generate_with_image(
                model=model,
                prompt=test_prompt,
                image_data="",  # Empty image for text-only test
                temperature=0.1,
                max_tokens=50
            )
            
            success = result.get("success", False)
            if self.debug_mode:
                if success:
                    print(f"✅ Model {model} test successful")
                else:
                    print(f"❌ Model {model} test failed: {result.get('error', 'Unknown error')}")
            
            return success
            
        except Exception as e:
            if self.debug_mode:
                print(f"❌ Model {model} test error: {e}")
            return False
    
    def close(self):
        """Close the HTTP session"""
        try:
            self.session.close()
            if self.debug_mode:
                print("🔌 Ollama client session closed")
        except Exception as e:
            if self.debug_mode:
                print(f"⚠️ Error closing ollama session: {e}") 