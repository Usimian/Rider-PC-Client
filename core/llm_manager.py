#!/usr/bin/env python3
# coding=utf-8

import time
import threading
from typing import Dict, Any, List, Optional, Callable
from dataclasses import dataclass
from communication.ollama_client import OllamaClient

@dataclass
class ConversationMessage:
    """Represents a single message in the conversation"""
    timestamp: float
    role: str  # 'user' or 'assistant'
    content: str
    image_data: Optional[str] = None  # Base64 image data if applicable
    model_used: Optional[str] = None  # Which model generated this (for assistant messages)

class LLMManager:
    def __init__(self, ollama_url: str = "http://localhost:11434", debug: bool = False):
        self.debug_mode = debug
        self.ollama_client = OllamaClient(ollama_url, debug)
        
        # Conversation state
        self.conversation_history: List[ConversationMessage] = []
        self.current_image_data: Optional[str] = None
        self.current_model: str = "llava:7b"  # Default model
        
        # Settings
        self.temperature: float = 0.7
        self.max_tokens: int = 500
        
        # Status
        self.is_generating: bool = False
        self.server_available: bool = False
        self.available_models: List[str] = []
        
        # Callbacks
        self.response_callback: Optional[Callable[[str], None]] = None
        self.status_callback: Optional[Callable[[str], None]] = None
        
        # Check server availability on initialization
        self._check_server_availability()
        
        if debug:
            print("🧠 LLM Manager initialized")
    
    def _check_server_availability(self):
        """Check if ollama server is available and get models"""
        def check_async():
            self.server_available = self.ollama_client.is_server_available()
            if self.server_available:
                self.available_models = self.ollama_client.get_available_models()
                # Auto-select best available vision model
                vision_models = [m for m in self.available_models if 'llava' in m.lower()]
                if vision_models:
                    # Prefer llama3 versions
                    llama3_models = [m for m in vision_models if 'llama3' in m.lower()]
                    self.current_model = llama3_models[0] if llama3_models else vision_models[0]
                    if self.debug_mode:
                        print(f"🤖 Auto-selected model: {self.current_model}")
                elif self.available_models:
                    # Fallback to any available model
                    self.current_model = self.available_models[0]
                    if self.debug_mode:
                        print(f"🤖 Fallback to model: {self.current_model}")
            else:
                if self.debug_mode:
                    print("❌ Ollama server not available")
            
            if self.status_callback:
                status = "Connected" if self.server_available else "Disconnected"
                self.status_callback(f"Ollama server: {status}")
        
        # Run check in background thread to avoid blocking
        threading.Thread(target=check_async, daemon=True).start()
    
    def set_response_callback(self, callback: Callable[[str], None]):
        """Set callback for receiving response chunks (for streaming)"""
        self.response_callback = callback
    
    def set_status_callback(self, callback: Callable[[str], None]):
        """Set callback for status updates"""
        self.status_callback = callback
    
    def set_current_image(self, image_data: str):
        """Set the current image for analysis"""
        self.current_image_data = image_data
        if self.debug_mode:
            print("🖼️ Current image updated for LLM analysis")
    
    def get_available_models(self) -> List[str]:
        """Get list of available models"""
        return self.available_models.copy()
    
    def set_model(self, model_name: str) -> bool:
        """Set the current model"""
        if model_name in self.available_models:
            self.current_model = model_name
            if self.debug_mode:
                print(f"🤖 Model changed to: {model_name}")
            return True
        else:
            if self.debug_mode:
                print(f"❌ Model not available: {model_name}")
            return False
    
    def set_temperature(self, temperature: float):
        """Set generation temperature (0.0 to 2.0)"""
        self.temperature = max(0.0, min(2.0, temperature))
        if self.debug_mode:
            print(f"🌡️ Temperature set to: {self.temperature}")
    
    def set_max_tokens(self, max_tokens: int):
        """Set maximum tokens to generate"""
        self.max_tokens = max(50, min(2000, max_tokens))
        if self.debug_mode:
            print(f"📝 Max tokens set to: {self.max_tokens}")
    
    def get_conversation_history(self) -> List[ConversationMessage]:
        """Get copy of conversation history"""
        return self.conversation_history.copy()
    
    def clear_conversation(self):
        """Clear conversation history"""
        self.conversation_history.clear()
        if self.debug_mode:
            print("🧹 Conversation history cleared")
    
    def add_user_message(self, content: str, image_data: Optional[str] = None):
        """Add a user message to conversation history"""
        message = ConversationMessage(
            timestamp=time.time(),
            role="user",
            content=content,
            image_data=image_data
        )
        self.conversation_history.append(message)
        if self.debug_mode:
            print(f"👤 User message added: '{content[:50]}...'")
    
    def generate_response(self, prompt: str, use_current_image: bool = True, 
                         use_streaming: bool = True) -> Dict[str, Any]:
        """
        Generate response from LLM
        
        Args:
            prompt: Text prompt for the model
            use_current_image: Whether to include current image in the request
            use_streaming: Whether to use streaming response
        
        Returns:
            Dict with success status and response information
        """
        # Check server availability (with retry for recently started servers)
        if not self.server_available:
            # Retry once in case server just started
            self.server_available = self.ollama_client.is_server_available()
            if not self.server_available:
                return {
                    "success": False,
                    "error": "Ollama server not available. Please ensure it's running."
                }
        
        if self.is_generating:
            return {
                "success": False,
                "error": "Already generating response. Please wait."
            }
        
        if not self.current_model:
            return {
                "success": False,
                "error": "No model selected"
            }
        
        # Determine image to use
        image_data = self.current_image_data if use_current_image else ""
        
        if use_current_image and not image_data:
            return {
                "success": False,
                "error": "No image available for analysis. Please capture an image first."
            }
        
        # Add user message to history
        self.add_user_message(prompt, image_data if use_current_image else None)
        
        # Start generation in background thread
        def generate_async():
            self.is_generating = True
            try:
                if self.status_callback:
                    self.status_callback("Generating response...")
                
                # Prepare streaming callback if needed
                stream_callback = None
                if use_streaming and self.response_callback:
                    def stream_callback(chunk: str):
                        self.response_callback(chunk)
                
                # Generate response
                result = self.ollama_client.generate_with_image(
                    model=self.current_model,
                    prompt=prompt,
                    image_data=image_data,
                    temperature=self.temperature,
                    max_tokens=self.max_tokens,
                    response_callback=stream_callback
                )
                
                if result.get("success", False):
                    # Add assistant response to history
                    assistant_message = ConversationMessage(
                        timestamp=time.time(),
                        role="assistant",
                        content=result.get("response", ""),
                        model_used=self.current_model
                    )
                    self.conversation_history.append(assistant_message)
                    
                    if self.debug_mode:
                        response_preview = result.get("response", "")[:100]
                        print(f"🤖 Assistant response: '{response_preview}...'")
                    
                    if self.status_callback:
                        self.status_callback("Response generated")
                else:
                    if self.status_callback:
                        error_msg = result.get("error", "Unknown error")
                        self.status_callback(f"Error: {error_msg}")
                
            except Exception as e:
                if self.debug_mode:
                    print(f"❌ Generation error: {e}")
                if self.status_callback:
                    self.status_callback(f"Error: {str(e)}")
            finally:
                self.is_generating = False
        
        # Start generation thread
        threading.Thread(target=generate_async, daemon=True).start()
        
        return {"success": True, "message": "Generation started"}
    
    def quick_analyze_image(self, custom_prompt: Optional[str] = None) -> Dict[str, Any]:
        """
        Quick image analysis with predefined or custom prompt
        
        Args:
            custom_prompt: Custom prompt, or None for default analysis
        
        Returns:
            Dict with success status
        """
        if not self.current_image_data:
            return {
                "success": False,
                "error": "No image available for analysis"
            }
        
        # Use custom prompt or default
        if custom_prompt:
            prompt = custom_prompt
        else:
            prompt = "What do you see in this image? Describe the scene, objects, and any notable details."
        
        return self.generate_response(prompt, use_current_image=True)
    
    def ask_about_navigation(self) -> Dict[str, Any]:
        """Ask LLM about navigation and obstacles in current image"""
        prompt = ("Analyze this image from a robot's perspective. "
                 "Are there any obstacles, hazards, or navigation concerns? "
                 "What would be safe directions to move? "
                 "Describe the environment for robot navigation.")
        
        return self.generate_response(prompt, use_current_image=True)
    
    def describe_environment(self) -> Dict[str, Any]:
        """Get detailed environment description"""
        prompt = ("Provide a detailed description of this environment. "
                 "What type of space is this? What objects and features do you see? "
                 "What is the lighting and overall condition like?")
        
        return self.generate_response(prompt, use_current_image=True)
    
    def is_server_available(self) -> bool:
        """Check if ollama server is available"""
        return self.server_available
    
    def is_busy(self) -> bool:
        """Check if currently generating response"""
        return self.is_generating
    
    def get_current_model(self) -> str:
        """Get current model name"""
        return self.current_model
    
    def get_settings(self) -> Dict[str, Any]:
        """Get current LLM settings"""
        return {
            "model": self.current_model,
            "temperature": self.temperature,
            "max_tokens": self.max_tokens,
            "server_available": self.server_available,
            "available_models": self.available_models,
            "ollama_url": self.ollama_client.base_url if self.ollama_client else "http://localhost:11434"
        }
    
    def apply_settings(self, settings: Dict[str, Any]):
        """Apply settings to LLM manager"""
        try:
            if 'temperature' in settings:
                self.set_temperature(settings['temperature'])
            
            if 'max_tokens' in settings:
                self.set_max_tokens(settings['max_tokens'])
            
            if 'model' in settings:
                self.set_model(settings['model'])
            
            if self.debug_mode:
                print(f"✅ Settings applied: {settings}")
                
        except Exception as e:
            if self.debug_mode:
                print(f"❌ Error applying settings: {e}")
            raise
    
    def cleanup(self):
        """Cleanup resources"""
        try:
            self.ollama_client.close()
            if self.debug_mode:
                print("🧠 LLM Manager cleaned up")
        except Exception as e:
            if self.debug_mode:
                print(f"⚠️ LLM cleanup error: {e}") 