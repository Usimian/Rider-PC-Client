# LLM Integration Setup Guide

This guide will help you set up Ollama with vision models for the Rider Robot PC Client.

## Prerequisites

- Python 3.8 or higher
- Working Rider Robot PC Client
- Internet connection for downloading models

## Step 1: Install Ollama

### Linux/macOS
```bash
curl -fsSL https://ollama.ai/install.sh | sh
```

### Windows
Download and install from: https://ollama.ai/download

## Step 2: Start Ollama Server

```bash
ollama serve
```

The server will start on `http://localhost:11434` by default.

**Note**: Keep this terminal open while using the LLM features.

## Step 3: Install Vision Models

Install at least one vision-language model:

### Recommended: LLaVA 7B (Smaller, faster)
```bash
ollama pull llava:7b
```

### Alternative: LLaVA 13B (Larger, more capable)
```bash
ollama pull llava:13b
```

### Alternative: LLaVA 34B (Largest, best quality)
```bash
ollama pull llava:34b
```

## Step 4: Install Python Dependencies

```bash
pip install -r requirements.txt
```

## Step 5: Test the Integration

Run the test script to verify everything works:

```bash
python test_llm_integration.py
```

Expected output:
```
🧪 LLM Integration Test Suite
==================================================
=== Testing Basic Ollama Connection ===
Ollama URL: http://localhost:11434
LLM Enabled: True
🤖 Ollama client initialized with base URL: http://localhost:11434
🧠 LLM Manager initialized
🔍 Ollama server availability check: ✅ Available
🤖 Available models: ['llava:7b']
```

## Step 6: Run the Main Application

```bash
python pc_client_standalone.py
```

or with debug mode:

```bash
python pc_client_standalone.py -d
```

## Configuration

The LLM settings are stored in `rider_config.ini`:

```ini
[llm]
ollama_url = http://localhost:11434
default_model = llava:7b
temperature = 0.7
max_tokens = 500
enabled = true
```

### Configuration Options

- **ollama_url**: URL of the ollama server
- **default_model**: Default vision model to use
- **temperature**: Response creativity (0.0 = deterministic, 2.0 = very creative)
- **max_tokens**: Maximum response length
- **enabled**: Enable/disable LLM features

## Using LLM Features

### Basic Image Analysis
1. Capture an image using "🔄 Refresh" button
2. Click "🤖 Ask AI" button (will be added in next phase)
3. The AI will analyze the image and provide description

### Custom Prompts
- Enter custom questions about the image
- Ask about navigation, obstacles, or environment details
- Conversation history is maintained

### Pre-defined Questions
- "What do you see?" - General image description
- "Navigation analysis" - Robot navigation perspective  
- "Environment details" - Detailed environmental analysis

## Troubleshooting

### "Ollama server not available"
- Ensure ollama is running: `ollama serve`
- Check if port 11434 is available
- Verify firewall settings

### "No models available"
- Install a vision model: `ollama pull llava:7b`
- Check model installation: `ollama list`

### "Model not responding"
- Large models need more RAM (8GB+ recommended for llava:13b)
- Try smaller model: `ollama pull llava:7b`
- Check ollama server logs

### Performance Issues
- Use smaller models (llava:7b vs llava:34b)
- Reduce max_tokens in configuration
- Ensure sufficient RAM available

## Model Comparison

| Model | Size | RAM Required | Speed | Quality |
|-------|------|--------------|-------|---------|
| llava:7b | ~4.5GB | 8GB | Fast | Good |
| llava:13b | ~7.4GB | 16GB | Medium | Better |
| llava:34b | ~19GB | 32GB | Slow | Best |

## Next Steps

Once everything is working:
1. The next implementation phase will add the UI components
2. Integration with the image capture system
3. Chat interface for conversation with the AI

## Support

If you encounter issues:
1. Run the test script to diagnose problems
2. Check ollama server logs
3. Verify model installation with `ollama list`
4. Ensure sufficient system resources 