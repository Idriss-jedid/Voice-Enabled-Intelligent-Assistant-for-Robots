import requests
from helpers.config import config

def call_openrouter(prompt: str, model: str = None) -> str:
    """
    Call OpenRouter API with the given prompt.

    Args:
        prompt: The text prompt to send to the LLM
        model: The model to use (defaults to config.LLM_MODEL)

    Returns:
        The response text from the LLM
    """
    if model is None:
        model = config.LLM_MODEL

    headers = {
        "Authorization": f"Bearer {config.OPENROUTER_API_KEY}",
        "Content-Type": "application/json"
    }

    url = "https://openrouter.ai/api/v1/chat/completions"
    payload = {
        "model": model,
        "messages": [{"role": "user", "content": prompt}]
    }

    response = requests.post(url, headers=headers, json=payload)
    response.raise_for_status()
    return response.json()["choices"][0]["message"]["content"]
