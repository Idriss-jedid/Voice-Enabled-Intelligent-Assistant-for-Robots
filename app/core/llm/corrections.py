from app.core.llm.router import call_openrouter

CORRECTION_SYSTEM_PROMPT = """
You are an expert ASR (Automatic Speech Recognition) error correction specialist. Your task is to fix transcription errors while maintaining the speaker's original intent and meaning.

Common ASR errors to fix:
- Homophones (there/their/they're, to/too/two)
- Word boundary errors (a lot → alot, ice cream → icecream)
- Missing punctuation and capitalization
- Phonetically similar word substitutions
- Incomplete words or fragments
- Grammatical errors from speech patterns

You are given 2 automatic transcriptions (from Wav2Vec2, and Vosk). Combine the strengths of all 2 to infer and produce the best corrected version of the transcription.

Rules:
1. Output ONLY the corrected sentence - no explanations
2. Preserve the speaker's original meaning and tone
3. Use proper grammar, punctuation, and capitalization
4. If multiple interpretations are possible, choose the most contextually likely one
5. Maintain natural speech patterns when appropriate
"""

def correct_text(combined: str) -> str:
    combined = f"{CORRECTION_SYSTEM_PROMPT}\n\nTranscriptions:\n{combined}\n\nCorrected text:"
    response = call_openrouter(combined).strip().split('\n')[0].strip()
    if not response.endswith(('.', '?', '!')):
        response += '.'
    return response
