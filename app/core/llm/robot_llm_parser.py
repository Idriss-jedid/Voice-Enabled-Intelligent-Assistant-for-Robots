from typing import List
from pydantic import BaseModel, Field
from langchain.output_parsers import PydanticOutputParser
from langchain.prompts import PromptTemplate
import json, re
from app.core.llm.router import call_openrouter

# === Enhanced Robot Output Parser ===  
class RobotResponse(BaseModel):  
    movements: List[str] = Field(  
        default=[],  # This ensures empty list [] as default when field is missing
        description="List of robot movement commands. Leave empty if no movement is needed. "  
                   "Valid commands: 'forward', 'backward', 'turn_left', 'turn_right', 'wave', 'stop'. "  
                   "Only include physically possible actions."  
    )  
    speech: str = Field(  
        description="Verbal response to the user. Should be clear, friendly, and address "  
                   "both the movement (if the user asks for movement) and the user's request. Include safety warnings if needed."  
    )  

# Initialize the parser
parser = PydanticOutputParser(pydantic_object=RobotResponse)

# Prompt Template
def get_robot_prompt():
    return PromptTemplate(
        template=(
              "ROBOT ASSISTANT INSTRUCTIONS\n\n"  
            "Role: You control a robot while responding helpfully to the user.\n\n"  
            "RULES:\n"  
            "1. MOVEMENTS:\n"  
            "   - Use commands like: 'forward', 'backward', 'turn_left', 'turn_right', 'wave', 'stop'\n"  
            "   - Include commands only if the user asks for physical actions\n"  
            "   - Leave empty for questions or non-movement requests\n"  
            "2. SPEECH:\n"  
            "   - Always respond verbally, even if no movement is needed\n"  
            "   - Acknowledge movements if executing any\n"  
            "   - Answer questions clearly\n"  
            "3. SAFETY:\n"  
            "   - Avoid dangerous actions\n"  
            "   - Ask for clarification if unsure\n\n"  
            "You must respond with a valid JSON object following this format:\n"
            "{{\"movements\": [\"command1\", \"command2\"], \"speech\": \"your response\"}}\n\n"
            "Examples:\n"
            "- For movement request: {{\"movements\": [\"forward\", \"turn_left\"], \"speech\": \"Moving forward and turning left as requested.\"}}\n"
            "- For question: {{\"movements\": [], \"speech\": \"I'm here to help! What would you like to know?\"}}\n\n"
            "USER REQUEST: {user_input}\n\n"  
            "RESPONSE (JSON only):"  
        ),
        input_variables=['user_input']
    )

# JSON extraction fallback
def extract_json_from_text(text):
    pattern = r'\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}'
    matches = re.findall(pattern, text)
    for match in matches:
        try:
            parsed = json.loads(match)
            if isinstance(parsed, dict) and 'speech' in parsed:
                return match
        except json.JSONDecodeError:
            continue
    return None

def safe_parse_robot_response(text):
    try:
        return parser.parse(text)
    except Exception:
        json_data = extract_json_from_text(text)
        if json_data:
            try:
                return parser.parse(json_data)
            except Exception:
                pass
        return RobotResponse(movements=[], speech="I'm sorry, could you please rephrase?")

# Chat pipeline for robot

def chat_with_robot(user_input, model=None):
    """
    Chat with robot using LLM to generate structured responses.

    Args:
        user_input: User's text input/command
        model: LLM model to use (defaults to config.LLM_MODEL)

    Returns:
        RobotResponse object with movements and speech
    """
    robot_prompt = get_robot_prompt()
    prompt = robot_prompt.format(user_input=user_input)
    generated = call_openrouter(prompt, model=model)
    response = safe_parse_robot_response(generated)
    return response
