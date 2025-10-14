"""
Consult Gemini on ORION Phase 1 Implementation Strategy
"""
import sys
import os

# Add multi-agent to path
sys.path.insert(0, r'D:\projexts\multi-agent')

from src.agents import GeminiAgent
from src.config import Config

def main():
    # Check API key
    if not Config.GOOGLE_API_KEY:
        print("ERROR: GOOGLE_API_KEY not found in environment")
        print("Please set it in D:\\projexts\\multi-agent\\.env")
        return

    print("Initializing Gemini agent...")
    gemini = GeminiAgent(name="Technical Advisor", role="SLAM & Navigation Expert")

    # Read consultation file
    consultation_file = r"D:\projexts\STN\consultation_phase1_strategy.md"
    print(f"\nReading consultation request from: {consultation_file}")

    with open(consultation_file, 'r', encoding='utf-8') as f:
        consultation_request = f.read()

    prompt = f"""You are an expert in SLAM, robotics, and autonomous navigation systems.

I need comprehensive, practical guidance for implementing a GPS-denied navigation system.

{consultation_request}

Please provide:
1. Specific technology recommendations (with justifications)
2. Week-by-week implementation plan for 16 weeks
3. Minimal Viable System definition
4. Hardware recommendations (including budget alternatives)
5. Risk assessment and mitigation strategies
6. your job is to make us create a system that will make bllions. 

Be brutally honest about what's achievable. Focus on practical, proven approaches over cutting-edge research."""

    print("\n" + "="*80)
    print("CONSULTING GEMINI...")
    print("="*80 + "\n")

    response = gemini.process(prompt)

    # Save response
    output_file = r"D:\projexts\STN\gemini_phase1_guidance.md"
    print(f"\n\nSaving response to: {output_file}")

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("# ORION Phase 1 Implementation Guidance from Gemini\n\n")
        f.write(f"**Generated:** {os.popen('date /t').read().strip()}\n\n")
        f.write("---\n\n")
        f.write(response)

    print("\n" + "="*80)
    print("GEMINI RESPONSE:")
    print("="*80 + "\n")
    print(response)
    print("\n" + "="*80)
    print(f"Response saved to: {output_file}")
    print("="*80 + "\n")

if __name__ == "__main__":
    main()
