import sounddevice as sd
import scipy.io.wavfile as wav
import whisper

from langchain_community.llms import Ollama

import json

llm = Ollama(model="llama3")
def record_audio(filename, duration, fs=44100):
    print("Recording...")
    audio = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='int16')
    sd.wait()  # Wait until recording is finished
    print("Recording finished.")
    wav.write(filename, fs, audio)  # Save as WAV file


# Function to execute commands
def execute_command(command):
    service = command["service"]
    args = command["args"]
    
    # Execute based on service
    if service == "/goToPose":
        x = args["x"]
        y = args["y"]
        theta = args["theta"]
        print(f"Executing goToPose with x={x}, y={y}, theta={theta}")
        # Implement your logic to execute goToPose command
    elif service == "/wait":
        print("Executing wait")
        # Implement your logic to execute wait command
    else:
        print(f"Unknown service: {service}")


def main():
    input("Press Enter to start recording...")
    filename = "recorded_audio.wav"
    duration = 10  # seconds
    record_audio(filename, duration)
    
    print("Transcribing...")
    model = whisper.load_model("base")
    result = model.transcribe(filename)
    print("Transcription:", result["text"])
    prompt = """
    Use this JSON schema to achieve the user's goals:\n\
            {
    "$schema": "http://json-schema.org/draft-04/schema#",
    "type": "object",
    "properties": {
        "service": {
            "type": "string",
            "default": "/goToPose"
        },
        "args": {
            "type": "object",
            "properties": {
                "x": {
                    "type": "number",
                    "min": 0,
                    "max": 10
                },
                "y": {
                    "type": "number",
                    "min": 0,
                    "max": 10
                },
                "theta": {
                    "type": "number",
                    "min": -3.14,
                    "max": 3.14
                }
            },
            "required": [
                "x",
                "y",
                "theta"
            ]
        }
    },
    "required": [
        "service",
        "args"
    ]
}
\n\
            Respond as a list of JSON objects.\
            Do not include explanations or conversation in the response
            
            
"role": "user",
                "content": f"\
                remember these the coordinates of the kitchen is x: 5, y: 10, theta: 180

    the coordinates of the bedroom is x: 20, y:30, theta: 0
                    
                    
    """

    prompt += result["text"]

    prompt += """
    

                        
                        Respond only with the output in the exact format specified in the system prompt, with no explanation or conversation.\
                    ",
    """

    # print(prompt)

    result = llm.invoke(prompt)
    print(result)

    commands = json.loads(result)

    # Iterate through each command and execute
    for command in commands:
        execute_command(command)


    # Go to Pose for kitchen  in theta\n\

if __name__ == "__main__":
    main()
