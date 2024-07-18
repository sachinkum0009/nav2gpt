#!/usr/bin/env python3

import whisper

model = whisper.load_model("base")
result = model.transcribe("test.mp3")
print(result["text"])