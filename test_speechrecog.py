#test microphone
#vosk
import sys
import sounddevice as sd
import queue
import json
from vosk import Model, KaldiRecognizer

q = queue.Queue()

def callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

# Load model folder you unzipped
model = Model("vosk-model-small-en-us-0.15")
recognizer = KaldiRecognizer(model, 16000)  # 16k sample rate

with sd.RawInputStream(samplerate=16000, blocksize=8000,
                       dtype='int16', channels=1, callback=callback):
    print(" Speak now (Ctrl+C to stop)")
    while True:
        data = q.get()
        if recognizer.AcceptWaveform(data):
            print('full: ',json.loads(recognizer.Result())["text"])
        else:
            print(json.loads(recognizer.PartialResult())["partial"])
