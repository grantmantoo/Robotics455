import argparse
import json
import queue
import sys

import requests
import sounddevice as sd
from vosk import KaldiRecognizer, Model


def normalize_destination(text):
    text = text.lower().strip()

    if any(word in text for word in ["bathroom", "restroom", "washroom"]):
        return "bathroom"

    if "robot lab" in text or "robotics lab" in text or text == "lab" or " lab" in text:
        return "robot lab"

    return None


def is_dialog_command(text):
    text = text.lower().strip()
    if normalize_destination(text):
        return True

    command_phrases = [
        "hello",
        "hi",
        "howdy",
        "hi there",
        "hey robot",
        "yes",
        "yeah",
        "yep",
        "sure",
        "of course",
        "no",
        "nope",
        "nah",
        "no way",
        "dance",
        "boogie",
        "do a dance",
        "dance for me",
        "arm",
        "wave",
        "raise your arm",
        "raise arm",
        "wave at me",
        "thanks",
        "thank you",
        "bye",
        "goodbye",
        "stop",
        "cancel",
        "reset",
        "quit",
    ]
    return any(phrase in text for phrase in command_phrases)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", required=True, help="Robot Flask base URL, e.g. http://192.168.43.95:5000")
    parser.add_argument("--model", required=True, help="Path to Vosk model folder")
    parser.add_argument("--samplerate", type=int, default=16000)
    parser.add_argument("--device", default=None, help="Optional sounddevice input device id/name")
    parser.add_argument("--list-devices", action="store_true", help="List audio devices and exit")
    args = parser.parse_args()

    if args.list_devices:
        print(sd.query_devices())
        return

    model = Model(args.model)
    recognizer = KaldiRecognizer(model, args.samplerate)
    recognizer.SetWords(False)

    audio_q = queue.Queue()

    def callback(indata, frames, time_info, status):
        if status:
            print(status, file=sys.stderr)
        audio_q.put(bytes(indata))

    print("Listening locally with Vosk.")
    print("Say a destination or key dialog command like hello, dance, yes, no, wave, stop.")
    endpoint = args.server.rstrip("/") + "/api/dialog_input"

    print("Posting recognized commands to:", endpoint)
    print("Press Ctrl+C to stop.")

    try:
        with sd.RawInputStream(
            samplerate=args.samplerate,
            blocksize=8000,
            device=args.device,
            dtype="int16",
            channels=1,
            callback=callback,
        ):
            while True:
                data = audio_q.get()

                if recognizer.AcceptWaveform(data):
                    result = json.loads(recognizer.Result())
                    text = result.get("text", "").strip()
                    if not text:
                        continue

                    print("Heard:", text)

                    if not is_dialog_command(text):
                        print("No key dialog command found.")
                        continue

                    destination_text = normalize_destination(text)
                    if destination_text:
                        print("Matched destination:", destination_text)
                    else:
                        print("Matched dialog command.")

                    try:
                        res = requests.post(
                            endpoint,
                            json={
                                "text": text,
                                "source": "vosk",
                                "destination": destination_text,
                            },
                            timeout=3,
                        )
                        print("POST", res.status_code, res.text[:300])
                    except requests.RequestException as e:
                        print("Could not reach robot server:", e)
    except sd.PortAudioError as e:
        print("Could not open microphone:", e)
        print()
        print("Run this to list input devices:")
        print("  python3 local_vosk_listener.py --list-devices --server", args.server, "--model", args.model)
        print()
        print("Then rerun with an input device number, for example:")
        print("  python3 local_vosk_listener.py --device 1 --server", args.server, "--model", args.model)


if __name__ == "__main__":
    main()
