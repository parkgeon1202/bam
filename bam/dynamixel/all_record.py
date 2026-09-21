# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import argparse
import os
import time


arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--arm-mass", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument("--motor", type=str, required=True)
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--vin", type=float, required=False, default=15.0)
arg_parser.add_argument("--speak", action="store_true")
args = arg_parser.parse_args()

KPS = {
    "mx64": [4, 8, 16, 32],
    "mx106": [4, 8, 16, 32],
    "xl320": [4, 8, 16, 32],
    "xl330": [50, 100, 200, 300, 400],
    "xl330i": [50, 100, 200, 300, 400],
    "xh540": [round(800 / 8), round(800 / 6), round(800 / 4), round(800 / 3), round(800 / 2), 800],
    # mx64v2/mx106v2: v1(Protocol 1.0) 로그를 이미 확보한 kp=[4,8,16,32]와 동등한
    # duty_cycle(제어 강도)을 내도록, kp_v2 = kp_v1 * (error_gain_v1/error_gain_v2)로 환산.
    "mx106v2": [126, 252, 504, 1009],
    "mx64v2": [121, 243, 485, 970],
    "mx28": [round(850 / 8), round(850 / 6), round(850 / 4), round(850 / 3), round(850 / 2), 850],
}

if args.motor not in KPS:
    raise ValueError(f"Unknown motor '{args.motor}'. Known motors: {list(KPS.keys())}")

kps = KPS[args.motor]
trajectories = ["sin_sin", "lift_and_drop", "up_and_down", "sin_time_square", "half_sine", "steps"]

command_base = f"uv run -m bam.dynamixel.record --mass {args.mass} --arm-mass {args.arm_mass} --length {args.length}"
command_base += (
    f" --port {args.port} --logdir {args.logdir} --motor {args.motor} --vin {args.vin}"
)


for kp in kps:
    for trajectory in trajectories:
        sentence = f"Kp {kp}, trajectory {trajectory.replace('_', ' ')}"
        print(sentence)

        if args.speak:
            from gtts import gTTS

            myobj = gTTS(text=sentence, lang="en", slow=False)
            myobj.save("/tmp/message.mp3")
            os.system("mpg321 /tmp/message.mp3")

        command = f"{command_base} --kp {kp} --trajectory {trajectory}"
        os.system(command)

        if trajectory == "sin_time_square":
            time.sleep(10)
