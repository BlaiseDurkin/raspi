#test tts
import os
import subprocess
#import espeak

#espeak.init()
#speaker = espeak.Espeak()
#speaker.say("hello ")
#speaker.rate(300)
#speaker.say("oh my god i can talk so fast oh my")
os.environ['ALSA_LOG_LEVEL'] = '0'

def speak(text):
    vol = 70
    print(vol)
    cmd = f'espeak -a "{vol}" "{text}" -- stdout | aplay -D hw:1,0 2>/dev/null'
    subprocess.run(cmd, shell=True)
    
speak("")
#message = "sorry to wake you up, please go back to sleep"
#message = "this is not an emergency, please go back to sleep"
#message = "I'm sorry Dave, I'm afraid I can't do that"
#message = "Acidity refers to the concentration of hydrogen ions (H⁺) in a solution or medium like soil. In chemistry, a substance is acidic if it has a pH below 7, indicating a higher concentration of H⁺ ions compared to hydroxide ions (OH⁻).The pH scale measures this: lower pH (e.g., 4) means more acidic, higher pH (e.g., 9) means more alkaline (basic), and 7 is neutral. In soil, acidity affects nutrient availability, microbial activity, and plant growth."
message = "I am the secretary of health"
volume = 200
speak(message)

