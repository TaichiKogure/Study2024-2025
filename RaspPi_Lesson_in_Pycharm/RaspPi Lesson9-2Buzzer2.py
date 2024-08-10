from gpiozero import TonalBuzzer
from gpiozero import LED
from time import sleep
from signal import pause
import threading

makerobo_buzzer = Buzzer(4)

Tone_CL = [220, 220, 220, 220, 220, 220, 220, 248]
Tone_CM = [220, 262, 294, 330, 350, 393, 441, 495]
Tone_CH = [220, 525, 589, 661, 700, 786, 800, 880]

makerobo_song1 = [
    Tone_CH[3],Tone_CM[5],Tone_CM[6],Tone_CM[3], Tone_CM[2], Tone_CM[3], Tone_CM[5], Tone_CM[6],
    Tone_CH[1], Tone_CM[6],Tone_CM[5], Tone_CM[1], Tone_CM[3], Tone_CM[2], Tone_CM[2], Tone_CM[3],
    Tone_CM[5], Tone_CM[2], Tone_CM[3], Tone_CM[3],Tone_CL[6], Tone_CL[6], Tone_CL[6], Tone_CM[1],
    Tone_CM[2], Tone_CM[3], Tone_CM[2], Tone_CL[7], Tone_CL[6], Tone_CM[1], Tone_CL[5]
]

makerobo_beat_1 = [
    1,1,3,1,1,3,1,1,
    1,1,1,1,1,1,3,1,
    1,3,1,1,1,1,1,1,
    1,2,1,1,1,1,1,1,
    1,1,3
]
makerobo_song2 = [
    Tone_CH[1],Tone_CM[1],Tone_CM[1],Tone_CL[5], Tone_CM[3], Tone_CM[3], Tone_CM[3], Tone_CM[1],
    Tone_CM[1], Tone_CM[3],Tone_CM[5], Tone_CM[5], Tone_CM[4], Tone_CM[3], Tone_CM[2], Tone_CM[2],
    Tone_CM[3], Tone_CM[4], Tone_CM[4], Tone_CM[3],Tone_CM[2], Tone_CL[3], Tone_CL[1], Tone_CM[1],
    Tone_CM[3], Tone_CM[2], Tone_CL[5], Tone_CL[7], Tone_CM[2], Tone_CM[1]
]

makerobo_beat_2 = [
    1,1,2,2,1,1,2,2,
    1,1,2,2,1,1,3,1,
    1,2,2,1,1,2,2,1,
    1,2,2,1,1,3
]

def exit_script():
    makerobo_destroy()
    exit(0)

def makerobo_setup():
    global bz
    bz = TonalBuzzer(makerobo_buzzer)
    bz.stop()


def loop():
    while True:
        for i in range(1, len(makerobo_song1)):
            bz.play(Tone(makerobo_song1[i]))
            sleep(makerobo_beat_1[i] * 0.5)
        sleep(1)

        for i in range(1, len(makerobo_song2)):
            bz.play(Tone(makerobo_song2[i]))
            sleep(makerobo_beat_2[i] * 0.5)

def makerobo_destroy():
    bz.stop()

exit_timer = threading.Timer(60, exit_script)
exit_timer.start()

if __name__ == '__main__':
    makerobo_setup()
    try:
        loop()
    except KeyboardInterrupt:
        pass
    finally:
        makerobo_destroy()