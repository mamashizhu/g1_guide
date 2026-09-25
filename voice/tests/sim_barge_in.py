r"""Barge-in closed loop without a microphone: synthetic speech is fed to the real VAD/ASR in real time.

    .venv\Scripts\python.exe tests\sim_barge_in.py

Scenarios: A echo only (must not interrupt), B echo + visitor (must interrupt),
C headphones + visitor (must interrupt).  Takes about 45 s.
"""
import sys, time, numpy as np
from pathlib import Path
sys.stdout.reconfigure(encoding="utf-8")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import asr
from config import Settings, SAMPLE_RATE
from engine import GuideEngine
from tts import OfflineEngine
tts = OfflineEngine()
def speech(text, gain=1.0):
    x = tts.synthesize(text); n = int(len(x)*SAMPLE_RATE/tts.sample_rate)
    return (np.interp(np.linspace(0,len(x)-1,n), np.arange(len(x)), x)*gain).astype(np.float32)
ROBOT = "我们到了，这就是地球仪。地球仪的轴是倾斜的，和竖直方向大约差二十三点五度，正是这个倾斜带来了四季变化。"
USER = "等一下，带我去看植物细胞。"
robot_audio = speech(ROBOT, 0.5)             # echo is quieter than a visitor right at the mic
user_audio = speech(USER, 1.0)
rng = np.random.default_rng(0)

class FakeSpeaker:
    def __init__(s): s.stopped_at=None; s.until=0
    @property
    def busy(s): return s.stopped_at is None and time.monotonic() < s.until
    def say(s, x): pass
    def wait(s): pass
    def stop(s):
        if s.stopped_at is None and time.monotonic() < s.until: s.stopped_at = time.monotonic()
    def recent_text(s, window=4.0): return ROBOT

asr.Listener.start = lambda self: None
asr.Listener.stop = lambda self: None

def scenario(name, user_part, echo_part):
    settings = Settings("deepseek","m","Realtek","offline","v","localhost",9090)
    log=[]
    eng = GuideEngine(settings, use_robot=False, speak=False, listen=True, barge_in=True, offline=True,
                      ui=lambda k,t="": log.append((k,t)))
    eng.speaker = FakeSpeaker()
    eng.start()
    eng.speaker.until = time.monotonic() + len(robot_audio)/SAMPLE_RATE + 0.5
    t0 = time.monotonic()
    for i in range(0, len(user_part), 1600):   # real-time feed, 100 ms blocks
        block = user_part[i:i+1600].copy()
        if eng.speaker.stopped_at is None:      # echo only while the robot is still talking
            block += echo_part[i:i+1600]
        eng.listener._on_audio(block[:, None], 1600, None, None)
        time.sleep(0.1)
    time.sleep(1.5); eng.wait_idle()
    stop = eng.speaker.stopped_at
    users = [t for k,t in log if k=="user"]
    notes = [t for k,t in log if k=="log" and ("打断" in t or "回声" in t)]
    print(f"--- {name}\n  stopped: {'no' if stop is None else f'{stop-t0:.2f}s after start'}\n  user turns: {users}\n  notes: {notes}")
    eng.close()

pad = lambda s: np.zeros(int(s*SAMPLE_RATE), np.float32)
noise = lambda x: x + rng.normal(0, 0.003, len(x)).astype(np.float32)
total = int(13*SAMPLE_RATE)
echo = np.zeros(total, np.float32); echo[int(0.3*SAMPLE_RATE):int(0.3*SAMPLE_RATE)+len(robot_audio)] = robot_audio
visitor = np.zeros(total, np.float32); start = int(2.8*SAMPLE_RATE); visitor[start:start+len(user_audio)] = user_audio
silence = np.zeros(total, np.float32)
print(f"robot speech {len(robot_audio)/SAMPLE_RATE:.1f}s, visitor starts at 2.8s")
scenario("A 外放、只有回声（不应打断）", noise(silence), echo)
scenario("B 外放、回声中游客插话（应打断）", noise(visitor), echo)
scenario("C 戴耳机、游客插话（应打断）", noise(visitor), silence)
