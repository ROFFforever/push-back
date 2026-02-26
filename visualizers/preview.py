import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import os
import shutil
import filecmp
from datetime import datetime

# --- SETTINGS ---
LOG_FILE = "logs.txt"
ARCHIVE_DIR = "oldLogs"
FIELD_IMAGE = "field.png"
TIME_STEP = 0.05
ROBOT_SIZE_IN = 12
POINT_SIZE = 25

def copy_to_archive():
    if not os.path.exists(LOG_FILE): return
    if not os.path.exists(ARCHIVE_DIR): os.makedirs(ARCHIVE_DIR)
    is_duplicate = False
    for existing_file in os.listdir(ARCHIVE_DIR):
        existing_path = os.path.join(ARCHIVE_DIR, existing_file)
        if os.path.isfile(existing_path) and filecmp.cmp(LOG_FILE, existing_path, shallow=False):
            is_duplicate = True
            break
    if not is_duplicate:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(LOG_FILE, os.path.join(ARCHIVE_DIR, f"oldLog_{timestamp}.txt"))

def load_and_calculate():
    x, y, h = [], [], []
    if not os.path.exists(LOG_FILE): return None
    try:
        try:
            with open(LOG_FILE, 'r', encoding='utf-16') as f: lines = f.readlines()
        except:
            with open(LOG_FILE, 'r', encoding='utf-8') as f: lines = f.readlines()
        
        for line in lines:
            line = line.strip().replace('\ufeff', '').replace('X:', '').replace('Y:', '').replace('Heading:', '')
            if not line: continue
            parts = [float(p) for p in line.split(',')]
            if len(parts) >= 3:
                x.append(parts[0]); y.append(parts[1]); h.append(parts[2])
    except Exception as e:
        print(f"File Load Error: {e}")
        return None
    
    if len(x) < 2: return None
    x_arr, y_arr, h_arr = np.array(x), np.array(y), np.array(h)
    v = np.sqrt(np.gradient(x_arr, TIME_STEP)**2 + np.gradient(y_arr, TIME_STEP)**2)
    accel = np.gradient(v, TIME_STEP)
    copy_to_archive()
    return x_arr, y_arr, h_arr, v, accel

data = load_and_calculate()
if data:
    x, y, h, v, accel = data
    fig, ax = plt.subplots(figsize=(10, 9))
    plt.subplots_adjust(left=0.07, right=0.88, top=0.92, bottom=0.15)
    
    try:
        img = mpimg.imread(FIELD_IMAGE)
        # FIXED: Removed extra quotes in extent
        ax.imshow(img, extent=[-72, 72, -72, 72], alpha=0.6, zorder=1)
    except:
        ax.set_facecolor('#111111')
    
    ax.plot(x, y, color='cyan', linewidth=1, alpha=0.3, zorder=2)
    points = ax.scatter(x, y, c=v, cmap='plasma', s=POINT_SIZE, alpha=0.7, zorder=3, picker=True)
    
    # FIXED: Fixed tuple/string formatting in Rectangle
    ghost_robot = Rectangle((-ROBOT_SIZE_IN/2, -ROBOT_SIZE_IN/2), ROBOT_SIZE_IN, ROBOT_SIZE_IN, alpha=0.5, color='orange', zorder=5)
    ax.add_patch(ghost_robot)
    ghost_robot.set_visible(False)
    
    timer_text = ax.text(0.95, 0.95, 'Time: 0.00s', transform=ax.transAxes, fontsize=12, fontweight='bold', color='white', bbox=dict(facecolor='black', alpha=0.5), ha='right')

    class PlaybackController:
        def __init__(self):
            self.ani = None
            self.is_paused = True
            self.started = False
            self.init_animation()
            self.update(0)

        def init_animation(self):
            self.ani = FuncAnimation(fig, self.update, frames=len(x), interval=TIME_STEP*1000, repeat=False, blit=True)
            self.ani.event_source.stop()

        def update(self, frame):
            idx = int(frame)
            ghost_robot.set_visible(True)
            ghost_robot.set_xy([x[idx] - ROBOT_SIZE_IN/2, y[idx] - ROBOT_SIZE_IN/2])
            transform = Affine2D().rotate_deg_around(x[idx], y[idx], 90 - h[idx]) + ax.transData
            ghost_robot.set_transform(transform)
            
            # Use idx * TIME_STEP to calculate current timestamp
            current_time = idx * TIME_STEP
            timer_text.set_text(f"Time: {current_time:.2f}s")
            
            if idx >= len(x) - 1:
                self.ani.event_source.stop()
                self.is_paused = True
                self.started = False
                btn_play.label.set_text('▶ Play')
            return ghost_robot, timer_text

        def toggle(self, event):
            if self.is_paused:
                self.started = True
                self.ani.event_source.start()
                btn_play.label.set_text('|| Pause')
            else:
                self.ani.event_source.stop()
                btn_play.label.set_text('▶ Resume')
            self.is_paused = not self.is_paused

        def stop(self, event):
            self.ani.event_source.stop()
            self.is_paused = True
            self.started = False
            btn_play.label.set_text('▶ Play')
            timer_text.set_text('Time: 0.00s')
            self.update(0)
            fig.canvas.draw_idle()

    controller = PlaybackController()
    ax_play = plt.axes([0.38, 0.02, 0.12, 0.05])
    ax_stop = plt.axes([0.52, 0.02, 0.1, 0.05])
    btn_play = Button(ax_play, '▶ Play', color='#222222', hovercolor='#333333')
    btn_stop = Button(ax_stop, '■ Stop', color='#222222', hovercolor='#333333')
    btn_play.label.set_color('cyan')
    btn_stop.label.set_color('red')
    btn_play.on_clicked(controller.toggle)
    btn_stop.on_clicked(controller.stop)

    annot = ax.annotate("", xy=(0,0), xytext=(15,15), textcoords="offset points", 
                        bbox=dict(boxstyle="round", fc="black", ec="cyan", alpha=0.8), 
                        arrowprops=dict(arrowstyle="->", color='white'), color="white", fontsize=9)
    annot.set_visible(False)

    def on_pick(event):
        idx = int(event.ind[0])
        if not controller.is_paused:
            controller.toggle(None)
        
        controller.update(idx)
        
        # Calculate time for the annotation
        point_time = idx * TIME_STEP
        
        annot.xy = [x[idx], y[idx]]
        annot.set_text(f"Time: {point_time:.2f}s\nX: {x[idx]:.2f}\nY: {y[idx]:.2f}\nθ: {h[idx]:.1f}°")
        annot.set_visible(True)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("pick_event", on_pick)
    ax.set_xlim(-72, 72); ax.set_ylim(-72, 72)
    ax.set_title("LemLib Playback Tool 2026", pad=10)
    plt.show()
else:
    print("Could not load data. Ensure logs.txt contains valid VRC coordinates.")
