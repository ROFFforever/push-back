import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D
import os
import shutil
import filecmp
from datetime import datetime

# --- SETTINGS ---
LOG_FILE = "logs.txt"
ARCHIVE_DIR = "oldLogs"
FIELD_IMAGE = "field.png"
TIME_STEP = 0.05     
ROBOT_SIZE_IN = 6   
POINT_SIZE = 25  

def copy_to_archive():
    """Copies log to archive, unless an identical copy already exists."""
    if not os.path.exists(LOG_FILE):
        return

    if not os.path.exists(ARCHIVE_DIR):
        os.makedirs(ARCHIVE_DIR)

    # Check if an identical file already exists in the archive
    is_duplicate = False
    for existing_file in os.listdir(ARCHIVE_DIR):
        existing_path = os.path.join(ARCHIVE_DIR, existing_file)
        
        # Compare content of logs.txt with archived files
        if os.path.isfile(existing_path) and filecmp.cmp(LOG_FILE, existing_path, shallow=False):
            is_duplicate = True
            break

    if is_duplicate:
        print(f"Skipping copy: An identical log already exists in '{ARCHIVE_DIR}'.")
    else:
        # Create a timestamped COPY
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        new_filename = f"oldLog_{timestamp}.txt"
        dest_path = os.path.join(ARCHIVE_DIR, new_filename)
        
        shutil.copy2(LOG_FILE, dest_path) # copy2 preserves metadata
        print(f"New unique log copied to: {dest_path}")

def load_and_calculate():
    x, y, h = [], [], []
    if not os.path.exists(LOG_FILE):
        print(f"Error: {LOG_FILE} not found.")
        return None

    try:
        with open(LOG_FILE, 'r') as f:
            for line in f:
                line = line.strip().replace('X:', '').replace('Y:', '').replace('Heading:', '')
                if not line: continue
                
                parts = [float(p) for p in line.split(',')]
                if len(parts) >= 3:
                    x.append(parts[0])
                    y.append(parts[1])
                    h.append(parts[2])
    except Exception as e:
        print(f"Error loading logs: {e}")
        return None

    if len(x) < 2:
        print("Not enough data points in log file.")
        return None

    x, y, h = np.array(x), np.array(y), np.array(h)
    v = np.sqrt(np.gradient(x, TIME_STEP)**2 + np.gradient(y, TIME_STEP)**2)
    accel = np.gradient(v, TIME_STEP)
    
    # COPY AFTER LOADING
    copy_to_archive()
    
    return x, y, h, v, accel

# --- Main Visualization Logic ---
data = load_and_calculate()
if data:
    x, y, h, v, accel = data
    fig, ax = plt.subplots(figsize=(10, 9))
    plt.subplots_adjust(left=0.07, right=0.88, top=0.92, bottom=0.07)

    try:
        img = mpimg.imread(FIELD_IMAGE)
        ax.imshow(img, extent=[-72, 72, -72, 72], alpha=0.6, zorder=1)
    except:
        ax.set_facecolor('#111111')

    ax.plot(x, y, color='cyan', linewidth=1, alpha=0.4, zorder=2)
    points = ax.scatter(x, y, c=v, cmap='plasma', s=POINT_SIZE, 
                        edgecolors='none', alpha=0.9, zorder=3, picker=True)

    ax.set_xticks(np.arange(-72, 73, 12))
    ax.set_yticks(np.arange(-72, 73, 12))
    ax.grid(True, color='white', linestyle=':', alpha=0.2, zorder=1)
    ax.set_xlim(-72, 72); ax.set_ylim(-72, 72)
    
    cbar = plt.colorbar(points, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label('Velocity (in/s)', rotation=270, labelpad=15)
    
    annot = ax.annotate("", xy=(0,0), xytext=(15,15), textcoords="offset points",
                        bbox=dict(boxstyle="round", fc="black", ec="cyan", alpha=0.8),
                        arrowprops=dict(arrowstyle="->", color='white'), color="white", fontsize=9)
    annot.set_visible(False)
    
    robot_footprint = Rectangle((-ROBOT_SIZE_IN/2, -ROBOT_SIZE_IN/2), 
                                 ROBOT_SIZE_IN, ROBOT_SIZE_IN, 
                                 alpha=0.4, color='orange', zorder=4)
    ax.add_patch(robot_footprint)
    robot_footprint.set_visible(False)

    def on_pick(event):
        idx = event.ind[0]
        pos = points.get_offsets()[idx]
        annot.xy = pos
        text = (f"Point: {idx}\n"
                f"X: {x[idx]:.1f}\"  Y: {y[idx]:.1f}\"\n"
                f"H: {h[idx]:.1f}°\n"
                f"V: {v[idx]:.2f} in/s\n"
                f"A: {accel[idx]:.2f} in/s²")
        annot.set_text(text)
        annot.set_visible(True)
        robot_footprint.set_visible(True)
        
        # Heading: 90 - h[idx] because VEX is CW-North and Matplotlib is CCW-East
        transform = Affine2D().rotate_deg_around(x[idx], y[idx], 90 - h[idx]) + ax.transData
        robot_footprint.set_xy([x[idx] - ROBOT_SIZE_IN/2, y[idx] - ROBOT_SIZE_IN/2])
        robot_footprint.set_transform(transform)
        fig.canvas.draw_idle()

    def on_click(event):
        if not event.dblclick:
            cont, _ = points.contains(event)
            if not cont and annot.get_visible():
                annot.set_visible(False)
                robot_footprint.set_visible(False)
                fig.canvas.draw_idle()

    fig.canvas.mpl_connect("pick_event", on_pick)
    fig.canvas.mpl_connect("button_press_event", on_click)
    plt.title("LemLib Physics Analysis | Click points to inspect robot pose", fontsize=12, pad=15)
    plt.show()
