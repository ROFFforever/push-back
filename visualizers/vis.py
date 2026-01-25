import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

# --- SETTINGS ---
LOG_FILE = "logs.txt"
FIELD_IMAGE = "field.png"
TIME_STEP = 0.1  # 100ms intervals
POINT_SIZE = 25  

def load_and_calculate():
    x, y, h = [], [], []
    try:
        with open(LOG_FILE, 'r') as f:
            for line in f:
                # Handle both CSV and Labeled formats
                line = line.strip().replace('X:', '').replace('Y:', '').replace('Heading:', '')
                if not line: continue
                parts = [float(p) for p in line.split(',')]
                if len(parts) >= 3:
                    x.append(parts[0]); y.append(parts[1]); h.append(parts[2])
    except Exception as e:
        print(f"Error loading logs: {e}")
        return None

    x, y, h = np.array(x), np.array(y), np.array(h)
    # Velocity (inches/sec) - using gradient for instant velocity at each point
    v = np.sqrt(np.gradient(x, TIME_STEP)**2 + np.gradient(y, TIME_STEP)**2)
    # Acceleration (inches/sec^2)
    accel = np.gradient(v, TIME_STEP)
    return x, y, h, v, accel

data = load_and_calculate()
if data:
    x, y, h, v, accel = data

    # 1. Setup Canvas (10x10 is the "Sweet Spot" for laptop screens)
    fig, ax = plt.subplots(figsize=(10, 9)) # Slightly wider for colorbar
    plt.subplots_adjust(left=0.07, right=0.88, top=0.92, bottom=0.07)

    # 2. Draw Field
    try:
        img = mpimg.imread(FIELD_IMAGE)
        ax.imshow(img, extent=[-72, 72, -72, 72], alpha=0.6, zorder=1)
    except:
        ax.set_facecolor('#111111')
        print("field.png not found. Using dark background.")

    # 3. Optimized Plotting
    ax.plot(x, y, color='cyan', linewidth=1, alpha=0.3, zorder=2)
    
    # Scatter points colored by Velocity (v)
    points = ax.scatter(x, y, c=v, cmap='plasma', s=POINT_SIZE, 
                        edgecolors='none', alpha=0.9, zorder=3, picker=True)

    # 4. Units & Grid (VEX Tile standard is 12 inches)
    ax.set_xticks(np.arange(-72, 73, 12))
    ax.set_yticks(np.arange(-72, 73, 12))
    ax.grid(True, color='white', linestyle=':', alpha=0.2, zorder=1)
    ax.set_xlim(-72, 72)
    ax.set_ylim(-72, 72)
    ax.set_xlabel("Inches (X)", fontsize=10)
    ax.set_ylabel("Inches (Y)", fontsize=10)

    # 5. Velocity Gradient Bar
    cbar = plt.colorbar(points, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label('Velocity (in/s)', rotation=270, labelpad=15, fontsize=10)

    # 6. High-Speed Annotation (Instant GUI response)
    annot = ax.annotate("", xy=(0,0), xytext=(15,15), textcoords="offset points",
                        bbox=dict(boxstyle="round", fc="black", ec="cyan", alpha=0.8),
                        arrowprops=dict(arrowstyle="->", color='white'), color="white", fontsize=9)
    annot.set_visible(False)

    def on_pick(event):
        # Triggered only when a point is clicked
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
        fig.canvas.draw_idle()

    def on_click(event):
        # Hide the box if you click on the empty field
        if not event.dblclick and annot.get_visible():
            cont, _ = points.contains(event)
            if not cont:
                annot.set_visible(False)
                fig.canvas.draw_idle()

    # Bind interactions
    fig.canvas.mpl_connect("pick_event", on_pick)
    fig.canvas.mpl_connect("button_press_event", on_click)

    plt.title("LemLib Physics Analysis | Click points to inspect", fontsize=14, pad=15)
    print("Visualizer successfully launched at 100ms resolution.")
    plt.show()
