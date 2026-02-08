import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os
import shutil
import filecmp
from datetime import datetime

#all the settings
LOG_FILE = "logs.txt"
ARCHIVE_DIR = "oldLogs" #path relative to current dir
FIELD_IMAGE = "field.png"
TIME_STEP = 0.05
ROBOT_SIZE_IN = 12 #doesn't really matter that much, just for the visualization
POINT_SIZE = 25

#in order to save past runs and see whats different
def copy_to_archive():
    if not os.path.exists(LOG_FILE): return
    if not os.path.exists(ARCHIVE_DIR): os.makedirs(ARCHIVE_DIR) #make sure the archive_dir exists
    is_duplicate = False #chcek for duplicate path already saved(but name different)
    for existing_file in os.listdir(ARCHIVE_DIR):
        existing_path = os.path.join(ARCHIVE_DIR, existing_file) #path of a file or potentially a folder
        if os.path.isfile(existing_path) and filecmp.cmp(LOG_FILE, existing_path, shallow=False): #make sure not comparing folder to a file(hence the os.path.isfile()). Then compare file data to new log file to make sure it's not duplicate
            is_duplicate = True
            break #no need to copy if duplicate
    if not is_duplicate: #if same file doesn't already exist
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S") #save file as the current date and time
        shutil.copy2(LOG_FILE, os.path.join(ARCHIVE_DIR, f"oldLog_{timestamp}.txt")) #copy the log file into archive


def load_and_calculate():
    x, y, h = [], [], [] #define our array of x,y,theta values
    if not os.path.exists(LOG_FILE): return None
    try:
        try: #try reading with encoding utf-16
            with open(LOG_FILE, 'r', encoding='utf-16') as f: lines = f.readlines()
        except: #if its not encoded using utf-16 try reading with encoding utf-8
            with open(LOG_FILE, 'r', encoding='utf-8') as f: lines = f.readlines()
        for line in lines:
            line = line.strip().replace('\ufeff', '') #Remove the BOM character if present(sometimes appears in utf-8 files)
            if not line: continue
            parts = [float(p) for p in line.split(',')] #store as float values in array
            if len(parts) >= 3: x.append(parts[0]); y.append(parts[1]); h.append(parts[2]) #add respective values to arrays
    except Exception as e: #if ran into any other error loading file
        print(f"File Load Error: {e}")
        return None
    if len(x) < 2: return None #need at least 2 data points to calculate velocity and acceleration
    x_arr, y_arr, h_arr = np.array(x), np.array(y), np.array(h) #use numpy arrays for faster and easier calculations
    v = np.sqrt(np.gradient(x_arr, TIME_STEP)**2 + np.gradient(y_arr, TIME_STEP)**2) #calculate velocity of robot using 
    # we use equation sqrt(dx/dt^2 + dy/dt^2), basically the hypotenuse of the triangle formed by dx and dy
    #TIME_STEP defines the interval over which the derivative will be calculated, less is more accurate, but 0.05 is good enough
    #np.gradient calculates derivative of the numpy arrays
    accel = np.gradient(v, TIME_STEP) #take derivative of velocity using TIME_STEP as our interval to get acceleration 
    copy_to_archive() #archive  now that we know it's a legit log file
    return x_arr, y_arr, h_arr, v, accel #return all calculated data

#PROGRAM STARTS HERE
data = load_and_calculate() #get calculated points
if data: #make sure data exists/loaded properly
    x, y, h, v, accel = data
    fig, ax = plt.subplots(figsize=(10, 9)) #fig is the main window, ax is the field area with points
    plt.subplots_adjust(left=0.07, right=0.88, top=0.92, bottom=0.15) #make room for buttons and place it comfortably
    try:
        img = mpimg.imread(FIELD_IMAGE)
        ax.imshow(img, extent=[-72, 72, -72, 72], alpha=0.6, zorder=1) #set field imgae to be 144x144 just like real world + slightly transparent for better visibility of points
    except:
        ax.set_facecolor('#111111') #in case field fails to load set this to background color
    
    ax.plot(x, y, color='cyan', linewidth=1, alpha=0.3, zorder=2) #draw line showing path
    points = ax.scatter(x, y, c=v, cmap='plasma', s=POINT_SIZE, alpha=0.7, zorder=3, picker=True) #draw points colored by velocity
    #using plasma colormap
    ax.set_xlim(-72, 72) #x and y limits to match field size
    ax.set_ylim(-72, 72)
    ax.set_title("LemLib Playback Tool 2026", pad=10) #title of window
    plt.show() #actually display the window
