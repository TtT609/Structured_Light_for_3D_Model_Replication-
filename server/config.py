import os
from datetime import datetime

# ==========================================
# CONFIGURATION
# ==========================================

# --- Folder Structure (โครงสร้างโฟลเดอร์) ---
# Main folder for storing scan data, named after the current date (e.g., 28_02_2026_3Dscan)
DEFAULT_ROOT = os.path.join(os.getcwd(), f"{datetime.now().strftime('%d_%m_%Y')}_3Dscan")

# --- Hardware Settings ---
# Projector horizontal screen position (starting at 1920 pixels, indicating the 2nd monitor)
SCREEN_OFFSET_X = 1920  
# Projector screen width (1920 pixels)
SCREEN_WIDTH = 1920     
# Projector screen height (1080 pixels)
SCREEN_HEIGHT = 1080    
# Projector light brightness value (0-255)
PROJ_VALUE = 200        
# Downsampling ratio to increase speed (1 is no reduction, >1 reduces image size)
D_SAMPLE_PROJ = 1       

# --- Checkerboard Settings (การตั้งค่ากระดานหมากรุกสำหรับ Calibrate) ---
# Number of inner corner grid points - rows
CHECKER_ROWS = 7        
# Number of inner corner grid points - columns
CHECKER_COLS = 7
# Size of squares in the checkerboard (in millimeters)
SQUARE_SIZE = 35.0      
