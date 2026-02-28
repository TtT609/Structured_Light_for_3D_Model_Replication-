import threading
import tkinter as tk

# Import main components of the 3D scanner system, controlling the simulated Flask server
from server import run_flask
from gui import ScannerGUI

def main():
    # ----------------------------------------------------------------
    # Main entry point where the system begins running and welcomes the user
    # ----------------------------------------------------------------
    print("----------------------------------------------------------------")
    print("   Project 3D Scanner Controller (With 360 Radial Fix)")
    print("----------------------------------------------------------------")
    
    # Start the web server as a background daemon thread to receive mobile app data concurrently
    threading.Thread(target=run_flask, daemon=True).start()
    
    # Initialize the root window of the GUI using the Tkinter library
    root = tk.Tk()
    
    # Pass the root window to the ScannerGUI class to build the 6-tab menu layout
    app = ScannerGUI(root)
    
    # Run the mainloop to keep the window active and prevent the program from exiting
    root.mainloop()

# Execute the main function if this script is run directly
if __name__ == "__main__":
    main()
