import os
import time
import threading
from flask import Flask, request, jsonify
from flask_cors import CORS
import logging

# Create a Flask application to serve as the web server
app = Flask(__name__)
# Allow other devices (e.g., mobile apps) to connect to the API via CORS
CORS(app) 

# Disable Flask's default warning logs to keep the console clean
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# System state variables for communicating with the mobile application
SERVER_STATE = {
    "command": "idle",                              # Current command for the app (e.g., idle, capture)
    "command_id": "",                               # Random command ID to prevent duplicate command execution
    "last_image_path": None,                        # File path where the image should be saved upon receipt
    "upload_received_event": threading.Event(),     # Signal event to notify other parts of the program that an image was received
    "last_seen": 0,                                 # Timestamp of the last request from the mobile device
    "connected": False                              # Connection status of the mobile device (True/False)
}

@app.route('/poll_command', methods=['GET'])
def poll_command():
    # API endpoint for the mobile app to poll for current status and commands periodically
    current_time = time.time() # Capture current time
    
    # If the time since the last connection exceeds 5 seconds, treat it as a new connection
    if current_time - SERVER_STATE["last_seen"] > 5.0:
        # Log that a phone has connected, including its IP Address
        print(f"[System] 📱 Phone Connected! (IP: {request.remote_addr})")
        # Update system status to connected
        SERVER_STATE["connected"] = True 
        
    # Update the timestamp of the most recent polling request
    SERVER_STATE["last_seen"] = current_time 

    # Return the current command action and command ID to the mobile device in JSON format
    return jsonify({
        "action": SERVER_STATE["command"],
        "id": SERVER_STATE["command_id"]
    })

@app.route('/upload', methods=['POST'])
def upload_file():
    # API endpoint to receive target images captured and uploaded by the mobile device
    print(f"[System] 📥 Receiving image...")
    
    # Verify if the request contains a file; return Error 400 if missing
    if 'file' not in request.files: 
        return "No file", 400
        
    # Extract the image file from the request
    file = request.files['file'] 
    
    # Ensure the uploaded file has a filename to prevent empty files
    if file.filename == '': 
        return "No filename", 400
    
    # If the system is ready and a destination path for the image is specified
    if SERVER_STATE["last_image_path"]:
        # Create the destination directory if it does not already exist
        os.makedirs(os.path.dirname(SERVER_STATE["last_image_path"]), exist_ok=True)
        
        # Save the image file to the specified path
        file.save(SERVER_STATE["last_image_path"])
        
        # Log successful save with the filename
        print(f"[System] ✅ Saved: {os.path.basename(SERVER_STATE['last_image_path'])}")
        
        # Trigger the Event signal indicating the upload is complete so scanning can proceed
        SERVER_STATE["upload_received_event"].set()
        
    return "Success", 200 # Return HTTP 200 Success status to the mobile device

def monitor_disconnect():
    # Background thread function to monitor if the mobile device has disconnected
    while True:
        # Pause for 2 seconds per loop to minimize CPU usage
        time.sleep(2) 
        
        # Check current connection status
        if SERVER_STATE["connected"]:
            # If the current time minus the last seen time exceeds 5 seconds
            if time.time() - SERVER_STATE["last_seen"] > 5.0:
                # Log that the phone has disconnected
                print("[System] ❌ Phone Disconnected") 
                # Reset system status to disconnected (False)
                SERVER_STATE["connected"] = False 

def run_flask():
    # Main function to initialize and start the web server
    # Start a background daemon thread to monitor disconnections simultaneously
    threading.Thread(target=monitor_disconnect, daemon=True).start()
    
    # Launch the Flask server
    # host='0.0.0.0' allows connections from other devices on the same local network
    # port=5000 is the designated port for receiving messages
    # use_reloader=False prevents Flask from restarting automatically when code changes
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

