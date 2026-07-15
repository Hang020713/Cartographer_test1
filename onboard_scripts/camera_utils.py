import subprocess
import time
import signal
import os
from datetime import datetime

class CameraRecorder:
    def __init__(self):
        self.process = None
        self.start_time = None
        self.duration = None  # in seconds
        self.is_running = False
        self.filename = None
    
    def _generate_filename(self):
        """Generate filename in DDMMYYYY_HH_MM_SS format"""
        now = datetime.now()
        return now.strftime("%d%m%Y_%H_%M_%S") + ".mp4"
    
    def start_recording(self, camera, duration_seconds):
        self.duration = duration_seconds
        self.start_time = time.time()
        self.is_running = True
        self.filename = self._generate_filename()
        
        command = f"rpicam-vid --camera {camera} -t {duration_seconds * 1000} --codec yuv420 --width 1280 --height 720 -o - | ffmpeg -f rawvideo -pix_fmt yuv420p -s 1280x720 -framerate 30 -i - -c:v libx264 -preset veryfast {self.filename}"
        print(f"command: {command}")
        print(f"Output file: {self.filename}")
        
        try:
            self.process = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            print(f"Recording started (PID: {self.process.pid}) for {duration_seconds} seconds")
            return True
        except Exception as e:
            print(f"Error starting recording: {e}")
            self.is_running = False
            self.filename = None
            return False
    
    def update(self, current_time=None):
        if not self.is_running:
            return False
        
        if current_time is None:
            current_time = time.time()
        
        elapsed = current_time - self.start_time
        remaining = self.duration - elapsed
        
        # Check if duration has elapsed
        if remaining <= 0:
            self.stop_recording()
            return False
        
        # Optional: Print progress every few seconds
        if int(elapsed) % 5 == 0:  # Print every 5 seconds
            print(f"Recording: {elapsed:.1f}s / {self.duration}s elapsed ({remaining:.1f}s remaining)")
        
        return True
    
    def stop_recording(self):
        """Stop the recording process"""
        if self.process and self.is_running:
            try:
                self.process.terminate()
                # Wait a bit for graceful shutdown
                time.sleep(0.5)
                if self.process.poll() is None:
                    self.process.kill()  # Force kill if still running
                print(f"Recording stopped after {time.time() - self.start_time:.1f} seconds")
                print(f"Video saved as: {self.filename}")
            except Exception as e:
                print(f"Error stopping recording: {e}")
            finally:
                self.process = None
                self.is_running = False
                # Keep filename for reference
    
    def get_elapsed_time(self, current_time=None):
        """Get elapsed time since recording started"""
        if not self.is_running or self.start_time is None:
            return 0
        if current_time is None:
            current_time = time.time()
        return current_time - self.start_time
    
    def get_remaining_time(self, current_time=None):
        """Get remaining time in seconds"""
        if not self.is_running or self.start_time is None:
            return 0
        if current_time is None:
            current_time = time.time()
        return max(0, self.duration - (current_time - self.start_time))
    
    def get_filename(self):
        """Return the current filename"""
        return self.filename
    
if __name__ == "__main__":
    recorder = CameraRecorder()
    
    # Start recording for 10 seconds
    recorder.start_recording(0, 5)
    
    # Main loop - update every second
    while recorder.is_running:
        recorder.update()
        time.sleep(1)
    
    print("Recording completed")