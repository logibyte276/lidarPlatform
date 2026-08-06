import os
import time

class HardwarePWM:
    def __init__(self, chip, channel, frequency=50):
        self.chip = chip
        self.channel = channel
        
        # Base paths for the Linux sysfs architecture
        self.chip_path = f"/sys/class/pwm/pwmchip{self.chip}"
        self.path = f"{self.chip_path}/pwm{self.channel}"
        
        # 1. Export the channel if it isn't already visible
        if not os.path.exists(self.path):
            with open(f"{self.chip_path}/export", "w") as f:
                f.write(str(self.channel))
            # Give Linux a microsecond to populate the new virtual directory
            time.sleep(0.1)
            
        # 2. Calculate and set the total period (Frequency to Nanoseconds)
        # Period (ns) = 1,000,000,000 / Frequency (Hz)
        self.period_ns = int(1e9 / frequency)
        
        # Safely set period: zero out duty cycle first to avoid (duty > period) crashes
        with open(f"{self.path}/duty_cycle", "w") as f:
            f.write("0")
        with open(f"{self.path}/period", "w") as f:
            f.write(str(self.period_ns))
            
        # 3. Open the duty cycle file handle for rapid reuse
        self.duty = open(f"{self.path}/duty_cycle", "w")
        
        # 4. Turn the PWM signal ON
        with open(f"{self.path}/enable", "w") as f:
            f.write("1")

    def write_microseconds(self, us):
        # Convert microseconds to nanoseconds
        ns = int(us * 1000)
        
        # Guard rail: prevent values exceeding the configured period
        if ns > self.period_ns:
            ns = self.period_ns
            
        self.duty.seek(0)
        self.duty.write(str(ns))
        self.duty.flush()

    def stop(self):
        # Turn off signal and clean up file handlers gracefully
        with open(f"{self.path}/enable", "w") as f:
            f.write("0")
        self.duty.close()
        
        # Optional: unexport to clean up the sysfs directory completely
        try:
            with open(f"{self.chip_path}/unexport", "w") as f:
                f.write(str(self.channel))
        except IOError:
            pass
