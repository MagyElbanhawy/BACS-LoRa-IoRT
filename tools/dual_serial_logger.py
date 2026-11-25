#!/usr/bin/env python3
import sys
import time
import threading
import serial

def log_port(port, baud, output_filename, duration_sec=120):
    print(f"[{port}] Starting capture -> {output_filename}")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        with open(output_filename, 'w') as f:
            f.write("timestamp_ns,raw_line\n")
            start_time = time.time()
            while time.time() - start_time < duration_sec:
                line = ser.readline()
                if line:
                    ts = time.time_ns()
                    decoded = line.decode(errors="replace").strip()
                    f.write(f"{ts},{decoded}\n")
        ser.close()
        print(f"[{port}] Completed successfully.")
    except Exception as e:
        print(f"[{port}] Error: {e}")

if __name__ == "__main__":
    # Adjust ports to match your hardware layout
    port_sender = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    port_receiver = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM1"
    
    t1 = threading.Thread(target=log_port, args=(port_sender, 115200, "sender_serial_log.csv", 120))
    t2 = threading.Thread(target=log_port, args=(port_receiver, 115200, "receiver_serial_log.csv", 120))
    
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    print("Dual-board logging session finished. Both CSV files generated.")