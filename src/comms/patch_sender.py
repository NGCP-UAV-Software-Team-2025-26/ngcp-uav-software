import sys

filename = "fusion_sender.py"
try:
    with open(filename, 'r', encoding='utf-8') as f:
        content = f.read()

    # The missing wait_heartbeat
    target = "active_path:   Path | None = None"
    replacement = "print('[fusion_sender] Waiting for MAVLink heartbeat...')\n    mav.wait_heartbeat()\n    print('[fusion_sender] Heartbeat received, ready to send.')\n    active_path:   Path | None = None"
    
    if "mav.wait_heartbeat()" not in content:
        new_content = content.replace(target, replacement)
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(new_content)
        print("Successfully patched fusion_sender.py to wait for heartbeat")
    else:
        print("Already patched.")

except Exception as e:
    print("Error patching:", e)
