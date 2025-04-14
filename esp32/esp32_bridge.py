import RPi.GPIO as GPIO
import serial
import time
import zmq
import json

# Serial port and baud rate for ESP32
SERIAL_PORT = '/dev/esp32'
BAUD_RATE = 115200

# ZMQ connection details
ZMQ_ADDRESS = "tcp://localhost:5555" # Assuming the publisher is on the same machine

print(f"Initializing serial connection to {SERIAL_PORT} at {BAUD_RATE} baud...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
print("Serial connection established.")

print(f"Connecting to ZMQ publisher at {ZMQ_ADDRESS}...")
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect(ZMQ_ADDRESS)
socket.subscribe("")  # Subscribe to all topics
print("ZMQ subscriber connected.")

try:
    # --- SIMPLE TEST LOOP (COMMENTED OUT) --- 
    # print("Starting simple serial test loop...")
    # counter = 0
    # while True:
    #     try:
    #         test_string = f"test_{counter}\\n"
    #         encoded_data = test_string.encode('utf-8')
    #         print(f"Attempting to send: '{test_string.replace('\\n', '\\\\n')}' (Length: {len(encoded_data)} bytes)")
    #         ser.write(encoded_data)
    #         print("Test string sent.")
    #         counter += 1
    #         time.sleep(1) # Send every second
    #     except KeyboardInterrupt:
    #         print("Interrupted.")
    #         break
    #     except Exception as e:
    #         print(f"Error in simple loop: {e}")
    #         break
    # --- END SIMPLE TEST LOOP ---
    
    # --- ORIGINAL ZMQ LOOP (UNCOMMENTED) ---
    print("Listening for detection messages...")
    message_counter = 0 # Initialize message counter
    while True:
        try:
            # Try to receive a message without blocking indefinitely
            message_str = socket.recv_string(flags=zmq.NOBLOCK)
            message_counter += 1 # Increment counter for every received message
            
            # --- Process only every 3rd message ---
            if message_counter % 5 != 0:
                # print(f"Skipping message {message_counter}") # Optional debug print
                continue # Skip processing for this message

            # print(f"Processing message {message_counter}") # Optional debug print
            # If we are processing, reset counter to avoid large numbers (optional but tidy)
            # message_counter = 0 
            
            # --- Original processing logic for the 3rd message ---
            try:
                message = json.loads(message_str)
                # print(f"Parsed message: {message}") # Optional: Debug print

                if 'detections' in message and message['detections']:
                    # Get the label from the first detection
                    first_detection = message['detections'][0]
                    label = first_detection.get('label')

                    if label:
                        print(f"(Msg {message_counter // 3}) Detected object: {label}. Preparing to send to ESP32...") # Modified print
                        # Prepare the string to send
                        string_to_send = label + "\n"
                        encoded_data = string_to_send.encode('utf-8')
                       
                        # --- DEBUG PRINT ---
                        print(f"Attempting to send: '{string_to_send.replace("\\n", "\\\\n")}' (Length: {len(encoded_data)} bytes)") 
                       
                        # Send the label followed by a newline character
                        ser.write(encoded_data)
                        print("Label sent.")
                       
                        # --- RATE LIMITING DELAY ---
                        # time.sleep(0.1) # We might not need the delay anymore, comment out for now
                    else:
                        print("No label found in the first detection.")
                # else: # Optional: Debug print if no detections
                    # print("No detections found in message.")

            except json.JSONDecodeError:
                print(f"Error decoding JSON: {message_str}")
            except KeyError as e:
                print(f"Missing key in message structure: {e}")
            except Exception as e:
                print(f"Error processing message: {e}")

        except zmq.Again:
            # No message received, wait a bit before trying again
            time.sleep(0.01)
            continue # Continue the loop to check again
        except zmq.ZMQError as e:
            print(f"ZMQ Error: {e}")
            time.sleep(1) # Wait before retrying connection/receive
        except KeyboardInterrupt:
            print("Interrupted by user.")
            break
        except Exception as e:
             print(f"An unexpected error occurred: {e}")
             break
    # --- END ORIGINAL ZMQ LOOP ---

finally:
    print("Cleaning up...")
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")
    if 'socket' in locals():
       socket.close()
       print("ZMQ socket closed.")
    if 'context' in locals():
       context.term()
       print("ZMQ context terminated.")
    GPIO.cleanup() # If you were using GPIO elsewhere
    print("Cleanup complete.")

