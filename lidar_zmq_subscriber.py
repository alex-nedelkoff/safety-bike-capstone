import zmq
import time

print("Initializing ZMQ subscriber...")
context = zmq.Context()
subscriber = context.socket(zmq.SUB)

print("Connecting to publisher...")
subscriber.connect("tcp://localhost:5556")
print("Setting subscription filter...")
subscriber.setsockopt_string(zmq.SUBSCRIBE, "LIDAR_DATA")
print("Ready to receive messages!")

message_count = 0
while True:
    try:
        message = subscriber.recv_string()
        message_count += 1
        if message_count % 10 == 0:  # Print every 10th message
            print(f"Received message {message_count}")
            # Print first few measurements as sample
            parts = message.split(';')
            for i, part in enumerate(parts[:5]):  # Show first 5 measurements
                if part.strip():
                    print(f"Measurement {i}: {part}")
    except KeyboardInterrupt:
        print("\nStopping subscriber...")
        break
    except Exception as e:
        print(f"Error receiving message: {e}")
        break

print("Closing subscriber...")
subscriber.close()
print("Terminating context...")
context.term()
print("Done!")