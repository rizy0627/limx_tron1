import asyncio
import websockets
import json
import time
import uuid

# Configuration
URI = "ws://127.0.0.1:5000"
ACCID = "TEST_CLIENT_FULL"

def create_request(title, data=None):
    if data is None:
        data = {}
    return {
        "accid": ACCID,
        "title": title,
        "timestamp": int(time.time() * 1000),
        "guid": str(uuid.uuid4()),
        "data": data
    }

async def wait_for_response(websocket, target_title, target_guid=None, timeout=3.0):
    """Wait for a specific response title, optionally matching GUID."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            # Set a small timeout for recv to allow checking time.time()
            message = await asyncio.wait_for(websocket.recv(), timeout=0.5)
            data = json.loads(message)
            
            # Check title
            if data.get("title") == target_title:
                # Check GUID if provided (for responses)
                if target_guid:
                    if data.get("guid") == target_guid:
                        return data
                else:
                    # For notifications (no request GUID to match)
                    return data
            
            # Print other messages for debug
            # print(f"Ignored: {data.get('title')}")
            
        except asyncio.TimeoutError:
            continue
        except Exception as e:
            print(f"Error reading socket: {e}")
            break
            
    return None

async def test_mode_switch(websocket, mode_name, request_title, response_title):
    print(f"\n--- Testing {mode_name} ---")
    req = create_request(request_title)
    print(f"Sending: {request_title}")
    await websocket.send(json.dumps(req))
    
    resp = await wait_for_response(websocket, response_title, req["guid"])
    
    if resp:
        result = resp.get("data", {}).get("result", "unknown")
        print(f"✅ Success! Received {response_title}. Result: {result}")
        return True
    else:
        print(f"❌ Failed! Timeout waiting for {response_title}")
        return False

async def test_imu_stream(websocket):
    print("\n--- Testing IMU Stream ---")
    # Enable IMU
    req = create_request("request_enable_imu", {"enable": True})
    print("Sending: request_enable_imu = True")
    await websocket.send(json.dumps(req))
    
    # Wait for ack
    resp = await wait_for_response(websocket, "response_enable_imu", req["guid"])
    if not resp:
        print("❌ Failed to enable IMU (No response)")
        return
    print("✅ IMU Enabled. Waiting for data...")

    # Wait for 3 notifications
    count = 0
    for _ in range(10): # Try 10 times to get 3 messages
        notify = await wait_for_response(websocket, "notify_imu", timeout=1.0)
        if notify:
            print(f"  -> Received IMU Data: {notify['data'].keys()}")
            count += 1
        if count >= 3:
            print("✅ IMU Stream verified!")
            break
    
    if count < 3:
        print("⚠️ Warning: IMU data stream seems unstable or silent.")

    # Disable IMU (Clean up)
    req = create_request("request_enable_imu", {"enable": False})
    await websocket.send(json.dumps(req))
    print("Sent: request_enable_imu = False")

async def main():
    print(f"Connecting to {URI}...")
    try:
        async with websockets.connect(URI) as websocket:
            print("Connected!")
            
            # 1. Test Stand Mode
            await test_mode_switch(websocket, "Stand Mode", "request_stand_mode", "response_stand_mode")
            
            # 2. Test Walk Mode
            # await test_mode_switch(websocket, "Walk Mode", "request_walk_mode", "response_walk_mode")
            
            # # 3. Test Sit Down
            # await test_mode_switch(websocket, "Sit Down", "request_sitdown", "response_sitdown")
            
            # 4. Test IMU Stream
            await test_imu_stream(websocket)
            
            print("\n=== All Tests Completed ===")
            
    except ConnectionRefusedError:
        print("Error: Could not connect to WebSocket server. Is 'websocket_cmd_vel_bridge' running?")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nTest interrupted.")

