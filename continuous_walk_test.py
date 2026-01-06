import asyncio
import websockets
import json
import time
import uuid

async def send_continuous_walk():
    uri = "ws://127.0.0.1:5000"
    
    print(f"Connecting to {uri}...")
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected! Starting continuous walk command test...")
            
            # 持续发送 5 秒
            start_time = time.time()
            duration = 5.0 # seconds
            frequency = 0.1 # 10Hz (sleep 0.1s)
            
            # 生成一个固定的 ACCID (模拟)
            accid = "TEST_CLIENT_001"

            while time.time() - start_time < duration:
                # 构建请求数据
                # 轮足/点足模式: x=0.5 (前进)
                payload = {
                    "accid": accid,
                    "title": "request_twist",
                    "timestamp": int(time.time() * 1000),
                    "guid": str(uuid.uuid4()),
                    "data": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": -1.0
                    }
                }
                
                # 发送 JSON
                await websocket.send(json.dumps(payload))
                print(f"Sent walk command: x=0.5 (Time left: {duration - (time.time() - start_time):.1f}s)")
                
                # 保持频率
                await asyncio.sleep(frequency)

            # 发送停止指令
            print("Time's up! Sending stop command...")
            stop_payload = {
                "accid": accid,
                "title": "request_twist",
                "timestamp": int(time.time() * 1000),
                "guid": str(uuid.uuid4()),
                "data": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                }
            }
            await websocket.send(json.dumps(stop_payload))
            print("Stop command sent.")
            
    except ConnectionRefusedError:
        print("Error: Could not connect to WebSocket server. Is 'websocket_cmd_vel_bridge' running?")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # Install websockets if needed: pip install websockets
    try:
        asyncio.run(send_continuous_walk())
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")

