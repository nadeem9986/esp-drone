"""
  Robust Serial to WebSocket Bridge Server
"""
import asyncio
import websockets
import serial
import json
import os
import sys

COM_PORT = 'COM4'
BAUD_RATE = 115200
WS_PORT = 8080

connected_clients = set()

async def broadcast(message):
    if connected_clients:
        await asyncio.gather(*[client.send(message) for client in connected_clients], return_exceptions=True)

async def serial_reader():
    print(f"Connecting to ESP32 on {COM_PORT} at {BAUD_RATE} baud...")
    while True:
        try:
            ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
            print(f"[SUCCESS] Connected to {COM_PORT}! Telemetry active...")
            while True:
                line = ser.readline()
                if line:
                    line_str = line.decode('utf-8', errors='ignore').strip()
                    if line_str:
                        print("ESP32:", line_str)
                        if line_str.startswith('{') and line_str.endswith('}'):
                            await broadcast(line_str)
                        else:
                            # Send non-JSON logs as status updates to Web UI
                            status_msg = json.dumps({"status_log": line_str})
                            await broadcast(status_msg)
                await asyncio.sleep(0.01)
        except Exception as e:
            print(f"[RETRY] Serial error on {COM_PORT}: {e}. Retrying in 2 seconds...")
            await asyncio.sleep(2)

async def ws_handler(websocket):
    print("[CLIENT CONNECTED] Web UI connected!")
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            pass
    finally:
        connected_clients.remove(websocket)
        print("[CLIENT DISCONNECTED]")

async def main():
    print(f"==================================================")
    print(f"   ESP32 MPU6050 Live Bridge Server (COM4 -> 8080)")
    print(f"==================================================")
    
    server = await websockets.serve(ws_handler, "localhost", WS_PORT)
    await asyncio.gather(server.wait_closed(), serial_reader())

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
