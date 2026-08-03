"""
  Drone GCS Live Server (HTTP: 8000, WS: 8080)
"""
import asyncio
import websockets
import serial
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import os
import json

COM_PORT = 'COM4'
BAUD_RATE = 115200
WS_PORT = 8080
HTTP_PORT = 8000
DIRECTORY = r'C:\Users\nad\Desktop\mpu6050_drone_test'

connected_clients = set()

def start_http():
    os.chdir(DIRECTORY)
    server = HTTPServer(('0.0.0.0', HTTP_PORT), SimpleHTTPRequestHandler)
    print(f"HTTP Server running on http://localhost:{HTTP_PORT}/mpu_3d_visualizer.html")
    server.serve_forever()

async def broadcast(message):
    if connected_clients:
        await asyncio.gather(*[client.send(message) for client in connected_clients], return_exceptions=True)

async def serial_reader():
    print(f"Connecting to ESP32 on {COM_PORT} at {BAUD_RATE} baud...")
    while True:
        try:
            ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
            print(f"[SUCCESS] Connected to ESP32 on {COM_PORT}!")
            while True:
                line = ser.readline()
                if line:
                    line_str = line.decode('utf-8', errors='ignore').strip()
                    if line_str:
                        if line_str.startswith('{') and line_str.endswith('}'):
                            await broadcast(line_str)
                        else:
                            status_msg = json.dumps({"status_log": line_str})
                            await broadcast(status_msg)
                await asyncio.sleep(0.005)
        except Exception as e:
            print(f"[RETRY] Serial error: {e}. Retrying in 2 seconds...")
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
    t = threading.Thread(target=start_http, daemon=True)
    t.start()

    print(f"WebSocket Server running on ws://localhost:{WS_PORT}")
    server = await websockets.serve(ws_handler, "localhost", WS_PORT)
    await asyncio.gather(server.wait_closed(), serial_reader())

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
