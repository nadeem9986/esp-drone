"""
  RC Drift Car Python Bridge Server (HTTP + WebSockets)
"""
import asyncio
import websockets
import serial
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import os

COM_PORT = 'COM4'
BAUD_RATE = 115200
WS_PORT = 8080
HTTP_PORT = 8000
DIRECTORY = r'C:\Users\nad\Desktop\rc_drift_car_gyro'

connected_clients = set()

def start_http():
    os.chdir(DIRECTORY)
    server = HTTPServer(('0.0.0.0', HTTP_PORT), SimpleHTTPRequestHandler)
    print(f"HTTP Server running on http://localhost:{HTTP_PORT}/drift_visualizer.html")
    server.serve_forever()

async def serial_reader():
    print(f"Connecting to ESP32 on {COM_PORT} at {BAUD_RATE} baud...")
    while True:
        try:
            ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
            print(f"[SUCCESS] Connected to RC Drift Gyro on {COM_PORT}!")
            while True:
                line = ser.readline()
                if line:
                    line_str = line.decode('utf-8', errors='ignore').strip()
                    if line_str.startswith('{') and line_str.endswith('}'):
                        if connected_clients:
                            await asyncio.gather(*[client.send(line_str) for client in connected_clients], return_exceptions=True)
                    else:
                        print("ESP32 Log:", line_str)
                await asyncio.sleep(0.005)
        except Exception as e:
            print(f"[RETRY] Serial error: {e}. Retrying in 2s...")
            await asyncio.sleep(2)

async def ws_handler(websocket):
    print("[UI CONNECTED] 3D RC Drift Car UI connected!")
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            pass
    finally:
        connected_clients.remove(websocket)

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
