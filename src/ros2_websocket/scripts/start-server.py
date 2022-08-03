import asyncio
from websockets import WebSocketServerProtocol, serve

async def handle_client(websocket : WebSocketServerProtocol):
    
    async for message in websocket:
        await websocket.send(message)

async def main():
    async with serve(handle_client, "localhost", 8765):
        await asyncio.Future()  # run forever 

asyncio.run(main())