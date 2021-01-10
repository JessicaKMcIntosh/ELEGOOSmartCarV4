import asyncio


class EchoClientProtocol(asyncio.Protocol):
    def __init__(self, end_connection):
        self.end_connection = end_connection

    def connection_made(self, transport):
        print("Connected to the Smart Car.")
        self.transport = transport

    def data_received(self, data):
        message = data.decode()
        print('Data received: {!r}'.format(message))

        if len(data) == 0:
            print('No data received.')
            self.end_connection.set_result(True)
        elif message == "{Heartbeat}":
            self.transport.write(message.encode())

    def connection_lost(self, exc):
        print('Lost the connection to the Smart Car.')
        self.end_connection.set_result(True)


async def main():
    # Get a reference to the event loop as we plan to use low-level APIs.
    loop = asyncio.get_running_loop()

    # How to tell when to close the connection.
    end_connection = loop.create_future()

    transport, protocol = await loop.create_connection(
        lambda: EchoClientProtocol(end_connection), '192.168.4.1', 100)

    # Wait until the protocol signals that the connection
    # is lost and close the transport.
    try:
        await end_connection
    finally:
        transport.close()


asyncio.run(main())
