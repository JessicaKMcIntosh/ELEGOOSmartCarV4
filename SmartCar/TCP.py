"""Interface with the Smart Car ESP using TCP sockets"""

import json
import socket
import sys

# Create the connect to the Smart Car ESP.
def OpenSocket(ip_address, port, timeout):
    """Create a TCP connection to the Smart Car ESP and return the socket."""
    # Create the socket.
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.settimeout(timeout)

    # Connect to the Smart Car ESP.
    try:
        client_socket.connect((ip_address, port))
    except socket.timeout:
        print('Connection timed out connecting to {0}:{1}'.format(ip_address, port))
        quit()
    except:
        print('Error connecting to {0}:{1}: {2}'.format(ip_address, port, sys.exc_info()[0]))
        quit()

    return client_socket

# Close the socket politely.
def CloseSocket(client_socket):
    """Close the TCP socket to the Smart Car ESP."""
    client_socket.shutdown(0)
    client_socket.close()

# Interact with the Smart Car.
# The callback is used when a heartbeat is received.
# The socket is passed to the callback function.
# NOTE This blocks and waits for the Smart Car to send data.
def Interact(client_socket, callback=None):
    """Interact with the Smart Car."""

    while 1:
        try:
            data = client_socket.recv(1024).decode()
        except socket.timeout:
            print('Connection timed out.')
            quit()

        if len(data) == 0:
            print('No data received. Exiting.')
            CloseSocket(client_socket)
            quit()

        elif data == "{Heartbeat}":
            client_socket.send(data.encode())
            diagnostic = 'Responding to heartbeat.'

            # Execute the callback if one was passed.
            # The socket is passed to the callback.
            if callback:
                callback(client_socket)

        else:
            message = json.dumps(data)
            diagnostic = 'Decoded message: ' + message

        print('Received from server:', data)