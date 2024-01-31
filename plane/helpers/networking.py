import zmq

import logging
import atexit


def make_zmq_socket(ip, port, parent_name, bind_or_connect, pub_or_sub, context=None):
    """
    Helper that makes ZMQ socket. We do this in many places, with different variations on bind or connect, ip, and port
    so it's nice to have a helper function to do it, especially there's lots of lines that need to be read in order
    to understand what's going on every time we make a socket. This is made for only PUB and SUB sockets, ZMQ has other
    options but they don't suit our needs.
    ZMQ docs for our team: https://docs.google.com/document/d/1J_Gdc7iPWV1LekobKUTpKMzVt-vNSunPHbohVICuwz4/

    Args:
        ip: ip address. Probably 1 of: 0.0.0.0 (same as *), 127.0.0.1 (same as localhost), the remote server IP, or
            the local computer's IP
        port: Port that the socket will be bound to or connected to, e.g. 5557. Our system has one publisher and one subscriber for
              each port.
        parent_name: Just the name of the class that made this, for logging purposes
        bind_or_connect: Whether to bind or connect the socket. It's harder to bind to a remote server from a local
                         computer, so generally the server binds itself and the plane and ground connect to it.
                         Read the docs above for more info.
        pub_or_sub: Whether this socket is a publisher or subscriber.
        context: zmq context, if a context is passed in, we use that, otherwise we make a new one. Useful if the
                 calling script wants a handle on the context to be able to close it later.


    Returns: ZMQ socket. Can recv() or send_json(), depending on whether it's a publisher or subscriber.
    """
    if bind_or_connect not in ["bind", "connect"]:
        raise ValueError("bind_or_connect must be either 'bind' or 'connect'")
    if pub_or_sub not in ["pub", "sub"]:
        raise ValueError("pub_or_sub must be either 'pub' or 'sub'")

    if context is None:
        context = zmq.Context()

    # HWM = high water mark. If we don't set this to 0, then if the subscriber is not receiving data fast enough,
    # it will drop data. We don't want this, we want to keep all data, so we set it to 0.
    if pub_or_sub == "pub":
        socket = context.socket(zmq.PUB)
        socket.setsockopt(zmq.SNDHWM, 0)
    else:
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b"")  # accept any bytes in this subscriber
        socket.setsockopt(zmq.RCVHWM, 0)
    # don't wait for pending messages when a script is killed
    context.setsockopt(zmq.LINGER, 1)

    address = f"tcp://{ip}:{port}"
    getattr(socket, bind_or_connect)(address)  # bind or connect

    past_tense = "bound" if bind_or_connect == "bind" else "connected"

    logging.info(f"{parent_name} {pub_or_sub} {past_tense} to {address}")

    def cleanup_zmq_context():
        socket.close()
        context.term()

    atexit.register(cleanup_zmq_context)
    return socket
