# pigrade pro - A CAN-over-IP file trans tool using MQHP protocol or original format

The original purpose of pigrade was to allow multiple machines to communicate with
a CAN bus without the need for multiple CAN controllers. It was originally meant
for development/evaluation and is **NOT** intended to be used in production
environments.
There are no security precautions taken to authenticate devices connecting from
the IP-network, and messages are blindly forwarded without sanitizing their
contents. It should go without saying that this is not meant to be used in an
actual car or industrial control network.


## Usage

There are two possible use-cases for pigrade:

* To connect IP-based machines to a CAN bus
* To connect two CAN buses over an IP network

How to use pigrade in these situations will be described in the following sections.


### IP-to-CAN gateway

In the first case, pigrade can be started like this:

```
$ pigrade -p 1234
```

This way, pigrade will listen for incoming TCP connections on port 1234 and forward
messages between *all* connected CAN buses and established TCP connections. However,
messages received on one CAN bus will not be forwarded to another.


### CAN-over-IP tunnel

To connect two CAN buses over an IP network, start pigrade on two different machines
like the following example.

Start a pigrade server on machine A (let's assume its IP is 10.0.0.1):

```
$ pigrade -p 1234
```

Then, start pigrade as a client on machine B:

```
$ pigrade -c 10.0.0.1 -p 1234
```

Now, messages received on any CAN interfaces on machine A will be forwarded to all
CAN interfaces of machine B, and vice-versa. Again, messages will not be forwarded
between different CAN interfaces of the same machine.


### Notes

You do not have to tell pigrade what SocketCAN interfaces to use for sending and
receiving messages to and from the connected CAN buses. Upon startup, pigrade will
iterate over all network interfaces, looking for interfaces containing the string
"can". This way, most common interfaces (e.g. can0, vcan0) will be automatically
detected.


## License

pigrade is free software, distributed under the terms of the GNU GPLv3+. For further
information, see the COPYING file.
