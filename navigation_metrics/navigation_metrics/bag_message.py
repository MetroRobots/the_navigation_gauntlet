import collections

# Bag Message type for pairing a message with the floating point time it was recorded in the bag file
BagMessage = collections.namedtuple('BagMessage', 't msg')
