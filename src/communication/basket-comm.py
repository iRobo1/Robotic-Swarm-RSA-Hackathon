import struct

BASKET_MSG_TYPE = 10  # any number you pick, just be consistent




def on_custom(team_id, robot_id, internal_type, contents):
    if internal_type == BASKET_MSG_TYPE:
        try:
            x, y, team, scanned, item_delivered, measurement_distance = struct.unpack("<ff B ?? f", contents)
            basket = Basket(
                pos=Position(x, y),
                team=Team(team),
                measurement_distance=measurement_distance
            )
            basket.scanned = scanned
            basket.item_delivered = item_delivered
            print(f"Received basket from robot {robot_id}: {basket}")
            return basket
        except Exception as e:
            print(f"Could not parse basket message: {e}")

# Setup
comm.register_callback_custom(on_custom)

# Sending
send_basket(comm, my_basket)