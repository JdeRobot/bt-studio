import py_trees

class GlobalBlackboard:

    _instance = None

    @staticmethod
    def get_instance():

        if GlobalBlackboard._instance is None:
            GlobalBlackboard._instance = py_trees.blackboard.Client(name="Global")
            
        return GlobalBlackboard._instance

def get_port_content(port_value):

    if port_value.startswith("{") and port_value.endswith("}"):
        bb_key = port_value.strip("{}")
        blackboard = GlobalBlackboard.get_instance()
        
        # Return the value of the blackboard entry if it exists, otherwise None
        return getattr(blackboard, bb_key, "")
    else:
        return port_value

def set_port_content(port_value, value):

    if not (port_value.startswith("{") and port_value.endswith("}")):
        raise ValueError(f"'{port_value}' is a read-only port. Only ports connected to the blackboard are writable")

    # Extract the key from the port_value
    key = port_value.strip("{}")

    blackboard = GlobalBlackboard.get_instance()

    # Lazy creation: if key doesn't exist, register it
    if not hasattr(blackboard, key):
        blackboard.register_key(key=key, access=py_trees.common.Access.WRITE, required=True)

    # Set the value for the key in the blackboard
    setattr(blackboard, key, value)
