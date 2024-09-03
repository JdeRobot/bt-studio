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
        raise ValueError(
            f"'{port_value}' is a read-only port. Only ports connected to the blackboard are writable"
        )

    # Extract the key from the port_value
    key = port_value.strip("{}")

    blackboard = GlobalBlackboard.get_instance()

    # Lazy creation: if key doesn't exist, register it
    if not hasattr(blackboard, key):
        blackboard.register_key(
            key=key, access=py_trees.common.Access.WRITE, required=True
        )

    # Set the value for the key in the blackboard
    setattr(blackboard, key, value)


########### ASCII BT STATUS TO JSON ############################################
def ascii_state_to_state(state_raw):
    letter = [x for x in state_raw]
    state = letter[1]

    match state:
        case "*":
            return "RUNNING"
        case "o":
            return "SUCCESS"
        case "x":
            return "FAILURE"
        case "-":
            return "INVALID"
        case _:
            return "INVALID"


def ascii_tree_to_json(tree):
    indent_levels = 4  # 4 spaces = 1 level deep
    do_append_coma = False
    last_indent_level = -1
    json_str = '"tree":{'

    for line in iter(tree.splitlines()):
        entry = line.strip().split(" ")
        name = entry[1]
        if len(entry) == 2:
            state = "NONE"
        else:
            state = ascii_state_to_state(entry[2])

        indent = int((len(line) - len(line.lstrip())) / indent_levels)
        if not (indent > last_indent_level):
            json_str += "}" * (last_indent_level - indent + 1)

        last_indent_level = indent

        if do_append_coma:
            json_str += ","
        else:
            do_append_coma = True
        json_str += '"' + name + '":{'
        json_str += f'"state":"{state}"'

    json_str += "}" * (last_indent_level + 1) + "}"
    return json_str


def ascii_blackboard_to_json(blackboard):
    json_str = '"blackboard":{'
    do_append_coma = False
    for line in iter(blackboard.splitlines()):
        if line == "Blackboard Data":
            continue
        if do_append_coma:
            json_str += ","
        else:
            do_append_coma = True
        # Remove whitespaces with strip and remove / from entry
        [entry, value] = line.strip()[1:].split(":")
        json_str += f'"{entry.strip()}":"{value.strip()}"'
    json_str += "}"
    return json_str


def ascii_bt_to_json(tree, blackboard, file):
    file.write("{")
    file.write(f"{ascii_tree_to_json(tree)},{ascii_blackboard_to_json(blackboard)}")
    file.write("}")
    file.close()
