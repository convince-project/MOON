import oracle

# property to verify
PROPERTY = "historically( not ( (not {PLACE}) since[7:] (once {MOVE}) ) )"

# declaration of predicates used in the property (initialization at time 0)
predicates = dict(
    time = 0,
    IDLE = True,
    PICK = False,
    MOVE = False,
    PLACE = False,
    strength = 0.0,
)

# function to abstract a dictionary (obtained from Json message) into a list of predicates
# the behavior of the function must be defined by the user depending on the property and topic/service message
def abstract_message(message):
    predicates['time'] = message['time']
    predicates['strength'] = message['data'] if message['topic'] == 'gripper_strength' else predicates['strength']
    predicates['IDLE'] = message['data'] == 'IDLE' if message['topic'] == 'action_status' else predicates['IDLE']
    predicates['PICK'] = message['data'] == 'PICK' if message['topic'] == 'action_status' else predicates['PICK']
    predicates['MOVE'] = message['data'] == 'MOVE' if message['topic'] == 'action_status' else predicates['MOVE']
    predicates['PLACE'] = message['data'] == 'PLACE' if message['topic'] == 'action_status' else predicates['PLACE']
    return predicates
