import oracle

# property to verify
PROPERTY = "historically( {hello} )"

# declaration of predicates used in the property (initialization at time 0)
predicates = dict(
    time = 0,
    hello = True,
)

# function to abstract a dictionary (obtained from JSON message) into a list of predicates
# the behavior of the function must be defined by the user depending on the property and topic/service message
def abstract_message(message):
    predicates['time'] = message['time']
    predicates['hello'] = "Hello World" in message['data']
    return predicates
