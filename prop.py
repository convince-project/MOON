import oracle

# property to verify
PROPERTY = "historically{p}"

# declaration of predicates used in the property (initialization at time 0)
predicates = dict(
    time = 0,
    p = True,
)

# function to abstract a dictionary (obtained from Json message) into a list of predicates
# the behavior of the function must be defined by the user depending on the property and topic/service message
def abstract_message(message):
    predicates['time'] = message['time']
    predicates['p'] = message['data'] is not None
    return predicates
