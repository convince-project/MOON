# MOON

MOON (MOnitoring ONline) is a runtime monitor developed for CONVINCE on top of the ROSMonitoring tool.
MOON accepts the same description of
SCAN and provides monitor generation for properties and
models. Currently, only monitor generation for properties is
implememented on top of [ROSMonitoring](https://github.com/autonomy-and-verification-uol/ROSMonitoring),
working for ROS2 topics and services.

In perspective, the tool will include also monitoring for
models, i.e., the capability of ensuring that the concrete execution
of some elements of the control architecture or the environment
correspond to the abstract model utilized at design-time for
SCAN. MOON will notify violations of properties and models so that
other tools can be invoked to amend plans or models and adapt the
control architecture to new and unforeseen situations. 


