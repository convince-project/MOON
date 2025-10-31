MOON (MOnitoring ONline)
================================================================

MOON (MOnitoring ONline) is a runtime monitoring framework developed for CONVINCE on top of the `ROSMonitoring <https://github.com/autonomy-and-verification-uol/ROSMonitoring/tree/ros2/>`_ tool, providing monitor generation for properties and models.

MOON will notify violations of properties and that other tools can be invoked to amend plans or models and adapt the control architecture to new and unforeseen situations.
MOON uses PastMTL as property specification language, relying on Reelay for their verification.

The tool includes also monitoring for models, i.e., the capability of ensuring that the concrete execution of some elements of the control architecture or the environment correspond to the abstract model utilized at design-time.

Contents
--------

.. toctree::
   :maxdepth: 2

   installation
   usage
