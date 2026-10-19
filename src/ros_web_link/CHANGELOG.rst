^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_web_link
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2026-10-19)
------------------
* Independent-package API/examples closeout: Python/JS/HTTP/WS/launch reference
  docs, runnable examples, JS barrel, ``python -m ros_web_link`` entry, and
  ``FoxgloveWsClient.get_params``/``set_param`` helpers.
* ``/health`` gains additive ``version`` and ``protocol`` fields.
* Examples/docs are installed under ``share/ros_web_link``; no wire-protocol
  change (Foxglove WebSocket v1 / CDR remains frozen).
* Console visual pass from RViz2/Gazebo/rqt reference windows: PointCloud2 round
  sprites + Size(Pixels)/AxisColor (cfg v5), scene HUD/fixed frame, Displays type
  color dots + hover duplicate, and a dedicated graph workspace with rqt_graph
  ``Nodes only`` mode / filters / hover neighborhood.

1.0.0 (2026-10-16)
------------------
* Initial independent transport package: gateway, Foxglove WS proxy, JS client
  library, monitor console, comm fixture, MoveIt/official-sim E2E suites.
