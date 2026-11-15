# Beat the Humanoid — browser prototype

Source: fujitamasahir/humanoid-spatial-memory@b50f81255756a9c8bd55dddd2ea72c54d58ebb7d. The 36-maze catalog and Python grid evaluator, memory, scheduling, navigation, observation and simulator are vendored unchanged. The demo pedestrian path generator is copied from engine.py. The browser adapter uses the original grid Sensor (radius 3, occlusion, zero noise), not MuJoCo rays, G1 footprint, independent emergency sensor or forward-first action planner. It blocks occupied-cell moves and counts a pedestrian entering the robot's previous cell as a contact. 2D-only, stylized SVG avatars, no physical collision claims. No FastAPI, WebSocket, backend or telemetry. Pyodide 0.27.7 and NumPy download via jsDelivr; computation in a browser Worker.

## Test locally
From the repository root: `python3 -m http.server 8000` and visit http://localhost:8000/games/beat-the-humanoid/ or the project page. Do not open via file://. Initial loading requires internet. Browser runtime has not yet been tested. The public portfolio branch is unchanged.
