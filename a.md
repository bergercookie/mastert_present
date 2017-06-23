- Modular design; Makes a clear distinction between:
  - Acquisition of measurements, initial graph construction
  - Computations part - graph optimization
- Backend works the same for 2D/3D constraints `\( \rightarrow \)` 2D/3D SLAM
- Not restrained to a particular map format. We can:
  - include landmarks in the mathematical formulation **or**
  - execute *"mapping with known poses"* and then construct the map by aligning
      the measurements.
- Computational complexity: *linear* in the number of edges
- Any sensor can be used as long as it provides inter-pose constraints


