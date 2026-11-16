# Attribution

NASA/JPL-Caltech: PROMISE visualization meshes and reference imagery. Earlier revisions also used Curiosity and Perseverance meshes and textures. NASA public asset pages and exact downloaded-file hashes are listed in README.md and references/source_checksums.json.

The NASA 3D Resources repository describes its assets as free and without copyright:
https://github.com/nasa/NASA-3D-Resources

NASA media and identifier usage guidance:
https://www.nasa.gov/nasa-brand-center/images-and-media/

Retain NASA/JPL-Caltech credit when sharing the derived model. The included source/derived NASA assets are not relicensed under the code license. NASA identifiers, if present in original textures, do not imply NASA endorsement of the modified rover or its simulation accuracy.

The new assembly, joint structure, lidar enclosure geometry, export scripts, controller examples and documentation were generated for this project. The existing Space ROS and other community URDF repositories were consulted as references; their source code is not bundled here.

Ouster: OS1 Rev 8 exterior STEP CAD and datasheet, obtained from https://ouster.com/downloads. The OS1 housing is derived from its OS1 XX no-halo model; the illustrative FOV solid was removed. Ouster assets and derived housing geometry retain the publisher's applicable terms and are not relicensed under this project's MIT code license. The recessed mounting cradle, 22.5-degree downward tilt, front aperture, placement and sensor calibration are illustrative user-requested modifications.

VisionSIM 0.2.0 is an external dependency. The included integration calls its local Blender service for RGB/depth. VisionSIM code is not bundled. Lidar ray casting is implemented by this project, separately from VisionSIM.
