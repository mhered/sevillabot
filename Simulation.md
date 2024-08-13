# Simulation

Spawn the robot


```bash
$ ros2 launch sevillabot rsp.launch.py use_sim_time:=true
```

Launch gazebo

```bash
$ ros2 launch sevillabot launch_sim.launch.py
```

Teleop with keyboard

```bash
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Exporting a FreeCAD assembly to STL

Issue: Body placement is not respected when exporting an assembly as STL.
Workaround (see https://github.com/FreeCAD/FreeCAD/issues/12278):

1. Move all the items you want to export to a Part
2. Go to the Mesh Workbench
3. Create a Mesh from the Part
4. Export those meshes as STL

To maintain local origin, copy the Part to root axes then export this copy of the part

Issue: cannot export to DAE format (missing pycollada error message)

## Adding a mesh to URDF

Place meshes in `meshes` folder, modify to `CMakeLists.txt` to install this directory 

Use `filename="file://$(find sevillabot)/..."` for path

Note: need to scale when importing (FreeCAD units are mm, Gazebo units are m)

```xml
<mesh filename="file://$(find sevillabot)/meshes/ASSY_sevillabot_base.stl" scale="1.0e-3 1.0e-3 1.0e-3"/>
```

## To Do

- [ ] add wheels mesh
- [ ] add castor mesh
- [ ] align height of castor
- [ ] is collision between wheels and chassis a problem?
- [ ] adjust angle of lidar
- [ ] measure and correct position of wheels
- [ ] measure and correct CoGs, masses, inertias, 
