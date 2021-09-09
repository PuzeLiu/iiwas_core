# Package iiwas_description

This package contains the urdf and mesh file of iiwa14 and the base frame.

Load two iiwas robot with frame the urdf in ros parameter server (no rviz):
```console
roslaunch iiwas_description iiwas_description.launch
```

The *robot\_description* is loaded within different namespaces: *iiwa_front*, *iiwa_back*, and *iiwas\_frame*.

Load _rviz_ with two two robots and *joint\_state\_publisher_gui* :
```console
roslaunch iiwas_description iiwas_rviz.launch
```




