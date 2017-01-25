# atlas_walking_control
simple walking control for atlas robot based BDI Controllers

- This is based on drcsim, check for [installation](http://gazebosim.org/tutorials?tut=drcsim_install&cat=drcsim)

**As the instructions above are for older versions of gazebo and drcsim, For installing drcsim with gazebo7, follow below instructions:**                        
`sudo apt-get install gazebo7 drcsim7`


### To test this (with drcsim on inidgo)

- launch the gazebo with atlas         
`roslaunch drcsim_gazebo atlas.launch hand_suffix:=_sandia_hands`
- launch the primitive                
`rosrun atlas_control atlas_control_node`


