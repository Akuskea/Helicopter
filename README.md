# Helicopter
This project was created in Python, and its goal is to create an animation of an object represented by a triangulated mesh. I animated the helicopter's two rotor (the top rotor and the tail rotor). The animation also showed the chopper take off and land vertically. To perform this maneuver, I used my knowledge of 3D transformations and the concept of changing coordinate frames. I started by using the mesh-cutting tool to separate the different mesh parts that would be in motion; then, I labeled them. Afterward, I assigned a local coordinate frame to each part and related them by transformation matrices. The placement of local frames (the main-body coordinate frame, the top-rotor coordinate frame, and the tail-rotor coordinate frame ) was chosen using: https://nbviewer.org/github/eraldoribeiro/rendering3DinColab/blob/main/displayMeshInColabUsingOpen3DandPlotly.ipynb. The tail rotor rotates in the x-axis, the top rotor rotate in the z-axis and the motion was created by transforming (rotating and translating) the points cloud of the parts.

# Video Youtube:

https://youtu.be/12yDAkh1FgE 
