# KinovaMicoNonLinearControlDemo
This repo is an extensional of my earlier Puma560ManipulatorDemo work. This repo extends all the same algorithms to a 6 DOF manipulator. 
Be sure to run KinovaMicoSimscapeInitializeScript.m prior to attempting to run the model. Once the script has been run once, you can run the appropriate section depending which algorithm you are attempting to use. It should be noted that the manipulator may run into singularities when operating in task space. This may cause numerical issues for the Simulink engine. These singularities can be avoided by calculating the desired joint trajectories using the damped least squares method. I have not yet implemented this in the repo, this change will be coming soon.

<img src="kinovaMic.gif" width="400" height="500"> 
