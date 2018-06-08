# Tuning the walk

## Testing the walk

If you want to test the robot walk, you first need to run it, type the following in your rhio shell:

    init
    
This will initialize the robot, and then run the walk:

    walk
    
**Note: you also might want to tare the robot, to do this hold it in the air and run the `tare` command, then put it back on the floor after it's over (you should see "Tare on 2 devices." appearing)**
    
The robot should go to its walk position.

All the parameters are in `/moves/walk`, you can use the following:

* `walkEnable` (`true` or `false`), this enable or disable the swing of the robot (rising the feet sequentially)
* `walkStep` (mm/step), this is the speed of walk for the robot (positive on the front, negative for the back)
* `walkLateral` (mm/step), this is the lateral speed of the robot (positive on the left)
* `walkTurn` (deg/step), this is the rotation speed of the robot

## Dynamic parameters

Here are some dynamic parameters:

* `freq`, this is the frequency of the robot (steps per second)
* `swingGain` (m), this is the amplitude of the robot swinging, which is the lateral move of the robot at each step
* `swingPhase` (0 to 1), this is the phase of swinging, regarding the rest of the body
* `riseGain` (m) is the maximum height of the foot rises during walking
* 

## Walk limits

* `maxStep` is the maximum limit for `walkStep`
* `maxStepBackward` is the negative maximum limit for `walkStep`
* `maxLateral` is the absolute maximum limit for `walkLateral`
* `maxRotation` is the absolute maximum limit for `walkTurn`

## Robot position

* `trunkXOffset_backward` (m) this is the front offset of the body when the robot is walking backward, or not walking
* `trunkXOffset_forward` (m) this is the front offset of the body when the robot is walking forward
* `trunkPitch_backward` (deg) this is the trunk pitch of the body when the robot is walking backward, or not walking
* `trunkPitch_forward` (deg) this is the trunk pitch of the body when the robot is walking forward
* `footYOffset` (m) this is the lateral distance between the two feet when the robot is walking

## Configuring triming

If the robot is moving even with `walkStep`, `walkLateral` and `walkTurn` at `0`, you can ajust the trimming values which are:

* `stepTrim` for the forward/backward trimming
* `turnTrim` for the rotation trimming
* `lateralTrim` for the lateral trimming

For instance, if the robot moves forward even with `walkStep=0`, you can set `stepTrim` to `-4`

## Adjust the walk starting

You can adjust the walk start phase with `startPhase` (0 to 1), which can improve the quality of the start (`walkEnable` going from `false` to `true`

## Saving the parameters

You can have a look at the difference between the robot parameters and the parameters you changed using `diff` in RhIO in the `/moves/walk` node.

If you type `save`, this will save the parameters on the files on the robot.

You can download the parameters on your computer, using for instance the `download.sh` script from `workspace/env`, so it is not only saved on the robot and may be re-deployed later and shared with other developers.
