# Startup procedure

## Deploying

First, be sure you have the correct version compiled and deployed on the robot.

If you don't, you have to run the deploy script:

    ./deploy

**Note: be sure that you have the correct version of everything**

If everything is already deployed, you can ignore this step and run (next step)

## Running

Start the program on the robot by running:

    ./run
    
You should have outputs on your screen, you can exit the output (by `CTRL+C` for instance).

## Run the startup script

The startup script setups everything for the begining of a game, simply run:

    ./startup
    
And follow the interactive procedure, the steps are detailed below:

* **Low level checks**, the startup script checks that everything is ok (all devices are present) and warns you if it's not
* **Drop-In**, the startup asks if you are engaging a drop-in game, because the IDs of the robot in the team play is not the same. If it is indeed a Drop-In, the team ID will then be asked, you should get this information before the begining of the game if you want to get the good info from the referee
* **Current field**, the field is asked. This should be updated at the competition and it is used to join the correct Wi-Fi network.
* **Wi-Fi**, the startup asks if it needs to run the wifi script. This is the `./wifi` that can be found in workspace that configures the interfaces. It also need to be updated during the competition according to the network setup.
* **Referee check**, the presence of the referee is checked, this is really important if you want to play a game to have no warning at this point, because it means that you have correct information from the referee.
* **Init**, when you will press enter, the robot will be initialized and go to its "zero" position
* **Walk**, then, another step will run the walk to put the robot in the walk position
* **Tare**, before you press enter to run this procedure, you need to hold the robot in the air. It will then calibrate its pressure sensor. The feet should not touch the ground or anything else to have this procedure going OK.
* **Gyro-Tare**, before you press enter to run this procedure, put the robot on the ground and be sure it doesn't move at all. This will calibrate the zero of the gyroscopes. The robot should not move during the calibration.
* **Role**, the robot role is asked, pre-defined roles can be attributed in the startup scripts, and it will depend on when you are putting the robot
* **Side**, the attacking side is asked, this is the side (`l` for left and `r` for right) where the robot should attack, that is seen from the technical table where the referee box is installed 
* **RoboCup**, this is the final step, if you press enter the robot head should start moving, searching for the ball, and the robot can be put in its starting position



