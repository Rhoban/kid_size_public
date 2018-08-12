# Rhoban HL kid public code


## Installing APT dependencies

    apt-get install gcc cmake git libtinyxml-dev libncurses5-dev \
        php-cli php-xml libopencv-dev libv4l-dev gnuplot5-qt \
        python-pip python-empy python-setuptools python-nose chrpath ffmpeg

## Installing catkin

    sudo pip install -U catkin_tools mock
    
## Having a GitHub account with your public key

If you don't have a SSH key, generate one using:

    ssh-keygen -t rsa
    
Sign in on your GitHub account and go to Settings, and then "SSH and GPG keys". Click "New SSH key" and copy the content of `.ssh/id_rsa.pub` in the key field, choose any name you want and validate the new key.

## Installing the workspace

First, you need to clone the rhoban workspace:

    git clone https://github.com/rhoban/workspace.git
    cd workspace
    
And then run the setup:

    ./workspace setup
    
And install the repository:

    ./workspace install rhoban/hl_kid_public
    
## Installing FlyCapture dependency

To use the camera, you'll need to have the FlyCapture installed, clone this repository:

    git clone https://github.com/RhobanDeps/flycapture.git
    
And run the install script:

    cd flycapture
    sudo ./install_flycapture.sh
    
Maybe there will be issues with apt packages, in this case, run:

    sudo apt --fix-broken install
    
And try again (you might need to repeat this 2 or 3 times)

## Installing your environment

You'll need to clone and put your robots environments in the `env` folder

## Compiling Rhio Shell

You can now compile the RhIO Shell:

    ./workspace build RhIOShell
    
Run this command to add `rhio` to your `$PATH`:

    echo export "PATH=\"\$PATH:$PWD/devel_release/lib/RhIOShell/\"" >> ~/.bashrc
    
Don't forget to re-run the shell to have the change, you should then be able to run the `rhio` command
    
## Compiling the robot program (KidSize)

Run the following to build the program:

    ./workspace build kid_size
    
## Communicating with the robot

Our robots communicate with the `10.0.0.1` ip address, so you need to configure your computer to a compatible static address like `10.0.0.2`

It is strongly recommended that you add your private key to the robot. By copying the content of `.ssh/id_rsa.pub` in the `.ssh/authorized_keys` inside the robot.
    
## Deploying the program to the robot

This will deploy the program on the robot:
    
    ./deploy
    
## Running the program on the robot

This will remotely run the program on the robot

    ./run
    
## Connecting to the robot with rhio

    rhio 10.0.0.1
    
