# Hexabot
some python scripts control motor's speed, synchrony


For SerialCom1, it's control the motors as the Set, the synchrony also based on the Set, so this script only possible to move robot forward, if do not make any changes in the source code. Of course, if want to execute other movement, need a new script.


SerialCom2 control motors separately, feed each motor forwad speed and rotate speed, it will update the position for each motor, then do the synchronous operaion. This is possible realize all movement with in one script. Also combine keyboard resding script to it, so press the arrow keys will keep send speed value to motors.
