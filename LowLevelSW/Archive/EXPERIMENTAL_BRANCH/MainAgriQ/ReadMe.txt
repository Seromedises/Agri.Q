#######################################
# Agri.Q experimental version v1.1_CL #
#######################################

Last edit 18/01/2022

Branced from Agri.Q stable version v1.0

#########################
# EXPERIMENTAL FEATURES #
#########################

- Receive Pitch, roll and advance references by serial communication
- Pitch, roll and andavance closed loop position controls by swicthing SW1 (Enable brakes - Disable brakes - Programmable function)
- closed loop position controls mode instead of serial log mode


###########
# RESULTS #
###########

- Closed loop reference acquired by serial ONLY in automatic mode (SW1 in upper position)
- Closed loops don't work or stop if switched out of automatic mode
- Good closed loop precision, pitch and roll are close to their sensor limits, advance could be improved, but odometry has its limitations. Slowing down the advance seems to improve the accuracy.
- GetBaseMeasures() convenient to group several functions
- Manual operation still working
- Easy to switch between modes


###############
# PAST ISSUES #
###############

- Pitch ON/OFF control generates EM noise on joystick receiver -> undesired commands are sent to ALL channels. Seems to be solved.

##########
# ISSUES #
##########

- Serial log not implemented to use SW1 (Enable brakes - Disable brakes - Programmable function) to switch to closed loop posiiton control. It can be implemented but not related to SW1.

