"""demo4testscript controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor


print("Running")

# create the Robot instance.

demo4controller = Supervisor()

esp1 = demo4controller.getFromDef("esp1")

print(esp1.getPosition())
