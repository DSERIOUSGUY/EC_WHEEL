from controller import Keyboard

class CruiseControlInput:
    def __init__(self, keyboard):
        self.keyboard = keyboard

        self.targetMemory = (1.0, 0)
        self.active = False
        self.lastClickState = False

    def clicked(self):
        #checks if 'space' is one of the 7 keys pressed
        keyDown = False
        key = self.keyboard.getKey()
        while key > 0:
            if ord(' ') == (key & Keyboard.KEY):
                keyDown = True
                break
            key = self.keyboard.getKey()

        if keyDown and not self.lastClickState:
            self.lastClickState = keyDown
            return True
        else:
            self.lastClickState = keyDown
            return False

    def setActive(self, flag):
        if self.active != flag:
            self.active = flag

    def checkActive(self):
        if self.clicked():
            print(f"{'dis' if self.active else 'en'}abling cruise control")
            self.setActive(not self.active)

    def getTarget(self):
        return self.targetMemory

    def setMemory(self, target):
        self.targetMemory = target
