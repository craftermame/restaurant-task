from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import hsrb

def test_hsrb_speak():
    hsrb.speak("This is the sample sentence.")

if __name__ == "__main__":
    test_hsrb_speak()
