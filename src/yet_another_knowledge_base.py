#!/usr/bin/python3

import roslib
import rospy


class YetAnotherKnowledgeBase:
    def __init__(self) -> None:
        pass

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("YAKoB is running!")
            rospy.sleep(1.0)

if __name__ == "__main__":
    rospy.init_node("yet_another_knowledge_base", anonymous=False)
    YAKoB = YetAnotherKnowledgeBase()
    YAKoB.run()
