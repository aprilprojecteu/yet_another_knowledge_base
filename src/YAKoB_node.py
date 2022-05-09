#!/usr/bin/python3

import rospy
from yet_another_knowledge_base.srv import YakobUpdate, YakobUpdateResponse
from yet_another_knowledge_base.FusekiConnector import FusekiConnector


class YetAnotherKnowledgeBase:
    def __init__(self) -> None:
        self.service_add_ontology = rospy.Service(
            "yakob_add_ontology", YakobUpdate, self.handle_yakob_add_ontology)
        self.service_add_fact = rospy.Service(
            "yakob_add_fact", YakobUpdate, self.handle_yakob_add_fact)
        self.service_remove_fact = rospy.Service(
            "yakob_remove_fact", YakobUpdate, self.handle_yakob_remove_fact)

        self.c = FusekiConnector()

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("YAKoB is running!")
            rospy.sleep(1.0)

    def handle_yakob_add_ontology(self, req):
        res = self.c.add_ontology(ontology=req.data)
        return YakobUpdateResponse(res)

    def handle_yakob_add_fact(self, req):
        res = self.c.add_fact(fact=req.data)
        return YakobUpdateResponse(res)

    def handle_yakob_remove_fact(self, req):
        res = self.c.remove_fact(req.data)
        return YakobUpdateResponse(res)


if __name__ == "__main__":
    rospy.init_node("yet_another_knowledge_base", anonymous=False)
    YAKoB = YetAnotherKnowledgeBase()
    YAKoB.run()
