#!/usr/bin/python3

import rospy
from yet_another_knowledge_base.srv import YakobUpdateFacts, YakobUpdateFactsResponse, YakobUpdateGraph, YakobUpdateGraphResponse
from yet_another_knowledge_base.FusekiConnector import FusekiConnector


class YetAnotherKnowledgeBase:
    def __init__(self) -> None:
        self.service_add_ontology = rospy.Service(
            "yakob_add_ontology", YakobUpdateGraph, self.handle_yakob_add_ontology)
        self.service_add_fact = rospy.Service(
            "yakob_add_facts", YakobUpdateFacts, self.handle_yakob_add_facts)
        self.service_remove_fact = rospy.Service(
            "yakob_remove_facts", YakobUpdateFacts, self.handle_yakob_remove_facts)

        self.datatype_map_path = rospy.get_param("/YAKoB/datatype_map")
        self.c = FusekiConnector(self.datatype_map_path)

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("YAKoB is running!")
            rospy.sleep(1.0)

    def handle_yakob_add_ontology(self, req):
        self.c.add_ontology(ontology=req.graph)
        return YakobUpdateGraphResponse()

    def handle_yakob_add_facts(self, req):
        self.c.add_facts(facts=req.facts)
        return YakobUpdateFactsResponse()

    def handle_yakob_remove_facts(self, req):
        self.c.remove_facts(facts=req.facts)
        return YakobUpdateFactsResponse()


if __name__ == "__main__":
    rospy.init_node("yet_another_knowledge_base", anonymous=False)
    YAKoB = YetAnotherKnowledgeBase()
    YAKoB.run()
