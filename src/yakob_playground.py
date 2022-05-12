#!/usr/bin/python3

import rospy
from yet_another_knowledge_base.srv import YakobUpdateFacts
from yet_another_knowledge_base.msg import Fact
from rdflib.namespace import RDF, Namespace

incorap = Namespace('http://dfki.de/incorap/incorap_domain#')


def get_dummy_objects():

    return [
        ('relay_1', incorap["Physical_Object"].toPython()),
        ('screwdriver_1', incorap["Physical_Object"].toPython()),
        ('powerdrill_1', incorap["Physical_Object"].toPython()),
        ('multimeter_1', incorap["Physical_Object"].toPython())
    ]


def test_add_facts():

    print("In test_add_facts()")

    facts = []
    for obj in get_dummy_objects():
        f = Fact()
        f.fact = (obj[0], RDF.type, obj[1])
        f.object_type = "URI"
        facts.append(f)

    rospy.wait_for_service("yakob_add_facts")
    try:
        add_facts = rospy.ServiceProxy("yakob_add_facts", YakobUpdateFacts)
        add_facts(facts)
    except rospy.ServiceException:
        rospy.logerr("[yakob_add_fact] something went wrong")


if __name__ == "__main__":
    rospy.init_node("yakob_test_node")
    while not rospy.is_shutdown():
        test_add_facts()
        rospy.sleep(5)
