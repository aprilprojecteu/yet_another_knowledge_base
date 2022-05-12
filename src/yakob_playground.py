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


def get_dummy_mobipick_facts():
    return [
        ('mobipick', RDF.type, incorap["Robot"]),
        ('arm_mobipick', RDF.type, incorap["Arm"]),
        ('mobipick', incorap.has_arm, 'arm_mobipick'),
        ('table_1_grasp_pose', RDF.type, incorap.Waypoint),
        ('mobipick', incorap.at, 'table_1_grasp_pose'),
        ('mobipick', incorap.has_arm_posture, 'default')
    ]


def test_add_facts():

    print("In test_add_facts()")

    facts = []
    for obj in get_dummy_objects():
        f = Fact()
        f.fact = (obj[0], RDF.type, obj[1])
        f.object_type = "URI"
        facts.append(f)

    for triple in get_dummy_mobipick_facts():
        f = Fact()
        f.fact = triple
        if triple[1] == incorap.has_arm_posture:
            f.object_type = "string"
        else:
            f.object_type = "URI"
            facts.append(f)

    rospy.wait_for_service("yakob_add_facts")
    try:
        add_facts = rospy.ServiceProxy("yakob_add_facts", YakobUpdateFacts)
        add_facts(facts)
    except rospy.ServiceException:
        rospy.logerr("[yakob_add_fact] something went wrong")


def test_remove_facts():

    print("In test_remove_facts()")

    deletes = [
        ('mobipick', incorap.at, 'table_1_grasp_pose'),
        ('relay_1', RDF.type, incorap["Physical_Object"].toPython())
    ]

    facts = []

    for fact in deletes:
        f = Fact()
        f.fact = fact
        f.object_type = "URI"
        facts.append(f)

    rospy.wait_for_service("yakob_remove_facts")
    try:
        del_facts = rospy.ServiceProxy("yakob_remove_facts", YakobUpdateFacts)
        del_facts(facts)
    except rospy.ServiceException:
        rospy.logerr("[yakob_delete_fact] something went wrong")


if __name__ == "__main__":
    rospy.init_node("yakob_test_node")
    test_add_facts()
    # rospy.sleep(5)
    test_remove_facts()
