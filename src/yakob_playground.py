import rospy
from yet_another_knowledge_base.srv import YakobUpdate

from rdflib import Graph, URIRef
from rdflib.namespace import RDF, Namespace

ontology_path = "/home/mrenz/projects/incorap_rdf/incorap_domain.owl"

f = open(ontology_path, "r")
o = f.read()
f.close()

rospy.init_node("yakob_test_node")
rospy.wait_for_service("yakob_add_ontology")
try:
    add_ontology = rospy.ServiceProxy("yakob_add_ontology", YakobUpdate)
    res = add_ontology(o)
    rospy.loginfo(f"[YAKoB]: add_ontology response code: {res}")
except rospy.ServiceException:
    rospy.logerr("[yakob_add_ontology] something went wrong")

incorap = Namespace('http://dfki.de/incorap/incorap_domain#')
fs = Graph()
fs.bind('incorap', incorap)
subj = 'mobipick'
label = 'http://dfki.de/incorap/incorap_domain#Robot'
fs.add((URIRef(subj), RDF.type, URIRef(label)))

rospy.wait_for_service("yakob_add_fact")
try:
    add_fact = rospy.ServiceProxy("yakob_add_fact", YakobUpdate)
    res = add_fact(fs.serialize(format='ttl'))
    rospy.loginfo(f"[YAKoB]: add_fact response code: {res}")
except rospy.ServiceException:
    rospy.logerr("[yakob_add_fact] something went wrong")

fs.subject((RDF.type, incorap.Robot))
fs.subject_predicates((URIRef('mobipick'), RDF.type))
