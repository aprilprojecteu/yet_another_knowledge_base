from abc import abstractmethod
import yaml
from rdflib import URIRef, Literal, Namespace
import re


class KBConnectorInterface:

    def __init__(self, typemap_path, namespace) -> None:
        with open(typemap_path, "r") as file:
            self.type_to_xsd = yaml.safe_load(file)

        if isinstance(namespace, Namespace):
            self.ns = namespace
        else:
            self.ns = Namespace(namespace)

    @ abstractmethod
    def add_facts(self, facts: list) -> None:

        raise NotImplementedError

    @ abstractmethod
    def remove_facts(self, facts: list) -> None:
        raise NotImplementedError

    @ abstractmethod
    def add_ontology(self, ontology: str) -> None:
        raise NotImplementedError

    def get_URI(self, uri: str) -> URIRef:
        """
        Checks if given string is a uri with namespace using a regular expression.
        If no, adds uri to the internal namespace.
        Args:
            uri (str): uri to be generated

        Returns:
            URIRef: uri as rdflib.URIRef object
        """

        if re.match("(http|ftp|https):\/\/([\w_-]+(?:(?:\.[\w_-]+)+))([\w.,@?^=%&:\/~+#-]*[\w@?^=%&\/~+#-])", uri):
            return URIRef(uri)
        else:
            return self.ns[uri]

    def get_node_triple(self, fact: tuple, object_type: str) -> tuple:
        if object_type == "URI":
            return (self.get_URI(fact[0]), self.get_URI(fact[1]), self.get_URI(fact[2]))
        else:
            return (self.get_URI(fact[0]), self.get_URI(fact[1]), Literal(fact[2], datatype=self.type_to_xsd[object_type]))
