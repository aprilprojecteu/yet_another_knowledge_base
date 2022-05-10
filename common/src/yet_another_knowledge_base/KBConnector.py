from abc import abstractmethod
import yaml
from rdflib import URIRef, Literal


class KBConnectorInterface:

    def __init__(self, typemap_path) -> None:
        with open(typemap_path, "r") as file:
            self.type_to_xsd = yaml.safe_load(file)

    @ abstractmethod
    def add_facts(self, facts: list) -> None:

        raise NotImplementedError

    @ abstractmethod
    def remove_facts(self, facts: list) -> None:
        raise NotImplementedError

    @ abstractmethod
    def add_ontology(self, ontology: str) -> None:
        raise NotImplementedError

    def get_node_triple(self, fact: tuple, object_type: str) -> tuple:
        if object_type == "URI":
            return (URIRef(fact[0]), URIRef(fact[1]), URIRef(fact[2]))
        else:
            return (URIRef(fact[0]), URIRef(fact[1]), Literal(fact[2], datatype=self.type_to_xsd[object_type]))
