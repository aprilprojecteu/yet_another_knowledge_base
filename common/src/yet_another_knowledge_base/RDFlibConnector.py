from .KBConnector import KBConnectorInterface
from rdflib import Graph


class RDFlibConnector(KBConnectorInterface):

    def __init__(self, typemap_path, namespace) -> None:
        super().__init__(typemap_path, namespace)
        self._fs = Graph()
        self._o = Graph()

    def add_facts(self, facts: list) -> None:

        for fact in facts:
            self._fs.add(self.get_node_triple(fact.fact, fact.object_type))

    def add_ontology(self, ontology: str) -> None:

        print("[WARNING] Not implemented yet")

    def remove_facts(self, facts: list) -> None:

        for fact in facts:
            self._fs.remove(self.get_node_triple(fact.fact, fact.object_type))
