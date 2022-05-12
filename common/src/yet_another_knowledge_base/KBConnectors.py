from abc import abstractmethod
import yaml
from rdflib import URIRef, Literal, Namespace, Graph
import requests
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


class FusekiConnector(KBConnectorInterface):

    def __init__(self, typemap_path, namespace, fuseki="http://127.0.0.1:3030") -> None:
        super().__init__(typemap_path, namespace)
        self.fuseki = fuseki

    def add_facts(self, facts: list) -> None:

        g = Graph()
        for fact in facts:
            g.add(self.get_node_triple(fact.fact, fact.object_type))

        r = requests.post(self.fuseki + "/fs/data",
                          data=g.serialize(format="ttl"), headers={'Content-Type': 'text/turtle'})
        if r.status_code != 200:
            print(f"[WARNING] Fuseki returned {r.status_code}")

    def add_ontology(self, ontology: str) -> None:
        r = requests.post(self.fuseki + "/o/data",
                          data=ontology, headers={'Content-Type': 'text/turtle'})
        if r.status_code != 200:
            print(f"[WARNING] Fuseki returned {r.status_code}")

    def remove_facts(self, facts: list) -> None:

        for fact in facts:
            f = self.get_node_triple(fact.fact, fact.object_type)
            q = f"""
            DELETE DATA {{ <{f[0]}> <{f[1]}> <{f[2]}> }}
            """
            print(q)

            r = requests.post(self.fuseki + '/fs/update', data=q,
                              headers={'Content-Type': 'application/sparql-update'})
            if r.status_code != 200:
                print(f"[WARNING] Fuseki returned {r.status_code}")


class RDFlibConnector(KBConnectorInterface):

    def __init__(self, typemap_path, namespace) -> None:
        super().__init__(typemap_path, namespace)
        self._fs = Graph()
        self._o = Graph()
        self._fs.bind("", self.ns)
        self._o.bind("", self.ns)

    def add_facts(self, facts: list) -> None:

        for fact in facts:
            self._fs.add(self.get_node_triple(fact.fact, fact.object_type))

    def add_ontology(self, ontology: str) -> None:

        print("[WARNING] Not implemented yet")

    def remove_facts(self, facts: list) -> None:

        for fact in facts:
            self._fs.remove(self.get_node_triple(fact.fact, fact.object_type))

        print(self._fs.serialize())
